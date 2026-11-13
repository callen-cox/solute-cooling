import random

import pytest
from fastapi.testclient import TestClient

from coolant.core import MODES, Config, Controller, Engine, State, equilibrium, sizing
from coolant.server import app


@pytest.fixture(autouse=True)
def repeatable():
    random.seed(1234)                   # sensor noise is random; seed it so every run is identical


def run(e, n):
    for _ in range(n):
        e.step()


def extremes(e, n):
    lo, hi = 1e9, -1e9
    for _ in range(n):
        e.step()
        lo, hi = min(lo, e.last["true_T"]), max(hi, e.last["true_T"])
    return lo, hi


def states_during(e, n, watch=None):
    """The sequence of distinct states over n steps (and the max of watch(e), if given)."""
    seen, top = [], -1e9
    for _ in range(n):
        e.step()
        if watch:
            top = max(top, watch(e))
        if not seen or seen[-1] != e.last["state"]:
            seen.append(e.last["state"])
    return seen, top


def logged(e, text):
    return any(text in ev["msg"] for ev in e.events)


def titles(e):
    return [a["title"] for a in e.last["alerts"]]


# ---------- safety configuration and physics ----------
def test_config_enforces_order_and_margins():
    Config().validate()
    for bad in ({"t_shutdown": 80.0}, {"t_warn": 40.0}, {"t_freeze_warn": 50.0},
                {"chip_shutdown": 95.0}, {"chip_warn": 88.0}, {"drift_limit": 5.0}):
        with pytest.raises(ValueError):
            Config(**bad).validate()


def test_energy_balance_forecast():
    emit_at_28 = 0.85 * 5.67e-8 * 1.5 * (28 + 273.15) ** 4     # 1.5 m² of radiator, facing away from the sun
    assert 14 < equilibrium(28, 500, 0, emit_at_28) < 16       # normal load: radiators settle near 15 °C
    assert 104 < equilibrium(28, 1500, 0, emit_at_28) < 108    # a surge: they alone would settle near 106 °C
    assert equilibrium(28, 500, 0, emit_at_28, exposure=0) == float("inf")


def test_sizing_shows_the_wax_shrinks_the_radiator():
    s = sizing()                        # 0.5 kW normal, 1.5 kW surges for 30 min every 6 h, radiator at 30 °C
    peak, pcm = s["rows"]
    assert 3.6 < peak["area"] < 3.8 and 1.35 < pcm["area"] < 1.5 and s["area_saved_pct"] == 62
    assert s["mass_saved_kg"] > 0
    assert sizing(panel_kg_m2=2)["mass_saved_kg"] < 0          # very light panels: the wax costs more than it saves
    for bad in ({"normal_kw": -1}, {"normal_kw": 2, "surge_kw": 1}, {"panel_kg_m2": 0}):
        with pytest.raises(ValueError):
            sizing(**bad)


# ---------- tier 1: radiators ----------
@pytest.mark.parametrize("mode,lo,hi", [("efficiency", 26, 30), ("performance", 19, 21.5)])
def test_radiators_alone_hold_each_mode(mode, lo, hi):
    e = Engine(mode=mode)
    run(e, 900)
    assert e.last["state"] == "NORMAL" and lo < e.last["true_T"] < hi and e.last["alerts"] == []
    assert e.hal.melt == 0 and e.hal.theta == 180              # wax untouched, radiators facing away from the sun


def test_a_close_solar_pass_changes_nothing_when_facing_away():
    e = Engine()
    run(e, 200)
    e.scenario("solar_pass")
    lo, hi = extremes(e, 600)
    assert e.last["state"] == "NORMAL" and hi - lo < 1


# ---------- tier 2: wax PCM, then tier 3 ----------
def test_surge_goes_radiators_then_wax_then_shed_load():
    e = Engine()
    run(e, 100)
    e.scenario("spike")
    seen, hi = [], 0.0
    for _ in range(900):                # 75 min of a surge that doesn't stop
        e.step()
        hi = max(hi, e.last["true_T"])
        if e.last["pcm"] == "absorbing" and 0.1 < e.hal.melt < 0.7:
            assert 37 < e.last["true_T"] < 41                  # the melting wax pins the water just above 36.6 °C
        if not seen or seen[-1] != e.last["state"]:
            seen.append(e.last["state"])
    assert seen == ["NORMAL", "WARNING", "THROTTLE"]           # wax, a warning as it runs low, then load shed
    assert logged(e, "wax PCM absorbing") and "wax spent" in e.last["reason"]
    assert e.last["load"] == 0.5 and hi < e.cfg.t_warn


def test_after_the_surge_spare_capacity_refreezes_the_wax():
    e = Engine()
    run(e, 100)
    e.scenario("spike")
    run(e, 600)
    e.scenario("spike")
    melted = e.hal.melt
    run(e, 1440)                        # 2 h
    assert e.last["state"] == "NORMAL" and e.last["load"] == 1.0
    assert e.hal.melt < melted - 0.3 and logged(e, "re-freezing the wax")


def test_performance_refreezes_the_wax_without_warming_up():
    e = Engine(mode="performance")
    run(e, 600)
    e.scenario("spike")
    run(e, 300)
    e.scenario("spike")
    run(e, 1440)
    assert e.last["pcm"] == "re-freezing" and e.last["true_T"] < 23


def test_stealth_stores_heat_in_the_wax_until_a_window():
    e = Engine(mode="stealth")
    run(e, 600)
    quiet = e.hal.radiated
    assert e.last["pcm"] == "storing" and e.hal.melt > 0.3 and quiet < 200 and e.last["alerts"] == []
    e.scenario("window")
    run(e, 300)
    assert e.hal.radiated > quiet + 200


def test_attitude_failure_in_a_solar_pass_goes_down_safely():
    e = Engine()
    run(e, 100)
    e.scenario("solar_pass")
    e.scenario("adcs_fault")
    seen, hi = states_during(e, 2000, lambda e: e.last["true_T"])
    assert seen == ["NORMAL", "WARNING", "THROTTLE", "SHUTDOWN"]
    assert logged(e, "stowing them behind the shield") and logged(e, "wax PCM absorbing")
    assert e.hal.dep == 0 and hi < e.cfg.t_shutdown + 1


# ---------- freeze guard ----------
@pytest.mark.parametrize("mode", MODES)
def test_freeze_guard_keeps_the_water_liquid_at_idle(mode):
    e = Engine(mode=mode)
    run(e, 100)
    e.scenario("idle")
    lo, _ = extremes(e, 3000)           # 4 h with the servers almost off
    assert lo > e.cfg.t_freeze and e.ctl.cold and e.hal.dep == 0
    assert e.hal.sun_in > 0 and e.hal.heater == 0              # sunlight does it; no electric heater needed


def test_the_wax_heat_keeps_the_loop_warm_while_it_refreezes():
    e = Engine()
    run(e, 100)
    e.scenario("spike")
    run(e, 500)                         # the surge melts most of the wax
    e.scenario("spike")
    e.scenario("idle")
    run(e, 300)
    assert e.last["pcm"] == "re-freezing" and e.last["true_T"] > 30
    lo, _ = extremes(e, 3000)
    assert lo > e.cfg.t_freeze and e.hal.melt == 0


# ---------- the watchdog ----------
def test_drifting_sensor_is_caught_before_it_can_fool_the_vote():
    e = Engine()
    run(e, 100)
    e.scenario("sensor_drift")          # S2 creeps upward 0.015 °C per second
    for _ in range(200):
        e.step()
        if logged(e, "drifting"):
            break
    assert e.hal.drift < 2.0            # caught long before the 3 °C vote would notice
    run(e, 10)
    assert e.last["state"] == "WARNING" and abs(e.last["T"] - e.last["true_T"]) < 0.15


def test_one_bad_sensor_is_outvoted_not_fatal():
    e = Engine()
    run(e, 100)
    e.scenario("sensor_fault")
    run(e, 100)
    assert e.last["state"] == "WARNING" and e.last["health"] == 1 and e.last["load"] == 1.0


def test_losing_all_sensors_fails_safe():
    e = Engine()
    run(e, 20)
    e.scenario("sensors_lost")
    lo, _ = extremes(e, 3000)
    assert e.last["state"] == "SHUTDOWN" and e.last["load"] == 0.0
    assert lo > e.cfg.t_freeze          # the survival heater's own thermostat keeps the water liquid


def test_chip_limits_trip_even_when_the_coolant_is_fine():
    e = Engine(mode="performance")
    run(e, 300)
    e.scenario("spike")
    e.scenario("cold_plate")
    run(e, 12)                          # one minute
    assert e.last["state"] == "THROTTLE" and "chips" in e.last["reason"] and e.last["true_T"] < 40


@pytest.mark.parametrize("mode", MODES)
def test_clogged_cold_plate_is_caught_early_without_power_cycling(mode):
    e = Engine(mode=mode)
    run(e, 300)
    e.scenario("spike")
    e.scenario("cold_plate")
    seen, hottest = states_during(e, 600, lambda e: e.hal.chip)
    assert seen[0] == "NORMAL" and seen[-1] == "THROTTLE" and len(seen) <= 3      # one step down, no flapping
    assert "SHUTDOWN" not in seen and hottest < e.cfg.chip_throttle


def test_a_projection_sheds_load_before_it_can_shut_down():
    ctl = Controller(Config())
    soon = dict(T=50.0, hot=65.0, health=0, low=False, out=False)       # coolant heading past 60 °C
    ctl.state = State.WARNING
    assert ctl.supervise(**soon)[0] == State.THROTTLE
    ctl.state = State.THROTTLE                                          # load already shed, still heading over
    assert ctl.supervise(**soon)[0] == State.SHUTDOWN
    chips = dict(T=40.0, hot=40.0, health=0, low=False, out=False, chip=95.0, chip_now=75.0)
    ctl.state = State.WARNING
    assert ctl.supervise(**chips) == (State.THROTTLE, "chips heating fast: shedding load early")
    ctl.state = State.NORMAL                                            # a real reading over the limit
    assert ctl.supervise(**{**chips, "chip_now": 91.0})[0] == State.SHUTDOWN


def test_throttle_releases_only_once_the_coolant_is_cooling():
    ctl = Controller(Config())
    ctl.state, ctl.clock, ctl.throttle_until = State.THROTTLE, 1000.0, 900.0   # 5 min hold already over
    ctl.slope = 0.01                                                            # still warming
    assert ctl.supervise(T=45.0, hot=45.0, health=0, low=False, out=False)[0] == State.THROTTLE
    ctl.slope = -0.01                                                           # cooling now
    assert ctl.supervise(T=45.0, hot=45.0, health=0, low=False, out=False)[0] == State.NORMAL
    assert ctl.supervise(T=45.0, hot=45.0, health=0, low=False, out=False, coolant_full=106.0)[0] == State.THROTTLE


def test_restart_after_a_shutdown_is_gentle():
    ctl = Controller(Config())
    cool = dict(T=40.0, hot=40.0, health=0, low=False, out=False, chip=60.0)
    ctl.state, ctl.clock, ctl.shutdown_until = State.SHUTDOWN, 100.0, 150.0
    assert ctl.supervise(**cool)[0] == State.SHUTDOWN                           # off for at least 2 minutes
    ctl.clock = 200.0
    assert ctl.supervise(**{**cool, "chip": 85.0})[0] == State.SHUTDOWN         # and until the chips have cooled
    assert ctl.supervise(**cool) == (State.THROTTLE, "restarting at reduced load")


# ---------- alerts ----------
def test_no_alerts_when_all_is_well():
    for mode in MODES:
        e = Engine(mode=mode)
        run(e, 300)
        assert e.last["alerts"] == [], mode


@pytest.mark.parametrize("kw,faults,steps,expected", [
    ({}, ["sensor_fault"], 100, "Sensor S1 failed"),
    ({}, ["sensor_drift"], 200, "Sensor S2 drifting"),
    ({}, ["sensors_lost"], 50, "All temperature sensors lost"),
    ({}, ["spike"], 900, "Wax spent and the radiators can't keep up"),
    ({"mode": "performance"}, ["spike", "cold_plate"], 120, "IT load cut to 50%"),
    ({}, ["solar_pass", "adcs_fault"], 200, "Attitude control failed"),
    ({}, ["idle"], 3000, "Freeze guard active"),
])
def test_each_fault_raises_a_clear_alert_most_serious_first(kw, faults, steps, expected):
    e = Engine(**kw)
    run(e, 300)
    for f in faults:
        e.scenario(f)
    run(e, steps)
    levels = [a["level"] for a in e.last["alerts"]]
    assert expected in titles(e) and levels == sorted(levels, key=lambda lvl: lvl != "critical")


def test_alerts_go_away_once_the_fault_is_fixed():
    e = Engine()
    run(e, 300)
    e.scenario("sensor_fault")
    run(e, 50)
    assert titles(e) == ["Sensor S1 failed"]
    e.scenario("clear")
    run(e, 10)
    assert e.last["alerts"] == []


# ---------- engine + API ----------
def test_mode_switch_keeps_the_simulation_running():
    e = Engine()
    run(e, 100)
    t = e.t
    e.setup(mode="performance")
    assert e.t == t and e.ctl.mode == "performance" and logged(e, "mode switched")


def test_clear_all_turns_every_scenario_off_without_resetting():
    e = Engine()
    run(e, 100)
    for name in e.hal.flags:
        e.scenario(name)
    t = e.t
    e.scenario("clear")
    assert not any(e.hal.flags.values()) and e.t == t and len(e.hist) > 50


def test_api_roundtrip():
    with TestClient(app) as c:
        s = c.get("/api/state").json()
        assert s["state"] in {"NORMAL", "WARNING", "THROTTLE", "SHUTDOWN"} and s["kind"] == "relay"
        assert s["history"] and s["tiles"] and s["scenarios"] and isinstance(s["alerts"], list)
        assert c.post("/api/scenario/nope").status_code == 404
        assert c.post("/api/config", json={"t_shutdown": 80}).status_code == 422
        assert c.post("/api/setup", json={"env": "terrestrial"}).status_code == 422   # Earth options are gone
        assert c.post("/api/setup", json={"speed": 3}).status_code == 422
        assert c.post("/api/setup", json={"mode": "stealth", "speed": 5}).json()["options"] == {"mode": "stealth",
                                                                                              "speed": 5}
        assert c.post("/api/scenario/spike").json()["flags"]["spike"] is True
        assert 3.6 < c.post("/api/size", json={}).json()["rows"][0]["area"] < 3.8
        assert c.post("/api/size", json={"normal_kw": -5}).status_code == 422
        page = c.get("/").text              # the dashboard, with its quality-of-life controls
        for el in ('id="theme"', 'id="welcome"', 'id="totop"', 'id="share"', 'id="alerts"'):
            assert el in page
