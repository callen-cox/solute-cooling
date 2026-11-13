import random

import pytest
from fastapi.testclient import TestClient

from coolant.core import MODES, Config, Controller, Engine, Orbit, State, solubility
from coolant.server import app


def run(e, n):
    for _ in range(n):
        e.step()


def extremes(e, n):
    lo, hi = 1e9, -1e9
    for _ in range(n):
        e.step()
        lo, hi = min(lo, e.last["true_T"]), max(hi, e.last["true_T"])
    return lo, hi


def logged(e, text):
    return any(text in ev["msg"] for ev in e.events)


# ---------- safety configuration ----------
def test_config_enforces_order_and_margins():
    Config().validate()
    for bad in ({"t_shutdown": 80.0}, {"t_warn": 40.0}, {"sat_switch": 1.2}, {"t_freeze_warn": 50.0}):
        with pytest.raises(ValueError):
            Config(**bad).validate()


# ---------- terrestrial: salt ----------
@pytest.mark.parametrize("mode,lo,hi", [("efficiency", 38, 42), ("performance", 23.5, 27), ("stealth", 26.5, 30)])
def test_salt_modes_hold_their_setpoints(mode, lo, hi):
    e = Engine(mode=mode)
    run(e, 900)
    assert e.last["state"] == "NORMAL" and lo < e.last["true_T"] < hi


def test_salt_stealth_keeps_the_thermal_signature_near_zero():
    e = Engine(mode="stealth")
    run(e, 900)
    assert e.hal.released_w < 30


def test_salt_swaps_tank_before_crystals_can_form():
    e = Engine(mode="performance")
    for _ in range(1500):
        e.step()
        assert e.hal.dissolved <= solubility(e.hal.solute, e.hal.T) * e.hal.WATER_KG + 1e-6
    assert e.hal.tanks == 0 and logged(e, "reserve tank")


def test_performance_mode_precools_before_a_scheduled_job():
    e = Engine(mode="performance")
    run(e, 400)
    e.scenario("burst")
    run(e, 80)
    assert e.last["setpoint"] == 21.0 and e.last["true_T"] < 24.5


def test_pump_degradation_is_learned():
    e = Engine()
    run(e, 600)
    e.scenario("pump_fault")
    run(e, 600)
    assert e.ctl.eff < 0.5


# ---------- terrestrial: PCM ----------
def test_pcm_switches_modules_and_schedules_regeneration():
    e = Engine(sink="pcm")
    e.scenario("spike")
    run(e, 600)
    assert logged(e, "switched to fresh module M2") and logged(e, "low-power state")
    assert e.last["load"] == 0.4


def test_pcm_stealth_keeps_heat_in_until_a_safe_window():
    e = Engine(sink="pcm", mode="stealth")
    e.scenario("spike")
    run(e, 300)
    quiet = e.hal.released_w
    e.scenario("window")
    run(e, 5)
    assert e.hal.released_w > quiet + 250


# ---------- the safety supervisor ----------
@pytest.mark.parametrize("kw", [{}, {"mode": "performance"}, {"sink": "pcm"}, {"sink": "pcm", "mode": "performance"}])
def test_sink_exhaustion_never_exceeds_t_safe(kw):
    e = Engine(**kw)
    e.scenario("spike")
    _, hi = extremes(e, 8000)
    assert hi < e.cfg.t_shutdown + 1


def test_one_bad_sensor_is_outvoted_not_fatal():
    e = Engine()
    run(e, 100)
    e.scenario("sensor_fault")
    run(e, 100)
    assert e.last["state"] == "WARNING" and e.last["health"] == 1 and e.last["load"] == 1.0


@pytest.mark.parametrize("kw", [{}, {"sink": "pcm"}, {"env": "orbital"}])
def test_losing_all_sensors_fails_safe(kw):
    e = Engine(**kw)
    run(e, 20)
    e.scenario("sensors_lost")
    lo, _ = extremes(e, 1500)
    assert e.last["state"] == "SHUTDOWN" and e.last["load"] == 0.0
    assert lo > e.cfg.t_freeze          # in orbit the survival heater keeps the water liquid


@pytest.mark.parametrize("mode", ["efficiency", "performance"])
def test_throttle_does_not_flicker(mode):
    e = Engine(env="orbital", mode=mode)
    e.scenario("high_beta")             # no eclipse to cool it back down mid-test
    run(e, 300)
    e.scenario("solar_pass")
    e.scenario("adcs_fault")
    states = []
    for _ in range(600):
        e.step()
        if not states or states[-1] != e.last["state"]:
            states.append(e.last["state"])
    assert states == ["NORMAL", "WARNING", "THROTTLE", "SHUTDOWN"]


def test_throttle_releases_only_once_the_coolant_is_cooling():
    ctl = Controller(Config())
    ctl.state, ctl.clock, ctl.throttle_until = State.THROTTLE, 1000.0, 900.0   # 5 min hold already over
    ctl.slope = 0.01                                                            # still warming
    assert ctl.supervise(T=45.0, hot=45.0, health=0, low=False, out=False)[0] == State.THROTTLE
    ctl.slope = -0.01                                                           # cooling now
    assert ctl.supervise(T=45.0, hot=45.0, health=0, low=False, out=False)[0] == State.NORMAL


# ---------- orbital ----------
@pytest.mark.parametrize("mode", MODES)
def test_orbit_freeze_guard_keeps_the_water_liquid(mode):
    e = Engine(env="orbital", mode=mode)
    run(e, 100)
    e.scenario("idle")
    lo, _ = extremes(e, 1500)
    assert lo > e.cfg.t_freeze


def test_orbit_attitude_shrugs_off_a_close_solar_pass():
    e = Engine(env="orbital")
    run(e, 200)
    e.scenario("solar_pass")
    run(e, 300)
    assert e.last["state"] == "NORMAL"


def test_orbit_stuck_facing_the_sun_retracts_and_stays_safe():
    e = Engine(env="orbital")
    run(e, 150)
    e.scenario("solar_pass")
    e.scenario("adcs_fault")
    _, hi = extremes(e, 1500)
    assert e.hal.dep < 0.2 and logged(e, "retracting behind MLI") and hi < e.cfg.t_shutdown + 1


def test_orbit_stealth_holds_heat_until_the_window():
    e = Engine(env="orbital", mode="stealth")
    run(e, 250)
    assert e.last["true_T"] > 43
    e.scenario("window")
    run(e, 250)
    assert e.last["true_T"] < 33


@pytest.mark.parametrize("mode,lo,hi", [("efficiency", 28, 32), ("performance", 24, 29), ("stealth", 43, 48)])
def test_orbit_holds_temperature_through_eclipses(mode, lo, hi):
    e = Engine(env="orbital", mode=mode)
    run(e, 200)
    seen = set()
    for _ in range(5 * 270):                     # five 90-minute orbits
        e.step()
        seen.add(e.hal.sunlit())
        assert e.last["state"] == "NORMAL" and lo < e.last["true_T"] < hi
    assert seen == {True, False}


def test_orbit_tracks_the_sun_as_it_moves():
    e = Engine(env="orbital", mode="performance")
    for _ in range(3 * 270):
        e.step()
        if e.hal.sunlit():
            assert abs(e.hal.theta) > 80        # kept edge-on while the sun sweeps around
    eff = Engine(env="orbital")
    run(eff, 3 * 270)
    assert eff.hal.slewed < e.hal.slewed / 2  # efficiency lets the attitude float in eclipse


def test_orbit_eclipse_and_earth_heat_are_modelled():
    o = Orbit()
    o.theta = 90.0
    day = o.absorb_full()
    o.clock = o.PERIOD - 60                      # in Earth's shadow
    assert not o.sunlit() and 0 < o.absorb_full() < day   # only Earth's infrared glow remains
    o.flags["high_beta"] = True
    assert o.sunlit() and o.next_transition_s() is None


@pytest.mark.parametrize("seed", range(5))
def test_radiation_storm_is_shrugged_off(seed):
    random.seed(seed)
    e = Engine(env="orbital")
    run(e, 100)
    e.scenario("radiation")
    states = set()
    for _ in range(2700):
        e.step()
        states.add(e.last["state"])
    assert states <= {"NORMAL", "WARNING"}
    assert logged(e, "memory scrub") and e.cfg == Config()


def test_memory_scrub_repairs_a_corrupted_threshold():
    e = Engine(env="orbital")
    e.ctl.cfg.t_shutdown = 999.0                 # a bit flip disables the shutdown limit
    e.step()
    assert e.cfg.t_shutdown == 60.0 and logged(e, "repaired corrupted t_shutdown")
    e.update_config({"t_shutdown": 62.0})        # deliberate changes are kept
    e.step()
    assert e.cfg.t_shutdown == 62.0


# ---------- engine + API ----------
def test_mode_switch_keeps_the_simulation_running():
    e = Engine()
    run(e, 100)
    t = e.t
    e.setup(mode="performance")
    assert e.t == t and e.ctl.mode == "performance" and logged(e, "mode switched")


def test_api_roundtrip():
    with TestClient(app) as c:
        s = c.get("/api/state").json()
        assert s["state"] in {"NORMAL", "WARNING", "THROTTLE", "SHUTDOWN"}
        assert s["history"] and s["tiles"] and s["scenarios"]
        assert c.post("/api/scenario/nope").status_code == 404
        assert c.post("/api/config", json={"t_shutdown": 80}).status_code == 422
        assert c.post("/api/setup", json={"env": "mars"}).status_code == 422
        assert c.post("/api/setup", json={"colour": "red"}).status_code == 422
        assert c.post("/api/setup", json={"speed": 3}).status_code == 422
        assert c.post("/api/setup", json={"env": "orbital"}).json()["kind"] == "orbit"
        assert c.post("/api/setup", json={"mode": "stealth", "speed": 5}).json()["options"]["speed"] == 5
        assert c.post("/api/setup", json={"env": "terrestrial", "sink": "pcm"}).json()["kind"] == "pcm"
        assert c.post("/api/scenario/spike").json()["flags"]["spike"] is True
