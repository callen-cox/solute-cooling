import pytest
from fastapi.testclient import TestClient

from coolant.core import Config, Engine
from coolant.server import app


def run(e, n):
    for _ in range(n):
        e.step()


def test_config_requires_margins():
    with pytest.raises(ValueError):
        Config(t_shutdown=53.0).validate()  # too close to damage
    with pytest.raises(ValueError):
        Config(t_warn=29.0).validate()


def test_normal_operation_holds_setpoint():
    e = Engine()
    run(e, 600)
    assert e.last["state"] == "NORMAL" and abs(e.last["true_T"] - 30) < 4


def test_never_reaches_damage_even_when_solute_runs_out():
    e = Engine()
    e.scenario("spike")
    for _ in range(6000):
        e.step()
        assert e.last["true_T"] < e.cfg.t_damage


def test_single_bad_sensor_is_outvoted_not_fatal():
    e = Engine()
    run(e, 100)
    e.scenario("sensor_fault")
    run(e, 100)
    assert e.last["state"] == "WARNING" and e.last["health"] == 1 and e.last["load"] == 1.0


def test_losing_all_sensors_fails_safe():
    e = Engine()
    e.scenario("sensors_lost")
    run(e, 3)
    assert e.last["state"] == "SHUTDOWN" and e.last["load"] == 0.0


def test_api_roundtrip():
    with TestClient(app) as c:
        s = c.get("/api/state").json()
        assert s["state"] in {"NORMAL", "WARNING", "THROTTLE", "SHUTDOWN"} and s["history"]
        assert c.post("/api/scenario/nope").status_code == 404
        assert c.post("/api/config", json={"t_shutdown": 54}).status_code == 422
        assert c.post("/api/scenario/reset").json()["t"] > 0
