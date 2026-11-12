"""Solute-cooling core: safety controller, hardware layer, simulator, engine."""
import random
import threading
import time
from collections import deque
from dataclasses import asdict, dataclass, fields
from enum import Enum

CP_WATER = 4180.0  # J/(kg*K)
# solute: (J absorbed per g dissolved, solubility g/kg water at 20C, extra g/kg per degC)
SOLUTES = {
    "ammonium_nitrate": (321.0, 1500.0, 30.0),
    "urea": (256.0, 1080.0, 25.0),
    "ammonium_chloride": (277.0, 372.0, 6.0),
}


class State(str, Enum):
    NORMAL = "NORMAL"
    WARNING = "WARNING"    # alert; dose at max if hot
    THROTTLE = "THROTTLE"  # shed IT load
    SHUTDOWN = "SHUTDOWN"  # safe server shutdown, latched until cooled


LOAD_CAP = {State.NORMAL: 1.0, State.WARNING: 1.0, State.THROTTLE: 0.5, State.SHUTDOWN: 0.0}


@dataclass
class Config:
    setpoint: float = 30.0
    t_warn: float = 38.0
    t_throttle: float = 42.0
    t_shutdown: float = 46.0
    t_damage: float = 55.0      # hardware damage; must stay >=5C above shutdown
    restart_below: float = 35.0  # hysteresis for leaving SHUTDOWN
    min_hopper_g: float = 1500.0
    max_dose: float = 25.0
    max_sensor_gap: float = 3.0
    lookahead_s: float = 60.0

    def validate(self):
        if not (self.setpoint < self.restart_below < self.t_warn < self.t_throttle
                < self.t_shutdown and self.t_damage - self.t_shutdown >= 5):
            raise ValueError("need setpoint < restart < warn < throttle < shutdown, "
                             "with >=5C margin before damage")
        return self


class Controller:
    """Predictive PI dosing + safety state machine. Hardware-agnostic."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.reset()

    def reset(self):
        self.integ = self.slope = self.last_dose = 0.0
        self.prev = None
        self.eff = 1.0  # learned dosing effectiveness
        self.state, self.reason = State.NORMAL, "ok"

    def fuse(self, readings):
        """Median vote -> (temp, health). health: 0 ok, 1 degraded, 2 failed."""
        c = self.cfg
        good = sorted(r for r in readings if r is not None and -20 < r < 120)
        if len(good) < 2:
            return (good[-1] if good else c.t_shutdown + 1), 2
        health = 1 if len(good) < len(readings) else 0
        if good[-1] - good[0] > c.max_sensor_gap:
            med = good[len(good) // 2]
            kept = [g for g in good if abs(g - med) <= c.max_sensor_gap]
            if len(kept) < 2:
                return good[-1], 2  # no majority: assume the worst
            good, health = kept, 1
        return sum(good) / len(good), health

    def step(self, readings, hopper_g, saturation, delivered=None, dt=1.0):
        c = self.cfg
        T, health = self.fuse(readings)
        if self.prev is not None:
            self.slope = 0.8 * self.slope + 0.2 * (T - self.prev) / dt
        self.prev = T
        Tp = T + self.slope * c.lookahead_s
        hot = max(T, Tp)
        low = hopper_g < c.min_hopper_g or saturation > 0.95
        if self.last_dose > 1.0 and delivered is not None:  # learn under-delivery
            self.eff = 0.98 * self.eff + 0.02 * max(0.2, min(1.0, delivered / self.last_dose))

        if hot >= c.t_shutdown or (health == 2 and T >= c.t_throttle):
            s, why = State.SHUTDOWN, ("no trustworthy temperature data" if health == 2
                                      else f"{hot:.0f}C reached or predicted")
        elif self.state == State.SHUTDOWN and T > c.restart_below:
            s, why = State.SHUTDOWN, self.reason  # latched
        elif hot >= c.t_throttle or (low and hot >= c.t_warn):
            s, why = State.THROTTLE, ("cooling reserve low while hot" if low
                                      else "too hot, shedding load")
        elif hot >= c.t_warn or low or health:
            s, why = State.WARNING, ("cooling reserve low" if low
                                     else "sensor fault" if health else "approaching limit")
        else:
            s, why = State.NORMAL, "ok"
        self.state, self.reason = s, why

        err = Tp - c.setpoint
        self.integ = min(300.0, self.integ + err * dt) if 0 < err < 8 else self.integ * 0.98
        dose = (1.2 * max(err, 0.0) + 0.02 * self.integ) / self.eff
        dose = min(max(dose, 0.0), c.max_dose)
        if hot >= c.t_warn or s in (State.THROTTLE, State.SHUTDOWN):
            dose = c.max_dose
        self.last_dose = dose
        return {"T": round(T, 2), "T_pred": round(Tp, 2), "dose": round(dose, 2),
                "load": LOAD_CAP[s], "state": s.value, "reason": why,
                "health": health, "eff": round(self.eff, 2)}


class HAL:
    """Implement these methods to drive real hardware (see README)."""

    def read_temps(self): raise NotImplementedError      # deg C list, None = dead sensor
    def read_hopper_g(self): raise NotImplementedError   # solute left, grams
    def read_saturation(self): return 0.0                # 0..1 (e.g. conductivity probe)
    def set_dose(self, g_per_s): raise NotImplementedError
    def set_load(self, fraction): raise NotImplementedError  # IT power cap 0..1
    def delivered_dose(self): return None                # optional flow-meter feedback
    def tick(self, dt): pass                             # simulation hook


class SimHAL(HAL):
    SPIKE_W = 3500.0

    def __init__(self, solute="ammonium_nitrate", hopper_g=15000.0, water_kg=20.0, base_w=2000.0):
        self.hj, self.s0, self.ds = SOLUTES[solute]
        self.cap, self.water, self.base = hopper_g, water_kg, base_w
        self.reset()

    def reset(self):
        self.T, self.hopper, self.dissolved = 30.0, self.cap, 0.0
        self.cmd = self.delivered = 0.0
        self.frac = 1.0
        self.flags = {"spike": False, "sensor_fault": False,
                      "sensors_lost": False, "pump_fault": False}

    @property
    def load_w(self):
        return (self.base + (self.SPIKE_W if self.flags["spike"] else 0.0)) * self.frac

    def solubility_g(self):
        return (self.s0 + self.ds * (self.T - 20.0)) * self.water

    def read_temps(self):
        if self.flags["sensors_lost"]:
            return [None, None, None]
        t = [self.T + random.gauss(0, 0.15) for _ in range(3)]
        if self.flags["sensor_fault"]:
            t[0] = 99.0
        return t

    def read_hopper_g(self): return self.hopper
    def read_saturation(self): return self.dissolved / self.solubility_g()
    def set_dose(self, g_per_s): self.cmd = g_per_s
    def set_load(self, fraction): self.frac = fraction
    def delivered_dose(self): return self.delivered

    def tick(self, dt):
        want = self.cmd * (0.2 if self.flags["pump_fault"] else 1.0)
        d = max(0.0, min(want, 30.0, self.hopper / dt, (self.solubility_g() - self.dissolved) / dt))
        self.delivered = d
        self.hopper -= d * dt
        self.dissolved += d * dt
        q = self.load_w + 15.0 * (28.0 - self.T) - d * self.hj  # load + room leak - dissolution
        self.T += q * dt / (self.water * CP_WATER)


class Engine:
    """Runs the control loop; keeps history and events for the API."""

    def __init__(self, hal=None, cfg=None, dt=1.0, tick_s=0.1):
        self.cfg = (cfg or Config()).validate()
        self.hal, self.dt, self.tick_s = hal or SimHAL(), dt, tick_s
        self.ctl = Controller(self.cfg)
        self.lock = threading.RLock()
        self._running = False
        self.t, self.last, self.prev_state = 0.0, None, None
        self.hist, self.events = deque(maxlen=900), deque(maxlen=40)
        self.step()

    def log(self, msg):
        self.events.append({"t": self.t, "msg": msg})

    def step(self):
        with self.lock:
            h = self.hal
            d = self.ctl.step(h.read_temps(), h.read_hopper_g(), h.read_saturation(),
                              h.delivered_dose(), self.dt)
            if d["state"] != self.prev_state:
                self.log(f'{d["state"]}: {d["reason"]}')
                self.prev_state = d["state"]
            h.set_dose(d["dose"])
            h.set_load(d["load"])
            h.tick(self.dt)
            self.t += self.dt
            true_t = round(getattr(h, "T", d["T"]), 2)
            self.last = {**d, "t": self.t, "true_T": true_t,
                         "hopper_kg": round(h.read_hopper_g() / 1000, 2),
                         "hopper_cap_kg": round(getattr(h, "cap", 15000.0) / 1000, 2),
                         "saturation": round(h.read_saturation(), 3),
                         "delivered": round(h.delivered_dose() or 0.0, 2),
                         "load_w": round(getattr(h, "load_w", 0.0))}
            self.hist.append({"t": self.t, "T": true_t, "dose": d["dose"], "state": d["state"]})

    def scenario(self, name):
        with self.lock:
            h = self.hal
            if name == "reset":
                h.reset()
                self.ctl.reset()
                self.hist.clear()
                self.events.clear()
                self.t, self.prev_state = 0.0, None
                self.step()
            elif name == "refill" and hasattr(h, "cap"):
                h.hopper = h.cap
                self.log("hopper refilled")
            elif name in getattr(h, "flags", {}):
                h.flags[name] = not h.flags[name]
                self.log(f'scenario {name}: {"ON" if h.flags[name] else "off"}')
            else:
                raise KeyError(name)
            return self.snapshot()

    def update_config(self, patch):
        with self.lock:
            known = {f.name for f in fields(Config)}
            if not set(patch) <= known:
                raise ValueError(f"unknown keys: {sorted(set(patch) - known)}")
            new = Config(**{**asdict(self.cfg), **{k: float(v) for k, v in patch.items()}}).validate()
            self.cfg = self.ctl.cfg = new
            self.log("config updated")
            return asdict(new)

    def snapshot(self, since=None, history=False):
        with self.lock:
            s = {**self.last, "events": list(self.events)[-10:],
                 "flags": dict(getattr(self.hal, "flags", {})), "config": asdict(self.cfg)}
            if since is not None:
                since = -1.0 if self.t < since else since  # engine was reset
                s["new"] = [p for p in self.hist if p["t"] > since]
            if history:
                s["history"] = list(self.hist)
            return s

    def start(self):
        if not self._running:
            self._running = True
            threading.Thread(target=self._loop, daemon=True).start()

    def stop(self):
        self._running = False

    def _loop(self):
        while self._running:
            t0 = time.time()
            try:
                self.step()
            except Exception as e:  # watchdog: any fault -> safe state
                try:
                    self.hal.set_load(0.0)
                    self.log(f"LOOP ERROR, load cut: {e}")
                except Exception:
                    pass
            time.sleep(max(0.0, self.tick_s - (time.time() - t0)))
