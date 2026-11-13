"""Autonomous thermal controller for data centers with no room for a radiator.

Every control step does three things:
  1. Sense      Three redundant temperature sensors are median-voted.
  2. Supervise  A deterministic safety supervisor (highest priority) picks
                NORMAL / WARNING / THROTTLE / SHUTDOWN and overrides everything below it.
  3. Act        The selected optimisation mode (efficiency / performance / stealth)
                drives one of three cooling sinks:
                  salt   terrestrial: endothermic salt dissolution (consumable)
                  pcm    terrestrial: reusable phase-change material modules
                  orbit  orbital: radiators that can only shed heat as thermal radiation

Ground and Orbit are lumped-capacitance simulators for the demo.
Real hardware plugs in by subclassing HAL (see README).
"""
import math
import random
import threading
import time
from collections import deque
from dataclasses import asdict, dataclass, fields
from enum import Enum

CP_WATER = 4180.0   # J/(kg K)
SIGMA = 5.67e-8     # Stefan-Boltzmann constant, W/(m^2 K^4)
AMBIENT = 28.0      # terrestrial room temperature, C
MELT = 48.3         # sodium thiosulfate pentahydrate melting point, C
LATENT = 200e3      # its latent heat of fusion, J/kg
# Endothermic solutes: J absorbed per gram dissolved, solubility (g per kg water) at 20 C, extra g/kg per C
SOLUTES = {"ammonium_chloride": (277.0, 372.0, 4.3),   # dH(soln) = +14.8 kJ/mol
           "urea": (256.0, 1080.0, 29.0)}               # dH(soln) = +15.4 kJ/mol
ENVS, SINKS, MODES = ("terrestrial", "orbital"), ("salt", "pcm"), ("efficiency", "performance", "stealth")
SETPOINT = {"salt": {"efficiency": 40.0, "performance": 25.0, "stealth": AMBIENT},
            "pcm": dict.fromkeys(MODES, MELT + 0.3),
            "orbit": {"efficiency": 30.0, "performance": 20.0, "stealth": 45.0}}
DT = {"salt": 1.0, "pcm": 2.0, "orbit": 20.0}   # simulated seconds per control step
SPEEDS = (1, 5, 20)


def solubility(solute, temp_c):
    """Grams of solute that 1 kg of water can hold at temp_c (linear fit to handbook data)."""
    _, s20, per_c = SOLUTES[solute]
    return s20 + per_c * (temp_c - 20.0)


def tile(label, value, pct=None, sub=""):
    """One dashboard tile."""
    return {"label": label, "value": value, "pct": pct, "sub": sub}


class State(str, Enum):
    NORMAL = "NORMAL"
    WARNING = "WARNING"    # alert; maximum cooling when hot
    THROTTLE = "THROTTLE"  # shed IT load to 50%
    SHUTDOWN = "SHUTDOWN"  # clean server shutdown, latched until the coolant cools down


LOAD_CAP = {State.NORMAL: 1.0, State.WARNING: 1.0, State.THROTTLE: 0.5, State.SHUTDOWN: 0.0}


@dataclass
class Config:
    """Safety thresholds in C. validate() enforces their order and the margins between them."""
    t_freeze: float = 5.0          # orbital: the water must never get this cold
    t_freeze_warn: float = 10.0    # orbital: the freeze guard starts here
    restart_below: float = 46.0    # a SHUTDOWN only clears once the coolant is back under this
    t_warn: float = 52.0           # alert + maximum cooling
    t_throttle: float = 56.0       # shed IT load
    t_shutdown: float = 60.0       # T_safe: clean shutdown of the servers
    t_damage: float = 85.0         # hardware limit; must stay at least 10 C above T_safe
    sat_switch: float = 0.85       # salt: swap to the reserve tank at this saturation
    min_hopper_g: float = 1500.0   # salt: warn when the solute hopper drops below this
    max_dose: float = 25.0         # salt: dosing pump limit, g/s
    max_sensor_gap: float = 3.0    # sensors further apart than this get voted on
    max_rate: float = 0.05         # C/s the coolant can't physically exceed: faster readings are glitches
    lookahead_s: float = 60.0      # prediction horizon (x10 in orbit, where things move slower)

    def validate(self):
        ordered = (self.t_freeze < self.t_freeze_warn < self.restart_below < self.t_warn
                   < self.t_throttle < self.t_shutdown)
        if not ordered or self.t_damage - self.t_shutdown < 10 or not 0.5 <= self.sat_switch < 1.0:
            raise ValueError("need freeze < freeze_warn < restart < warn < throttle < shutdown, "
                             ">=10 C margin before damage, and 0.5 <= sat_switch < 1")
        return self


class Controller:
    """The brain. Hardware-agnostic: an observation dict goes in, a command dict comes out."""

    def __init__(self, cfg, mode="efficiency"):
        self.mode = mode
        self.set_config(cfg)
        self.reset()

    def set_config(self, cfg):
        """Keep three copies of the safety thresholds (triple modular redundancy)."""
        self.cfg = cfg
        self.copies = [asdict(cfg) for _ in range(3)]

    def scrub(self, notes):
        """Radiation can flip bits in memory. Majority-vote every threshold across the live config and
        its three copies each step, and repair whichever one disagrees."""
        live = asdict(self.cfg)
        for k in live:
            votes = [live[k]] + [c[k] for c in self.copies]
            good = max(set(votes), key=votes.count)
            if votes.count(good) < 3:       # no clear majority: fall back to the factory default
                good = getattr(Config(), k)
            if live[k] != good:
                setattr(self.cfg, k, good)
                notes.append(f"memory scrub: repaired corrupted {k}")
            for c in self.copies:
                c[k] = good

    def reset(self):
        self.integ = self.slope = self.last_dose = self.clock = self.throttle_until = 0.0
        self.bad_steps = self.solo_steps = 0
        self.prev = self.module = self.last_module = None
        self.eff = 1.0                      # learned dosing-pump effectiveness
        self.regen = self.cold = self.hide = False
        self.state, self.reason = State.NORMAL, "ok"

    # ---------- 1. sense ----------
    def fuse(self, readings, dt=1.0):
        """Median vote -> (temperature, health). health: 0 ok, 1 degraded, 2 no usable data."""
        c = self.cfg
        good = sorted(r for r in readings if r is not None and -40 < r < 150)
        solo = False
        if self.prev is not None:
            # The coolant can't physically move max_jump in one step, so such a reading is a glitch
            # (e.g. a radiation bit flip), even if two sensors are hit at once.
            jump = max(0.5, c.max_rate * dt)
            plausible = [r for r in good if abs(r - self.prev) <= jump]
            if not plausible and good and self.solo_steps < 1:
                self.solo_steps += 1            # every reading hit at once: hold the last trusted value
                return self.prev, 0
            if len(plausible) == 1 and len(good) > 1 and self.solo_steps < 5:
                good, solo = plausible, True    # briefly trust the one that agrees with recent history
            elif plausible:
                good = plausible
        self.solo_steps = self.solo_steps + 1 if solo else 0
        if solo:
            self.bad_steps += 1
            return good[0], (1 if self.bad_steps >= 5 else 0)
        if len(good) < 2:
            return (good[-1] if good else c.t_shutdown + 1), 2   # can't vote: assume the worst
        if good[-1] - good[0] > c.max_sensor_gap:
            med = good[len(good) // 2]
            kept = [g for g in good if abs(g - med) <= c.max_sensor_gap]
            if len(kept) < 2:
                return good[-1], 2
            good = kept
        # One-off glitches are outvoted silently; a sensor that stays bad is flagged for maintenance.
        self.bad_steps = self.bad_steps + 1 if len(good) < len(readings) else 0
        return sum(good) / len(good), (1 if self.bad_steps >= 5 else 0)

    def setpoint(self, o):
        kind = o["kind"]
        mode = "efficiency" if self.mode == "stealth" and o.get("window") else self.mode
        sp = SETPOINT[kind][mode]
        if kind == "salt" and mode == "performance" and o["forecast_w"] > 1.2 * o["demand_w"]:
            sp -= 4.0                       # pre-cool before a scheduled heavy job
        return sp

    def reserve(self, o, T, sp):
        """How much cooling the sink has left -> (saturation, low, exhausted)."""
        c, kind = self.cfg, o["kind"]
        if kind == "salt":
            # Saturation is measured against the coldest temperature we plan to run at:
            # NH4Cl is less soluble when cold, so cooling a nearly saturated tank would grow crystals.
            sat = o["conc_g_per_kg"] / solubility(o["solute"], min(T, sp))
            spare = o["tanks"] > 0
            out = o["hopper_g"] < 1.0 or (sat >= c.sat_switch and not spare)
            low = out or o["hopper_g"] < c.min_hopper_g or (sat >= c.sat_switch - 0.1 and not spare)
            return sat, low, out
        if kind == "pcm":
            usable = len(self.usable_modules(o["melt"]))
            return None, usable <= 1, usable == 0
        return None, False, False

    # ---------- 2. supervise (highest priority) ----------
    def supervise(self, T, hot, health, low, out):
        c, prev = self.cfg, self.state
        if hot >= c.t_shutdown or (health == 2 and T >= c.t_throttle):
            return State.SHUTDOWN, ("no trustworthy temperature data" if health == 2
                                    else f"{hot:.0f} °C reached or predicted")
        if prev == State.SHUTDOWN and T > c.restart_below:
            return State.SHUTDOWN, self.reason                  # latched until cooled
        if prev == State.SHUTDOWN and out:                      # no point restarting with nothing to cool
            return State.SHUTDOWN, "cooling sink exhausted: refill or regenerate to restart"
        if out:
            return State.THROTTLE, "cooling sink exhausted: graceful power-down"
        # Once throttled, hold for >= 5 min, and until the coolant is under t_warn AND actually cooling.
        held = prev == State.THROTTLE and (hot >= c.t_warn or self.slope > 0.001 or self.clock < self.throttle_until)
        if hot >= c.t_throttle or held:
            return State.THROTTLE, "too hot: shedding load"
        if low and hot >= c.t_warn:
            return State.THROTTLE, "cooling reserve low while hot"
        if hot >= c.t_warn:
            return State.WARNING, "approaching limit: maximum cooling"
        if health:
            return State.WARNING, "sensor fault: outvoted, maintenance needed"
        if self.cold:
            return State.WARNING, "freeze risk: guard active"
        if low:
            return State.WARNING, "cooling reserve low"
        return State.NORMAL, "ok"

    # ---------- 3. act ----------
    def act_salt(self, o, Tp, sp, sat, hot, out, health, dt, notes):
        c = self.cfg
        cmd = {"swap": sat >= c.sat_switch and o["tanks"] > 0, "dose": 0.0}
        if cmd["swap"]:
            notes.append(f"saturation {sat:.0%}: switched to the reserve tank")
        if self.last_dose > 1.0:            # learn if the pump delivers less than asked
            ratio = o["delivered"] / self.last_dose
            self.eff = 0.98 * self.eff + 0.02 * max(0.2, min(1.0, ratio))
        if out or health == 2:              # stop: crystals would clog the cold plates / flying blind
            self.last_dose = 0.0
            return cmd
        err = Tp - sp
        self.integ = min(300.0, self.integ + err * dt) if 0 < err < 8 else self.integ * 0.98
        # Feedforward: dose for the heat the servers are making now (performance: also what's coming).
        power = max(o["load_w"], o["forecast_w"]) if self.mode == "performance" else o["load_w"]
        feedforward = power * 0.8 / o["heat_j_per_g"] * min(1.0, max(0.0, err + 1.0))
        dose = (1.2 * max(err, 0.0) + 0.02 * self.integ + feedforward) / self.eff
        if hot >= c.t_warn:
            dose = c.max_dose
        self.last_dose = cmd["dose"] = min(max(dose, 0.0), c.max_dose)
        return cmd

    def usable_modules(self, melt):
        """The module in use until it is 95% melted; others only once re-solidified below 80%."""
        return [i for i, x in enumerate(melt) if x < 0.8 or (i == self.module and x < 0.95)]

    def act_pcm(self, o, T, Tp, health, notes):
        m, eng = o["melt"], (None if health == 2 else self.module)   # blind: servers off, park the modules
        if eng is not None and (m[eng] >= 0.95 or T < MELT):   # below MELT it would hand heat back
            if m[eng] >= 0.95:
                notes.append(f"module M{eng + 1} spent")
            eng = None
        if eng is None and Tp > MELT + 0.3 and health < 2:
            fresh = [i for i in self.usable_modules(m) if m[i] < 0.8]
            if fresh:
                eng = min(fresh, key=lambda i: m[i])
                if eng != self.last_module:
                    notes.append(f"switched to fresh module M{eng + 1}")
                self.last_module = eng
        self.module = eng
        spent = sum(m) / len(m)
        regen = self.mode != "performance" and (spent > 0.5 or (self.regen and spent > 0.2))
        if regen != self.regen:
            notes.append("PCM half spent: low-power state so modules re-solidify" if regen
                         else "PCM recovered: full power")
        self.regen = regen
        fan = self.mode != "stealth" or bool(o.get("window"))   # stealth: keep heat in until a window
        return {"module": eng, "fan": fan}

    def act_orbit(self, o, T, Tp, sp, hot, health, notes):
        c = self.cfg
        theta, dep = o["theta"], o["dep"]
        edge_on = 90.0 if theta >= 0 else -90.0   # nearest attitude with no sunlight on the radiator face
        if health == 2:
            # Blind with the servers off: retract to slow every temperature change, hold attitude,
            # and hand freeze protection to the survival heater's hardware thermostat.
            return {"theta": theta, "dep": 0.05, "heater": 0.0, "survival": True}
        # A fully deployed radiator would remove net = eps*sigma*A*T^4 - absorbed sunlight.
        net = o["emit_w"] - o["absorb_w"]
        if self.cold:                       # freeze guard: keep the water liquid
            heater = min(300.0, max(0.0, 60.0 * (c.t_freeze_warn + 3 - T)))
            return {"theta": 0.0, "dep": 1.0 if net < 0 else 0.05, "heater": heater, "survival": False}
        hide = net < -0.05 * o["emit_w"] or (self.hide and net < 0.05 * o["emit_w"])
        if hide != self.hide:
            notes.append("radiators absorb more than they emit: retracting behind MLI" if hide
                         else "radiators emit more than they absorb: redeploying")
        self.hide = hide
        # The sun sweeps around the craft every orbit. Efficiency lets the attitude float (None) in
        # eclipse and while the sun stays near-grazing, saving reaction-wheel wear; otherwise it
        # slews back to edge-on.
        grazing = not o.get("sunlit", True) or math.cos(math.radians(theta)) < 0.26
        theta = None if self.mode == "efficiency" and grazing and Tp <= sp + 2 else edge_on
        # Feedforward: the deployment that exactly balances the heat coming in (servers + absorbed)
        # against what the radiators emit. Eclipse entry/exit changes absorb_w, so this reacts at once.
        if net > 1.0:
            ff = min(1.0, max(0.0, (o["load_w"] / net - 0.05) / 0.95))
        else:
            ff = 1.0
        if self.mode == "performance":      # as cold as possible, but no colder than the setpoint
            dep = ff + 0.25 * (Tp - sp)
        elif self.mode == "stealth" and not o.get("window"):
            dep = 0.05 if Tp < sp else ff + 0.25 * (Tp - sp + 1)
        else:                               # efficiency: float in a wide band
            dep = ff + 0.1 * (Tp - sp)
        dep = min(1.0, max(0.05, dep))
        if hot >= c.t_warn:
            dep = 1.0
        if hide:
            dep = 0.05
        return {"theta": theta, "dep": dep, "heater": 0.0, "survival": False}

    def step(self, o, dt):
        notes = []
        self.scrub(notes)
        c, kind = self.cfg, o["kind"]
        T, health = self.fuse(o["temps"], dt)
        if health == 2:                     # blind: freeze the predictor, restart it when data returns
            self.prev, self.slope = None, 0.0
        elif self.prev is not None:
            self.slope = 0.8 * self.slope + 0.2 * (T - self.prev) / dt
        if health < 2:
            self.prev = T
        Tp = T + self.slope * c.lookahead_s * (10 if kind == "orbit" else 1)
        hot, sp = max(T, Tp), self.setpoint(o)
        sat, low, out = self.reserve(o, T, sp)
        if kind == "orbit" and health < 2:
            low_t = min(T, Tp)
            cold = low_t < c.t_freeze_warn or (self.cold and low_t < c.t_freeze_warn + 8)
            if cold != self.cold:
                notes.append("freeze guard ON: retract, face the sun, heater" if cold else "freeze guard off")
            self.cold = cold
        self.clock += dt
        prev = self.state
        self.state, self.reason = self.supervise(T, hot, health, low, out)
        if self.state == State.THROTTLE and prev != State.THROTTLE:
            self.throttle_until = self.clock + 300.0
        cmd = {"load": LOAD_CAP[self.state]}
        if kind == "salt":
            cmd.update(self.act_salt(o, Tp, sp, sat, hot, out, health, dt, notes))
            act = cmd["dose"] / c.max_dose
        elif kind == "pcm":
            cmd.update(self.act_pcm(o, T, Tp, health, notes))
            act = sum(o["melt"]) / len(o["melt"])
        else:
            cmd.update(self.act_orbit(o, T, Tp, sp, hot, health, notes))
            act = cmd["dep"]
        reason = self.reason
        if self.regen:
            cmd["load"] = min(cmd["load"], 0.4)
            if self.state == State.NORMAL:
                reason = "regenerating PCM: load capped at 40%"
        return cmd, {"T": round(T, 2), "T_pred": round(Tp, 2), "setpoint": round(sp, 1),
                     "state": self.state.value, "reason": reason, "health": health,
                     "eff": round(self.eff, 2), "sat": None if sat is None else round(sat, 3),
                     "regen": self.regen, "mode": self.mode, "act": round(act, 3), "notes": notes}


class HAL:
    """Hardware contract. read() returns sensors, apply(cmd) drives actuators (keys in README)."""
    kind = "salt"
    flags = {}

    def read(self): raise NotImplementedError
    def apply(self, cmd): raise NotImplementedError
    def tick(self, dt): pass                          # simulators advance physics here
    def reset(self): pass
    def tiles(self, info): return []
    def scenarios(self): return [("reset", "↺ Reset")]


class Sim(HAL):
    """Shared parts of the demo simulators."""

    def temps(self):
        if self.flags.get("sensors_lost"):
            return [None, None, None]
        t = [self.T + random.gauss(0, 0.02) for _ in range(3)]
        if self.flags.get("sensor_fault"):
            t[0] = 99.0
        return t

    def apply(self, cmd):
        self.cmd.update(cmd)


class Ground(Sim):
    """Server room with no radiator: a 20 kg water loop that leaks a little heat to the room."""
    BASE_W, SPIKE_W, WATER_KG, LEAK_W_PER_K = 2000.0, 3500.0, 20.0, 15.0
    MODULES, MODULE_KG, MODULE_UA, REGEN_W_PER_K = 4, 8.0, 4000.0, 20.0

    def __init__(self, kind="salt", solute="ammonium_chloride", hopper_g=15000.0):
        self.kind, self.solute, self.cap = kind, solute, hopper_g
        self.heat_j_per_g = SOLUTES[solute][0]
        self.reset()

    def reset(self):
        self.T = 30.0 if self.kind == "salt" else 48.0
        self.hopper, self.dissolved, self.tanks = self.cap, 0.0, 1
        self.total_g = self.delivered = self.leak_w = self.released_w = 0.0
        self.melt, self.engaged = [0.0] * self.MODULES, None
        self.burst_in = self.burst_left = 0.0
        self.cmd = {"load": 1.0, "dose": 0.0, "swap": False, "module": None, "fan": True}
        self.flags = {"spike": False, "window": False, "sensor_fault": False, "sensors_lost": False}
        if self.kind == "salt":
            self.flags["pump_fault"] = False

    def refill(self):
        self.hopper, self.tanks, self.dissolved = self.cap, 1, 0.0

    def schedule_burst(self):
        self.burst_in = 90.0

    def demand_w(self):
        return self.BASE_W + (self.SPIKE_W if self.flags["spike"] or self.burst_left > 0 else 0.0)

    @property
    def load_w(self):
        return self.demand_w() * self.cmd["load"]

    def read(self):
        soon = self.flags["spike"] or self.burst_in > 0 or self.burst_left > 0
        o = {"kind": self.kind, "temps": self.temps(), "load_w": self.load_w, "demand_w": self.demand_w(),
             "forecast_w": self.BASE_W + (self.SPIKE_W if soon else 0.0), "window": self.flags["window"]}
        if self.kind == "salt":
            o.update(hopper_g=self.hopper, conc_g_per_kg=self.dissolved / self.WATER_KG, solute=self.solute,
                     tanks=self.tanks, delivered=self.delivered, heat_j_per_g=self.heat_j_per_g)
        else:
            o.update(melt=list(self.melt))
        return o

    def tick(self, dt):
        if self.burst_in > 0:
            self.burst_in -= dt
            if self.burst_in <= 0:
                self.burst_left = 600.0
        elif self.burst_left > 0:
            self.burst_left -= dt
        self.leak_w = self.LEAK_W_PER_K * (self.T - AMBIENT)    # heat lost to the room
        q = self.load_w - self.leak_w
        q -= self.dissolve(dt) if self.kind == "salt" else self.pcm(dt)
        self.T += q * dt / (self.WATER_KG * CP_WATER)

    def dissolve(self, dt):
        c = self.cmd
        if c.get("swap") and self.tanks > 0:
            self.tanks, self.dissolved, c["swap"] = self.tanks - 1, 0.0, False
        room = solubility(self.solute, self.T) * self.WATER_KG - self.dissolved
        want = c["dose"] * (0.2 if self.flags["pump_fault"] else 1.0)
        g = max(0.0, min(want, 30.0, self.hopper / dt, room / dt))
        self.delivered = g
        self.hopper -= g * dt
        self.dissolved += g * dt
        self.total_g += g * dt
        self.released_w = max(0.0, self.leak_w)
        return g * self.heat_j_per_g                   # watts absorbed by dissolving

    def pcm(self, dt):
        c, cap = self.cmd, self.MODULE_KG * LATENT
        self.engaged = c.get("module")
        absorbed = released = 0.0
        for i, m in enumerate(self.melt):
            if i == self.engaged:                      # coolant loop <-> module
                new = min(1.0, max(0.0, m + self.MODULE_UA * (self.T - MELT) * dt / cap))
                absorbed += (new - m) * cap / dt
            else:                                      # idle: re-solidify by shedding heat to the room
                rate = self.REGEN_W_PER_K * (MELT - AMBIENT) * (1.0 if c.get("fan", True) else 0.1)
                new = max(0.0, m - rate * dt / cap)
                released += (m - new) * cap / dt
            self.melt[i] = new
        self.released_w = max(0.0, self.leak_w) + released
        return absorbed

    def tiles(self, info):
        if self.kind == "salt":
            sat = info["sat"] or 0.0
            name = self.solute.replace("_", " ")
            return [tile("Solute dosing", f"{self.delivered:.1f} g/s", None,
                         f"commanded {self.cmd['dose']:.1f} · pump health {info['eff']:.0%}"),
                    tile("Solute hopper", f"{self.hopper / 1000:.1f} kg", self.hopper / self.cap * 100,
                         f"{name} · used {self.total_g / 1000:.1f} kg"),
                    tile("Saturation", f"{sat:.0%}", sat * 100,
                         f"reserve swap at 85% · reserve tanks {self.tanks} · pump load ×{1 + 2 * sat:.2f}"),
                    tile("Heat released to room", f"{self.released_w:.0f} W", None,
                         "heat escaping into the room (stealth mode keeps this near 0)")]
        avg = sum(self.melt) / self.MODULES
        mods = " · ".join(f"M{k + 1} {m:.0%}{'◀' if k == self.engaged else ''}" for k, m in enumerate(self.melt))
        fan = "fan on" if self.cmd.get("fan", True) else "insulated (stealth)"
        return [tile("PCM melted", f"{avg:.0%}", avg * 100, mods + "   ◀ = in use"),
                tile("Regeneration", "low-power" if info["regen"] else "normal", None,
                     f"idle modules: {fan}"),
                tile("Heat released to room", f"{self.released_w:.0f} W", None,
                     "heat escaping from the loop and from modules as they re-solidify")]

    def scenarios(self):
        s = [("spike", "⚡ Sustained load"), ("burst", "📅 Scheduled job"), ("window", "🪟 Safe window"),
             ("sensor_fault", "🌡 1 sensor fails"), ("sensors_lost", "🚫 All sensors lost")]
        if self.kind == "salt":
            s += [("pump_fault", "🔧 Pump degrades"), ("refill", "⛽ Refill")]
        return s + [("reset", "↺ Reset")]


class Orbit(Sim):
    """Relay in low Earth orbit: heat leaves only by radiation, Q = eps*sigma*A*T^4.

    Models a ~400 km orbit: 90-minute period with a 35-minute eclipse, the sun's apparent motion
    (the craft points at Earth, so the sun sweeps around it once per orbit and the ADCS has to keep
    slewing to stay edge-on), Earth's infrared glow and reflected sunlight (albedo).
    """
    kind = "orbit"
    BASE_W, AREA, EPS, ALPHA, SUN, WATER_KG = 500.0, 1.5, 0.85, 0.15, 1361.0, 4.0
    PERIOD, ECLIPSE = 5400.0, 2100.0           # s; ~400 km, sun in the orbit plane (beta = 0)
    EARTH_IR, ALBEDO, VIEW = 237.0, 0.30, 0.25  # W/m2, fraction reflected, radiator's view factor to Earth
    SEU_RATE = 0.05                            # radiation storm: chance per sensor reading of a bit flip

    def __init__(self):
        self.reset()

    def reset(self):
        self.T, self.theta, self.dep, self.heater, self.slewed, self.clock = 25.0, 90.0, 1.0, 0.0, 0.0, 0.0
        self.cmd = {"load": 1.0, "theta": 90.0, "dep": 1.0, "heater": 0.0}
        self.flags = {"solar_pass": False, "high_beta": False, "adcs_fault": False, "idle": False,
                      "window": False, "radiation": False, "sensor_fault": False, "sensors_lost": False}

    # ---------- orbit geometry ----------
    def phase(self):
        return (self.clock % self.PERIOD) / self.PERIOD

    def sunlit(self):
        """High beta angle: the orbit plane faces the sun and the craft never enters Earth's shadow."""
        return self.flags["high_beta"] or self.phase() < 1.0 - self.ECLIPSE / self.PERIOD

    def next_transition_s(self):
        if self.flags["high_beta"]:
            return None
        t, day = self.clock % self.PERIOD, self.PERIOD - self.ECLIPSE
        return day - t if t < day else self.PERIOD - t

    def exposure(self):
        return 0.05 + 0.95 * self.dep          # fully retracted = 5% still exposed behind the MLI shield

    def absorb_full(self):
        """Heat a fully deployed radiator picks up: sunlight + Earth's infrared + reflected sunlight."""
        earth_ir = self.EPS * self.EARTH_IR * self.VIEW * self.AREA      # IR absorptivity = emissivity
        if not self.sunlit():
            return earth_ir
        sun = self.SUN * (3.0 if self.flags["solar_pass"] else 1.0)
        direct = self.ALPHA * sun * self.AREA * max(0.0, math.cos(math.radians(self.theta)))
        day = self.PERIOD - self.ECLIPSE
        noon = 0.0 if self.flags["high_beta"] else math.cos(math.pi * ((self.clock % self.PERIOD) / day - 0.5))
        albedo = self.ALPHA * sun * self.ALBEDO * self.VIEW * self.AREA * max(0.0, noon) * (0.3 if self.flags["high_beta"] else 1.0)
        return direct + albedo + earth_ir

    def emit_full(self):                       # what a fully deployed radiator would emit
        return self.EPS * SIGMA * self.AREA * (self.T + 273.15) ** 4

    @property
    def load_w(self):
        return self.BASE_W * (0.05 if self.flags["idle"] else 1.0) * self.cmd["load"]

    def temps(self):
        t = super().temps()
        if self.flags["radiation"]:            # single-event upsets: random bit flips in sensor readings
            t = [random.uniform(-60, 200) if x is not None and random.random() < self.SEU_RATE else x for x in t]
        return t

    def read(self):
        return {"kind": "orbit", "temps": self.temps(), "load_w": self.load_w, "demand_w": self.BASE_W,
                "forecast_w": self.BASE_W, "theta": self.theta, "dep": self.dep,
                "absorb_w": self.absorb_full(), "emit_w": self.emit_full(), "sunlit": self.sunlit(),
                "transition_s": self.next_transition_s(),
                "window": self.flags["window"], "adcs_ok": not self.flags["adcs_fault"]}

    def tick(self, dt):
        c = self.cmd
        # Pointing at Earth, the sun sweeps 360 degrees around the craft every orbit.
        drift = 360.0 * dt / self.PERIOD
        if self.flags["adcs_fault"]:
            self.theta = 20.0                  # stuck facing the sun
        elif c["theta"] is None:               # attitude left to float: no reaction-wheel effort
            self.theta = ((self.theta + drift + 180.0) % 360.0) - 180.0
        else:
            want = ((c["theta"] - (self.theta + drift) + 180.0) % 360.0) - 180.0
            turn = max(-dt, min(dt, want))     # reaction wheels slew at 1 deg/s
            self.theta = ((self.theta + drift + turn + 180.0) % 360.0) - 180.0
            self.slewed += abs(turn)
        self.dep += max(-0.02 * dt, min(0.02 * dt, c["dep"] - self.dep))
        survival = 60.0 if c.get("survival") and self.T < 10.0 else 0.0   # hardware thermostat, no software
        self.heater = c["heater"] + survival
        q = self.load_w + self.heater + (self.absorb_full() - self.emit_full()) * self.exposure()
        self.T += q * dt / (self.WATER_KG * CP_WATER)
        self.clock += dt

    def tiles(self, info):
        f = self.exposure()
        stuck = " · ADCS FAULT" if self.flags["adcs_fault"] else ""
        nxt = self.next_transition_s()
        where = "☀ Sunlit" if self.sunlit() else "🌑 Eclipse"
        sub = ("high beta angle: no eclipse" if nxt is None
               else f"{'eclipse' if self.sunlit() else 'sunrise'} in {nxt / 60:.0f} min · orbit {self.phase():.0%}")
        return [tile("Orbit", where, self.phase() * 100, sub),
                tile("Radiators deployed", f"{self.dep:.0%}", self.dep * 100, "retracted = behind the MLI shield"),
                tile("Attitude to sun", f"{self.theta:.0f}°", None,
                     f"±90° = edge-on · slewed {self.slewed:.0f}° total{stuck}"),
                tile("Radiated to space", f"{self.emit_full() * f:.0f} W", None,
                     f"absorbed (sun, Earth IR, albedo) {self.absorb_full() * f:.0f} W · heater {self.heater:.0f} W")]

    def scenarios(self):
        return [("solar_pass", "☀ Close solar pass"), ("high_beta", "🌞 No eclipse (high β)"),
                ("adcs_fault", "🛰 ADCS fault"), ("idle", "❄ Servers idle"), ("window", "🪟 Safe window"),
                ("radiation", "☢ Radiation storm"), ("sensor_fault", "🌡 1 sensor fails"),
                ("sensors_lost", "🚫 All sensors lost"), ("reset", "↺ Reset")]


class Engine:
    """Runs the control loop in a background thread and keeps history and an event log for the API."""

    def __init__(self, env="terrestrial", sink="salt", mode="efficiency", cfg=None, hal=None, tick_s=0.1):
        self.cfg = (cfg or Config()).validate()
        self.tick_s, self.custom, self.speed = tick_s, hal, 1
        self.lock, self._running = threading.RLock(), False
        self.build(env, sink, mode)

    def build(self, env, sink, mode):
        if env not in ENVS or sink not in SINKS or mode not in MODES:
            raise ValueError(f"env must be one of {ENVS}, sink {SINKS}, mode {MODES}")
        with self.lock:
            self.env, self.sink, self.mode = env, sink, mode
            kind = "orbit" if env == "orbital" else sink
            self.hal = self.custom or (Orbit() if kind == "orbit" else Ground(kind))
            self.dt = self.tick_s if self.custom else DT[kind]    # real hardware runs in real time
            self.ctl = Controller(self.cfg, mode)
            self.t, self.prev_state = 0.0, None
            self.hist, self.events = deque(maxlen=900), deque(maxlen=40)
            self.step()

    def setup(self, env=None, sink=None, mode=None, speed=None):
        with self.lock:
            env, sink, mode = env or self.env, sink or self.sink, mode or self.mode
            if speed is not None:
                if int(speed) not in SPEEDS:
                    raise ValueError(f"speed must be one of {SPEEDS}")
                self.speed = int(speed)
            if (env, sink) != (self.env, self.sink):
                if self.custom:
                    raise ValueError("custom hardware: environment is fixed")
                self.build(env, sink, mode)
            elif mode != self.mode:
                if mode not in MODES:
                    raise ValueError(f"mode must be one of {MODES}")
                self.mode = self.ctl.mode = mode
                self.ctl.integ = 0.0
                self.log(f"mode switched to {mode}")
            return self.snapshot(history=True)

    def log(self, msg):
        self.events.append({"t": self.t, "msg": msg})

    def step(self):
        with self.lock:
            h = self.hal
            if h.flags.get("radiation") and random.random() < 0.02:   # a bit flip in the controller's RAM
                k = random.choice([f.name for f in fields(Config)])
                setattr(self.ctl.cfg, k, getattr(self.ctl.cfg, k) * random.choice((0.0, 4.0, -1.0)))
            cmd, info = self.ctl.step(h.read(), self.dt)
            if info["state"] != self.prev_state:
                self.log(f'{info["state"]}: {info["reason"]}')
                self.prev_state = info["state"]
            for note in info.pop("notes"):
                self.log(note)
            h.apply(cmd)
            h.tick(self.dt)
            self.t += self.dt
            true_t = round(getattr(h, "T", info["T"]), 2)
            self.last = {**info, "t": self.t, "true_T": true_t, "kind": h.kind, "load": cmd["load"],
                         "tiles": h.tiles(info) + [tile("IT load allowed", f'{cmd["load"]:.0%}', cmd["load"] * 100)]}
            self.hist.append({"t": self.t, "T": true_t, "act": info["act"], "state": info["state"]})

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
            elif name == "refill" and hasattr(h, "refill"):
                h.refill()
                self.log("hopper and reserve tank refilled")
            elif name == "burst" and hasattr(h, "schedule_burst"):
                h.schedule_burst()
                self.log("heavy job scheduled in 90 s")
            elif name in h.flags:
                h.flags[name] = not h.flags[name]
                self.log(f'scenario {name.replace("_", " ")}: {"ON" if h.flags[name] else "off"}')
            else:
                raise KeyError(name)
            return self.snapshot(history=True)

    def update_config(self, patch):
        with self.lock:
            known = {f.name for f in fields(Config)}
            if not set(patch) <= known:
                raise ValueError(f"unknown keys: {sorted(set(patch) - known)}")
            new = Config(**{**asdict(self.cfg), **{k: float(v) for k, v in patch.items()}}).validate()
            self.cfg = new
            self.ctl.set_config(new)
            self.log("config updated")
            return asdict(new)

    def snapshot(self, since=None, history=False):
        with self.lock:
            s = {**self.last, "events": list(self.events)[-12:], "flags": dict(self.hal.flags),
                 "config": asdict(self.cfg), "scenarios": self.hal.scenarios(),
                 "options": {"env": self.env, "sink": self.sink, "mode": self.mode, "speed": self.speed}}
            if since is not None:
                since = -1.0 if self.t < since else since   # the engine was reset
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
                for _ in range(self.speed):
                    self.step()
            except Exception as e:              # watchdog: any software fault cuts IT load
                try:
                    self.hal.apply({"load": 0.0})
                    self.log(f"LOOP ERROR, load cut: {e}")
                except Exception:
                    pass
            time.sleep(max(0.0, self.tick_s - (time.time() - t0)))
