"""Autonomous thermal controller for data centers with no room for a radiator.

Every control step does three things:
  1. Sense      Three redundant temperature sensors are median-voted, and a sensor that slowly
                drifts away from the others is caught and dropped from the vote.
  2. Supervise  A deterministic safety supervisor (highest priority) watches the coolant AND the
                hottest chip, picks NORMAL / WARNING / THROTTLE / SHUTDOWN and overrides everything below.
  3. Act        The selected mode (efficiency / performance / stealth) drives one of three cooling sinks:
                  salt   terrestrial: endothermic salt dissolution (consumable). Dosed by model-predictive
                         control (MPC) using a thermal model of the room that the controller learns online.
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
CHIP_RISE = 25.0    # the hottest chip runs this much above the coolant at 100% load (cold-plate resistance)
CHIP_TAU = 15.0     # s: chips have thermal mass, so they follow a change in load or cooling over ~15 s
CHIP_LOOKAHEAD = 10.0   # s: how far ahead the supervisor projects the chips' own trend
HOPPER_G = 15000.0  # salt hopper in the demo, g
# Endothermic solutes: heat absorbed per gram dissolved, and solubility as (C, grams per kg of water) points.
SOLUTES = {
    "ammonium_chloride": {"name": "ammonium chloride", "j_per_g": 277.0,            # dH(soln) +14.8 kJ/mol
                          "sol": [(0, 294), (20, 372), (40, 458), (60, 553), (80, 656)]},
    "urea": {"name": "urea", "j_per_g": 256.0,                                         # dH(soln) +15.4 kJ/mol
             "sol": [(20, 1080), (40, 1670), (60, 2510)]},
    "sodium_thiosulfate": {"name": "sodium thiosulfate pentahydrate", "j_per_g": 195.0,  # about +48 kJ/mol
                           "sol": [(0, 1102), (10, 1421), (20, 1830), (30, 2442), (40, 3935)]},
}
ENVS, SINKS, MODES = ("terrestrial", "orbital"), ("salt", "pcm"), ("efficiency", "performance", "stealth")
SETPOINT = {"salt": {"efficiency": 40.0, "performance": 25.0, "stealth": AMBIENT},
            "pcm": dict.fromkeys(MODES, MELT + 0.3),
            "orbit": {"efficiency": 30.0, "performance": 20.0, "stealth": 45.0}}
DT = {"salt": 1.0, "pcm": 2.0, "orbit": 20.0}   # simulated seconds per control step
SPEEDS = (1, 5, 20)
DESIGN_GUESS = {"water_kg": 25.0, "leak_w_per_k": 10.0}   # salt loop's design-sheet values; it learns the real ones


def solubility(solute, temp_c):
    """Grams of solute that 1 kg of water can hold at temp_c (interpolated from handbook data)."""
    pts = SOLUTES[solute]["sol"]
    i = 0
    while i < len(pts) - 2 and temp_c > pts[i + 1][0]:
        i += 1
    (t0, s0), (t1, s1) = pts[i], pts[i + 1]
    return max(50.0, s0 + (s1 - s0) * (temp_c - t0) / (t1 - t0))


def sizing(kw=2.0, hours=1.0, coolant_c=40.0, radiator_c=30.0):
    """How much of each cooling option it takes to soak up `kw` of server heat for `hours`."""
    kw, hours, coolant_c, radiator_c = (float(v) for v in (kw, hours, coolant_c, radiator_c))
    if not (0 < kw <= 10000 and 0 < hours <= 10000 and 0 <= coolant_c <= 45 and -50 <= radiator_c <= 150):
        raise ValueError("use 0-10000 kW, 0-10000 hours, coolant 0-45 °C and radiator -50 to 150 °C")
    joules = kw * 1000 * hours * 3600
    rows = []
    for key, s in SOLUTES.items():
        solid = joules / s["j_per_g"] / 1000
        water = solid / (0.85 * solubility(key, coolant_c) / 1000)
        rows.append({"option": f"Salt: {s['name']}", "amount": round(solid, 1), "unit": "kg", "reusable": False,
                     "also": f"plus about {water:.0f} L of water, to stay under 85% saturation at {coolant_c:g} °C"})
    rows.append({"option": "PCM: sodium thiosulfate pentahydrate", "amount": round(joules / LATENT, 1),
                 "unit": "kg", "reusable": True,
                 "also": f"melts at {MELT} °C, then re-solidifies once it can shed the heat"})
    area = kw * 1000 / (0.85 * SIGMA * ((radiator_c + 273.15) ** 4 - 3.0 ** 4))
    rows.append({"option": "Orbital radiator", "amount": round(area, 2), "unit": "m²", "reusable": True,
                 "also": f"at {radiator_c:g} °C, edge-on to the sun (emissivity 0.85)"})
    hopper_min = HOPPER_G * SOLUTES["ammonium_chloride"]["j_per_g"] / (kw * 1000) / 60
    return {"kwh": round(kw * hours, 2), "mj": round(joules / 1e6, 1), "rows": rows,
            "hopper_minutes": round(hopper_min, 1), "kw": kw}


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
    t_warn: float = 52.0           # coolant: alert + maximum cooling
    t_throttle: float = 56.0       # coolant: shed IT load
    t_shutdown: float = 60.0       # coolant T_safe: clean shutdown of the servers
    t_damage: float = 85.0         # loop hardware limit (pumps, seals, tubing)
    chip_warn: float = 80.0        # hottest chip: alert + maximum cooling
    chip_throttle: float = 85.0    # hottest chip: shed IT load
    chip_shutdown: float = 90.0    # hottest chip: clean shutdown
    chip_damage: float = 100.0     # chip junction limit
    sat_switch: float = 0.85       # salt: swap to the reserve tank at this saturation
    min_hopper_g: float = 1500.0   # salt: warn when the solute hopper drops below this
    max_dose: float = 25.0         # salt: dosing pump limit, g/s
    max_sensor_gap: float = 3.0    # sensors further apart than this get voted on
    drift_limit: float = 0.6       # a sensor this far from the others for about 30 s is drifting
    lookahead_s: float = 60.0      # prediction horizon (x10 in orbit, where things move slower)

    def validate(self):
        coolant = (self.t_freeze < self.t_freeze_warn < self.restart_below < self.t_warn
                   < self.t_throttle < self.t_shutdown)
        chips = self.chip_warn < self.chip_throttle < self.chip_shutdown
        margins = self.t_damage - self.t_shutdown >= 10 and self.chip_damage - self.chip_shutdown >= 10
        if not (coolant and chips and margins and 0.5 <= self.sat_switch < 1.0
                and 0 < self.drift_limit < self.max_sensor_gap):
            raise ValueError("need freeze < freeze_warn < restart < warn < throttle < shutdown, "
                             "chip_warn < chip_throttle < chip_shutdown, >=10 C margins before damage, "
                             "0.5 <= sat_switch < 1 and 0 < drift_limit < max_sensor_gap")
        return self


class ThermalModel:
    """A model of the room that the controller learns while it runs:

        C dT/dt = P - h (T - T_room) - q g      P = server heat (W), g = salt dissolved (g/s)

    It starts from the design-sheet guess and refines C (heat capacity), h (leak to the room) and
    q (heat absorbed per gram) with recursive least squares in Kalman-filter form, one update per
    30 s window (the temperature slope is a least-squares fit over the window, which averages out
    sensor noise). MPC uses it to predict the coolant temperature minutes ahead."""
    WINDOW = 30.0

    def __init__(self, c_j_per_k, h_w_per_k, q_j_per_g):
        self.design = (c_j_per_k, h_w_per_k, q_j_per_g)
        self.th = self.theta(c_j_per_k, h_w_per_k, q_j_per_g)
        self.P = [[(0.5 * self.th[i]) ** 2 if i == j else 0.0 for j in range(3)] for i in range(3)]
        self.R = 0.001 ** 2            # noise + model error of a 30 s temperature slope, (K/s)^2
        self.updates, self.win, self.started = 0, [], False

    @staticmethod
    def theta(c, h, q):                # parameters in scaled form: features are [P/1000, (T-T_room)/10, g/10]
        return [1000.0 / c, -10.0 * h / c, -10.0 * q / c]

    @property
    def params(self):                  # -> C (J/K), h (W/K), q (J/g)
        c = 1000.0 / self.th[0]
        return c, -self.th[1] * c / 10.0, -self.th[2] * c / 10.0

    def observe(self, t, T, p_w, room_c, g):
        """One control step: time, coolant temperature, server heat (W), room temperature, salt (g/s).
        The inputs describe the step that just ended, so the very first call (no step yet) is skipped."""
        if not self.started:
            self.started = True
            return
        self.win.append((t, T, p_w / 1000.0, (T - room_c) / 10.0, g / 10.0))
        if t - self.win[0][0] < self.WINDOW:
            return
        n = len(self.win)
        tm = sum(w[0] for w in self.win) / n
        Tm = sum(w[1] for w in self.win) / n
        sxx = sum((w[0] - tm) ** 2 for w in self.win)
        slope = sum((w[0] - tm) * (w[1] - Tm) for w in self.win) / sxx
        self.update([sum(w[k] for w in self.win) / n for k in (2, 3, 4)], slope)
        self.win = []

    def update(self, phi, y):
        P, th = self.P, self.th
        Pphi = [sum(P[i][j] * phi[j] for j in range(3)) for i in range(3)]
        s = sum(phi[i] * Pphi[i] for i in range(3)) + self.R
        k = [v / s for v in Pphi]
        err = y - sum(phi[i] * th[i] for i in range(3))
        new = [th[i] + k[i] * err for i in range(3)]
        if new[0] <= 0 or new[1] > 0 or new[2] > 0:
            return                     # physically impossible (negative heat capacity etc.): ignore the sample
        c, h, q = 1000.0 / new[0], -new[1] * 100.0 / new[0], -new[2] * 100.0 / new[0]
        if not (2e3 <= c <= 2e6 and h <= 500 and q <= 2000):
            return
        self.th = new
        self.P = [[P[i][j] - k[i] * Pphi[j] for j in range(3)] for i in range(3)]
        for i in range(3):             # a little process noise keeps it adapting (pump wear, water topped up)
            self.P[i][i] += (0.002 * self.th[i]) ** 2
        self.updates += 1

    def info(self):
        """Learned values with their uncertainty (one standard deviation, from the filter's covariance)."""
        c, h, q = self.params
        dc, dh, _ = self.design
        t0, t1, t2 = self.th
        P = self.P

        def sd(g0, gi, i):             # delta method for f(theta0, theta_i)
            return math.sqrt(max(0.0, g0 * g0 * P[0][0] + gi * gi * P[i][i] + 2 * g0 * gi * P[0][i]))
        water_sd = 1000.0 * math.sqrt(max(0.0, P[0][0])) / t0 ** 2 / CP_WATER
        return {"water_kg": round(c / CP_WATER, 1), "water_sd": round(water_sd, 1),
                "leak_w_per_k": round(h, 1), "leak_sd": round(sd(100 * t1 / t0 ** 2, -100 / t0, 1), 1),
                "j_per_g": round(q), "j_sd": round(sd(100 * t2 / t0 ** 2, -100 / t0, 2)),
                "design_water_kg": round(dc / CP_WATER, 1), "design_leak": round(dh, 1), "updates": self.updates}


class Controller:
    """The brain. Hardware-agnostic: an observation dict goes in, a command dict comes out."""
    PLAN_STEP, PLAN_STEPS, PLAN_EVERY = 10.0, 30, 5.0   # MPC: 10 s steps, 5 min horizon, re-plan every 5 s

    def __init__(self, cfg, mode="efficiency"):
        self.cfg, self.mode = cfg, mode
        self.reset()

    def reset(self):
        self.slope = self.last_dose = self.clock = self.throttle_until = self.plan = self.next_plan = 0.0
        self.shutdown_until = self.chip_slope = 0.0
        self.prev = self.module = self.last_module = self.model = self.chip_prev = None
        self.eff = 1.0                      # learned dosing-pump effectiveness
        self.regen = self.cold = self.hide = False
        self.resid, self.drifting, self.sensor_note = {}, set(), ""
        self.state, self.reason = State.NORMAL, "ok"

    # ---------- 1. sense ----------
    def fuse(self, readings, dt, notes):
        """Median vote -> (temperature, health). health: 0 ok, 1 degraded, 2 no usable data.
        Each sensor is also compared with the median of all of them over about 30 s, so one that
        slowly drifts is caught and dropped long before it is far enough off to be outvoted."""
        c = self.cfg
        valid = {i: r for i, r in enumerate(readings) if r is not None and -40 < r < 150}
        if len(valid) >= 3:
            vals = sorted(valid.values())
            med, a = vals[len(vals) // 2], min(1.0, dt / 30.0)     # residuals averaged over about 30 s
            for i, r in valid.items():
                if abs(r - med) <= c.max_sensor_gap:     # gross faults are left to the vote below
                    self.resid[i] = (1 - a) * self.resid.get(i, 0.0) + a * (r - med)
                res = self.resid.get(i, 0.0)
                if i not in self.drifting and abs(res) > c.drift_limit:
                    self.drifting.add(i)
                    notes.append(f"sensor S{i + 1} drifting ({res:+.1f} °C): dropped from the vote, recalibrate it")
                elif i in self.drifting and abs(res) < c.drift_limit / 3:
                    self.drifting.discard(i)
                    notes.append(f"sensor S{i + 1} agrees again: back in the vote")
        good = sorted(r for i, r in valid.items() if i not in self.drifting)
        self.sensor_note = "sensor drifting: dropped from the vote" if self.drifting & set(valid) else ""
        if len(good) < 2:
            return (good[-1] if good else c.t_shutdown + 1), 2   # can't vote: assume the worst
        health = 1 if len(good) < len(readings) else 0
        if good[-1] - good[0] > c.max_sensor_gap:
            med = good[len(good) // 2]
            kept = [g for g in good if abs(g - med) <= c.max_sensor_gap]
            if len(kept) < 2:
                return good[-1], 2
            good, health = kept, 1
            self.sensor_note = self.sensor_note or "sensor fault: outvoted, maintenance needed"
        if health and not self.sensor_note:
            self.sensor_note = "sensor missing: running on the others"
        return sum(good) / len(good), health

    def setpoint(self, o):
        mode = "efficiency" if self.mode == "stealth" and o.get("window") else self.mode
        return SETPOINT[o["kind"]][mode]

    def reserve(self, o, T, sp):
        """How much cooling the sink has left -> (saturation, low, exhausted)."""
        c, kind = self.cfg, o["kind"]
        if kind == "salt":
            # Saturation is measured against the coldest temperature we plan to run at:
            # these salts are less soluble when cold, so cooling a nearly saturated tank would grow crystals.
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
    def supervise(self, T, hot, health, low, out, chip=None, chip_full=None, chip_now=None):
        """hot = coolant temperature, now or predicted; chip = hottest chip, now or predicted;
        chip_full = where the chips would settle if the full requested load were allowed back;
        chip_now = the hottest chip as measured right now."""
        c, prev = self.cfg, self.state
        chip = -1e9 if chip is None else chip
        chip_now = chip if chip_now is None else chip_now
        chip_full = chip if chip_full is None else chip_full
        # A projected chip temperature can shed load. It can only shut the servers down once load is
        # already shed and the projection still says too hot; a real reading over the limit always can.
        chips_off = chip_now >= c.chip_shutdown or (prev == State.THROTTLE and chip >= c.chip_shutdown)
        if hot >= c.t_shutdown or chips_off or (health == 2 and T >= c.t_throttle):
            return State.SHUTDOWN, ("no trustworthy temperature data" if health == 2
                                    else f"chips at {max(chip, chip_now):.0f} °C reached or predicted" if chips_off
                                    else f"{hot:.0f} °C reached or predicted")
        if prev == State.SHUTDOWN and (T > c.restart_below or chip >= c.chip_warn or self.clock < self.shutdown_until):
            return State.SHUTDOWN, self.reason                  # latched: off >= 2 min, and until everything cooled
        if prev == State.SHUTDOWN and out:                      # no point restarting with nothing to cool
            return State.SHUTDOWN, "cooling sink exhausted: refill or regenerate to restart"
        if out:
            return State.THROTTLE, "cooling sink exhausted: graceful power-down"
        if prev == State.SHUTDOWN:                              # come back gently: half load, held >= 5 min
            return State.THROTTLE, "restarting at reduced load"
        # Once throttled, hold for >= 5 min, and until coolant and chips are under their warn limits,
        # the chips would stay under it at full load, AND the coolant is actually cooling.
        held = prev == State.THROTTLE and (hot >= c.t_warn or chip >= c.chip_warn or chip_full >= c.chip_warn
                                           or self.slope > 0.001 or self.clock < self.throttle_until)
        if hot >= c.t_throttle or chip >= c.chip_throttle or held:
            return State.THROTTLE, (("chips too hot: shedding load" if chip_now >= c.chip_throttle
                                     else "chips heating fast: shedding load early") if chip >= c.chip_throttle
                                    else "too hot: shedding load" if hot >= c.t_throttle or prev != State.THROTTLE
                                    else "chips would overheat at full load: holding the throttle"
                                    if chip_full >= c.chip_warn and self.clock >= self.throttle_until
                                    else self.reason)
        if low and (hot >= c.t_warn or chip >= c.chip_warn):
            return State.THROTTLE, "cooling reserve low while hot"
        if hot >= c.t_warn or chip >= c.chip_warn:
            return State.WARNING, ("chips approaching their limit" if chip >= c.chip_warn
                                   else "approaching limit: maximum cooling")
        if health:
            return State.WARNING, self.sensor_note
        if self.cold:
            return State.WARNING, "freeze risk: guard active"
        if low:
            return State.WARNING, "cooling reserve low"
        return State.NORMAL, "ok"

    # ---------- 3. act ----------
    def mpc(self, o, T, sp):
        """Model-predictive control: simulate the learned room model 5 minutes ahead for each candidate
        dosing rate, and pick the one that keeps the coolant in its band for the least salt.
        A scheduled job in the forecast makes it pre-cool on its own."""
        c, m = self.cfg, self.model
        mode = "efficiency" if self.mode == "stealth" and o.get("window") else self.mode
        hi, lo, w_lo, w_salt = {"efficiency": (sp, -1e9, 0.0, 1.0),
                                "performance": (sp + 0.3, sp - 2.0, 20.0, 0.05),
                                "stealth": (sp + 0.5, sp - 0.5, 500.0, 0.2)}[mode]
        room, p_now, p_next, t_next = o["ambient_c"], o["load_w"], o["forecast_w"], o.get("forecast_in_s")
        k0, k1, k2 = m.th[0] / 1000.0, m.th[1] / 10.0, m.th[2] / 10.0
        h, n, ceiling = self.PLAN_STEP, self.PLAN_STEPS, c.t_warn - 2.0

        def cost(g):
            Tk, J = T, 0.0
            for k in range(1, n + 1):
                p = p_next if t_next is not None and k * h >= t_next else p_now
                Tk += h * (k0 * p + k1 * (Tk - room) + k2 * g)
                J += 500.0 * max(0.0, Tk - hi) ** 2 + w_lo * max(0.0, lo - Tk) ** 2 + 1e5 * max(0.0, Tk - ceiling) ** 2
            return J + w_salt * g * h * n

        top = c.max_dose * self.eff                     # what the pump can actually deliver
        best = min((top * i / 20 for i in range(21)), key=cost)
        return min((min(top, max(0.0, best + top * j / 80)) for j in range(-4, 5)), key=cost)

    def act_salt(self, o, T, sp, sat, cooling_needed, out, health, notes):
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
        if self.clock >= self.next_plan:
            self.plan = self.mpc(o, T, sp)  # grams per second actually dissolved
            self.next_plan = self.clock + self.PLAN_EVERY
        dose = c.max_dose if cooling_needed else self.plan / self.eff
        self.last_dose = cmd["dose"] = min(max(dose, 0.0), c.max_dose)
        return cmd

    def usable_modules(self, melt):
        """The module in use until it is 95% melted; others only once re-solidified below 80%."""
        return [i for i, x in enumerate(melt) if x < 0.8 or (i == self.module and x < 0.95)]

    def act_pcm(self, o, T, Tp, health, notes):
        m, eng = o["melt"], (None if health == 2 else self.module)   # blind: servers off, park the modules
        if eng is not None and (m[eng] >= 0.95 or T < MELT):         # below MELT it would hand heat back
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

    def act_orbit(self, o, T, Tp, sp, cooling_needed, health, notes):
        c = self.cfg
        theta, dep = o["theta"], o["dep"]
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
        if Tp > sp or self.mode == "performance":
            theta = 90.0                    # edge-on to the sun (efficiency only slews when needed)
        if self.mode == "performance":      # as cold as possible, but no colder than the setpoint
            dep = min(1.0, max(0.05, 1.0 - 0.25 * (sp - Tp)))
        elif self.mode == "stealth" and not o.get("window"):
            dep = 0.05 if Tp < sp else min(1.0, 0.05 + 0.25 * (Tp - sp + 1))
        else:                               # efficiency: float in a wide band
            dep = min(1.0, max(0.05, 1.0 - 0.1 * (sp - Tp)))
        if cooling_needed:
            dep = 1.0
        if hide:
            dep = 0.05
        return {"theta": theta, "dep": dep, "heater": 0.0, "survival": False}

    def step(self, o, dt):
        c, kind, notes = self.cfg, o["kind"], []
        T, health = self.fuse(o["temps"], dt, notes)
        if health == 2:                     # blind: freeze the predictor, restart it when data returns
            self.prev, self.slope = None, 0.0
        elif self.prev is not None:
            self.slope = 0.8 * self.slope + 0.2 * (T - self.prev) / dt
        if health < 2:
            self.prev = T
        Tp = T + self.slope * c.lookahead_s * (10 if kind == "orbit" else 1)
        hot, sp = max(T, Tp), self.setpoint(o)
        chip = o.get("chip_c")
        chip_hot = chip_full = None
        if chip is None:
            self.chip_prev, self.chip_slope = None, 0.0
        else:
            if self.chip_prev is not None:
                self.chip_slope = 0.5 * self.chip_slope + 0.5 * (chip - self.chip_prev) / dt
            self.chip_prev = chip
            # where the chips are heading: their own trend a few seconds out, plus any rise in the coolant
            chip_hot = chip + max(0.0, self.chip_slope) * CHIP_LOOKAHEAD + max(0.0, Tp - T)
            # where they would settle at the full requested load (chip rise scales with power)
            load_w, demand_w = o.get("load_w", 0.0), o.get("demand_w", 0.0)
            chip_full = T + (chip - T) * demand_w / load_w if 1.0 < load_w < demand_w else chip
        sat, low, out = self.reserve(o, T, sp)
        self.clock += dt
        if kind == "salt" and health < 2:   # learn how this room actually behaves
            if self.model is None:
                self.model = ThermalModel(DESIGN_GUESS["water_kg"] * CP_WATER, DESIGN_GUESS["leak_w_per_k"],
                                          o["heat_j_per_g"])
            self.model.observe(self.clock, T, o["load_w"], o["ambient_c"], o["delivered"])
        if kind == "orbit" and health < 2:
            low_t = min(T, Tp)
            cold = low_t < c.t_freeze_warn or (self.cold and low_t < c.t_freeze_warn + 8)
            if cold != self.cold:
                notes.append("freeze guard ON: retract, face the sun, heater" if cold else "freeze guard off")
            self.cold = cold
        prev = self.state
        self.state, self.reason = self.supervise(T, hot, health, low, out, chip_hot, chip_full, chip)
        if self.state == State.THROTTLE and prev != State.THROTTLE:
            self.throttle_until = self.clock + 300.0
        if self.state == State.SHUTDOWN and prev != State.SHUTDOWN:
            self.shutdown_until = self.clock + 120.0
        cmd = {"load": LOAD_CAP[self.state]}
        cooling_needed = hot >= c.t_warn or (chip_hot is not None and chip_hot >= c.chip_warn)
        model = None
        if kind == "salt":
            cmd.update(self.act_salt(o, T, sp, sat, cooling_needed, out, health, notes))
            act = cmd["dose"] / c.max_dose
            brain = (f"model-predictive control: {self.plan:.1f} g/s planned over the next 5 min"
                     if self.model else "waiting for sensor data")
            model = self.model.info() if self.model else None
        elif kind == "pcm":
            cmd.update(self.act_pcm(o, T, Tp, health, notes))
            act = sum(o["melt"]) / len(o["melt"])
            brain = "rule-based module scheduler with regeneration planning"
        else:
            cmd.update(self.act_orbit(o, T, Tp, sp, cooling_needed, health, notes))
            act = cmd["dep"]
            brain = "physics-based radiator control (Q = εσAT⁴ vs absorbed sunlight)"
        reason = self.reason
        if self.regen:
            cmd["load"] = min(cmd["load"], 0.4)
            if self.state == State.NORMAL:
                reason = "regenerating PCM: load capped at 40%"
        return cmd, {"T": round(T, 2), "T_pred": round(Tp, 2), "setpoint": round(sp, 1),
                     "state": self.state.value, "reason": reason, "health": health,
                     "eff": round(self.eff, 2), "sat": None if sat is None else round(sat, 3),
                     "chip": None if chip is None else round(chip, 1), "brain": brain, "model": model,
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
        t[1] += self.drift                            # 0 unless the "sensor drifts" scenario is on
        return t

    def drift_tick(self, dt):
        self.drift = self.drift + 0.015 * dt if self.flags.get("sensor_drift") else 0.0   # sped up for the demo

    def chip_target(self, utilization):
        rise = CHIP_RISE * (2.6 if self.flags.get("cold_plate") else 1.0)   # clogged cold plate: chips run hot
        return self.T + rise * utilization

    def chip_c(self, utilization):
        if self.chip is None:                         # first reading: start in equilibrium
            self.chip = self.chip_target(utilization)
        return self.chip + random.gauss(0, 0.05)

    def chip_tick(self, dt, utilization):             # thermal mass: the chips lag their load by ~CHIP_TAU
        target = self.chip_target(utilization)
        self.chip = target if self.chip is None else self.chip + (target - self.chip) * (1.0 - math.exp(-dt / CHIP_TAU))

    def apply(self, cmd):
        self.cmd.update(cmd)


class Ground(Sim):
    """Server room with no radiator: a 20 kg water loop that leaks a little heat to the room."""
    BASE_W, SPIKE_W, WATER_KG, LEAK_W_PER_K = 2000.0, 3500.0, 20.0, 15.0
    MODULES, MODULE_KG, MODULE_UA, REGEN_W_PER_K = 4, 8.0, 4000.0, 20.0

    def __init__(self, kind="salt", solute="ammonium_chloride", hopper_g=HOPPER_G):
        self.kind, self.solute, self.cap = kind, solute, hopper_g
        self.heat_j_per_g = SOLUTES[solute]["j_per_g"]
        self.reset()

    def reset(self):
        self.T = 30.0 if self.kind == "salt" else 48.0
        self.chip = None
        self.hopper, self.dissolved, self.tanks = self.cap, 0.0, 1
        self.total_g = self.delivered = self.leak_w = self.released_w = self.drift = 0.0
        self.melt, self.engaged = [0.0] * self.MODULES, None
        self.burst_in = self.burst_left = 0.0
        self.cmd = {"load": 1.0, "dose": 0.0, "swap": False, "module": None, "fan": True}
        self.flags = {"spike": False, "window": False, "sensor_fault": False, "sensor_drift": False,
                      "sensors_lost": False, "cold_plate": False}
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
             "forecast_w": self.BASE_W + (self.SPIKE_W if soon else 0.0),
             "forecast_in_s": self.burst_in if self.burst_in > 0 else None,
             "ambient_c": AMBIENT, "window": self.flags["window"],
             "chip_c": self.chip_c(self.load_w / (self.BASE_W + self.SPIKE_W))}
        if self.kind == "salt":
            o.update(hopper_g=self.hopper, conc_g_per_kg=self.dissolved / self.WATER_KG, solute=self.solute,
                     tanks=self.tanks, delivered=self.delivered, heat_j_per_g=self.heat_j_per_g)
        else:
            o.update(melt=list(self.melt))
        return o

    def tick(self, dt):
        self.drift_tick(dt)
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
        self.chip_tick(dt, self.load_w / (self.BASE_W + self.SPIKE_W))

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
            name = SOLUTES[self.solute]["name"]
            t = [tile("Solute dosing", f"{self.delivered:.1f} g/s", None,
                      f"commanded {self.cmd['dose']:.1f} · pump health {info['eff']:.0%}"),
                 tile("Solute hopper", f"{self.hopper / 1000:.1f} kg", self.hopper / self.cap * 100,
                      f"{name} · used {self.total_g / 1000:.1f} kg"),
                 tile("Saturation", f"{sat:.0%}", sat * 100,
                      f"reserve swap at 85% · reserve tanks {self.tanks} · pump load ×{1 + 2 * sat:.2f}"),
                 tile("Heat released to room", f"{self.released_w:.0f} W", None,
                      "thermal signature (stealth holds it near 0)")]
            m = info.get("model")
            if m:
                t.append(tile("Learned room model", f"{m['water_kg']} ± {m['water_sd']} kg water", None,
                              f"leak {m['leak_w_per_k']} ± {m['leak_sd']} W/K · {m['j_per_g']} ± {m['j_sd']} J/g · "
                              f"from {m['updates']} windows (design sheet: {m['design_water_kg']} kg, "
                              f"{m['design_leak']} W/K)"))
            return t
        avg = sum(self.melt) / self.MODULES
        mods = " · ".join(f"M{k + 1} {m:.0%}{'◀' if k == self.engaged else ''}" for k, m in enumerate(self.melt))
        fan = "fan on" if self.cmd.get("fan", True) else "insulated (stealth)"
        return [tile("PCM melted", f"{avg:.0%}", avg * 100, mods + "   ◀ = in use"),
                tile("Regeneration", "low-power" if info["regen"] else "normal", None,
                     f"idle modules: {fan}"),
                tile("Heat released to room", f"{self.released_w:.0f} W", None,
                     "loop + modules re-solidifying (thermal signature)")]

    def scenarios(self):
        s = [("spike", "⚡ Sustained load"), ("burst", "📅 Scheduled job"), ("window", "🪟 Safe window"),
             ("cold_plate", "🔥 Cold plate clogged"), ("sensor_drift", "📈 Sensor drifts"),
             ("sensor_fault", "🌡 1 sensor fails"), ("sensors_lost", "🚫 All sensors lost")]
        if self.kind == "salt":
            s += [("pump_fault", "🔧 Pump degrades"), ("refill", "⛽ Refill")]
        return s + [("clear", "✖ Clear all"), ("reset", "↺ Reset")]


class Orbit(Sim):
    """Orbital relay: heat leaves only by radiation, Q = eps*sigma*A*T^4."""
    kind = "orbit"
    BASE_W, AREA, EPS, ALPHA, SUN, WATER_KG = 500.0, 1.5, 0.85, 0.15, 1361.0, 4.0

    def __init__(self):
        self.reset()

    def reset(self):
        self.T, self.theta, self.dep, self.heater, self.slewed, self.drift = 25.0, 90.0, 1.0, 0.0, 0.0, 0.0
        self.chip = None
        self.cmd = {"load": 1.0, "theta": 90.0, "dep": 1.0, "heater": 0.0}
        self.flags = {"solar_pass": False, "adcs_fault": False, "idle": False, "window": False,
                      "sensor_fault": False, "sensor_drift": False, "sensors_lost": False}

    def exposure(self):
        return 0.05 + 0.95 * self.dep          # fully retracted = 5% still exposed behind the MLI shield

    def absorb_full(self):                     # sunlight absorbed by a fully deployed radiator
        sun = self.SUN * (3.0 if self.flags["solar_pass"] else 1.0)
        return self.ALPHA * sun * self.AREA * max(0.0, math.cos(math.radians(self.theta)))

    def emit_full(self):                       # what a fully deployed radiator would emit
        return self.EPS * SIGMA * self.AREA * (self.T + 273.15) ** 4

    @property
    def load_w(self):
        return self.BASE_W * (0.05 if self.flags["idle"] else 1.0) * self.cmd["load"]

    def read(self):
        return {"kind": "orbit", "temps": self.temps(), "load_w": self.load_w, "demand_w": self.BASE_W,
                "forecast_w": self.BASE_W, "theta": self.theta, "dep": self.dep,
                "absorb_w": self.absorb_full(), "emit_w": self.emit_full(), "chip_c": self.chip_c(self.load_w / self.BASE_W),
                "window": self.flags["window"], "adcs_ok": not self.flags["adcs_fault"]}

    def tick(self, dt):
        c = self.cmd
        self.drift_tick(dt)
        if self.flags["adcs_fault"]:
            self.theta = 20.0                  # stuck facing the sun
        else:
            turn = max(-dt, min(dt, c["theta"] - self.theta))   # slews at 1 deg/s
            self.theta += turn
            self.slewed += abs(turn)
        self.dep += max(-0.02 * dt, min(0.02 * dt, c["dep"] - self.dep))
        survival = 60.0 if c.get("survival") and self.T < 10.0 else 0.0   # hardware thermostat, no software
        self.heater = c["heater"] + survival
        q = self.load_w + self.heater + (self.absorb_full() - self.emit_full()) * self.exposure()
        self.T += q * dt / (self.WATER_KG * CP_WATER)
        self.chip_tick(dt, self.load_w / self.BASE_W)

    def tiles(self, info):
        f = self.exposure()
        stuck = " · ADCS FAULT" if self.flags["adcs_fault"] else ""
        return [tile("Radiators deployed", f"{self.dep:.0%}", self.dep * 100, "retracted = behind the MLI shield"),
                tile("Attitude to sun", f"{self.theta:.0f}°", None,
                     f"90° = edge-on · slewed {self.slewed:.0f}° total{stuck}"),
                tile("Radiated to space", f"{self.emit_full() * f:.0f} W", None,
                     f"sunlight absorbed {self.absorb_full() * f:.0f} W · heater {self.heater:.0f} W")]

    def scenarios(self):
        return [("solar_pass", "☀ Close solar pass"), ("adcs_fault", "🛰 ADCS fault"), ("idle", "❄ Servers idle"),
                ("window", "🪟 Safe window"), ("sensor_drift", "📈 Sensor drifts"), ("sensor_fault", "🌡 1 sensor fails"),
                ("sensors_lost", "🚫 All sensors lost"), ("clear", "✖ Clear all"), ("reset", "↺ Reset")]


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
                self.ctl.next_plan = 0.0           # re-plan right away for the new goal
                self.log(f"mode switched to {mode}")
            return self.snapshot(history=True)

    def log(self, msg):
        self.events.append({"t": self.t, "msg": msg})

    def step(self):
        with self.lock:
            h = self.hal
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
            elif name == "clear":               # every scenario off, without resetting the simulation
                for k in h.flags:
                    h.flags[k] = False
                if hasattr(h, "burst_in"):
                    h.burst_in = h.burst_left = 0.0
                self.log("all scenarios cleared")
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
            self.cfg = self.ctl.cfg = new
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
