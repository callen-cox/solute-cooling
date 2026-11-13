"""Autonomous thermal watchdog for a solar-orbit communication relay.

Escape to space: a relay that links habitat ships has to keep its servers at a safe temperature with as
little radiator, piping and power as possible. In space, heat can only leave as thermal radiation, so the
cooling works in tiers, with a deterministic watchdog above all of them:

  Tier 1  Radiators  A pumped water loop carries server heat to radiator panels that face away from the
                     sun and shed it as infrared light (Q = εσAT⁴), working with the attitude (flight) computer.
  Tier 2  PCM        When the radiators alone can't keep up (a traffic surge, attitude trouble), a valve routes
                     the loop through n-eicosane wax, which soaks up heat as it melts at 36.6 °C. It re-freezes
                     later on the radiators' spare capacity, ready for the next surge.
  Tier 3  Go down    If that is still not enough, the watchdog sheds load, then shuts the servers down and
                     stows the radiators behind their shield until things cool off.
  Freeze  At low load the water could freeze, so the radiators are stowed and the loop is warmed with
          the wax's stored heat, then a pass past the sun-facing side, and only then an electric heater.

Every control step:
  1. Sense      Three redundant temperature sensors are median-voted, and one that slowly drifts away
                from the others is caught and dropped from the vote. A reading the coolant can't physically
                produce (a radiation glitch) is ignored, and the safety limits are kept in three spare copies
                that out-vote any bit flip in memory. The pump's flow and the chip sensor are watched too.
  2. Supervise  The watchdog (highest priority) watches the coolant AND the hottest chip, picks
                NORMAL / WARNING / THROTTLE / SHUTDOWN and overrides everything below.
  3. Act        The mode (efficiency / performance / stealth) drives the radiators, the PCM valve and the
                freeze guard, using an energy-balance forecast of where the coolant is heading.

Every command carries a heartbeat, so a hardware watchdog can cut server power if the program ever freezes.

Relay is a lumped-capacitance simulator for the demo. Real hardware plugs in by subclassing HAL (see README).
"""
import math
import random
import threading
import time
from collections import deque
from dataclasses import asdict, dataclass, fields
from enum import Enum

CP_WATER = 4180.0       # J/(kg K)
SIGMA = 5.67e-8         # Stefan-Boltzmann constant, W/(m^2 K^4)
SUN_1AU = 1361.0        # sunlight at 1 AU, W/m^2
RADIATOR_EPS = 0.85     # radiator emissivity
PCM_NAME = "n-eicosane wax"
PCM_MELT = 36.6         # n-eicosane (C20H42) melting point, C
PCM_LATENT = 247e3      # its latent heat of fusion, J/kg
CHIP_RISE = 25.0        # the hottest chip runs this much above the coolant at full surge load
CHIP_TAU = 15.0         # s: chips have thermal mass, so they follow a change in load or cooling over ~15 s
CHIP_LOOKAHEAD = 10.0   # s: how far ahead the watchdog projects the chips' own trend
MODES = ("efficiency", "performance", "stealth")
SETPOINT = {"efficiency": 30.0, "performance": 20.0, "stealth": 45.0}
DT = 5.0                # simulated seconds per control step
SPEEDS = (1, 5, 20)
PANEL_KG_M2 = 5.0       # radiator areal density incl. structure: 2-10 kg/m^2 is typical, 5 a common base case


def equilibrium(T, heat_w, absorb_w, emit_w, exposure=1.0):
    """Coolant temperature (C) at which the radiators would shed heat_w on their own (energy balance:
    heat = exposure x (emitted - absorbed), emission scaling with T^4 in kelvin). absorb_w and emit_w
    are for fully deployed radiators at temperature T."""
    if emit_w <= 0 or exposure <= 0:
        return float("inf")
    need = heat_w / exposure + absorb_w
    return -273.15 if need <= 0 else (T + 273.15) * (need / emit_w) ** 0.25 - 273.15


def sizing(normal_kw=0.5, surge_kw=1.5, surge_min=30.0, between_h=6.0, radiator_c=30.0, panel_kg_m2=PANEL_KG_M2):
    """Radiator sized for the surge, versus a radiator sized for the average load plus wax for the surges."""
    normal_kw, surge_kw, surge_min, between_h, radiator_c, panel_kg_m2 = (
        float(v) for v in (normal_kw, surge_kw, surge_min, between_h, radiator_c, panel_kg_m2))
    if not (0 < normal_kw <= surge_kw <= 10000 and 0 < surge_min <= 10000 and 0 < between_h <= 10000
            and -50 <= radiator_c <= 150 and 0.5 <= panel_kg_m2 <= 50):
        raise ValueError("use a normal load above 0 kW and no bigger than the surge, positive times, "
                         "a radiator between -50 and 150 °C, and 0.5-50 kg/m²")
    flux = RADIATOR_EPS * SIGMA * ((radiator_c + 273.15) ** 4 - 3.0 ** 4)   # W/m^2, facing away from the sun
    r = surge_min / 60.0 / between_h
    avg_kw = (normal_kw + r * surge_kw) / (1 + r)    # the wax only re-freezes if the radiator carries the average
    wax = (surge_kw - avg_kw) * 1000 * surge_min * 60 / PCM_LATENT
    box = 0.5 * wax                                   # container and fins, assumed half the wax mass
    a_peak, a_avg = surge_kw * 1000 / flux, avg_kw * 1000 / flux
    m_peak, m_pcm = a_peak * panel_kg_m2, a_avg * panel_kg_m2 + wax + box
    rows = [{"option": "Radiator sized for the surge (no PCM)", "area": round(a_peak, 2), "mass": round(m_peak, 1),
             "notes": f"sheds the full {surge_kw:g} kW at {radiator_c:g} °C"},
            {"option": "Radiator for the average load + wax PCM", "area": round(a_avg, 2), "mass": round(m_pcm, 1),
             "notes": f"{wax:.1f} kg of {PCM_NAME} + {box:.1f} kg container; re-freezes between surges"}]
    return {"rows": rows, "flux": round(flux), "avg_kw": round(avg_kw, 3),
            "area_saved_pct": round(100 * (1 - a_avg / a_peak)), "mass_saved_kg": round(m_peak - m_pcm, 1),
            "panel_kg_m2": panel_kg_m2}


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
    t_freeze: float = 5.0          # the water must never get this cold
    t_freeze_warn: float = 10.0    # the freeze guard starts here
    restart_below: float = 46.0    # a SHUTDOWN only clears once the coolant is back under this
    t_warn: float = 52.0           # coolant: alert + maximum cooling
    t_throttle: float = 56.0       # coolant: shed IT load
    t_shutdown: float = 60.0       # coolant T_safe: clean shutdown of the servers
    t_damage: float = 85.0         # loop hardware limit (pump, seals, tubing)
    chip_warn: float = 80.0        # hottest chip: alert + maximum cooling
    chip_throttle: float = 85.0    # hottest chip: shed IT load
    chip_shutdown: float = 90.0    # hottest chip: clean shutdown
    chip_damage: float = 100.0     # chip junction limit
    max_sensor_gap: float = 3.0    # sensors further apart than this get voted on
    drift_limit: float = 0.6       # a sensor this far from the others for about 30 s is drifting
    lookahead_s: float = 300.0     # how far ahead the coolant trend is projected
    max_rate: float = 0.5          # °C/s the coolant can't physically beat: a faster jump is a glitch (radiation)
    glitch_steps: float = 3.0      # a sensor may glitch this many readings in a row before it counts as failed
    min_flow: float = 0.5          # coolant flow under this fraction of normal means the pump has stopped

    def validate(self):
        coolant = (self.t_freeze < self.t_freeze_warn < self.restart_below < self.t_warn
                   < self.t_throttle < self.t_shutdown)
        chips = self.chip_warn < self.chip_throttle < self.chip_shutdown
        margins = self.t_damage - self.t_shutdown >= 10 and self.chip_damage - self.chip_shutdown >= 10
        sensing = (0 < self.drift_limit < self.max_sensor_gap and self.max_rate > 0
                   and 0 <= self.glitch_steps <= 10 and 0 < self.min_flow < 1)
        if not (coolant and chips and margins and sensing):
            raise ValueError("need freeze < freeze_warn < restart < warn < throttle < shutdown, "
                             "chip_warn < chip_throttle < chip_shutdown, >=10 C margins before damage, "
                             "0 < drift_limit < max_sensor_gap, max_rate > 0, glitch_steps 0-10 "
                             "and 0 < min_flow < 1")
        return self


class Controller:
    """The watchdog and the brain. Hardware-agnostic: an observation dict goes in, a command dict comes out."""

    def __init__(self, cfg, mode="efficiency"):
        self.mode = mode
        self.set_config(cfg)
        self.reset()

    def set_config(self, cfg):
        """Use cfg, and keep three spare copies of it: radiation can flip bits in memory."""
        self.cfg, self.copies = cfg, [asdict(cfg) for _ in range(3)]

    def scrub(self, notes):
        """Every step, majority-vote each setting across the live config and its three copies, and repair
        whichever disagrees, so a bit flip can't quietly switch a protection off."""
        for k, v in asdict(self.cfg).items():
            votes = [v] + [copy[k] for copy in self.copies]
            good = max(votes, key=votes.count)
            if votes.count(good) < 3:           # no clear majority: fall back to the factory setting
                good = getattr(Config(), k)
            if v != good:
                setattr(self.cfg, k, good)
                notes.append(f"memory scrub: repaired a corrupted setting ({k})")
            for copy in self.copies:
                copy[k] = good

    def reset(self):
        self.slope = self.clock = self.throttle_until = self.shutdown_until = self.chip_slope = 0.0
        self.prev = self.chip_prev = None
        self.cold = self.hide = False
        self.pcm_mode, self.warming = "closed", {"wax": 0.0, "sun": 0.0, "heater": 0.0}
        self.resid, self.drifting, self.sensor_note = {}, set(), ""
        self.bad, self.glitched, self.glitch_note_t = {}, set(), -1e9   # per sensor: impossible readings in a row
        self.pump, self.no_flow_s, self.retry_at, self.no_flow = "A", 0.0, 0.0, False
        self.chip_missing_s, self.chip_lost, self.beat = 0.0, False, 0
        self.state, self.reason = State.NORMAL, "ok"

    # ---------- 1. sense ----------
    def fuse(self, readings, dt, notes):
        """Median vote -> (temperature, health). health: 0 ok, 1 degraded, 2 no usable data.
        Each sensor is also compared with the median of all of them over about 30 s, so one that
        slowly drifts is caught and dropped long before it is far enough off to be outvoted."""
        c = self.cfg
        # A reading the coolant can't produce (out of range, or further from the last trusted value than it can
        # move in dt) is a glitch, e.g. a radiation bit flip: ignore it. A sensor that keeps doing it for more
        # than glitch_steps readings in a row is treated as failed and left to the vote below.
        jump = None if self.prev is None else max(0.5, c.max_rate * dt)
        valid, glitched = {}, []
        for i, r in enumerate(readings):
            if r is None:
                continue
            in_range = -40 < r < 150
            if in_range and (jump is None or abs(r - self.prev) <= jump):
                self.bad[i], valid[i] = 0, r
                continue
            self.bad[i] = self.bad.get(i, 0) + 1
            if self.bad[i] <= c.glitch_steps:
                glitched.append(i)
            elif in_range:
                valid[i] = r
        self.glitched = set(glitched)
        if glitched and self.clock - self.glitch_note_t >= 60:
            self.glitch_note_t = self.clock
            notes.append("glitch ignored (radiation?): " + ", ".join(f"S{i + 1} read {readings[i]:.0f} °C"
                                                                       for i in glitched))
        if glitched and not valid and self.prev is not None:
            # every working sensor glitched at once: hold the last trusted value (still flag a dead sensor)
            down = len(readings) > len(glitched)
            if down:
                self.sensor_note = "sensor missing: running on the others"
            return self.prev, (1 if down else 0)
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
        missing = len(readings) - len(glitched) > len(good)     # a sensor down for longer than a glitch
        if len(good) < 2:
            if glitched and good:               # the others glitched this reading; this one agrees with history
                if missing and not self.sensor_note:
                    self.sensor_note = "sensor missing: running on the others"
                return good[0], (1 if missing else 0)
            self.sensor_note = "too few working sensors: assuming the worst"
            return (good[-1] if good else c.t_shutdown + 1), 2   # can't vote: assume the worst
        health = 1 if missing else 0
        if good[-1] - good[0] > c.max_sensor_gap:
            med = good[len(good) // 2]
            kept = [g for g in good if abs(g - med) <= c.max_sensor_gap]
            if len(kept) < 2:
                self.sensor_note = "sensors disagree: assuming the worst"
                return good[-1], 2
            good, health = kept, 1
            self.sensor_note = self.sensor_note or "sensor fault: outvoted, maintenance needed"
        if health and not self.sensor_note:
            self.sensor_note = "sensor missing: running on the others"
        return sum(good) / len(good), health

    def setpoint(self, o):
        return SETPOINT["efficiency" if self.mode == "stealth" and o.get("window") else self.mode]

    def forecast(self, o, T, load_w):
        """Energy balance: where the coolant would settle with the radiators fully out at load_w."""
        if self.hide:
            return float("inf")             # radiators facing the sun can't shed anything
        return equilibrium(T, load_w, max(0.0, o["absorb_w"]), o["emit_w"])

    def reserve(self, o, eq_now):
        """How much buffer the wax has left, if it is needed -> (low, out)."""
        needed = eq_now >= o["pcm_melt_c"] - 0.5
        return o["pcm_melt"] >= 0.8 and needed, o["pcm_melt"] >= 0.97 and needed

    # ---------- 2. supervise (highest priority) ----------
    def supervise(self, T, hot, health, low, out, chip=None, chip_full=None, chip_now=None, coolant_full=None,
                  no_flow=False, chip_lost=False, tripped=False):
        """hot = coolant temperature, now or projected; chip = hottest chip, now or projected;
        chip_now = the hottest chip as measured right now; chip_full / coolant_full = where the chips /
        the coolant would settle if the full requested load were allowed back."""
        c, prev = self.cfg, self.state
        chip = -1e9 if chip is None else chip
        chip_now = chip if chip_now is None else chip_now
        chip_full = chip if chip_full is None else chip_full
        coolant_full = -1e9 if coolant_full is None else coolant_full
        if tripped:
            return State.SHUTDOWN, "the hardware watchdog cut the power: restarting gently"
        if no_flow:                         # in free fall nothing moves the heat without a pump
            return State.SHUTDOWN, "no coolant flow: both pumps stopped"
        # A projection can shed load. It can only shut the servers down once load is already shed and it
        # still says too hot; a real reading over a limit always can.
        shed = prev == State.THROTTLE
        coolant_off = T >= c.t_shutdown or (shed and hot >= c.t_shutdown)
        chips_off = chip_now >= c.chip_shutdown or (shed and chip >= c.chip_shutdown)
        if coolant_off or chips_off or (health == 2 and T >= c.t_throttle):
            return State.SHUTDOWN, ("no trustworthy temperature data" if health == 2
                                    else f"chips at {max(chip, chip_now):.0f} °C reached or predicted" if chips_off
                                    else f"{hot:.0f} °C reached or predicted")
        if prev == State.SHUTDOWN and (T > c.restart_below or chip >= c.chip_warn or self.clock < self.shutdown_until):
            return State.SHUTDOWN, self.reason                  # latched: off >= 2 min, and until everything cooled
        if prev == State.SHUTDOWN and out:                      # no point restarting into the same overload
            return State.SHUTDOWN, "radiators can't carry the load and the wax is spent: waiting"
        if out:
            return State.THROTTLE, "wax spent and the radiators can't keep up: graceful power-down"
        if chip_lost:                       # can't watch the chips, so don't run them hard
            return State.THROTTLE, "chip temperature unknown: load held at half until the sensor is back"
        if prev == State.SHUTDOWN:                              # come back gently: half load, held >= 5 min
            return State.THROTTLE, "restarting at reduced load"
        # Once throttled, hold for >= 5 min, and until coolant and chips are under their warn limits,
        # both would stay under them at full load, AND the coolant is actually cooling.
        held = prev == State.THROTTLE and (hot >= c.t_warn or chip >= c.chip_warn or chip_full >= c.chip_warn
                                           or coolant_full >= c.t_warn or self.slope > 0.001
                                           or self.clock < self.throttle_until)
        if hot >= c.t_throttle or chip >= c.chip_throttle or held:
            done = self.clock >= self.throttle_until
            return State.THROTTLE, (("chips too hot: shedding load" if chip_now >= c.chip_throttle
                                     else "chips heating fast: shedding load early") if chip >= c.chip_throttle
                                    else "too hot: shedding load" if hot >= c.t_throttle or prev != State.THROTTLE
                                    else "radiators can't carry full load yet: holding the throttle"
                                    if coolant_full >= c.t_warn and done
                                    else "chips would overheat at full load: holding the throttle"
                                    if chip_full >= c.chip_warn and done
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
            return State.WARNING, "wax nearly spent: radiators can't keep up"
        if self.no_flow_s > 0:
            return State.WARNING, "coolant flow lost: switching pumps"
        if self.pump != "A":
            return State.WARNING, "running on the backup pump"
        return State.NORMAL, "ok"

    def alerts(self, o, T, health, chip, low, out, load):
        """Everything wrong right now, as the controller sees it, most serious first.
        Built only from what the controller measures and infers, so it works the same on real hardware."""
        c, found = self.cfg, []
        coolant_hot = health < 2 and T >= c.t_warn
        chips_hot = chip is not None and chip >= c.chip_warn

        def add(level, title, detail=""):
            found.append({"level": level, "title": title, "detail": detail})

        # warning reasons that get a more specific alert of their own below
        covered = {self.sensor_note, "freeze risk: guard active", "wax nearly spent: radiators can't keep up",
                   "running on the backup pump"}
        if coolant_hot:
            covered.add("approaching limit: maximum cooling")
        if chips_hot:
            covered.add("chips approaching their limit")
        if self.state == State.SHUTDOWN:
            add("critical", "Servers shut down", self.reason)
        elif self.state == State.THROTTLE:
            add("critical", f"IT load cut to {load:.0%}", self.reason)
        elif self.state == State.WARNING and self.reason not in covered:
            add("warning", self.reason[:1].upper() + self.reason[1:])   # e.g. chips approaching their limit
        temps = o["temps"]
        dead = [i for i, r in enumerate(temps) if i not in self.glitched and (r is None or not -40 < r < 150)]
        if len(dead) == len(temps):
            add("critical", "All temperature sensors lost", "flying blind: servers kept off")
        else:
            if health == 2:
                add("critical", "No trustworthy temperature reading", "too few sensors agree: assuming the worst")
            for i, r in enumerate(temps):
                name = f"Sensor S{i + 1}"
                if i in self.glitched:          # a one-off glitch, already ignored
                    continue
                if i in dead:
                    add("warning", f"{name} not responding", "running on the others")
                elif i in self.drifting:
                    add("warning", f"{name} drifting", f"{self.resid.get(i, 0.0):+.1f} °C off: dropped from the vote, "
                                                       "recalibrate it")
                elif health < 2 and abs(r - T) > c.max_sensor_gap:
                    add("warning", f"{name} failed", f"reads {r:.0f} °C: outvoted by the others")
        if chips_hot:
            add("critical" if chip >= c.chip_throttle else "warning", f"Hottest chip {chip:.0f} °C",
                f"shutdown limit {c.chip_shutdown:.0f} °C")
        if coolant_hot:
            add("critical" if T >= c.t_throttle else "warning", f"Coolant {T:.1f} °C",
                f"shutdown limit {c.t_shutdown:.0f} °C")
        if out:
            add("critical", "Wax spent and the radiators can't keep up", "shedding load until the heat fits")
        elif low:
            add("warning", "PCM wax nearly spent", f"{o['pcm_melt']:.0%} melted while the radiators can't keep up")
        if not o.get("adcs_ok", True):
            add("critical", "Attitude control failed", "radiators stuck facing the sun")
        if self.hide:
            add("warning", "Radiators stowed behind the shield", "they would absorb more sunlight than they emit")
        if self.no_flow:
            add("critical", "No coolant flow", "both pumps stopped: servers off to protect the chips")
        elif self.pump != "A":
            add("warning", "Running on backup pump B", "pump A stopped moving the water: repair it")
        if self.chip_lost:
            add("warning", "Chip temperature sensor lost", "load held at half until it's back")
        if self.cold:
            w = self.warming
            add("warning", "Freeze guard active", f"radiators stowed; warming the loop: wax {w['wax']:.0f} W, "
                                                  f"sun-side pass {w['sun']:.0f} W, heater {w['heater']:.0f} W")
        found.sort(key=lambda a: a["level"] != "critical")      # stable: critical first, then warnings
        return found

    # ---------- 3. act ----------
    def act(self, o, T, Tp, sp, cooling_needed, health, eq_now, notes):
        c = self.cfg
        melt_c, pcm_w = o["pcm_melt_c"], o["pcm_w"]
        cmd = {"theta": 180.0, "dep": o["dep"], "pcm": 0.0, "sun": 0.0, "heater": 0.0, "survival": False}
        self.warming = {"wax": 0.0, "sun": 0.0, "heater": 0.0}
        if health == 2:
            # Blind with the servers off: stow the radiators to slow every temperature change, hold attitude,
            # close the valves, and hand freeze protection to the survival heater's hardware thermostat.
            self.pcm_mode = "closed"
            return {**cmd, "theta": o["theta"], "dep": 0.0, "survival": True}
        net = o["emit_w"] - o["absorb_w"]   # what fully deployed radiators would shed right now
        hide = net < -0.05 * o["emit_w"] or (self.hide and net < 0.05 * o["emit_w"])
        if hide != self.hide:
            notes.append("radiators absorb more than they emit: stowing them behind the shield" if hide
                         else "radiators emit more than they absorb: redeploying")
        self.hide = hide
        if self.cold:
            # Freeze guard: stow the radiators, then warm the loop with the cheapest heat first: heat stored
            # in the wax (which re-freezes it for the next surge), the sun-side pass, and only then the heater.
            want = min(400.0, max(0.0, 80.0 * (c.t_freeze_warn + 3.0 - T)))
            wax = min(want, -pcm_w) if pcm_w < 0 else 0.0
            sun = min(want - wax, o["sun_w"])
            heater = min(300.0, want - wax - sun)
            self.warming = {"wax": wax, "sun": sun, "heater": heater}
            self.pcm_mode = "warming" if wax > 0 else "closed"
            return {**cmd, "dep": 0.0, "pcm": wax / -pcm_w if wax > 0 else 0.0,
                    "sun": sun / o["sun_w"] if sun > 0 else 0.0, "heater": heater}
        if self.mode == "performance":      # as cold as possible, but no colder than the setpoint
            dep = min(1.0, max(0.0, 1.0 - 0.25 * (sp - Tp)))
        elif self.mode == "stealth" and not o.get("window"):
            dep = 0.0 if Tp < sp else min(1.0, 0.25 * (Tp - sp + 1))
        else:                               # efficiency: float in a wide band
            dep = min(1.0, max(0.0, 1.0 - 0.1 * (sp - Tp)))
        if cooling_needed:
            dep = 1.0
        if hide:
            dep = 0.0
        # Tier 2: the wax. Absorb when the radiators can't hold the coolant below its melting point (or, in
        # stealth, to keep heat out of sight); re-freeze it using only the radiators' spare capacity.
        stealth_store = self.mode == "stealth" and not o.get("window")
        was, pcm, self.pcm_mode = self.pcm_mode, 0.0, "closed"
        if pcm_w > 0 and (eq_now >= melt_c - 0.5 or cooling_needed):
            pcm, self.pcm_mode = 1.0, "absorbing"
        elif pcm_w > 0 and stealth_store:
            pcm, self.pcm_mode = 1.0, "storing"
        elif pcm_w < 0 and T < melt_c - (0.5 if was == "re-freezing" else 1.0) and not stealth_store and not hide:
            # Re-freeze with what the radiators can spare at the mode's target: performance stays at its
            # setpoint, the others may run a little warm (just under the melting point) to re-freeze faster.
            target = sp if self.mode == "performance" else max(sp, melt_c - 3.0)
            emit_at = o["emit_w"] * ((target + 273.15) / (T + 273.15)) ** 4
            spare = max(0.0, emit_at - o["absorb_w"] - o["load_w"])
            want = min(1.0, spare / -pcm_w)
            if want > (0.005 if was == "re-freezing" else 0.02):    # hysteresis: no flicker near zero
                pcm, self.pcm_mode = want, "re-freezing"
        return {**cmd, "dep": dep, "pcm": pcm}

    def step(self, o, dt):
        notes = []
        self.scrub(notes)                   # repair any corrupted limit before it is used
        c = self.cfg
        T, health = self.fuse(o["temps"], dt, notes)
        if health == 2:                     # blind: freeze the predictor, restart it when data returns
            self.prev, self.slope = None, 0.0
        elif self.prev is not None:
            a = min(1.0, dt / 100.0)        # trend averaged over about 100 s
            self.slope = (1 - a) * self.slope + a * (T - self.prev) / dt
        if health < 2:
            self.prev = T
        Tp = T + self.slope * c.lookahead_s
        hot, sp = max(T, Tp), self.setpoint(o)
        chip = o.get("chip_c")
        if chip is not None and not -40 < chip < 150:
            chip = None                     # an impossible reading: the chip sensor is broken
        # Missing for 15 s means the chips can't be watched (a HAL without the chip_c key has no chip sensor).
        self.chip_missing_s = self.chip_missing_s + dt if chip is None and "chip_c" in o else 0.0
        lost = self.chip_missing_s >= 15.0
        if lost != self.chip_lost:
            notes.append("chip temperature sensor lost: load held at half until it's back" if lost
                         else "chip temperature sensor back")
        self.chip_lost = lost
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
        eq_now = self.forecast(o, T, o["load_w"])
        eq_full = self.forecast(o, T, o.get("demand_w", o["load_w"]))
        low, out = self.reserve(o, eq_now)
        self.clock += dt
        if health < 2:
            low_t = min(T, Tp)
            cold = low_t < c.t_freeze_warn or (self.cold and low_t < c.t_freeze_warn + 8)
            if cold != self.cold:
                notes.append("freeze guard ON: radiators stowed, loop warmed" if cold else "freeze guard off")
            self.cold = cold
        # The pump: in free fall the water only moves while a pump drives it. No flow: switch to the other pump
        # at once, then keep trying the other one every minute. A HAL without a flow meter leaves "flow" out.
        flow = o.get("flow")
        if flow is None or flow >= c.min_flow:
            self.no_flow_s, self.retry_at = 0.0, 0.0
        else:
            self.no_flow_s += dt
            if self.no_flow_s >= self.retry_at:
                other = "B" if self.pump == "A" else "A"
                notes.append(f"no coolant flow on pump {self.pump}: switching to pump {other}")
                self.pump, self.retry_at = other, self.no_flow_s + 60.0
        self.no_flow = self.no_flow_s > 10.0    # the other pump has had 10 s to get the water moving
        prev = self.state
        coolant_full = eq_full if o["pcm_melt"] >= 0.9 else None    # with wax left, full load can resume
        self.state, self.reason = self.supervise(T, hot, health, low, out, chip_hot, chip_full, chip, coolant_full,
                                                 self.no_flow, self.chip_lost, bool(o.get("watchdog_tripped")))
        if self.state == State.THROTTLE and prev != State.THROTTLE:
            self.throttle_until = self.clock + 300.0
        if self.state == State.SHUTDOWN and prev != State.SHUTDOWN:
            self.shutdown_until = self.clock + 120.0
        self.beat += 1                      # the heartbeat: hardware cuts server power if it stops changing
        cmd = {"load": LOAD_CAP[self.state], "pump": self.pump, "heartbeat": self.beat}
        cooling_needed = hot >= c.t_warn or (chip_hot is not None and chip_hot >= c.chip_warn)
        before = self.pcm_mode
        cmd.update(self.act(o, T, Tp, sp, cooling_needed, health, eq_now, notes))
        if self.pcm_mode != before and self.pcm_mode != "closed":
            notes.append({"absorbing": "radiators can't keep up: wax PCM absorbing the excess",
                          "storing": "stealth: storing heat in the wax instead of radiating it",
                          "re-freezing": "spare radiator capacity: re-freezing the wax",
                          "warming": "freeze guard: warming the loop with the wax's stored heat"}[self.pcm_mode])
        if health == 2:
            brain = "no trustworthy reading: servers off, radiators stowed, survival thermostat armed"
        elif self.cold:
            w = self.warming
            brain = (f"freeze guard: wax {w['wax']:.0f} W · sun-side pass {w['sun']:.0f} W · "
                     f"heater {w['heater']:.0f} W keep the loop liquid")
        else:
            where = ("radiators facing the sun can't shed anything" if self.hide
                     else f"radiators alone would settle at {eq_now:.0f} °C")
            then = {"absorbing": ": wax absorbing the excess", "storing": ": stealth, storing heat in the wax",
                    "re-freezing": ": spare capacity re-freezing the wax"}
            brain = f"energy balance at this load: {where}{then.get(self.pcm_mode, '')}"
        return cmd, {"T": round(T, 2), "T_pred": round(Tp, 2), "setpoint": round(sp, 1),
                     "state": self.state.value, "reason": self.reason, "health": health,
                     "chip": None if chip is None else round(chip, 1), "brain": brain, "mode": self.mode,
                     "act": round(o["pcm_melt"], 3), "pcm": self.pcm_mode, "pcm_melt_c": o["pcm_melt_c"],
                     "eq": None if math.isinf(eq_now) else round(eq_now, 1), "notes": notes,
                     "alerts": self.alerts(o, T, health, chip, low, out, cmd["load"])}


class HAL:
    """Hardware contract. read() returns sensors, apply(cmd) drives actuators (keys in README)."""
    kind = "relay"
    flags = {}

    def read(self): raise NotImplementedError
    def apply(self, cmd): raise NotImplementedError
    def tick(self, dt): pass                          # simulators advance physics here
    def reset(self): pass
    def tiles(self, info): return []
    def scenarios(self): return [("reset", "↺ Reset")]


class Relay(HAL):
    """Solar-orbit communication relay: servers on a 4 kg water loop driven by pump A (pump B is the backup),
    radiator panels that face away from the sun (and can be stowed behind a shield), a wax PCM bank, a sun-side
    pass, a heater, and a hardware watchdog that cuts server power if the heartbeat stops.
    The only way heat leaves is thermal radiation. Lumped-capacitance model, constants rough."""
    kind = "relay"
    BASE_W, SURGE = 500.0, 3.0                          # server heat, and the traffic-surge multiplier
    AREA, EPS, ALPHA, WATER_KG = 1.5, RADIATOR_EPS, 0.15, 4.0
    SHIELD_LEAK = 0.15                                  # stowed behind the shield, a radiator still sheds 15%
    PCM_KG, PCM_UA = 6.0, 300.0                         # wax mass, and heat transfer with the valve open (W/K)
    SUN_PASS_M2, SUN_PASS_ALPHA = 0.25, 0.9             # sun-facing absorber the loop can be routed past
    HEATER_MAX, SURVIVAL_W = 300.0, 150.0
    WATCHDOG_S = 30.0                                   # hardware watchdog: no heartbeat this long cuts server power
    SEU_RATE = 0.05                                     # radiation storm: chance a reading is corrupted
    FAR_AU = 1.52                                       # "out near Mars": sunlight falls with distance squared

    def __init__(self):
        self.reset()

    def reset(self):
        self.T, self.theta, self.dep, self.melt, self.drift, self.chip = 28.0, 180.0, 0.8, 0.0, 0.0, None
        self.heater = self.sun_in = self.pcm_flow = self.slewed = self.radiated = self.absorbed = 0.0
        self.since_beat, self.last_beat, self.watchdog_tripped = 0.0, None, False
        self.cmd = {"load": 1.0, "theta": 180.0, "dep": 0.8, "pcm": 0.0, "sun": 0.0, "heater": 0.0, "survival": False,
                    "pump": "A"}
        self.flags = {"spike": False, "solar_pass": False, "far": False, "adcs_fault": False, "idle": False,
                      "cold_plate": False, "window": False, "pump_fail": False, "backup_fail": False,
                      "radiation": False, "hang": False, "sensor_fault": False, "sensor_drift": False,
                      "chip_sensor_lost": False, "sensors_lost": False}

    # ---------- physics ----------
    def sunlight(self):                        # a close pass: ~0.58 AU from the sun; far: out near Mars
        return SUN_1AU * (3.0 if self.flags["solar_pass"] else 1 / self.FAR_AU ** 2 if self.flags["far"] else 1.0)

    def flow(self):
        """Coolant flow as a fraction of normal. Only the selected pump moves the water."""
        failed = self.flags["pump_fail"] if self.cmd.get("pump", "A") == "A" else self.flags["backup_fail"]
        return 0.0 if failed else 1.0

    def exposure(self):
        return self.SHIELD_LEAK + (1.0 - self.SHIELD_LEAK) * self.dep

    def absorb_full(self):                     # sunlight absorbed by fully deployed radiators
        return self.ALPHA * self.sunlight() * self.AREA * max(0.0, math.cos(math.radians(self.theta)))

    def emit_full(self):                       # what fully deployed radiators would emit
        return self.EPS * SIGMA * self.AREA * (self.T + 273.15) ** 4

    def sun_pass_w(self):                      # heat the sun-side pass adds with its valve fully open
        return self.SUN_PASS_ALPHA * self.sunlight() * self.SUN_PASS_M2

    def pcm_w(self):
        """Heat the wax would take from the loop with its valve fully open (negative: heat it gives back)."""
        q = self.PCM_UA * (self.T - PCM_MELT)
        return 0.0 if (q > 0 and self.melt >= 1.0) or (q < 0 and self.melt <= 0.0) else q

    def demand_w(self):
        return self.BASE_W * (self.SURGE if self.flags["spike"] else 0.05 if self.flags["idle"] else 1.0)

    @property
    def load_w(self):
        return self.demand_w() * self.cmd["load"]

    def utilization(self):
        return self.load_w / (self.BASE_W * self.SURGE)

    # ---------- sensors ----------
    def temps(self):
        if self.flags["sensors_lost"]:
            return [None, None, None]
        t = [self.T + random.gauss(0, 0.02) for _ in range(3)]
        if self.flags["sensor_fault"]:
            t[0] = 99.0
        t[1] += self.drift                            # 0 unless the "sensor drifts" scenario is on
        if self.flags["radiation"]:                   # single-event upsets corrupt the odd reading
            t = [random.uniform(-60, 200) if random.random() < self.SEU_RATE else r for r in t]
        return t

    def chip_target(self, utilization):
        rise = CHIP_RISE * (2.6 if self.flags["cold_plate"] else 1.0)   # clogged cold plate: chips run hot
        rise *= 1.0 if self.flow() > 0 else 4.0                         # no flow: the cold plates are starved
        return self.T + rise * utilization

    def chip_c(self):
        if self.chip is None:                         # first reading: start in equilibrium
            self.chip = self.chip_target(self.utilization())
        if self.flags["chip_sensor_lost"]:
            return None                               # the servers' BMC stops reporting it
        return self.chip + random.gauss(0, 0.05)

    def read(self):
        return {"kind": "relay", "temps": self.temps(), "load_w": self.load_w, "demand_w": self.demand_w(),
                "chip_c": self.chip_c(), "theta": self.theta, "dep": self.dep, "absorb_w": self.absorb_full(),
                "emit_w": self.emit_full(), "adcs_ok": not self.flags["adcs_fault"], "window": self.flags["window"],
                "pcm_melt": self.melt, "pcm_melt_c": PCM_MELT, "pcm_w": self.pcm_w(), "sun_w": self.sun_pass_w(),
                "flow": self.flow(), "pump": self.cmd.get("pump", "A"), "watchdog_tripped": self.watchdog_tripped}

    def apply(self, cmd):
        if "heartbeat" in cmd and cmd["heartbeat"] != self.last_beat:   # the program is alive: re-arm the watchdog
            self.last_beat, self.since_beat, self.watchdog_tripped = cmd["heartbeat"], 0.0, False
        self.cmd.update(cmd)

    def tick(self, dt):
        c = self.cmd
        self.since_beat += dt
        if self.since_beat > self.WATCHDOG_S:   # hardware watchdog: no heartbeat, so cut server power
            self.watchdog_tripped, c["load"] = True, 0.0
        self.drift = self.drift + 0.015 * dt if self.flags["sensor_drift"] else 0.0   # sped up for the demo
        if self.flags["adcs_fault"]:
            self.theta = 20.0                  # stuck facing the sun
        else:
            turn = max(-dt, min(dt, c["theta"] - self.theta))   # slews at 1 deg/s
            self.theta += turn
            self.slewed += abs(turn)
        self.dep += max(-0.02 * dt, min(0.02 * dt, c["dep"] - self.dep))
        survival = self.SURVIVAL_W if c.get("survival") and self.T < 10.0 else 0.0   # hardware thermostat
        self.heater = min(self.HEATER_MAX, max(0.0, c.get("heater", 0.0))) + survival
        moving = self.flow() > 0                # still water: heat no longer reaches the radiators, wax or sun pass
        self.sun_in = max(0.0, min(1.0, c.get("sun", 0.0))) * self.sun_pass_w() if moving else 0.0
        cap = self.PCM_KG * PCM_LATENT
        want = max(0.0, min(1.0, c.get("pcm", 0.0))) * self.pcm_w() if moving else 0.0
        new = min(1.0, max(0.0, self.melt + want * dt / cap))
        self.pcm_flow, self.melt = (new - self.melt) * cap / dt, new     # positive: the wax took heat
        self.radiated, self.absorbed = ((self.emit_full() * self.exposure(), self.absorb_full() * self.exposure())
                                        if moving else (0.0, 0.0))
        q = self.load_w + self.heater + self.sun_in + self.absorbed - self.radiated - self.pcm_flow
        # with the water still, only the water at the cold plates takes the servers' heat, so it warms 4x faster
        self.T += q * dt / (self.WATER_KG * (1.0 if moving else 0.25) * CP_WATER)
        target = self.chip_target(self.utilization())   # thermal mass: chips lag their load by ~CHIP_TAU
        self.chip = target if self.chip is None else self.chip + (target - self.chip) * (1 - math.exp(-dt / CHIP_TAU))

    # ---------- dashboard ----------
    def tiles(self, info):
        stuck = " · ADCS FAULT" if self.flags["adcs_fault"] else ""
        pcm = {"absorbing": f"absorbing {self.pcm_flow:.0f} W", "storing": f"storing {self.pcm_flow:.0f} W out of sight",
               "re-freezing": f"re-freezing, giving {-self.pcm_flow:.0f} W",
               "warming": f"warming the loop with {-self.pcm_flow:.0f} W"}.get(info["pcm"], "valve closed")
        warm = self.sun_in + max(0.0, -self.pcm_flow) * (info["pcm"] == "warming") + self.heater
        return [tile("Radiators deployed", f"{self.dep:.0%}", self.dep * 100,
                     "facing away from the sun · 0% = stowed behind the shield"),
                tile("Radiator angle to sun", f"{self.theta:.0f}°", None,
                     f"180° = facing away · slewed {self.slewed:.0f}° total{stuck}"),
                tile("Radiated to space", f"{self.radiated:.0f} W", None,
                     f"sunlight absorbed {self.absorbed:.0f} W · this is the thermal signature"),
                tile("PCM wax melted", f"{self.melt:.0%}", self.melt * 100,
                     f"{PCM_NAME}, melts at {PCM_MELT} °C · {pcm}"),
                tile("Loop warming", f"{warm:.0f} W", None,
                     f"sun-side pass {self.sun_in:.0f} W · heater {self.heater:.0f} W · only when it could freeze"),
                tile("Coolant pump", f"pump {self.cmd.get('pump', 'A')}", None,
                     f"{self.flow():.0%} flow · the other pump takes over if the water stops · hardware watchdog "
                     + ("TRIPPED: server power cut" if self.watchdog_tripped else "armed"))]

    def scenarios(self):
        return [("spike", "⚡ Traffic surge"), ("cold_plate", "🔥 Cold plate clogged"), ("idle", "💤 Servers idle"),
                ("solar_pass", "☀ Close solar pass"), ("far", "🪐 Out near Mars"),
                ("adcs_fault", "🛰 Attitude control fails"), ("window", "🪟 Safe window"),
                ("pump_fail", "💧 Pump fails"), ("backup_fail", "💧 Backup pump fails"),
                ("radiation", "☢ Radiation storm"), ("hang", "🧊 Program freezes"),
                ("sensor_drift", "📈 Sensor drifts"), ("sensor_fault", "🌡 1 sensor fails"),
                ("chip_sensor_lost", "🌡 Chip sensor fails"), ("sensors_lost", "🚫 All sensors lost"),
                ("clear", "✖ Clear all"), ("reset", "↺ Reset")]


class Engine:
    """Runs the control loop in a background thread and keeps history and an event log for the API."""

    def __init__(self, mode="efficiency", cfg=None, hal=None, tick_s=0.1):
        if mode not in MODES:
            raise ValueError(f"mode must be one of {MODES}")
        self.cfg = (cfg or Config()).validate()
        self.tick_s, self.custom, self.speed, self.mode = tick_s, hal, 1, mode
        self.lock, self._running = threading.RLock(), False
        self.hal = hal or Relay()
        self.dt = tick_s if hal else DT                 # real hardware runs in real time
        self.ctl = Controller(self.cfg, mode)
        self.t, self.prev_state, self.tripped = 0.0, None, False
        self.hist, self.events = deque(maxlen=900), deque(maxlen=40)
        self.step()

    def setup(self, mode=None, speed=None):
        with self.lock:
            if speed is not None:
                if int(speed) not in SPEEDS:
                    raise ValueError(f"speed must be one of {SPEEDS}")
                self.speed = int(speed)
            if mode is not None and mode != self.mode:
                if mode not in MODES:
                    raise ValueError(f"mode must be one of {MODES}")
                self.mode = self.ctl.mode = mode
                self.log(f"mode switched to {mode}")
            return self.snapshot(history=True)

    def log(self, msg):
        self.events.append({"t": self.t, "msg": msg})

    def step(self):
        with self.lock:
            h = self.hal
            frozen = h.flags.get("hang", False)  # demo: the control program has stopped responding
            if frozen:                           # no new commands, no heartbeat: only the physics goes on
                info = {k: v for k, v in self.last.items() if k not in ("t", "true_T", "kind", "load", "tiles")}
            else:
                if h.flags.get("radiation") and random.random() < 0.02:
                    # a radiation storm also flips bits in the controller's memory: corrupt one safety setting
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
            tripped = getattr(h, "watchdog_tripped", False)
            if tripped != self.tripped:
                self.tripped = tripped
                self.log("hardware watchdog: no heartbeat, server power cut" if tripped
                         else "heartbeat back: hardware watchdog re-armed")
            load = h.cmd["load"] if frozen else cmd["load"]
            if frozen:
                info["alerts"] = [{"level": "critical", "title": "Control program not responding",
                                   "detail": "no heartbeat: the hardware watchdog cuts server power after "
                                             f"{getattr(h, 'WATCHDOG_S', 30):.0f} s"}]
                if tripped:
                    info["alerts"].append({"level": "critical", "title": "Hardware watchdog cut server power",
                                           "detail": "it re-arms when the heartbeat comes back"})
                    info["state"], info["reason"] = "SHUTDOWN", "hardware watchdog: control program not responding"
            true_t = round(getattr(h, "T", info["T"]), 2)
            self.last = {**info, "t": self.t, "true_T": true_t, "kind": h.kind, "load": load,
                         "tiles": h.tiles(info) + [tile("IT load allowed", f'{load:.0%}', load * 100)]}
            self.hist.append({"t": self.t, "T": true_t, "act": info["act"], "state": info["state"]})

    def scenario(self, name):
        with self.lock:
            h = self.hal
            if name == "reset":
                h.reset()
                self.ctl.reset()
                self.hist.clear()
                self.events.clear()
                self.t, self.prev_state, self.tripped = 0.0, None, False
                self.step()
            elif name == "clear":               # every scenario off, without resetting the simulation
                for k in h.flags:
                    h.flags[k] = False
                self.log("all scenarios cleared")
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
            self.ctl.set_config(new)             # also refreshes the spare copies the memory scrub votes with
            self.log("config updated")
            return asdict(new)

    def snapshot(self, since=None, history=False):
        with self.lock:
            s = {**self.last, "events": list(self.events)[-12:], "flags": dict(self.hal.flags),
                 "config": asdict(self.cfg), "scenarios": self.hal.scenarios(),
                 "options": {"mode": self.mode, "speed": self.speed}}
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
            except Exception as e:              # software watchdog: any fault in the control loop cuts IT load
                try:
                    self.hal.apply({"load": 0.0})
                    self.log(f"LOOP ERROR, load cut: {e}")
                except Exception:
                    pass
            time.sleep(max(0.0, self.tick_s - (time.time() - t0)))
