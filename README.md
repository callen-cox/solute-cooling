# \---

# title: Solute Cooling Controller

# emoji: 🧊

# sdk: docker

# app\_port: 8000

# \---

# 🧊 Solute Cooling Controller

Radiator-free cooling for small or cramped data centers. Dissolving certain salts in water **absorbs heat**
(the chemistry of an instant cold pack). A predictive controller doses solute into a coolant loop only when
needed, wrapped in layered safety margins. Includes a live dashboard and a built-in guided demo.

## Quick start

```bash
pip install -r requirements.txt
python -m coolant.server        # open http://localhost:8000 (phones on the same wifi: http://<your-ip>:8000)
pytest                          # safety tests
```

Docker: `docker build -t solute . \&\& docker run -p 8000:8000 solute`

## Layout

* `coolant/core.py` — `Config`, `Controller` (safety + dosing), `HAL` interface, `SimHAL` simulator, `Engine` loop
* `coolant/server.py` — FastAPI: `GET /api/state`, `GET /api/stream` (SSE), `POST /api/scenario/{spike|sensor\_fault|sensors\_lost|pump\_fault|refill|reset}`, `GET/POST /api/config`
* `web/index.html` — dashboard + "How it works" tab + guided demo (no build step, works offline)

## Safety design

NORMAL → WARNING (38°C, max dosing) → THROTTLE (42°C, IT load halved) → SHUTDOWN (46°C, latched until <35°C).
Hardware damage is assumed at 55°C and `Config.validate()` enforces ≥5°C between shutdown and damage.
Decisions use *predicted* temperature (60 s lookahead). Three sensors are median-voted: one liar is outvoted,
no majority or no data = fail safe. Low solute or near-saturation escalates early. A loop error cuts IT load.

## Running on real hardware

Subclass `HAL` (5 methods: read temps, hopper level, set dose pump, cap IT load, optional saturation/flow) and run:

```bash
COOLANT\_HAL="my\_hw:PiHAL" COOLANT\_TICK\_S=1 python -m coolant.server
```

```python
class PiHAL(HAL):
    def read\_temps(self): return \[probe.read\_c() for probe in PROBES]   # e.g. DS18B20
    def read\_hopper\_g(self): return load\_cell.grams()
    def set\_dose(self, g\_per\_s): pump.set\_rate(g\_per\_s)                 # auger / dosing pump
    def set\_load(self, fraction): pdu.set\_power\_cap(fraction)           # PDU / server power-cap API
```

## ⚠️ Disclaimer

Physical constants are rough and unvalidated. **Always add an independent hardware thermal cutout**: software
must never be the only protection. The API has no authentication (demo only). Ammonium nitrate is an oxidizer;
use urea or ammonium chloride near electronics. Dissolution cooling consumes solute (\~22 kg/h per 2 kW), so this
suits peak shaving / backup unless you add solution regeneration.

