# 🧊 Autonomous Thermal Controller

**Live demo:** https://callen-cox.github.io/solute-cooling/ (runs right in your browser, nothing to install)

Cooling for small data centers that have **no room for a radiator**: an office building's server closet, a cramped site in a developing region, or a relay in orbit. The controller treats the coolant as a finite thermal sink and manages it autonomously. A deterministic **safety supervisor** sits above every optimisation mode and can always overrule it.

## What it does

| Layer | What it does |
|---|---|
| **1. Safety supervisor** (highest priority) | Median-votes 3 sensors, predicts 60 s ahead, escalates NORMAL → WARNING (52 °C) → THROTTLE (56 °C) → SHUTDOWN (60 °C = T_safe). The hardware limit is 85 °C, and the config refuses any setting with under 10 °C of margin. With no usable sensor data it assumes the worst. Running low on cooling escalates early. Running out triggers a graceful power-down. |
| **2. Optimisation mode** | **Efficiency** saves consumables and wear. **Performance** keeps the chips coolest and pre-cools before scheduled jobs. **Stealth** keeps the thermal signature minimal, storing heat until a safe window. |
| **3. Cooling sink** | **Salt** (terrestrial): endothermic NH₄Cl dissolution with feedforward from server power, saturation tracking against the coldest planned temperature, and an 85% reserve-tank swap. **PCM** (terrestrial): reusable sodium thiosulfate pentahydrate modules that melt at 48.3 °C, with module switching and scheduled low-power regeneration. **Orbital**: radiative cooling (Q = εσAT⁴), edge-on attitude, retraction behind MLI when the radiators absorb more than they emit, a freeze guard, and a survival heater when blind. |

## Run it

**Windows:** double-click `start.bat`. It checks for Python, installs the packages, starts the server and opens the dashboard.

**Any OS:**

```bash
pip install -r requirements.txt
python -m coolant.server        # open http://localhost:8000
python -m pytest                # safety and behaviour tests
```

### Put it online for free (GitHub Pages)

In the GitHub repo: **Settings → Pages → Build and deployment → Source: Deploy from a branch → Branch: `main`, folder `/ (root)` → Save**. After a minute or two the dashboard is live at `https://<username>.github.io/<repo>/`. With no server there, the page loads Python into the visitor's browser (Pyodide, WebAssembly) and runs the exact same `coolant/core.py`.

### Notes

Phones on the same Wi-Fi can open `http://<your-computer's-ip>:8000`. Docker: `docker build -t thermal . && docker run -p 8000:8000 thermal`.

On the dashboard, pick the environment, sink, mode and speed at the top, trigger faults with the scenario buttons, and open **How it works → Run guided demo** for a captioned walkthrough.

## Project layout

- `coolant/core.py`: `Config` (thresholds), `Controller` (sense → supervise → act), the `HAL` hardware interface, the `Ground` and `Orbit` simulators, and `Engine` (control loop and event log).
- `coolant/server.py`: FastAPI. `GET /api/state`, `GET /api/stream` (Server-Sent Events), `POST /api/setup` (`env`, `sink`, `mode`, `speed`), `POST /api/scenario/{name}`, `GET/POST /api/config`.
- `web/index.html`: dashboard, How it works page and guided demo. No build step. It talks to the server when there is one, and otherwise runs `core.py` in the browser.
- `index.html`: sends visitors of the GitHub Pages site to `web/`.
- `start.bat`: Windows one-click launcher.
- `tests/test_core.py`: safety and behaviour tests.

## Real hardware

Subclass `coolant.core.HAL`, then run with `COOLANT_HAL="my_hw:MyHAL" COOLANT_TICK_S=1 python -m coolant.server`.

`read()` returns a dict with `kind` (`"salt"`, `"pcm"` or `"orbit"`), `temps` (3 readings in °C, `None` for a dead sensor), `load_w`, `demand_w` and `forecast_w` (server power now, uncapped, and expected soon), and `window` (safe to release heat). Then, per kind:

- **salt**: `hopper_g`, `conc_g_per_kg` (from a conductivity probe), `solute`, `tanks`, `delivered` (g/s from a flow meter), `heat_j_per_g`
- **pcm**: `melt` (fraction melted per module)
- **orbit**: `theta` (angle to the sun), `dep` (0 to 1 deployed), `absorb_w` and `emit_w` (for a fully deployed radiator), `adcs_ok`

`apply(cmd)` receives `load` (IT power cap 0 to 1) plus `dose` and `swap` (salt), `module` and `fan` (pcm), or `theta`, `dep`, `heater` and `survival` (orbit).

## Honest limits

- The simulator constants are rough and unvalidated. The orbit uses a fixed sun direction (no ephemeris).
- Salt cooling is consumed: about 13 kg of NH₄Cl per kWh. It suits peak shaving or backup unless you regenerate the solution, and regenerating means dumping that heat somewhere.
- PCM only buffers heat. Sustained load above what the modules can shed to the room while re-solidifying will eventually force a shutdown (the tests prove it shuts down safely). Sodium thiosulfate pentahydrate also supercools, so real modules need a nucleating agent.
- **Always add an independent hardware thermal cutout.** Software must never be the only protection.
- The API has no authentication. It's a demo, so keep it on a trusted network.
- Ammonium nitrate absorbs more heat per gram but is an oxidizer, so it's deliberately left out.
