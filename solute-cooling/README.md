# 🛰️ Relay Thermal Watchdog

**Live demo:** https://callen-cox.github.io/solute-cooling/ (runs right in your browser, nothing to install)

**Escape to space.** Habitat ships need fast, reliable links, so communication relays orbit the sun between them. A relay's servers make heat, and in space there is no air or water to carry it away: heat can only leave as infrared light from a radiator. This project keeps those servers at operating temperature sustainably, autonomously, safely and with as little radiator, piping and power as possible. A deterministic **watchdog** above everything makes sure the processors never fry.

## How it cools: three tiers

| Tier | What happens |
|---|---|
| **1. Radiators** | A pumped water loop carries server heat to radiator panels that face away from the sun and shed it as infrared light (Q = εσAT⁴). The software works with the flight computer, which points the solar arrays at the sun, to keep the radiators facing away. |
| **2. Wax PCM** | When the radiators alone can't keep up (a traffic surge, attitude trouble), a valve routes the loop through n-eicosane wax. It absorbs 247 kJ/kg as it melts at 36.6 °C, pinning the water just above that. Spare radiator capacity re-freezes it later, ready for the next surge. |
| **3. Go down safely** | If that still isn't enough, the watchdog sheds load to 50%, then shuts the servers down until it cools. If the sun is on the radiators, they are stowed behind their shield. |
| **Freeze guard** | At low load the water could freeze. The guard stows the radiators and warms the loop with the cheapest heat first: the wax's stored heat, then a pass past the sun-facing side, and only then an electric heater. |

Every 5 s of relay time the controller does four things:
1. **Senses:** three sensors are median-voted, with drift detection.
2. **Forecasts:** an energy balance gives the temperature the radiators alone would settle at. Above the wax's melting point, they can't keep up on their own.
3. **Supervises:** the watchdog picks the state.
4. **Acts:** it sets the radiators, the wax valve and the freeze guard.

## The watchdog

- **Two safety ladders.**
  - Coolant: WARNING 52 °C → THROTTLE 56 °C → SHUTDOWN 60 °C (T_safe), loop hardware limit 85 °C.
  - Hottest chip: WARNING 80 °C → THROTTLE 85 °C → SHUTDOWN 90 °C, damage 100 °C.
  - The config refuses any setting with under 10 °C of margin.
- **Forecasts shed load early but never shut the relay down on their own.** A shutdown needs a real reading over the limit, or a forecast that still says too hot after load was shed.
- **It watches the chips, not just the water,** so a clogged cold plate is caught before a chip cooks.
- **It catches bad sensors.** A broken one is outvoted, and one that slowly drifts is dropped before it can fool the vote.
- **It doesn't flap.** Load stays shed for at least 5 minutes, and until full power would fit again. After a shutdown it stays off at least 2 minutes and restarts at half load.
- **It fails safe.**
  - With no trustworthy reading it shuts the servers down and stows the radiators.
  - A survival heater with its own hardware thermostat keeps the water liquid.
  - If the control software itself crashes, a software watchdog cuts server power.
- **The alerts panel** lists everything wrong right now under a flashing red warning sign, most serious first.

## Modes

| Mode | Goal | What it does |
|---|---|---|
| **Efficiency** | Save power and wear | Floats the water near 28 °C, and re-freezes the wax as fast as spare radiator capacity allows |
| **Performance** | Coolest chips | Radiators out, holds about 20 °C, and re-freezes the wax without warming up |
| **Stealth** | Minimal thermal signature | Keeps the radiators stowed and stores heat in the wax, then radiates it in a safe window |

## Run it

**Windows:** double-click `start.bat`. It checks for Python, installs the packages, starts the server and opens the dashboard.

**Any OS:**

```bash
pip install -r requirements.txt
python -m coolant.server        # open http://localhost:8000
python -m pytest                # safety and behaviour tests
```

### Put it online for free (GitHub Pages)

In the GitHub repo:
1. Open **Settings → Pages → Build and deployment**.
2. Set Source to **Deploy from a branch**, Branch to `main`, folder `/ (root)`, and click **Save**.

After a minute or two the dashboard is live at `https://<username>.github.io/<repo>/`. With no server there, the page loads Python into the visitor's browser (Pyodide, WebAssembly) and runs the exact same `coolant/core.py`.

### Using the dashboard

- **Top bar:** pick the mode and speed. At 1× the relay runs 50 seconds per second.
- **Scenario buttons:** **⚡ Traffic surge**, **🔥 Cold plate clogged**, **💤 Servers idle**, **☀ Close solar pass**, **🛰 Attitude control fails**, **🪟 Safe window** and three sensor faults. **✖ Clear all** turns every fault off without resetting the simulation.
- **How it works → Run guided demo:** a captioned one-minute walkthrough of all three tiers, the chips, the freeze guard and a failure.
- **Sizing tab:** compares a radiator sized for the worst surge with a smaller radiator for the average load plus wax. Radiator panels typically weigh 2–10 kg/m² including structure, so the panel mass is adjustable, with 5 kg/m² as the default.
- **Alerts panel:** a column on the right on wide screens, and a button in the bottom-left corner on phones.
- **📱 Share** shows a QR code for the live demo, and **☀️ / 🌙** switches light and dark mode.
- **On the live link:** a first-time visitor gets a one-time **New here?** banner. Each browser remembers it has been shown; nothing is sent anywhere.

## Project layout

- `coolant/core.py`:
  - `Config` (thresholds)
  - `Controller` (sense → forecast → supervise → act)
  - `equilibrium()` (energy balance) and `sizing()`
  - the `HAL` hardware interface, the `Relay` simulator, and `Engine` (control loop and event log)
- `coolant/server.py`: FastAPI.
  - `GET /api/state`, `GET /api/stream` (Server-Sent Events)
  - `POST /api/setup` (`mode`, `speed`) and `POST /api/scenario/{name}`
  - `POST /api/size` (`normal_kw`, `surge_kw`, `surge_min`, `between_h`, `radiator_c`, `panel_kg_m2`)
  - `GET/POST /api/config`
- `web/index.html`: dashboard, How it works page, sizing and guided demo. No build step. It talks to the server when there is one, and otherwise runs `core.py` in the browser.
- `web/qr.png`: QR code for the live demo. `index.html` sends GitHub Pages visitors to `web/`.
- `start.bat`: Windows one-click launcher.
- `tests/test_core.py`: tests for every tier, the freeze guard, the watchdog rules and the alerts.

## Real hardware

Subclass `coolant.core.HAL`, then run with `COOLANT_HAL="my_hw:MyHAL" COOLANT_TICK_S=1 python -m coolant.server`.

`read()` returns a dict with:

- `temps`: 3 readings in °C, `None` for a dead sensor
- `load_w` and `demand_w`: server power now, and what is requested
- `chip_c`: hottest chip, from the servers' BMC
- `theta`: radiator angle to the sun (180 = facing away)
- `dep`: 0 = stowed, 1 = deployed
- `absorb_w` and `emit_w`: what fully deployed radiators would absorb and emit right now
- `adcs_ok` and `window`
- `pcm_melt`: fraction of the wax melted
- `pcm_melt_c`: the wax's melting point
- `pcm_w`: heat the wax would take with its valve fully open (negative means it would give heat back)
- `sun_w`: heat the sun-side pass would add with its valve fully open

`apply(cmd)` receives:

- `load`: IT power cap, 0 to 1
- `theta` and `dep`
- `pcm`: wax valve, 0 to 1
- `sun`: sun-side pass valve, 0 to 1
- `heater`: watts
- `survival`: arms the hardware thermostat

## The physics, honestly

- **In orbit, hot water doesn't rise.** Convection needs gravity, so the loop has a small pump. A loop heat pipe, which moves its fluid with a capillary wick (usually ammonia, which doesn't freeze until −78 °C), would do it with no pump.
- **Radiators shed heat in proportion to their area,** so they are thin panels, not tanks. Sizing them for the average load and letting the wax absorb the peaks is what saves material.
- **Water expands about 9% when it freezes** and can split pipes, so the freeze guard is not optional.
- **The simulator is a demo, not a validated model.**
  - 4 kg of water and 1.5 m² of radiator (ε 0.85, α 0.15).
  - 6 kg of wax, and a 15% leak when the radiators are stowed.
  - Constant sunlight, since a solar orbit has no planet shadow. A close pass triples it.
  - Sensor drift is sped up for the demo.
- **The hottest chip is modelled** as the coolant plus a load-proportional rise with a 15 s lag. Real chips also throttle themselves in hardware.
- **n-eicosane is a common paraffin PCM.** A real wax bank needs metal fins or foam, because melted wax doesn't circulate without gravity, and room for the wax to expand as it melts.
- **Always add an independent hardware thermal cutout.** Software must never be the only protection.
- **The API has no authentication.** It's a demo, so keep it on a trusted network.

Source for the radiator panel mass: S. G. Turyshev, "Orbital Data Centers: Spacecraft Constraints and Economic Viability", arXiv:2604.27197 (2026), which gives 2–10 kg/m² including structure and uses 5 kg/m² as its base case.
