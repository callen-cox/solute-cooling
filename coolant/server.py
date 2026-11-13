"""FastAPI backend: REST + Server-Sent Events + the static dashboard.

Run:  python -m coolant.server   ->  http://localhost:8000
Env:  COOLANT_HAL="module:Class"  drive real hardware instead of the simulator (also set COOLANT_TICK_S=1)
"""
import asyncio
import importlib
import json
import os
from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import Body, FastAPI, HTTPException
from fastapi.responses import StreamingResponse
from fastapi.staticfiles import StaticFiles

from .core import Engine, sizing


def make_engine():
    hal, spec = None, os.getenv("COOLANT_HAL")
    if spec:
        mod, cls = spec.split(":")
        hal = getattr(importlib.import_module(mod), cls)()
    return Engine(hal=hal, tick_s=float(os.getenv("COOLANT_TICK_S", "0.1")))


engine = make_engine()


@asynccontextmanager
async def lifespan(_):
    engine.start()
    yield
    engine.stop()


app = FastAPI(title="Autonomous Thermal Controller", lifespan=lifespan)


@app.get("/api/state")
def state():
    return engine.snapshot(history=True)


@app.get("/api/stream")
async def stream():
    async def gen():
        last = -1.0
        while True:
            snap = engine.snapshot(since=last)
            if snap["new"]:
                last = snap["new"][-1]["t"]
            yield f"data: {json.dumps(snap)}\n\n"
            await asyncio.sleep(0.25)
    return StreamingResponse(gen(), media_type="text/event-stream",
                             headers={"Cache-Control": "no-cache", "X-Accel-Buffering": "no"})


@app.post("/api/setup")
def setup(options: dict = Body(...)):
    """Change environment, sink, mode and/or speed, e.g. {"env": "orbital", "mode": "stealth"}."""
    try:
        return engine.setup(**options)
    except (ValueError, TypeError) as e:
        raise HTTPException(422, str(e))


@app.post("/api/size")
def size(options: dict = Body(...)):
    """Sizing calculator, e.g. {"kw": 2, "hours": 1, "coolant_c": 40, "radiator_c": 30}."""
    try:
        return sizing(**options)
    except (ValueError, TypeError) as e:
        raise HTTPException(422, str(e))


@app.post("/api/scenario/{name}")
def scenario(name: str):
    try:
        return engine.scenario(name)
    except KeyError:
        raise HTTPException(404, f"unknown scenario: {name}")


@app.get("/api/config")
def get_config():
    return engine.snapshot()["config"]


@app.post("/api/config")
def set_config(patch: dict = Body(...)):
    try:
        return engine.update_config(patch)
    except (ValueError, TypeError) as e:
        raise HTTPException(422, str(e))


app.mount("/", StaticFiles(directory=Path(__file__).resolve().parent.parent / "web", html=True))

if __name__ == "__main__":
    import sys
    import threading
    import webbrowser

    import uvicorn
    port = int(os.getenv("PORT", "8000"))
    if "--open" in sys.argv:            # start.bat: open the dashboard once the server is up
        threading.Timer(2.0, webbrowser.open, [f"http://localhost:{port}"]).start()
    uvicorn.run(app, host=os.getenv("HOST", "0.0.0.0"), port=port)
