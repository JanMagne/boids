# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Setup and running

Python 3.14 with **pygame-ce** (the community fork; upstream `pygame` 2.6.1 has no Python 3.14 wheels and fails to build). pygame-ce is a drop-in replacement — code still does `import pygame`. Never install both `pygame` and `pygame-ce` in the same venv; they share the `pygame` module name.

```bash
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
.venv/bin/python boids.py
```

Headless smoke test (no window; posts QUIT after 3 s):

```bash
SDL_VIDEODRIVER=dummy .venv/bin/python -c "
import pygame, boids
from threading import Timer
Timer(3, lambda: pygame.event.post(pygame.event.Event(pygame.QUIT))).start()
boids.run()"
```

There are no tests or linter configured.

## Architecture

Single file, `boids.py`. All tunables are module-level constants at the top. Units are pixels/second for speeds and pixels/second² for forces; the simulation steps with a fixed `DT = 1/FPS`.

- Reynolds-style steering: each rule produces a force, `Boid.acceleration()` sums them, `Boid.move()` integrates and clamps speed to `[MIN_SPEED, MAX_SPEED]` (`PANIC_SPEED` while fleeing). `MAX_FORCE` caps the flocking forces, which is what makes turns smooth arcs (turn radius ≈ speed² / force).
- `neighbours()` uses a `VIEW_RADIUS` plus a `VIEW_ANGLE` blind spot behind the boid.
- Separation is a direct force graded by closeness, not `steer_towards()`. Because boids mostly see neighbours ahead, separation pushes backwards; a full-strength (binary) separation pinned every boid at `MIN_SPEED`. The `CRUISE_SPEED` thrust is added **after** the `MAX_FORCE` limit for the same reason — if it is clipped with the other forces, separation wins and the flock slows to a crawl.
- Food is only noticed within `FOOD_RADIUS` (fading over the last `FOOD_FADE` px); birds further away reach it only by following flockmates. `FOOD_WEIGHT` was raised to 1.5 with the radius — at 0.6 birds circled out past the radius, lost sight of the food and the crowd dispersed.
- Edges and `flee()` are also added after the limit so flocking can never override them. `flee()` must weaken with distance and stop at `DANGER_RADIUS`; a distance-proportional repulsion throws boids off-screen permanently.
- Wander is an Ornstein–Uhlenbeck random walk (`WANDER_TIME`), not per-frame noise — per-frame Gaussian jitter made boids visibly twitch.
- `run()` computes every boid's acceleration from one snapshot, then moves them all. This only works because of the cruise thrust; the original rules (no speed source) stalled under snapshot updates.
- Neighbour lookup goes through `Grid`, rebuilt every frame: cells are `VIEW_RADIUS` wide, so a boid only checks its own and the 8 surrounding cells. If you ever make perception reach further than `VIEW_RADIUS`, the cell size must grow with it or neighbours will be missed. 352 boids cost ~3.3 ms/frame (13 ms with the old all-pairs loop).
- `NUM_BOIDS` is sized to keep ~133 boids per million screen pixels; scale it with the window to keep the same density.
- Vectors are `pygame.Vector2`; "no target" for food/danger is `None`.

## Verifying behaviour

Judge flocking from metrics over time, not single frames: mean speed, local alignment (dot of each boid's heading with its neighbours' mean heading), nearest-neighbour distance, number of groups, boids off-screen. Healthy values for the current tuning: speed ~100–130, local alignment ~0.85–0.98, nearest-neighbour mean ~25–30 px, groups that split and merge over time, off-screen ≈ 0. For the look, draw each boid's last ~1.5 s of positions as lines ("long exposure") — paths should be smooth parallel arcs, not zig-zags. To drive the window without a person, monkeypatch `pygame.mouse.get_pressed`/`get_pos` and wrap `pygame.display.flip` to capture frames.
