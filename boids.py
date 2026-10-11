# Jan Magne Kampen.
"""Boids flocking simulation.

Left mouse button places food the boids are attracted to,
right mouse button places danger the boids flee from. Esc quits.
"""

import math
import random

import pygame
from pygame import Vector2

# Display size. Change these to resize the window.
SCREEN_RES_X = 2200
SCREEN_RES_Y = 1200
FPS = 60
DT = 1 / FPS  # Fixed simulation step in seconds; keeps behaviour frame-rate independent of lag.

# Flock size and movement. Speeds are in pixels/second, forces in pixels/second².
NUM_BOIDS = 352  # ~133 boids per million pixels of screen.
MIN_SPEED = 90  # Birds never hover; they always keep some forward speed.
CRUISE_SPEED = 135  # Speed boids settle back to when nothing else is going on.
MAX_SPEED = 170
SPEED_GAIN = 1.5  # How quickly boids return to CRUISE_SPEED (per second).
PANIC_SPEED = 260  # Speed cap while fleeing danger.
MAX_FORCE = 260  # Limits how sharply a boid can turn: radius ≈ speed² / force.

# Perception.
VIEW_RADIUS = 75  # Neighbours further away than this are ignored.
VIEW_ANGLE = 270  # Field of view in degrees; boids can't see straight behind them.
SEPARATION_RADIUS = 28  # Neighbours closer than this are pushed away.

# Rule weights.
SEPARATION_FORCE = 900  # Push from a neighbour right on top of us; fades to 0 at SEPARATION_RADIUS.
ALIGNMENT_WEIGHT = 1.0
COHESION_WEIGHT = 0.35
WANDER_FORCE = 60  # Slow random drift that keeps the flock from looking mechanical.
WANDER_TIME = 1.0  # Seconds the drift keeps roughly the same direction.

# Screen edges: boids start turning back when they come within EDGE_MARGIN.
EDGE_MARGIN = 110
EDGE_FORCE = 420

# Mouse interaction.
FOOD_WEIGHT = 1.5
FOOD_RADIUS = 300  # Boids only notice food closer than this.
FOOD_FADE = 60  # The pull fades out over the last FOOD_FADE pixels of the radius.
DANGER_RADIUS = 220  # Boids only flee danger closer than this.
DANGER_FORCE = 900

# Colors: dark birds against a dusk sky, like a starling murmuration.
SKY_TOP = (38, 52, 88)
SKY_BOTTOM = (226, 150, 112)
BIRD_COLOR = (22, 20, 30)
BERRY_COLOR = (58, 30, 92)
BERRY_SHINE = (190, 160, 225)
STEM_COLOR = (70, 52, 34)
LEAF_COLOR = (72, 132, 58)
HAWK_COLOR = (58, 38, 26)

# Hawk silhouette seen from below, nose pointing along +x. Only the right half
# (y >= 0) is listed; draw_hawk() mirrors it. Points with |y| > 8 are wing.
_HAWK_HALF = [
    (20, 0), (18, 3), (14, 4), (10, 5),  # Small round head.
    (9, 12), (7, 22), (4, 32), (1, 38),  # Broad wing, leading edge.
    (-3, 40), (-4, 36), (-7, 38), (-8, 33), (-11, 34), (-11, 28),  # Fingered tip.
    (-12, 20), (-10, 10), (-8, 6),  # Trailing edge.
    (-12, 6), (-24, 10), (-27, 7), (-28, 3), (-28, 0),  # Fanned tail.
]

_VIEW_COS = math.cos(math.radians(VIEW_ANGLE / 2))


def limit(v: Vector2, max_length: float) -> Vector2:
    """Return v shortened to at most max_length."""
    if v.length_squared() > max_length * max_length:
        v.scale_to_length(max_length)
    return v


class Grid:
    """Buckets boids into VIEW_RADIUS-sized cells for fast neighbour lookup.

    A boid can only see others within VIEW_RADIUS, so they all lie in its own
    cell or the 8 around it. This replaces checking every pair of boids.
    """

    def __init__(self, boids: list["Boid"]):
        self.cells: dict[tuple[int, int], list[Boid]] = {}
        for b in boids:
            self.cells.setdefault(self._key(b.pos), []).append(b)

    @staticmethod
    def _key(pos: Vector2) -> tuple[int, int]:
        return int(pos.x // VIEW_RADIUS), int(pos.y // VIEW_RADIUS)

    def nearby(self, pos: Vector2) -> list["Boid"]:
        """Boids in the 3x3 block of cells around pos."""
        cx, cy = self._key(pos)
        found = []
        for x in (cx - 1, cx, cx + 1):
            for y in (cy - 1, cy, cy + 1):
                found.extend(self.cells.get((x, y), ()))
        return found


class Boid:
    """A bird with a position and velocity, steered by Reynolds-style forces."""

    def __init__(self):
        self.pos = Vector2(random.uniform(0, SCREEN_RES_X), random.uniform(0, SCREEN_RES_Y))
        self.vel = Vector2(random.uniform(MIN_SPEED, MAX_SPEED), 0).rotate(random.uniform(0, 360))
        self.flap = random.uniform(0, math.tau)  # Wing-beat phase, only for drawing.
        self.panicking = False
        self.wander = 0.0  # Smoothly varying sideways drift, roughly in [-1, 1].

    def neighbours(self, grid: Grid) -> list[tuple["Boid", Vector2, float]]:
        """Visible boids within VIEW_RADIUS, as (boid, offset to it, distance)."""
        heading = self.vel.normalize()
        found = []
        for b in grid.nearby(self.pos):
            if b is self:
                continue
            offset = b.pos - self.pos
            dist_sq = offset.length_squared()
            if dist_sq >= VIEW_RADIUS * VIEW_RADIUS or dist_sq == 0:
                continue
            dist = math.sqrt(dist_sq)
            # Skip boids in the blind spot behind us.
            if heading.dot(offset) / dist < _VIEW_COS:
                continue
            found.append((b, offset, dist))
        return found

    def steer_towards(self, direction: Vector2) -> Vector2:
        """Force that turns the boid to fly at full speed in direction."""
        if direction.length_squared() == 0:
            return Vector2()
        desired = direction.normalize() * MAX_SPEED
        return limit(desired - self.vel, MAX_FORCE)

    def flocking(self, grid: Grid) -> Vector2:
        """Separation, alignment and cohesion combined into one force."""
        near = self.neighbours(grid)
        if not near:
            return Vector2()

        away = Vector2()
        heading = Vector2()
        centre = Vector2()
        for b, offset, dist in near:
            if dist < SEPARATION_RADIUS:
                # Push away harder the closer the neighbour is.
                away -= offset / dist * (1 - dist / SEPARATION_RADIUS)
            heading += b.vel
            centre += offset

        force = away * SEPARATION_FORCE
        force += self.steer_towards(heading) * ALIGNMENT_WEIGHT
        force += self.steer_towards(centre / len(near)) * COHESION_WEIGHT
        return force

    def edges(self) -> Vector2:
        """Turn back from the screen edges, harder the closer the boid gets."""
        force = Vector2()
        if self.pos.x < EDGE_MARGIN:
            force.x = 1 - self.pos.x / EDGE_MARGIN
        elif self.pos.x > SCREEN_RES_X - EDGE_MARGIN:
            force.x = -(1 - (SCREEN_RES_X - self.pos.x) / EDGE_MARGIN)
        if self.pos.y < EDGE_MARGIN:
            force.y = 1 - self.pos.y / EDGE_MARGIN
        elif self.pos.y > SCREEN_RES_Y - EDGE_MARGIN:
            force.y = -(1 - (SCREEN_RES_Y - self.pos.y) / EDGE_MARGIN)
        return force * EDGE_FORCE

    def seek_food(self, food: Vector2 | None) -> Vector2:
        """Steer towards food the boid can see (within FOOD_RADIUS).

        Boids further away don't notice it themselves, but alignment and
        cohesion still let them follow flockmates that do.
        """
        if food is None:
            return Vector2()
        to_food = food - self.pos
        dist = to_food.length()
        if dist >= FOOD_RADIUS:
            return Vector2()
        # Fade in at the edge so boids don't jerk as they cross the radius.
        strength = min(1.0, (FOOD_RADIUS - dist) / FOOD_FADE)
        return self.steer_towards(to_food) * FOOD_WEIGHT * strength

    def flee(self, danger: Vector2 | None) -> Vector2:
        """Force away from danger, fading to zero at DANGER_RADIUS.

        It must weaken with distance; a repulsion that grows with distance
        overpowers edges() and throws boids off-screen permanently.
        """
        self.panicking = False
        if danger is None:
            return Vector2()
        away = self.pos - danger
        dist = away.length()
        if dist >= DANGER_RADIUS:
            return Vector2()
        self.panicking = True
        if dist == 0:
            away = Vector2(self.vel)
        return away.normalize() * DANGER_FORCE * (1 - dist / DANGER_RADIUS)

    def acceleration(self, grid: Grid, food: Vector2 | None,
                     danger: Vector2 | None) -> Vector2:
        """Total steering force for this step."""
        force = self.flocking(grid)
        force += self.seek_food(food)
        heading = self.vel.normalize()
        # Smooth random walk (Ornstein-Uhlenbeck), not per-frame noise, so
        # boids meander instead of twitching.
        decay = math.exp(-DT / WANDER_TIME)
        self.wander = self.wander * decay + random.gauss(0, math.sqrt(1 - decay * decay))
        force += heading.rotate(90) * self.wander * WANDER_FORCE
        force = limit(force, MAX_FORCE)
        # Added after the limit so flocking can't override them.
        force += heading * (CRUISE_SPEED - self.vel.length()) * SPEED_GAIN
        return force + self.edges() + self.flee(danger)

    def move(self, accel: Vector2) -> None:
        self.vel += accel * DT
        speed = self.vel.length()
        max_speed = PANIC_SPEED if self.panicking else MAX_SPEED
        if speed > max_speed:
            self.vel.scale_to_length(max_speed)
        elif speed < MIN_SPEED:
            self.vel.scale_to_length(MIN_SPEED)
        self.pos += self.vel * DT
        # Faster flight means faster wing beats.
        self.flap += DT * (10 + 10 * speed / MAX_SPEED)

    def draw(self, surface: pygame.Surface) -> None:
        """Draw the boid as a small bird silhouette with flapping wings."""
        fwd = self.vel.normalize()
        side = fwd.rotate(90)
        beat = math.sin(self.flap)
        span = 6 + 1.0 * beat
        sweep = 4 + 2.5 * beat  # Wingtips swing back and forth with each beat.
        p = self.pos
        points = [
            p + fwd * 5,  # Beak.
            p - fwd * sweep + side * span,  # Wing tip.
            p,  # Notch between the wings, so it reads as a bird, not an arrow.
            p - fwd * sweep - side * span,  # Other wing tip.
        ]
        pygame.draw.polygon(surface, BIRD_COLOR, points)


def draw_food(surface: pygame.Surface, pos: Vector2) -> None:
    """A sprig of dark berries with a leaf."""
    berries = [(0, 0), (8, -3), (-7, -4), (4, 8), (-5, 7), (10, 6)]
    top = pos + Vector2(1, -16)
    for dx, dy in berries:
        pygame.draw.aaline(surface, STEM_COLOR, top, pos + Vector2(dx, dy))
    leaf = [top, top + Vector2(10, -9), top + Vector2(19, -6), top + Vector2(9, 1)]
    pygame.draw.polygon(surface, LEAF_COLOR, leaf)
    pygame.draw.aalines(surface, LEAF_COLOR, True, leaf)
    for dx, dy in berries:
        c = pos + Vector2(dx, dy)
        pygame.draw.aacircle(surface, BERRY_COLOR, c, 5)
        pygame.draw.aacircle(surface, BERRY_SHINE, c + Vector2(-1.7, -1.7), 1.4)


def draw_hawk(surface: pygame.Surface, pos: Vector2, heading: Vector2, flap: float) -> None:
    """A soaring hawk facing heading, its wings flexing slowly."""
    angle = heading.as_polar()[1]
    wing = 1 + 0.08 * math.sin(flap)
    half = [Vector2(x, y * wing if y > 8 else y) for x, y in _HAWK_HALF]
    outline = half + [Vector2(v.x, -v.y) for v in reversed(half[1:-1])]
    points = [pos + v.rotate(angle) for v in outline]
    pygame.draw.polygon(surface, HAWK_COLOR, points)
    pygame.draw.aalines(surface, HAWK_COLOR, True, points)


def make_sky() -> pygame.Surface:
    """Vertical dusk gradient used as the background."""
    sky = pygame.Surface((SCREEN_RES_X, SCREEN_RES_Y))
    for y in range(SCREEN_RES_Y):
        t = y / (SCREEN_RES_Y - 1)
        color = [round(a + (b - a) * t) for a, b in zip(SKY_TOP, SKY_BOTTOM)]
        pygame.draw.line(sky, color, (0, y), (SCREEN_RES_X, y))
    return sky


def run() -> None:
    """Main loop."""
    pygame.init()
    pygame.display.set_caption("Boids")
    screen = pygame.display.set_mode((SCREEN_RES_X, SCREEN_RES_Y))
    clock = pygame.time.Clock()
    sky = make_sky()

    boids = [Boid() for _ in range(NUM_BOIDS)]
    hawk_heading = Vector2(0, 1)  # Hawk faces the way the mouse last moved.
    last_mouse = Vector2(pygame.mouse.get_pos())
    ticks = 0

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT or (
                event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE
            ):
                running = False

        left, _, right = pygame.mouse.get_pressed()
        mouse = Vector2(pygame.mouse.get_pos())
        food = mouse if left and not right else None
        danger = mouse if right and not left else None
        moved = mouse - last_mouse
        if moved.length() > 1:
            # Ease towards the new direction so the hawk banks instead of snapping.
            hawk_heading = hawk_heading.lerp(moved.normalize(), 0.2)
            if hawk_heading.length() > 0:
                hawk_heading.normalize_ip()
        last_mouse = mouse
        ticks += 1

        # All boids steer from the same snapshot of the flock, then move.
        grid = Grid(boids)
        accels = [b.acceleration(grid, food, danger) for b in boids]
        for b, accel in zip(boids, accels):
            b.move(accel)

        screen.blit(sky, (0, 0))
        if food is not None:
            draw_food(screen, food)
        for b in boids:
            b.draw(screen)
        if danger is not None:
            # Drawn last: the hawk flies above the flock.
            draw_hawk(screen, danger, hawk_heading, ticks * DT * 3)
        pygame.display.flip()

        clock.tick(FPS)

    pygame.quit()


if __name__ == "__main__":
    run()
