#!/usr/bin/env python3
"""Standalone 2D robot simulator (Python standard library only - no ROS needed).

    python3 sim/tk_sim.py

Controls
    w/s or Up/Down     change forward speed
    a/d or Left/Right  change turn rate
    space / x          stop
    Buttons            Square, Circle, Stop, Reset pose, Clear trail

The robot model, teleop and square/circle logic are imported from
``src/robotics_starter/robotics_starter/motion.py`` - the very same code
the ROS 2 nodes use.
"""

from __future__ import annotations

import math
import os
import sys
import time

# Make the shared ROS package's pure-Python module importable without ROS.
_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, '..', 'src', 'robotics_starter'))

from robotics_starter.motion import (  # noqa: E402
    Pose, ShapeFollower, TELEOP_HELP, TeleopController, integrate, twist_to_wheels,
)

# --------------------------------------------------------------------------- #
# Simulation core (no GUI - can be tested headless)
# --------------------------------------------------------------------------- #
WHEEL_BASE = 0.4      # m, used to show wheel speeds
MAX_TRAIL = 20000     # stored trail points


class SimCore:
    """Robot state + current controller. Advance it with :meth:`step`."""

    def __init__(self, shape_size: float = 2.0, linear_speed: float = 0.5,
                 angular_speed: float = 0.8):
        self.shape_size = shape_size
        self.linear_speed = linear_speed
        self.angular_speed = angular_speed
        self.teleop = TeleopController()
        self.reset()

    # --- state -------------------------------------------------------------
    def reset(self) -> None:
        self.pose = Pose()
        self.v = 0.0
        self.w = 0.0
        self.time = 0.0
        self.distance = 0.0
        self.mode = 'TELEOP'
        self.follower = None
        self.follower_start = 0.0
        self.teleop.stop()
        self.trail = [(0.0, 0.0)]

    def clear_trail(self) -> None:
        self.trail = [(self.pose.x, self.pose.y)]

    # --- commands ----------------------------------------------------------
    def key(self, key: str) -> bool:
        """Feed a key to teleop. A valid teleop key cancels an autonomous shape."""
        probe = TeleopController()
        if probe.handle_key(key) is None:
            return False
        if self.mode != 'TELEOP':
            self.stop()
        self.teleop.handle_key(key)
        return True

    def start_shape(self, shape: str) -> None:
        self.teleop.stop()
        self.follower = ShapeFollower(shape, self.shape_size,
                                      self.linear_speed, self.angular_speed)
        self.follower_start = self.time
        self.mode = shape.upper()

    def stop(self) -> None:
        self.mode = 'TELEOP'
        self.follower = None
        self.teleop.stop()

    # --- physics -----------------------------------------------------------
    def step(self, dt: float) -> None:
        if self.follower is None:
            self.v, self.w = self.teleop.v, self.teleop.w
            self._advance(dt)
            return
        # Autonomous: sub-step at segment boundaries so corners are exact.
        while dt > 1e-12 and self.follower is not None:
            elapsed = self.time - self.follower_start
            self.v, self.w, done = self.follower.command(elapsed)
            if done:
                self.stop()
                self.v = self.w = 0.0
                break
            # Small epsilon makes sure we cross the boundary despite float error.
            h = min(dt, self.follower.time_to_next_change(elapsed) + 1e-9)
            self._advance(h)
            dt -= h
        if self.follower is None:
            self.v = self.w = 0.0

    def _advance(self, dt: float) -> None:
        if dt <= 0:
            return
        old = self.pose
        self.pose = integrate(self.pose, self.v, self.w, dt)
        self.time += dt
        moved = math.hypot(self.pose.x - old.x, self.pose.y - old.y)
        self.distance += moved
        if moved > 1e-4:
            self.trail.append((self.pose.x, self.pose.y))
            if len(self.trail) > MAX_TRAIL:
                del self.trail[: len(self.trail) - MAX_TRAIL]

    def readout(self) -> str:
        vl, vr = twist_to_wheels(self.v, self.w, WHEEL_BASE)
        return (f'mode   {self.mode}\n'
                f'x      {self.pose.x:+7.2f} m\n'
                f'y      {self.pose.y:+7.2f} m\n'
                f'theta  {math.degrees(self.pose.theta):+7.1f} deg\n'
                f'v      {self.v:+7.2f} m/s\n'
                f'w      {self.w:+7.2f} rad/s\n'
                f'wheels L {vl:+.2f}  R {vr:+.2f} m/s\n'
                f'dist   {self.distance:7.2f} m\n'
                f'time   {self.time:7.1f} s')


# --------------------------------------------------------------------------- #
# Tkinter GUI
# --------------------------------------------------------------------------- #
CANVAS_W, CANVAS_H = 800, 600
SCALE = 50.0          # pixels per metre  -> 16 m x 12 m world
ROBOT_RADIUS = 0.25   # m
FRAME_MS = 20         # ~50 FPS

COLORS = {
    'bg': '#1e1f24', 'grid': '#2c2e35', 'axis': '#4a4d57', 'trail': '#36b3a8',
    'robot': '#e8a33d', 'heading': '#1e1f24', 'text': '#d8dae0', 'panel': '#26282e',
}


def run_gui() -> None:
    try:
        import tkinter as tk
    except ImportError:
        sys.exit('tkinter is not installed. On Ubuntu/Debian run: sudo apt install python3-tk')

    sim = SimCore()
    root = tk.Tk()
    root.title('Robotics Starter - 2D Simulator')
    root.configure(bg=COLORS['panel'])
    root.resizable(False, False)

    canvas = tk.Canvas(root, width=CANVAS_W, height=CANVAS_H, bg=COLORS['bg'],
                       highlightthickness=0)
    canvas.grid(row=0, column=0, rowspan=2, padx=8, pady=8)

    side = tk.Frame(root, bg=COLORS['panel'])
    side.grid(row=0, column=1, sticky='n', padx=(0, 8), pady=8)

    def button(text, cmd):
        b = tk.Button(side, text=text, command=cmd, width=16, takefocus=0)
        b.pack(pady=2)
        return b

    tk.Label(side, text='Autonomous', bg=COLORS['panel'], fg=COLORS['text'],
             font=('TkDefaultFont', 10, 'bold')).pack(anchor='w', pady=(0, 2))
    button('Square', lambda: sim.start_shape('square'))
    button('Circle', lambda: sim.start_shape('circle'))
    button('Stop', sim.stop)

    tk.Label(side, text='Size (m)', bg=COLORS['panel'], fg=COLORS['text']).pack(anchor='w',
                                                                                pady=(6, 0))
    size_var = tk.DoubleVar(value=sim.shape_size)

    def on_size(_value):
        sim.shape_size = size_var.get()

    tk.Scale(side, from_=0.5, to=5.0, resolution=0.5, orient='horizontal',
             variable=size_var, command=on_size, length=150, takefocus=0).pack()

    tk.Label(side, text='View', bg=COLORS['panel'], fg=COLORS['text'],
             font=('TkDefaultFont', 10, 'bold')).pack(anchor='w', pady=(8, 2))
    button('Reset pose', sim.reset)
    button('Clear trail', sim.clear_trail)

    readout = tk.Label(side, text='', justify='left', anchor='w', bg=COLORS['panel'],
                       fg=COLORS['text'], font=('TkFixedFont', 10))
    readout.pack(anchor='w', pady=(10, 0))

    tk.Label(side, text=TELEOP_HELP, justify='left', anchor='w', bg=COLORS['panel'],
             fg='#8a8d96', font=('TkFixedFont', 8)).pack(anchor='w', pady=(10, 0))

    # --- drawing helpers --------------------------------------------------
    cx, cy = CANVAS_W / 2.0, CANVAS_H / 2.0

    def to_px(x: float, y: float):
        return cx + x * SCALE, cy - y * SCALE   # y up in the world, down on screen

    def draw_grid():
        for i in range(int(-cx // SCALE), int(cx // SCALE) + 1):
            px = cx + i * SCALE
            canvas.create_line(px, 0, px, CANVAS_H,
                               fill=COLORS['axis'] if i == 0 else COLORS['grid'])
        for j in range(int(-cy // SCALE), int(cy // SCALE) + 1):
            py = cy + j * SCALE
            canvas.create_line(0, py, CANVAS_W, py,
                               fill=COLORS['axis'] if j == 0 else COLORS['grid'])
        canvas.create_text(8, CANVAS_H - 8, anchor='sw', fill='#6b6e78',
                           text='grid = 1 m   |   origin at centre')

    draw_grid()
    trail_id = canvas.create_line(0, 0, 0, 0, fill=COLORS['trail'], width=2)
    r_px = ROBOT_RADIUS * SCALE
    body_id = canvas.create_oval(0, 0, 0, 0, fill=COLORS['robot'], outline='')
    head_id = canvas.create_line(0, 0, 0, 0, fill=COLORS['heading'], width=3)

    def draw():
        if len(sim.trail) >= 2:
            pts = []
            for x, y in sim.trail:
                pts.extend(to_px(x, y))
            canvas.coords(trail_id, *pts)
        else:
            canvas.coords(trail_id, 0, 0, 0, 0)
        px, py = to_px(sim.pose.x, sim.pose.y)
        canvas.coords(body_id, px - r_px, py - r_px, px + r_px, py + r_px)
        hx = px + r_px * math.cos(sim.pose.theta)
        hy = py - r_px * math.sin(sim.pose.theta)
        canvas.coords(head_id, px, py, hx, hy)
        readout.config(text=sim.readout())

    # --- input ------------------------------------------------------------
    def on_key(event):
        key = event.keysym.lower() if len(event.keysym) > 1 else event.char
        sim.key(key)

    root.bind('<KeyPress>', on_key)

    # --- main loop ----------------------------------------------------------
    last = [time.monotonic()]

    def tick():
        now = time.monotonic()
        dt = min(now - last[0], 0.1)    # clamp if the window was dragged / paused
        last[0] = now
        sim.step(dt)
        draw()
        root.after(FRAME_MS, tick)

    print(TELEOP_HELP)
    tick()
    root.mainloop()


if __name__ == '__main__':
    run_gui()
