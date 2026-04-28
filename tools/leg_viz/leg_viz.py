"""leg_viz — 3D chassis-level PC debug GUI for FengMH.

This tool intentionally avoids Matplotlib widget buttons. On macOS the 3D
backend can keep accepting mouse rotation while widget callbacks stop firing.
The app uses keyboard events and one backend timer, so the window does not get
re-raised by a tight plt.pause loop.
"""
from __future__ import annotations

import sys
import time
from collections import deque
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from tools.leg_viz.sim_bridge import (  # noqa: E402
    GAIT_LEG_NUM,
    HostSystemSim,
    LEG_ACT_HIP,
    LEG_ACT_KNEE,
    LEG_ACT_WHEEL,
    LEG_DIM_DEFAULT,
    list_builtin_scripts,
    list_leg_configs,
)


ACTIVE_ALL = 0
ACTIVE_FIRST_LEG = 1


class LegVizApp:
    def __init__(self):
        self.legs = list_leg_configs()
        if len(self.legs) != GAIT_LEG_NUM:
            raise RuntimeError(f"expected {GAIT_LEG_NUM} legs, got {len(self.legs)}")
        self.host = HostSystemSim()

        self.thigh = LEG_DIM_DEFAULT.thigh_length
        self.shin = LEG_DIM_DEFAULT.shin_length
        self.height = 0.25
        self.body_z = 0.25
        self.body_pitch_deg = 0.0
        self.body_roll_deg = 0.0

        self.foot_x = [0.0 for _ in self.legs]
        self.foot_z = [0.0 for _ in self.legs]
        self.last_raw = [(np.pi / 2, np.pi / 2) for _ in self.legs]
        self.last_cmd = [(0.0, 0.0) for _ in self.legs]
        self.ik_ok = [True for _ in self.legs]
        self.trails = [deque(maxlen=240) for _ in self.legs]

        self.active_selection = ACTIVE_ALL
        self.scripts = list_builtin_scripts()
        self.active_script_index = self._default_script_index()
        self.playing = False
        self.t_play = 0.0
        self.dt = 0.05
        self._dirty = True
        self._view_index = 0
        self._last_step = time.monotonic()
        self._timer = None
        self._script_loaded = None

        self._apply_manual_targets()

        self._build_figure()
        self._draw_all()

    # ------------------------------------------------------------------ UI

    @property
    def active_script(self):
        if self.active_script_index < 0 or self.active_script_index >= len(self.scripts):
            return None
        return self.scripts[self.active_script_index]

    def _default_script_index(self):
        if not self.scripts:
            return -1
        if "trot_step" in self.scripts:
            return self.scripts.index("trot_step")
        return 0

    @property
    def active_label(self):
        if self.active_selection == ACTIVE_ALL:
            return "ALL"
        return self.legs[self.active_selection - ACTIVE_FIRST_LEG].name

    def _build_figure(self):
        self.fig = plt.figure(figsize=(15, 9))
        self.fig.canvas.manager.set_window_title("FengMH 3D chassis leg debug")
        self.fig.canvas.mpl_connect("key_press_event", self._on_key)

        self.ax3d = self.fig.add_axes([0.04, 0.08, 0.64, 0.86], projection="3d")
        self.ax3d.set_title("3D chassis / four-leg IK")
        self.ax3d.set_xlabel("X forward (m)")
        self.ax3d.set_ylabel("Y left (m)")
        self.ax3d.set_zlabel("Z up (m)")
        self._apply_view()
        self.ax3d.set_box_aspect((1.25, 0.75, 0.75))

        self.body_poly = Poly3DCollection([], facecolor="#5b7c99", alpha=0.18, edgecolor="#2f4858")
        self.ax3d.add_collection3d(self.body_poly)
        self.body_outline, = self.ax3d.plot([], [], [], color="#243b53", lw=2)
        self.ground = None

        colors = ["tab:blue", "tab:green", "tab:orange", "tab:purple"]
        self.leg_lines = []
        self.foot_points = []
        self.trail_lines = []
        self.hip_labels = []
        for i, leg in enumerate(self.legs):
            line, = self.ax3d.plot([], [], [], "o-", lw=3, color=colors[i], label=leg.name)
            foot, = self.ax3d.plot([], [], [], "o", ms=8, color=colors[i], markeredgecolor="white")
            trail, = self.ax3d.plot([], [], [], "-", lw=1.2, color=colors[i], alpha=0.45)
            label = self.ax3d.text(0, 0, 0, leg.name, fontsize=9)
            self.leg_lines.append(line)
            self.foot_points.append(foot)
            self.trail_lines.append(trail)
            self.hip_labels.append(label)
        self.ax3d.legend(loc="upper left")

        self.ax_info = self.fig.add_axes([0.71, 0.08, 0.27, 0.86])
        self.ax_info.axis("off")
        self.txt_info = self.ax_info.text(
            0.0, 1.0, "", transform=self.ax_info.transAxes,
            va="top", family="monospace", fontsize=8.4,
        )

    # --------------------------------------------------------------- math

    def _selected_leg_indices(self):
        if self.active_selection == ACTIVE_ALL:
            return range(len(self.legs))
        return [self.active_selection - ACTIVE_FIRST_LEG]

    def _body_rotation(self):
        pitch = np.radians(self.body_pitch_deg)
        roll = np.radians(self.body_roll_deg)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cr, sr = np.cos(roll), np.sin(roll)
        ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
        rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
        return ry @ rx

    def _world_from_body(self, point):
        return self._body_rotation() @ np.asarray(point, dtype=float) + np.array([0.0, 0.0, self.body_z])

    def _hip_world(self, leg):
        return self._world_from_body([leg.body_x_m, leg.body_y_m, 0.0])

    def _leg_points_world(self, i):
        leg = self.legs[i]
        pose = self.host.leg_pose(i)
        self.ik_ok[i] = bool(pose.ik_ok)
        hip_raw = float(pose.hip_raw_rad)
        knee_raw = float(pose.knee_raw_rad)
        hip_cmd = float(pose.hip_cmd_rad)
        knee_cmd = float(pose.knee_cmd_rad)
        self.last_raw[i] = (hip_raw, knee_raw)
        self.last_cmd[i] = (hip_cmd, knee_cmd)

        world_knee_x = leg.foot_x_dir * self.thigh * np.cos(hip_raw)
        knee_body = np.array([
            leg.body_x_m + world_knee_x,
            leg.body_y_m,
            -self.thigh * np.sin(hip_raw),
        ])
        foot_body = np.array([
            pose.foot_body_x_m,
            pose.foot_body_y_m,
            -pose.foot_down_z_m,
        ])
        hip = self._hip_world(leg)
        knee = self._world_from_body(knee_body)
        foot = self._world_from_body(foot_body)
        return hip, knee, foot, hip_raw, knee_raw, hip_cmd, knee_cmd

    # --------------------------------------------------------------- drawing

    def _draw_body(self):
        xs = [leg.body_x_m for leg in self.legs]
        ys = [leg.body_y_m for leg in self.legs]
        x_min, x_max = min(xs) - 0.035, max(xs) + 0.035
        y_min, y_max = min(ys) - 0.045, max(ys) + 0.045
        corners_body = [
            [x_max, y_max, 0.0],
            [x_max, y_min, 0.0],
            [x_min, y_min, 0.0],
            [x_min, y_max, 0.0],
        ]
        corners = [self._world_from_body(p) for p in corners_body]
        self.body_poly.set_verts([corners])
        closed = corners + [corners[0]]
        arr = np.array(closed)
        self.body_outline.set_data(arr[:, 0], arr[:, 1])
        self.body_outline.set_3d_properties(arr[:, 2])

    def _draw_ground(self):
        if self.ground is not None:
            return
        x = np.linspace(-0.58, 0.58, 2)
        y = np.linspace(-0.36, 0.36, 2)
        xx, yy = np.meshgrid(x, y)
        zz = np.zeros_like(xx)
        self.ground = self.ax3d.plot_surface(xx, yy, zz, color="#d0d7de", alpha=0.18, linewidth=0)

    def _draw_all(self):
        self._draw_body()
        self._draw_ground()
        info_rows = []
        for i, leg in enumerate(self.legs):
            hip, knee, foot, hip_raw, knee_raw, hip_cmd, knee_cmd = self._leg_points_world(i)
            points = np.vstack([hip, knee, foot])
            self.leg_lines[i].set_data(points[:, 0], points[:, 1])
            self.leg_lines[i].set_3d_properties(points[:, 2])
            self.foot_points[i].set_data([foot[0]], [foot[1]])
            self.foot_points[i].set_3d_properties([foot[2]])
            selected = i in self._selected_leg_indices()
            self.foot_points[i].set_markeredgecolor("yellow" if selected else "white")
            self.foot_points[i].set_markeredgewidth(2.0 if selected else 1.0)

            self.trails[i].append(tuple(foot))
            trail = np.array(self.trails[i])
            self.trail_lines[i].set_data(trail[:, 0], trail[:, 1])
            self.trail_lines[i].set_3d_properties(trail[:, 2])
            self.hip_labels[i].set_position((hip[0], hip[1]))
            self.hip_labels[i].set_3d_properties(hip[2] + 0.018)

            status = "OK" if self.ik_ok[i] else "OOB"
            info_rows.append(
                f"{leg.name} {status:3s} {leg.leg_type_name[:4]:4s} "
                f"foot=({foot[0]:+.3f},{foot[1]:+.3f},{foot[2]:+.3f}) "
                f"cmd=({np.degrees(hip_cmd):+6.1f},{np.degrees(knee_cmd):+6.1f})"
            )

        self._set_axes_limits()
        self.txt_info.set_text(self._info_text(info_rows))

    def _set_axes_limits(self):
        self.ax3d.set_xlim(-0.62, 0.62)
        self.ax3d.set_ylim(-0.40, 0.40)
        self.ax3d.set_zlim(-0.05, 0.48)

    def _info_text(self, rows):
        lines = [
            "KEYS",
            "space play/pause | n script | 0 all | 1-4 leg",
            "left/right foot x | up/down lift | w/s height",
            "q/e pitch | a/d roll | v view | r reset",
            "",
            f"mode     = {self.active_label}",
            f"playing  = {self.playing}",
            f"script   = {self.active_script or '(none)'}",
            f"host     = {self.host.active_gait_name()} / real leg_controller",
            f"body z   = {self.body_z:.3f} m",
            f"height   = {self.height:.3f} m",
            f"pitch    = {self.body_pitch_deg:+.1f} deg",
            f"roll     = {self.body_roll_deg:+.1f} deg",
            f"L1/L2    = {self.thigh:.3f} / {self.shin:.3f} m",
            "",
            "world foot position + motor command:",
            *rows,
            "",
            "real motor map:",
        ]
        for leg in self.legs:
            hip = leg.motors[LEG_ACT_HIP]
            knee = leg.motors[LEG_ACT_KNEE]
            wheel = leg.motors[LEG_ACT_WHEEL]
            lines.append(
                f"{leg.name} xdir={leg.foot_x_dir:+.0f} "
                f"hip {hip.name:8s} d={hip.direction:+d} "
                f"knee {knee.name:8s} d={knee.direction:+d} "
                f"wheel {wheel.name:8s} bus={wheel.can_bus} id=0x{wheel.can_id:X}"
            )
        return "\n".join(lines)

    # --------------------------------------------------------------- events

    def _on_key(self, ev):
        key = ev.key or ""
        if key in (" ", "space"):
            if self.playing:
                self.playing = False
            elif self.active_script is not None:
                self._load_script()
                self.playing = True
        elif key == "n":
            if self.scripts:
                self.active_script_index = (self.active_script_index + 1) % len(self.scripts)
                self.t_play = 0.0
                self._script_loaded = None
                self._clear_trails()
                if self.playing:
                    self._load_script()
        elif key in ("0", "1", "2", "3", "4"):
            idx = int(key)
            self.active_selection = ACTIVE_ALL if idx == 0 else ACTIVE_FIRST_LEG + idx - 1
        elif key == "right":
            self._adjust_foot(dx=+0.01)
        elif key == "left":
            self._adjust_foot(dx=-0.01)
        elif key == "up":
            self._adjust_foot(dz=+0.01)
        elif key == "down":
            self._adjust_foot(dz=-0.01)
        elif key == "w":
            self.height = min(0.38, self.height + 0.01)
            self.body_z = self.height
            self.host.set_stand_height(self.height)
        elif key == "s":
            self.height = max(0.08, self.height - 0.01)
            self.body_z = self.height
            self.host.set_stand_height(self.height)
        elif key == "q":
            self.body_pitch_deg = max(-15.0, self.body_pitch_deg - 1.0)
        elif key == "e":
            self.body_pitch_deg = min(+15.0, self.body_pitch_deg + 1.0)
        elif key == "a":
            self.body_roll_deg = max(-15.0, self.body_roll_deg - 1.0)
        elif key == "d":
            self.body_roll_deg = min(+15.0, self.body_roll_deg + 1.0)
        elif key == "v":
            self._view_index = (self._view_index + 1) % 4
            self._apply_view()
        elif key == "r":
            self._reset()
        else:
            return
        if key in ("left", "right", "up", "down") or (key in ("w", "s") and not self.playing):
            self._apply_manual_targets()
        self._dirty = True

    def _adjust_foot(self, dx=0.0, dz=0.0):
        self.playing = False
        for i in self._selected_leg_indices():
            self.foot_x[i] = float(np.clip(self.foot_x[i] + dx, -0.20, 0.20))
            self.foot_z[i] = float(np.clip(self.foot_z[i] + dz, -0.08, 0.10))

    def _apply_manual_targets(self):
        self.host.set_stand_height(self.height)
        self.host.apply_foot_targets(self.foot_x, self.foot_z)

    def _load_script(self):
        if self.active_script is None:
            return
        if self._script_loaded == self.active_script:
            return
        self.host.set_stand_height(self.height)
        self.host.play_script(self.active_script)
        self._script_loaded = self.active_script

    def _clear_trails(self):
        for trail in self.trails:
            trail.clear()

    def _reset(self):
        for i in range(len(self.legs)):
            self.foot_x[i] = 0.0
            self.foot_z[i] = 0.0
        self._clear_trails()
        self.t_play = 0.0
        self.playing = False
        self._script_loaded = None
        self.height = 0.25
        self.body_z = self.height
        self.body_pitch_deg = 0.0
        self.body_roll_deg = 0.0
        self.host = HostSystemSim()
        self._apply_manual_targets()

    def _apply_view(self):
        views = [(22, -54), (16, -90), (55, -45), (0, -90)]
        elev, azim = views[self._view_index]
        self.ax3d.view_init(elev=elev, azim=azim)

    # ------------------------------------------------------------- loop

    def step(self):
        now = time.monotonic()
        elapsed = now - self._last_step
        if elapsed < self.dt:
            return True
        self._last_step = now

        if self.playing and self.active_script is not None:
            self._load_script()
            self.host.step(elapsed)
            self.t_play += elapsed
            self._dirty = True

        if self._dirty:
            self._draw_all()
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
            self._dirty = False
        return True

    def start_timer(self):
        if self._timer is not None:
            return
        self._timer = self.fig.canvas.new_timer(interval=20)
        self._timer.add_callback(self.step)
        self._timer.start()


def main():
    app = LegVizApp()
    app.start_timer()
    plt.show()


if __name__ == "__main__":
    main()
