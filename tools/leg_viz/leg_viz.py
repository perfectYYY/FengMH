"""leg_viz — single-leg interactive GUI for FengMH IK / scripts.

Drag the foot marker in the side view to see live IK + FK reconstruction.
Sliders adjust thigh/shin lengths and stand height. Radio buttons switch
LEG_TYPE_ORIGINAL <-> LEG_TYPE_MIRROR. Builtin scripts (wave_up_down etc.)
can be played back; the foot trace is overlaid on the leg view.

All math comes from the C library libfengmh_sim.dylib via sim_bridge.py,
so what you see here is byte-for-byte what the firmware will compute.
"""
from __future__ import annotations

import sys
from collections import deque
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button, RadioButtons, Slider

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from tools.leg_viz.sim_bridge import (  # noqa: E402
    LEG_DIM_DEFAULT, LEG_TYPE_MIRROR, LEG_TYPE_ORIGINAL,
    list_builtin_scripts, make_dim, sample_script, script_duration,
    solve_fk, solve_ik,
)


class LegVizApp:
    def __init__(self):
        self.thigh = LEG_DIM_DEFAULT.thigh_length
        self.shin = LEG_DIM_DEFAULT.shin_length
        self.height = 0.20
        self.leg_type = LEG_TYPE_ORIGINAL

        # foot target in IK coords: x forward, z down (z is the *displacement*
        # added to height). Initial pose = straight down at -height.
        self.foot_x = 0.0
        self.foot_z = -0.05
        self.last_valid_thetas = (np.pi / 2, np.pi / 2)
        self.ik_ok = True

        # script playback state
        self.scripts = list_builtin_scripts()
        self.active_script = None
        self.playing = False
        self.t_play = 0.0
        self.dt = 0.05  # 20 Hz redraw
        self.trail = deque(maxlen=200)

        self._dragging = False

        self._build_figure()
        self._draw_all()

    # ------------------------------------------------------------------ UI

    def _build_figure(self):
        self.fig = plt.figure(figsize=(12, 7))
        self.fig.canvas.manager.set_window_title("FengMH leg_viz")

        # main leg side view (left)
        self.ax_leg = self.fig.add_axes([0.05, 0.30, 0.50, 0.65])
        self.ax_leg.set_aspect("equal")
        self.ax_leg.set_xlim(-0.45, 0.45)
        self.ax_leg.set_ylim(-0.50, 0.10)
        self.ax_leg.set_xlabel("x (forward, m)")
        self.ax_leg.set_ylabel("y (up, m)  [= -z]")
        self.ax_leg.grid(True, alpha=0.3)
        self.ax_leg.axhline(0, color="gray", lw=0.5)
        self.ax_leg.axvline(0, color="gray", lw=0.5)
        self.ax_leg.set_title("drag the red foot dot")

        (self.line_leg,) = self.ax_leg.plot([], [], "o-", lw=3, color="C0")
        (self.foot_dot,) = self.ax_leg.plot([], [], "o", ms=12, color="red")
        (self.trail_line,) = self.ax_leg.plot([], [], "-", lw=1, color="orange", alpha=0.6)
        self.txt_status = self.ax_leg.text(
            0.02, 0.97, "", transform=self.ax_leg.transAxes,
            va="top", fontsize=10, family="monospace",
            bbox=dict(boxstyle="round", fc="white", ec="gray", alpha=0.8),
        )

        # joint-angle time series (right)
        self.ax_joints = self.fig.add_axes([0.62, 0.55, 0.35, 0.40])
        self.ax_joints.set_xlabel("sample")
        self.ax_joints.set_ylabel("rad")
        self.ax_joints.set_title("joint angles (live trace)")
        self.ax_joints.grid(True, alpha=0.3)
        self.t1_hist = deque(maxlen=400)
        self.t2_hist = deque(maxlen=400)
        (self.line_t1,) = self.ax_joints.plot([], [], lw=1.5, label="theta1 (hip)")
        (self.line_t2,) = self.ax_joints.plot([], [], lw=1.5, label="theta2 (knee)")
        self.ax_joints.legend(loc="upper right", fontsize=9)
        self.ax_joints.set_ylim(-np.pi, np.pi)

        # info panel (below joint trace)
        self.ax_info = self.fig.add_axes([0.62, 0.30, 0.35, 0.20])
        self.ax_info.axis("off")
        self.txt_info = self.ax_info.text(
            0.0, 1.0, "", transform=self.ax_info.transAxes,
            va="top", family="monospace", fontsize=10,
        )

        # sliders
        self.ax_thigh = self.fig.add_axes([0.10, 0.20, 0.45, 0.025])
        self.s_thigh = Slider(self.ax_thigh, "thigh L1 (m)", 0.05, 0.40,
                              valinit=self.thigh)
        self.ax_shin = self.fig.add_axes([0.10, 0.16, 0.45, 0.025])
        self.s_shin = Slider(self.ax_shin, "shin L2 (m)", 0.05, 0.40,
                             valinit=self.shin)
        self.ax_height = self.fig.add_axes([0.10, 0.12, 0.45, 0.025])
        self.s_height = Slider(self.ax_height, "stand height (m)", 0.0, 0.40,
                               valinit=self.height)
        for s in (self.s_thigh, self.s_shin, self.s_height):
            s.on_changed(self._on_slider)

        # leg-type radio
        self.ax_radio = self.fig.add_axes([0.62, 0.10, 0.12, 0.15])
        self.radio_type = RadioButtons(
            self.ax_radio, ("ORIGINAL", "MIRROR"), active=0,
        )
        self.radio_type.on_clicked(self._on_leg_type)
        self.ax_radio.set_title("leg_type", fontsize=10)

        # script radio + play/pause buttons
        self.ax_script = self.fig.add_axes([0.76, 0.10, 0.13, 0.15])
        labels = ("(none)", *self.scripts)
        self.radio_script = RadioButtons(self.ax_script, labels, active=0)
        self.radio_script.on_clicked(self._on_script)
        self.ax_script.set_title("script", fontsize=10)

        self.ax_play = self.fig.add_axes([0.90, 0.18, 0.07, 0.05])
        self.btn_play = Button(self.ax_play, "Play")
        self.btn_play.on_clicked(self._on_play)

        self.ax_reset = self.fig.add_axes([0.90, 0.11, 0.07, 0.05])
        self.btn_reset = Button(self.ax_reset, "Reset")
        self.btn_reset.on_clicked(self._on_reset)

        # mouse
        self.fig.canvas.mpl_connect("button_press_event", self._on_press)
        self.fig.canvas.mpl_connect("button_release_event", self._on_release)
        self.fig.canvas.mpl_connect("motion_notify_event", self._on_motion)

        # animation tick (drives both redraw and script playback)
        self.anim = FuncAnimation(
            self.fig, self._tick, interval=int(self.dt * 1000),
            blit=False, cache_frame_data=False,
        )

    # --------------------------------------------------------------- math

    def _current_dim(self):
        return make_dim(thigh=self.thigh, shin=self.shin)

    def _ik_now(self):
        return solve_ik(self.foot_x, self.foot_z,
                        dim=self._current_dim(), height=self.height,
                        leg_type=self.leg_type)

    def _draw_all(self):
        dim = self._current_dim()
        ik = self._ik_now()
        if ik is None:
            self.ik_ok = False
            t1, t2 = self.last_valid_thetas
        else:
            self.ik_ok = True
            t1, t2 = ik
            self.last_valid_thetas = (t1, t2)

        # FK to draw the linkage from joint angles (so users see what IK believes)
        knee_x = self.thigh * np.cos(t1)
        knee_z = self.thigh * np.sin(t1)
        fk_x, fk_z = solve_fk(t1, t2, dim=dim)

        xs = [0.0, knee_x, fk_x]
        ys = [0.0, -knee_z, -fk_z]  # screen up = -z
        self.line_leg.set_data(xs, ys)
        self.foot_dot.set_data(
            [self.foot_x], [-(self.foot_z + self.height)],
        )
        self.foot_dot.set_color("red" if self.ik_ok else "darkred")
        self.foot_dot.set_markeredgecolor("white" if self.ik_ok else "yellow")

        # workspace circles
        if not hasattr(self, "_ws_outer"):
            self._ws_outer, = self.ax_leg.plot([], [], "--", color="gray", lw=0.6)
            self._ws_inner, = self.ax_leg.plot([], [], "--", color="gray", lw=0.6)
        ang = np.linspace(0, 2 * np.pi, 64)
        rmax = self.thigh + self.shin
        rmin = abs(self.thigh - self.shin)
        self._ws_outer.set_data(rmax * np.cos(ang), rmax * np.sin(ang))
        self._ws_inner.set_data(rmin * np.cos(ang), rmin * np.sin(ang))

        # trail
        if self.trail:
            tx, ty = zip(*self.trail)
            self.trail_line.set_data(tx, ty)
        else:
            self.trail_line.set_data([], [])

        # joint trace
        self.t1_hist.append(t1)
        self.t2_hist.append(t2)
        n = len(self.t1_hist)
        idx = np.arange(n)
        self.line_t1.set_data(idx, list(self.t1_hist))
        self.line_t2.set_data(idx, list(self.t2_hist))
        self.ax_joints.set_xlim(max(0, n - 400), max(400, n))

        ok = "OK " if self.ik_ok else "OOB"
        self.txt_status.set_text(
            f"foot  x={self.foot_x:+.3f}  z={self.foot_z:+.3f}  h={self.height:.3f}\n"
            f"IK    {ok}   t1={np.degrees(t1):+7.2f}°   t2={np.degrees(t2):+7.2f}°\n"
            f"FK    x={fk_x:+.3f}  z={fk_z:+.3f}"
        )
        self.txt_info.set_text(
            f"thigh L1 = {self.thigh:.3f} m\n"
            f"shin  L2 = {self.shin:.3f} m\n"
            f"height   = {self.height:.3f} m\n"
            f"leg_type = {'MIRROR' if self.leg_type else 'ORIGINAL'}\n"
            f"script   = {self.active_script or '(none)'}\n"
            f"playing  = {self.playing}"
        )

    # --------------------------------------------------------------- evt

    def _on_slider(self, _):
        self.thigh = float(self.s_thigh.val)
        self.shin = float(self.s_shin.val)
        self.height = float(self.s_height.val)

    def _on_leg_type(self, label):
        self.leg_type = LEG_TYPE_MIRROR if label == "MIRROR" else LEG_TYPE_ORIGINAL

    def _on_script(self, label):
        if label == "(none)":
            self.active_script = None
            self.playing = False
        else:
            self.active_script = label
            self.t_play = 0.0
        self.trail.clear()

    def _on_play(self, _):
        if self.active_script is None:
            return
        self.playing = not self.playing
        self.btn_play.label.set_text("Pause" if self.playing else "Play")

    def _on_reset(self, _):
        self.foot_x, self.foot_z = 0.0, -0.05
        self.trail.clear()
        self.t1_hist.clear()
        self.t2_hist.clear()
        self.t_play = 0.0
        self.playing = False
        self.btn_play.label.set_text("Play")

    def _on_press(self, ev):
        if ev.inaxes is not self.ax_leg or ev.button != 1:
            return
        self._dragging = True
        self._set_foot_from_screen(ev.xdata, ev.ydata)

    def _on_release(self, ev):
        self._dragging = False

    def _on_motion(self, ev):
        if not self._dragging or ev.inaxes is not self.ax_leg:
            return
        if ev.xdata is None or ev.ydata is None:
            return
        self._set_foot_from_screen(ev.xdata, ev.ydata)

    def _set_foot_from_screen(self, sx, sy):
        # screen y = -(z + height) → z = -sy - height
        self.foot_x = float(sx)
        self.foot_z = float(-sy - self.height)

    # ------------------------------------------------------------- tick

    def _tick(self, _frame):
        if self.playing and self.active_script is not None:
            out = sample_script(self.active_script, self.t_play)
            if out is not None:
                # FL leg's hip_rad/knee_rad carry foot displacement (dx, dz)
                self.foot_x = out.leg[0].hip_rad
                self.foot_z = out.leg[0].knee_rad
                # FL is MIRROR per leg_ik.c mapping
                self.leg_type = LEG_TYPE_MIRROR
                self.radio_type.set_active(1)
                self.trail.append((self.foot_x, -(self.foot_z + self.height)))
            self.t_play += self.dt
            dur = script_duration(self.active_script)
            if dur > 0 and self.t_play > dur:
                self.t_play = 0.0  # loop
        self._draw_all()


def main():
    app = LegVizApp()
    plt.show()


if __name__ == "__main__":
    main()
