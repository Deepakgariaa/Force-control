#!/usr/bin/env python3
"""
AR523: Robot Manipulators -- Assignment 2
Force-Controlled Writing & Haptic Material Identification
"""

import argparse
import os
import time

import numpy as np
import pybullet as p
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from utils import *


# ==============================================================================
# PART 1: Trajectory Generation
# ==============================================================================

def generate_pattern(shape, scale=0.1, n_points=120):
    """
    TODO 1: Implement shape trajectory generation.
    Returns a list of strokes (each stroke is a list of (x,y) tuples).
    """
    r = scale / 2.0                                   # every shape fits in a scale x scale box
    if shape == "circle":
        t = np.linspace(0, 2 * np.pi, n_points)
        x, y = r * np.cos(t), r * np.sin(t)
    elif shape == "spiral":                           # Archimedean spiral, 3 turns
        t = np.linspace(0, 6 * np.pi, 3 * n_points)
        rho = r * t / (6 * np.pi)
        x, y = rho * np.cos(t), rho * np.sin(t)
    elif shape == "infinity":                         # lemniscate of Gerono
        t = np.linspace(0, 2 * np.pi, n_points)
        x, y = r * np.sin(t), r * np.sin(t) * np.cos(t)
    elif shape == "heart":
        t = np.linspace(0, 2 * np.pi, n_points)
        x = r * np.sin(t) ** 3
        y = (r / 16) * (13 * np.cos(t) - 5 * np.cos(2 * t) - 2 * np.cos(3 * t) - np.cos(4 * t))
    else:
        raise ValueError(f"generate_pattern: unknown shape {shape!r}")
    return [list(zip(x, y))]


# ==============================================================================
# BONUS: Cursive writing
# ==============================================================================

# Control points of joined-up lowercase letters on a unit box (baseline y=0,
# small letters reach y=0.5, tall letters y=1). Each letter starts where the
# previous one ended, so a word is one continuous pen-down stroke.
# Marks drawn after the word (cross of the t) are listed separately.
CURSIVE_LETTERS = {
    "r": dict(w=0.45, pts=[(0.0, 0.05), (0.12, 0.45), (0.2, 0.42), (0.3, 0.46),
                           (0.26, 0.2), (0.3, 0.02), (0.45, 0.1)]),
    "o": dict(w=0.5, pts=[(0.3, 0.46), (0.12, 0.44), (0.03, 0.25), (0.1, 0.03),
                          (0.27, 0.02), (0.37, 0.22), (0.32, 0.46), (0.4, 0.4), (0.5, 0.38)]),
    "b": dict(w=0.45, pts=[(0.06, 0.45), (0.17, 0.85), (0.13, 1.0), (0.04, 0.85),
                           (0.03, 0.4), (0.06, 0.04), (0.22, 0.0), (0.34, 0.18),
                           (0.3, 0.4), (0.15, 0.38), (0.3, 0.37), (0.45, 0.38)]),
    "t": dict(w=0.4, pts=[(0.1, 0.25), (0.2, 0.8), (0.17, 0.4), (0.18, 0.06),
                          (0.3, 0.0), (0.4, 0.12)],
              marks=[[(0.04, 0.55), (0.36, 0.55)]]),
}


def catmull_rom(points, samples_per_seg=12):
    """Smooth curve passing through every control point (uniform Catmull-Rom)."""
    P = np.asarray(points, dtype=float)
    P = np.vstack([P[0], P, P[-1]])                   # repeat ends so the curve reaches them
    out = []
    for i in range(1, len(P) - 2):
        p0, p1, p2, p3 = P[i - 1], P[i], P[i + 1], P[i + 2]
        for t in np.linspace(0, 1, samples_per_seg, endpoint=False):
            out.append(0.5 * (2 * p1 + (p2 - p0) * t + (2 * p0 - 5 * p1 + 4 * p2 - p3) * t**2
                              + (3 * p1 - p0 - 3 * p2 + p3) * t**3))
    out.append(P[-2])
    return out


def get_cursive_strokes(word, height=LETTER_HEIGHT):
    """One continuous spline for the word, then the extra marks (t-crosses)."""
    word = word.lower()
    pts, marks, cursor = [], [], 0.0
    for ch in word:
        if ch not in CURSIVE_LETTERS:
            raise KeyError(f"get_cursive_strokes: no cursive entry for {ch!r} "
                           f"(available: {''.join(CURSIVE_LETTERS)})")
        L = CURSIVE_LETTERS[ch]
        pts += [(cursor + px, py) for px, py in L["pts"]]
        marks += [[(cursor + px, py) for px, py in m] for m in L.get("marks", [])]
        cursor += L["w"]
    strokes = [catmull_rom(pts)] + marks
    # scale to letter height and centre horizontally, like get_word_strokes()
    return [[((x - cursor / 2) * height, y * height) for x, y in s] for s in strokes]


# ==============================================================================
# PART 2: Cartesian Impedance Control Law
# ==============================================================================

class CartesianImpedanceController:

    def __init__(self, arm):
        self.arm = arm

    def step(self, pos_des, quat_des, K_trans, K_rot, damping_ratio=1.0, vel_des=None):
        # ----------------------------------------------------------------------
        # TODO 2: Implement the Cartesian impedance torque law.
        # Inputs:
        #   pos_des: (3,) desired position
        #   quat_des: (4,) desired orientation quaternion
        #   K_trans: (3,) translational stiffness (x,y,z)
        #   K_rot: (3,) rotational stiffness (x,y,z)
        #   damping_ratio: scalar
        #   vel_des: (3,) desired velocity (optional, default zero)
        # Returns:
        #   tau: (7,) joint torques
        #   info: dict for logging (provided in solution)
        # ----------------------------------------------------------------------
        q, qdot = self.arm.get_joint_state()
        pos, quat, vel, omega = self.arm.get_tool_pose()
        vel_des = np.zeros(3) if vel_des is None else np.asarray(vel_des)

        # Errors  e_x = x_d - x,  e_dx = xd_dot - x_dot  (orientation via quaternion error)
        e_pos = np.asarray(pos_des) - pos
        e_vel = vel_des - vel
        e_rot = -quat_to_axis_angle_error(quat, quat_des)    # utils returns (current - desired)
        e_omega = -omega

        # Damping from the damping ratio:  D = 2 * zeta * sqrt(K * m)
        D_trans = 2 * damping_ratio * np.sqrt(np.asarray(K_trans) * REFLECTED_MASS_TRANS)
        D_rot = 2 * damping_ratio * np.sqrt(np.asarray(K_rot) * REFLECTED_INERTIA_ROT)

        # Spring + damper wrench:  F_c = K (x_d - x) + D (xd_dot - x_dot)
        F = np.concatenate([K_trans * e_pos + D_trans * e_vel,
                            K_rot * e_rot + D_rot * e_omega])

        # tau = J^T(q) F_c + C(q, q_dot) q_dot + g(q)
        J = self.arm.jacobian(q, qdot)
        zeros = [0.0] * self.arm.n_joints
        tau_g = self.arm.gravity_compensation(q)
        tau_c = (np.array(p.calculateInverseDynamics(self.arm.robot_id, list(q), list(qdot), zeros,
                                                    physicsClientId=self.arm.cid))
                 - np.array(p.calculateInverseDynamics(self.arm.robot_id, list(q), zeros, zeros,
                                                      physicsClientId=self.arm.cid)))
        tau = J.T @ F + tau_c + tau_g
        return tau, dict(F=F, e_pos=e_pos, e_rot=e_rot)


# ==============================================================================
# PART 3: Admittance Control Outer Loop
# ==============================================================================

class AdmittanceController:
    def __init__(self, mass=ADMITTANCE_MASS, damping=ADMITTANCE_DAMPING):
        self.M = mass
        self.B = damping
        self.xc = 0.0
        self.xc_dot = 0.0

    def reset(self):
        self.xc = 0.0
        self.xc_dot = 0.0

    def step(self, f_measured, f_desired, dt, xc_limit=0.03, xc_dot_limit=0.08):
        """
        TODO 3: Implement admittance mass-damper outer loop step.
        Returns: float (self.xc)
        """
        # M_v xc_ddot + B_v xc_dot = F_meas - F_des
        # (xc is measured UP, away from the surface: too much force -> move up)
        xc_ddot = (f_measured - f_desired - self.B * self.xc_dot) / self.M
        self.xc_dot = np.clip(self.xc_dot + xc_ddot * dt, -xc_dot_limit, xc_dot_limit)
        self.xc = np.clip(self.xc + self.xc_dot * dt, -xc_limit, xc_limit)
        return float(self.xc)

ADMITTANCE_NOMINAL_PENETRATION = 0.015
IMPEDANCE_KI = 5.0      # 1/s, integral gain on the force error (impedance writing)
# Impedance writing: compliant (150 N/m) only along the surface normal; stiff in
# the plane so the ~1 N friction force does not bend the pen off the path.
GAINS_IMPEDANCE_WRITING = dict(GAINS_IMPEDANCE, K_trans=np.array([600., 600., 150.]))


# ==============================================================================
# LIVE VISUALIZATION (TODO 8)
# ==============================================================================

class LiveDashboard:
    def __init__(self, mode="writing"):
        """
        TODO 7: Implement a real-time matplotlib dashboard using plt.ion().
        """
        # The module uses the non-interactive Agg backend for saving figures;
        # switch to a window backend when one is available (falls back to Agg
        # on headless machines, where the dashboard is only saved to file).
        try:
            plt.switch_backend("TkAgg")
        except Exception:
            pass
        plt.ion()
        self.mode = mode
        self.last_update = 0.0
        self.t_data, self.f_act, self.f_des = [], [], []
        if mode == "writing":
            self.x_act, self.y_act, self.x_des, self.y_des = [], [], [], []
            self.fig, (self.ax_xy, self.ax_f) = plt.subplots(1, 2, figsize=(12, 4.5),
                                                             gridspec_kw=dict(width_ratios=[1, 1.4]))
            self.line_xy_des, = self.ax_xy.plot([], [], color=GREY, ls="--", lw=1.2, label="desired")
            self.line_xy_act, = self.ax_xy.plot([], [], color=NAVY, lw=1.6, label="actual")
            self.ax_xy.set_aspect("equal", adjustable="datalim")
            self.ax_xy.set(xlabel="x (m)", ylabel="y (m)", title="Path (pen down)")
            self.ax_xy.legend(fontsize=9, frameon=False)
            self.line_f_des, = self.ax_f.plot([], [], color=GREY, ls="--", lw=1.5, label="desired Fz")
            self.line_f_act, = self.ax_f.plot([], [], color=RED, lw=1.2, label="measured Fz")
            self.ax_f.set(xlabel="Time (s)", ylabel="Force (N)", title="Contact force")
            self.ax_f.legend(fontsize=9, frameon=False, loc="upper right")
        else:
            self.depths = []
            self.fig, self.ax = plt.subplots(figsize=(6.5, 4.5))
            self.line_fd, = self.ax.plot([], [], color=RED, lw=1.6, label="measured")
            self.line_fit, = self.ax.plot([], [], color=GREY, ls="--", lw=1.5, label="fit F = K̂·d")
            self.ax.set(xlabel="Commanded depth d (m)", ylabel="Force (N)", title="Loading curve")
            self.ax.legend(fontsize=9, frameon=False)
        self.fig.tight_layout()
        plt.show(block=False)

    def update_writing(self, t, pos_act, pos_des, f_act, f_des):
        self.t_data.append(t)
        if f_des > 0:
            self.x_act.append(pos_act[0]); self.y_act.append(pos_act[1])
            self.x_des.append(pos_des[0]); self.y_des.append(pos_des[1])
        else:
            self.x_act.append(np.nan); self.y_act.append(np.nan)
            self.x_des.append(np.nan); self.y_des.append(np.nan)
        self.f_act.append(f_act[2])
        self.f_des.append(f_des)
        self._draw()

    def update_material(self, depth, f_act, k_hat=None):
        self.depths.append(depth)
        self.f_act.append(f_act)
        self.k_hat = k_hat
        self._draw(force=k_hat is not None)

    def _draw(self, force=False):
        # Redraw at most every 50 ms; the timestamp is taken AFTER drawing so a
        # slow redraw cannot end up running on every 1 ms simulation step.
        if not force and time.time() - self.last_update < 0.05:
            return
        if self.mode == "writing":
            self.line_xy_act.set_data(self.x_act, self.y_act)
            self.line_xy_des.set_data(self.x_des, self.y_des)
            self.line_f_act.set_data(self.t_data, self.f_act)
            self.line_f_des.set_data(self.t_data, self.f_des)
            axes = [self.ax_xy, self.ax_f]
        else:
            self.line_fd.set_data(self.depths, self.f_act)
            if getattr(self, "k_hat", None) is not None and self.depths:
                d_max = max(self.depths)
                self.line_fit.set_data([0, d_max], [0, self.k_hat * d_max])
            axes = [self.ax]
        for ax in axes:
            ax.relim()
            ax.autoscale_view()
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()
        self.last_update = time.time()

    def close(self, save_path=None):
        self._draw(force=True)
        if save_path:
            self.fig.savefig(save_path, dpi=150, facecolor="white")
        plt.ioff()
        plt.close(self.fig)


# ==============================================================================
# WRITING & PROBING RUNNERS
# ==============================================================================

def build_stroke_plan(world, letter_origin_xy, word=None, pattern=None, cursive=False):
    strokes = []
    if word:
        strokes.extend(get_cursive_strokes(word) if cursive else get_word_strokes(word))
    if pattern:
        pattern_strokes = generate_pattern(pattern)
        offset = -0.08 if word else 0.0          # 8 cm below the word (+0.5 m in x was off the table)
        pattern_strokes = [[(px, py + offset) for (px, py) in seg] for seg in pattern_strokes]
        strokes.extend(pattern_strokes)
        
    # Each pen-down entry is a whole polyline, drawn in one smooth motion.
    # It is cut only at sharp corners (> 30 deg), where the pen must stop.
    plan = []
    last_xy = world.pre_contact_pos[:2]
    for stroke in strokes:
        pts = np.array(letter_origin_xy) + np.array(stroke)
        pts = pts[np.r_[True, np.linalg.norm(np.diff(pts, axis=0), axis=1) > 1e-9]]   # drop repeated points
        d = np.diff(pts, axis=0)
        d /= np.linalg.norm(d, axis=1, keepdims=True)
        corners = np.where(np.sum(d[:-1] * d[1:], axis=1) < np.cos(np.deg2rad(30)))[0] + 1
        plan.append(("up", last_xy, pts[0]))
        for piece in np.split(np.arange(len(pts)), corners):
            idx = np.r_[piece, piece[-1] + 1] if piece[-1] + 1 < len(pts) else piece
            if len(idx) > 1:
                plan.append(("down", pts[idx[0]], pts[idx]))
        last_xy = pts[-1]
    return plan


class PathTrajectory:
    """Moves along a polyline with min-jerk timing of the arc length s(t)."""

    def __init__(self, pts, speed=STROKE_SPEED):
        self.pts = np.asarray(pts, dtype=float)
        self.cum = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(self.pts, axis=0), axis=1))]
        self.L = self.cum[-1]
        self.T = max(self.L / speed, 0.15)

    def eval(self, t):
        tau = np.clip(t / self.T, 0.0, 1.0)
        s = self.L * (10 * tau**3 - 15 * tau**4 + 6 * tau**5)
        sdot = self.L * (30 * tau**2 - 60 * tau**3 + 30 * tau**4) / self.T
        i = int(np.clip(np.searchsorted(self.cum, s, side="right") - 1, 0, len(self.pts) - 2))
        u = (self.pts[i + 1] - self.pts[i]) / (self.cum[i + 1] - self.cum[i])   # unit direction
        return self.pts[i] + u * (s - self.cum[i]), u * sdot


def run_writing_task(world, ctrl, controller_mode, f_desired, gui=False, save_dir="outputs", tag="writing", word="Robot", pattern=None, live=False, cursive=False):
    plan = build_stroke_plan(world, world.surface_center_xy, word=word, pattern=pattern, cursive=cursive)
    adm = AdmittanceController()

    drawer = TrajectoryDrawer(world.cid, color=(0, 0, 0)) if gui else None
    dashboard = LiveDashboard(mode="writing") if live else None
    logger = TrialLogger()
    logger.start_timer()

    first_xy = plan[0][2]
    lift_pos = np.array([first_xy[0], first_xy[1], world.surface_top_z + PEN_LIFT_HEIGHT])
    traj_in = MinJerkTrajectory(world.pre_contact_pos, lift_pos, APPROACH_DURATION)

    t = 0.0
    for step in range(int(APPROACH_DURATION / DT)):
        pos_des, vel_des = traj_in.eval(t)
        tau, _ = ctrl.step(pos_des, world.nominal_quat, GAINS_PEN_UP["K_trans"], GAINS_PEN_UP["K_rot"], GAINS_PEN_UP["damping_ratio"], vel_des)
        world.arm.apply_joint_torques(tau)
        p.stepSimulation(physicsClientId=world.cid)
        world.arm.clamp_joint_velocities()
        tool_pos, _, _, _ = world.arm.get_tool_pose()
        logger.record(t, tool_pos, pos_des, force=np.zeros(3), force_des=0.0)
        if dashboard:
            dashboard.update_writing(t, tool_pos, pos_des, np.zeros(3), 0.0)
        t += DT

    prev_kind = None
    for kind, xy_from, xy_to in plan:
        if kind == "up":
            p_from = np.array([xy_from[0], xy_from[1], world.surface_top_z + PEN_LIFT_HEIGHT])
            p_to = np.array([xy_to[0], xy_to[1], world.surface_top_z + PEN_LIFT_HEIGHT])
            traj = MinJerkTrajectory(p_from, p_to, PEN_UP_DURATION)
            for step in range(int(PEN_UP_DURATION / DT)):
                pos_des, vel_des = traj.eval(step * DT)
                tau, _ = ctrl.step(pos_des, world.nominal_quat, GAINS_PEN_UP["K_trans"], GAINS_PEN_UP["K_rot"], GAINS_PEN_UP["damping_ratio"], vel_des)
                world.arm.apply_joint_torques(tau)
                p.stepSimulation(physicsClientId=world.cid)
                world.arm.clamp_joint_velocities()
                tool_pos, _, _, _ = world.arm.get_tool_pose()
                logger.record(t, tool_pos, pos_des, force=np.zeros(3), force_des=0.0)
                if dashboard:
                    dashboard.update_writing(t, tool_pos, pos_des, np.zeros(3), 0.0)
                t += DT
            adm.reset()
        else:
            if prev_kind != "down":
                adm.reset()
                z_start = world.surface_top_z + PEN_LIFT_HEIGHT
                z_end = world.surface_top_z + TOOL_RADIUS
                traj_app = MinJerkTrajectory(np.array([xy_from[0], xy_from[1], z_start]), np.array([xy_from[0], xy_from[1], z_end]), 0.15)
                for step in range(int(0.15 / DT)):
                    pos_des, vel_des = traj_app.eval(step * DT)
                    tau, _ = ctrl.step(pos_des, world.nominal_quat, GAINS_PEN_UP["K_trans"], GAINS_PEN_UP["K_rot"], GAINS_PEN_UP["damping_ratio"], vel_des)
                    world.arm.apply_joint_torques(tau)
                    p.stepSimulation(physicsClientId=world.cid)
                    world.arm.clamp_joint_velocities()
                    tool_pos, _, _, _ = world.arm.get_tool_pose()
                    logger.record(t, tool_pos, pos_des, force=np.zeros(3), force_des=0.0)
                    if dashboard:
                        dashboard.update_writing(t, tool_pos, pos_des, np.zeros(3), 0.0)
                    t += DT
                f_int = 0.0
                touched = False     # the stroke clock waits until the pen is on the surface
                t_land = t

            xy_traj = PathTrajectory(xy_to)          # xy_to is the whole pen-down polyline
            duration = xy_traj.T
            step = 0
            while step < int(duration / DT):
                xy_pos, xy_vel = xy_traj.eval(step * DT)
                if not touched:
                    xy_vel = np.zeros(2)
                tool_pos, _, _, _ = world.arm.get_tool_pose()
                force, _, _ = get_contact_wrench(world.cid, world.arm.tool_id, world.static_ids, tool_pos)
                f_meas_z = force[2]

                if controller_mode == "admittance":
                    xc = adm.step(f_meas_z, f_desired, DT)
                    z_des = world.surface_top_z + TOOL_RADIUS - ADMITTANCE_NOMINAL_PENETRATION + xc
                    gains = GAINS_INNER_STIFF
                else:
                    # ----------------------------------------------------------
                    # TODO 2 (continued): Impedance Writing
                    # Write the logic to achieve the desired 5N contact force.
                    # ----------------------------------------------------------
                    # A spring pushes with F = K_z * (z_d - z): put the virtual
                    # target F_d / K_z below the surface. The surface itself
                    # gives a little, so a slow integral of the force error
                    # (only while touching, so it cannot wind up in the air)
                    # removes the remaining steady-state error.
                    gains = GAINS_IMPEDANCE_WRITING
                    if f_meas_z > 0:
                        f_int += IMPEDANCE_KI * (f_desired - f_meas_z) * DT
                    z_des = world.surface_top_z + TOOL_RADIUS - (f_desired + f_int) / gains["K_trans"][2]

                pos_des = np.array([xy_pos[0], xy_pos[1], z_des])
                vel_des = np.array([xy_vel[0], xy_vel[1], 0.0])
                tau, _ = ctrl.step(pos_des, world.nominal_quat, gains["K_trans"], gains["K_rot"], gains["damping_ratio"], vel_des)
                world.arm.apply_joint_torques(tau)
                p.stepSimulation(physicsClientId=world.cid)
                world.arm.clamp_joint_velocities()

                tool_pos, _, _, _ = world.arm.get_tool_pose()
                force, torque, _ = get_contact_wrench(world.cid, world.arm.tool_id, world.static_ids, tool_pos)
                
                # Writing (and the 5 N force target) starts at first contact;
                # until then the pen is still landing and stays at the stroke start.
                touched = touched or force[2] > 0 or t - t_land > 1.0     # (1 s safety timeout)
                f_des_now = f_desired if touched else 0.0
                logger.record(t, tool_pos, pos_des, force=force, force_des=f_des_now, torque=torque)
                if dashboard:
                    dashboard.update_writing(t, tool_pos, pos_des, force, f_des_now)
                if drawer is not None and touched and step % 3 == 0:
                    drawer.update(tool_pos)
                if touched:
                    step += 1
                t += DT
        prev_kind = kind
    
    logger.stop_timer()
    os.makedirs(save_dir, exist_ok=True)
    if dashboard: dashboard.close(save_path=os.path.join(save_dir, f"{tag}_dashboard.png"))

    arr = logger.arrays()
    
    # --------------------------------------------------------------------------
    # TODO 4: Compute Metrics
    # Compute force tracking RMSE (only when f_des > 0) and path smoothness (mean squared jerk).
    # Store them in variables named `rmse` and `smoothness`.
    # --------------------------------------------------------------------------
    pen_down = arr["force_des"] > 0
    f_err = arr["force"][pen_down, 2] - arr["force_des"][pen_down]
    rmse = float(np.sqrt(np.mean(f_err ** 2)))                        # RMSE = sqrt(mean((F - F_d)^2))
    vel = np.gradient(arr["pos"], DT, axis=0)
    acc = np.gradient(vel, DT, axis=0)
    jerk = np.gradient(acc, DT, axis=0)
    smoothness = float(np.mean(np.sum(jerk[pen_down] ** 2, axis=1)))   # mean ||d^3x/dt^3||^2 while writing
    
    plot_force_tracking(arr, os.path.join(save_dir, f"{tag}_force_tracking.png"), title=f"{tag}: Force Tracking While Writing")
    plot_xy_path(arr, os.path.join(save_dir, f"{tag}_xy_path.png"), title=f"{tag}: Traced Path (top-down)")
    
    summary = dict(force_tracking_rmse_N=rmse, path_smoothness_jerk2=smoothness, wall_clock_time_s=logger.wall_clock_time(), sim_time_s=float(arr["t"][-1]))
    save_json(os.path.join(save_dir, f"{tag}_summary.json"), summary)
    print(f"[{tag}] force RMSE = {rmse:.3f} N")
    return summary, arr


def probe_material(world, ctrl, pad_id, pad_top_z, xy, save_dir, tag, fixed_penetration=0.010, settle_duration=3.0, live=False):
    gains = dict(K_trans=np.array([600., 600., 600.]), K_rot=np.array([30., 30., 30.]), damping_ratio=1.8)
    just_touching_z = pad_top_z + TOOL_RADIUS
    target = np.array([xy[0], xy[1], just_touching_z - fixed_penetration])

    hover = np.array([xy[0], xy[1], just_touching_z + 0.05])
    q_hover = world.arm.inverse_kinematics(hover, world.nominal_quat)
    world.arm.set_joint_positions(q_hover)
    world.arm.sync_tool_pose()

    traj = MinJerkTrajectory(hover, target, APPROACH_DURATION)
    dashboard = LiveDashboard(mode="material") if live else None
    logger = TrialLogger()
    t = 0.0
    for step in range(int((APPROACH_DURATION + settle_duration) / DT)):
        pos_des, _ = traj.eval(t)
        tau, _ = ctrl.step(pos_des, world.nominal_quat, gains["K_trans"], gains["K_rot"], gains["damping_ratio"])
        world.arm.apply_joint_torques(tau)
        p.stepSimulation(physicsClientId=world.cid)
        world.arm.clamp_joint_velocities()

        tool_pos, _, _, _ = world.arm.get_tool_pose()
        force, torque, _ = get_contact_wrench(world.cid, world.arm.tool_id, [pad_id], tool_pos)
        
        depth = max(0.0, just_touching_z - pos_des[2])
        logger.record(t, tool_pos, pos_des, force=force, force_des=0.0, torque=torque)
        if dashboard:
            dashboard.update_material(depth, force[2])
        t += DT

    arr = logger.arrays()
    forces = arr["force"][:, 2]
    depths = np.maximum(0.0, just_touching_z - arr["pos_des"][:, 2])

    # --------------------------------------------------------------------------
    # TODO 5: Estimate Contact Stiffness
    # Estimate contact stiffness K_hat via least-squares fit (F = K * depth).
    # --------------------------------------------------------------------------
    # Least squares through the origin: minimise sum (F_i - K d_i)^2  ->  K_hat = sum(d F) / sum(d^2)
    K_hat = float(np.sum(depths * forces) / np.sum(depths ** 2))

    if dashboard:
        dashboard.update_material(depths[-1], forces[-1], k_hat=K_hat)
        time.sleep(1)
        dashboard.close(save_path=os.path.join(save_dir, f"{tag}_dashboard.png"))

    os.makedirs(save_dir, exist_ok=True)
    steady_force = float(np.percentile(forces[-max(int(2.0/DT), 5):], 75))
    fig, ax = plt.subplots(figsize=(6, 4))
    ax.plot(arr["t"], forces, color=RED, lw=1.8)
    ax.axhline(steady_force, color=GREY, ls="--", lw=1.2, label=f"steady state: F={steady_force:.1f} N, K̂={K_hat:.0f} N/m")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Contact force (N)")
    ax.legend(fontsize=9, frameon=False)
    ax.spines[["top", "right"]].set_visible(False)
    ax.set_title(f"Material Probe -- {tag}", fontsize=12, weight="bold", color=NAVY)
    plt.tight_layout()
    plt.savefig(os.path.join(save_dir, f"{tag}_loading_curve.png"), dpi=180, facecolor="white")
    plt.close(fig)
    return K_hat


# ==============================================================================
# MAIN EXECUTIONS
# ==============================================================================

def run_part1(save_dir="outputs/part1"):
    print("--- Part 1: Trajectory Generation ---")
    os.makedirs(save_dir, exist_ok=True)
    fig, axes = plt.subplots(1, 4, figsize=(12, 3))
    shapes = ["circle", "spiral", "infinity", "heart"]
    for i, shape in enumerate(shapes):
        strokes = generate_pattern(shape, scale=0.1)
        ax = axes[i]
        for s in strokes:
            pts = np.array(s)
            ax.plot(pts[:, 0], pts[:, 1], 'b-', lw=2)
        ax.set_aspect("equal")
        ax.set_title(shape.capitalize())
        ax.axis("off")
    plt.tight_layout()
    plt.savefig(os.path.join(save_dir, "shapes.png"), dpi=150)
    plt.close()

def run_part2_writing(gui=False, save_dir="outputs/part3", f_desired=F_DESIRED, word="Robot", pattern="circle", live=False):
    print("--- Part 2 (continued): Impedance Writing ---")
    world = build_world(gui=gui)
    ctrl = CartesianImpedanceController(world.arm)
    s, _ = run_writing_task(world, ctrl, "impedance", f_desired, gui=gui, save_dir=save_dir, tag="part3_impedance", word=word, pattern=pattern, live=live)
    world.disconnect()
    return s

def run_part2_writing_admittance(gui=False, save_dir="outputs/part4", f_desired=F_DESIRED, word="Robot", pattern="circle", live=False):
    print("--- Part 3: Admittance Writing ---")
    world = build_world(gui=gui)
    ctrl = CartesianImpedanceController(world.arm)
    s, _ = run_writing_task(world, ctrl, "admittance", f_desired, gui=gui, save_dir=save_dir, tag="part4_admittance", word=word, pattern=pattern, live=live)
    world.disconnect()
    return s

def run_part3(save_dir="outputs/part5", s3=None, s4=None, word="Robot", pattern="circle"):
    print("--- Part 4: Analysis and Comparison ---")
    os.makedirs(save_dir, exist_ok=True)
    s3 = s3 or run_part2_writing(gui=False, word=word, pattern=pattern)
    s4 = s4 or run_part2_writing_admittance(gui=False, word=word, pattern=pattern)
    
    dynamic_cmp = {"Impedance (Part 2)": s3, "Admittance (Part 3)": s4}
    bar_compare(["Force RMSE (N)"], {k: [v["force_tracking_rmse_N"]] for k, v in dynamic_cmp.items()}, "RMSE (N)", os.path.join(save_dir, "part5_writing_force_rmse.png"), "Writing: Force Tracking RMSE")
    bar_compare(["Path Smoothness (mean sq. jerk)"], {k: [v["path_smoothness_jerk2"]] for k, v in dynamic_cmp.items()}, "Mean squared jerk", os.path.join(save_dir, "part5_writing_smoothness.png"), "Writing: Motion Smoothness")

def run_part4(gui=False, save_dir="outputs/part6", live=False):
    print("--- Part 5: Haptic Material Identification ---")
    os.makedirs(save_dir, exist_ok=True)
    world = build_world(gui=gui, with_material_samples=True)
    ctrl = CartesianImpedanceController(world.arm)
    results = {}
    for name, (pad_id, pad_top_z, true_label) in world.material_ids.items():
        spec = MATERIAL_SAMPLES[name]
        xy = world.surface_center_xy + np.array(spec["offset_xy"])
        K_hat = probe_material(world, ctrl, pad_id, pad_top_z, xy, save_dir, name, live=live)
        
        # ----------------------------------------------------------------------
        # TODO 6: Classify Material
        # Classify material as 'soft', 'medium', or 'hard' based on K_hat and MATERIAL_CLASS_THRESHOLDS.
        # ----------------------------------------------------------------------
        t_soft, t_hard = MATERIAL_CLASS_THRESHOLDS
        if K_hat < t_soft:
            pred_label = "soft"
        elif K_hat < t_hard:
            pred_label = "medium"
        else:
            pred_label = "hard"
            
        results[name] = dict(true_stiffness_N_per_m=spec["contact_stiffness"], estimated_stiffness_N_per_m=K_hat, true_label=true_label, predicted_label=pred_label, correct=(pred_label == true_label))
        print(f"[Part 5] {name}: estimated K̂={K_hat:.0f} N/m -> predicted={pred_label}")
    n_correct = sum(1 for r in results.values() if r["correct"])
    save_json(os.path.join(save_dir, "part6_results.json"), dict(accuracy=n_correct/3.0, results=results))
    world.disconnect()
    return results

def run_bonus(word="robot", gui=False, save_dir="outputs/bonus", live=False):
    print("--- Bonus: Cursive Writing ---")
    results = {}
    for mode in ["impedance", "admittance"]:
        world = build_world(gui=gui)
        ctrl = CartesianImpedanceController(world.arm)
        results[mode], _ = run_writing_task(world, ctrl, mode, F_DESIRED, gui=gui, save_dir=save_dir,
                                            tag=f"cursive_{mode}", word=word, live=live, cursive=True)
        world.disconnect()
    for shape in ["spiral", "infinity", "heart"]:           # complex shapes, drawn by the robot
        world = build_world(gui=gui)
        ctrl = CartesianImpedanceController(world.arm)
        results[shape], _ = run_writing_task(world, ctrl, "impedance", F_DESIRED, gui=gui, save_dir=save_dir,
                                             tag=f"shape_{shape}", word=None, pattern=shape, live=live)
        world.disconnect()
    return results

def main():
    parser = argparse.ArgumentParser(description="Force-Controlled Writing")
    parser.add_argument("--part", choices=["1", "2", "3", "4", "5", "all", "bonus"], default="all")
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--word", type=str, default="Robot", help="Word to write")
    parser.add_argument("--pattern", type=str, default="circle", help="Pattern to draw")
    parser.add_argument("--live", action="store_true", help="Enable live dashboard visualization")
    args = parser.parse_args()
    
    pattern = None if args.pattern == "none" else args.pattern
    
    s3 = s4 = None
    if args.part in ["1", "all"]: run_part1()
    if args.part in ["2", "all"]: s3 = run_part2_writing(gui=args.gui, word=args.word, pattern=pattern, live=args.live)
    if args.part in ["3", "all"]: s4 = run_part2_writing_admittance(gui=args.gui, word=args.word, pattern=pattern, live=args.live)
    if args.part in ["4", "all"]: run_part3(s3=s3, s4=s4, word=args.word, pattern=pattern)
    if args.part in ["5", "all"]: run_part4(gui=args.gui, live=args.live)
    if args.part in ["bonus", "all"]: run_bonus(gui=args.gui, live=args.live)

if __name__ == '__main__':
    main()
