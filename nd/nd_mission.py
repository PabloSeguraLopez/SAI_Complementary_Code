#!/usr/bin/env python3
from __future__ import annotations

import csv
import json
import math
import os
import time
import threading
from dataclasses import dataclass
from datetime import datetime
from typing import List, Optional, Tuple

import matplotlib.pyplot as plt
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from as2_python_api.drone_interface_teleop import DroneInterfaceTeleop


# ============================================================
# Utilidades
# ============================================================

def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def wrap_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def ang_diff(a: float, b: float) -> float:
    return abs(wrap_pi(a - b))


def circular_mean(a: float, b: float) -> float:
    x = math.cos(a) + math.cos(b)
    y = math.sin(a) + math.sin(b)
    return math.atan2(y, x)


def mean_or_default(values: List[float], default: float) -> float:
    return sum(values) / len(values) if values else default


# ============================================================
# Estructuras
# ============================================================

@dataclass
class Discontinuity:
    left_idx: int
    right_idx: int
    delta: float
    rising_from_left_to_right: bool


@dataclass
class Valley:
    start_idx: int
    end_idx: int
    left_disc: Discontinuity
    right_disc: Discontinuity


@dataclass
class NDDecision:
    situation: str
    valley: Optional[Valley]
    s_goal: int
    s_theta: Optional[int]
    s_rd: Optional[int]
    s_left_obs: Optional[int]
    s_right_obs: Optional[int]
    desired_heading_body: float
    reason: str


@dataclass
class MetricClearances:
    front_min: float
    front_left_min: float
    front_right_min: float
    left_min: float
    right_min: float
    rear_min: float
    global_min: float
    front_mean: float
    left_mean: float
    right_mean: float


# ============================================================
# Controlador ND pragmático para dron con métricas
# ============================================================

class PragmaticNDDrone(DroneInterfaceTeleop):
    def __init__(
        self,
        drone_id: str = "drone0",
        verbose: bool = False,
        use_sim_time: bool = True,
        enable_radar_plot: bool = True,
    ):
        super().__init__(drone_id=drone_id, verbose=verbose, use_sim_time=use_sim_time)

        # ----------------------------------------------------
        # Misión
        # ----------------------------------------------------

        # Meta mapa de prueba columnas
        # self.goal_xyz = [12.0, 0.0, 1.5]

        # Escenario Fácil
        # self.goal_xyz = [80.4, 34.3, 1.5]

        # Escenario Medio
        self.goal_xyz = [59.3, 87.7, 1.5]

        # Escenario Difícil
        # self.goal_xyz = [93.6, 74.1, 1.5]

        self.takeoff_height = 1.5

        # ----------------------------------------------------
        # Parámetros plataforma
        # ----------------------------------------------------
        self.robot_radius = 0.65
        self.max_xy_speed = 1.00
        self.max_z_speed = 0.12
        self.max_yaw_rate = 0.70

        self.goal_tol_xy = 1.0
        self.goal_tol_z = 0.20

        self.self_filter_radius = 0.35
        self.valid_obstacles: List[Tuple[float, float]] = []
        self.ignored_self_points_body: List[Tuple[float, float]] = []

        # ----------------------------------------------------
        # Parámetros ND
        # ----------------------------------------------------
        self.n_sectors = 144
        self.max_range = 6.0
        self.security_distance = 0.90
        self.wide_valley_angle_deg = 70.0
        self.front_valley_limit_deg = 110.0
        self.max_heading_deg = 80.0
        self.gap_local_window_deg = 30.0
        self.discontinuity_threshold = 2.0 * self.robot_radius
        self.min_valley_angle_deg = 12.0

        # ----------------------------------------------------
        # Ganancias / control
        # ----------------------------------------------------
        self.k_yaw = 1.2
        self.k_z = 0.45
        self.goal_slowdown_dist = 3.0

        # Pesos de acciones
        self.ls1_avoid_gain = math.radians(28.0)
        self.ls2_center_gain = math.radians(18.0)
        self.hswr_bias_deg = 12.0
        self.too_close_override_dist = 1.35

        # ----------------------------------------------------
        # Histéresis / memoria
        # ----------------------------------------------------
        self.last_selected_valley_center: Optional[int] = None
        self.valley_switch_penalty = 10.0
        self.last_desired_heading_body: Optional[float] = None
        self.max_heading_step_deg = 18.0

        self.last_situation: Optional[str] = None
        self.last_situation_time = 0.0
        self.situation_hold_time = 0.8

        # ----------------------------------------------------
        # Stuck detector propio del controlador
        # ----------------------------------------------------
        self.stuck_history: List[Tuple[float, float, float, float]] = []
        self.stuck_window_sec = 2.5
        self.stuck_min_progress = 0.05
        self.stuck_min_goal_progress = 0.20
        self.stuck_min_samples = 6
        self.escape_mode_until = 0.0
        self.escape_start = 0.0
        self.post_escape_grace_sec = 0.8
        self.post_escape_ignore_until = 0.0

        # ----------------------------------------------------
        # Estado LiDAR
        # ----------------------------------------------------
        self.scan_ready = False
        self.last_scan_time = 0.0
        self.scan_timeout = 0.8

        self.pnd = [self.max_range] * self.n_sectors
        self.rnd = [self.max_range] * self.n_sectors
        self.obstacle_points_body: List[Tuple[float, float]] = []
        self.closest_obs_sector = 0

        self.last_log_time = 0.0
        self.log_period = 0.25

        # ----------------------------------------------------
        # Métricas de misión
        # ----------------------------------------------------
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.metrics_dir = f"mission_metrics_{stamp}_nd_10"
        os.makedirs(self.metrics_dir, exist_ok=True)

        self.mission_started = False
        self.metrics_finished = False
        self.mission_start_time: Optional[float] = None
        self.mission_end_time: Optional[float] = None

        self.trajectory_log: List[dict] = []
        self.min_goal_dist_xy = float("inf")
        self.final_goal_error: Optional[float] = None
        self.success = False

        self.metrics_crash_distance = 0.35
        self.metrics_crash_count = 0
        self.metrics_last_crash_time = 0.0
        self.metrics_crash_cooldown = 2.0

        self.metrics_stuck_count = 0
        self.metrics_stuck_start_time: Optional[float] = None
        self.metrics_stuck_speed_threshold = 0.03
        self.metrics_stuck_time_threshold = 5.0
        self.metrics_stuck_goal_dist_threshold = 1.0

        self.metrics_last_goal_dist_for_progress: Optional[float] = None
        self.metrics_last_progress_time = time.monotonic()
        self.metrics_no_progress_timeout = 8.0
        self.metrics_no_progress_epsilon = 0.15

        # ----------------------------------------------------
        # Radar plot
        # ----------------------------------------------------
        self.enable_radar_plot = enable_radar_plot

        self.fig = None
        self.ax_radar = None
        self.ax_ts1 = None  # distancias / altura
        self.ax_ts2 = None  # velocidades lineales
        self.ax_ts3 = None  # velocidad angular

        self.last_plot_time = 0.0
        self.plot_period = 0.30

        self.max_plot_points = 800

        self.scan_sub = self.create_subscription(
            LaserScan,
            "sensor_measurements/lidar/scan",
            self.scan_callback,
            qos_profile_sensor_data,
        )

    # =====================================================
    # Geometría
    # =====================================================

    def sector_width(self) -> float:
        return 2.0 * math.pi / self.n_sectors

    def angle_to_sector(self, angle_rad: float) -> int:
        a = wrap_pi(angle_rad)
        idx = int((a + math.pi) / self.sector_width())
        return max(0, min(self.n_sectors - 1, idx))

    def sector_to_angle(self, idx: int) -> float:
        idx = idx % self.n_sectors
        return -math.pi + (idx + 0.5) * self.sector_width()

    def sector_distance(self, a: int, b: int) -> int:
        d = abs((a % self.n_sectors) - (b % self.n_sectors))
        return min(d, self.n_sectors - d)

    def sectors_in_arc(self, start: int, end: int) -> List[int]:
        out = []
        i = start % self.n_sectors
        end = end % self.n_sectors
        while True:
            out.append(i)
            if i == end:
                break
            i = (i + 1) % self.n_sectors
        return out

    def clamp_heading_for_drone(self, heading_body: float) -> float:
        limit = math.radians(self.max_heading_deg)
        return clamp(wrap_pi(heading_body), -limit, limit)

    def get_goal_in_body_frame(self) -> Tuple[float, float, float, float, float]:
        px, py, pz = self.position
        _, _, yaw = self.orientation

        dx_w = self.goal_xyz[0] - px
        dy_w = self.goal_xyz[1] - py
        dz = self.goal_xyz[2] - pz

        c = math.cos(-yaw)
        s = math.sin(-yaw)

        gx = c * dx_w - s * dy_w
        gy = s * dx_w + c * dy_w

        goal_heading_body = math.atan2(gy, gx)
        goal_dist_xy = math.hypot(gx, gy)

        return gx, gy, dz, goal_heading_body, goal_dist_xy

    # =====================================================
    # Radar chart
    # =====================================================

    def setup_debug_plot(self) -> None:
        if not self.enable_radar_plot:
            return

        plt.ion()

        self.fig = plt.figure(figsize=(13, 9))
        gs = self.fig.add_gridspec(3, 2)

        self.ax_radar = self.fig.add_subplot(gs[:, 0], projection="polar")
        self.ax_ts1 = self.fig.add_subplot(gs[0, 1])
        self.ax_ts2 = self.fig.add_subplot(gs[1, 1])
        self.ax_ts3 = self.fig.add_subplot(gs[2, 1])

        try:
            self.fig.canvas.manager.set_window_title("ND Debug Plot")
        except Exception:
            pass


    def plot_debug(
        self,
        decision: NDDecision,
        vx: float,
        vy: float,
        vz: float,
        yaw_rate: float,
    ) -> None:
        if not self.enable_radar_plot:
            return

        now = time.monotonic()

        if (now - self.last_plot_time) < self.plot_period:
            return

        self.last_plot_time = now

        if self.fig is None:
            self.setup_debug_plot()

        if (
            self.fig is None
            or self.ax_radar is None
            or self.ax_ts1 is None
            or self.ax_ts2 is None
            or self.ax_ts3 is None
        ):
            return

        _gx, _gy, dz, goal_heading_body, goal_dist_xy = self.get_goal_in_body_frame()

        # =====================================================
        # Radar Chart
        # =====================================================

        self.ax_radar.clear()
        self.ax_radar.set_theta_zero_location("N")
        self.ax_radar.set_theta_direction(-1)
        self.ax_radar.set_rlim(0, self.max_range)
        self.ax_radar.set_title(f"Radar | situation={decision.situation}")

        theta_circle = [i * 2.0 * math.pi / 360.0 for i in range(361)]

        self.ax_radar.plot(
            theta_circle,
            [self.security_distance] * len(theta_circle),
            linestyle="--",
            linewidth=1.0,
            label="hard_stop",
        )

        self.ax_radar.plot(
            theta_circle,
            [self.too_close_override_dist] * len(theta_circle),
            linestyle="--",
            linewidth=1.0,
            label="caution",
        )

        self.ax_radar.plot(
            theta_circle,
            [self.security_distance] * len(theta_circle),
            linestyle=":",
            linewidth=1.0,
            label="side_hard",
        )

        self.ax_radar.plot(
            theta_circle,
            [self.self_filter_radius] * len(theta_circle),
            linestyle=":",
            linewidth=1.0,
            label="self_filter",
        )

        self.ax_radar.plot(
            theta_circle,
            [self.goal_slowdown_dist] * len(theta_circle),
            linestyle="-.",
            linewidth=1.0,
            label="goal_slowdown_zone",
        )

        if self.obstacle_points_body:
            obs_theta = []
            obs_r = []

            for x, y in self.obstacle_points_body:
                r = math.hypot(x, y)
                if 0.0 < r <= self.max_range:
                    obs_theta.append(math.atan2(y, x))
                    obs_r.append(r)

            if obs_theta:
                self.ax_radar.scatter(
                    obs_theta,
                    obs_r,
                    s=10,
                    alpha=0.85,
                    label="obs",
                )

        if self.ignored_self_points_body:
            ign_theta = []
            ign_r = []

            for x, y in self.ignored_self_points_body:
                r = math.hypot(x, y)
                if 0.0 < r <= self.max_range:
                    ign_theta.append(math.atan2(y, x))
                    ign_r.append(r)

            if ign_theta:
                self.ax_radar.scatter(
                    ign_theta,
                    ign_r,
                    s=8,
                    alpha=0.35,
                    label="ignored_self",
                )

        self.ax_radar.plot(
            [goal_heading_body, goal_heading_body],
            [0.0, min(goal_dist_xy, self.max_range)],
            linestyle="-.",
            linewidth=2.0,
            label="goal",
        )

        self.ax_radar.plot(
            [decision.desired_heading_body, decision.desired_heading_body],
            [0.0, min(self.max_range, 2.5)],
            linewidth=2.2,
            label="heading_cmd",
        )

        if decision.s_theta is not None:
            s_theta_angle = self.sector_to_angle(decision.s_theta)
            self.ax_radar.plot(
                [s_theta_angle, s_theta_angle],
                [0.0, min(self.max_range, 2.2)],
                linestyle=":",
                linewidth=2.0,
                label="s_theta",
            )

        self.ax_radar.scatter(
            [0.0],
            [0.0],
            s=70,
            marker="o",
            label="drone",
        )

        min_pnd = min(self.pnd) if self.pnd else self.max_range
        min_rnd = min(self.rnd) if self.rnd else self.max_range
        vxy = math.hypot(vx, vy)

        valley_txt = "None"
        if decision.valley is not None:
            valley_txt = (
                f"{decision.valley.start_idx}->{decision.valley.end_idx} "
                f"w={self.valley_width_sectors(decision.valley)}"
            )

        self.ax_radar.text(
            0.02,
            0.02,
            (
                f"valley={valley_txt}\n"
                f"min_pnd={min_pnd:.2f} m\n"
                f"min_rnd={min_rnd:.2f} m\n"
                f"vxy={vxy:.2f} m/s\n"
                f"yaw_rate={math.degrees(yaw_rate):.1f} deg/s"
            ),
            transform=self.ax_radar.transAxes,
            fontsize=9,
        )

        self.ax_radar.legend(
            loc="upper right",
            bbox_to_anchor=(1.30, 1.10),
            fontsize=8,
        )

        # =====================================================
        # Series temporales desde trajectory_log
        # =====================================================

        rows = self.trajectory_log[-self.max_plot_points:]
        t = [row["t"] for row in rows]

        # -----------------------------------------------------
        # Plot superior derecho: Distancia a meta
        # -----------------------------------------------------

        self.ax_ts1.clear()

        if t:
            self.ax_ts1.plot(t, [row["z"] for row in rows], label="z")
            self.ax_ts1.plot(
                t,
                [self.goal_xyz[2]] * len(t),
                linestyle="--",
                label="z target",
            )
            self.ax_ts1.plot(t, [row["goal_dist_xy"] for row in rows], label="dist XY goal")
            self.ax_ts1.plot(t, [row["front_min"] for row in rows], label="front min")
            self.ax_ts1.plot(t, [row["left_min"] for row in rows], label="left min")
            self.ax_ts1.plot(t, [row["right_min"] for row in rows], label="right min")
            self.ax_ts1.plot(t, [row["global_min"] for row in rows], label="global min")

        self.ax_ts1.set_title("Altura / meta / distancias")
        self.ax_ts1.set_xlabel("t [s]")
        self.ax_ts1.set_ylabel("m")
        self.ax_ts1.grid(True)
        self.ax_ts1.legend(fontsize=8)

        # -----------------------------------------------------
        # Plot inferior derecho: comandos
        #   - eje izquierdo: velocidades lineales en m/s
        #   - eje derecho: yaw_rate en deg/s
        # -----------------------------------------------------

        self.ax_ts2.clear()

        if t:
            self.ax_ts2.plot(t, [row["vxy_cmd"] for row in rows], label="|vxy|")
            self.ax_ts2.plot(t, [row["vx_cmd"] for row in rows], label="vx")
            self.ax_ts2.plot(t, [row["vy_cmd"] for row in rows], label="vy")
            self.ax_ts2.plot(t, [row["vz_cmd"] for row in rows], label="vz")

        self.ax_ts2.set_title("Velocidades lineales")
        self.ax_ts2.set_xlabel("t [s]")
        self.ax_ts2.set_ylabel("m/s")
        self.ax_ts2.grid(True)
        self.ax_ts2.legend(fontsize=8)

        # -----------------------------------------------------
        # Plot inferior derecho: velocidad angular
        # -----------------------------------------------------

        self.ax_ts3.clear()

        if t:
            self.ax_ts3.plot(
                t,
                [math.degrees(row["yaw_rate_cmd"]) for row in rows],
                label="yaw_rate",
            )

        self.ax_ts3.set_title("Velocidad angular")
        self.ax_ts3.set_xlabel("t [s]")
        self.ax_ts3.set_ylabel("deg/s")
        self.ax_ts3.grid(True)
        self.ax_ts3.legend(fontsize=8)

        last_row = rows[-1] if rows else None

        self.fig.tight_layout()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    # =====================================================
    # LiDAR + filtro de retornos propios
    # =====================================================

    def scan_callback(self, msg: LaserScan) -> None:
        sectors = [self.max_range] * self.n_sectors

        valid_points: List[Tuple[float, float]] = []
        ignored_points: List[Tuple[float, float]] = []

        angle = msg.angle_min
        for r in msg.ranges:
            if math.isfinite(r) and (msg.range_min <= r <= msg.range_max):
                rr = clamp(r, msg.range_min, min(msg.range_max, self.max_range))
                a = wrap_pi(angle)

                x = rr * math.cos(a)
                y = rr * math.sin(a)

                if rr < self.self_filter_radius:
                    ignored_points.append((x, y))
                    angle += msg.angle_increment
                    continue

                idx = self.angle_to_sector(a)
                if rr < sectors[idx]:
                    sectors[idx] = rr

                valid_points.append((x, y))

            angle += msg.angle_increment

        self.pnd = sectors
        self.rnd = [max(0.0, d - self.robot_radius) for d in self.pnd]

        self.obstacle_points_body = valid_points
        self.ignored_self_points_body = ignored_points

        for i in range(self.n_sectors):
            ang = self.sector_to_angle(i)
            if abs(ang) > math.radians(110):
                self.pnd[i] = self.max_range
                self.rnd[i] = self.max_range - self.robot_radius

        min_dist = min(self.rnd)
        if min_dist < self.max_range:
            self.closest_obs_sector = self.rnd.index(min_dist)
        else:
            self.closest_obs_sector = 0

        self.scan_ready = True
        self.last_scan_time = time.monotonic()

    # =====================================================
    # Discontinuidades / valles
    # =====================================================

    def detect_discontinuities(self) -> List[Discontinuity]:
        discs: List[Discontinuity] = []

        for i in range(self.n_sectors):
            j = (i + 1) % self.n_sectors
            delta = self.pnd[j] - self.pnd[i]

            if abs(delta) > self.discontinuity_threshold:
                discs.append(
                    Discontinuity(
                        left_idx=i,
                        right_idx=j,
                        delta=delta,
                        rising_from_left_to_right=(delta > 0.0),
                    )
                )
        return discs

    def build_valleys(self, discs: List[Discontinuity]) -> List[Valley]:
        valleys: List[Valley] = []

        if not discs:
            fake = Discontinuity(
                left_idx=self.n_sectors - 1,
                right_idx=0,
                delta=0.0,
                rising_from_left_to_right=True,
            )
            valleys.append(Valley(0, self.n_sectors - 1, fake, fake))
            return valleys

        discs_sorted = sorted(discs, key=lambda d: d.right_idx)

        for k in range(len(discs_sorted)):
            d_left = discs_sorted[k]
            d_right = discs_sorted[(k + 1) % len(discs_sorted)]

            has_rising = d_left.rising_from_left_to_right or d_right.rising_from_left_to_right
            if not has_rising:
                continue

            valleys.append(
                Valley(
                    start_idx=d_left.right_idx,
                    end_idx=d_right.left_idx,
                    left_disc=d_left,
                    right_disc=d_right,
                )
            )

        return valleys

    def valley_width_sectors(self, valley: Valley) -> int:
        return len(self.sectors_in_arc(valley.start_idx, valley.end_idx))

    def valley_min_clearance(self, valley: Valley) -> float:
        return min(self.rnd[s] for s in self.sectors_in_arc(valley.start_idx, valley.end_idx))

    def valley_is_navigable(self, valley: Valley) -> bool:
        min_sectors = max(1, int(round(math.radians(self.min_valley_angle_deg) / self.sector_width())))
        if self.valley_width_sectors(valley) < min_sectors:
            return False

        required_clearance = self.robot_radius + self.security_distance
        min_inside = self.valley_min_clearance(valley)
        if min_inside < required_clearance:
            return False

        r_min = min(self.pnd[s] for s in self.sectors_in_arc(valley.start_idx, valley.end_idx))
        theta = self.valley_width_sectors(valley) * self.sector_width()
        est_width = 2.0 * r_min if theta >= math.pi else 2.0 * r_min * math.sin(0.5 * theta)
        return est_width >= (2.0 * required_clearance)

    def valley_center_sector(self, valley: Valley) -> int:
        arc = self.sectors_in_arc(valley.start_idx, valley.end_idx)
        return arc[len(arc) // 2]

    def valley_center_angle(self, valley: Valley) -> float:
        return self.sector_to_angle(self.valley_center_sector(valley))

    def valley_is_wide(self, valley: Valley) -> bool:
        th = int(math.radians(self.wide_valley_angle_deg) / self.sector_width())
        return self.valley_width_sectors(valley) >= th

    def valley_is_frontish(self, valley: Valley) -> bool:
        ang = self.valley_center_angle(valley)
        return abs(math.degrees(ang)) <= self.front_valley_limit_deg

    def valley_contains_sector(self, valley: Valley, sector: int) -> bool:
        return sector in self.sectors_in_arc(valley.start_idx, valley.end_idx)

    def valley_rising_sector_closest_to_goal(self, valley: Valley, s_goal: int) -> int:
        candidates = []
        if valley.left_disc.rising_from_left_to_right:
            candidates.append(valley.left_disc.right_idx)
        if valley.right_disc.rising_from_left_to_right:
            candidates.append(valley.right_disc.right_idx)

        if not candidates:
            return self.valley_center_sector(valley)

        return min(candidates, key=lambda s: self.sector_distance(s, s_goal))

    def valley_other_boundary_sector(self, valley: Valley, s_theta: int) -> int:
        c1 = valley.left_disc.right_idx
        c2 = valley.right_disc.right_idx
        return c2 if s_theta == c1 else c1

    def select_valley(self, valleys: List[Valley], s_goal: int, goal_heading_body: float) -> Optional[Valley]:
        candidates = []

        for v in valleys:
            if not self.valley_is_frontish(v):
                continue
            if not self.valley_is_navigable(v):
                continue

            c = self.valley_center_sector(v)
            width = self.valley_width_sectors(v)
            cost = self.sector_distance(c, s_goal) - 0.20 * width

            center_ang = abs(self.valley_center_angle(v))
            if center_ang > math.radians(60):
                cost += 20.0

            if self.last_selected_valley_center is not None:
                cost += self.valley_switch_penalty * (
                    self.sector_distance(c, self.last_selected_valley_center) / self.n_sectors
                )

            candidates.append((cost, v))

        if not candidates:
            return None

        candidates.sort(key=lambda x: x[0])
        best = candidates[0][1]
        self.last_selected_valley_center = self.valley_center_sector(best)
        return best

    # =====================================================
    # Obstáculos peligrosos
    # =====================================================

    def dangerous_sectors_near_gap(self, s_theta: int, window_deg: float) -> Tuple[Optional[int], Optional[int]]:
        ds = self.security_distance
        win = max(1, int(math.radians(window_deg) / self.sector_width()))

        left = None
        right = None

        for k in range(1, win + 1):
            sl = (s_theta - k) % self.n_sectors
            sr = (s_theta + k) % self.n_sectors

            ang_left = abs(self.sector_to_angle(sl))
            ang_right = abs(self.sector_to_angle(sr))

            if ang_left <= math.radians(110) and left is None and self.rnd[sl] < ds:
                left = sl
            if ang_right <= math.radians(110) and right is None and self.rnd[sr] < ds:
                right = sr

            if left is not None and right is not None:
                break

        return left, right

    # =====================================================
    # Histeresis de situación
    # =====================================================

    def apply_situation_hysteresis(self, new_situation: str) -> str:
        now = time.monotonic()

        if self.last_situation is None:
            self.last_situation = new_situation
            self.last_situation_time = now
            return new_situation

        if new_situation == self.last_situation:
            self.last_situation_time = now
            return new_situation

        if (now - self.last_situation_time) < self.situation_hold_time:
            return self.last_situation

        self.last_situation = new_situation
        self.last_situation_time = now
        return new_situation

    # =====================================================
    # Clasificación ND
    # =====================================================

    def classify_and_decide(self) -> NDDecision:
        _gx, _gy, _dz, goal_heading_body, _goal_dist_xy = self.get_goal_in_body_frame()
        s_goal = self.angle_to_sector(goal_heading_body)

        discs = self.detect_discontinuities()
        valleys = self.build_valleys(discs)
        valley = self.select_valley(valleys, s_goal, goal_heading_body)

        if valley is None:
            desired = self.clamp_heading_for_drone(goal_heading_body)
            return NDDecision(
                situation="HSGR",
                valley=None,
                s_goal=s_goal,
                s_theta=None,
                s_rd=None,
                s_left_obs=None,
                s_right_obs=None,
                desired_heading_body=desired,
                reason="Sin valle frontal válido, fallback hacia meta.",
            )

        s_theta = self.valley_rising_sector_closest_to_goal(valley, s_goal)
        s_rd = self.valley_other_boundary_sector(valley, s_theta)

        s_left_obs, s_right_obs = self.dangerous_sectors_near_gap(
            s_theta,
            window_deg=self.gap_local_window_deg,
        )

        low_safety = (s_left_obs is not None) or (s_right_obs is not None)
        goal_in_valley = self.valley_contains_sector(valley, s_goal)

        if low_safety:
            if (s_left_obs is not None) and (s_right_obs is not None):
                situation = self.apply_situation_hysteresis("LS2")
                desired = self.action_ls2(s_theta, s_left_obs, s_right_obs)
                return NDDecision(
                    situation=situation,
                    valley=valley,
                    s_goal=s_goal,
                    s_theta=s_theta,
                    s_rd=s_rd,
                    s_left_obs=s_left_obs,
                    s_right_obs=s_right_obs,
                    desired_heading_body=desired,
                    reason="Obstáculos peligrosos a ambos lados del gap local.",
                )

            situation = self.apply_situation_hysteresis("LS1")
            s_obs = s_left_obs if s_left_obs is not None else s_right_obs
            desired = self.action_ls1(s_theta, s_obs)
            return NDDecision(
                situation=situation,
                valley=valley,
                s_goal=s_goal,
                s_theta=s_theta,
                s_rd=s_rd,
                s_left_obs=s_left_obs,
                s_right_obs=s_right_obs,
                desired_heading_body=desired,
                reason="Obstáculo peligroso a un solo lado del gap local.",
            )

        if goal_in_valley:
            situation = self.apply_situation_hysteresis("HSGR")
            desired = self.action_hsgr(goal_heading_body)
            return NDDecision(
                situation=situation,
                valley=valley,
                s_goal=s_goal,
                s_theta=s_theta,
                s_rd=s_rd,
                s_left_obs=None,
                s_right_obs=None,
                desired_heading_body=desired,
                reason="Meta dentro del valle frontal seleccionado.",
            )

        if self.valley_is_wide(valley):
            situation = self.apply_situation_hysteresis("HSWR")
            desired = self.action_hswr(valley, s_theta, goal_heading_body)
            return NDDecision(
                situation=situation,
                valley=valley,
                s_goal=s_goal,
                s_theta=s_theta,
                s_rd=s_rd,
                s_left_obs=None,
                s_right_obs=None,
                desired_heading_body=desired,
                reason="Valle ancho: bordeo controlado.",
            )

        situation = self.apply_situation_hysteresis("HSNR")
        desired = self.action_hsnr(valley)
        return NDDecision(
            situation=situation,
            valley=valley,
            s_goal=s_goal,
            s_theta=s_theta,
            s_rd=s_rd,
            s_left_obs=None,
            s_right_obs=None,
            desired_heading_body=desired,
            reason="Valle estrecho: paso por el centro.",
        )

    # =====================================================
    # Acciones
    # =====================================================

    def action_ls1(self, s_theta: int, s_obs: int) -> float:
        a_gap = self.sector_to_angle(s_theta)
        a_obs = self.sector_to_angle(s_obs)

        sign = 1.0 if wrap_pi(a_gap - a_obs) >= 0.0 else -1.0
        desired = wrap_pi(a_gap + sign * self.ls1_avoid_gain)
        return self.clamp_heading_for_drone(desired)

    def action_ls2(self, s_theta: int, s_left_obs: int, s_right_obs: int) -> float:
        a_gap = self.sector_to_angle(s_theta)
        dl = self.rnd[s_left_obs]
        dr = self.rnd[s_right_obs]

        err = clamp((dr - dl) / max(0.10, self.security_distance), -1.0, 1.0)
        corr = clamp(self.ls2_center_gain * err, -math.radians(14.0), math.radians(14.0))
        desired = wrap_pi(a_gap + corr)
        return self.clamp_heading_for_drone(desired)

    def action_hsgr(self, goal_heading_body: float) -> float:
        return self.clamp_heading_for_drone(goal_heading_body)

    def action_hswr(self, valley: Valley, s_theta: int, goal_heading_body: float) -> float:
        center = self.valley_center_angle(valley)
        goal = self.clamp_heading_for_drone(goal_heading_body)
        bias = math.radians(self.hswr_bias_deg)

        cand_center = wrap_pi(center)
        cand1 = wrap_pi(0.75 * cand_center + 0.25 * goal)
        cand2 = wrap_pi(center + bias)
        cand3 = wrap_pi(center - bias)

        options = [cand1, cand2, cand3]
        desired = min(options, key=lambda a: ang_diff(a, goal))
        return self.clamp_heading_for_drone(desired)

    def action_safety_escape(self) -> float:
        obs_ang = self.sector_to_angle(self.closest_obs_sector)
        _gx, _gy, _dz, goal_heading_body, _goal_dist_xy = self.get_goal_in_body_frame()
        away_from_obs = wrap_pi(obs_ang + math.pi)
        desired = wrap_pi(0.70 * away_from_obs + 0.30 * goal_heading_body)
        return self.clamp_heading_for_drone(desired)

    def smooth_desired_heading(self, desired_heading_body: float) -> float:
        desired = wrap_pi(desired_heading_body)

        if self.last_desired_heading_body is None:
            self.last_desired_heading_body = desired
            return desired

        max_step = math.radians(self.max_heading_step_deg)
        delta = wrap_pi(desired - self.last_desired_heading_body)
        delta = clamp(delta, -max_step, max_step)
        smoothed = wrap_pi(self.last_desired_heading_body + delta)
        self.last_desired_heading_body = smoothed
        return smoothed

    def action_hsnr(self, valley: Valley) -> float:
        desired = self.valley_center_angle(valley)
        return self.clamp_heading_for_drone(desired)

    # =====================================================
    # Stuck detector del controlador
    # =====================================================

    def update_stuck_history(self, goal_dist_xy: float) -> None:
        now = time.monotonic()
        px, py, _pz = self.position
        self.stuck_history.append((now, px, py, goal_dist_xy))
        tmin = now - self.stuck_window_sec
        self.stuck_history = [x for x in self.stuck_history if x[0] >= tmin]

    def is_stuck(self) -> bool:
        if len(self.stuck_history) < self.stuck_min_samples:
            return False
        _t0, x0, y0, goal0 = self.stuck_history[0]
        _t1, x1, y1, goal1 = self.stuck_history[-1]
        progress = math.hypot(x1 - x0, y1 - y0)
        goal_progress = goal0 - goal1

        if goal_progress > self.stuck_min_goal_progress:
            return False

        return progress < self.stuck_min_progress

    def stuck_progress(self) -> float:
        if len(self.stuck_history) < 2:
            return 0.0
        _t0, x0, y0, _g0 = self.stuck_history[0]
        _t1, x1, y1, _g1 = self.stuck_history[-1]
        return math.hypot(x1 - x0, y1 - y0)

    def stuck_goal_progress(self) -> float:
        if len(self.stuck_history) < 2:
            return 0.0
        _t0, _x0, _y0, goal0 = self.stuck_history[0]
        _t1, _x1, _y1, goal1 = self.stuck_history[-1]
        return goal0 - goal1

    # =====================================================
    # Velocidades
    # =====================================================

    def translational_speed(self, situation: str, desired_heading_body: float, goal_dist_xy: float) -> float:
        d_near = min(self.rnd) if self.rnd else self.max_range
        goal_factor = clamp(goal_dist_xy / self.goal_slowdown_dist, 0.35, 1.0)
        heading_factor = clamp(0.50 + 0.50 * max(0.0, math.cos(desired_heading_body)), 0.35, 1.0)
        safety_factor = clamp(d_near / max(0.10, self.security_distance), 0.15, 1.0)

        if situation in ("LS1", "LS2"):
            if d_near < 0.5:
                safety_factor *= 0.5
            v = self.max_xy_speed * 0.6 * safety_factor * heading_factor * goal_factor
            return clamp(v, 0.10, 0.30)

        if situation == "HSNR":
            v = self.max_xy_speed * 0.75 * heading_factor * goal_factor * safety_factor
            return clamp(v, 0.12, 0.35)

        if situation == "HSWR":
            v = self.max_xy_speed * 0.80 * heading_factor * goal_factor * safety_factor
            return clamp(v, 0.10, 0.28)

        if situation == "SAFETY":
            v = self.max_xy_speed * 0.45 * safety_factor * clamp(goal_factor, 0.20, 0.60)
            return clamp(v, 0.08, 0.18)

        v = self.max_xy_speed * heading_factor * goal_factor * safety_factor
        return clamp(v, 0.12, self.max_xy_speed)

    # =====================================================
    # Comando final
    # =====================================================

    def compute_command(self) -> Tuple[float, float, float, float, bool, NDDecision]:
        _gx, _gy, dz, goal_heading_body, goal_dist_xy = self.get_goal_in_body_frame()

        goal_reached = (goal_dist_xy <= self.goal_tol_xy and abs(dz) <= self.goal_tol_z)
        if goal_reached:
            dummy = NDDecision(
                situation="GOAL",
                valley=None,
                s_goal=self.angle_to_sector(goal_heading_body),
                s_theta=None,
                s_rd=None,
                s_left_obs=None,
                s_right_obs=None,
                desired_heading_body=0.0,
                reason="Meta alcanzada.",
            )
            return 0.0, 0.0, 0.0, 0.0, True, dummy

        now = time.monotonic()

        if now < self.escape_mode_until:
            if now - self.escape_start < 0.3:
                vx = -0.2
                vy = 0.0
                vz = 0.0
                yaw_rate = 0.0
            else:
                yaw = self.orientation[2]
                desired_heading_world = wrap_pi(yaw + math.pi / 2)
                yaw_error = wrap_pi(desired_heading_world - yaw)
                yaw_rate = clamp(self.k_yaw * yaw_error, -0.4, 0.4)
                vx = 0.15
                vy = 0.0
                vz = 0.0

            decision = NDDecision(
                situation="ESCAPE",
                valley=None,
                s_goal=self.angle_to_sector(goal_heading_body),
                s_theta=None,
                s_rd=None,
                s_left_obs=None,
                s_right_obs=None,
                desired_heading_body=0.0,
                reason="Modo escape por atasco (retroceso + giro 90°).",
            )
            return vx, vy, vz, yaw_rate, False, decision

        decision = self.classify_and_decide()
        desired_heading_body = decision.desired_heading_body

        min_dist = min(self.rnd) if self.rnd else self.max_range
        if min_dist < self.too_close_override_dist:
            desired_heading_body = self.action_safety_escape()
            decision = NDDecision(
                situation="SAFETY",
                valley=decision.valley,
                s_goal=decision.s_goal,
                s_theta=decision.s_theta,
                s_rd=decision.s_rd,
                s_left_obs=decision.s_left_obs,
                s_right_obs=decision.s_right_obs,
                desired_heading_body=desired_heading_body,
                reason=(
                    "Override de seguridad: obstáculo demasiado cerca, alejando el dron "
                    "antes de volver al ND normal."
                ),
            )

        desired_heading_body = self.smooth_desired_heading(desired_heading_body)
        decision = NDDecision(
            situation=decision.situation,
            valley=decision.valley,
            s_goal=decision.s_goal,
            s_theta=decision.s_theta,
            s_rd=decision.s_rd,
            s_left_obs=decision.s_left_obs,
            s_right_obs=decision.s_right_obs,
            desired_heading_body=desired_heading_body,
            reason=decision.reason,
        )

        desired_speed = self.translational_speed(decision.situation, desired_heading_body, goal_dist_xy)

        yaw = self.orientation[2]
        desired_heading_world = wrap_pi(yaw + desired_heading_body)

        vx = desired_speed * math.cos(desired_heading_world)
        vy = desired_speed * math.sin(desired_heading_world)

        if abs(dz) < 0.10:
            vz = 0.0
        else:
            vz = clamp(self.k_z * dz, -self.max_z_speed, self.max_z_speed)

        yaw_error = wrap_pi(desired_heading_world - yaw)

        if decision.situation in ("LS1", "LS2"):
            yaw_limit = 0.55
        elif decision.situation == "SAFETY":
            yaw_limit = 0.40
        elif decision.situation == "ESCAPE":
            yaw_limit = 0.40
        else:
            yaw_limit = self.max_yaw_rate

        yaw_rate = clamp(self.k_yaw * yaw_error, -yaw_limit, yaw_limit)

        if decision.situation in ("LS1", "LS2") and min_dist < 0.6:
            yaw_rate = clamp(yaw_rate, -0.25, 0.25)
        if decision.situation == "SAFETY":
            yaw_rate = clamp(yaw_rate, -0.25, 0.25)

        if min_dist < 0.7:
            obs_ang = self.sector_to_angle(self.closest_obs_sector)
            if yaw_error * obs_ang > 0:
                yaw_rate *= 0.2

        return vx, vy, vz, yaw_rate, False, decision

    # =====================================================
    # Métricas
    # =====================================================

    def obstacle_values_in_window(self, center_deg: float, half_deg: float) -> List[float]:
        center = math.radians(center_deg)
        half = math.radians(half_deg)

        values = []
        for x, y in self.obstacle_points_body:
            r = math.hypot(x, y)
            if r <= 0.0 or r > self.max_range:
                continue
            a = math.atan2(y, x)
            if abs(wrap_pi(a - center)) <= half:
                values.append(r)

        return values

    def compute_metric_clearances(self) -> MetricClearances:
        front_vals = self.obstacle_values_in_window(0.0, 18.0)
        front_left_vals = self.obstacle_values_in_window(25.0, 20.0)
        front_right_vals = self.obstacle_values_in_window(-25.0, 20.0)
        left_vals = self.obstacle_values_in_window(70.0, 25.0)
        right_vals = self.obstacle_values_in_window(-70.0, 25.0)
        rear_vals = self.obstacle_values_in_window(180.0, 30.0)

        all_vals = [math.hypot(x, y) for x, y in self.obstacle_points_body]
        global_min = min(all_vals) if all_vals else self.max_range

        return MetricClearances(
            front_min=min(front_vals) if front_vals else self.max_range,
            front_left_min=min(front_left_vals) if front_left_vals else self.max_range,
            front_right_min=min(front_right_vals) if front_right_vals else self.max_range,
            left_min=min(left_vals) if left_vals else self.max_range,
            right_min=min(right_vals) if right_vals else self.max_range,
            rear_min=min(rear_vals) if rear_vals else self.max_range,
            global_min=global_min,
            front_mean=mean_or_default(front_vals, self.max_range),
            left_mean=mean_or_default(left_vals, self.max_range),
            right_mean=mean_or_default(right_vals, self.max_range),
        )

    def start_metrics(self) -> None:
        if self.mission_started:
            return

        self.mission_started = True
        self.metrics_finished = False
        self.mission_start_time = time.monotonic()
        self.mission_end_time = None

        self.trajectory_log.clear()
        self.min_goal_dist_xy = float("inf")
        self.final_goal_error = None
        self.success = False

        self.metrics_crash_count = 0
        self.metrics_last_crash_time = 0.0
        self.metrics_stuck_count = 0
        self.metrics_stuck_start_time = None
        self.metrics_last_goal_dist_for_progress = None
        self.metrics_last_progress_time = time.monotonic()

        self.get_logger().info(f"Métricas iniciadas. Carpeta: {self.metrics_dir}")

    def update_metrics(
        self,
        decision: NDDecision,
        vx: float,
        vy: float,
        vz: float,
        yaw_rate: float,
    ) -> None:
        if not self.mission_started or self.metrics_finished or self.mission_start_time is None:
            return

        now = time.monotonic()
        elapsed = now - self.mission_start_time

        _gx, _gy, dz, goal_heading_body, goal_dist_xy = self.get_goal_in_body_frame()
        px, py, pz = self.position
        roll, pitch, yaw = self.orientation

        clearances = self.compute_metric_clearances()
        vxy = math.hypot(vx, vy)
        self.min_goal_dist_xy = min(self.min_goal_dist_xy, goal_dist_xy)

        crashed_now = False
        if clearances.global_min <= self.metrics_crash_distance:
            if now - self.metrics_last_crash_time > self.metrics_crash_cooldown:
                self.metrics_crash_count += 1
                self.metrics_last_crash_time = now
                crashed_now = True
                self.get_logger().warn(
                    f"Posible choque/contacto cercano detectado: global_min={clearances.global_min:.2f} m"
                )

        stuck_now = False
        if goal_dist_xy > self.metrics_stuck_goal_dist_threshold and vxy < self.metrics_stuck_speed_threshold:
            if self.metrics_stuck_start_time is None:
                self.metrics_stuck_start_time = now
            elif now - self.metrics_stuck_start_time >= self.metrics_stuck_time_threshold:
                self.metrics_stuck_count += 1
                stuck_now = True
                self.metrics_stuck_start_time = None
                self.get_logger().warn(
                    "Posible atasco métrico: velocidad comandada casi nula lejos de la meta"
                )
        else:
            self.metrics_stuck_start_time = None

        no_progress_now = False
        if self.metrics_last_goal_dist_for_progress is None:
            self.metrics_last_goal_dist_for_progress = goal_dist_xy
            self.metrics_last_progress_time = now
        else:
            improvement = self.metrics_last_goal_dist_for_progress - goal_dist_xy

            if improvement > self.metrics_no_progress_epsilon:
                self.metrics_last_goal_dist_for_progress = goal_dist_xy
                self.metrics_last_progress_time = now
            elif now - self.metrics_last_progress_time > self.metrics_no_progress_timeout:
                self.metrics_stuck_count += 1
                no_progress_now = True
                self.metrics_last_progress_time = now
                self.metrics_last_goal_dist_for_progress = goal_dist_xy
                self.get_logger().warn(
                    "Posible atasco métrico: sin progreso suficiente hacia la meta"
                )

        valley_width = None
        valley_start = None
        valley_end = None
        valley_center_deg = None
        valley_min_clearance = None

        if decision.valley is not None:
            valley_width = self.valley_width_sectors(decision.valley)
            valley_start = decision.valley.start_idx
            valley_end = decision.valley.end_idx
            valley_center_deg = math.degrees(self.valley_center_angle(decision.valley))
            valley_min_clearance = self.valley_min_clearance(decision.valley)

        self.trajectory_log.append({
            "t": elapsed,
            "x": px,
            "y": py,
            "z": pz,
            "roll": roll,
            "pitch": pitch,
            "yaw": yaw,
            "goal_x": self.goal_xyz[0],
            "goal_y": self.goal_xyz[1],
            "goal_z": self.goal_xyz[2],
            "goal_dist_xy": goal_dist_xy,
            "goal_error_z": dz,
            "goal_error_3d": math.sqrt(goal_dist_xy ** 2 + dz ** 2),
            "front_min": clearances.front_min,
            "front_left_min": clearances.front_left_min,
            "front_right_min": clearances.front_right_min,
            "left_min": clearances.left_min,
            "right_min": clearances.right_min,
            "rear_min": clearances.rear_min,
            "global_min": clearances.global_min,
            "front_mean": clearances.front_mean,
            "left_mean": clearances.left_mean,
            "right_mean": clearances.right_mean,
            "min_pnd": min(self.pnd) if self.pnd else self.max_range,
            "min_rnd": min(self.rnd) if self.rnd else self.max_range,
            "closest_obs_sector": self.closest_obs_sector,
            "situation": decision.situation,
            "reason": decision.reason,
            "s_goal": decision.s_goal,
            "s_theta": decision.s_theta,
            "s_rd": decision.s_rd,
            "s_left_obs": decision.s_left_obs,
            "s_right_obs": decision.s_right_obs,
            "valley_start_idx": valley_start,
            "valley_end_idx": valley_end,
            "valley_width_sectors": valley_width,
            "valley_center_deg": valley_center_deg,
            "valley_min_clearance": valley_min_clearance,
            "vx_cmd": vx,
            "vy_cmd": vy,
            "vz_cmd": vz,
            "vxy_cmd": vxy,
            "yaw_rate_cmd": yaw_rate,
            "heading_body_deg": math.degrees(decision.desired_heading_body),
            "goal_heading_body_deg": math.degrees(goal_heading_body),
            "escape_active": now < self.escape_mode_until,
            "crashed_now": crashed_now,
            "stuck_now": stuck_now,
            "no_progress_now": no_progress_now,
            "crash_count_total": self.metrics_crash_count,
            "stuck_count_total": self.metrics_stuck_count,
        })

    def finish_metrics(self, success: bool, finish_reason: str = "") -> None:
        if not self.mission_started or self.metrics_finished or self.mission_start_time is None:
            return

        self.metrics_finished = True
        self.success = success
        self.mission_end_time = time.monotonic()

        _gx, _gy, dz, _goal_heading_body, goal_dist_xy = self.get_goal_in_body_frame()
        self.final_goal_error = math.sqrt(goal_dist_xy ** 2 + dz ** 2)
        total_time = self.mission_end_time - self.mission_start_time

        situation_counts = {}
        for row in self.trajectory_log:
            situation = row.get("situation", "UNKNOWN")
            situation_counts[situation] = situation_counts.get(situation, 0) + 1

        summary = {
            "success": self.success,
            "finish_reason": finish_reason,
            "goal_xyz": self.goal_xyz,
            "final_position": list(self.position),
            "final_goal_error_3d": self.final_goal_error,
            "final_goal_error_xy": goal_dist_xy,
            "final_goal_error_z_abs": abs(dz),
            "min_goal_dist_xy": self.min_goal_dist_xy,
            "crash_count": self.metrics_crash_count,
            "stuck_count": self.metrics_stuck_count,
            "total_time_s": total_time,
            "num_samples": len(self.trajectory_log),
            "situation_counts": situation_counts,
            "metrics_crash_distance": self.metrics_crash_distance,
            "metrics_stuck_speed_threshold": self.metrics_stuck_speed_threshold,
            "metrics_stuck_time_threshold": self.metrics_stuck_time_threshold,
            "metrics_no_progress_timeout": self.metrics_no_progress_timeout,
            "metrics_no_progress_epsilon": self.metrics_no_progress_epsilon,
        }

        summary_path = os.path.join(self.metrics_dir, "summary.json")
        csv_path = os.path.join(self.metrics_dir, "trajectory.csv")
        map_path = os.path.join(self.metrics_dir, "trajectory_map.png")

        with open(summary_path, "w") as f:
            json.dump(summary, f, indent=4)

        if self.trajectory_log:
            with open(csv_path, "w", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=self.trajectory_log[0].keys())
                writer.writeheader()
                writer.writerows(self.trajectory_log)

            self.save_trajectory_map(map_path)

        self.get_logger().info(f"Métricas guardadas en: {self.metrics_dir}")
        self.get_logger().info(f"Resumen métricas: {summary}")

    def save_trajectory_map(self, path: str) -> None:
        if not self.trajectory_log:
            return

        xs = [p["x"] for p in self.trajectory_log]
        ys = [p["y"] for p in self.trajectory_log]

        crash_xs = [p["x"] for p in self.trajectory_log if p.get("crashed_now")]
        crash_ys = [p["y"] for p in self.trajectory_log if p.get("crashed_now")]
        stuck_xs = [p["x"] for p in self.trajectory_log if p.get("stuck_now") or p.get("no_progress_now")]
        stuck_ys = [p["y"] for p in self.trajectory_log if p.get("stuck_now") or p.get("no_progress_now")]

        plt.figure(figsize=(8, 8))
        plt.plot(xs, ys, label="Trayectoria")
        plt.scatter([xs[0]], [ys[0]], marker="o", s=80, label="Inicio")
        plt.scatter([self.goal_xyz[0]], [self.goal_xyz[1]], marker="*", s=150, label="Meta")
        plt.scatter([xs[-1]], [ys[-1]], marker="x", s=100, label="Final")

        if crash_xs:
            plt.scatter(crash_xs, crash_ys, marker="X", s=90, label="Posible choque")
        if stuck_xs:
            plt.scatter(stuck_xs, stuck_ys, marker="s", s=70, label="Posible atasco")

        plt.axis("equal")
        plt.grid(True)
        plt.xlabel("X world [m]")
        plt.ylabel("Y world [m]")
        plt.title("Mapa de trayectoria ND")
        plt.legend()
        plt.tight_layout()
        plt.savefig(path, dpi=150)
        plt.close()

    # =====================================================
    # Logging
    # =====================================================

    def format_valley(self, valley: Optional[Valley]) -> str:
        if valley is None:
            return "None"
        return f"[{valley.start_idx}->{valley.end_idx}] width={self.valley_width_sectors(valley)}"

    def maybe_log_decision(
        self,
        decision: NDDecision,
        vx: float,
        vy: float,
        vz: float,
        yaw_rate: float,
    ) -> None:
        now = time.monotonic()
        if (now - self.last_log_time) < self.log_period:
            return
        self.last_log_time = now

        min_rnd = min(self.rnd)
        self.get_logger().info(
            "ND "
            f"sit={decision.situation} "
            f"reason='{decision.reason}' "
            f"goal_sector={decision.s_goal} "
            f"s_theta={decision.s_theta} "
            f"s_rd={decision.s_rd} "
            f"s_left_obs={decision.s_left_obs} "
            f"s_right_obs={decision.s_right_obs} "
            f"valley={self.format_valley(decision.valley)} "
            f"min_rnd={min_rnd:.2f} "
            f"closest_sector={self.closest_obs_sector} "
            f"heading_body={math.degrees(decision.desired_heading_body):.1f}deg "
            f"cmd=({vx:.2f},{vy:.2f},{vz:.2f}) "
            f"yaw_rate={yaw_rate:.2f}"
        )

    # =====================================================
    # Navegación
    # =====================================================

    def wait_for_scan(self, timeout: float = 5.0) -> bool:
        t0 = time.monotonic()
        while rclpy.ok() and (time.monotonic() - t0) < timeout:
            if self.scan_ready:
                return True
            time.sleep(0.05)
        return False

    def navigate(self, control_rate_hz: float = 10.0, timeout_s: float = 2200.0) -> bool:
        self.start_metrics()

        period = 1.0 / control_rate_hz
        t0 = time.monotonic()

        while rclpy.ok():
            if time.monotonic() - t0 > timeout_s:
                self.get_logger().error("Timeout de navegación")
                self.motion_ref_handler.hover()
                self.finish_metrics(success=False, finish_reason="timeout")
                return False

            if (not self.scan_ready) or ((time.monotonic() - self.last_scan_time) > self.scan_timeout):
                self.get_logger().warn("Sin LiDAR reciente -> hover")
                self.motion_ref_handler.hover()
                time.sleep(period)
                continue

            _gx, _gy, _dz, _goal_heading_body, goal_dist_xy = self.get_goal_in_body_frame()

            if time.monotonic() >= self.post_escape_ignore_until:
                self.update_stuck_history(goal_dist_xy)
                if self.is_stuck() and time.monotonic() > self.escape_mode_until:
                    progress = self.stuck_progress()
                    goal_progress = self.stuck_goal_progress()
                    self.get_logger().warn("ATASCO detectado -> activando escape (2.0 s)")
                    self.get_logger().warn(
                        f"Stuck window: progress={progress:.3f} m, goal_progress={goal_progress:.3f} m, "
                        f"samples={len(self.stuck_history)}, pose={self.position}, "
                        f"yaw={math.degrees(self.orientation[2]):.1f} deg"
                    )
                    self.escape_start = time.monotonic()
                    self.escape_mode_until = self.escape_start + 2.0
                    self.post_escape_ignore_until = self.escape_mode_until + 1.2
                    self.stuck_history.clear()

            vx, vy, vz, yaw_rate, goal_reached, decision = self.compute_command()

            self.update_metrics(decision, vx, vy, vz, yaw_rate)

            if goal_reached:
                self.get_logger().info("Meta alcanzada")
                self.motion_ref_handler.hover()
                self.finish_metrics(success=True, finish_reason=decision.reason)
                return True

            self.maybe_log_decision(decision, vx, vy, vz, yaw_rate)
            self.plot_debug(decision, vx, vy, vz, yaw_rate)

            try:
                self.motion_ref_handler.speed.send_speed_command_with_yaw_speed(
                    [float(vx), float(vy), float(vz)],
                    twist_frame_id="earth",
                    yaw_speed=float(yaw_rate),
                )
            except Exception as e:
                self.get_logger().error(f"Fallo enviando speed command: {e}")
                try:
                    self.motion_ref_handler.hover()
                except Exception:
                    pass
                self.finish_metrics(success=False, finish_reason="speed_command_error")
                return False

            time.sleep(period)

        try:
            self.motion_ref_handler.hover()
        except Exception:
            pass

        self.finish_metrics(success=False, finish_reason="rclpy_not_ok")
        return False


# ============================================================
# Spin seguro
# ============================================================

def safe_spin(node) -> None:
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    except Exception as e:
        try:
            node.get_logger().warn(f"Spin thread terminado con excepción: {e}")
        except Exception:
            pass


# ============================================================
# main
# ============================================================

def main() -> None:
    rclpy.init()

    drone = PragmaticNDDrone(
        drone_id="drone0",
        verbose=True,
        use_sim_time=True,
        enable_radar_plot=True,
    )

    spin_thread = threading.Thread(
        target=safe_spin,
        args=(drone,),
        daemon=True,
    )
    spin_thread.start()

    try:
        if not drone.wait_for_scan():
            drone.get_logger().error("No se recibió LiDAR al iniciar")
            return

        logger = drone.get_logger()

        logger.info("Armando...")
        arm_ok = drone.arm()
        logger.info(f"arm() -> {arm_ok}")
        if not arm_ok:
            logger.error("No se pudo armar")
            return

        logger.info("Enviando hover antes de offboard...")
        for _ in range(20):
            drone.motion_ref_handler.hover()
            time.sleep(0.05)

        logger.info("Entrando en offboard...")
        offboard_ok = drone.offboard()
        logger.info(f"offboard() -> {offboard_ok}")
        if not offboard_ok:
            logger.error("No se pudo entrar en offboard")
            return

        for _ in range(10):
            drone.motion_ref_handler.hover()
            time.sleep(0.05)

        logger.info(f"Posición actual: {drone.position}")
        logger.info(f"Orientación actual: {drone.orientation}")
        logger.info(f"Meta: {drone.goal_xyz}")

        logger.info("Despegando...")
        takeoff_ok = drone.takeoff(height=drone.takeoff_height, speed=1.0)
        logger.info(f"takeoff() -> {takeoff_ok}")
        if not takeoff_ok:
            logger.error("No se pudo despegar")
            return

        logger.info("Despegue completado. Iniciando navegación ND pragmática con métricas")
        ok = drone.navigate(control_rate_hz=10.0, timeout_s=2200.0)
        logger.info(f"Navigation success: {ok}")

        try:
            drone.motion_ref_handler.hover()
        except Exception:
            pass

        try:
            drone.land(speed=0.5)
        except Exception as e:
            drone.get_logger().warn(f"land() falló: {e}")

        try:
            drone.manual()
        except Exception as e:
            drone.get_logger().warn(f"manual() falló: {e}")

    except KeyboardInterrupt:
        drone.get_logger().warn("Interrumpido por teclado")
        try:
            drone.motion_ref_handler.hover()
        except Exception:
            pass
        drone.finish_metrics(success=False, finish_reason="keyboard_interrupt")

    except Exception as e:
        drone.get_logger().error(f"Excepción en main: {e}")
        drone.finish_metrics(success=False, finish_reason=f"exception: {e}")

    finally:
        if drone.mission_started and not drone.metrics_finished:
            drone.finish_metrics(success=False, finish_reason="main_finally_before_shutdown")

        if drone.enable_radar_plot:
            try:
                plt.ioff()
                plt.show()
            except Exception:
                pass

        try:
            drone.shutdown()
        except Exception:
            pass

        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == "__main__":
    main()
