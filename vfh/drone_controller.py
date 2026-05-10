import math
import csv
from pathlib import Path
from time import sleep, monotonic

import matplotlib.pyplot as plt
import rclpy

from as2_python_api.drone_interface_teleop import DroneInterfaceTeleop

from utils import clamp, wrap_pi
from lidar_processor import LidarProcessor
from vfh_planner import VFHPlanner


class VFHDebugPlotter:
    def __init__(self, hist_bins, threshold):
        self.hist_bins = hist_bins
        self.threshold = threshold
        self.step = 0

        self.angles = [
            -math.pi + (i + 0.5) * (2.0 * math.pi / hist_bins)
            for i in range(hist_bins)
        ]

        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(12, 5))

        self.raw_line, = self.ax.plot(self.angles, [0]*hist_bins, label="raw", alpha=0.4)
        self.smooth_line, = self.ax.plot(self.angles, [0]*hist_bins, label="smooth", linewidth=2)

        self.th_line = self.ax.axhline(self.threshold, linestyle="--", label="threshold")

        self.goal_line = self.ax.axvline(0, linestyle="--", label="goal")
        self.desired_line = self.ax.axvline(0, linestyle="-", label="desired")
        self.yaw_line = self.ax.axvline(0, linestyle=":", label="yaw")

        # NUEVAS líneas clave
        self.kn_line = self.ax.axvline(0, linestyle=":", color="green", label="k_n")
        self.kf_line = self.ax.axvline(0, linestyle=":", color="red", label="k_f")

        self.text_box = self.ax.text(
            0.02, 0.95, "", transform=self.ax.transAxes, va="top"
        )

        self.ax.set_xlim(-math.pi, math.pi)
        self.ax.set_xlabel("angle [rad]")
        self.ax.set_ylabel("polar obstacle density")
        self.ax.legend()
        self.ax.grid(True)

        self.free_fill = None
        self.valley_fill = None

        plt.show(block=False)

    def update(self,
               raw_hist,
               smooth_hist,
               yaw,
               goal_heading,
               desired_heading,
               k_targ=None,
               k_n=None,
               k_f=None,
               free_bins=None,
               valley_bins=None,
               diversion_mode=0,
               trap=False,
               speed=0.0,
               yaw_rate=0.0):

        # -------- curvas --------
        self.raw_line.set_ydata(raw_hist)
        self.smooth_line.set_ydata(smooth_hist)

        max_val = max(max(smooth_hist), self.threshold, 1e-6)
        self.ax.set_ylim(0, max_val * 1.2)

        # -------- líneas --------
        self.goal_line.set_xdata([goal_heading, goal_heading])
        self.desired_line.set_xdata([desired_heading, desired_heading])
        self.yaw_line.set_xdata([yaw, yaw])

        # k_n y k_f
        if k_n is not None:
            self.kn_line.set_xdata([self.angles[k_n], self.angles[k_n]])
        if k_f is not None:
            self.kf_line.set_xdata([self.angles[k_f], self.angles[k_f]])

        # -------- limpiar fills previos --------
        if self.free_fill:
            self.free_fill.remove()
        if self.valley_fill:
            self.valley_fill.remove()

        # -------- sectores libres --------
        if free_bins is not None:
            mask = [smooth_hist[i] < self.threshold for i in range(self.hist_bins)]
            self.free_fill = self.ax.fill_between(
                self.angles,
                0,
                smooth_hist,
                where=mask,
                alpha=0.15,
                label="free"
            )

        # -------- valley seleccionado --------
        if valley_bins is not None:
            mask = [i in valley_bins for i in range(self.hist_bins)]
            self.valley_fill = self.ax.fill_between(
                self.angles,
                0,
                smooth_hist,
                where=mask,
                alpha=0.3,
                color="yellow",
                label="selected valley"
            )

        # -------- texto debug --------
        self.text_box.set_text(
            f"step={self.step}\n"
            f"diversion={diversion_mode}\n"
            f"trap={trap}\n"
            f"speed={speed:.2f}\n"
            f"yaw_rate={yaw_rate:.2f}"
        )

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

        self.step += 1


class VFHDrone(DroneInterfaceTeleop):
    def __init__(self, drone_id, config, verbose=False, use_sim_time=True, debug_plot=False):
        super().__init__(drone_id=drone_id, verbose=verbose, use_sim_time=use_sim_time)

        self.config = config
        self.debug_plot = debug_plot

        self.lidar = LidarProcessor(
            self,
            config["influence_distance"],
            config["body_exclusion_radius"],
            config["point_z_filter"],
        )

        self.planner = VFHPlanner(config)

        self.debugger = None
        if self.debug_plot:
            self.debugger = VFHDebugPlotter(
                hist_bins=config["hist_bins"],
                threshold=config["threshold"],
            )
        
        self.min_height = config["min_height"]

    def _goal_reached(self, pos, goal_xyz):
        dx = goal_xyz[0] - pos[0]
        dy = goal_xyz[1] - pos[1]
        dz = goal_xyz[2] - pos[2]
        return (
            math.hypot(dx, dy) <= self.config["goal_tol_xy"]
            and abs(dz) <= self.config["goal_tol_z"]
        )

    def navigate_to_goal(self, goal_xyz, timeout_s=480.0, control_rate_hz=10.0):
        period = 1.0 / control_rate_hz
        deadline = monotonic() + timeout_s

        self.planner.reset()

        while rclpy.ok():
            if monotonic() > deadline:
                self.motion_ref_handler.hover()
                return False

            pos = self.position
            yaw = self.orientation[2]

            if self._goal_reached(pos, goal_xyz):
                self.motion_ref_handler.hover()
                self.get_logger().info("Goal reached!")
                return True

            dx = goal_xyz[0] - pos[0]
            dy = goal_xyz[1] - pos[1]
            dz = goal_xyz[2] - pos[2]

            dist_xy = math.hypot(dx, dy)
            goal_heading = math.atan2(dy, dx)

            measurements, source = self.lidar.get_measurements(self.config["sensor_timeout"])

            if measurements is None:
                self.motion_ref_handler.hover()
                sleep(period)
                continue

            raw_hist = self.planner.build_histogram(
                position=pos,
                yaw=yaw,
                measurements=measurements,
            )
            smooth_hist = self.planner.smooth_histogram(raw_hist)

            desired_heading = self.planner.select_heading(
                yaw=yaw,
                goal_heading=goal_heading,
                histogram=smooth_hist,
            )

            if self.planner.trap_detected:
                self.get_logger().warning(
                    "VFH trap/no-valley condition detected. Navigation aborted."
                )
                self.motion_ref_handler.hover()
                return False

            if self.debugger is not None:
                self.debugger.update(
                    raw_hist=raw_hist,
                    smooth_hist=smooth_hist,
                    yaw=yaw,
                    goal_heading=goal_heading,
                    desired_heading=desired_heading,
                )

            heading_error = wrap_pi(desired_heading - yaw)

            # si no tienes velocidad angular real del dron, usa una aproximación conservadora
            omega_est = min(abs(heading_error) * control_rate_hz, self.config["yaw_rate_max"])

            speed_xy, yaw_rate = self.planner.compute_speed(
                dist_to_goal=dist_xy,
                desired_heading=desired_heading,
                current_yaw=yaw,
                histogram=smooth_hist,
                config=self.config,
                omega_actual=omega_est,
            )

            vx = speed_xy * math.cos(desired_heading)
            vy = speed_xy * math.sin(desired_heading)
            vz = clamp(dz, -self.config["v_max_z"], self.config["v_max_z"])

            if max(smooth_hist) < 1.0:
                self.motion_ref_handler.speed.send_speed_command_with_yaw_speed(
                    [vx, vy, vz], twist_frame_id="earth", yaw_speed=0.0  # sin rotación hasta tener lidar
                )
                sleep(period)
                continue

            self.motion_ref_handler.speed.send_speed_command_with_yaw_speed(
                [vx, vy, vz],
                twist_frame_id="earth",
                yaw_speed=float(yaw_rate),
            )

            sleep(period)

        return False