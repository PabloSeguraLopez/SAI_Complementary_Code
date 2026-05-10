#!/usr/bin/env python3

import argparse
import rclpy

from drone_controller import VFHDrone
from config import DEFAULTS


def drone_start(drone, takeoff_height, takeoff_speed):
    if not drone.arm():
        return False

    if not drone.offboard():
        return False

    return drone.takeoff(height=takeoff_height, speed=takeoff_speed)


def drone_end(drone, land_speed):
    drone.motion_ref_handler.hover()

    if not drone.land(speed=land_speed):
        return False

    return drone.manual()


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument("-n", "--namespace", default="drone0")
    parser.add_argument("--goal", type=float, nargs=3, default=[12.0, 0.0, 1.5])

    args = parser.parse_args()

    rclpy.init()

    drone = VFHDrone(
        drone_id=args.namespace,
        config=DEFAULTS,
        debug_plot=True,
    )

    success = drone_start(drone, 1.5, 1.0)

    if success:
        success = drone.navigate_to_goal(args.goal)

    drone_end(drone, 0.5)

    drone.shutdown()
    rclpy.shutdown()


if __name__ == "__main__":
    main()