"""Overlay VAMP's collision spheres on a robot's visual URDF."""

import time
from pathlib import Path
from typing import Optional, Sequence

from fire import Fire

import vamp
from vamp import pybullet_interface as vpb


def main(robot: str = "g1_up", configuration: Optional[Sequence[float]] = None) -> None:
    if robot not in vamp.robots:
        raise ValueError(f"Unknown VAMP robot '{robot}'. Available robots: {vamp.robots}")

    vamp_robot = getattr(vamp, robot)
    q = (
        list(configuration)
        if configuration is not None
        else vamp_robot.default_configuration().tolist()
    )
    if len(q) != vamp_robot.dimension():
        raise ValueError(f"Expected {vamp_robot.dimension()} coordinates, received {len(q)}.")

    robot_dir = Path(__file__).parents[1] / "resources" / robot
    simulator = vpb.PyBulletSimulator(
        str(robot_dir / f"{robot}_spherized.urdf"),
        vamp_robot.joint_names(),
        True,
    )
    simulator.set_joint_positions(q)

    for sphere in vamp_robot.fk(q):
        simulator.add_sphere(
            sphere.r,
            [sphere.x, sphere.y, sphere.z],
            color=[1.0, 0.2, 0.1, 0.35],
        )

    simulator.set_camera((2.2, -2.2, 1.6), (0.0, 0.0, 0.8))
    print("Collision spheres are overlaid in translucent red. Close the PyBullet window to exit.")
    while simulator.client.isConnected():
        time.sleep(0.1)


if __name__ == "__main__":
    Fire(main)
