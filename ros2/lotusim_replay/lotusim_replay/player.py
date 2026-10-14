"""ROS 2 node that feeds verified frozen poses to the LOTUSim renderer."""

import argparse
import time
from pathlib import Path

from .core import interpolate, load_verified_payload, lotusim_pose, select_arms, select_scene


def _arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, default=Path.cwd())
    parser.add_argument("--scene", default="double_baffle_detour")
    parser.add_argument("--arm", action="append", choices=("P1", "APPROACH"), dest="arms")
    parser.add_argument("--rate", type=float, default=1.0, help="Wall-clock playback multiplier")
    parser.add_argument("--publish-hz", type=float, default=30.0)
    parser.add_argument("--loop", action="store_true")
    parser.add_argument("--lateral-separation", type=float, default=0.0)
    parser.add_argument("--renderer-type", default="lrauv")
    parser.add_argument("--namespace", default="/lotusim")
    parser.add_argument(
        "--validate-only",
        action="store_true",
        help="Verify and summarize frozen input without importing ROS 2",
    )
    return parser.parse_args(argv)


def main(argv=None):
    args = _arguments(argv)
    if args.rate <= 0.0 or args.publish_hz <= 0.0:
        raise SystemExit("--rate and --publish-hz must be positive")

    # Verify all frozen source hashes before importing ROS or publishing.
    payload = load_verified_payload(args.repo_root)
    scene = select_scene(payload, args.scene)
    arms = select_arms(scene, args.arms or ("APPROACH",))
    if args.validate_only:
        details = ", ".join(
            f"{name}={len(data['samples'])} samples/{data['status']}"
            for name, data in arms.items()
        )
        print(f"Verified LOTUSim replay: {args.scene}; {details}")
        return

    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from lotusim_msgs.msg import RendererCmd, VesselPosition, VesselPositionArray
    except ImportError as error:
        raise SystemExit(
            "ROS 2/lotusim_msgs is unavailable. Source the LOTUSim ROS workspace first."
        ) from error

    namespace = "/" + args.namespace.strip("/")
    rclpy.init(args=None)
    node = Node("frozen_uuv_lotusim_replay")
    qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=max(10, len(arms)),
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    command_pub = node.create_publisher(RendererCmd, namespace + "/renderer_cmd", qos)
    pose_pub = node.create_publisher(VesselPositionArray, namespace + "/renderer_poses", qos)
    names = {arm: f"uuv_replay_{arm.lower()}" for arm in arms}
    offsets = {
        arm: (index - (len(arms) - 1) / 2.0) * args.lateral_separation
        for index, arm in enumerate(arms)
    }

    def fill_pose(target, pose):
        target.position.x, target.position.y, target.position.z = pose.x, pose.y, pose.z
        target.orientation.x, target.orientation.y = pose.qx, pose.qy
        target.orientation.z, target.orientation.w = pose.qz, pose.qw

    def publish_commands(command_type):
        for arm, data in arms.items():
            command = RendererCmd()
            command.cmd_type = command_type
            command.renderer_obj_name = args.renderer_type
            command.vessel_name = names[arm]
            fill_pose(command.vessel_position, lotusim_pose(data["samples"][0], offsets[arm]))
            command_pub.publish(command)

    starts = [data["samples"][0][0] for data in arms.values()]
    ends = [data["samples"][-1][0] for data in arms.values()]
    source_start, source_end = min(starts), max(ends)
    publish_commands(RendererCmd.CREATE_CMD)
    # Give transient-local discovery a short spin before the first pose array.
    deadline = time.monotonic() + 0.25
    while time.monotonic() < deadline and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.01)

    wall_start = time.monotonic()
    period = 1.0 / args.publish_hz
    try:
        while rclpy.ok():
            elapsed = (time.monotonic() - wall_start) * args.rate
            source_time = source_start + elapsed
            if source_time > source_end:
                if args.loop:
                    wall_start = time.monotonic()
                    source_time = source_start
                else:
                    source_time = source_end

            message = VesselPositionArray()
            stamp = node.get_clock().now().to_msg()
            message.header.stamp = stamp
            message.header.frame_id = "world"
            for arm, data in arms.items():
                item = VesselPosition()
                item.vessel_name = names[arm]
                item.model_name = args.renderer_type
                item.header.stamp = stamp
                item.header.frame_id = "world"
                pose = lotusim_pose(interpolate(data["samples"], source_time), offsets[arm])
                fill_pose(item.pose, pose)
                message.vessels.append(item)
            pose_pub.publish(message)
            rclpy.spin_once(node, timeout_sec=0.0)
            if source_time >= source_end and not args.loop:
                break
            time.sleep(period)
        node.get_logger().info(
            f"Frozen replay complete: {args.scene}, {', '.join(arms)}, source t={source_end:g}s"
        )
        # Preserve the final pose until interrupted, matching LOTUSim's renderer behavior.
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        publish_commands(RendererCmd.DELETE_CMD)
        rclpy.spin_once(node, timeout_sec=0.1)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

