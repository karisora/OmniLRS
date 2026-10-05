"""Record a finite headless ROS 2 simulation from an external camera."""

import argparse
import json
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=15.0)
    parser.add_argument("--speed", type=float, default=0.2)
    parser.add_argument("--output", type=Path,
                        default=ROOT / "recordings/lunaryard_forward_15s.mp4")
    args, overrides = parser.parse_known_args()
    if args.duration <= 0 or args.speed <= 0:
        parser.error("Duration and forward speed must be positive.")
    output = args.output.resolve()
    if output.exists():
        parser.error(f"Output already exists: {output}")
    output.parent.mkdir(parents=True, exist_ok=True)

    from hydra import compose, initialize_config_dir
    from run import instantiateConfigs, omegaconfToDict
    from src.environments_wrappers import startSim

    domain = int(os.environ.get("ROS_DOMAIN_ID", "42"))
    with initialize_config_dir(config_dir=str(ROOT / "cfg"), version_base="1.3"):
        cfg = compose(config_name="config", overrides=[
            "environment=lunaryard_40m", "mode=ROS2",
            f"mode.ROS_DOMAIN_ID={domain}",
            f"++environment.robots_settings.parameters.domain_id={domain}",
            "environment.terrain_manager.moon_yard.deformation_engine.enable=true",
            "rendering.renderer.headless=true",
            "environment.enforce_realtime=false",
            *overrides,
        ])
    plain_cfg = omegaconfToDict(cfg)
    manager, app = startSim(instantiateConfigs(plain_cfg))

    import cv2
    import numpy as np
    import rclpy
    import omni.usd
    from geometry_msgs.msg import Twist
    from isaacsim.sensors.camera import Camera
    from pxr import Gf, Sdf, UsdGeom, UsdPhysics

    node = rclpy.create_node("omnilrs_video_forward")
    publisher = node.create_publisher(Twist, "/cmd_vel", 10)
    robot = manager.ROSRobotManager.RM.robot
    robot_path = robot.robot_path
    graph_path = f"{robot_path}/Graphs/differential_controller"
    # Use the native articulation API to apply received ROS Twist commands.
    # Disable the legacy graph for this session so it cannot overwrite targets.
    omni.usd.get_context().get_stage().GetPrimAtPath(graph_path).SetActive(False)
    wheel_radius = 0.165
    wheel_distance = 0.545
    received_speed = 0.0

    def drive_callback(message):
        nonlocal received_speed
        received_speed = message.linear.x
        left = (message.linear.x - message.angular.z * wheel_distance / 2) / wheel_radius
        right = (message.linear.x + message.angular.z * wheel_distance / 2) / wheel_radius
        if robot.is_articulation_api_ready():
            robot.set_joint_velocities({
                "front_left_wheel": left, "rear_left_wheel": left,
                "front_right_wheel": right, "rear_right_wheel": right,
            })

    subscription = node.create_subscription(Twist, "/cmd_vel", drive_callback, 10)
    # The environment intends to leave collision geometry unchanged during
    # deformation (update_collider=False). A separate invisible copy prevents
    # visual point updates from repeatedly scheduling PhysX mesh cooking.
    terrain = manager.ROSEnvironmentManager.EC.T
    stage = omni.usd.get_context().get_stage()
    collider_path = f"{terrain._root_path}/Terrain/recording_collision_mesh"
    Sdf.CopySpec(stage.GetRootLayer(), terrain._mesh_path,
                 stage.GetRootLayer(), collider_path)
    UsdGeom.Imageable(stage.GetPrimAtPath(collider_path)).CreateVisibilityAttr("invisible")
    UsdPhysics.CollisionAPI(stage.GetPrimAtPath(terrain._mesh_path)).GetCollisionEnabledAttr().Set(False)
    camera = Camera("/RecordingCamera", resolution=(1280, 720))
    camera.initialize()
    # Camera API values are one tenth of USD's millimetre-style values.
    camera.set_focal_length(2.4)
    camera.set_clipping_range(0.1, 1000.0)
    start_position, _ = manager.ROSRobotManager.RM.robot_RG.get_pose_of_base_link()
    camera_position = np.asarray(start_position) + [-4.0, -5.0, 3.5]
    camera_target = np.asarray(start_position) + [1.5, 0.0, 0.25]
    look_at = Gf.Matrix4d().SetLookAt(
        Gf.Vec3d(*camera_position), Gf.Vec3d(*camera_target), Gf.Vec3d(0, 0, 1)
    ).GetInverse().ExtractRotationQuat()
    camera.set_world_pose(camera_position,
                          np.array([look_at.GetReal(), *look_at.GetImaginary()]),
                          camera_axes="usd")

    dt = manager.world.get_physics_dt()
    frame_count = round(args.duration / dt)
    fps = frame_count / args.duration
    writer = cv2.VideoWriter(str(output), cv2.VideoWriter_fourcc(*"mp4v"),
                             fps, (1280, 720))
    if not writer.isOpened():
        raise RuntimeError("Could not open the MP4 encoder.")
    warmup_steps = round(3.0 / dt)
    steps = 0
    frames = 0
    trajectory = []
    original_step = manager.world.step
    original_is_running = app.is_running
    command = Twist()

    def recorded_step(*step_args, **step_kwargs):
        nonlocal steps, frames
        command.linear.x = args.speed if steps >= warmup_steps else 0.0
        robot.update_articulation_api()
        publisher.publish(command)
        rclpy.spin_once(node, timeout_sec=0.0)
        # Advance one physics timestep explicitly, then render that state.
        # Kit app.update() alone may skip physics while USD changes are cooking.
        step_kwargs["render"] = False
        result = original_step(*step_args, **step_kwargs)
        manager.world.render()
        steps += 1
        if steps <= warmup_steps:
            return result
        if publisher.get_subscription_count() == 0:
            raise RuntimeError("No ROS 2 subscriber matched /cmd_vel.")
        rgba = camera.get_rgba()
        if rgba is None or rgba.shape != (720, 1280, 4):
            raise RuntimeError(f"Camera frame unavailable at step {steps}.")
        bgr = cv2.cvtColor(rgba, cv2.COLOR_RGBA2BGR)
        writer.write(bgr)
        position, orientation = robot.telemetry.articulation.get_world_pose()
        trajectory.append({"time": frames * dt, "world_time": manager.world.current_time,
                           "position": position.tolist(),
                           "orientation": orientation.tolist()})
        if frames in (0, frame_count // 2, frame_count - 1):
            cv2.imwrite(str(output.with_name(f"{output.stem}_frame_{frames:04d}.jpg")), bgr)
        frames += 1
        if frames % 30 == 0:
            print(f"RECORDING: {frames}/{frame_count} frames, position={position.tolist()}, "
                  f"received={received_speed}, joints={robot.get_joint_velocities()}", flush=True)
        return result

    manager.world.step = recorded_step
    app.is_running = lambda: frames < frame_count and original_is_running()
    try:
        print(f"RECORDING_SETUP: headless, deformation enabled, speed={args.speed}, output={output}", flush=True)
        manager.run_simulation()
    finally:
        manager.world.step = original_step
        app.is_running = original_is_running
        publisher.publish(Twist())
        writer.release()
        node.destroy_node()
        for executor, thread in ((manager.exec1, manager.exec1_thread),
                                 (manager.exec2, manager.exec2_thread)):
            executor.shutdown()
            thread.join(timeout=5.0)
        manager.ROSEnvironmentManager.destroy_node()
        manager.ROSRobotManager.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        output.with_suffix(".json").write_text(json.dumps({
            "frames": frames, "fps": fps, "duration": frames / fps,
            "physics_dt": dt, "command_speed": args.speed,
            "headless": plain_cfg["rendering"]["renderer"]["headless"],
            "environment": plain_cfg["environment"]["name"],
            "deformation_enabled": manager.enable_deformation,
            "drive_control": "ROS2 Twist subscriber to native articulation wheel velocity targets",
            "collision_geometry": "Separate immutable copy of initial terrain, deformation updates visual mesh",
            "trajectory": trajectory,
        }, indent=2))
        print(f"RECORDING_SAVED: {output}, {frames} frames", flush=True)
        app.close()
    if frames != frame_count:
        raise RuntimeError(f"Recording ended early: {frames}/{frame_count} frames")


if __name__ == "__main__":
    main()
