"""Record an actual CanSat drive with OmniLRS sand deformation and a moving camera."""

import argparse
import json
import math
import os
from pathlib import Path
import sys
import traceback

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def record(args, manager, app, plain_cfg):
    import cv2
    import numpy as np
    import omni.usd
    from geometry_msgs.msg import Twist
    from isaacsim.sensors.camera import Camera
    from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics, UsdShade

    world = manager.world
    ros = manager.ROSRobotManager
    robot = ros.RM.robot
    environment = manager.ROSEnvironmentManager.EC
    terrain = environment.T
    stage = omni.usd.get_context().get_stage()
    output = args.output.resolve()
    frames_expected = round(args.duration * args.fps)
    dt = world.get_physics_dt()
    substeps = round(1 / (dt * args.fps))
    if not math.isclose(substeps * dt, 1 / args.fps, rel_tol=1e-7):
        raise ValueError("Physics timestep must divide the video frame interval exactly.")

    # OmniLRS deforms the rendered DEM while preserving the initial collider.
    # Keep collision on an immutable mesh to avoid repeated PhysX mesh cooking.
    collision_path = terrain._root_path + "/Terrain/pv_collision"
    Sdf.CopySpec(stage.GetRootLayer(), terrain._mesh_path, stage.GetRootLayer(), collision_path)
    UsdGeom.Imageable(stage.GetPrimAtPath(collision_path)).CreateVisibilityAttr("invisible")
    UsdPhysics.CollisionAPI(stage.GetPrimAtPath(terrain._mesh_path)).CreateCollisionEnabledAttr(False)
    if stage.GetPrimAtPath("/Lunaryard/Earth"):
        stage.GetPrimAtPath("/Lunaryard/Earth").SetActive(False)

    # Grazing warm key light reveals the tyre grooves; cool fill reveals the PCB.
    sun = UsdLux.DistantLight(stage.GetPrimAtPath("/Lunaryard/Sun/sun"))
    sun.CreateIntensityAttr(1800.0)
    sun.CreateColorAttr(Gf.Vec3f(1.0, 0.87, 0.70))
    sun.CreateAngleAttr(4.0)
    UsdGeom.Xformable(stage.GetPrimAtPath("/Lunaryard/Sun")).ClearXformOpOrder()
    rotation = (
        Gf.Matrix4d()
        .SetLookAt(Gf.Vec3d(-1, -1, 0.65), Gf.Vec3d(0), Gf.Vec3d(0, 0, 1))
        .GetInverse()
        .ExtractRotationQuat()
    )
    UsdGeom.Xformable(sun).ClearXformOpOrder()
    UsdGeom.Xformable(sun).AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(rotation)
    fill = UsdLux.DomeLight.Define(stage, "/PV/FillLight")
    fill.CreateIntensityAttr(120.0)
    fill.CreateColorAttr(Gf.Vec3f(0.75, 0.82, 1.0))
    shader = UsdShade.Shader(stage.GetPrimAtPath("/Lunaryard/Looks/Sand/Shader"))
    if shader:
        shader.CreateInput("albedo_brightness", Sdf.ValueTypeNames.Float).Set(0.7)
        shader.CreateInput("diffuse_tint", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.92, 0.80, 0.62))
        shader.CreateInput("texture_scale", Sdf.ValueTypeNames.Float2).Set(Gf.Vec2f(5, 5))

    camera = Camera("/PV/Camera", resolution=(args.width, args.height))
    camera.initialize()
    camera.set_focal_length(3.2)
    camera.set_horizontal_aperture(3.6)
    camera.set_clipping_range(0.01, 100.0)
    initial_dem = terrain.getDEM().copy()
    writer = cv2.VideoWriter(str(output), cv2.VideoWriter_fourcc(*"mp4v"), args.fps, (args.width, args.height))
    if not writer.isOpened():
        raise RuntimeError("MP4 encoder could not be opened.")
    trajectory = []
    frames_written = 0
    command = Twist()

    def camera_pose(position, progress):
        # A continuous dolly move around the right side, ending on the tracks.
        smooth = progress * progress * (3 - 2 * progress)
        offset = np.array([0.30, -0.45, 0.38]) * (1 - smooth) + np.array([-0.44, -0.52, 0.46]) * smooth
        target = np.asarray(position) + np.array([-0.10 * smooth, 0.0, 0.015])
        eye = np.asarray(position) + offset
        quat = (
            Gf.Matrix4d()
            .SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*target), Gf.Vec3d(0, 0, 1))
            .GetInverse()
            .ExtractRotationQuat()
        )
        camera.set_world_pose(eye, np.array([quat.GetReal(), *quat.GetImaginary()]), camera_axes="usd")

    try:
        manager.timeline.play()
        for step in range(round(3 / dt)):
            world.step(render=False)
            robot.update_articulation_api()
            if step % substeps == 0:
                world.render()
        if not robot.is_articulation_api_ready():
            raise RuntimeError("CanSat articulation is not ready.")
        position, _ = ros.RM.robot_RG.get_pose_of_base_link()
        camera_pose(position, 0)
        for _ in range(24):
            world.render()
        start = np.asarray(position).copy()
        for frame in range(frames_expected):
            if not app.is_running():
                raise RuntimeError("Isaac Sim ended before the recording completed.")
            # Gentle acceleration, then steady forward travel for the whole clip.
            time_s = frame / args.fps
            command.linear.x = args.speed * min(1.0, (time_s + 0.25) / 1.25)
            for _ in range(substeps):
                robot.update_articulation_api()
                ros.receive_velocity(command)
                ros.apply_modifications()
                world.step(render=False)
            # Deform via the actual two wheel poses and measured contact forces.
            # USD poses are synchronized on render; deformation uses that state.
            world.render()
            environment.deform_terrain()
            position, orientation = ros.RM.robot_RG.get_pose_of_base_link()
            if not np.isfinite(position).all():
                raise RuntimeError("Invalid rover physics state.")
            camera_pose(position, frame / max(1, frames_expected - 1))
            world.render()
            rgba = camera.get_rgba()
            if rgba is None or rgba.shape != (args.height, args.width, 4):
                raise RuntimeError(f"Camera image unavailable at frame {frame}.")
            image = cv2.cvtColor(rgba, cv2.COLOR_RGBA2BGR)
            writer.write(image)
            frames_written += 1
            trajectory.append(
                dict(
                    time_s=time_s, position=np.asarray(position).tolist(), orientation=np.asarray(orientation).tolist()
                )
            )
            if frame in (0, frames_expected // 2, frames_expected - 1):
                cv2.imwrite(str(output.with_name(f"{output.stem}_frame_{frame:04d}.jpg")), image)
            if frame % 30 == 0:
                print(
                    f"PV {frame + 1}/{frames_expected}, position={np.round(position, 3).tolist()}, wheels={robot.get_joint_velocities()}",
                    flush=True,
                )
        displacement = float(np.linalg.norm(np.asarray(position)[:2] - start[:2]))
        changed = terrain.getDEM() - initial_dem
        deformed_cells = int(np.count_nonzero(np.abs(changed) > 1e-6))
        if displacement < args.speed * args.duration * 0.5:
            raise RuntimeError(f"Rover did not travel enough: {displacement:.3f} m.")
        if deformed_cells == 0:
            raise RuntimeError("Sand geometry did not deform.")
        if changed.min() >= -1e-4:
            raise RuntimeError("Tyre tracks must indent the sand, rather than raise it.")
        metadata = dict(
            status="complete",
            frames=frames_written,
            fps=args.fps,
            duration_s=frames_written / args.fps,
            resolution=[args.width, args.height],
            speed_m_s=args.speed,
            distance_m=displacement,
            deformation_enabled=True,
            deformed_cells=deformed_cells,
            min_height_change_m=float(changed.min()),
            max_height_change_m=float(changed.max()),
            collision_geometry="immutable initial mesh; wheel contacts deform the visible sand DEM",
            lighting="warm grazing distant key and cool dome fill",
            camera="continuous following dolly",
            trajectory=trajectory,
        )
        output.with_suffix(".json").write_text(json.dumps(metadata, indent=2) + "\n")
        np.save(output.with_name(output.stem + "_sand_delta.npy"), changed)
        print(
            f"PV_COMPLETE: {output}, {frames_written} frames, {displacement:.3f} m, {deformed_cells} deformed cells",
            flush=True,
        )
    finally:
        writer.release()
        robot.stop_drive()
        world.stop()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--speed", type=float, default=0.06)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--output", type=Path, default=ROOT / "recordings/cansat_pv_20s.mp4")
    args = parser.parse_args()
    if args.duration <= 0 or args.fps <= 0 or args.speed <= 0:
        parser.error("Duration, FPS and speed must be positive.")
    if args.output.exists():
        parser.error(f"Output already exists: {args.output}")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    os.chdir(ROOT)
    from hydra import compose, initialize_config_dir
    from run import instantiateConfigs, omegaconfToDict
    from src.environments_wrappers import startSim

    domain = int(os.environ.get("ROS_DOMAIN_ID", "42"))
    with initialize_config_dir(config_dir=str(ROOT / "cfg"), version_base="1.3"):
        cfg = compose(
            config_name="config",
            overrides=[
                "environment=cansat_pv",
                "robot=cansat",
                "mode=ROS2",
                f"mode.ROS_DOMAIN_ID={domain}",
                f"environment.robots_settings.parameters.domain_id={domain}",
                "environment.robots_settings.parameters.pose.position=[1.0,2.0,0.15]",
                "rendering.renderer.headless=true",
                "rendering.renderer.samples_per_pixel_per_frame=32",
            ],
        )
    plain_cfg = omegaconfToDict(cfg)
    manager, app = startSim(instantiateConfigs(plain_cfg))
    try:
        record(args, manager, app, plain_cfg)
    except Exception:
        traceback.print_exc()
        raise
    finally:
        import rclpy

        for executor, thread in [(manager.exec1, manager.exec1_thread), (manager.exec2, manager.exec2_thread)]:
            executor.shutdown()
            thread.join(timeout=5)
        manager.ROSEnvironmentManager.destroy_node()
        manager.ROSRobotManager.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        app.close()


if __name__ == "__main__":
    main()
