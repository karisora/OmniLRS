"""Validate CanSat USD offline; --simulate also checks locomotion in Isaac Sim."""

import argparse
import hashlib
import json
from pathlib import Path
import sys

from convert_cansat import ROOT, ensure_usd


def check_asset():
    import numpy as np
    from omegaconf import OmegaConf
    from pxr import Gf, Usd, UsdGeom, UsdPhysics

    asset_dir = ROOT / "assets/USD_Assets/robots/cansat"
    manifest = json.loads((asset_dir / "manifest.json").read_text())
    config = OmegaConf.load(ROOT / "cfg/robot/cansat.yaml").robots_settings.parameters
    assert (
        manifest["source_sha256"] == hashlib.sha256((ROOT / "assets/cansat.fbx").read_bytes()).hexdigest()
    ), "FBX changed: regenerate USD."
    assert abs(manifest["wheel_radius_m"] - config.differential_drive.wheel_radius) < 1e-7
    assert abs(manifest["wheel_track_m"] - config.differential_drive.wheel_track) < 1e-7
    assert manifest["settings"]["max_wheel_speed_rad_s"] == config.differential_drive.max_wheel_speed
    assert manifest["settings"] == json.loads(
        (ROOT / "cfg/robot/cansat_physics.json").read_text()
    ), "Physics settings changed: regenerate USD."
    stage = Usd.Stage.CreateInMemory()
    robot = UsdGeom.Xform.Define(stage, "/Robots/cansat").GetPrim()
    robot.GetReferences().AddReference(str(ROOT / config.usd_path))
    assert robot.HasAPI(UsdPhysics.ArticulationRootAPI)
    assert UsdGeom.GetStageMetersPerUnit(Usd.Stage.Open(str(ROOT / config.usd_path))) == 1
    bodies = [p for p in stage.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)]
    assert {p.GetName() for p in bodies} == {"base_link", "left_wheel_link", "right_wheel_link"}
    for body in bodies:
        assert body.IsA(UsdGeom.Xform)
        assert UsdPhysics.MassAPI(body).GetMassAttr().Get() > 0
    joints = [p for p in stage.Traverse() if p.IsA(UsdPhysics.RevoluteJoint)]
    assert {p.GetName() for p in joints} == {"left_wheel_joint", "right_wheel_joint"}
    cache = UsdGeom.XformCache()
    for prim in joints:
        joint = UsdPhysics.RevoluteJoint(prim)
        assert joint.GetAxisAttr().Get() == "Y"
        anchors = []
        for rel, pos in [
            (joint.GetBody0Rel(), joint.GetLocalPos0Attr()),
            (joint.GetBody1Rel(), joint.GetLocalPos1Attr()),
        ]:
            targets = rel.GetTargets()
            assert len(targets) == 1 and str(targets[0]).startswith(
                "/Robots/cansat/"
            ), "Joint reference was not remapped."
            body = stage.GetPrimAtPath(targets[0])
            assert body.HasAPI(UsdPhysics.RigidBodyAPI)
            anchors.append(cache.GetLocalToWorldTransform(body).Transform(Gf.Vec3d(pos.Get())))
        assert np.linalg.norm(np.asarray(anchors[0]) - anchors[1]) < 1e-7, "Misaligned joint frames."
        drive = UsdPhysics.DriveAPI(prim, "angular")
        assert drive.GetStiffnessAttr().Get() == 0 and drive.GetDampingAttr().Get() > 0
        assert joint.GetLowerLimitAttr().Get() == -float("inf")
        assert joint.GetUpperLimitAttr().Get() == float("inf")
    meshes = [UsdGeom.Mesh(p) for p in stage.Traverse() if p.IsA(UsdGeom.Mesh)]
    assert len(meshes) == manifest["mesh_instances"]
    for mesh in meshes:
        points = np.asarray(mesh.GetPointsAttr().Get())
        indices = np.asarray(mesh.GetFaceVertexIndicesAttr().Get())
        assert np.isfinite(points).all()
        assert sum(mesh.GetFaceVertexCountsAttr().Get()) == len(indices)
        assert indices.min() >= 0 and indices.max() < len(points)
        assert len(mesh.GetNormalsAttr().Get()) == len(indices)
    print(
        f"PASS: {len(meshes)} meshes, 3 rigid bodies, 2 continuous wheel joints; references, units, geometry and config agree."
    )


def simulate(app):
    import numpy as np
    from geometry_msgs.msg import Twist
    from isaacsim.core.api import World
    from omegaconf import OmegaConf
    from pxr import Gf, UsdPhysics
    import omni.usd
    import rclpy

    from src.environments_wrappers.ros2.robot_manager_ros2 import ROS_RobotManager

    world = World(stage_units_in_meters=1.0, physics_dt=1 / 120, rendering_dt=1 / 60)
    world.scene.add_default_ground_plane()
    stage = omni.usd.get_context().get_stage()
    scene = next(UsdPhysics.Scene(p) for p in stage.Traverse() if p.IsA(UsdPhysics.Scene))
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    scene.CreateGravityMagnitudeAttr(1.62)
    cfg = OmegaConf.to_container(OmegaConf.load(ROOT / "cfg/robot/cansat.yaml"), resolve=True)["robots_settings"]
    cfg["parameters"]["usd_path"] = str(ROOT / cfg["parameters"]["usd_path"])
    cfg["parameters"]["pose"]["position"] = [0.0, 0.0, 0.15]
    rclpy.init()
    manager = ROS_RobotManager(cfg)
    try:
        manager.RM.preload_robot(world)
        world.reset()
        for _ in range(360):
            world.step(render=(_ % 8 == 0))
            manager.RM.robot.update_articulation_api()
        assert manager.RM.robot.is_articulation_api_ready(), "Articulation initialization failed."
        world.render()
        start, _ = manager.RM.robot_RG.get_pose_of_base_link()
        command = Twist()
        command.linear.x = 0.1
        for _ in range(240):
            manager.receive_velocity(command)
            manager.apply_modifications()
            world.step(render=(_ % 8 == 0))
            manager.RM.robot.update_articulation_api()
        world.render()
        end, _ = manager.RM.robot_RG.get_pose_of_base_link()
        delta = np.asarray(end) - start
        assert np.isfinite(delta).all() and delta[0] > 0.03, f"Robot did not drive forward: {delta}"
        assert abs(delta[1]) < 0.05, f"Robot veered sideways: {delta}"
        command.linear.x = 0.0
        command.angular.z = 0.5
        _, before = manager.RM.robot_RG.get_pose_of_base_link()
        for _ in range(240):
            manager.receive_velocity(command)
            manager.apply_modifications()
            world.step(render=(_ % 8 == 0))
            manager.RM.robot.update_articulation_api()
        world.render()
        _, after = manager.RM.robot_RG.get_pose_of_base_link()
        from scipy.spatial.transform import Rotation

        before_yaw = Rotation.from_quat([before[1], before[2], before[3], before[0]]).as_euler("xyz")[2]
        after_yaw = Rotation.from_quat([after[1], after[2], after[3], after[0]]).as_euler("xyz")[2]
        yaw_change = (after_yaw - before_yaw + np.pi) % (2 * np.pi) - np.pi
        assert yaw_change > 0.1, f"Robot did not turn left: {yaw_change}"
        manager.drive.reset()
        manager.apply_modifications()
        for _ in range(120):
            world.step(render=(_ % 8 == 0))
            manager.RM.robot.update_articulation_api()
        assert max(abs(v) for v in manager.RM.robot.get_joint_velocities().values()) < 0.5, "Robot failed to stop."
        print(f"PASS: Isaac Sim forward travel {delta[0]:.3f} m, left turn {yaw_change:.3f} rad, stop.")
    finally:
        world.stop()
        manager.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--simulate", action="store_true", help="Requires a working Isaac Sim GPU/driver.")
    args = parser.parse_args()
    sys.path.insert(0, str(ROOT))
    app = None
    if args.simulate:
        from isaacsim import SimulationApp

        app = SimulationApp({"headless": True})
        from src.environments_wrappers.ros2 import enable_ros2

        enable_ros2(app, bridge_name="humble")
    else:
        ensure_usd()
    try:
        check_asset()
        if args.simulate:
            simulate(app)
    except Exception:
        # Kit fast shutdown can exit before Python prints a pending exception.
        import traceback

        traceback.print_exc()
        raise
    finally:
        if app:
            app.close()


if __name__ == "__main__":
    main()
