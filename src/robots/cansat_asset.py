"""Offline conversion of the supplied static CAD CanSat FBX, in metres.

This intentionally supports the static mesh subset used by this Fusion export.
Unsupported animation, skinning and pivot transforms fail rather than silently
producing the wrong robot. It is not a general purpose FBX importer.
"""

from collections import defaultdict
from functools import lru_cache
import hashlib
import json
import os
from pathlib import Path
import struct
import tempfile
import zlib

import numpy as np
from scipy.spatial.transform import Rotation
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt


def read_fbx(path):
    data = Path(path).read_bytes()
    if data[:23] != b"Kaydara FBX Binary  \x00\x1a\x00":
        raise ValueError("Expected binary FBX.")
    version = struct.unpack_from("<I", data, 23)[0]
    if not 7000 <= version < 7500:
        raise ValueError(f"Unsupported FBX version {version}; expected 7.0–7.4.")
    cursor = 27

    def property_value():
        nonlocal cursor
        kind = chr(data[cursor])
        cursor += 1
        scalars = {"Y": "h", "C": "?", "I": "i", "F": "f", "D": "d", "L": "q"}
        if kind in scalars:
            fmt = "<" + scalars[kind]
            value = struct.unpack_from(fmt, data, cursor)[0]
            cursor += struct.calcsize(fmt)
            return value
        if kind in "SR":
            size = struct.unpack_from("<I", data, cursor)[0]
            cursor += 4
            value = data[cursor : cursor + size]
            cursor += size
            return value.decode("utf-8") if kind == "S" else value
        types = {"f": "<f4", "d": "<f8", "l": "<i8", "i": "<i4", "b": "?", "c": "i1"}
        if kind in types:
            count, encoding, size = struct.unpack_from("<III", data, cursor)
            cursor += 12
            payload = data[cursor : cursor + size]
            cursor += size
            if encoding not in (0, 1):
                raise ValueError(f"Unsupported array encoding {encoding}")
            if encoding:
                payload = zlib.decompress(payload)
            value = np.frombuffer(payload, dtype=types[kind]).copy()
            if len(value) != count:
                raise ValueError("Invalid FBX array length.")
            return value
        raise ValueError(f"Unsupported FBX property {kind}")

    def node():
        nonlocal cursor
        end, count, _, name_length = struct.unpack_from("<IIIB", data, cursor)
        cursor += 13
        if end == 0:
            return None
        if not cursor <= end <= len(data):
            raise ValueError("Invalid FBX node bounds.")
        name = data[cursor : cursor + name_length].decode("utf-8")
        cursor += name_length
        values = [property_value() for _ in range(count)]
        children = []
        while cursor < end:
            child = node()
            if child is not None:
                children.append(child)
        cursor = end
        return name, values, children

    nodes = []
    while cursor < len(data):
        value = node()
        if value is None:
            break
        nodes.append(value)
    return nodes


def child_values(children, name, default=None):
    return next((values for key, values, _ in children if key == name), default)


def properties(children):
    items = next((items for name, _, items in children if name == "Properties70"), [])
    return {values[0]: values[4:] for _, values, _ in items}


class CadScene:
    def __init__(self, source):
        nodes = read_fbx(source)
        objects = next(children for name, _, children in nodes if name == "Objects")
        unsupported = {name for name, _, _ in objects} - {"Model", "Geometry", "Material", "NodeAttribute"}
        if unsupported:
            raise ValueError(f"Unsupported CAD objects: {unsupported}")
        self.objects = {values[0]: (name, values, children) for name, values, children in objects}
        self.models = {key: value for key, value in self.objects.items() if value[0] == "Model"}
        self.parent = {}
        self.children = defaultdict(list)
        connections = next(children for name, _, children in nodes if name == "Connections")
        for _, values, _ in connections:
            if values[0] != "OO":
                raise ValueError("Only static object connections are supported.")
            child, parent = values[1:3]
            self.children[parent].append(child)
            if child in self.models:
                self.parent[child] = parent
        settings = properties(next(children for name, _, children in nodes if name == "GlobalSettings"))
        if settings["UpAxis"] != [1] or settings["UpAxisSign"] != [1]:
            raise ValueError("Expected Y-up Fusion CAD export.")
        self.metres = settings["UnitScaleFactor"][0] * 0.01
        self.wheels = [
            key
            for key, (_, values, _) in self.models.items()
            if values[1].startswith("NEWタイヤ") and ":1\x00" in values[1]
        ]
        if len(self.wheels) != 2:
            raise ValueError(f"Expected two NEWタイヤ wheel assemblies, found {len(self.wheels)}.")
        # Camera faces CAD -X; CAD +Z is the left wheel and CAD +Y is up.
        self.axes = np.array([[-1, 0, 0], [0, 0, 1], [0, 1, 0]], dtype=float)
        self.origin = np.mean([self.matrix(key)[:3, 3] for key in self.wheels], axis=0)

    @lru_cache(None)
    def matrix(self, key):
        if key not in self.models:
            return np.eye(4)
        props = properties(self.models[key][2])
        unsupported = set(props) - {
            "ScalingMax",
            "DefaultAttributeIndex",
            "Lcl Translation",
            "Lcl Rotation",
            "Lcl Scaling",
        }
        if unsupported:
            raise ValueError(f"Unsupported model transforms: {unsupported}")
        matrix = np.eye(4)
        matrix[:3, :3] = Rotation.from_euler(
            "xyz", props.get("Lcl Rotation", [0, 0, 0]), degrees=True
        ).as_matrix() @ np.diag(props.get("Lcl Scaling", [1, 1, 1]))
        matrix[:3, 3] = props.get("Lcl Translation", [0, 0, 0])
        return self.matrix(self.parent.get(key)) @ matrix

    def ancestry(self, key):
        while key in self.models:
            yield key
            key = self.parent.get(key)

    def meshes(self):
        # Geometry IDs can be instanced by several models; preserve every instance.
        for key, (_, values, _) in self.models.items():
            ancestors = list(self.ancestry(key))
            wheel = next((a for a in ancestors if a in self.wheels), None)
            stabilizer = any(self.models[a][1][1].startswith("stabilizer TRC") for a in ancestors)
            for geo in self.children[key]:
                kind, _, children = self.objects[geo]
                if kind != "Geometry":
                    continue
                vertices = child_values(children, "Vertices")[0].reshape(-1, 3)
                matrix = self.matrix(key)
                points = ((vertices @ matrix[:3, :3].T + matrix[:3, 3] - self.origin) @ self.axes.T) * self.metres
                raw = child_values(children, "PolygonVertexIndex")[0]
                ends = np.flatnonzero(raw < 0)
                counts = np.diff(np.r_[-1, ends])
                if not len(ends) or ends[-1] != len(raw) - 1:
                    raise ValueError("Unterminated FBX polygon.")
                indices = np.where(raw < 0, -raw - 1, raw).astype(np.int32)
                material_ids = [c for c in self.children[key] if self.objects[c][0] == "Material"]
                material_layer = next(c for n, _, c in children if n == "LayerElementMaterial")
                mapping = child_values(material_layer, "MappingInformationType")
                face_materials = child_values(material_layer, "Materials")[0]
                if mapping == ["AllSame"]:
                    face_materials = np.full(len(counts), face_materials[0], dtype=np.int32)
                elif mapping != ["ByPolygon"] or len(face_materials) != len(counts):
                    raise ValueError("Unsupported CAD material mapping.")
                if not material_ids or min(face_materials) < -1 or max(face_materials) >= len(material_ids):
                    raise ValueError("Invalid CAD material indices.")
                if -1 in face_materials:
                    face_materials = np.where(face_materials == -1, len(material_ids), face_materials)
                    material_ids.append(None)  # FBX -1 denotes an unassigned face.
                normal_layer = next((c for n, _, c in children if n == "LayerElementNormal"), None)
                normals = None
                if normal_layer is not None:
                    if child_values(normal_layer, "MappingInformationType") != ["ByPolygonVertex"] or child_values(
                        normal_layer, "ReferenceInformationType"
                    ) != ["Direct"]:
                        raise ValueError("Unsupported CAD normal mapping.")
                    normals = child_values(normal_layer, "Normals")[0].reshape(-1, 3)
                    normals = normals @ np.linalg.inv(matrix[:3, :3]) @ self.axes.T
                    normals /= np.linalg.norm(normals, axis=1, keepdims=True)
                    if len(normals) != len(indices):
                        raise ValueError("Invalid face normal count.")
                uv_layer = next((c for n, _, c in children if n == "LayerElementUV"), None)
                uv = None
                if uv_layer is not None:
                    if child_values(uv_layer, "MappingInformationType") != ["ByPolygonVertex"] or child_values(
                        uv_layer, "ReferenceInformationType"
                    ) != ["IndexToDirect"]:
                        raise ValueError("Unsupported CAD UV mapping.")
                    uv = child_values(uv_layer, "UV")[0].reshape(-1, 2)[child_values(uv_layer, "UVIndex")[0]]
                if np.linalg.det(matrix[:3, :3]) < 0:
                    raise ValueError("Mirrored mesh transforms require winding conversion.")
                yield dict(
                    key=key,
                    name=values[1].split("\x00")[0],
                    points=points,
                    counts=counts,
                    indices=indices,
                    normals=normals,
                    uv=uv,
                    materials=material_ids,
                    face_materials=face_materials,
                    wheel=wheel,
                    stabilizer=stabilizer,
                )


def physics_material(stage, path, static, dynamic):
    mat = UsdShade.Material.Define(stage, path)
    api = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    api.CreateStaticFrictionAttr(static)
    api.CreateDynamicFrictionAttr(dynamic)
    api.CreateRestitutionAttr(0.0)
    return mat


def collider(prim, material):
    UsdPhysics.CollisionAPI.Apply(prim)
    prim.AddAppliedSchema("PhysxCollisionAPI")
    prim.CreateAttribute("physxCollision:contactOffset", Sdf.ValueTypeNames.Float, custom=False).Set(0.001)
    prim.CreateAttribute("physxCollision:restOffset", Sdf.ValueTypeNames.Float, custom=False).Set(0.0)
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(material, materialPurpose="physics")


def build_asset(source, settings_path, output):
    output = Path(output).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    # Build everything first: a failed conversion must not erase working assets.
    with tempfile.TemporaryDirectory(prefix=".cansat-", dir=output.parent) as temporary:
        temporary_output = Path(temporary) / output.name
        _build_asset(source, settings_path, temporary_output)
        for name in ("cansat_visual.usdc", output.name, "manifest.json"):
            os.replace(Path(temporary) / name, output.parent / name)


def _build_asset(source, settings_path, output):
    source, output = Path(source), Path(output)
    settings = json.loads(Path(settings_path).read_text())
    for key, value in settings.items():
        if not np.isfinite(value) or value <= 0:
            raise ValueError(f"{key} must be finite and positive.")
    scene = CadScene(source)
    meshes = list(scene.meshes())
    output.parent.mkdir(parents=True, exist_ok=True)
    visual_path = output.with_name("cansat_visual.usdc")
    visuals = Usd.Stage.CreateNew(str(visual_path))
    root = UsdGeom.Xform.Define(visuals, "/Cansat")
    visuals.SetDefaultPrim(root.GetPrim())
    UsdGeom.SetStageMetersPerUnit(visuals, 1.0)
    UsdGeom.SetStageUpAxis(visuals, UsdGeom.Tokens.z)
    for link in ("base_link", "left_wheel_link", "right_wheel_link"):
        UsdGeom.Xform.Define(visuals, f"/Cansat/{link}")
        UsdGeom.Scope.Define(visuals, f"/Cansat/{link}/visuals")
    materials = {}
    for key, (kind, values, children) in scene.objects.items():
        if kind != "Material":
            continue
        mat = UsdShade.Material.Define(visuals, f"/Cansat/Looks/m_{key}")
        shader = UsdShade.Shader.Define(visuals, mat.GetPath().AppendChild("PreviewSurface"))
        props = properties(children)
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
            Gf.Vec3f(*props.get("DiffuseColor", [0.7, 0.7, 0.7]))
        )
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.6)
        shader.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(props.get("Opacity", [1.0])[0])
        mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        materials[key] = mat
    wheel_centers = {
        key: (scene.matrix(key)[:3, 3] - scene.origin) @ scene.axes.T * scene.metres for key in scene.wheels
    }
    wheel_sides = {key: "left" if center[1] > 0 else "right" for key, center in wheel_centers.items()}
    radii, widths, collision_centers = {}, {}, {}
    for key in scene.wheels:
        # Tire meshes directly beneath the tire assembly; exclude shaft/bearings.
        tire = [
            m["points"]
            for m in meshes
            if m["wheel"] == key and not any("drive shaft" in scene.models[a][1][1] for a in scene.ancestry(m["key"]))
        ]
        points = np.concatenate(tire) - wheel_centers[key]
        radii[key] = float(np.max(np.linalg.norm(points[:, [0, 2]], axis=1)))
        widths[key] = float(np.ptp(points[:, 1]))
        collision_centers[key] = float((points[:, 1].min() + points[:, 1].max()) / 2)
    for mesh in meshes:
        link = "base_link" if mesh["wheel"] is None else f"{wheel_sides[mesh['wheel']]}_wheel_link"
        path = f"/Cansat/{link}/visuals/mesh_{mesh['key']}"
        usd_mesh = UsdGeom.Mesh.Define(visuals, path)
        points = mesh["points"] - (wheel_centers[mesh["wheel"]] if mesh["wheel"] else 0)
        usd_mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(points.astype(np.float32)))
        usd_mesh.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(mesh["counts"].astype(np.int32)))
        usd_mesh.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(mesh["indices"]))
        usd_mesh.CreateSubdivisionSchemeAttr("none")
        usd_mesh.CreateExtentAttr([Gf.Vec3f(*points.min(0)), Gf.Vec3f(*points.max(0))])
        if mesh["normals"] is not None:
            usd_mesh.CreateNormalsAttr(Vt.Vec3fArray.FromNumpy(mesh["normals"].astype(np.float32)))
            usd_mesh.SetNormalsInterpolation("faceVarying")
        if mesh["uv"] is not None:
            UsdGeom.PrimvarsAPI(usd_mesh).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, "faceVarying").Set(
                Vt.Vec2fArray.FromNumpy(mesh["uv"].astype(np.float32))
            )
        usd_mesh.GetPrim().SetCustomDataByKey("fbx_name", mesh["name"])
        binding = UsdShade.MaterialBindingAPI.Apply(usd_mesh.GetPrim())
        for index in np.unique(mesh["face_materials"]):
            if mesh["materials"][index] is None:
                continue
            subset = binding.CreateMaterialBindSubset(
                f"material_{index}",
                Vt.IntArray.FromNumpy(np.flatnonzero(mesh["face_materials"] == index).astype(np.int32)),
            )
            UsdShade.MaterialBindingAPI.Apply(subset.GetPrim()).Bind(materials[mesh["materials"][index]])
        binding.SetMaterialBindSubsetsFamilyType(UsdGeom.Tokens.nonOverlapping)
    visuals.GetRootLayer().Save()

    stage = Usd.Stage.CreateNew(str(output))
    root = UsdGeom.Xform.Define(stage, "/Cansat").GetPrim()
    root.GetReferences().AddReference(visual_path.name)
    stage.SetDefaultPrim(root)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdPhysics.ArticulationRootAPI.Apply(root)
    root.AddAppliedSchema("PhysxArticulationAPI")
    root.CreateAttribute("physxArticulation:enabledSelfCollisions", Sdf.ValueTypeNames.Bool, custom=False).Set(False)
    root.CreateAttribute("physxArticulation:solverPositionIterationCount", Sdf.ValueTypeNames.Int, custom=False).Set(16)
    root.CreateAttribute("physxArticulation:solverVelocityIterationCount", Sdf.ValueTypeNames.Int, custom=False).Set(4)
    rubber = physics_material(
        stage, "/Cansat/PhysicsMaterials/rubber", settings["static_friction"], settings["dynamic_friction"]
    )
    skid = physics_material(
        stage, "/Cansat/PhysicsMaterials/skid", settings["stabilizer_friction"], settings["stabilizer_friction"]
    )
    for link in ["base_link", "left_wheel_link", "right_wheel_link"]:
        prim = stage.GetPrimAtPath(f"/Cansat/{link}")
        UsdPhysics.RigidBodyAPI.Apply(prim)
        api = UsdPhysics.MassAPI.Apply(prim)
        api.CreateMassAttr(settings["body_mass_kg"] if link == "base_link" else settings["wheel_mass_kg"])
        prim.AddAppliedSchema("PhysxRigidBodyAPI")
        prim.CreateAttribute("physxRigidBody:enableCCD", Sdf.ValueTypeNames.Bool, custom=False).Set(True)
    # A chassis proxy avoids hundreds of small electronic/fastener colliders.
    chassis_points = np.concatenate([m["points"] for m in meshes if m["wheel"] is None and not m["stabilizer"]])
    low, high = chassis_points.min(0), chassis_points.max(0)
    box = UsdGeom.Cube.Define(stage, "/Cansat/base_link/collision")
    box.CreateSizeAttr(1.0)
    box.AddTranslateOp().Set(Gf.Vec3d(*(low + high) / 2))
    box.AddScaleOp().Set(Gf.Vec3f(*(high - low)))
    box.CreatePurposeAttr("guide")
    collider(box.GetPrim(), rubber)
    # Preserve the rear stabilizer as the third support on this two-wheel robot.
    for m in meshes:
        if m["stabilizer"]:
            prim = stage.GetPrimAtPath(f"/Cansat/base_link/visuals/mesh_{m['key']}")
            collider(prim, skid)
            UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr("convexHull")
    for key in scene.wheels:
        side = wheel_sides[key]
        link_path = f"/Cansat/{side}_wheel_link"
        UsdGeom.Xformable(stage.GetPrimAtPath(link_path)).AddTranslateOp().Set(Gf.Vec3d(*wheel_centers[key]))
        cylinder = UsdGeom.Cylinder.Define(stage, link_path + "/collision")
        cylinder.CreateAxisAttr("Y")
        cylinder.CreateRadiusAttr(radii[key])
        cylinder.CreateHeightAttr(widths[key])
        cylinder.AddTranslateOp().Set(Gf.Vec3d(0, collision_centers[key], 0))
        cylinder.CreatePurposeAttr("guide")
        collider(cylinder.GetPrim(), rubber)
        joint = UsdPhysics.RevoluteJoint.Define(stage, f"/Cansat/joints/{side}_wheel_joint")
        joint.CreateBody0Rel().SetTargets(["/Cansat/base_link"])
        joint.CreateBody1Rel().SetTargets([link_path])
        joint.CreateAxisAttr("Y")
        joint.CreateLocalPos0Attr(Gf.Vec3f(*wheel_centers[key]))
        joint.CreateLocalPos1Attr(Gf.Vec3f(0))
        joint.CreateLocalRot0Attr(Gf.Quatf(1))
        joint.CreateLocalRot1Attr(Gf.Quatf(1))
        # No limits: a continuous wheel. Both +Y axes drive forward at +speed.
        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(0.0)
        drive.CreateDampingAttr(settings["wheel_damping"])
        drive.CreateMaxForceAttr(settings["max_wheel_torque_nm"])
        drive.CreateTargetVelocityAttr(0.0)
        joint.GetPrim().AddAppliedSchema("PhysxJointAPI")
        joint.GetPrim().CreateAttribute("physxJoint:maxJointVelocity", Sdf.ValueTypeNames.Float, custom=False).Set(
            float(np.degrees(settings["max_wheel_speed_rad_s"]))
        )
    stage.GetRootLayer().Save()
    manifest = dict(
        source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        mesh_instances=len(meshes),
        metres_per_fbx_unit=scene.metres,
        wheel_radius_m=float(np.mean(list(radii.values()))),
        wheel_track_m=float(abs(np.diff([c[1] + collision_centers[k] for k, c in wheel_centers.items()])[0])),
        wheel_width_m=widths,
        settings=settings,
        total_mass_kg=settings["body_mass_kg"] + 2 * settings["wheel_mass_kg"],
    )
    output.with_name("manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(json.dumps(manifest, indent=2))
