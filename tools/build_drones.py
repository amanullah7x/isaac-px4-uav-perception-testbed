#!/usr/bin/env python3
"""
build_drones.py - Build Pegasus-layout drone USDs (body + rotor0..3 rigid bodies, revolute joints, Z-up, metres).

    python build_drones.py x500  ../assets/x500/x500.usd     # Holybro X500 from PX4's Gazebo model (meshes + mass)
    python build_drones.py fpv   ../assets/fpv/fpv.usd       # 5-inch FPV quad built from primitives

Rotor order is PX4 quad-X as Pegasus expects it: rotor0 front-right, rotor1 rear-left, rotor2 front-left,
rotor3 rear-right (FLU body frame: X forward, Y left, Z up). Needs usd-core, trimesh, pycollada.
"""
import math
import os
import sys

import numpy as np
import trimesh
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt

X500_DIR = os.path.expanduser("~/PX4-Autopilot/Tools/simulation/gz/models/x500/meshes")


def pose(x=0, y=0, z=0, r=0, p=0, yw=0, s=1.0):
    """SDF pose -> 4x4 (T * Rz * Ry * Rx * S)."""
    R = trimesh.transformations.euler_matrix(r, p, yw, "sxyz")
    R[:3, 3] = [x, y, z]
    return R @ trimesh.transformations.scale_matrix(s)


class Builder:
    def __init__(self, out):
        os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
        self.stage = Usd.Stage.CreateNew(out)
        UsdGeom.SetStageUpAxis(self.stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(self.stage, 1.0)
        root = UsdGeom.Xform.Define(self.stage, "/vehicle")
        self.stage.SetDefaultPrim(root.GetPrim())
        UsdPhysics.ArticulationRootAPI.Apply(root.GetPrim())
        UsdGeom.Scope.Define(self.stage, "/vehicle/Looks")
        self.mats = {}
        self.n = 0

    def material(self, rgb, rough=0.5, metal=0.0):
        key = (tuple(np.round(rgb, 3)), rough, metal)
        if key not in self.mats:
            p = f"/vehicle/Looks/m{len(self.mats)}"
            m = UsdShade.Material.Define(self.stage, p)
            sh = UsdShade.Shader.Define(self.stage, p + "/Surface")
            sh.CreateIdAttr("UsdPreviewSurface")
            sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
            sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(rough)
            sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metal)
            m.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
            self.mats[key] = m
        return self.mats[key]

    def mesh(self, parent, src, T, rgb, rough=0.5, metal=0.0):
        """Embed every geometry of a mesh file (DAE/STL/OBJ) under parent, transformed by T."""
        scene = trimesh.load(src, force="scene", process=False)
        for node in scene.graph.nodes_geometry:
            M, g = scene.graph[node]
            g = scene.geometry[g].copy()
            if not isinstance(g, trimesh.Trimesh) or len(g.faces) == 0:
                continue
            g.apply_transform(T @ M)
            self._trimesh(parent, g, rgb, rough, metal)

    def _trimesh(self, parent, g, rgb, rough=0.5, metal=0.0):
        self.n += 1
        m = UsdGeom.Mesh.Define(self.stage, f"{parent}/visual_{self.n}")
        m.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(g.vertices.astype(np.float32)))
        m.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(np.full(len(g.faces), 3, np.int32)))
        m.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(g.faces.astype(np.int32).ravel()))
        m.CreateSubdivisionSchemeAttr("none")
        m.CreateDoubleSidedAttr(True)
        m.CreateExtentAttr([Gf.Vec3f(*g.vertices.min(0)), Gf.Vec3f(*g.vertices.max(0))])
        UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(self.material(rgb, rough, metal))
        return m

    def box(self, parent, name, size, T, rgb, collide=False, rough=0.6, metal=0.0):
        g = trimesh.creation.box(extents=size, transform=T)
        if collide:   # analytic Cube collider (convex, cheap) with the same pose/size
            c = UsdGeom.Cube.Define(self.stage, f"{parent}/{name}")
            c.CreateSizeAttr(1.0)
            xf = UsdGeom.Xformable(c)
            xf.AddTransformOp().Set(Gf.Matrix4d(*(T @ np.diag([*size, 1.0])).T.ravel()))
            UsdPhysics.CollisionAPI.Apply(c.GetPrim())
            c.CreatePurposeAttr("guide")   # collider only, not rendered
            return c
        return self._trimesh(parent, g, rgb, rough, metal)

    def cylinder(self, parent, r, h, T, rgb, sections=24, rough=0.4, metal=0.0):
        return self._trimesh(parent, trimesh.creation.cylinder(radius=r, height=h, sections=sections, transform=T),
                             rgb, rough, metal)

    def body(self, mass, inertia):
        b = UsdGeom.Xform.Define(self.stage, "/vehicle/body")
        UsdPhysics.RigidBodyAPI.Apply(b.GetPrim())
        mp = UsdPhysics.MassAPI.Apply(b.GetPrim())
        mp.CreateMassAttr(mass)
        mp.CreateDiagonalInertiaAttr(Gf.Vec3f(*inertia))
        mp.CreateCenterOfMassAttr(Gf.Vec3f(0, 0, 0))
        return "/vehicle/body"

    def rotor(self, i, pos, mass=0.016, inertia=(4e-7, 2.6e-5, 2.65e-5)):
        p = f"/vehicle/rotor{i}"
        r = UsdGeom.Xform.Define(self.stage, p)
        UsdGeom.XformCommonAPI(r).SetTranslate(Gf.Vec3d(*pos))
        UsdPhysics.RigidBodyAPI.Apply(r.GetPrim())
        mp = UsdPhysics.MassAPI.Apply(r.GetPrim())
        mp.CreateMassAttr(mass)
        mp.CreateDiagonalInertiaAttr(Gf.Vec3f(*inertia))
        j = UsdPhysics.RevoluteJoint.Define(self.stage, f"{p}/joint{i}")
        j.CreateAxisAttr("Z")
        j.CreateBody0Rel().SetTargets(["/vehicle/body"])
        j.CreateBody1Rel().SetTargets([p])
        j.CreateLocalPos0Attr(Gf.Vec3f(*pos))
        j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        return p

    def save(self):
        self.stage.GetRootLayer().Save()


def build_x500(out):
    """Holybro X500 V2 as in PX4's gz model x500/model.sdf (same meshes, mass, inertia, rotor positions)."""
    B = Builder(out)
    body = B.body(2.0, (0.021667, 0.021667, 0.04))
    carbon, alu, prop = (0.08, 0.08, 0.09), (0.55, 0.56, 0.6), (0.12, 0.12, 0.14)
    B.mesh(body, f"{X500_DIR}/NXP-HGD-CF.dae", pose(0, 0, .025, 0, 0, math.pi), carbon, 0.45)
    for x, y in [(0.174, 0.174), (-0.174, 0.174), (0.174, -0.174), (-0.174, -0.174)]:
        B.mesh(body, f"{X500_DIR}/5010Base.dae", pose(x, y, .032, 0, 0, -.45), alu, 0.35, 0.8)
    # Colliders copied from the SDF: frame plate + the two landing-gear legs and skids
    B.box(body, "col_plate", (0.3536, 0.3536, 0.05), pose(0, 0, .007), None, collide=True)
    B.box(body, "col_leg_r", (0.015, 0.015, 0.21), pose(0, -0.098, -.123, -0.35), None, collide=True)
    B.box(body, "col_leg_l", (0.015, 0.015, 0.21), pose(0, 0.098, -.123, 0.35), None, collide=True)
    B.box(body, "col_skid_r", (0.25, 0.015, 0.015), pose(0, -0.132, -.2195), None, collide=True)
    B.box(body, "col_skid_l", (0.25, 0.015, 0.015), pose(0, 0.132, -.2195), None, collide=True)
    # PX4 order: 0 FR (ccw prop), 1 RL (ccw), 2 FL (cw), 3 RR (cw)
    for i, (x, y, kind) in enumerate([(0.174, -0.174, "ccw"), (-0.174, 0.174, "ccw"), (0.174, 0.174, "cw"),
                                      (-0.174, -0.174, "cw")]):
        r = B.rotor(i, (x, y, 0.06))
        B.mesh(r, f"{X500_DIR}/1345_prop_{kind}.stl", pose(-0.022, -0.14638, -0.016, s=0.84615), prop, 0.6)
        B.mesh(r, f"{X500_DIR}/5010Bell.dae", pose(0, 0, -.032), alu, 0.3, 0.8)
    B.save()


def build_fpv(out):
    """5-inch freestyle FPV quad (~0.6 kg, 220 mm motor-to-motor) from primitives: X frame, stack, battery, camera."""
    B = Builder(out)
    body = B.body(0.60, (0.0012, 0.0013, 0.0022))
    carbon, alu, black = (0.05, 0.05, 0.06), (0.6, 0.6, 0.65), (0.02, 0.02, 0.02)
    a = 0.078                                   # 0.110 m * cos(45) arm reach in X and Y
    arm_len = 2 * math.hypot(a, a) + 0.03
    for yaw in (math.radians(45), math.radians(-45)):
        B.box(body, "arm", (arm_len, 0.022, 0.005), pose(0, 0, 0, 0, 0, yaw), carbon, rough=0.35)
    B.box(body, "plate_bottom", (0.12, 0.045, 0.004), pose(0, 0, 0.004), carbon, rough=0.35)
    B.box(body, "plate_top", (0.10, 0.045, 0.003), pose(0, 0, 0.036), carbon, rough=0.35)
    for x in (-0.04, 0.04):
        for y in (-0.018, 0.018):
            B.cylinder(body, 0.0025, 0.032, pose(x, y, 0.02), alu, 8, metal=0.8)        # standoffs
    B.box(body, "stack", (0.036, 0.036, 0.018), pose(0, 0, 0.017), (0.1, 0.35, 0.15))   # FC/ESC stack (green PCB)
    B.box(body, "battery", (0.075, 0.035, 0.032), pose(-0.005, 0, 0.055), (0.15, 0.15, 0.17))
    B.box(body, "strap", (0.012, 0.037, 0.034), pose(-0.005, 0, 0.055), (0.75, 0.1, 0.1))
    B.box(body, "fpv_cam", (0.02, 0.02, 0.02), pose(0.055, 0, 0.02, 0, -0.5, 0), black)
    B.cylinder(body, 0.0035, 0.01, pose(0.068, 0, 0.026, 0, math.pi / 2 - 0.5, 0), (0.2, 0.2, 0.8), 12)  # VTX ant.
    for x, y in [(a, a), (-a, a), (a, -a), (-a, -a)]:
        B.cylinder(body, 0.0115, 0.008, pose(x, y, 0.006), alu, 20, metal=0.8)          # motor stator base
    # Colliders: frame + battery as simple boxes
    B.box(body, "col_frame", (0.2, 0.2, 0.012), pose(0, 0, 0.004), None, collide=True)
    B.box(body, "col_batt", (0.075, 0.035, 0.032), pose(-0.005, 0, 0.055), None, collide=True)
    for i, (x, y) in enumerate([(a, -a), (-a, a), (a, a), (-a, -a)]):
        r = B.rotor(i, (x, y, 0.024), mass=0.004, inertia=(2e-7, 5e-6, 5e-6))
        B.cylinder(r, 0.0115, 0.012, pose(0, 0, -0.008), (0.75, 0.2, 0.05), 20, metal=0.6)  # motor bell
        prop = (0.9, 0.35, 0.05) if i in (0, 2) else (0.05, 0.05, 0.05)                    # orange front props
        for k in range(3):                                                                  # tri-blade 5" prop
            B.box(r, f"blade{k}", (0.062, 0.012, 0.002), pose(0.031 * math.cos(k * 2.094), 0.031 * math.sin(
                k * 2.094), 0.0, 0.18, 0, k * 2.094), prop, rough=0.5)
    B.save()


if __name__ == "__main__":
    kind, out = sys.argv[1], sys.argv[2]
    {"x500": build_x500, "fpv": build_fpv}[kind](out)
    print(f"wrote {out}")
