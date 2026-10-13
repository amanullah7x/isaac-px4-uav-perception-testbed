#!/usr/bin/env python
"""
drone_usd_check.py - Validate that a custom multirotor USD has the prim layout Pegasus Simulator expects.

Pegasus (pegasus.simulator 6.0.0, the version installed in ~/PegasusSimulator) applies rotor thrust and body torque
to fixed sub-paths of the vehicle, so a custom drone USD must look like this (names are case sensitive):

    <defaultPrim>                 Xform, PhysicsArticulationRootAPI
      body                        Xform, PhysicsRigidBodyAPI + PhysicsMassAPI   (frame: X forward, Y left, Z up)
      rotor0 .. rotor3            Xform, PhysicsRigidBodyAPI + PhysicsMassAPI   (PX4 quad-X motor order, see GUIDE.md)
        jointN                    PhysicsRevoluteJoint body -> rotorN           (only used to spin the props visually)
      colliders                   at least one PhysicsCollisionAPI prim under body (otherwise it falls through the floor)

Usage (no GPU needed, uses Isaac Sim's bundled USD libraries):
    ./check_drone_usd.sh /path/to/my_drone.usd

The same check runs automatically inside launch_custom_drone.py before the drone is spawned.
"""
import sys

NUM_ROTORS = 4


def check_drone_usd(usd_path: str):
    """Return (errors, warnings, info) lists for the drone USD at usd_path."""
    from pxr import Usd, UsdGeom, UsdPhysics

    errors, warnings, info = [], [], []

    stage = Usd.Stage.Open(usd_path)
    if stage is None:
        return [f"Could not open USD file: {usd_path}"], warnings, info

    root = stage.GetDefaultPrim()
    if not root:
        return ["USD has no defaultPrim. Pegasus references the file's defaultPrim, so set one "
                "(in Isaac Sim: right-click the drone root prim > Set as Default Prim, then save)."], warnings, info
    rp = root.GetPath()
    info.append(f"defaultPrim = {rp}")

    up = UsdGeom.GetStageUpAxis(stage)
    mpu = UsdGeom.GetStageMetersPerUnit(stage)
    info.append(f"upAxis = {up}, metersPerUnit = {mpu}")
    if up != "Z":
        errors.append(f"upAxis is {up}; Pegasus/PX4 assume Z-up (ENU). Re-export the asset Z-up.")
    if abs(mpu - 1.0) > 1e-6:
        errors.append(f"metersPerUnit is {mpu}; must be 1.0 (meters) or thrust/mass will be off by "
                      f"{1/mpu:.0f}x. Re-export in meters.")

    if not root.HasAPI(UsdPhysics.ArticulationRootAPI):
        # Root API on 'body' also works for PhysX, but Pegasus creates its Articulation at the root path.
        errors.append(f"{rp} is missing PhysicsArticulationRootAPI (Add > Physics > Articulation Root on the root prim).")

    total_mass = 0.0

    def rigid(name):
        p = stage.GetPrimAtPath(rp.AppendChild(name))
        if not p or not p.IsValid():
            errors.append(f"Missing required prim {rp}/{name}")
            return None
        if not p.HasAPI(UsdPhysics.RigidBodyAPI):
            errors.append(f"{p.GetPath()} is missing PhysicsRigidBodyAPI")
        if p.HasAPI(UsdPhysics.MassAPI):
            m = UsdPhysics.MassAPI(p).GetMassAttr().Get()
            if m:
                return float(m)
        warnings.append(f"{p.GetPath()} has no explicit mass; PhysX will derive it from collider density "
                        f"(usually wrong for drones). Set PhysicsMassAPI mass explicitly.")
        return 0.0

    m = rigid("body")
    total_mass += m or 0.0
    for i in range(NUM_ROTORS):
        m = rigid(f"rotor{i}")
        total_mass += m or 0.0
        joint = stage.GetPrimAtPath(rp.AppendChild(f"rotor{i}").AppendChild(f"joint{i}"))
        if not joint or not joint.IsA(UsdPhysics.RevoluteJoint):
            # search anywhere under the vehicle, the joint name is what matters
            found = [p for p in Usd.PrimRange(root) if p.GetName() == f"joint{i}" and p.IsA(UsdPhysics.RevoluteJoint)]
            if not found:
                warnings.append(f"No PhysicsRevoluteJoint named joint{i}: rotor{i} will not be attached to the body. "
                                f"Add a revolute joint body -> rotor{i} (axis Z).")

    body = stage.GetPrimAtPath(rp.AppendChild("body"))
    if body:
        colliders = [p for p in Usd.PrimRange(body) if p.HasAPI(UsdPhysics.CollisionAPI)]
        if not colliders:
            errors.append(f"No PhysicsCollisionAPI prim under {rp}/body; the drone will fall through the ground.")
        else:
            info.append(f"{len(colliders)} collider prim(s) under body")

    for i in range(NUM_ROTORS):
        r = stage.GetPrimAtPath(rp.AppendChild(f"rotor{i}"))
        if r:
            cols = [p for p in Usd.PrimRange(r) if p.HasAPI(UsdPhysics.CollisionAPI)]
            if cols:
                warnings.append(f"rotor{i} has colliders; propellers touching the frame or ground cause jitter. "
                                f"Disable collisions on propellers.")

    if total_mass > 0:
        info.append(f"total explicit mass = {total_mass:.3f} kg")
        # Pegasus default thrust curve: T = 8.54858e-6 * w^2, w_max = 1100 rad/s  -> ~10.34 N per rotor
        max_thrust = NUM_ROTORS * 8.54858e-6 * 1100.0 ** 2
        twr = max_thrust / (total_mass * 9.81)
        info.append(f"thrust/weight with default Pegasus (Iris) thrust curve = {twr:.2f}")
        if twr < 1.6:
            warnings.append(f"Thrust/weight {twr:.2f} is low (<1.6). Lighten the model or raise rotor_constant / "
                            f"max_rotor_velocity in the thrust curve (see --rotor-constant).")

    return errors, warnings, info


def resting_height(usd_path: str) -> float:
    """Height [m] at which to spawn the vehicle root so its lowest point just touches z=0.

    Spawning too high makes the drone free-fall for a few physics steps while PX4 is already receiving IMU data.
    PX4's EKF2 aligns its tilt from those first accelerometer samples; with ~0 g during the drop it can initialise
    upside down and then refuses to arm ("Attitude failure (roll)", "High Gyro Bias"). Spawning at rest avoids it.
    """
    from pxr import Usd, UsdGeom

    stage = Usd.Stage.Open(usd_path)
    root = stage.GetDefaultPrim() if stage else None
    if not root:
        return 0.07
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy])
    # bound relative to the root prim's own frame (the spawn pose replaces the root transform)
    rng = cache.ComputeUntransformedBound(root).ComputeAlignedRange()
    if rng.IsEmpty():
        return 0.07
    return max(0.0, -rng.GetMin()[2]) + 0.002


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(2)
    rc = 0
    for f in sys.argv[1:]:
        errors, warnings, info = check_drone_usd(f)
        print(f"== {f}")
        for s in info:
            print(f"   info   : {s}")
        for s in warnings:
            print(f"   WARNING: {s}")
        for s in errors:
            print(f"   ERROR  : {s}")
        print(f"   info   : resting spawn height = {resting_height(f):.3f} m")
        print("   RESULT : " + ("OK for Pegasus" if not errors else "NOT compatible with Pegasus as-is"))
        rc |= 1 if errors else 0
    sys.exit(rc)


if __name__ == "__main__":
    main()
