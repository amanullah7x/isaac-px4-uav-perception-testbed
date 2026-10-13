#!/usr/bin/env python3
"""
mavlink_smoke_test.py - Scripted equivalent of "connect QGC, arm, take off, hover, land" over PX4's offboard
MAVLink port (UDP 14540). Useful to verify the Isaac Sim <-> PX4 loop without clicking through QGC, and safe to run
while QGC is open (QGC uses 14550).

    python3 mavlink_smoke_test.py --alt 5 --hover 10

Needs pymavlink (pip install --user pymavlink). Exit code 0 = the drone reached the target altitude and landed.
"""
import argparse
import sys
import time

from pymavlink import mavutil

ap = argparse.ArgumentParser()
ap.add_argument("--url", default="udpin:0.0.0.0:14540")
ap.add_argument("--alt", type=float, default=5.0)
ap.add_argument("--hover", type=float, default=10.0)
ap.add_argument("--timeout", type=float, default=120.0, help="max seconds to wait for PX4 to be ready to arm")
ap.add_argument("--climb-timeout", type=float, default=150.0, help="wall-clock seconds allowed for the climb (the sim may run slower than real time)")
a = ap.parse_args()


def say(m):
    print(f"[smoke] {m}", flush=True)


m = mavutil.mavlink_connection(a.url, source_system=245)
say(f"waiting for heartbeat on {a.url} ...")
hb = m.wait_heartbeat(timeout=60)
if hb is None:
    say("FAIL: no heartbeat. Is the sim playing and PX4 running?")
    sys.exit(1)
say(f"heartbeat from system {m.target_system} comp {m.target_component}, autopilot={hb.autopilot}")

m.mav.request_data_stream_send(m.target_system, m.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)


def rel_alt():
    msg = m.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=2)
    return None if msg is None else msg.relative_alt / 1000.0


# Wait until PX4 says it is armable (EKF + GPS ok): poll SYS_STATUS health + try arming
t0 = time.time()
gps_ok = False
while time.time() - t0 < a.timeout:
    msg = m.recv_match(type=["GPS_RAW_INT", "STATUSTEXT"], blocking=True, timeout=1)
    if msg is None:
        continue
    if msg.get_type() == "GPS_RAW_INT" and msg.fix_type >= 3 and not gps_ok:
        gps_ok = True
        say(f"GPS 3D fix, {msg.satellites_visible} sats")
    if msg.get_type() == "STATUSTEXT":
        say(f"PX4: {msg.text}")
    if gps_ok:
        break
if not gps_ok:
    say("FAIL: no GPS fix from PX4 (Pegasus GPS sensor not reaching PX4?)")
    sys.exit(1)

armed = False
for attempt in range(30):
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
    ack = m.recv_match(type="COMMAND_ACK", blocking=True, timeout=2)
    if ack and ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM and ack.result == 0:
        armed = True
        break
    st = m.recv_match(type="STATUSTEXT", blocking=False)
    if st:
        say(f"PX4: {st.text}")
    time.sleep(2)
if not armed:
    say("FAIL: PX4 refused to arm (check QGC preflight messages)")
    sys.exit(1)
say("ARMED")

# PX4 interprets the MAV_CMD_NAV_TAKEOFF altitude as AMSL, so add the home altitude
gp = m.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=5)
home_amsl = (gp.alt - gp.relative_alt) / 1000.0 if gp else 0.0
m.mav.command_long_send(m.target_system, m.target_component,
                        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, float("nan"), float("nan"), float("nan"),
                        home_amsl + a.alt)
say(f"takeoff to {a.alt} m commanded")

t0 = time.time()
max_alt = 0.0
while time.time() - t0 < a.climb_timeout:
    h = rel_alt()
    if h is None:
        continue
    max_alt = max(max_alt, h)
    if h > 0.9 * a.alt:
        break
say(f"altitude reached: {max_alt:.2f} m")
reached = max_alt > 0.8 * a.alt

t0 = time.time()
while time.time() - t0 < a.hover:
    h = rel_alt()
    if h is not None:
        max_alt = max(max_alt, h)
say(f"hovered {a.hover:.0f} s, current altitude {rel_alt()}")

m.mav.command_long_send(m.target_system, m.target_component, mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0)
say("land commanded")
t0 = time.time()
landed = False
while time.time() - t0 < a.climb_timeout:
    msg = m.recv_match(type="EXTENDED_SYS_STATE", blocking=True, timeout=2)
    if msg and msg.landed_state == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND:
        landed = True
        break
say(f"landed: {landed}, max altitude {max_alt:.2f} m")
sys.exit(0 if reached and landed else 1)
