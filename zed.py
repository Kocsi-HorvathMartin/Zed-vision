import pyzed.sl as sl
import os
import keyboard
import math
import numpy as np
from pymavlink import mavutil
import time

# Armolás/disarmolás
def arm(arm):
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component, 
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 
        0, arm, 0, 0, 0, 0, 0, 0
    )
    msg = connection.recv_match(type='COMMAND_ACK', blocking=True)
    return msg

# Leszállás és disarmolás
def leszall():
    # Leszállás parancs küldése
    connection.mav.command_long_send(
        connection.target_system,                       
        connection.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    msg = connection.recv_match(type='COMMAND_ACK', blocking=True)
    print(msg)
    print(arm(0))

# Jelenlegi pozíció lekérdezése
def akt_poz():
    
    msg = connection.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=5)
    return msg

#Kamera kép által számított pozíció  küldése
def send_vision_position_estimate(x, y, z, roll, pitch, yaw):
    msg = connection.mav.vision_position_estimate_encode(
        int(time.time() * 1000),  # Időbélyeg (ms)
        x,  # X pozíció (méter)
        y,  # Y pozíció (méter)
        z,  # Z pozíció (méter)
        roll,  # Roll szög (radián)
        pitch,  # Pitch szög (radián)
        yaw  # Yaw szög (radián)
    )
    connection.mav.send(msg)
    print(f"Vision Position Estimate sent: x={x}, y={y}, z={z}, roll={roll}, pitch={pitch}, yaw={yaw}")

# Kvaternion normalizálása
def normalize_quaternion(q):
    norm = np.linalg.norm(q)
    return q / norm

# Kvaternion átalakítása Euler szögekké
def quaternion_to_euler(q):
    w, x, y, z = q
    q = normalize_quaternion(q)
    w, x, y, z = q

    # Pitch (x-tengely forgatás)
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    pitch = math.atan2(t0, t1)

    # Roll (y-tengely forgatás)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    roll = math.asin(t2)

    # Yaw (z-tengely forgatás)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4) * (-1)

    return roll, pitch, yaw

# Pozíció lekérdezése
def translation():
    py_translation = sl.Translation()
    tx = zed_pose.get_translation(py_translation).get()[0]
    ty = zed_pose.get_translation(py_translation).get()[1]
    tz = zed_pose.get_translation(py_translation).get()[2]
    return tx, ty, tz * (-1)

# Orientáció lekérdezése
def orient():
    py_orientation = sl.Orientation()
    orientation = [
        zed_pose.get_orientation(py_orientation).get()[3],
        zed_pose.get_orientation(py_orientation).get()[0],
        zed_pose.get_orientation(py_orientation).get()[1],
        zed_pose.get_orientation(py_orientation).get()[2]
    ]
    return orientation

# ZED kamera betöltése
zed = sl.Camera()

# Konfigurációs paraméterek beállítása
init_params = sl.InitParameters()
init_params.camera_resolution = sl.RESOLUTION.AUTO  # Automatikus felbontás
init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP  # Jobbkezes koordináta rendszer
init_params.coordinate_units = sl.UNIT.METER  # Mértékegység méterben

# Kamera megnyitása
err = zed.open(init_params)
if err != sl.ERROR_CODE.SUCCESS:
    exit(-1)

# Kamera információk lekérése (sorozatszám)
zed_serial = zed.get_camera_information().serial_number
print("Hello! Ez az én sorozatszámom: ", zed_serial)

# Pozícionális követés engedélyezése alapértelmezett paraméterekkel
py_transform = sl.Transform()  # Transform objektum létrehozása a TrackingParameters objektumhoz
tracking_parameters = sl.PositionalTrackingParameters(_init_pos=py_transform)

err = zed.enable_positional_tracking(tracking_parameters)
if err != sl.ERROR_CODE.SUCCESS:
    print("Pozícionális követés engedélyezése: "+repr(err)+". Kilépés.")
    zed.close()
    exit()

# Kamera pozíció követése
zed_pose = sl.Pose()
runtime_parameters = sl.RuntimeParameters()

# MAVLink kapcsolat létrehozása
connection = mavutil.mavlink_connection('127.0.0.1:14552')
connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (connection.target_system, connection.target_component))

#Pozíció stream indítása 1000us-es intervallummal
connection.mav.command_long_send(
        connection.target_system,
        connection.target_component, 
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 
        0, 32, 1000, 0, 0, 0, 0, 1
    )

while not keyboard.is_pressed('h'):
    if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
        os.system('cls')
        
        # Pozíció lekérése
        zed.get_position(zed_pose, sl.REFERENCE_FRAME.WORLD)
        
        # Transzláció és időbélyeg lekérése
        ty, tx, tz = translation()

        # Orientáció kvaternion lekérése
        orientation = orient()

        # Kvaternion átalakítása Euler szögekké
        roll, pitch, yaw = quaternion_to_euler(orientation)
        send_vision_position_estimate(tx, ty, tz, roll, pitch, yaw)
        print(akt_poz())

    # Új kép lekérése
    err = zed.grab(runtime_parameters)
    if err != sl.ERROR_CODE.SUCCESS:
        leszall()

    # Armolás billentyűparancsok
    if keyboard.is_pressed('1'):
        arm(1)
    if keyboard.is_pressed('0'):
        print(arm(0))
        if arm(0).result!=0:
            leszall()

# Kamera bezárása
zed.disable_positional_tracking()
zed.close()
print(arm(0))
if arm(0).result!=0:
    leszall()