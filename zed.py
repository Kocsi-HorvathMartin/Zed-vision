import pyzed.sl as sl
import os
import keyboard
import math
import numpy as np
from pymavlink import mavutil
import time

def arm(arm):            #Armolás/disarmolás
        connection.mav.command_long_send(connection.target_system,
                                 connection.target_component, 
                                 mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 
                                 0, arm, 0,0,0,0,0,0)
        msg=connection.recv_match(type='COMMAND_ACK', blocking=True)
        print(msg)

def akt_poz():           #Jelenlegi pozícióba
    connection.mav.command_long_send(connection.target_system,
                                 connection.target_component, 
                                 mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 
                                 0, 32, 0,0,0,0,0,1)
    msg=connection.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=5)
    return msg

def send_vision_position_estimate(x, y, z, roll, pitch, yaw):
    
    # Create a VISION_POSITION_ESTIMATE message
    msg = connection.mav.vision_position_estimate_encode(
        int(time.time() * 1000),  # Timestamp in milliseconds since boot
        x,  # X position in meters
        y,  # Y position in meters
        z,  # Z position in meters
        roll,  # Roll angle in radians
        pitch,  # Pitch angle in radians
        yaw  # Yaw angle in radians
    )
    # Send the message
    connection.mav.send(msg)
    print(f"Vision Position Estimate sent: x={x}, y={y}, z={z}, roll={roll}, pitch={pitch}, yaw={yaw}")

# Function to normalize a quaternion
def normalize_quaternion(q):
    norm = np.linalg.norm(q)
    return q / norm

def quaternion_to_euler(q):
    w, x, y, z = q

    # Normalize the quaternion
    q = normalize_quaternion(q)
    w, x, y, z = q

    # Pitch (x-axis rotation)
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    pitch = math.atan2(t0, t1)

    # Roll (y-axis rotation)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    roll = math.asin(t2)

    # Yaw (z-axis rotation)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)*(-1)

    return roll, pitch, yaw

zed = sl.Camera()

# Set configuration parameters
init_params = sl.InitParameters()
init_params.camera_resolution = sl.RESOLUTION.AUTO # Use HD720 or HD1200 video mode (default fps: 60)
init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP # Use a right-handed Y-up coordinate system
init_params.coordinate_units = sl.UNIT.METER # Set units in meters

err = zed.open(init_params)
if (err != sl.ERROR_CODE.SUCCESS) :
    exit(-1)

# Get camera information (serial number)
zed_serial = zed.get_camera_information().serial_number
print("Hello! This is my serial number: ", zed_serial)

connection=mavutil.mavlink_connection('127.0.0.1:14550')
connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (connection.target_system, connection.target_component))
#mode(4)
#arm(1)
time.sleep(2)
# Enable positional tracking with default parameters
py_transform = sl.Transform()  # First create a Transform object for TrackingParameters object
tracking_parameters = sl.PositionalTrackingParameters(_init_pos=py_transform)
err = zed.enable_positional_tracking(tracking_parameters)
if err != sl.ERROR_CODE.SUCCESS:
    print("Enable positional tracking : "+repr(err)+". Exit program.")
    zed.close()
    exit()

# Track the camera positionc
zed_pose = sl.Pose()

zed_sensors = sl.SensorsData()
runtime_parameters = sl.RuntimeParameters()
    
can_compute_imu = zed.get_camera_information().camera_model != sl.MODEL.ZED
while not keyboard.is_pressed('c'):
    if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
        # Get the pose of the left eye of the camera with reference to the world frame
        zed.get_position(zed_pose, sl.REFERENCE_FRAME.WORLD)
        
        # Display the translation and timestamp
        py_translation = sl.Translation()
        tx = zed_pose.get_translation(py_translation).get()[0]
        ty = zed_pose.get_translation(py_translation).get()[1]
        tz = zed_pose.get_translation(py_translation).get()[2]

        # Display the orientation quaternion
        py_orientation = sl.Orientation()
        orientation = [zed_pose.get_orientation(py_orientation).get()[3], zed_pose.get_orientation(py_orientation).get()[0], zed_pose.get_orientation(py_orientation).get()[1], zed_pose.get_orientation(py_orientation).get()[2]]
        # Convert quaternion to Euler angles
        roll, pitch, yaw = quaternion_to_euler(orientation)
        send_vision_position_estimate(tx,ty,tz*(-1),roll,pitch,yaw)
        print(akt_poz())
        os.system('cls')
    if keyboard.is_pressed('1'):
        arm(1)
    if keyboard.is_pressed('0'):
        arm(0)
# Close the camera
zed.disable_positional_tracking()
zed.close()