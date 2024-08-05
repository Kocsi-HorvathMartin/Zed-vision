import pyzed.sl as sl
import os
import keyboard
import math
import numpy as np
from pymavlink import mavutil
import time

# Function to arm or disarm the drone
def arm(arm):            
        connection.mav.command_long_send(connection.target_system,
                                 connection.target_component, 
                                 mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 
                                 0, arm, 0,0,0,0,0,0)
        msg=connection.recv_match(type='COMMAND_ACK', blocking=True)
        print(msg)

# Function to land the drone
def leszall():          
    connection.mav.command_long_send(connection.target_system,                       #Leszállás
                                     connection.target_component,
                                     mavutil.mavlink.MAV_CMD_NAV_LAND,
                                     0,0,0,0,0,0,0,0)
    msg=connection.recv_match(type='COMMAND_ACK', blocking=True)
    print(msg)
    arm(0)

# Function to get the current position of the drone
def akt_poz():          
    msg=connection.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=0.01)
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

# Function to convert quaternion to Euler angles
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

# Function to get translation from the ZED camera
def translation():
    py_translation = sl.Translation()
    tx = zed_pose.get_translation(py_translation).get()[0]
    ty = zed_pose.get_translation(py_translation).get()[1]
    tz = zed_pose.get_translation(py_translation).get()[2]
    return tx,ty,tz*(-1)

# Function to get orientation from the ZED camera
def orient():
    py_orientation = sl.Orientation()
    orientation = [zed_pose.get_orientation(py_orientation).get()[3], zed_pose.get_orientation(py_orientation).get()[0], zed_pose.get_orientation(py_orientation).get()[1], zed_pose.get_orientation(py_orientation).get()[2]]
    return orientation

# Function to change the flight mode of the drone
def mode(mode):     
    connection.mav.set_mode_send(
                connection.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode)


# Function to move the drone to a specified point (x, y, z)
def mozgas(pont):           
    connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10,connection.target_system,
                                                                                      connection.target_component, 
                                                                                      mavutil.mavlink.MAV_FRAME_LOCAL_NED, 
                                                                                      int(0b110111111000), pont[0], pont[1], pont[2], 10, 10, 5, 0, 0, 0, 0, 0))

# Function to constrain the heading angle between 0 and 360 degrees
def hatar_szog(szog):   
    if szog>=360:
        szog-=360
    if szog<0:
        szog+=360
    return szog


# Function to move in the direction of the specified heading angle
def head_irany(fok):   
    global x,y,z
    msg=akt_poz()
    z=msg.z
    fok=hatar_szog(fok)
    fok=round(math.radians(fok),2)
    y+=0.1*math.sin(fok)
    x+=0.1*math.cos(fok)
    mozgas()

# Function to take off    
def felszall():         
    connection.mav.command_long_send(connection.target_system,                      #GUIDED MODE-ba váltás
                                 connection.target_component,
                                 mavutil.mavlink.MAV_CMD_DO_SET_MODE, 
                                 0, 1, 4, 0,0,0,0,0)
    msg=connection.recv_match(type='COMMAND_ACK', blocking=True)
    print(msg)

    print(arm(1))

    connection.mav.command_long_send(connection.target_system,                      #Felszállás
                                 connection.target_component, 
                                 mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 
                                 0, 0,0,0,0,0,0, 1)
    msg=connection.recv_match(type='COMMAND_ACK', blocking=True)
    print(msg)

# Function to land the drone
def leszall():          
    connection.mav.command_long_send(connection.target_system,
                                     connection.target_component,
                                     mavutil.mavlink.MAV_CMD_NAV_LAND,
                                     0,0,0,0,0,0,0,0)
    msg=connection.recv_match(type='COMMAND_ACK', blocking=True)
    print(msg)

    global z
    z=0.0

# Function to send vision position estimates and monitor position errors
def vision_position_send(etx,ety,etz):
    global vx,vy,vz
    if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
        os.system('cls')
        # Get the pose of the left eye of the camera with reference to the world frame
        zed.get_position(zed_pose, sl.REFERENCE_FRAME.WORLD)
        
        # Display the translation and timestamp
        ty,tx,tz=translation()

        # Display the orientation quaternion
        orientation=orient()

        # Convert quaternion to Euler angles
        roll, pitch, yaw = quaternion_to_euler(orientation)
        send_vision_position_estimate(tx,ty,tz,roll,pitch,yaw)
        msg=akt_poz()
        if msg!=None:
            if abs(vx)<abs(msg.vx):
                vx=msg.vx
            if abs(vy)<abs(msg.vy):
                vy=msg.vy
            if abs(vz)<abs(msg.vz):
                vz=msg.vz
        print(msg)
        if abs(etx-tx)>=0.2 or abs(ety-ty)>=0.2 or abs(etz-tz)>=0.2:
            leszall()

    # Try to grab a new frame
    err = zed.grab(runtime_parameters)
    if err != sl.ERROR_CODE.SUCCESS:
        leszall()

    return tx,ty,tz

# Initialize the ZED camera
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

# Enable positional tracking with default parameters
py_transform = sl.Transform()  # First create a Transform object for TrackingParameters object
tracking_parameters = sl.PositionalTrackingParameters(_init_pos=py_transform)
err = zed.enable_positional_tracking(tracking_parameters)
if err != sl.ERROR_CODE.SUCCESS:
    print("Enable positional tracking : "+repr(err)+". Exit program.")
    zed.close()
    exit()

# Track the camera position
zed_pose = sl.Pose()
runtime_parameters = sl.RuntimeParameters()

# Establish MAVLink connection
connection=mavutil.mavlink_connection('COM7')       #127.0.0.1:14552
connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (connection.target_system, connection.target_component))

# Set message interval for VISION_POSITION_ESTIMATE
connection.mav.command_long_send(connection.target_system,
                                 connection.target_component, 
                                 mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 
                                 0, 32, 100000,0,0,0,0,1)

# Initialize variables
angle=0
etx=0.0
ety=0.0
etz=0.0
vx=0
vy=0
vz=0

# Define waypoints
destination=[
    [0.6,-0.79,-0.27],
    [0.6,0.0,-0.27],
    [0.6,0.8,-0.27],
    [0.6,0.8,-0.76],
    [0.6,0.0,-0.76],
    [0.6,-0.79,-0.76],
    [0.6,-0.79,-1.23],
    [0.6,0.0,-1.23],
    [0.6,0.8,-1.23],
    [0.6,0.8,-1.71],
    [0.6,0.0,-1.71],
    [0.6,-0.79,-1.71]
]
etx,ety,etz=vision_position_send(etx,ety,etz)
felszall()
mozgas(destination[0])
i=0

# Main loop to follow waypoints
while i<12:
    etx,ety,etz=vision_position_send(etx,ety,etz)
    if etx<=destination[i][0]+0.05 and etx>=destination[i][0]-0.05:
        if ety<=destination[i][1]+0.05 and ety>=destination[i][1]-0.05:
            if etz<=destination[i][2]+0.05 and etz>=destination[i][2]-0.05:
                i+=1
                if i<12:
                    mozgas(destination[i])
    print(f'Előző pont sorszáma: {i}.')
    
print(f"vx:{vx}\tvy:{vy}\tvz:{vz}")

# Close the camera
zed.disable_positional_tracking()
zed.close()
arm(0)
