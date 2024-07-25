from pymavlink import mavutil
import time

def akt_poz():           #Jelenlegi pozícióba
    connection.mav.command_long_send(connection.target_system,
                                 connection.target_component, 
                                 mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 
                                 0, 32, 0,0,0,0,0,1)
    msg=connection.recv_match(type='LOCAL_POSITION_NED', blocking=True)
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


# Example usage
x_position = 10.0   # X position in meters
y_position = 2.0   # Y position in meters
z_position = -1.0  # Z position in meters
roll_angle = 9.1   # Roll in radians
pitch_angle = 6.2  # Pitch in radians
yaw_angle = 1.0    # Yaw in radians

connection=mavutil.mavlink_connection('tcp:127.0.0.1:5762')
connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (connection.target_system, connection.target_component))
while True:
    send_vision_position_estimate(x_position, y_position, z_position, roll_angle, pitch_angle, yaw_angle)