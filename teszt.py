from pymavlink import mavutil

def akt_poz():           #Jelenlegi pozícióba
    connection.mav.command_long_send(connection.target_system,
                                 connection.target_component, 
                                 mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 
                                 0, 32, 0,0,0,0,0,1)
    msg=connection.recv_match(type='LOCAL_POSITION_NED', blocking=True)
    return msg


connection=mavutil.mavlink_connection('127.0.0.1:14550')
connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (connection.target_system, connection.target_component))
akt_poz()