from pymavlink import mavutil

connection=mavutil.mavlink_connection('tcp:127.0.0.1:5762')
connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (connection.target_system, connection.target_component))

connection.mav.mission_clear_all_send(connection.target_system, connection.target_component,0)
msg=connection.recv_match(type='MISSION_ACK',blocking=True)
print(msg)