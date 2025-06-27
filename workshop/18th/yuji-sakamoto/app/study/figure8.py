from pymavlink import mavutil
import math
import time

# TCP接続（ポート5762）
master = mavutil.mavlink_connection('tcp:127.0.0.1:5762')
master.wait_heartbeat()
print("接続完了")

# GUIDEDモードへ
master.set_mode_apm('GUIDED')

# アーミング
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0, 1, 0, 0, 0, 0, 0, 0)
master.motors_armed_wait()
print("Armed")

# 離陸
altitude = 3.5  # 試験仕様に合わせて高度3.5m
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0, 0, 0, 0, 0, 0, 0, altitude)
time.sleep(8)

# 八の字パラメータ
radius = 5
center_offset = 10
points_per_circle = 12  # 30度刻み

def send_local_ned(x, y, z, yaw_deg):
    yaw_rad = math.radians(yaw_deg)
    master.mav.set_position_target_local_ned_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        int(0b110111111000),
        x, y, z,
        0, 0, 0,
        0, 0, 0,
        0, yaw_rad)

# 八の字を2周
for loop in range(2):
    # 右回り（右円）
    for i in range(points_per_circle + 1):
        theta = math.radians(360 - i * 360 / points_per_circle)
        x = radius * math.cos(theta)
        y = center_offset / 2 + radius * math.sin(theta)
        yaw = math.degrees(math.atan2(-x, -y))
        send_local_ned(x, y, -altitude, yaw)
        time.sleep(1.5)

    # 左回り（左円）
    for i in range(points_per_circle + 1):
        theta = math.radians(i * 360 / points_per_circle)
        x = radius * math.cos(theta)
        y = -center_offset / 2 + radius * math.sin(theta)
        yaw = math.degrees(math.atan2(-x, -y))
        send_local_ned(x, y, -altitude, yaw)
        time.sleep(1.5)

# ホバリングして終了
send_local_ned(0, 0, -altitude, 0)
print("八の字2周完了")
