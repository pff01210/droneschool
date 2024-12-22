from pymavlink import mavutil
import time

# 機体への接続(TCPの部分は個人のIPv4のIPアドレス)
master: mavutil.mavfile = mavutil.mavlink_connection(
    "tcp:172.21.15.170:14551", source_system=1, source_component=90)
master.wait_heartbeat()

# ターゲットシステムID、コンポーネントIDを表示
print(f"target_system: {master.target_system}, target_component: {master.target_component}")

# HEARTBEATメッセージを1秒おきに送信
while True:
    time.sleep(1)

    # メッセージ直接送信
    master.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
        mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
        0, 0, 0)

    # メッセージ作成して送信
    # to_send_msg = master.mav.heartbeat_encode(
    #     mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
    #     mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
    #     0, 0, 0 )
    # master.mav.send(to_send_msg)