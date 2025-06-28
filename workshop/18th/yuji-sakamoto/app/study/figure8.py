from pymavlink import mavutil
import math
import time

mode_map = {
    "STABILIZE": 0,
    "ACRO": 1,
    "ALT_HOLD": 2,
    "AUTO": 3,
    "GUIDED": 4,
    "LOITER": 5,
    "RTL": 6,
    "CIRCLE": 7,
    "LAND": 9,
    "DRIFT": 11,
    "SPORT": 13,
    "FLIP": 14,
    "AUTOTUNE": 15,
    "POSHOLD": 16,
    "BRAKE": 17,
    "THROW": 18,
    "SMART_RTL": 21
}

def get_current_mode(master) -> str:
    """
    フライトコントローラ（component_id = 1）からの HEARTBEAT のみ受け入れて
    現在のフライトモード名を返す（custom_mode -> モード名に変換）
    """
    from pymavlink import mavutil

    while True:
        hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
        if hb is None:
            return "UNKNOWN"
        if hb.get_srcComponent() != 1:
            continue  # フライトコントローラ以外は無視

        if not hasattr(hb, "custom_mode"):
            return "UNKNOWN"

        mode_id = hb.custom_mode
        mode_name = next((k for k, v in mode_map.items() if v == mode_id), f"UNKNOWN({mode_id})")
        return mode_name

def intervalReq(master: mavutil.mavfile, intsec=0.1, msgid=mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT):
  global flcnt
  if intsec < 0 :
    intusec = INT_DISABLE # desable
    flcnt = 0
  else :
    intusec = intsec * 100000
  master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
    0, msgid, intusec, 0, 0, 0, 0, 0)
  ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=10)
  if ack :
    print("intervalReq ack :",ack)

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

def flight8(master: mavutil.mavfile):
    # GUIDEDモードへ
    nowmode = get_current_mode(master)
    if nowmode != 'GUIDED':
        return

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
    time.sleep(5)

# 機体への接続（単体実行用：親スクリプトで接続していない時実行）
# SITL : tcp:127.0.0.1:5762
# mavlink-routerd : 127.0.0.1:14551
def setup() -> mavutil.mavfile:
  # master: mavutil.mavfile = mavutil.mavlink_connection(
  #  "/dev/serial0", baud=115200, source_system=1, source_component=90)
  # mavlink-router経由での接続（uart接続はmavlink-routerに任せる）
  global lasttime
  master: mavutil.mavfile = mavutil.mavlink_connection(
      "tcp:127.0.0.1:5762", source_system=1, source_component=90)
#      "127.0.0.1:14551", source_system=1, source_component=90)

  master.wait_heartbeat()
  intervalReq(master)  # GLOBAL_POSITION_INTインターバル要求
  lasttime = time.monotonic()

  return master

if __name__ == "__main__":
    master: mavutil.mavfile = setup()
    print('接続')
    while True:
        flight8(master)
        time.sleep(0.05)

