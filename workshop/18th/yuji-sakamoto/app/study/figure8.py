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

def oneshotReq(master: mavutil.mavfile, msgid=mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT):
  master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
    0, msgid, 0, 0, 0, 0, 0, 0)
  ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=10)
  if ack :
    print("oneshotReq ack :",ack)

# GUIDEDモード時のプリアームチェックを行う
def isPreArmOk(master: mavutil.mavfile) :
  master.mav.command_long_send(master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_RUN_PREARM_CHECKS, 0,
        0, 0, 0, 0, 0, 0, 0)
  ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=10)
  print("PREARM CHECK Command Ack :",ack)
  if ack :
    print("ack.command : ",ack.command)
    print("ack.result : ",ack.result)
  else :
    return False
  if (ack.command == mavutil.mavlink.MAV_CMD_RUN_PREARM_CHECKS and
      ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED) :
    oneshotReq(master, mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS)
    recv = master.recv_match(type='SYS_STATUS', blocking=True, timeout=10)
    if recv :
      print("SYS_STATUS recv : ",recv)
      print("SYS_STATUS recv.onboard_control_sensors_present: {:#034b}"
        .format(recv.onboard_control_sensors_present))
      print("SYS_STATUS recv.onboard_control_sensors_enabled: {:#034b}"
        .format(recv.onboard_control_sensors_enabled))
      print("SYS_STATUS recv.onboard_control_sensors_health : {:#034b}"
        .format(recv.onboard_control_sensors_health))
      print("MAV_SYS_STATUS_PREARM_CHECK                    : {:#034b}"
        .format(mavutil.mavlink.MAV_SYS_STATUS_PREARM_CHECK))
      # SYS_STSTUSのonboard_control_sensor_healthをチェックして判断する
      # 判定途中でモードが切り替わっていると動作不正となるので再度モードをチェック
      if get_current_mode(master) == 'GUIDED' :
        return recv.onboard_control_sensors_health & mavutil.mavlink.MAV_SYS_STATUS_PREARM_CHECK
      else :
        print("Error : Invalid Mode Change")
        return False
    else :
      return False
  else :
    return False

def send_local_ned(master, x, y, z, yaw_deg):
    yaw_rad = math.radians(yaw_deg)
    type_mask = 0b000111111000  # yaw有効
    master.mav.set_position_target_local_ned_send(
        0,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        type_mask,
        x, y, z,
        0, 0, 0,     # 速度無視
        0, 0, 0,     # 加速度無視
        0, yaw_rad   # yaw有効
    )

def setmode(master: mavutil.mavfile,mode):
    print(mode,"にモード変更")
    master.mav.command_long_send(
          master.target_system, master.target_component,
          mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
          mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, master.mode_mapping()[mode], 0, 0, 0, 0, 0)

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

import math

def generate_figure8_path(center_offset, radius, altitude, loops=2, points_per_circle=72):
    path = []

    for loop in range(loops):
        # ▶ 右回り円：左端スタート（θ=270° → 時計回り）
        cx, cy = center_offset, 0
        for i in range(points_per_circle + 1):
            # ▶ 右回り：左端スタート（θ=180°）→ 時計回り
            theta = math.radians(180 - i * 360 / points_per_circle)
            x = cx + radius * math.cos(theta)
            y = cy + radius * math.sin(theta)

            next_theta = math.radians(270 - (i + 1) * 360 / points_per_circle)
            next_x = cx + radius * math.cos(next_theta)
            next_y = cy + radius * math.sin(next_theta)
            dx = next_x - x
            dy = next_y - y
            yaw = math.degrees(math.atan2(dy, dx))  # 進行ベクトルからyaw算出

            path.append((x, y, -altitude, yaw))

        # ▶ 左回り円：右端スタート（θ=90° → 反時計回り）
        cx, cy = -center_offset, 0
        for i in range(points_per_circle + 1):
            theta = math.radians(90 + i * 360 / points_per_circle)  # θ増加で反時計回り
            x = cx + radius * math.cos(theta)
            y = cy + radius * math.sin(theta)

            next_theta = math.radians(90 + (i + 1) * 360 / points_per_circle)
            next_x = cx + radius * math.cos(next_theta)
            next_y = cy + radius * math.sin(next_theta)
            dx = next_x - x
            dy = next_y - y
            yaw = math.degrees(math.atan2(dy, dx))

            path.append((x, y, -altitude, yaw))

    return path

def flight8(master: mavutil.mavfile):
    # GUIDEDモードへ
    nowmode = get_current_mode(master)
    if nowmode != 'GUIDED':
        return

    # アーミング
    if isPreArmOk(master) :
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0)
        master.motors_armed_wait()
        print("Armed")
    else:
        return

    # 離陸
    altitude = 3.5 + 2 # 試験仕様に合わせて高度3.5m
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, altitude)
    time.sleep(4)

    # 例：generate_figure8_path呼び出し
    path = generate_figure8_path(center_offset=5, radius=5, altitude=5, loops=1, points_per_circle=4)

    for idx, (x, y, z, yaw) in enumerate(path):
        send_local_ned(master, x, y, z, yaw)
        mode = get_current_mode(master)
        print(f"WP{idx:02d}: mode={mode}, pos=({x:.2f}, {y:.2f}, {z:.1f}), yaw={yaw:.1f}")
        if mode != 'GUIDED':
            print("モードがGUIDED以外に変化。飛行中断。")
            return
        time.sleep(1.2)  # 遷移の猶予時間（必要に応じて調整）

    # ホバリング
    send_local_ned(master, 0, 0, -altitude, 0)
    print("八の字2周完了")
    nowmode = get_current_mode(master)
    if nowmode != 'GUIDED':
        return
    time.sleep(5)
    # LAND
    setmode(master, 'LAND')

if __name__ == "__main__":
    master: mavutil.mavfile = setup()
    print('接続')
    while True:
        flight8(master)
        time.sleep(0.05)

