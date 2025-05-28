# PreARMチェックを周期的に実行するテストスクリプト
#
import sys
import time
import datetime
import signal
from pymavlink import mavutil

lastmode = 'none'
lastarmable = 'unknown'
prearmReqDone = False
cnt = 0

def process_message(master: mavutil.mavfile):
    """すべての受信メッセージを処理する"""
    global lastarmable
    global cnt
    msg = master.recv_msg()
    if msg is None:
        return

    # タイムスタンプを生成（ミリ秒まで）
    timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

    # メッセージ種別に応じて処理を振り分ける
    if msg.get_type() == "GLOBAL_POSITION_INT":
        cnt = cnt + 1
        #print(f"[{timestamp}] GLOBAL_POSITION_INT: lat={msg.lat}, lon={msg.lon}, alt={msg.alt}")

    elif msg.get_type() == "STATUSTEXT":
        print(f"[{timestamp}] STATUSTEXT: {msg.text}")

    elif msg.get_type() == "SYS_STATUS":
        if msg.onboard_control_sensors_health & mavutil.mavlink.MAV_SYS_STATUS_PREARM_CHECK :
            armable = 'true'
        else :
            armable = 'false'
        if armable != lastarmable:
            timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]  # ミリ秒まで表示
            print(f"[{timestamp}] SYS_STATUS: onboard_control_sensors_health={msg.onboard_control_sensors_health:#034b}")
            print(f"[{timestamp}] armable: {armable}, {msg}")
            lastarmable = armable

    #else:
        # その他のメッセージ（デバッグ用途）
    #    print(f"[{timestamp}] Unhandled: {msg.get_type()}")

# PreARMチェックの実行を要求する
def reqPrearmCheck(master: mavutil.mavfile) :
  master.mav.command_long_send(master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_RUN_PREARM_CHECKS, 0,
        0, 0, 0, 0, 0, 0, 0)
  ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=10)
  #print("PREARM CHECK Command Ack :",ack)
  if ack :
#    print("ack.command : ",ack.command)
#    print("ack.result : ",ack.result)
    if (ack.command == mavutil.mavlink.MAV_CMD_RUN_PREARM_CHECKS and
        ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED) :
      oneshotReq(master, mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS)
      return True
    else :
      return False
  else :
    return False

def intervalReq(master: mavutil.mavfile, intsec=0.1, msgid=mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT):
  if intsec < 0 :
    intusec = INT_DISABLE # desable
  else :
    intusec = intsec * 100000
  master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
    0, msgid, intusec, 0, 0, 0, 0, 0)
  ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=10)
  #if ack :
  #  print("intervalReq ack :",ack)

def oneshotReq(master: mavutil.mavfile, msgid=mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT):
  master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
    0, msgid, 0, 0, 0, 0, 0, 0)
  ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=10)
  #if ack :
  #  print("oneshotReq ack :",ack)

# 機体への接続（単体実行用：親スクリプトで接続していない時実行）
# SITL : tcp:127.0.0.1:5762
# mavlink-routerd : 127.0.0.1:14551
def setup() -> mavutil.mavfile:
  # master: mavutil.mavfile = mavutil.mavlink_connection(
  #  "/dev/serial0", baud=115200, source_system=1, source_component=90)
  # mavlink-router経由での接続（uart接続はmavlink-routerに任せる）
  master: mavutil.mavfile = mavutil.mavlink_connection(
#      "tcp:127.0.0.1:5762", source_system=1, source_component=90)
      "127.0.0.1:14552", source_system=1, source_component=90)

  master.wait_heartbeat()
  intervalReq(master)  # GLOBAL_POSITION_INTインターバル要求
  #intervalReq(master,0.1,mavutil.mavlink.MAVLINK_MSG_ID_STATUSTEXT)  # STATUSTEXTインターバル要求
  #STATUSTEXTはINTERVAL指定はDENIEDで返るので設定不可
  #request_statustext(master)
  # 上記もFAILEDで返る
  return master

def check(master: mavutil.mavfile, delay = 0.2):
    global lastmode
    global prearmReqDone
    if prearmReqDone==False :
      prearmReqDone = reqPrearmCheck(master)

    # 機体状態を取得する
    process_message(master)
    nowmode = master.flightmode
    if nowmode != lastmode :
      print(nowmode)
      lastmode = nowmode

if __name__ == "__main__":
    master: mavutil.mavfile = setup()
    print('接続')
    while True:
        check(master,0.2)
        #time.sleep(0.1)

