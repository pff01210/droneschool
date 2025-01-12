# ドローンエンジニア養成塾18期Day5課題
#
# 固定翼機体インスタンス処理
#
# SITLでplaneベースにVTOLモデルを設定する
# (1) GUIDEDモードはMPなどの操作で設定すると動作開始
# (2) WPはMPで設定後ファイル保存されたものをファイル読み込みではなくソースコードデータ展開して設定する
# (3) ARMは失敗するのでARM FORCEをMPで操作するとTAKEOFF実行する
# (4) AUTOモードでのWP移動の最終ポイントではRTLで一旦戻して再度AUTOで開始できるようにしている
# (5) modeを変えても確認できるまでかなり時間がかかる部分は他のスクリプトでも同様なので対応保留
#
# 以下を参考に処理する
# https://mavlink.io/en/mavgen_python/howto_requestmessages.html
# https://mavlink.io/en/messages/common.html
# https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html
# https://mavlink.io/en/messages/common.html#MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN
# https://www.colorado.edu/recuv/2015/05/25/mavlink-protocol-waypoints
#
import sys
import time
import signal
from pymavlink import mavutil

flcnt = 0
flstate = 0
lastmode = 'none'
target_alt = 10
staytime = 0
retrytime = 0
PAI = 3.14159265
TURN180 = PAI
TURN2LEFT = -( PAI / 2 )
INT_DISABLE = -1
stableCnt = 0
stableCheck = 2
isActive = False
activeCnt = 0 # statusはstandbyモード時activeモードと交互に現れるので回数で判断するためのカウンタ

# 状態遷移制御する場合の特別な状態番号のみ名前を付ける
# ※enumを使うと行数を浪費するので省略
STATE_INVALID = 37

tick = 0.2
# WayPointをMPで設定してファイルに保存後にテキストファイルなのでそのままコピペしてTABをカンマに変えてリスト化
wplist = [
[0,1,0,16,0,0,0,0,35.8793372,140.3396828,7.910000,1],
[1,0,3,16,0.00000000,0.00000000,0.00000000,0.00000000,35.88223680,140.34759680,10.000000,1],
[2,0,3,16,0.00000000,0.00000000,0.00000000,0.00000000,35.87564480,140.33767680,20.000000,1],
[3,0,3,16,0.00000000,0.00000000,0.00000000,0.00000000,35.87093440,140.33201920,20.000000,1],
[4,0,3,16,0.00000000,0.00000000,0.00000000,0.00000000,35.86623680,140.32271350,20.000000,1],
[5,0,3,16,0.00000000,0.00000000,0.00000000,0.00000000,35.86439360,140.31627520,20.000000,1],
[6,0,3,16,0.00000000,0.00000000,0.00000000,0.00000000,35.86484800,140.30970880,20.000000,1],
[7,0,3,16,0.00000000,0.00000000,0.00000000,0.00000000,35.86641280,140.30042880,20.000000,1],
[8,0,3,16,0.00000000,0.00000000,0.00000000,0.00000000,35.86954240,140.29377280,20.000000,1],
[9,0,3,16,0.00000000,0.00000000,0.00000000,0.00000000,35.87107520,140.29120000,20.000000,1],
[10,0,3,16,0.00000000,0.00000000,0.00000000,0.00000000,35.86992640,140.29034230,20.000000,1],
[11,0,3,16,0.00000000,0.00000000,0.00000000,0.00000000,35.86835840,140.29322240,20.000000,1],
[12,0,3,16,0.00000000,0.00000000,0.00000000,0.00000000,35.86544000,140.29936640,20.000000,1],
[13,0,3,16,0.00000000,0.00000000,0.00000000,0.00000000,35.86449920,140.30398720,20.000000,1],
[14,0,3,16,0.00000000,0.00000000,0.00000000,0.00000000,35.86376960,140.30909440,20.000000,1],
[15,0,3,16,0.00000000,0.00000000,0.00000000,0.00000000,35.86359360,140.31544320,20.000000,1],
[16,0,3,16,0.00000000,0.00000000,0.00000000,0.00000000,35.86418560,140.32069120,20.000000,1],
[17,0,3,16,0.00000000,0.00000000,0.00000000,0.00000000,35.86568000,140.32394240,20.000000,1],
[18,0,3,16,0.00000000,0.00000000,0.00000000,0.00000000,35.86780480,140.32828160,20.000000,1],
[19,0,3,16,0.00000000,0.00000000,0.00000000,0.00000000,35.87107520,140.33450240,20.000000,1],
[20,0,3,16,0.00000000,0.00000000,0.00000000,0.00000000,35.87507200,140.33854720,20.000000,1],
[21,0,3,16,0.00000000,0.00000000,0.00000000,0.00000000,35.87740480,140.34236160,20.000000,1],
[22,0,3,16,0.00000000,0.00000000,0.00000000,0.00000000,35.88088000,140.34785280,20.000000,1],
[23,0,3,16,0.00000000,0.00000000,0.00000000,0.00000000,35.88213120,140.34759680,20.000000,1]
]

def isPreArmOk(master: mavutil.mavfile) :
  return True

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

def delaySec2Cnt(sec):
  #global tick
  global staytime
  global retrytime
  staytime =  sec / tick
  retrytime = staytime * 1.5

def download_mission(master):
    mission = []

    # ミッションアイテムの数を取得
    master.mav.mission_request_list_send(
        master.target_system, master.target_component)
    mission_count = master.recv_match(
        type='MISSION_COUNT', blocking=True).count

    # ミッションアイテムをダウンロード
    for i in range(mission_count):
        master.mav.mission_request_int_send(
            master.target_system, master.target_component, i)
        item = master.recv_match(type='MISSION_ITEM_INT', blocking=True)
        new_waypoint = mavutil.mavlink.MAVLink_mission_item_int_message(
            master.target_system,
            master.target_component,
            item.seq,
            item.frame,
            item.command,
            item.current,
            item.autocontinue,
            item.param1, item.param2, item.param3, item.param4,
            item.x, item.y, item.z,
        )
        mission.append(new_waypoint)

    return mission

def upload_mission(master, mission):
    # ミッションをアップロード
    master.mav.mission_clear_all_send(
        master.target_system, master.target_component)
    master.mav.mission_count_send(
        master.target_system, master.target_component, len(mission))
    print('enumerate :',enumerate(mission))
    for i, item in enumerate(mission):
        master.mav.send(item)

    # アップロードしたミッションを機体に設定
    master.mav.mission_set_current_send(0, master.target_component, 0)
    master.mav.mission_request_list_send(
        master.target_system, master.target_component)

def print_mission(mission):
    for element in mission:
        print(element)

# 機体への接続（単体実行用：親スクリプトで接続していない時実行）
def setup(port) -> mavutil.mavfile:
  mission = []

  master: mavutil.mavfile = mavutil.mavlink_connection(
      port, source_system=1, source_component=90)
  master.wait_heartbeat()
  #mission = download_mission(master)
  #print("mission num :",len(mission))
  print("defined wplist num :",len(wplist))
#  '''
  # 定義したWPをFCにアップロードする
  master.mav.mission_clear_all_send(
        master.target_system, master.target_component)
  master.mav.mission_count_send(
        master.target_system, master.target_component, len(wplist))
  for i in range(len(wplist)):
      new_waypoint = mavutil.mavlink.MAVLink_mission_item_int_message(
           master.target_system,
           master.target_component,
           wplist[i][0],# seq
           wplist[i][2],# frame
           wplist[i][3],# comamnd
           wplist[i][1],# current
           wplist[i][11],# autocontinue
           wplist[i][4],# param1
           wplist[i][5],# param2
           wplist[i][6],# param3
           wplist[i][7],# param4
           int(wplist[i][8]*1e7), # x
           int(wplist[i][9]*1e7), # y
           wplist[i][10],         # z
           0,           # mission type
           )
      mission.append(new_waypoint)
  upload_mission(master, mission)
  #print_mission(mission)
#  '''
  return master

def flight(master: mavutil.mavfile, delay = 0.2):
    global flcnt
    global flstate
    global lastmode
    global staytime
    global tick
    global isActive
    global activeCnt
    nowmode = master.flightmode
    tick = delay
    try:
      #print('recv_match()',flcnt,flstate)
      recv = master.recv_match(type='HEARTBEAT', blocking=True)
      #print("recv.system_status :",recv.system_status)
      #print("mavutil.mavlink.MAV_STATE_ACTIVE :",mavutil.mavlink.MAV_STATE_ACTIVE)
      #print("mavutil.mavlink.MAV_STATE_STANDBY :",mavutil.mavlink.MAV_STATE_STANDBY)
      if recv.system_status == mavutil.mavlink.MAV_STATE_ACTIVE :
        if activeCnt<100 :
          activeCnt = activeCnt + 1
      elif recv.system_status == mavutil.mavlink.MAV_STATE_STANDBY :
        activeCnt = 0
      if activeCnt > 10 :
        isActive = True
      else :
        isActive = False

      #master.recv_match(type='SYS_STATUS',blocking=False)
      nowmode = master.flightmode
      if lastmode != nowmode :
        print(nowmode)
        lastmode = nowmode
        print("flstate :",flstate,"flcnt :",flcnt)
    except:
      pass
    if nowmode == 'GUIDED' :
      # GUIDEDに切り替わった初期状態
      # GUIDEDへの切り替えはプロポなど外部からの操作で行う
      if flstate == 0 :
        flstate = 1
        print('ACTIVATE GUIDED MODE')
    elif nowmode != 'AUTO' and nowmode != 'LAND' and nowmode != 'RTL' :
      # GUIDED/AUTO/LAND/RTLモード以外は状態初期化
      # print("initialize")
      flstate = 0
    if flstate == 1 :
      # 飛行制御処理開始
      flstate = flstate + 1
      print('FLIGHT CONTROL START')
      # 初期の機体状態を取得する
      recv = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    elif flstate == 2 :
      # ARM
      if isPreArmOk(master) :
        master.arducopter_arm()
        ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=10)
        #'''
        if ack and ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM and ack.result == 0:
          # ARM確認OK
          flstate = flstate + 1
          print('ARMED')
        else :
          print('ARM FAILED ack :',ack)
          # flstate = STATE_INVALID #無効にする 
        #'''
        #master.motors_armed_wait()
        #flstate = flstate + 1
    elif flstate == 3 :
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,0, 0, 0, 0, 0, 0, 0, target_alt)
        flstate = flstate + 1
    elif flstate == 4 :
        master.set_mode_auto()
        flstate = flstate + 1
    elif flstate == 5 :
        # WPによる飛行を一巡して一旦RTLしたのでもう一度飛行開始させる
        # WPのジャンプは制御の多様性を妨げるので基本1巡で一旦RTLさせる
        if nowmode=='RTL' :
          # AUTOにモードに戻す
          flstate = 4

    flcnt = flcnt + 1

if __name__ == "__main__":
    plane: mavutil.mavfile = setup("tcp:192.168.1.37:5762")
    print('plane接続 tick :',tick)
    intervalReq(plane)  # GLOBAL_POSITION_INTインターバル要求
    while True:
        #print('before')
        flight(plane)
        #print('after')
        time.sleep(tick)

