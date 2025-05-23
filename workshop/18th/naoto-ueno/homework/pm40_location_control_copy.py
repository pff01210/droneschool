import conn

# const
ADDR = "127.0.0.1:14550"
SRC_SYS_NO = 1
SRC_COM_NO = 9
MODE_GUIDED = "GUIDED"
TARGET_LAT = 35.8787276
TARGET_LON = 140.3388137
TARGET_ALT = 4

# Message code
GLOBAL_POSITION_INT = 33

# flight
with conn.MasterConnection(ADDR, SRC_SYS_NO, SRC_COM_NO) as master:
  conn.wait_until_be_(master, MODE_GUIDED)
  conn.arm(master)
  conn.take_off(master, alt = TARGET_ALT)
  conn.change_messaging_rate(master, GLOBAL_POSITION_INT, rate=10e5, mask = [0, 0, 0, 0, 0])
  conn.wait_until_be_reachd(master, alt=TARGET_ALT)
  conn.go_to_(master, lat=TARGET_LAT, lon=TARGET_LON, alt=TARGET_ALT)
  conn.wait_until_be_reachd(master, lat=TARGET_LAT, lon=TARGET_LON, alt=TARGET_ALT)
