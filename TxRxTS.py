#The purpose of this is to troubleshoot servo motor and figure out the error

from dynamixel_sdk import *  # Uses Dynamixel SDK

# === CONFIG ===
DEVICENAME = '/dev/ttyUSB0'  # Change if needed
BAUDRATE = 57600             # Change if your servos use another baudrate
PROTOCOL_VERSION = 1.0       # 1.0 or 2.0 depending on your servo
MOTOR_IDS = [0, 3, 6, 9]    # Your servo IDs

ADDR_MX_GOAL_POSITION = 30   # Goal Position address
ADDR_MX_MOVING_SPEED = 32    # Moving Speed address

# === Initialize Port and Packet Handler ===
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

if not portHandler.openPort():
    raise Exception("❌ Failed to open port")

if not portHandler.setBaudRate(BAUDRATE):
    raise Exception("❌ Failed to set baudrate")

print("✅ Port opened and baudrate set")

# === Ping each motor ===
print("\n=== PING TEST ===")
for motor_id in MOTOR_IDS:
    dxl_comm, dxl_err = packetHandler.ping(portHandler, motor_id)
    if dxl_comm != COMM_SUCCESS:
        print(f"❌ Motor {motor_id} ping failed: {packetHandler.getTxRxResult(dxl_comm)}")
    else:
        print(f"✅ Motor {motor_id} responded")

# === Set speed and move motor 0 to mid-position ===
mid_position = 512  # For MX series, 0–1023
speed = 200         # 0–1023

print("\n=== SPEED AND MOVE TEST ===")
dxl_comm, dxl_err = packetHandler.write2ByteTxRx(portHandler, MOTOR_IDS[0], ADDR_MX_MOVING_SPEED, speed)
if dxl_comm != COMM_SUCCESS:
    print(f"❌ Motor {MOTOR_IDS[0]} speed set failed: {packetHandler.getTxRxResult(dxl_comm)}")
else:
    print(f"✅ Motor {MOTOR_IDS[0]} speed set to {speed}")

dxl_comm, dxl_err = packetHandler.write2ByteTxRx(portHandler, MOTOR_IDS[0], ADDR_MX_GOAL_POSITION, mid_position)
if dxl_comm != COMM_SUCCESS:
    print(f"❌ Motor {MOTOR_IDS[0]} move failed: {packetHandler.getTxRxResult(dxl_comm)}")
else:
    print(f"✅ Motor {MOTOR_IDS[0]} moved to {mid_position}")

# === Close port ===
portHandler.closePort()
print("\n✅ Test finished")
