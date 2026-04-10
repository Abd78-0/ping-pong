from dynamixel_sdk import *  # Dynamixel SDK
 
# === Constants ===
ADDR_MX_GOAL_POSITION = 30
PROTOCOL_VERSION = 1.0
DEVICENAME = '/dev/ttyUSB0'  # Change if needed
BAUDRATE = 57600
MOTOR_IDS = [0, 3, 6, 9]
 
# === Initialize Port and Packet Handlers ===
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)
groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_MX_GOAL_POSITION, 2)  # 2 bytes
 
# === Open Port and Set Baudrate ===
if not portHandler.openPort():
    raise Exception("❌ Failed to open port")
 
if not portHandler.setBaudRate(BAUDRATE):
    raise Exception("❌ Failed to set baudrate")
 
print("✅ Port opened and baudrate set")
print("Controlling motors:", MOTOR_IDS)

ADDR_MX_MOVING_SPEED = 32           # Control Table Address
desired_speed = 100                 # range 0~2047，0=maximum speed，1~1023=CCW direction，1024~2047=CW direction

# Write in the speed
dxl_comm, dxl_err = packetHandler.write2ByteTxRx(
    portHandler, 0, ADDR_MX_MOVING_SPEED, desired_speed
)

dxl_comm, dxl_err = packetHandler.write2ByteTxRx(
    portHandler, 3, ADDR_MX_MOVING_SPEED, desired_speed
)

dxl_comm, dxl_err = packetHandler.write2ByteTxRx(
    portHandler, 6, ADDR_MX_MOVING_SPEED, desired_speed
)

dxl_comm, dxl_err = packetHandler.write2ByteTxRx(
    portHandler, 9, ADDR_MX_MOVING_SPEED, desired_speed
)
# if dxl_comm != COMM_SUCCESS:
#     print("❌ 设置舵机3速度失败：", packetHandler.getTxRxResult(dxl_comm))
# elif dxl_err != 0:
#     print("❌ 舵机3速度写入错误：", packetHandler.getRxPacketError(dxl_err))
# else:
#     print(f"✅ 舵机3速度已设置为 {desired_speed}")
 
# === Angle Conversion Function (per motor model) ===




# === Compliance Control Register（Protocol 1.0）===
ADDR_CW_MARGIN   = 26  # CW Compliance Margin (1 byte)
ADDR_CCW_MARGIN  = 27  # CCW Compliance Margin (1 byte)
ADDR_CW_SLOPE    = 28  # CW Compliance Slope  (1 byte)
ADDR_CCW_SLOPE   = 29  # CCW Compliance Slope  (1 byte)

def set_compliance(motor_ids, cw_margin, ccw_margin, cw_slope, ccw_slope):
    """
    Servo Motor Compliance Margin/Slope

    motor_ids   : single ID or [ID1, ID2, ...] 
    cw_margin   : CW Margin value (0~254)
    ccw_margin  : CCW Margin value (0~254)
    cw_slope    : CW Slope value (2,4,8,16,32,64,128)
    ccw_slope   : CCW Slope  (2,4,8,16,32,64,128)
    """
    if isinstance(motor_ids, int):
        motor_ids = [motor_ids]
    for mid in motor_ids:
        # Write Margin
        packetHandler.write1ByteTxRx(portHandler, mid, ADDR_CW_MARGIN,  cw_margin)
        packetHandler.write1ByteTxRx(portHandler, mid, ADDR_CCW_MARGIN, ccw_margin)
        # Write Slope
        packetHandler.write1ByteTxRx(portHandler, mid, ADDR_CW_SLOPE,   cw_slope)
        packetHandler.write1ByteTxRx(portHandler, mid, ADDR_CCW_SLOPE,  ccw_slope)
    print(f"✅ ID {motor_ids} Compliance Updated "
          f"[CW_M={cw_margin}, CCW_M={ccw_margin}, CW_S={cw_slope}, CCW_S={ccw_slope}]")





set_compliance(3, cw_margin=1, ccw_margin=1, cw_slope=32, ccw_slope=32)



def angle_to_dxl(motor_id, angle_deg):
    # Clamp 到合法物理范围
    if motor_id == 3:   # MX-106：360°
        angle_deg = max(0, min(360, angle_deg))
        # 4095 steps → 360°
        return int(angle_deg * 4095.0 / 360.0)
    else:               # EX-106+：250°
        angle_deg = max(0, min(250, angle_deg))
        # 4095 steps → 250°
        return int(angle_deg * 4095.0 / 250.0)



def motorMove(value_input):
    angle_vals = list(map(float, value_input.strip().split()))
    if len(angle_vals) != 4:
        print("⚠️ Please enter 4 angles.")
        
 
            # Clear previous params
    groupSyncWrite.clearParam()


    
 
    # for motor_id, angle in zip(MOTOR_IDS, angle_vals):
    #     dxl_position = angle_to_dxl(motor_id, angle)
    #     param_goal_position = [DXL_LOBYTE(dxl_position), DXL_HIBYTE(dxl_position)]
    #     success = groupSyncWrite.addParam(motor_id, param_goal_position)
    #     if not success:
    #         print(f"❌ Failed to add param for motor {motor_id}")
 
    #         # Send sync write packet
    # dxl_comm_result = groupSyncWrite.txPacket()

    




# === Main Input Loop ===
try:
    while True:
        print("\nEnter target angles for all motors (ID 0, 3, 6, 9).")
        print("Format: <angle0> <angle3> <angle6> <angle9> or type 'exit'")
 
        user_input = input("Angles: ")

 
        if user_input.lower() == "exit":
            break
 
        try:
            angle_vals = list(map(float, user_input.strip().split()))
            if len(angle_vals) != 4:
                print("⚠️ Please enter 4 angles.")
                continue
 
            # Clear previous params
            groupSyncWrite.clearParam()
 
            for motor_id, angle in zip(MOTOR_IDS, angle_vals):
                dxl_position = angle_to_dxl(motor_id, angle)
                param_goal_position = [DXL_LOBYTE(dxl_position), DXL_HIBYTE(dxl_position)]
                success = groupSyncWrite.addParam(motor_id, param_goal_position)
                if not success:
                    print(f"❌ Failed to add param for motor {motor_id}")
 
            # Send sync write packet
            dxl_comm_result = groupSyncWrite.txPacket()

            motorMove(user_input)

            

            if dxl_comm_result != COMM_SUCCESS:
                print(f"❌ SyncWrite Error: {packetHandler.getTxRxResult(dxl_comm_result)}")
            else:
                print("✅ All motors updated.")
 
        except ValueError:
            print("⚠️ Invalid input. Use: <angle0> <angle3> <angle6> <angle9>")
 
finally:
    portHandler.closePort()
    print("🔌 Port closed.")