import serial
import struct
import time

# ============ 配置区 ============
PORT = '/dev/ttyACM0'
BAUDRATE = 921600
SLAVE_ADDR = 1
TIMEOUT = 0.5
# ================================

def crc16_modbus(data: bytes) -> bytes:
    """计算 Modbus CRC16"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return struct.pack('<H', crc)

def send_read_command(ser, func_code: int, reg_count: int) -> bytes:
    """
    构造并发送 04H 读取命令
    func_code: 寄存器起始地址低字节（如 0x20）
    reg_count: 寄存器数量低字节
    """
    ser.reset_input_buffer()          # 发送前清空接收缓冲，避免残留字节
    frame = bytearray()
    frame.append(SLAVE_ADDR)      # 从机地址
    frame.append(0x04)            # 功能码 04H
    frame.append(0x00)            # 寄存器起始地址高字节
    frame.append(func_code)       # 寄存器起始地址低字节
    frame.append(0x00)            # 寄存器数量高字节
    frame.append(reg_count)       # 寄存器数量低字节
    frame.extend(crc16_modbus(frame))
    ser.write(frame)
    ser.flush()
    return bytes(frame)

def read_response(ser, timeout=0.5) -> bytes:
    """按 Modbus RTU 帧结构读取完整响应"""
    ser.timeout = timeout
    header = ser.read(3)          # 地址 + 功能码 + 字节数
    if len(header) < 3:
        return header
    byte_count = header[2]
    rest = ser.read(byte_count + 2)   # 数据 + CRC
    return header + rest

def parse_version(byte_val: int) -> str:
    """协议定义：十进制数的十位=主版本，个位=次版本"""
    major = byte_val // 10
    minor = byte_val % 10
    return f"V{major}.{minor}"

def parse_int16_be(data: bytes) -> int:
    return struct.unpack('>h', data)[0]

def parse_int32_be(data: bytes) -> int:
    return struct.unpack('>i', data)[0]

def parse_uint32_be(data: bytes) -> int:
    return struct.unpack('>I', data)[0]

def parse_float_be(data: bytes) -> float:
    return struct.unpack('>f', data)[0]

def main():
    ser = serial.Serial(
        port=PORT,
        baudrate=BAUDRATE,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=TIMEOUT
    )
    print(f"已打开 {PORT} @ {BAUDRATE}")
    time.sleep(0.1)

    # ---------- 测试1：读取软硬件版本 (0x20, 2字节) ----------
    print("\n[测试1] 读取软硬件版本 (0x20)")
    tx = send_read_command(ser, 0x20, 0x01)
    print(f"发送: {tx.hex(' ').upper()}")
    rx = read_response(ser)
    print(f"接收: {rx.hex(' ').upper()}")
    if len(rx) >= 7 and rx[1] == 0x04:
        print(f"  → 固件版本: {parse_version(rx[3])}")
        print(f"  → 硬件版本: {parse_version(rx[4])}")
    else:
        print("  → 无有效响应，请检查接线/地址/波特率")

    # ---------- 测试2：读取电机运行状态 (0x2C, 1字节) ----------
    print("\n[测试2] 读取电机运行状态 (0x2C)")
    tx = send_read_command(ser, 0x2C, 0x01)
    print(f"发送: {tx.hex(' ').upper()}")
    rx = read_response(ser)
    print(f"接收: {rx.hex(' ').upper()}")
    if len(rx) >= 6 and rx[1] == 0x04:
        status_map = {0: "停止", 1: "任务完成", 2: "正在运行",
                      3: "过载", 4: "堵转", 5: "欠压"}
        print(f"  → 运行状态: {status_map.get(rx[3], '未知')}")
    else:
        print("  → 无有效响应")

    # ---------- 测试3：读取实时位置 (0x2A, 4字节) ----------
    print("\n[测试3] 读取电机实时位置 (0x2A)")
    tx = send_read_command(ser, 0x2A, 0x02)
    print(f"发送: {tx.hex(' ').upper()}")
    rx = read_response(ser)
    print(f"接收: {rx.hex(' ').upper()}")
    if len(rx) >= 9 and rx[1] == 0x04:
        pos = parse_int32_be(rx[3:7])
        print(f"  → 实时位置: {pos} (约 {pos/51200:.4f} 圈)")
    else:
        print("  → 无有效响应")

    # ---------- 测试4：读取实时转速 (0x29, 2字节) ----------
    print("\n[测试4] 读取电机实时转速 (0x29)")
    tx = send_read_command(ser, 0x29, 0x01)
    print(f"发送: {tx.hex(' ').upper()}")
    rx = read_response(ser)
    print(f"接收: {rx.hex(' ').upper()}")
    if len(rx) >= 7 and rx[1] == 0x04:
        rpm = parse_int16_be(rx[3:5])
        print(f"  → 实时转速: {rpm} RPM")
    else:
        print("  → 无有效响应")

    # ---------- 测试5：读取总线电压 (0x24, 4字节 float) ----------
    print("\n[测试5] 读取总线电压 (0x24)")
    tx = send_read_command(ser, 0x24, 0x02)
    print(f"发送: {tx.hex(' ').upper()}")
    rx = read_response(ser)
    print(f"接收: {rx.hex(' ').upper()}")
    if len(rx) >= 9 and rx[1] == 0x04:
        voltage = parse_float_be(rx[3:7])
        print(f"  → 总线电压: {voltage:.2f} V")
    else:
        print("  → 无有效响应")

    # ---------- 测试6：读取相电流 (0x23, 2字节 int16) ----------
    print("\n[测试6] 读取相电流 (0x23)")
    tx = send_read_command(ser, 0x23, 0x01)
    print(f"发送: {tx.hex(' ').upper()}")
    rx = read_response(ser)
    print(f"接收: {rx.hex(' ').upper()}")
    if len(rx) >= 7 and rx[1] == 0x04:
        current = parse_int16_be(rx[3:5])
        print(f"  → 相电流: {current} mA")
    else:
        print("  → 无有效响应")

    # ---------- 测试7：读取使能状态 (0x2F, 1字节) ----------
    print("\n[测试7] 读取使能状态 (0x2F)")
    tx = send_read_command(ser, 0x2F, 0x01)
    print(f"发送: {tx.hex(' ').upper()}")
    rx = read_response(ser)
    print(f"接收: {rx.hex(' ').upper()}")
    if len(rx) >= 6 and rx[1] == 0x04:
        enable_map = {0: "使能", 1: "失能"}
        print(f"  → 使能状态: {enable_map.get(rx[3], '未知')}")
    else:
        print("  → 无有效响应")

    # ---------- 测试8：读取到位标志 (0x30, 1字节) ----------
    print("\n[测试8] 读取到位标志 (0x30)")
    tx = send_read_command(ser, 0x30, 0x01)
    print(f"发送: {tx.hex(' ').upper()}")
    rx = read_response(ser)
    print(f"接收: {rx.hex(' ').upper()}")
    if len(rx) >= 6 and rx[1] == 0x04:
        inpos_map = {0: "未到位", 1: "到位"}
        print(f"  → 到位标志: {inpos_map.get(rx[3], '未知')}")
    else:
        print("  → 无有效响应")

    ser.close()
    print("\n完成。")

if __name__ == '__main__':
    main()