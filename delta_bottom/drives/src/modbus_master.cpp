#include "modbus_master.hpp"
#include "modbus_crc.hpp"

namespace {

/**
 * @brief 在帧末尾追加 CRC-16/MODBUS 校验码
 * @param f 待追加 CRC 的帧数据（就地修改）
 * @note Modbus RTU 规定 CRC 两字节按低字节在前传输，与帧内其他多字节字段（大端）相反
 */
void appendCrc(std::vector<uint8_t>& f)
{
    uint16_t crc = crc16Modbus(f.data(), f.size());
    f.push_back((uint8_t)(crc & 0xFF));
    f.push_back((uint8_t)(crc >> 8));
}

} // namespace

/**
 * @brief 构造 ModbusMaster，保存串口引用
 * @param port 已打开的串口对象引用
 * @note 本函数不负责打开/关闭串口，串口生命周期由外部管理
 */
ModbusMaster::ModbusMaster(SerialPort& port) : port_(port) {}

/**
 * @brief 执行一次完整事务：发送请求帧、等待应答并校验
 * @param req 待发送的请求帧（含 CRC）
 * @param addr 期望的从站地址
 * @param func 期望的功能码
 * @param rsp 出参：成功时写入「func + 数据区」（去掉 addr 与 CRC）
 * @return 0=成功；-1=超时/长度不足；-2=地址或功能码不匹配；-3=CRC 错；>0=从站异常码
 * @note 校验顺序：帧长 → 从站地址 → 异常应答 → 功能码 → CRC，任何一步失败立即返回
 */
int ModbusMaster::transact_(const std::vector<uint8_t>& req, uint8_t addr, uint8_t func,
                            std::vector<uint8_t>* rsp)
{
    port_.flushInput();                 // 发前清残留（RS485 半双工必须）
    port_.write(req);                   // 内部 tcdrain，保证发完再等应答

    uint8_t buf[256];
    uint64_t ts = 0;
    ssize_t n = port_.read(buf, sizeof(buf), resp_timeout_ms, &ts, byte_timeout_ms);
    if (n < 4) return -1;               // 至少 addr + func + crc(2)

    if (buf[0] != addr) return -2;      // 从站地址不符

    if (buf[1] == (uint8_t)(func | 0x80))  // 异常应答：func|0x80 + 异常码
        return buf[2];

    if (buf[1] != func) return -2;      // 功能码不符

    uint16_t crc_rx = (uint16_t)(buf[n - 2] | (buf[n - 1] << 8));
    if (crc16Modbus(buf, (size_t)(n - 2)) != crc_rx) return -3;

    if (rsp) rsp->assign(buf + 1, buf + (n - 2));  // 去 addr 与 CRC，留 func+数据区
    return 0;
}

/**
 * @brief 读寄存器公共实现
 * @param addr 从站地址
 * @param func 功能码（0x03 保持寄存器 / 0x04 输入寄存器）
 * @param start 起始寄存器地址
 * @param count 寄存器数量
 * @param out 出参：读取结果，按大端写入 count 个 16 位寄存器
 * @return 成功返回 true，超时/校验错/异常应答返回 false
 */
bool ModbusMaster::readRegisters_(uint8_t addr, uint8_t func, uint16_t start,
                                  uint16_t count, uint16_t* out)
{
    std::vector<uint8_t> req = {
        addr, func,
        (uint8_t)(start >> 8), (uint8_t)(start & 0xFF),
        (uint8_t)(count >> 8), (uint8_t)(count & 0xFF),
    };
    appendCrc(req);

    std::vector<uint8_t> rsp;
    if (transact_(req, addr, func, &rsp) != 0) return false;

    // 读应答：func + 字节数N + data[N]
    if (rsp.size() < 2) return false;
    uint8_t n = rsp[1];
    if (n != count * 2 || rsp.size() < (size_t)(2 + n)) return false;

    for (uint16_t i = 0; i < count; ++i)
        out[i] = (uint16_t)((rsp[2 + 2 * i] << 8) | rsp[3 + 2 * i]);  // 大端
    return true;
}

/**
 * @brief 读保持寄存器（功能码 0x03）
 * @param addr 从站地址
 * @param start 起始寄存器地址
 * @param count 寄存器数量
 * @param out 出参：读取结果，按大端写入 count 个 16 位寄存器
 * @return 成功返回 true，否则 false
 */
bool ModbusMaster::readHoldingRegisters(uint8_t addr, uint16_t start, uint16_t count, uint16_t* out)
{
    return readRegisters_(addr, 0x03, start, count, out);
}

/**
 * @brief 读输入寄存器（功能码 0x04）
 * @param addr 从站地址
 * @param start 起始寄存器地址
 * @param count 寄存器数量
 * @param out 出参：读取结果，按大端写入 count 个 16 位寄存器
 * @return 成功返回 true，否则 false
 */
bool ModbusMaster::readInputRegisters(uint8_t addr, uint16_t start, uint16_t count, uint16_t* out)
{
    return readRegisters_(addr, 0x04, start, count, out);
}

/**
 * @brief 写单寄存器（功能码 0x06）
 * @param addr 从站地址
 * @param reg 寄存器地址
 * @param val 要写入的 16 位值
 * @return 成功返回 true，否则 false
 * @note 帧结构：addr + func + 寄存器地址(2) + 值(2) + crc
 */
bool ModbusMaster::writeRegister(uint8_t addr, uint16_t reg, uint16_t val)
{
    std::vector<uint8_t> req = {
        addr, 0x06,
        (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF),
        (uint8_t)(val >> 8), (uint8_t)(val & 0xFF),
    };
    appendCrc(req);
    return transact_(req, addr, 0x06, nullptr) == 0;
}

/**
 * @brief 写多寄存器（功能码 0x10）
 * @param addr 从站地址
 * @param start 起始寄存器地址
 * @param count 寄存器数量
 * @param vals 要写入的寄存器值数组（大端）
 * @return 成功返回 true，否则 false
 * @note 帧结构：addr + func + 起始地址(2) + 数量(2) + 字节数 + 数据 + crc
 */
bool ModbusMaster::writeRegisters(uint8_t addr, uint16_t start, uint16_t count, const uint16_t* vals)
{
    std::vector<uint8_t> req = {
        addr, 0x10,
        (uint8_t)(start >> 8), (uint8_t)(start & 0xFF),
        (uint8_t)(count >> 8), (uint8_t)(count & 0xFF),
        (uint8_t)(count * 2),           // 数据字节数
    };
    for (uint16_t i = 0; i < count; ++i) {
        req.push_back((uint8_t)(vals[i] >> 8));
        req.push_back((uint8_t)(vals[i] & 0xFF));
    }
    appendCrc(req);
    return transact_(req, addr, 0x10, nullptr) == 0;
}

/**
 * @brief 写 32 位值（占 2 个寄存器，高 16 位在前）
 * @param addr 从站地址
 * @param reg 起始寄存器地址
 * @param val 要写入的 32 位值
 * @return 成功返回 true，否则 false
 * @note 位置等需要 32 位数据的便捷封装，内部走 0x10 写多寄存器
 */
bool ModbusMaster::writeRegister32(uint8_t addr, uint16_t reg, uint32_t val)
{
    uint16_t v[2] = {(uint16_t)(val >> 16), (uint16_t)(val & 0xFFFF)};
    return writeRegisters(addr, reg, 2, v);
}
