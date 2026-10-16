#include "modbus_master.hpp"
#include "modbus_crc.hpp"

#include <algorithm>
#include <chrono>

namespace {

// —— 重同步（resync）预算：帧头对不上时不立即失败，在限额内继续找帧头 ——
constexpr int kResyncMaxBytes  = 24;   // 最多丢弃多少字节去找帧头
constexpr int kResyncBudgetMs  = 8;    // 找帧头的总时间上限（毫秒）
constexpr int kResyncByteMs    = 5;    // 找帧头期间等待下一字节的时间（毫秒）

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
 * @brief 返回码 → 可读描述
 * @param rc transact_ 的返回码（见 ModbusRc）
 * @return 静态字符串；从站异常码附带常见含义
 */
const char* modbusRcName(int rc)
{
    switch (rc) {
        case MB_RC_OK:      return "成功";
        case MB_RC_TIMEOUT: return "超时/应答长度不足（-1）";
        case MB_RC_BADADDR: return "从站地址或功能码不匹配（-2）";
        case MB_RC_BADCRC:  return "CRC 校验错（-3）";
        default: break;
    }
    if (rc > 0) {
        switch (rc) {
            case 0x01: return "从站异常码 0x01 非法功能";
            case 0x02: return "从站异常码 0x02 非法数据地址";
            case 0x03: return "从站异常码 0x03 非法数据值";
            case 0x04: return "从站异常码 0x04 从站设备故障";
            case 0x05: return "从站异常码 0x05 确认";
            case 0x06: return "从站异常码 0x06 从站忙";
            case 0x08: return "从站异常码 0x08 存储奇偶校验错";
            case 0x0A: return "从站异常码 0x0A 网关路径不可用";
            case 0x0B: return "从站异常码 0x0B 网关目标无响应";
            default:   return "从站异常码（未知）";
        }
    }
    return "未知返回码";
}

/**
 * @brief 构造 ModbusMaster，保存串口引用
 * @param port 已打开的串口对象引用
 * @note 本函数不负责打开/关闭串口，串口生命周期由外部管理
 */
ModbusMaster::ModbusMaster(SerialPort& port) : port_(port) {}

/**
 * @brief 执行一次完整事务：发送请求帧、按期望长度接收应答并校验
 * @param req 待发送的请求帧（含 CRC）
 * @param addr 期望的从站地址
 * @param func 期望的功能码
 * @param exp_len 期望的完整应答帧长度（含 addr 与 CRC），按此长度精确收帧
 * @param rsp 出参：成功时写入「func + 数据区」（去掉 addr 与 CRC）
 * @return 0=成功；-1=超时/长度不足；-2=地址或功能码不匹配；-3=CRC 错；>0=从站异常码
 * @note 校验顺序：帧长 → 从站地址 → 异常应答 → 功能码 → CRC，任何一步失败立即返回。
 *       发帧前先等总线静默；失败收尾时把残留字节丢弃到静默，避免迟到/多余的应答
 *       被下一次事务误认为自己的应答（这是"背靠背连发时越错越多"的根源）。
 *       若帧头对不上（地址不是本从站、或功能码是"上一次事务那种"的迟到帧），
 *       不再立即失败，而是在有限字节/时间内继续扫描帧头（重同步），
 *       跳过迟到帧、别站帧与被干扰的字节后再校验 CRC。
 */
int ModbusMaster::transact_(const std::vector<uint8_t>& req, uint8_t addr, uint8_t func,
                            uint16_t exp_len, std::vector<uint8_t>* rsp)
{
    // 上一次事务异常收尾时，总线上可能还有没读完的残帧：先等它静默再发，
    // 否则会被本次事务当成"自己的应答"（地址/CRC 不匹配并连锁污染后续帧）。
    // 正常路径不做这个等待（不占周期预算）。
    if (need_drain_) {
        port_.waitIdle(idle_gap_ms, std::max(20, idle_gap_ms * 4));
        need_drain_ = false;
    }
    port_.flushInput();

    port_.write(req);                   // 内部 tcdrain，保证发完再等应答

    uint8_t buf[256];
    if (exp_len < 5 || exp_len > sizeof(buf)) return MB_RC_TIMEOUT;

    // 异常收尾：把总线上可能残留的字节丢干净再返回，避免污染下一帧
    auto bail = [&](int rc) {
        port_.waitIdle(idle_gap_ms, std::max(20, idle_gap_ms * 4));
        need_drain_ = true;             // 下次发帧前再确认一次静默
        return rc;
    };

    // 重同步预算：最多丢 kResyncMaxBytes 个字节 / kResyncBudgetMs 毫秒，避免拖垮周期
    const auto t0 = std::chrono::steady_clock::now();
    auto over_budget = [&](int dropped) {
        if (dropped > kResyncMaxBytes) return true;
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                      std::chrono::steady_clock::now() - t0).count();
        return ms > kResyncBudgetMs;
    };

    int dropped = 0;                    // 为找帧头而丢弃的字节数
    int last_rc = MB_RC_BADADDR;        // 扫描没找到合格帧时对外报告的原因
    int wait_ms = resp_timeout_ms;      // 首字节用完整超时，之后用短超时扫描

    while (true) {
        // 1) 找本从站地址字节（对不上就丢掉继续找）
        if (port_.readExact(buf, 1, wait_ms, byte_timeout_ms) < 1) return bail(MB_RC_TIMEOUT);
        wait_ms = kResyncByteMs;
        if (buf[0] != addr) {
            last_rc = MB_RC_BADADDR;
            if (over_budget(++dropped)) break;
            continue;
        }

        // 2) 读功能码
        if (port_.readExact(buf + 1, 1, kResyncByteMs, byte_timeout_ms) < 1) return bail(MB_RC_TIMEOUT);

        // 2a) 异常应答：addr + func|0x80 + 异常码 + CRC(2) = 5 字节
        if (buf[1] == (uint8_t)(func | 0x80)) {
            if (port_.readExact(buf + 2, 3, byte_timeout_ms, byte_timeout_ms) < 3) return bail(MB_RC_TIMEOUT);
            uint16_t crc_rx = (uint16_t)(buf[3] | (buf[4] << 8));
            if (crc16Modbus(buf, 3) != crc_rx) {
                last_rc = MB_RC_BADCRC;
                if (over_budget(dropped += 2)) break;
                continue;
            }
            return bail(buf[2]);        // 从站异常码（>0）
        }

        // 2b) 功能码不是本次期望的（多半是上一帧的迟到应答，如读事务收到 0x10 回显）→ 跳过
        if (buf[1] != func) {
            last_rc = MB_RC_BADADDR;
            if (over_budget(dropped += 2)) break;
            continue;
        }

        // 3) 数据帧：收满期望长度
        if (port_.readExact(buf + 2, exp_len - 2, byte_timeout_ms, byte_timeout_ms) < (ssize_t)(exp_len - 2))
            return bail(MB_RC_TIMEOUT);

        // 4) CRC 校验
        uint16_t crc_rx = (uint16_t)(buf[exp_len - 2] | (buf[exp_len - 1] << 8));
        if (crc16Modbus(buf, exp_len - 2) != crc_rx) {
            last_rc = MB_RC_BADCRC;
            if (over_budget(dropped += 2)) break;
            continue;
        }

        if (rsp) rsp->assign(buf + 1, buf + (exp_len - 2));  // 去 addr 与 CRC，留 func+数据区
        return MB_RC_OK;
    }

    return bail(last_rc);               // 重同步超预算，按最后一次原因上报
}

/**
 * @brief 读寄存器公共实现
 * @param addr 从站地址
 * @param func 功能码（0x03 保持寄存器 / 0x04 输入寄存器）
 * @param start 起始寄存器地址
 * @param count 寄存器数量
 * @param out 出参：读取结果，按大端写入 count 个 16 位寄存器
 * @return 0=成功；其余见 ModbusRc
 */
int ModbusMaster::readRegisters_(uint8_t addr, uint8_t func, uint16_t start,
                                 uint16_t count, uint16_t* out)
{
    std::vector<uint8_t> req = {
        addr, func,
        (uint8_t)(start >> 8), (uint8_t)(start & 0xFF),
        (uint8_t)(count >> 8), (uint8_t)(count & 0xFF),
    };
    appendCrc(req);

    std::vector<uint8_t> rsp;
    // 读应答：addr + func + 字节数 + data(count*2) + crc = 5 + count*2
    int rc = transact_(req, addr, func, (uint16_t)(5 + count * 2), &rsp);
    if (rc != MB_RC_OK) return rc;

    // 读应答数据区：func + 字节数N + data[N]
    if (rsp.size() < 2) return MB_RC_TIMEOUT;
    uint8_t n = rsp[1];
    if (n != count * 2 || rsp.size() < (size_t)(2 + n)) return MB_RC_TIMEOUT;

    for (uint16_t i = 0; i < count; ++i)
        out[i] = (uint16_t)((rsp[2 + 2 * i] << 8) | rsp[3 + 2 * i]);  // 大端
    return MB_RC_OK;
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
    return readRegisters_(addr, 0x03, start, count, out) == MB_RC_OK;
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
    return readRegisters_(addr, 0x04, start, count, out) == MB_RC_OK;
}

/**
 * @brief 读输入寄存器（带返回码版本）
 * @return 0=成功，其余见 ModbusRc（-1 超时 / -2 地址功能码错 / -3 CRC 错 / >0 从站异常码）
 */
int ModbusMaster::readInputRegistersRc(uint8_t addr, uint16_t start, uint16_t count, uint16_t* out)
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
    return writeRegisterRc(addr, reg, val) == MB_RC_OK;
}

/**
 * @brief 写单寄存器（功能码 0x06，带返回码版本）
 * @return 0=成功，其余见 ModbusRc
 */
int ModbusMaster::writeRegisterRc(uint8_t addr, uint16_t reg, uint16_t val)
{
    std::vector<uint8_t> req = {
        addr, 0x06,
        (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF),
        (uint8_t)(val >> 8), (uint8_t)(val & 0xFF),
    };
    appendCrc(req);
    // 写应答：addr + func + 寄存器地址(2) + 值(2) + crc = 8 字节
    return transact_(req, addr, 0x06, 8, nullptr);
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
    return writeRegistersRc(addr, start, count, vals) == MB_RC_OK;
}

/**
 * @brief 写多寄存器（功能码 0x10，带返回码版本）
 * @return 0=成功，其余见 ModbusRc
 */
int ModbusMaster::writeRegistersRc(uint8_t addr, uint16_t start, uint16_t count, const uint16_t* vals)
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
    // 写应答：addr + func + 起始地址(2) + 数量(2) + crc = 8 字节
    return transact_(req, addr, 0x10, 8, nullptr);
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
