#include "serial_port.hpp"
#include <chrono>
#include <sys/ioctl.h>

// —— 任意波特率支持（256000 等非标值）——
// glibc 的 termios 接口只有标准 Bxxx 常量；内核的 termios2 + BOTHER 支持任意波特率。
// 这里手动声明 struct termios2（布局与内核 <asm-generic/termbits.h> 一致，44 字节），
// 避免包含 <asm/termbits.h> 与 <termios.h> 的 struct termios 重定义冲突。
#ifndef BOTHER
#define BOTHER 0x1000   // CBAUDEX：非标波特率标志
#endif
#ifndef CBAUD
#define CBAUD 0x100F    // 标准波特率掩码
#endif

struct termios2 {
    tcflag_t c_iflag;    // 输入模式标志
    tcflag_t c_oflag;    // 输出模式标志
    tcflag_t c_cflag;    // 控制模式标志
    tcflag_t c_lflag;    // 本地模式标志
    cc_t     c_line;     // 行规程
    cc_t     c_cc[19];   // 控制字符（NCCS）
    speed_t  c_ispeed;   // 输入波特率
    speed_t  c_ospeed;   // 输出波特率
};
#define TCGETS2 _IOR('T', 0x2A, struct termios2)
#define TCSETS2 _IOW('T', 0x2B, struct termios2)

namespace {
// 用 termios2 + BOTHER 设置任意波特率（Linux 无 B256000 常量，故走此路径）
bool setArbitraryBaud(int fd, int baudrate)
{
    struct termios2 tio;
    if (ioctl(fd, TCGETS2, &tio) != 0) {
        std::cerr << "TCGETS2 失败: " << strerror(errno) << "\n";
        return false;
    }
    tio.c_cflag &= ~CBAUD;
    tio.c_cflag |= BOTHER;
    tio.c_ispeed = baudrate;
    tio.c_ospeed = baudrate;
    if (ioctl(fd, TCSETS2, &tio) != 0) {
        std::cerr << "TCSETS2 失败: " << strerror(errno) << "\n";
        return false;
    }
    return true;
}
}  // namespace

SerialPort::~SerialPort() { close(); }

//串口类构造函数
SerialPort::SerialPort(int argc, char** argv)
{
    //读取yaml
    std::string path = (argc > 1) ? argv[1] : "configs/hardware_config.yaml";
    try 
    {
        YAML::Node root = YAML::LoadFile(path);
        auto serial = root["serial"];
        param.port_name = serial["port"].as<std::string>();
        param.baud_rate = serial["baudrate"].as<int>();
        std::cerr << "设置yaml文件参数 -- " << param.port_name << ", " << param.baud_rate << std::endl;
        yaml_err = 0; // 成功读取
    } 
    catch (const YAML::BadFile& e)
    {
        std::cerr << "yaml文件打开错误[" << path << "]: " << e.what() << "\n";
        yaml_err = 1; // 读取失败
    }
    catch (const YAML::ParserException& e)
    {
        std::cerr << "YAML 语法错误: " << e.what() << "\n";
        yaml_err = 2; // 语法错误
    }
    catch (const YAML::BadConversion& e)
    {
        std::cerr << "类型转换错误: " << e.what() << "\n";
        yaml_err = 3; // 类型转换错误
    }
    catch (const YAML::Exception& e)
    {
        std::cerr << "YAML 错误: " << e.what() << "\n";
        yaml_err = 4; // YAML 错误
    }
    catch (const std::exception& e)
    {
        std::cerr << "其他错误: " << e.what() << "\n";
        yaml_err = 5; // 其他错误
    }
    if(yaml_err != 0)
    {
        std::cerr << "yaml文件读取失败,将使用默认参数" << std::endl;
        param.port_name = "/dev/ttyUSB0";
        param.baud_rate = 921600;
        std::cerr << "设置默认参数 -- " << param.port_name << ", " << param.baud_rate << std::endl;
    }
    if(open(param.port_name, param.baud_rate))//开启串口
    {
        serial_opened_ = true;
    }
    else
    {
        serial_opened_ = false; 
    }
}

int SerialPort::baudrateToSpeed(int baudrate) 
{
    switch (baudrate) {
        case 9600:    return B9600;
        case 19200:   return B19200;
        case 38400:   return B38400;
        case 57600:   return B57600;
        case 115200:  return B115200;
        case 230400:  return B230400;
        case 460800:  return B460800;
        case 500000:  return B500000;
        case 576000:  return B576000;
        case 921600:  return B921600;
        case 1000000: return B1000000;
        case 1500000: return B1500000;
        case 2000000: return B2000000;
        default:
            // 非标准波特率（如 256000），返回 -1，由 setInterfaceAttrs 走 termios2 路径
            return -1;
    }
}


bool SerialPort::setInterfaceAttrs(int baudrate)
{
    struct termios tty;
    if (tcgetattr(fd_, &tty) != 0) {
        std::cerr << "tcgetattr 失败: " << strerror(errno) << "\n";
        return false;
    }

    int speed = baudrateToSpeed(baudrate);
    bool arbitrary = (speed < 0);   // 非标准波特率，稍后用 termios2 设置
    if (arbitrary) speed = B9600;   // 先占位，保证 tcsetattr 成功

    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    // 8N1
    tty.c_cflag &= ~PARENB;         // 无校验
    tty.c_cflag &= ~CSTOPB;         // 1 停止位
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;             // 8 数据位
    tty.c_cflag &= ~CRTSCTS;        // 无硬件流控
    tty.c_cflag |= CREAD | CLOCAL;  // 使能接收，忽略 modem 线

    // 原始模式（不处理特殊字符）
    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;
    tty.c_lflag &= ~ECHOE;
    tty.c_lflag &= ~ECHONL;
    tty.c_lflag &= ~ISIG;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);   // 无软件流控
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

    tty.c_oflag &= ~OPOST;          // 原始输出
    tty.c_oflag &= ~ONLCR;

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        std::cerr << "tcsetattr 失败: " << strerror(errno) << "\n";
        return false;
    }

    if (arbitrary)
        return setArbitraryBaud(fd_, baudrate);
    return true;
}

bool SerialPort::open(const std::string& port, int baudrate)
{
    close();
    // O_NONBLOCK 避免打开某些 USB 串口时卡住
    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        std::cerr << "打开串口失败 " << port << ": " << strerror(errno) << "\n";
        return false;
    }        
    // 切回阻塞模式
    int flags = fcntl(fd_, F_GETFL, 0);
    fcntl(fd_, F_SETFL, flags & ~O_NONBLOCK);

    if (!setInterfaceAttrs(baudrate)) {
        close();
        return false;
    }
    tcflush(fd_, TCIOFLUSH);
    std::cerr << "成功开启串口--" <<"端口:"<< param.port_name << "--波特率:" << param.baud_rate << std::endl;
    return true;
}

void SerialPort::close()
{
    if (fd_ != -1) {
        ::close(fd_);
        fd_ = -1;
    }
}

ssize_t SerialPort::write(const uint8_t* data, size_t len) 
{
    if (fd_ < 0) return -1;
    size_t total = 0;
    while (total < len) {
        ssize_t n = ::write(fd_, data + total, len - total);
        if (n < 0) {
            if (errno == EINTR) continue;         // 被信号打断，重试
            if (errno == EAGAIN) 
            {                // 缓冲满，等可写
                fd_set wfds;
                FD_ZERO(&wfds);
                FD_SET(fd_, &wfds);
                struct timeval tv{0, 100000};     // 100ms
                int ret = select(fd_ + 1, nullptr, &wfds, nullptr, &tv);
                if (ret <= 0) return total > 0 ? (ssize_t)total : -1;
                continue;
            }
            return total > 0 ? (ssize_t)total : -1;
        }
        total += n;
    }

    // 等数据真正从硬件发出（RS485 半双工必须）
    tcdrain(fd_);
    return (ssize_t)total;
}

ssize_t SerialPort::write(const std::vector<uint8_t>& data) 
{
    return write(data.data(), data.size());
}

ssize_t SerialPort::read(uint8_t* buffer, size_t len, int timeout_ms, uint64_t* first_byte_ns, int byte_timeout_ms) {
    if (fd_ < 0) return -1;

    size_t total = 0;
    while (total < len) {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(fd_, &rfds);

        struct timeval tv;
        struct timeval* tvp = nullptr;
        if (timeout_ms >= 0) {
            tv.tv_sec  = timeout_ms / 1000;
            tv.tv_usec = (timeout_ms % 1000) * 1000;
            tvp = &tv;
        }

        int ret = select(fd_ + 1, &rfds, nullptr, nullptr, tvp);
        if (ret < 0) {
            if (errno == EINTR) continue;
            return total > 0 ? (ssize_t)total : -1;
        }
        if (ret == 0) break;   // 超时

        ssize_t n = ::read(fd_, buffer + total, len - total);
        if (n < 0) {
            if (errno == EINTR || errno == EAGAIN) continue;
            return total > 0 ? (ssize_t)total : -1;
        }
        if (n == 0) break;

        // 收到首个字节时记录时间戳（单调时钟，用于测量响应延迟）
        if (total == 0 && first_byte_ns) {
            *first_byte_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count();
        }

        total += n;
        // 收到首个字节后，把超时缩短，用于等待帧内剩余字节
        // 帧内字节间隔远小于 byte_timeout_ms，超时说明帧已结束
        timeout_ms = byte_timeout_ms;
    }
    return (ssize_t)total;
}

ssize_t SerialPort::readExact(uint8_t* buffer, size_t len, int timeout_ms, int byte_timeout_ms) {
    if (fd_ < 0) return -1;

    size_t total = 0;
    while (total < len) {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(fd_, &rfds);

        // 首字节用完整超时，后续字节用帧内字节间隔超时
        int t = (total == 0) ? timeout_ms : byte_timeout_ms;
        struct timeval tv{t / 1000, (t % 1000) * 1000};

        int ret = select(fd_ + 1, &rfds, nullptr, nullptr, &tv);
        if (ret < 0) {
            if (errno == EINTR) continue;
            return total > 0 ? (ssize_t)total : -1;
        }
        if (ret == 0) return total > 0 ? (ssize_t)total : -1;  // 超时且未读满

        ssize_t n = ::read(fd_, buffer + total, len - total);
        if (n < 0) {
            if (errno == EINTR || errno == EAGAIN) continue;
            return total > 0 ? (ssize_t)total : -1;
        }
        if (n == 0) return total > 0 ? (ssize_t)total : -1;
        total += n;
    }
    return (ssize_t)total;
}

void SerialPort::flushInput()
{
    if (fd_ >= 0) tcflush(fd_, TCIFLUSH);
}

// 等总线静默：抛掉收到的字节，直到连续 quiet_ms 内无新数据
bool SerialPort::waitIdle(int quiet_ms, int max_ms)
{
    if (fd_ < 0) return false;
    if (quiet_ms < 1) quiet_ms = 1;

    uint8_t junk[256];
    auto t0 = std::chrono::steady_clock::now();
    while (true) {
        auto el = std::chrono::duration_cast<std::chrono::milliseconds>(
                      std::chrono::steady_clock::now() - t0).count();
        if (el >= max_ms) return false;   // 总等待超限，放弃（不阻塞实时循环）

        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(fd_, &rfds);
        struct timeval tv{quiet_ms / 1000, (quiet_ms % 1000) * 1000};

        int ret = select(fd_ + 1, &rfds, nullptr, nullptr, &tv);
        if (ret < 0) {
            if (errno == EINTR) continue;
            return false;
        }
        if (ret == 0) return true;        // 连续 quiet_ms 无数据 → 总线已静默

        ssize_t n = ::read(fd_, junk, sizeof(junk));   // 丢弃残余字节
        if (n < 0 && errno != EINTR && errno != EAGAIN) return false;
    }
}


