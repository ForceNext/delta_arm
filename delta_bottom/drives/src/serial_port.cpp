#include "serial_port.hpp"

SerialPort::~SerialPort() { close(); }

//串口类构造函数
SerialPort::SerialPort(int argc, char** argv)
{
    //读取yaml
    std::string path = (argc > 1) ? argv[1] : "config/Drive_Param.yaml";
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
    open(param.port_name, param.baud_rate);//开启串口
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
            std::cerr << "不支持的波特率: " << baudrate << "\n";
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
    if (speed < 0) return false;

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
            if (errno == EAGAIN) {                // 缓冲满，等可写
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

ssize_t SerialPort::read(uint8_t* buffer, size_t len, int timeout_ms) {
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

        total += n;
        // 收到首个字节后，把超时缩短，用于等待帧内剩余字节
        // 帧内字节间隔远小于 20ms，超时说明帧已结束
        timeout_ms = 20;
    }
    return (ssize_t)total;
}

void SerialPort::flushInput() 
{
    if (fd_ >= 0) tcflush(fd_, TCIFLUSH);
}


