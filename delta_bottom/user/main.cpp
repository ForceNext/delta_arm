#include "iostream"
#include "Utilities/PeriodicTask.h"
#include "yaml-cpp/yaml.h"
#include <vector>
#include <string>
#include <cstdint>

int main(int argc, char** argv)
{
    std::string path = (argc > 1) ? argv[1] : "config/Drive_Param.yaml";
    YAML::Node root = YAML::LoadFile(path);
    auto serial = root["serial"];
    std::string port = serial["port"].as<std::string>();
    int baudrate     = serial["baudrate"].as<int>();

    std::cout << "===== serial =====\n"<< "  port     = " << port << "\n"<< "  baudrate = " << baudrate << "\n";
    return 0;
}