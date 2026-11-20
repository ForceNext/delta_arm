#include "rclcpp/rclcpp.hpp"
//115 150 292 40

class Kinematics : public rclcpp::Node
{
private:
    
public:
    Kinematics(std::string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(),"Kinematics is open.");
    }
};

int main(int argc,char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<Kinematics>("node_01");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
