//when debug this define must comentout
#define ENABLE_pigpio

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <stepper_interfaces/msg/stepper_msgs.hpp>
#include <iostream>
#include <string>
#include <thread>
#include <stdio.h>
#include <unistd.h>
#ifdef ENABLE_pigpio
    #include <pigpiod_if2.h>
#endif
using namespace std;
using stepper_interfaces::msg::StepperMsgs;

//global nodes
rclcpp::Node::SharedPtr g_BioStep_node = nullptr;
//global variable
bool interrupt = false;
bool spin_finite = false;
bool spin_infinite = false;
bool power = false;
int step_spin = 0;
string cmd = "";
int mini_stepper_delay_us;
int max_stepper_delay_us;
int stepper_us = 0;

//callback topic command
void interruptCommand(const std_msgs::msg::String::SharedPtr msg){
    if(msg->data == "STOP"){    //cmd:STOP
        RCLCPP_INFO(g_BioStep_node->get_logger(), "Stop stepper motor");
        spin_finite = false;
        spin_infinite = false;
        power = false;
    }else if(msg->data == "BREAK"){    //cmd::BREAK
        RCLCPP_INFO(g_BioStep_node->get_logger(), "Break stepper motor");
        spin_finite = false;
        spin_infinite = false;
        power = true;
    }
}

//callback move stepper
void moveStepper(const StepperMsgs::SharedPtr msg){
    RCLCPP_INFO(g_BioStep_node->get_logger(), "get signal");
    //Define spin speed
    stepper_us = (max_stepper_delay_us * (msg->step_speed / 100.0)) + mini_stepper_delay_us;
    //Stepper motor power on
    power = msg->step_power;
    //Save step value
    step_spin = msg->step_val;
    //Doing check finite rotation or infinite rotation
    if(msg->step_specify == true){    //When finite rotation
        spin_finite = true;
        spin_infinite = false;
    }else{    //When infinite rotation
        spin_finite = false;
        spin_infinite = true;
    }
}

int main(int argc, char **argv){
    //init
    rclcpp::init(argc, argv);
    g_BioStep_node = rclcpp::Node::make_shared("bipolar_stepper");
    RCLCPP_INFO(g_BioStep_node->get_logger(), "start bipolar_stepper");
    auto subCmd = g_BioStep_node->create_subscription<std_msgs::msg::String>("interrCmd", 10, interruptCommand);
    auto subStepper = g_BioStep_node->create_subscription<StepperMsgs>("stepperCmd", 10, moveStepper);
    //read parameter
    int step_pin = g_BioStep_node->declare_parameter<int>("step_pin", 0);
    int dir_pin = g_BioStep_node->declare_parameter<int>("dir_pin", 0);
    int enable_pin = g_BioStep_node->declare_parameter<int>("enable_pin", 0);
    int step = g_BioStep_node->declare_parameter<int>("step", 200);
    double mini_rpm = g_BioStep_node->declare_parameter<double>("mini_rpm", 0.0);
    double max_rpm = g_BioStep_node->declare_parameter<double>("max_rpm", 100.0);
    RCLCPP_INFO(g_BioStep_node->get_logger(), "read stepper parameter \n  step_pin:%d,\n  dir_pin:%d,\n  enable_pin:%d,\n  step:%d,\n  mini_rpm: %f[rpm],\n  max_rpm: %f[rpm]", step_pin, dir_pin, enable_pin, step, mini_rpm, max_rpm);
    //connect to a pigpio deamon
    #ifdef ENABLE_pigpio
        int pi = pigpio_start(NULL, NULL);
        if (pi < 0){
            RCLCPP_INFO(g_BioStep_node->get_logger(), "ERROR: The pigpio daemon isn't running");
            return 1;
        }
        set_mode(pi, step_pin, PI_OUTPUT);
        set_mode(pi, dir_pin, PI_OUTPUT);
        set_mode(pi, enable_pin, PI_OUTPUT);
    #else
        RCLCPP_INFO(g_BioStep_node->get_logger(), "setup pigpio");
    #endif
    //setting speed[us]
    mini_stepper_delay_us = ((60000000.0 / mini_rpm) / step) / 2.0; 
    max_stepper_delay_us = ((60000000.0 / max_rpm) / step) / 2.0;
    //execute spin function in another thread
    rclcpp::WallRate(1);
    std::thread([]{rclcpp::spin(g_BioStep_node);}).detach();
    RCLCPP_INFO(g_BioStep_node->get_logger(), "Stepper motor can moving...");
    while(rclcpp::ok()){
        #ifdef ENABLE_pigpio
            //stepper use pigpio
            //Doing check finite rotation or infinite rotation
            if(spin_finite == true){    //When finite rotation
                int spinning = 0;
                gpio_write(pi, enable_pin, PI_ON);
                if(step_spin > 0){    //right spin
                    step_spin = step_spin;
                    gpio_write(pi, dir_pin, PI_ON);
                    while (spinning < step_spin && spin_finite == true){
                        //RCLCPP_INFO(g_BioStep_node->get_logger(), "Right spinning stepper %d[step], remaining %d[step], speed %d[us]", spinning, (step_spin - spinning), stepper_us);
                        spinning++;
                        gpio_write(pi, step_pin, PI_ON);
                        usleep(stepper_us);
                        gpio_write(pi, step_pin, PI_OFF);
                        usleep(stepper_us);
                    }
                }
                else if(step_spin < 0){    //lsft spin
                    step_spin = step_spin * (-1);
                    gpio_write(pi, dir_pin, PI_OFF);
                    while (spinning < step_spin && spin_finite == true){
                        //RCLCPP_INFO(g_BioStep_node->get_logger(), "Left spinning stepper %d[step], remaining %d[step], speed %d[us]", spinning, (step_spin - spinning), stepper_us);
                        spinning++;
                        gpio_write(pi, step_pin, PI_ON);
                        usleep(stepper_us);
                        gpio_write(pi, step_pin, PI_OFF);
                        usleep(stepper_us);
                    }
                }
                if(power == true){    //stepper power on
                    gpio_write(pi, enable_pin, PI_ON);
                }else{    //stepper power off
                    gpio_write(pi, enable_pin, PI_OFF);
                    power = false;
                }
                spin_finite = false;
            }else if(spin_infinite == true){    //When finite rotation
                gpio_write(pi, enable_pin, PI_ON);
                if(step_spin >= 0){    //right spin
                    gpio_write(pi, dir_pin, PI_ON);
                    while (spin_infinite == true){
                        //RCLCPP_INFO(g_BioStep_node->get_logger(), "Right infinite spinning stepper speed %d[us]", stepper_us);
                        gpio_write(pi, step_pin, PI_ON);
                        usleep(stepper_us);
                        gpio_write(pi, step_pin, PI_OFF);
                        usleep(stepper_us);
                    }
                }
                else if(step_spin < 0){    //lsft spin
                    gpio_write(pi, dir_pin, PI_OFF);
                    while (spin_infinite == true){
                        //RCLCPP_INFO(g_BioStep_node->get_logger(), "Left infinite spinning stepper speed %d[us]", stepper_us);
                        gpio_write(pi, step_pin, PI_ON);
                        usleep(stepper_us);
                        gpio_write(pi, step_pin, PI_OFF);
                        usleep(stepper_us);
                    }
                }
                if(power == true){    //stepper power on
                    gpio_write(pi, enable_pin, PI_ON);
                }else{    //stepper power off
                    gpio_write(pi, enable_pin, PI_OFF);
                    power = false;
                }
            }
        #else
            //Doing check finite rotation or infinite rotation
            if(spin_finite == true){    //When finite rotation
                int spinning = 0;
                if(step_spin > 0){    //right spin
                    step_spin = step_spin;
                    while (spinning < step_spin && spin_finite == true){
                        RCLCPP_INFO(g_BioStep_node->get_logger(), "Right spinning stepper %d[step], remaining %d[step], speed %d[us]", spinning, (step_spin - spinning), stepper_us);
                        spinning++;
                        usleep(stepper_us);
                        usleep(stepper_us);
                    }
                }
                else if(step_spin < 0){    //lsft spin
                    step_spin = step_spin * (-1);
                    while (spinning < step_spin && spin_finite == true){
                        RCLCPP_INFO(g_BioStep_node->get_logger(), "Left spinning stepper %d[step], remaining %d[step], speed %d[us]", spinning, (step_spin - spinning), stepper_us);
                        spinning++;
                        usleep(stepper_us);
                        usleep(stepper_us);
                    }
                }
                if(power == true){    //stepper power on
                    RCLCPP_INFO(g_BioStep_node->get_logger(), "Finish spin : power on");
                }else{    //stepper power off
                    RCLCPP_INFO(g_BioStep_node->get_logger(), "Finish spin : power off");
                    power = false;
                }
                spin_finite = false;
            }else if(spin_infinite == true){    //When finite rotation
                if(step_spin >= 0){    //right spin
                    while (spin_infinite == true){
                        RCLCPP_INFO(g_BioStep_node->get_logger(), "Right infinite spinning stepper speed %d[us]", stepper_us);
                        usleep(stepper_us);
                        usleep(stepper_us);
                    }
                }
                else if(step_spin < 0){    //lsft spin
                    while (spin_infinite == true){
                        RCLCPP_INFO(g_BioStep_node->get_logger(), "Left infinite spinning stepper speed %d[us]", stepper_us);
                        usleep(stepper_us);
                        usleep(stepper_us);
                    }
                }
                if(power == true){    //stepper power on
                    RCLCPP_INFO(g_BioStep_node->get_logger(), "Finish spin : power on");
                }else{    //stepper power off
                    RCLCPP_INFO(g_BioStep_node->get_logger(), "Finish spin : power off");
                    power = false;
                }
            }
        #endif
    }
    //disconnect from a pigpio deamon
    #ifdef ENABLE_pigpio
        pigpio_stop(pi);
        RCLCPP_INFO(g_BioStep_node->get_logger(), "stop pigpio");
    #else
        RCLCPP_INFO(g_BioStep_node->get_logger(), "stop stepper");
    #endif
    g_BioStep_node = nullptr;
    rclcpp::shutdown();
    return 0;
}