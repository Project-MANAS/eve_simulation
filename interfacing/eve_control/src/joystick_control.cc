/*
    Created on: 19-Sep-2018
    Author: naiveHobo
*/

#include <ros/ros.h>

#include <gazebo_msgs/SetModelState.h>
#include <eve_msgs/Control.h>

#include "joystick.hh"

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <memory>


class JoystickController {

    private:

        enum class ButtonMap {
                THROTTLE = 0,       // A
                BRAKE = 1,          // B
                GEAR_FORWARD = 5,   // RB
                GEAR_REVERSE = 4,   // LB
                GEAR_NEUTRAL = 2,   // X
                RESPAWN = 3         // Y
            };

        enum class AxisMap {
                STEER_X = 0,        // LEFT ANALOG
                STEER_Y = 1,        // LEFT ANALOG
                THROTTLE = 4,       // RIGHT ANALOG
                BRAKE = 5,          // RT
                NITRO = 2,          // LT
                GEAR_SHIFT = 7      // ARROW KEYS
            };

        ros::NodeHandle nh;

        ros::NodeHandle private_nh;
        ros::Publisher publisher;
        
        std::shared_ptr<Joystick> joystick;

        ros::ServiceClient respawn_client;
        gazebo_msgs::SetModelState model_state;

        // Current steering angle
        double steering;

        // Current throttle (0 - 1.0)
        double throttle;

        // Current brake (0 - 1.0)
        double brake;

        // Current gear state
        int directionState;

        // Current Analog Stick position for Steering
        double steer_x;
        double steer_y;

        // Current nitro value
        double nitro;

        // Max throttle
        double throttleRate;

        // Max brack
        double brakeRate;

        // Max steering angle
        double maxSteer;

        // Max nitro (1.0 - throttleRate)
        double maxNitro;

        // Angle change in steering position per unit change in joystick analog position
        double steerRatio;


    public:

        JoystickController() : private_nh("~") {
            
            int dev;
            private_nh.param("device_id", dev, 0);
            joystick.reset(new Joystick(dev));

            if (!this->joystick->isFound()) {
                ROS_ERROR("Could not find JoyStick device");
                exit(1);
            } else {
                ROS_INFO("Found JoyStick at device with id [%d]", dev);
            }

            this->steering = 0.0;
            this->throttle = 0.0;
            this->brake = 0.0;
            this->directionState = eve_msgs::Control::NO_COMMAND;
            this->steer_x = 0;
            this->steer_y = 0;
            this->nitro = 0.0;

            private_nh.param("throttle_rate", this->throttleRate, 0.5);
            private_nh.param("brake_rate", this->brakeRate, 1.0);
            private_nh.param("max_steer", this->maxSteer, 12.5664);

            ROS_INFO("THROTTLE RATE: %lf", this->throttleRate);
            ROS_INFO("BRAKE RATE: %lf", this->brakeRate);
            ROS_INFO("MAX STEER: %lf", this->maxSteer);

            this->maxNitro = (1.0 - this->throttleRate) / 65535.0;
            this->throttleRate /=  32767.0;
            this->brakeRate /=  65535.0;
            this->maxSteer /= 32767.0;

            this->steerRatio = 2 * this->maxSteer;

            this->publisher = nh.advertise<eve_msgs::Control> ("eve_drive", 10);

            this->respawn_client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

            this->model_state.request.model_state.model_name = "eve";
            this->model_state.request.model_state.pose.position.x = 0.0;
            this->model_state.request.model_state.pose.position.y = 2.0;
            this->model_state.request.model_state.pose.position.z = 0.3;
        }

        void run() {      

            while(ros::ok()) {
                
                usleep(1000); 
                
                JoystickEvent event;

                if (this->joystick->sample(&event)){
                    this->directionState = eve_msgs::Control::NO_COMMAND;
                    this->eventControl(event); 
                }

                this->publishCommand();

                ros::spinOnce();
            }    
        }

        void eventControl(JoystickEvent event) {
            
            if (event.isButton()) {

                // ROS_INFO("Button %u is %s", event.number, event.value == 0 ? "up" : "down");

                ButtonMap event_num = static_cast<ButtonMap>(event.number);
                
                switch(event_num) {
                    case ButtonMap::THROTTLE:
                    {
                        if(event.value == 0)
                            this->throttle = 0.0;
                        else {
                            this->throttle = this->throttleRate * 32767.0;
                            this->throttle += this->nitro;
                            this->brake = 0.0;
                        }
                        break;
                    }
                    case ButtonMap::BRAKE:
                    {
                        if(event.value == 0)
                            this->brake = 0.0;
                        else {
                            this->brake = this->brakeRate * 65535.0;
                            this->throttle = 0.0;
                        }
                        break;
                    }
                    case ButtonMap::GEAR_FORWARD:
                    {
                        this->directionState = eve_msgs::Control::FORWARD;
                        break;
                    }
                    case ButtonMap::GEAR_REVERSE:
                    {
                        this->directionState = eve_msgs::Control::REVERSE;
                        break;
                    }
                    case ButtonMap::GEAR_NEUTRAL:
                    {
                        this->directionState = eve_msgs::Control::NEUTRAL;
                        this->throttle = 0.0;
                        this->brake = this->brakeRate;
                        break;
                    }
                    case ButtonMap::RESPAWN:
                    {
                        if(event.value == 0) {
                            this->brake = 1.0;
                            this->throttle = 0.0;
                            this->steering = 0.0;
                            this->directionState = eve_msgs::Control::NEUTRAL;
                            this->respawn_client.call(this->model_state);
                        }
                        break;
                    }
                }
            }
            else if (event.isAxis()) {

                // ROS_INFO("Axis %u is at position %d", event.number, event.value);

                AxisMap event_num = static_cast<AxisMap>(event.number);
                
                switch(event_num) {
                    case AxisMap::STEER_X:
                    {
                        this->steer_x = -event.value * this->maxSteer;
                        this->steering = this->steer_x;
                        break;
                    }
                    case AxisMap::STEER_Y:
                    {
                        this->steer_y = event.value;
                        break;
                    }
                    case AxisMap::THROTTLE:
                    {
                        if(event.value < 0) {
                            this->directionState = eve_msgs::Control::FORWARD;
                            this->throttle = -event.value * this->throttleRate;
                            this->throttle += this->nitro;
                        }
                        else if (event.value > 0){
                            this->directionState = eve_msgs::Control::REVERSE;
                            this->throttle = event.value * this->throttleRate;
                            this->throttle += this->nitro;
                        }
                        else
                            this->throttle = 0.0;
                        this->brake = 0.0;
                        break;
                    }
                    case AxisMap::BRAKE:
                    {
                        this->brake = (event.value + 32767) * this->brakeRate;
                        break;
                    }
                    case AxisMap::NITRO:
                    {
                        this->nitro = (event.value + 32767) * this->maxNitro;
                        break;
                    }
                }
            }
        }

        void publishCommand() {
            eve_msgs::Control msg;
            msg.throttle = this->throttle;
            msg.brake = this->brake;
            msg.steer = this->steering;
            msg.shift_gears = this->directionState;
            publisher.publish(msg);
        }

};


int main(int argc, char ** argv) {
    ros::init(argc, argv, "joystick_controller");
    
    JoystickController control;
    control.run();

    return 0;
}
