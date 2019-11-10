/*
    Created on: 19-Sep-2018
    Author: naiveHobo
*/

#include <ros/ros.h>

#include <gazebo_msgs/SetModelState.h>
#include <eve_msgs/Control.h>

#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>


class KeyboardController {

    private:
        ros::NodeHandle nh;
        ros::NodeHandle private_nh;

        ros::Publisher publisher;

        ros::ServiceClient respawn_client;
        gazebo_msgs::SetModelState model_state;

        double steering;
        int directionState;
        double throttleCmd;
        double brakeCmd;

        double maxSteer;
        double throttleRate;
        double steerRate;

        struct termios oldSettings;
        struct termios newSettings;

    public:

        KeyboardController(): private_nh("~") {

            this->steering = 0.0;
            this->throttleCmd = 0.0;
            this->brakeCmd = 0.0;
            this->directionState = eve_msgs::Control::NO_COMMAND;

            private_nh.param("throttle_rate", this->throttleRate, 0.5);
            private_nh.param("steer_rate", this->steerRate, 1.0);
            private_nh.param("max_steer", this->maxSteer, 12.5664);
            
            ROS_INFO("THROTTLE RATE: %lf", this->throttleRate);
            ROS_INFO("STEER RATE: %lf", this->steerRate);
            ROS_INFO("MAX STEER: %lf", this->maxSteer);
            
            tcgetattr(fileno(stdin), &this->oldSettings);
            this->newSettings = this->oldSettings;
            this->newSettings.c_lflag &= (~ICANON & ~ECHO);
            tcsetattr(fileno(stdin), TCSANOW, &this->newSettings);

            this->publisher = nh.advertise<eve_msgs::Control> ("eve_drive", 10);

            this->respawn_client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

            this->model_state.request.model_state.model_name = "eve";
            this->model_state.request.model_state.pose.position.x = 0.0;
            this->model_state.request.model_state.pose.position.y = 2.0;
            this->model_state.request.model_state.pose.position.z = 0.3;
        }

        void run() {      

            while (ros::ok()) {
                fd_set set;
                struct timeval tv;

                tv.tv_sec = 10;
                tv.tv_usec = 0;

                FD_ZERO(&set);
                FD_SET(fileno(stdin), &set);

                int res = select(fileno(stdin)+1, &set, NULL, NULL, &tv);

                if(res > 0) {
                    char c;
                    read(fileno(stdin), &c, 1);
                    this->keyControl(c);
                }

                ros::spinOnce();
            }
            
            tcsetattr(fileno(stdin), TCSANOW, &this->oldSettings);
        }

        void publishCommand() {
            eve_msgs::Control msg;
            msg.throttle = this->throttleCmd;
            msg.brake = this->brakeCmd;
            msg.steer = this->steering;
            msg.shift_gears = this->directionState;
            publisher.publish(msg);
        }

        void keyControl(int key) {

            this->directionState = eve_msgs::Control::NO_COMMAND;
            
            switch (key) {
                // w - accelerate forward
                case 87:
                case 119:
                {
                    this->directionState = eve_msgs::Control::FORWARD;
                    this->throttleCmd = this->throttleRate;
                    this->brakeCmd = 0.0;
                    break;
                }
                // a - steer left
                case 65:
                case 97:
                {
                    this->steering += this->steerRate;
                    this->steering = std::min(this->steering, this->maxSteer);
                    this->throttleCmd = 0.0;
                    this->brakeCmd = 0.0;
                    break;
                }
                // s - reverse
                case 83:
                case 115:
                {
                    this->directionState = eve_msgs::Control::REVERSE;
                    this->throttleCmd = this->throttleRate;
                    this->brakeCmd = 0.0;
                    break;
                }
                // d - steer right
                case 68:
                case 100:
                {
                    this->steering -= this->steerRate;
                    this->steering = std::max(this->steering, -this->maxSteer);
                    this->throttleCmd = 0.0;
                    this->brakeCmd = 0.0;
                    break;
                }
                // e brake
                case 69:
                case 101:
                {
                    this->brakeCmd = 1.0;
                    this->throttleCmd = 0.0;
                    break;
                }
                // x neutral
                case 88:
                case 120:
                {
                    this->directionState = eve_msgs::Control::NEUTRAL;
                    break;
                }
                // q reset
                case 81:
                case 113:
                {
                    this->steering = 0.0;
                    break;
                }
                // r respawn
                case 82:
                case 114:
                {
                    this->respawn_client.call(this->model_state);
                }
                // f reset
                case 70:
                case 102:
                {
                    this->brakeCmd = 1.0;
                    this->throttleCmd = 0.0;
                    this->steering = 0.0;
                    this->directionState = eve_msgs::Control::NEUTRAL;
                    break;
                }
                default:
                {
                    return;
                }
            }

            this->publishCommand();

            ROS_INFO("THROTTLE: %lf", this->throttleCmd);
            ROS_INFO("BRAKE: %lf", this->brakeCmd);
            ROS_INFO("STEERING ANGLE: %lf", this->steering);
            ROS_INFO("DIRECTION STATE: %d", this->directionState);
        }

};


int main(int argc, char ** argv) {
    ros::init(argc, argv, "keyboard_controller");
    
    KeyboardController control;
    control.run();

    return 0;
}