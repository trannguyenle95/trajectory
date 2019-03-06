#include "ros/ros.h"
#include "std_msgs/String.h"
#include <math.h>
#include <sstream>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float64MultiArray.h"

#include <time.h>

#define looprate 100
#define init_t 30
#define pi 3.14159
int main(int argc, char** argv){

        ros::init(argc, argv, "trajectory");

        ros::NodeHandle n;

        ros::Publisher command_pub = n.advertise<std_msgs::Float64MultiArray>("/elfin/motion_control/command", 10);
        ros::Publisher command_pub1 = n.advertise<std_msgs::Float64MultiArray>("/elfin/computed_torque_controller_clik/command", 10);

        ros::Rate loop_rate(looprate);

        double start_time=ros::Time::now().toSec();

        std_msgs::Float64MultiArray msg;
        msg.layout.dim.push_back(std_msgs::MultiArrayDimension());

        msg.data.resize(18);
        for(int i=0;i<18;i++){
                msg.data[i]=0;
        }

        double x_init = 0;
        double y_init = -0.12;
        double z_init = 0.56;
        double amplitude=0.15;

        //starting position
        double y=0;
        double x=0;
        double z=0.89;

        double time=ros::Time::now().toSec()-start_time;

        //        for (int i=0; i<(init_t*looprate); i++){
        //        //while ((time<20) && (ros::ok())){

        //                double time=ros::Time::now().toSec()-start_time;

        //                //x+=x_init/(init_t*looprate);
        //                z+=(z_init-0.89)/(init_t*looprate);
        //                //msg.data[0]=x;
        //                msg.data[2]=z;
        //                ROS_INFO("time %f x %f ------ z %f ", time, x, z);

        //                command_pub.publish(msg);
        //                command_pub1.publish(msg);

        //                loop_rate.sleep();
        //        }

        //        ROS_INFO("done");
        //        sleep(5);

//        msg.data[0]=x_init;
//        msg.data[1]=y_init;
//        msg.data[2]=z_init;
//        command_pub.publish(msg);
//        command_pub1.publish(msg);

//        ROS_INFO("done");
//        sleep(5);

        while (ros::ok())
        {
                double time=ros::Time::now().toSec()-start_time;

                msg.data[0] = x_init;
                //msg.data[0] = amplitude*sin(time/2)+x_init;
                //msg.data[1] = y_init;
                msg.data[1] = amplitude*sin(time/2)+y_init;
                msg.data[2] = z_init;

                //msg.data[2] = 0.2*amplitude*sin(time)+z_init;

                msg.data[7] =0.5*amplitude*cos(time/2);
                //msg.data[8] =0.2*amplitude*cos(time);

                msg.data[10] = -0.5*0.5*amplitude*sin(time/2);
                //msg.data[11] = -0.2*amplitude*sin(time);

                command_pub.publish(msg);
                command_pub1.publish(msg);

                ros::spinOnce();
                loop_rate.sleep();
        }

        return 0;
}
