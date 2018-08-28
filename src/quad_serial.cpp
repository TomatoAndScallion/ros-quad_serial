#include <ros/ros.h>
#include "serial/serial.h"
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <nav_msgs/Path.h>
#include <quad_serial/UavAngle.h>
#include <quad_serial/UavState.h>
/////////////////////////////////////////////////////////////////////////////////////
//数据拆分宏定义，在发送大于1字节的数据类型时，比如int16、float等，需要把数据拆分成单独字节进行发送
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)	  ) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )

#define RAD2DEG 57.2957795

serial::Serial ser;
uint8_t TX_Buff[27];
bool is_avaliable(const uint8_t *temp,uint8_t length)
{
    uint8_t sum = 0;;
    if(temp[0] == 0xAA && temp[1] == 0xAF && temp[2] == 0xF1 && temp[3] == 0x13)
    {
        for(int i = 0;i < length-1;i++)
        {
            sum += temp[i];
        }
        if(sum == temp[length-1])
        {
            return true;
        }
        else 
        {
            return false;
        }

    }
    return false;
}
void pose_feedback_Callback(const geometry_msgs::PoseStamped &msg)
{
    int16_t send_temp = 0;
    send_temp = (int16_t)(msg.pose.position.x * 100);
    TX_Buff[22] = BYTE1(send_temp);
    TX_Buff[23] = BYTE0(send_temp);
    send_temp = (int16_t)(msg.pose.position.y * 100);
    TX_Buff[24] = BYTE1(send_temp);
    TX_Buff[25] = BYTE0(send_temp);
    TX_Buff[7] = 1;
}

void pose_des_Callback(const geometry_msgs::PoseStamped &msg)
{
    int16_t send_temp = 0;
    send_temp = (int16_t)(msg.pose.position.x * 100);
    TX_Buff[8] = BYTE1(send_temp);
    TX_Buff[9] = BYTE0(send_temp);
    send_temp = (int16_t)(msg.pose.position.y * 100);
    TX_Buff[10] = BYTE1(send_temp);
    TX_Buff[11] = BYTE0(send_temp);
    send_temp = (int16_t)(msg.pose.position.z * 100);
    TX_Buff[12] = BYTE1(send_temp);
    TX_Buff[13] = BYTE0(send_temp);
    TX_Buff[4] = 1;
}

void vel_des_Callback(const geometry_msgs::TwistStamped &msg)
{
    int16_t send_temp = 0;
    
    
        
        send_temp = (int16_t)(msg.twist.linear.x * 100);
        TX_Buff[14] = BYTE1(send_temp);
        TX_Buff[15] = BYTE0(send_temp);
        send_temp = (int16_t)(msg.twist.linear.y * 100);
        TX_Buff[16] = BYTE1(send_temp);
        TX_Buff[17] = BYTE0(send_temp);
        send_temp = (int16_t)(msg.twist.linear.z * 100);
        TX_Buff[18] = BYTE1(send_temp);
        TX_Buff[19] = BYTE0(send_temp);
        TX_Buff[5] = 1;
    
   
    
        send_temp = (int16_t)(msg.twist.angular.z * 100);
        TX_Buff[20] = BYTE1(send_temp);
        TX_Buff[21] = BYTE0(send_temp);
        TX_Buff[6] = 1;
    
}

int main (int argc, char** argv)
{
    ros::init(argc, argv, "quad_serial");
    ros::NodeHandle nh;
    ros::Subscriber sub_pose_feedback = nh.subscribe("pose_feedback",10,pose_feedback_Callback);
    ros::Subscriber sub_pose_des = nh.subscribe("pose_des",10,pose_des_Callback);
    ros::Subscriber sub_vel_des = nh.subscribe("vel_des",10,vel_des_Callback);

    ros::Publisher pub_pose = nh.advertise<geometry_msgs::PoseStamped>("local_pose", 10);
    ros::Publisher pub_vel = nh.advertise<geometry_msgs::TwistStamped>("local_vel",10);
    ros::Publisher pub_state = nh.advertise<quad_serial::UavState>("uav_state",10);
    ros::Publisher pub_angle = nh.advertise<quad_serial::UavAngle>("uav_angle",10);
    ros::Publisher pub_path_rviz = nh.advertise<nav_msgs::Path>("trajectory_uav1",1);
    geometry_msgs::PoseStamped local_pose_pub;
    geometry_msgs::TwistStamped local_vel_pub;
    quad_serial::UavState uav_state_pub;
    quad_serial::UavAngle uav_angle_pub;

    //creat a serial port
    try
    {
        ser.setPort("/dev/ch340");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
        return -1;
    }
    nav_msgs::Path path_uav1;
    path_uav1.header.stamp = ros::Time::now();
    path_uav1.header.frame_id = "local_origin_ned";
    ros::Rate loop_rate(20);
    while(ros::ok()){
        
        ros::spinOnce();
        if(ser.available()){
            
            uint8_t com_temp[30];
            uint8_t com_count;
	        bool command=false;
            com_count=ser.available();
            ser.read(com_temp,com_count);
            command = is_avaliable(com_temp,com_count);
            

            if(command)//命令信息
            {
                geometry_msgs::PoseStamped this_pose_stamped_uav1;
                geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(0.0);

                local_pose_pub.header.stamp = ros::Time::now();
                local_pose_pub.header.frame_id = "ned_local_pose";
                local_pose_pub.pose.position.x = (float)((short)(com_temp[4]*256+com_temp[5]))/100;
                local_pose_pub.pose.position.y = (float)((short)(com_temp[6]*256+com_temp[7]))/100;
                local_pose_pub.pose.position.z = (float)((short)(com_temp[8]*256+com_temp[9]))/100;
                pub_pose.publish(local_pose_pub);

                this_pose_stamped_uav1.pose.position.x = local_pose_pub.pose.position.x;
                this_pose_stamped_uav1.pose.position.y = local_pose_pub.pose.position.y;
                this_pose_stamped_uav1.pose.orientation.x = goal_quat.x;
                this_pose_stamped_uav1.pose.orientation.y = goal_quat.y;
                this_pose_stamped_uav1.pose.orientation.z = goal_quat.z;
                this_pose_stamped_uav1.pose.orientation.w = goal_quat.w;
                this_pose_stamped_uav1.header.stamp = ros::Time::now();
                this_pose_stamped_uav1.header.frame_id = "local_origin_ned";
                path_uav1.poses.push_back(this_pose_stamped_uav1);
                pub_path_rviz.publish(path_uav1);

                local_vel_pub.header.stamp = ros::Time::now();
                local_vel_pub.header.frame_id = "ned_local_vel";
                local_vel_pub.twist.linear.x = (float)((short)(com_temp[10]*256+com_temp[11]))/100;
                local_vel_pub.twist.linear.y = (float)((short)(com_temp[12]*256+com_temp[13]))/100;
                local_vel_pub.twist.linear.z = (float)((short)(com_temp[14]*256+com_temp[15]))/100;
                pub_vel.publish(local_vel_pub);
                
                uav_state_pub.header.stamp = ros::Time::now();
                uav_state_pub.header.frame_id = "uav_statue";
                switch(com_temp[16])
                {
                    case 0:
                    {
                        uav_state_pub.connected = true;
                        uav_state_pub.armed = false;
                        uav_state_pub.is_user = false;
                        uav_state_pub.mode = "manual";
                        uav_state_pub.system_status = com_temp[16];
                        break;
                    }
                    case 1:
                    {
                        uav_state_pub.connected = true;
                        uav_state_pub.armed = true;
                        uav_state_pub.is_user = false;
                        uav_state_pub.mode = "manual";
                        uav_state_pub.system_status = com_temp[16];
                        
                        break;
                    }
                    case 2:
                    {
                        uav_state_pub.connected = true;
                        uav_state_pub.armed = true;
                        uav_state_pub.is_user = false;
                        uav_state_pub.mode = "altmode";
                        uav_state_pub.system_status = com_temp[16];
                        
                        break;
                    }
                    case 3:
                    {
                        uav_state_pub.connected = true;
                        uav_state_pub.armed = true;
                        uav_state_pub.is_user = false;
                        uav_state_pub.mode = "posmode";
                        uav_state_pub.system_status = com_temp[16];
                        
                        break;
                    }
                    case 4:
                    {
                        uav_state_pub.connected = true;
                        uav_state_pub.armed = true;
                        uav_state_pub.is_user = true;
                        uav_state_pub.mode = "usermode";
                        uav_state_pub.system_status = com_temp[16];
                       
                        break;
                    }
                }
                pub_state.publish(uav_state_pub);
                
                uav_angle_pub.header.stamp = ros::Time::now();
                uav_angle_pub.header.frame_id = "uav_angle";
                uav_angle_pub.pitch = (float)((short)(com_temp[17]*256+com_temp[18]))/10;
                uav_angle_pub.roll = (float)((short)(com_temp[19]*256+com_temp[20]))/10;
                uav_angle_pub.yaw = (float)((short)(com_temp[21]*256+com_temp[22]))/10;
                pub_angle.publish(uav_angle_pub);


            }
        }

//send data to uav
        TX_Buff[0] = 0xAA;
        TX_Buff[1] = 0xAF;
        TX_Buff[2] = 0xF1;
        TX_Buff[3] = 0x16;
        TX_Buff[26] = 0;
        for(int i = 0; i < 26; i++)
        {
            TX_Buff[26] += TX_Buff[i];
        }
        ser.write(TX_Buff,27);
        TX_Buff[4] = 0;
        TX_Buff[5] = 0;
        TX_Buff[6] = 0;
        TX_Buff[7] = 0;

	loop_rate.sleep();
        
    }
    return 0;
}
