#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <math.h>

geometry_msgs::PoseStamped current_pose;


void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose = *msg;
}


mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    std::cout << "Test_prima" << std::endl;
  
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
  
    ROS_INFO("Test_info");
    
    
    
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, pose_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::Publisher set_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("mavros/setpoint_velocity/cmd_vel", 10);	
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
	ROS_INFO("State Not Connected");

    }

    geometry_msgs::PoseStamped set_pose;
    
   
    set_pose.pose.position.x = 0;
    set_pose.pose.position.y = 0;
    set_pose.pose.position.z = 1;
    
    set_pose.pose.orientation.x = 0;
    set_pose.pose.orientation.y = 0;
    set_pose.pose.orientation.z = 0;
    set_pose.pose.orientation.w = 1;
    
    
    
    
    
    
    
    
    
//     //send a few setpoints before starting
//     for(int i = 100; ros::ok() && i > 0; --i){
//         local_pos_pub.publish(set_pose);
//         ros::spinOnce();
//         rate.sleep();
// 	ROS_INFO("%d...",i);
//     }
    
    ROS_INFO("Position Selected");

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.success){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        local_pos_pub.publish(set_pose);
	
	
	
	
	
	
	
	
	
	
	
 std::cout << "pos x"<<"\n"<< current_pose.pose.position.x << "\n" << " pos y"<< "\n"<<current_pose.pose.position.y<<"\n"<< "pos z"<<"\n"<< current_pose.pose.position.z << std::endl;
      
    
        ros::spinOnce();
        rate.sleep();
    }
    
   
    return 0;

}