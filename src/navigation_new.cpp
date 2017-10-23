#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <math.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include "vel_cont.hpp"
#include "angles/angles.h"
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include "tf/transform_datatypes.h"
#include <tf/transform_listener.h>

#define _KP_LINEAR .1
#define _KP_ANGULAR .1
#define alpha  1.38   //  angolo in radianti sarebbe 78,5 gradi scegliendo il field of view sopra orizzonte pari a 5 gradi.

#define TRESHOLD_WP (0.1)

float diff=0.5;


nav_msgs::Odometry current_pose_iRobot;

void pose_cb_iRobot(const nav_msgs::Odometry::ConstPtr& msg){
    current_pose_iRobot = *msg;
}


geometry_msgs::TransformStamped transformStamped;

//costruttore della classe 

Movements::Movements(ros::NodeHandle& nh)
{
  
set_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
    ("mavros/setpoint_velocity/cmd_vel", 10);	 
 
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Movements::evaluate_error(float linear_error[3],float angular_error[3])
{
  
  //definisco le posizioni del mio drone rispetto al sistema di riferimento ENU.
  
  float x_drone=transformStamped.transform.translation.x;
  float y_drone=transformStamped.transform.translation.y;
  float z_drone=transformStamped.transform.translation.z;
  
  float roll_drone(0),pitch_drone(0),yaw_drone;
  float q0_drone=transformStamped.transform.rotation.x;
  float q1_drone=transformStamped.transform.rotation.y;
  float q2_drone=transformStamped.transform.rotation.z;
  float q3_drone=transformStamped.transform.rotation.w;
  
  
  yaw_drone= tf::getYaw(transformStamped.transform.rotation);

  
  // trasformo le coordinate del fov secondo il sistema ENU

  
  tf::TransformListener listener_fov;
  
  //coordinate del fov rispetto al sistema drone
  
  geometry_msgs::PoseStamped fov_drone;
  
  fov_drone.header.frame_id = "iris/base_link";
  fov_drone.header.stamp = ros::Time();
  fov_drone.pose.position.x=z_drone*tan(alpha);
  fov_drone.pose.position.y=0;
  fov_drone.pose.position.z=-z_drone;
  fov_drone.pose.orientation.w=1.0;
  
  
  float roll_fov(0),pitch_fov(0),yaw_fov;
  fov_drone.pose.orientation.x=q0_drone;
  fov_drone.pose.orientation.y=q1_drone;
  fov_drone.pose.orientation.z=q2_drone;
  fov_drone.pose.orientation.w=q3_drone;
  
  yaw_fov=yaw_drone;
  
   
  geometry_msgs::PoseStamped fov_enu;
  
  // scrivo le coordinate del fov rispetto al sistema ENU
  
  try{
      listener_fov.transformPose("ENU",fov_drone, fov_enu);

    }
    catch(tf::TransformException& ex){
        ROS_ERROR("Received an exception trying to transform a point : %s", ex.what());
    }
 
  
  //la posizione desiderata è quella che assume il centro del mio roomba.
  
  float x_des = current_pose_iRobot.pose.pose.position.x;
  float y_des = current_pose_iRobot.pose.pose.position.y;
  float z_des = current_pose_iRobot.pose.pose.position.z;

  float roll_des = current_pose_iRobot.pose.pose.orientation.x;
  float pitch_des = current_pose_iRobot.pose.pose.orientation.y;
  float yaw_des = current_pose_iRobot.pose.pose.orientation.z;
  
  linear_error[0] = x_des - fov_enu.pose.position.x;
  linear_error[1] = y_des - fov_enu.pose.position.y;
  linear_error[2] = z_des - fov_enu.pose.position.z;

  angular_error[0] =angles::normalize_angle(roll_des - roll_fov);
  angular_error[1] =angles::normalize_angle(pitch_des - pitch_fov);
  angular_error[2] =angles::normalize_angle( yaw_des -yaw_fov);
  
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

geometry_msgs::PoseStamped Movements::fov_evaluation()

{

  float z_drone=transformStamped.transform.translation.z;

  
  tf::TransformListener listener_fov(ros::Duration(10));
  //tf::TransformListener listener_fov;

  
  
  geometry_msgs::PoseStamped fov_drone;
  
  fov_drone.header.frame_id = "iris/base_link";
  fov_drone.header.stamp = ros::Time(0);
  fov_drone.pose.position.x=z_drone*tan(alpha);
  fov_drone.pose.position.y=0;
  fov_drone.pose.position.z=-z_drone;
  fov_drone.pose.orientation.w=1.0;
  
  
     
  geometry_msgs::PoseStamped fov_enu;
  
  try{
    
   listener_fov.waitForTransform(
  fov_drone.header.frame_id,"ENU",fov_drone.header.stamp,ros::Duration(5));
    
   // std:: cout <<" base_link to enu init " <<std::endl;
    
      listener_fov.transformPose("ENU",fov_drone, fov_enu);
      
   //   std:: cout <<" base_link to enu end " <<std::endl;

    }
    catch(tf::TransformException& ex){
        ROS_ERROR("Received an exception trying to transform a point : %s", ex.what());
    }

    return fov_enu;
    
    
  
} 
  
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

float Movements::norma_linear_error()
{
  float linear_error[3];
  float angular_error[3];
  this->evaluate_error(linear_error, angular_error);
  float norma=0;
  for(int i=0; i<3 ; i++)
    norma += pow(linear_error[i],2);
 
  norma=sqrt(norma);
  
  return norma;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

float Movements::norma_angular_error()
{
  float linear_error[3];
  float angular_error[3];
  this->evaluate_error(linear_error, angular_error);
  float norma=0;
  for(int i=0; i<3 ; i++)
    norma += pow(angular_error[i],2);
 
  norma=sqrt(norma);
  
  return norma;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Movements::set_command()
{
  
 
  float linear_error[3];
  float angular_error[3];
  
  this->evaluate_error(linear_error,angular_error);
  
  geometry_msgs::TwistStamped command;

  float Kp_linear=_KP_LINEAR;
  
  // settare linear command
  
  command.twist.linear.x = Kp_linear * linear_error[0];
  command.twist.linear.y = Kp_linear * linear_error[1];
  command.twist.linear.z = Kp_linear * linear_error[2];
  
  float Kp_angular=_KP_ANGULAR;
  
  // set angular command
  command.twist.angular.x = 0;//Kp * angular_error[0];
  command.twist.angular.y = 0;//Kp * angular_error[1];
  command.twist.angular.z = Kp_angular * angular_error[2];

  // send the command
  
  std::cout << "Twist Command: " << std::endl; 
  std::cout << "linear = " << command.twist.linear.x << ", " << command.twist.linear.y << ", " << command.twist.linear.z << std::endl;
  std::cout << "angular = " << command.twist.angular.x << ", " << command.twist.angular.y << ", " << command.twist.angular.z << std::endl;
  set_vel_pub.publish(command);
  
  return ;  

}

geometry_msgs::TwistStamped Movements::set_command_second()
{
  float x_drone=transformStamped.transform.translation.x;
  float y_drone=transformStamped.transform.translation.y;
  float z_drone=transformStamped.transform.translation.z;
  
  float roll_drone(0),pitch_drone(0),yaw_drone;
  float q0_drone=transformStamped.transform.rotation.x;
  float q1_drone=transformStamped.transform.rotation.y;
  float q2_drone=transformStamped.transform.rotation.z;
  float q3_drone=transformStamped.transform.rotation.w;
  
  
  yaw_drone= tf::getYaw(transformStamped.transform.rotation);
  
  geometry_msgs::PoseStamped set_pose;
  
  set_pose.pose.position.x=0;
  set_pose.pose.position.y=0;
  set_pose.pose.position.z=2;
  
  geometry_msgs::TwistStamped com;
  
  com.header.stamp=ros::Time::now();
  com.header.frame_id="iris/base_link";

  float Kp_linear=0;
  
  // settare linear com
  
  com.twist.linear.x = Kp_linear *(set_pose.pose.position.x-x_drone) ;
  com.twist.linear.y = Kp_linear *(set_pose.pose.position.y-y_drone) ;
  com.twist.linear.z = Kp_linear *(set_pose.pose.position.z-z_drone);
  
  float Kp_angular=_KP_ANGULAR;
  
  // set angular command
  com.twist.angular.x = 0;
  com.twist.angular.y = 0;
  com.twist.angular.z =0.5;

  
  set_vel_pub.publish(com);
  
  return com;  

}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{
  // init ros
  ros::init(argc, argv, "navigation_new");
  ros::NodeHandle nh;
    
  // advertise publisher and subscriber
  ros::Subscriber iRobot_pose_sub = nh.subscribe<nav_msgs::Odometry>
           ("iRobot/odom", 10, pose_cb_iRobot);
  ros::Publisher set_pose_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);	

  //the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(10.0);
  
  // tf init (listener for drone position)   
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  
   
  //instanziare l'oggetto mov nella classe movements cioè alloco in memoria il mio oggetto.
  Movements mov(nh);

  
  
   geometry_msgs::PoseStamped set_pose; 
   std::cout << "Definisco fino a dove si deve alzare il mio drone" <<std::endl;
   std::vector<float> p0={0,0,2};
   set_pose.pose.position.x=p0[0];
   set_pose.pose.position.y=p0[1];
   set_pose.pose.position.z=p0[2];
   //set_pose.pose.orientation.w=1;

   
  // float yaw= tf::getYaw(transformStamped.transform.rotation);
   
  typedef enum {SET_ALTITUDE, FIND_ROOMBA} tasks;
  tasks current_task = SET_ALTITUDE;
  

 /////////////////////////////////////////////////////////////////////


  while(ros::ok())
  {
    transformStamped.header.stamp = ros::Time::now();
  
    //set the name of the parent frame of the link we're creating and the name of the child
    transformStamped.header.frame_id = "ENU";
    transformStamped.child_frame_id = "iris/base_link";
    try{
      //std:: cout <<" ENU to base_link start" <<std::endl;
      transformStamped = tfBuffer.lookupTransform("ENU", "iris/base_link",
                               ros::Time(0));  
     // std:: cout <<" ENU to base_link end" <<std::endl;
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    
    
   // std::cout << "vado a calcolare il mov: "<< mov.fov_evaluation() <<std::endl;
     
    switch(current_task)
    {
      case SET_ALTITUDE:
	// set desired position
	set_pose_pub.publish(set_pose);
     
	// is pointh reached? 
        if((fabs(set_pose.pose.position.x - transformStamped.transform.translation.x) < TRESHOLD_WP) && 
	   (fabs(set_pose.pose.position.y - transformStamped.transform.translation.y) < TRESHOLD_WP) && 
	   (fabs(set_pose.pose.position.z - transformStamped.transform.translation.z) < TRESHOLD_WP))
	{
	  std::cout << "raggiunta la posizione stabilita" <<std::endl;
	  //current_task = FIND_ROOMBA;
	}
	break;
    }
	  
	  
 /* if(task ==2)
    
  {    
    
        mov.set_command_second();
	
	std::cout << "velocità a cui ruota il roomba "<< mov.set_command_second() <<std::endl;
	 
	 // il drone ruota su se stesso  e si ferma quando il centro del suo field of view è vicino al roomba
	 
	  
	   if(fabs(current_pose_iRobot.pose.pose.position.x ) < diff )
	    
	  {
	         std::cout << "roomba vicino al fov" <<std::endl;
	           task++;
	        
	  }
        
        
   
  }*/
    
   /*if(task==3)
     
     
   {
      
     std::cout << " inizia l'inseguimento del roomba" <<std::endl;
     mov.set_command();
     
     // ci andrà messo la parte di collision avoidance
     
   }*/
 
  
  
  //std::cout << __func__ << " new " << std::endl;
 
         ros::spinOnce();
         rate.sleep();
	 
 }
          
    
      
    return 0;   
    
    
}