#include "vel_cont.hpp"
#include "angles/angles.h"

#include "tf/transform_datatypes.h"

#define _KP_LINEAR 1
#define _KP_ANGULAR .1

geometry_msgs::Pose desired_goal_pose;

void quat2eul(float* roll,float* pitch,float* yaw,float q0,float q1,float q2,float q3 )
{
  //roll 
  float sinr = (2*(q0*q1+q2*q3));
  float cosr = 1-2*(pow(q1,2)+pow(q2,2));
  *roll = atan2(sinr, cosr);
  
  //pitch
   float sinp = 2*(q0*q2-q3*q1);
   if (fabs(sinp) >= 1)
     *pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range //copysign function that dà come valore il primo float con il segno del secondo.
   else
     *pitch = asin(sinp);

  //yaw
  float siny = 2*(q0*q3+q1*q2);
  float cosy = 1-2*(pow(q2,2)+pow(q3,2));
  *yaw = atan2(siny,cosy);  
}

//costruttore della classe 

Movements::Movements(ros::NodeHandle& nh)
{
  // topic subscriber 
  pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
    ("mavros/local_position/pose", 10, &Movements::pose_cb, this);

  // service subscriber
  set_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
    ("mavros/setpoint_velocity/cmd_vel", 10);	 
    
  // topic subscriber 
  goal_sub = nh.subscribe<geometry_msgs::Pose>
    ("/goal", 10, &Movements::goal_cb, this);
    
  this->set_desired_pos(0,0,1,0,0,0);
}

void Movements::evaluate_error(float linear_error[3],float angular_error[3])
{
  float x=current_pose.pose.position.x;
  float y=current_pose.pose.position.y;
  float z=current_pose.pose.position.z;
  
  // get current attitude
  float roll(0),pitch(0),yaw;
  float q0=current_pose.pose.orientation.x;
  float q1=current_pose.pose.orientation.y;
  float q2=current_pose.pose.orientation.z;
  float q3=current_pose.pose.orientation.w;
  
  //unlock 
  // transform quaternion to euler
  //quat2eul(&roll, &pitch, &yaw, q0, q1, q2, q3);
    
  yaw= tf::getYaw(current_pose.pose.orientation);
  
  float x_des = desired_pos[0];
  float y_des = desired_pos[1];
  float z_des = desired_pos[2];

  float roll_des = desired_att[0];
  float pitch_des = desired_att[1];
  float yaw_des = desired_att[2];
  
  linear_error[0] = x_des - x;
  linear_error[1] = y_des - y;
  linear_error[2] = z_des - z;

  angular_error[0] =angles::normalize_angle(roll_des - roll);
  angular_error[1] =angles::normalize_angle(pitch_des - pitch);
  angular_error[2] =angles::normalize_angle( yaw_des -yaw);
}

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

void Movements::set_desired_pos(float x, float y, float z, float roll, float pitch, float yaw)
{
    desired_pos[0]=x;
    desired_pos[1]=y;
    desired_pos[2]=z;
    desired_att[0]=roll;
    desired_att[1]=pitch;
    desired_att[2]=yaw;
}


void Movements::set_command()
{
  
  //dovrei usare i semafori lock
  // get current position
 
  float linear_error[3];
  float angular_error[3];
  
  this->evaluate_error(linear_error,angular_error);
  
  geometry_msgs::TwistStamped command;

  float Kp_linear=_KP_LINEAR;
  // set linear command
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

/////////////////////////////////////////////////////////////////////////
void Movements::pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  //lock
  current_pose = *msg;
  
  float toll = 0.05;
  //if (this->norma_linear_error()< toll)
  
  //unlock
}

/////////////////////////////////////////////////////////////////////////
void Movements::goal_cb(const geometry_msgs::Pose::ConstPtr& msg)
{
  
  double yaw = tf::getYaw(msg->orientation);
  
  // transform quaternion to euler
  //quat2eul(&roll, &pitch, &yaw, q0, q1, q2, q3);
  this->set_desired_pos(msg->position.x,msg->position.y, msg->position.z, 0, 0, yaw);
  
  std::cout << "desired pose: " << msg->position.x << ", " << msg->position.y << ", " << msg->position.z << ", " <<  yaw << std::endl;
}

/////////////////////////////////////////////////////////////////////////
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
  current_state = *msg;
}

/////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
  //Inizializzo nodo
  ros::init(argc, argv, "frames_node");
  ros::NodeHandle nh;
    
  //debug
  ROS_INFO("Test_info");
    
  // topic subscriber
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
    ("mavros/state", 10, state_cb);
    
  // services
  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
    ("mavros/cmd/arming");
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
    ("mavros/set_mode");
    
  //instanziare l'oggetto mov nella classe movements cioè alloco in memoria il mio oggetto.

  Movements mov(nh);
	    
  //the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(20.0);

  
  int count=0;
  
  // wait for FCU connection
  while(ros::ok() && current_state.connected){
    ros::spinOnce();
    rate.sleep();
    ROS_INFO("State Not Connected");

  }
    
  ROS_INFO("Position Selected");

  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";

  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;

  ros::Time last_request = ros::Time::now();

  while(ros::ok())
    {
      if( current_state.mode != "OFFBOARD" &&
	  (ros::Time::now() - last_request > ros::Duration(5.0)))
	{
	  if( set_mode_client.call(offb_set_mode) &&
	      offb_set_mode.response.success)
	    {
	      ROS_INFO("Offboard enabled");
            }
	  last_request = ros::Time::now();
        } 
      else
	{
	  if( !current_state.armed &&
	      (ros::Time::now() - last_request > ros::Duration(5.0)))
	    {
	      if( arming_client.call(arm_cmd) &&
		  arm_cmd.response.success)
		{
		  ROS_INFO("Vehicle armed");
                }
	      last_request = ros::Time::now();
            }
        }
    
   
/*    
      if(count==0)
      {
	std::cout <<"sono dentro primo count" <<std::endl;
	mov.set_desired_pos(1,0,1,0,0,0);
        if (mov.norma_linear_error()< toll)
          count++;
      }
      else if(count==1)
      {
	
	std::cout <<"sono dentro secondo count" <<std::endl;
	mov.set_desired_pos(0,0,1,0,0,1.57);
      }*/
      /*
      else if(count==2)
	mov.set_desired_pos(1,0,1,0,0,0);
      */	
      mov.set_command();
      ros::spinOnce();
      rate.sleep();
    }
    
   
  return 0;

}
