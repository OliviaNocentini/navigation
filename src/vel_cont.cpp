#include "frames.hpp"

void quat2eul(float* roll,float* pitch,float* yaw,float q0,float q1,float q2,float q3 )
{
  //roll 
  float sinr = (2*(q0*q1+q2*q3));
  float cosr = 1-2*(pow(q1,2)+pow(q2,2));
  *roll = atan2(sinr, cosr);
  
  //pitch
  //TODO wrap!!!
  float sinp = 2*(q0*q2-q3*q1);
  *pitch = asin(sinp);
  
  //yaw
  float siny = 2*(q0*q3+q1*q2);
  float cosy = 1-2*(pow(q2,2)+pow(q3,2));
  *yaw = atan2(siny,cosy);  
}

//costruttore della classe 

Movements::Movements(ros::NodeHandle& nh)
{
  // topic subscribe 
  pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
    ("mavros/local_position/pose", 10, &Movements::pose_cb, this);

  // service subscr
  set_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
    ("mavros/setpoint_velocity/cmd_vel", 10);	 
}

void Movements::evaluate_error(float linear_error[3],float angular_error[3])
{
  float x=current_pose.pose.position.x;
  float y=current_pose.pose.position.y;
  float z=current_pose.pose.position.z;
  
  // get current attitude
  float roll,pitch,yaw;
  float q0=current_pose.pose.orientation.x;
  float q1=current_pose.pose.orientation.y;
  float q2=current_pose.pose.orientation.z;
  float q3=current_pose.pose.orientation.w;
  
  //unlock 
  // transform quaternion to euler
  quat2eul(&roll, &pitch, &yaw, q0, q1, q2, q3);
    
  float x_des = desired_pos[0];
  float y_des = desired_pos[1];
  float z_des = desired_pos[2];

  float roll_des = desired_att[0];
  float pitch_des = desired_att[1];
  float yaw_des = desired_att[2];
  
  linear_error[0] = x_des - x;
  linear_error[1] = y_des - y;
  linear_error[2] = z_des - z;

  angular_error[0] = roll_des - roll;
  angular_error[1] = pitch_des - pitch;
  angular_error[2] = yaw_des -yaw;
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

  float Kp=1;
  // set linear command
  command.twist.linear.x = Kp *linear_error[0];
  command.twist.linear.y = Kp *linear_error[1];
  command.twist.linear.z = Kp *linear_error[2];
  
  // set angular command
  command.twist.angular.x=Kp*angular_error[0] *0;
  command.twist.angular.y==Kp*angular_error[1] *0;
  command.twist.angular.z==Kp*angular_error[2] *0;

  // send the command
  set_vel_pub.publish(command);
  
  return ;  

}


 
void Movements::pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  //lock
  current_pose = *msg;
  
  //unlock
}

///////////////////////////////
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
  current_state = *msg;
}

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
    
    float toll = 0.05;
    
      if(count==0)
      {
	mov.set_desired_pos(0,0,1,0,0,0);
        if (mov.norma_linear_error()< toll)
          count++;
      }
      else if(count==1)
	mov.set_desired_pos(1,0,1,0,0,0);
	
      mov.set_command();
      ros::spinOnce();
      rate.sleep();
    }
    
   
  return 0;

}
