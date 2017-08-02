#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

struct point{
  
std::vector<float> position;

} waypoint[3];

geometry_msgs::PoseStamped  set_pose;

void set_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    set_pose = *msg;
}


int main(int argc, char **argv)
{

  float soglia=0.5;
  
  std::vector<float> p0={0,0,5}; 
  std::vector<float> p1={3,0,5}; 
  std::vector<float> p2={0,0,2};
  
  waypoint[0].position=p0;
  waypoint[1].position=p1;
  waypoint[2].position=p2;
  
  std::cout << "p0x:"<<"\n"<< waypoint[0].position[0] << "\n" << "p0y:"<< "\n"<<waypoint[0].position[1] <<"\n"<< "p0z:"<<"\n"<< waypoint[0].position[2]  << std::endl;
  std::cout << "p1x:"<<"\n"<< waypoint[1].position[0] << "\n" << "p1y:"<< "\n"<<waypoint[1].position[1] <<"\n"<< "p1z:"<<"\n"<< waypoint[1].position[2]  << std::endl;
  std::cout << "p2x:"<<"\n"<< waypoint[2].position[0] << "\n" << "p2y:"<< "\n"<<waypoint[2].position[1] <<"\n"<< "p2z:"<<"\n"<< waypoint[2].position[2]  << std::endl;
  
  
  
    ros::init(argc, argv, "navig_node");
    ros::NodeHandle nh;
  
    ROS_INFO("Test_info");
   
    
   
    ros::Subscriber set_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
           ("mavros/setpoint_position/local", 10, set_cb);
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10);	


    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);
    
    geometry_msgs::PoseStamped current_pose;
    
    std::cout << "Definisco il mio primo obiettivo" <<std::endl;
   
   int waypoint_index = 0;
   
  ros::Time last_request = ros::Time::now();

    while(ros::ok()){
       
        
        
        pose_pub.publish(current_pose);
	
        
	if((fabs(set_pose.pose.position.x-current_pose.pose.position.x) < soglia) && 
	  ( fabs(set_pose.pose.position.y-current_pose.pose.position.y) <soglia) && 
	  ( fabs(set_pose.pose.position.z-current_pose.pose.position.z) < soglia))
      
	{
	    waypoint_index++;
	    set_pose.pose.position.x=waypoint[waypoint_index%3].position[0];
	    set_pose.pose.position.y=waypoint[waypoint_index%3].position[1];
	    set_pose.pose.position.z=waypoint[waypoint_index%3].position[2];
	      
	    std::cout << waypoint_index << "- new set pose (x,y,z):" << 
	    set_pose.pose.position.x << ", " 
	    <<set_pose.pose.position.y<<", "
	    <<set_pose.pose.position.z << std::endl;
	}
    
	//std::cout << "pose x:"<<"\n"<< current_pose.pose.position.x << "\n" << " pose y:"<< "\n"<<current_pose.pose.position.y<<"\n"<< "pose z:"<<"\n"<< current_pose.pose.position.z << std::endl;
      
	 
         ros::spinOnce();
         rate.sleep();
          
    }
      
    return 0;   
    
    
}