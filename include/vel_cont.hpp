#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <math.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>

class Movements
{
  private:
    
    
   // ros::Subscriber pose_sub;
    ros::Subscriber goal_sub;
    
    ros::Publisher set_vel_pub;

   // void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void goal_cb(const geometry_msgs::Pose::ConstPtr& msg);
    
   // geometry_msgs::PoseStamped current_pose;
    
   
    
    float desired_pos[3];
    float desired_att[3];


  public:
    Movements(ros::NodeHandle& nh);

    void set_command();
    void set_desired_pos(float,float,float,float,float,float);
    void evaluate_error(float linear_error[3],float angular_error[3]);
    float norma_linear_error();
    float norma_angular_error();
};
