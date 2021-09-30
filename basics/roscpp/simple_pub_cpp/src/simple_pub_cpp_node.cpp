#include <ros/ros.h>
#include <std_msgs/Int32.h>

class MyPublisher
{
public:
  MyPublisher()
  {
    increment_ = 0;
    ROS_INFO("I'm the Constructor!");
    pub_ = nh_.advertise<std_msgs::Int32>("/iterator", 1);
  }

  void
  step()
  {
    std_msgs::Int32 msg;
    msg.data = increment_;
    
    pub_.publish(msg);

    increment_++;
  }

private:
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  int increment_;
};

int
main(int argc, char ** argv)
{
  ros::init(argc, argv, "simple_pub_cpp_node");

  MyPublisher myPub;
  ros::Rate loop_rate(2); // 2Hz
  while (ros::ok())
  {
    myPub.step();
    ros::spinOnce();
    loop_rate.sleep();
  }
  

  exit(EXIT_SUCCESS);
}