#include <ros/ros.h>
#include <std_msgs/Int32.h>

class MySub
{
public:
  MySub()
  {
    last_num_ = -1;
    sub_ = nh_.subscribe("/iterator", 1, &MySub::callback, this);
  }

  void
  step()
  {
    if (last_num_ >= 0)
    {
      ROS_INFO("Received: %d", last_num_);
    }
  }

private:
  void
  callback(const std_msgs::Int32::ConstPtr & msg)
  {
    last_num_ = msg->data;
  }

  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  int last_num_;
};

int
main(int argc, char ** argv)
{
  ros::init(argc, argv, "simple_sub_cpp_node");

  MySub mySub;
  ros::Rate loop_rate(2);
  while (ros::ok())
  {
    mySub.step();
    ros::spinOnce();
    loop_rate.sleep();
  }

  exit(EXIT_SUCCESS);
}