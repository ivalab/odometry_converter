#include <odometry_converter/convert.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pose_to_odom_converter");
  
  std::string pose_topic_ = "msf_core/odometry";
  std::string odom_topic_ = "visual/odom";
  std::string pose_link_ = "base_footprint";
  std::string cam_link_ = "camera_rgb_frame";
  
  ros::NodeHandle pnh("~");
  if(pnh.getParam("pose_topic", pose_topic_))
  {
    ROS_INFO_STREAM("pose_topic = " << pose_topic_);
  }
  if(pnh.getParam("odom_topic", odom_topic_))
  {
    ROS_INFO_STREAM("odom_topic = " << odom_topic_);
  }
    if(pnh.getParam("pose_link", pose_link_))
  {
    ROS_INFO_STREAM("pose_link = " << pose_link_);
  }
  if(pnh.getParam("cam_link", cam_link_))
  {
    ROS_INFO_STREAM("cam_link = " << cam_link_);
  }
  
  /*
  tf::TransformListener listener_;
  tf::StampedTransform transform_;

  // get the tf between sensor frame and base frame
  ros::Rate rate(10.0);
  while (1){
      try{
	listener_.lookupTransform(pose_link_, odom_link_,
				  ros::Time(0), transform_);
      }
      catch (tf::TransformException &ex) {
	ROS_ERROR("%s",ex.what());
	ros::Duration(1.0).sleep();
	continue;
      }
  
      ROS_INFO_STREAM("transform to be applied = " << transform_.getOrigin());
  
      break ;
    }
  */
  
  MessageConverter converter_(pose_topic_, odom_topic_, pose_link_, cam_link_);
  converter_.init();

  ros::spin();

//  ros::AsyncSpinner spinner(4);
//  spinner.start();
//  ros::waitForShutdown();
  
}
