#ifndef TF_UTILS_ODOM_TO_TF_H
#define TF_UTILS_ODOM_TO_TF_H

#include <nav_msgs/Odometry.h>
#include <tf2/buffer_core.h>
//#include <tf2_ros/buffer.h>

namespace tf_utils
{
  inline
  geometry_msgs::TransformStamped toTransform(const nav_msgs::Odometry& odom)
  {
    geometry_msgs::TransformStamped transform;
    transform.header = odom.header;
    transform.child_frame_id = odom.child_frame_id;
    
    geometry_msgs::Vector3& trans = transform.transform.translation;
    const geometry_msgs::Point& pos = odom.pose.pose.position;
    
    trans.x = pos.x;
    trans.y = pos.y;
    trans.z = pos.z;
    
    transform.transform.rotation = odom.pose.pose.orientation;
    
    return transform;
  }
  
  inline
  geometry_msgs::TransformStamped toTransform(const nav_msgs::Odometry::ConstPtr& odom)
  {
    return toTransform(*odom);
  }
  

}

#endif //TF_UTILS_ODOM_TO_TF_H
