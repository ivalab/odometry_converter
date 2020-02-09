#ifndef CONVERT_H

#include "odom_to_tf.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

class MessageConverter
{
  private:
    ros::NodeHandle nh_;
    // ros::Subscriber sub_;
    message_filters::Subscriber<nav_msgs::Odometry> sub_;
    ros::Publisher pub_;
    ros::Publisher tf_pub_;
    int queue_size_ = 1000;
    std::string pose_topic_, odom_topic_, pose_link_, cam_link_;
    tf::Transform transform_;
    bool valid_transform;
        tf::TransformListener listener_;	

  public:
    MessageConverter(std::string pose_topic, std::string odom_topic, std::string pose_link, std::string cam_link) :
      pose_topic_(pose_topic),
      odom_topic_(odom_topic),
      pose_link_(pose_link),
      cam_link_(cam_link)
    {}
    
    bool init()
    {
      // sub_ = nh_.subscribe(pose_topic_, queue_size_, MessageConverter::msgCallback);
      sub_.subscribe(nh_, pose_topic_, queue_size_);
      sub_.registerCallback(boost::bind(&MessageConverter::msgCallback, this, _1));
      pub_ = nh_.advertise<nav_msgs::Odometry>(odom_topic_, queue_size_);
      tf_pub_ = nh_.advertise<tf2_msgs::TFMessage>("tf", queue_size_);

      valid_transform = false;
    }

  private:

    void msgCallback(const nav_msgs::OdometryConstPtr& msg)
    {
      // copy from the original message 
      nav_msgs::Odometry odom;
//
      odom.header.stamp = msg->header.stamp;
      odom.header.seq = msg->header.seq;
      odom.header.frame_id = "odom";
      odom.child_frame_id = pose_link_;
 //     odom.child_frame_id = "base_footprint";
      //
    if (valid_transform == false) {
      // look up the transform once
	// query the transform to be applied
	ROS_INFO_STREAM("Looking up transform between " << msg->child_frame_id << " and " << pose_link_);
	//listener_.waitForTransform(msg->child_frame_id, pose_link_, ros::Time(), ros::Duration(1.0)); //"visual/" + pose_link_, odom_link_
	try{
	  //should be msg->child_frame_id, but they don't use the actual frame_id
	  tf::StampedTransform tf_tmp;
	  tf::Transform tf_ib, tf_ci;
	  listener_.lookupTransform("gyro_link", cam_link_, // "camera_rgb_frame", // "camera_rgb_optical_frame", // "camera_left_optical_frame", // 
				      ros::Time(0), tf_tmp);
	  tf_ib = get_tf_from_stamped_tf(tf_tmp);
	  //
	  /*
	  listener_.lookupTransform( "camera_rgb_optical_frame", "camera_rgb_frame", 
				      ros::Time(0), tf_tmp);
	  tf_ci = get_tf_from_stamped_tf(tf_tmp);
	  */
	  //
	  transform_ = tf_ib; // * tf_ci;
	  valid_transform = true;
	  
	  ROS_INFO("found static transform for odom convertion!");
	}
	catch (tf::TransformException ex){
	  ROS_ERROR("%s",ex.what());
	  ros::Duration(1.0).sleep();
	}
    
      }
      else {
	// do nothing
      }
      
      // transform the "mean" values
      transformPose(*msg, odom);
      transformTwist(*msg, odom);
      
      // TODO
      // transform the "cov" values
      // for now a simple linear model is assumed;
      // more rigourous impl can be found at mrpt package: http://wiki.ros.org/pose_cov_ops
      odom.pose.covariance = msg->pose.covariance;
      odom.twist.covariance = msg->twist.covariance;
      
      odom.child_frame_id = "visual/" + pose_link_; //odom_link_.substr(1, odom_link_.size()-1);;
     // odom.pose.pose.position.z=0;
      
      pub_.publish(odom);
      
      // convert to tf
      geometry_msgs::TransformStamped tsf_ = tf_utils::toTransform(odom);
      tf2_msgs::TFMessage tf_;
      tf_.transforms.push_back(tsf_);
      
      tf_pub_.publish(tf_);

    }
    
    
    //some conversion utilities:
//getting a transform from a stamped transform is trickier than expected--there is not "get" fnc for transform
tf::Transform get_tf_from_stamped_tf(tf::StampedTransform sTf) {
   tf::Transform tf(sTf.getBasis(),sTf.getOrigin()); //construct a transform using elements of sTf
   return tf;
}
    
//a function to multiply two stamped transforms;  this function checks to make sure the
// multiplication is logical, e.g.: T_B/A * T_C/B = T_C/A
// returns false if the two frames are inconsistent as sequential transforms
// returns true if consistent A_stf and B_stf transforms, and returns result of multiply in C_stf
// The reference frame and child frame are populated in C_stf accordingly
bool multiply_stamped_tfs(tf::StampedTransform A_stf, 
        tf::StampedTransform B_stf, tf::StampedTransform &C_stf) {
   tf::Transform A,B,C; //simple transforms--not stamped
  std::string str1 (A_stf.child_frame_id_); //want to compare strings to check consistency
  std::string str2 (B_stf.frame_id_);
  if (str1.compare(str2) != 0) { //SHOULD get that child frame of A is parent frame of B
      std::cout<<"can't multiply transforms; mismatched frames"<<std::endl;
    std::cout << str1 << " is not " << str2 << '\n'; 
    return false;
  }
   //if here, the named frames are logically consistent
   A = get_tf_from_stamped_tf(A_stf); // get the transform from the stamped transform
   B = get_tf_from_stamped_tf(B_stf);
   C = A*B; //multiplication is defined for transforms 
   C_stf.frame_id_ = A_stf.frame_id_; //assign appropriate parent and child frames to result
   C_stf.child_frame_id_ = B_stf.child_frame_id_;
   C_stf.setOrigin(C.getOrigin()); //populate the origin and orientation of the result
   C_stf.setBasis(C.getBasis());
   C_stf.stamp_ = ros::Time::now(); //assign the time stamp to current time; 
     // alternatively, could assign this to the OLDER of A or B transforms
   return true; //if got here, the multiplication is valid
}

    
 void transformPose(const nav_msgs::Odometry& msg_in,
     nav_msgs::Odometry& msg_out) const {
      //
      tf::Transform input_pose, output_pose;
      tf::poseMsgToTF(msg_in.pose.pose, input_pose);
      output_pose = transform_ * input_pose;
      //tf::Transformer::transformPose(odom_link_, input_pose, output_pose);
      
      tf::poseTFToMsg(output_pose, msg_out.pose.pose);
    }
    
    
    //
    // NOTE
    // steal from ros hydro tf::transform__listener.cpp
    // http://docs.ros.org/hydro/api/tf/html/c++/transform__listener_8cpp_source.html
    //
    /* http://www.ros.org/wiki/tf/Reviews/2010-03-12_API_Review*/
 void transformTwist(const nav_msgs::Odometry& msg_in,
     nav_msgs::Odometry& msg_out) const {
   //
   tf::Vector3 twist_rot(msg_in.twist.twist.angular.x,
                         msg_in.twist.twist.angular.y,
                         msg_in.twist.twist.angular.z);
   tf::Vector3 twist_vel(msg_in.twist.twist.linear.x,
                         msg_in.twist.twist.linear.y,
                         msg_in.twist.twist.linear.z);
 
   tf::Vector3 out_rot = transform_.getBasis() * twist_rot;
   tf::Vector3 out_vel = transform_.getBasis()* twist_vel + transform_.getOrigin().cross(out_rot);
   
   /*
   geometry_msgs::TwistStamped interframe_twist;
   tf::lookupVelocity(target_frame, msg_in.header.frame_id, msg_in.header.stamp, ros::Duration(0.1), interframe_twist); //\todo get rid of hard coded number
 
   msg_out.header.stamp = msg_in.header.stamp;
   msg_out.header.frame_id = target_frame;
   msg_out.twist.linear.x =  out_vel.x() + interframe_twist.twist.linear.x;
   msg_out.twist.linear.y =  out_vel.y() + interframe_twist.twist.linear.y;
   msg_out.twist.linear.z =  out_vel.z() + interframe_twist.twist.linear.z;
   msg_out.twist.angular.x =  out_rot.x() + interframe_twist.twist.angular.x;
  msg_out.twist.angular.y =  out_rot.y() + interframe_twist.twist.angular.y;
   msg_out.twist.angular.z =  out_rot.z() + interframe_twist.twist.angular.z;
   */
   msg_out.twist.twist.linear.x =  out_vel.x();
   msg_out.twist.twist.linear.y =  out_vel.y();
   msg_out.twist.twist.linear.z =  out_vel.z();
   msg_out.twist.twist.angular.x =  out_rot.x();
   msg_out.twist.twist.angular.y =  out_rot.y();
   msg_out.twist.twist.angular.z =  out_rot.z();
   
   }

   
   /*
   void transformTwistCov(const std::string& target_frame,
     const nav_msgs::Odometry& msg_in,
     nav_msgs::Odometry& msg_out) const {
       //
   tf::Matrix3x3 Tb  = transform_.getBasis();
   tf::Matrix3x3 To; 
   skew33( transform_.getOrigin(), To );
   
   jacobian = [];
   
   Tb * Cov_in * Tb.transpose();
   

   tf::Vector3 twist_rot(msg_in.twist.angular.x,
                         msg_in.twist.angular.y,
                         msg_in.twist.angular.z);
   tf::Vector3 twist_vel(msg_in.twist.linear.x,
                         msg_in.twist.linear.y,
                         msg_in.twist.linear.z);
 
   tf::Vector3 out_rot = transform_.getBasis() * twist_rot;
   tf::Vector3 out_vel = transform_.getBasis() * twist_vel + transform_.getOrigin().cross(out_rot);
 
   msg_out.header.stamp = msg_in.header.stamp;
   msg_out.twist.linear.x =  out_vel.x();
   msg_out.twist.linear.y =  out_vel.y();
   msg_out.twist.linear.z =  out_vel.z();
   msg_out.twist.angular.x =  out_rot.x();
   msg_out.twist.angular.y =  out_rot.y();
   msg_out.twist.angular.z =  out_rot.z();
   
   }

   
   
   // refer to https://en.wikipedia.org/wiki/Skew-symmetric_matrix#Cross_product
   void skew33(const tf::Vector3 & v, tf::Matrix3x3 & m) {
     m[0, 0] = 0;
     m[0, 1] = -v.getZ();
     m[0, 2] = v.getY();
     //
      m[1, 0] = v.getZ();
     m[1, 1] = 0;
     m[1, 2] = -v.getX();
     //
      m[2, 0] = -v.getY();
     m[2, 1] = v.getX();
     m[2, 2] = 0;
  }
   
   */
   
};

#endif
