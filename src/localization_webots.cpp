#include <string>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <sensor_msgs/MagneticField.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

class LocalizationWebots {
private:
  ros::NodeHandle nh_;
  ros::Subscriber gps_sub_;
  ros::Subscriber compass_sub_;
  ros::Publisher pose_pub_;
  tf2_ros::TransformBroadcaster tf_pub_;

  bool publish_tf_;
  std::string tf_prefix_;
  std::string localization_type_;
  geometry_msgs::Pose curr_pose_;
  geometry_msgs::PointStamped curr_gps_;
  sensor_msgs::MagneticField curr_compass_;
  float publish_rate_;

  void gpsCallback(const geometry_msgs::PointStamped &msg) {
    curr_gps_ = msg;
    // ROS_INFO("got gps %6.3f %6.3f %6.3f", msg.point.x, msg.point.y, msg.point.z);
  }

  void compassCallback(const sensor_msgs::MagneticField &msg) {
    curr_compass_ = msg;
    // ROS_INFO("got compass %6.3f %6.3f %6.3f", msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z);
  }

public:
  LocalizationWebots(ros::NodeHandle nh) : nh_(nh) {
    ros::param::param<std::string>("~localization_type", localization_type_, "gps");
    ros::param::param<std::string>("~tf_prefix", tf_prefix_, "");
    ros::param::param<float>("~publish_rate", publish_rate_, 60);
    ros::param::param<bool>("~publish_tf", publish_tf_, true);
    ROS_INFO("tf_prefix: %s", tf_prefix_.c_str());
    ROS_INFO("publish_rate: %.3f", publish_rate_);
    ROS_INFO("localization_type: %s", localization_type_.c_str());

    if (localization_type_ == "gps") {
      gps_sub_ = nh_.subscribe("gps/values", 1, &LocalizationWebots::gpsCallback, this);
      compass_sub_ = nh_.subscribe("compass/values", 1, &LocalizationWebots::compassCallback, this);
    } else if (localization_type_ == "supervisor") {
      // TODO
    }

    pose_pub_ = nh_.advertise<geometry_msgs::Pose>("pose", 1);
  }

  void run() {
    while (ros::ok()) {
      curr_pose_.position = curr_gps_.point;

      double yaw = atan2(curr_compass_.magnetic_field.y, curr_compass_.magnetic_field.x);
      if (yaw < 0.0)
        yaw += M_PI * 2;
      tf2::Quaternion offset, origin;
      origin.setEuler(0, 0, yaw);
      offset.setRotation(tf2::Vector3(0, 0, 1), -1.57);
      tf2::Quaternion quat = origin * offset;  // offset for compass rotation

      curr_pose_.orientation.x = quat.getX();
      curr_pose_.orientation.y = quat.getY();
      curr_pose_.orientation.z = quat.getZ();
      curr_pose_.orientation.w = quat.getW();

      pose_pub_.publish(curr_pose_);

      if (publish_tf_) {
        geometry_msgs::TransformStamped transform_stamped;
        transform_stamped.header.stamp = ros::Time::now();
        transform_stamped.header.frame_id = "/world";
        transform_stamped.child_frame_id = tf_prefix_ + "/base_link";
        transform_stamped.transform.translation.x = curr_pose_.position.x;
        transform_stamped.transform.translation.y = curr_pose_.position.y;
        transform_stamped.transform.translation.z = curr_pose_.position.z;
        transform_stamped.transform.rotation.x = quat.getX();
        transform_stamped.transform.rotation.y = quat.getY();
        transform_stamped.transform.rotation.z = quat.getZ();
        transform_stamped.transform.rotation.w = quat.getW();
        tf_pub_.sendTransform(transform_stamped);
      }

      ros::Rate(publish_rate_).sleep();
      ros::spinOnce();
    }
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "localization_webots");
  ros::NodeHandle nh;

  LocalizationWebots localization(nh);
  localization.run();

  return 0;
}
