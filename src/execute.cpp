#include <cmath>
#include <ros/ros.h>
#include "tf/transform_broadcaster.h"
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/Marker.h>

class EstimateDiaboloStateNode
{
public:
  EstimateDiaboloStateNode(bool debug_print=false)
    : debug_print_(debug_print),
      nh_(""), pnh_("~"),
      pitch_(0), yaw_(0),
      min_cube_x_(0.4), max_cube_x_(1.0),
      min_cube_y_(-0.2), max_cube_y_(0.2),
      min_cube_z_(0.2), max_cube_z_(0.7)

  {
    // rosparam
    pnh_.getParam("topic_name", topic_name_);
    pnh_.getParam("frame_id", frame_id_);

    pnh_.getParam("min_cube_x", min_cube_x_);
    pnh_.getParam("max_cube_x", max_cube_x_);

    pnh_.getParam("min_cube_y", min_cube_y_);
    pnh_.getParam("max_cube_y", max_cube_y_);

    pnh_.getParam("min_cube_z", min_cube_z_);
    pnh_.getParam("max_cube_z", max_cube_z_);

    // subscriber
    sub_pointcloud_ = nh_.subscribe(topic_name_, 1, &EstimateDiaboloStateNode::estimateCallback, this);

    // publisher
    pub_diabolo_points_ = pnh_.advertise<sensor_msgs::PointCloud2>("diabolo_pointcloud", 1);
    pub_diabolo_state_ = pnh_.advertise<std_msgs::Float64MultiArray>("diabolo_state", 1);

    // publisher for marker
    pub_diabolo_state_marker_ = pnh_.advertise<visualization_msgs::Marker>("diabolo_state_marker", 1);
    pub_diabolo_cube_marker_ = pnh_.advertise<visualization_msgs::Marker>("diabolo_cube_marker", 1);
    pub_diabolo_plane_marker_ = pnh_.advertise<visualization_msgs::Marker>("diabolo_plane_marker", 1);

    // init marker
    initMarker();
  }

private:
  void initMarker() {
    // marker for diabolo state
    diabolo_state_marker_.header.frame_id = frame_id_;
    diabolo_state_marker_.ns = "diabolo_state_marker";
    diabolo_state_marker_.id = 0;
    diabolo_state_marker_.type = visualization_msgs::Marker::CYLINDER;
    diabolo_state_marker_.action = visualization_msgs::Marker::ADD;
    diabolo_state_marker_.scale.x = 0.02;
    diabolo_state_marker_.scale.y = 0.02;
    diabolo_state_marker_.scale.z = 0.5;
    diabolo_state_marker_.color.r = 0.0f;
    diabolo_state_marker_.color.g = 1.0f;
    diabolo_state_marker_.color.b = 0.0f;
    diabolo_state_marker_.color.a = 0.5;
    diabolo_state_marker_.lifetime = ros::Duration();

    // marker for diabolo cube
    diabolo_cube_marker_.header.frame_id = frame_id_;
    diabolo_cube_marker_.ns = "diabolo_cube_marker";
    diabolo_cube_marker_.id = 1;
    diabolo_cube_marker_.type = visualization_msgs::Marker::CUBE;
    diabolo_cube_marker_.action = visualization_msgs::Marker::ADD;
    diabolo_cube_marker_.pose.position.x = (max_cube_x_ + min_cube_x_) / 2.0;
    diabolo_cube_marker_.pose.position.y = (max_cube_y_ + min_cube_y_) / 2.0;
    diabolo_cube_marker_.pose.position.z = (max_cube_z_ + min_cube_z_) / 2.0;
    diabolo_cube_marker_.scale.x = max_cube_x_ - min_cube_x_;
    diabolo_cube_marker_.scale.y = max_cube_y_ - min_cube_y_;
    diabolo_cube_marker_.scale.z = max_cube_z_ - min_cube_z_;
    diabolo_cube_marker_.color.r = 1.0f;
    diabolo_cube_marker_.color.g = 0.2f;
    diabolo_cube_marker_.color.b = 0.0f;
    diabolo_cube_marker_.color.a = 0.2;
    diabolo_cube_marker_.lifetime = ros::Duration();

    // marker for diabolo plane
    diabolo_plane_marker_.header.frame_id = frame_id_;
    //diabolo_plane_marker_.header.stamp = ros::Time::now();
    diabolo_plane_marker_.ns = "diabolo_plane_marker";
    diabolo_plane_marker_.id = 0;
    diabolo_plane_marker_.type = visualization_msgs::Marker::CUBE;
    diabolo_plane_marker_.action = visualization_msgs::Marker::ADD;
    diabolo_plane_marker_.scale.x = 0.01;
    diabolo_plane_marker_.scale.y = 0.5;
    diabolo_plane_marker_.scale.z = 0.5;
    diabolo_plane_marker_.color.r = 0.0f;
    diabolo_plane_marker_.color.g = 0.0f;
    diabolo_plane_marker_.color.b = 1.0f;
    diabolo_plane_marker_.color.a = 0.2;
    diabolo_plane_marker_.lifetime = ros::Duration();
  }

  void estimateCallback(const sensor_msgs::PointCloud2::ConstPtr& msg_pointcloud)
  {
    // transform rosmsg to pointcloud
    pcl::PointCloud<pcl::PointXYZ> pointcloud;
    pcl::fromROSMsg(*msg_pointcloud, pointcloud);

    // calculate key variables
    double max_diabolo_x = -10000;
    double min_diabolo_x = 10000;
    double sum_diabolo_x = 0, sum_diabolo_y = 0, sum_diabolo_z = 0, sum_diabolo_xy = 0, sum_diabolo_xx = 0;
    pcl::PointCloud<pcl::PointXYZ> diabolo_points;
    int diabolo_points_num = 0;
    for (pcl::PointCloud<pcl::PointXYZ>::iterator p = pointcloud.points.begin(); p != pointcloud.points.end(); *p++) {
      if (min_cube_x_ < p->x and p->x < max_cube_x_ and min_cube_y_ < p->y and p->y < max_cube_y_ and min_cube_z_ < p->z and p->z < max_cube_z_) { // if point is in cube
        max_diabolo_x = (max_diabolo_x > p->x) ? max_diabolo_x : p->x;
        min_diabolo_x = (min_diabolo_x < p->x) ? min_diabolo_x : p->x;

        sum_diabolo_x += p->x;
        sum_diabolo_y += p->y;
        sum_diabolo_z += p->z;
        sum_diabolo_xy += p->x * p->y;
        sum_diabolo_xx += p->x * p->x;
        diabolo_points_num++;

        diabolo_points.push_back(pcl::PointXYZ(p->x, p->y, p->z));
      }
    }

    { // publish diabolo points
      sensor_msgs::PointCloud2 msg_diabolo_points;
      pcl::toROSMsg(diabolo_points, msg_diabolo_points);
      msg_diabolo_points.header.frame_id = frame_id_;
      //msg_diabolo_points.header.stamp = ros::Time::now();
      pub_diabolo_points_.publish(msg_diabolo_points);
    }

    // calculate key variables
    double max_diabolo_x_front = 0, max_diabolo_z_front = 0;
    double max_diabolo_x_back , max_diabolo_z_back = 0;
    double mid_diabolo_x = (max_diabolo_x + min_diabolo_x) / 2.;
    for (pcl::PointCloud<pcl::PointXYZ>::iterator p = pointcloud.points.begin(); p != pointcloud.points.end(); *p++) {
      if (p->x < max_cube_x_ and p->x > min_cube_x_ and p->y > min_cube_y_ and p->y < max_cube_y_ and p->z > min_cube_z_ and p->z < max_cube_z_) {
        if (p->x > mid_diabolo_x) {
          if (max_diabolo_z_back < p->z) {
            max_diabolo_z_back = p->z;
            max_diabolo_x_back = p->x;
          }
        } else {
          if (max_diabolo_z_front < p->z) {
            max_diabolo_z_front = p->z;
            max_diabolo_x_front = p->x;
          }
        }
      }
    }

    { // publish diabolo state
      std_msgs::Float64MultiArray msg_diabolo_state;

      // pitch
      double tmp_pitch = std::atan2(max_diabolo_z_back - max_diabolo_z_front, max_diabolo_x_back - max_diabolo_x_front) / 3.14 * 180;
      if (not std::isnan(tmp_pitch)) {
        pitch_ = tmp_pitch;
      }
      msg_diabolo_state.data.push_back(pitch_);

      // yaw
      double tmp_yaw = std::atan2((diabolo_points_num * sum_diabolo_xy - sum_diabolo_x * sum_diabolo_y), (diabolo_points_num * sum_diabolo_xx - sum_diabolo_x * sum_diabolo_x)) / 3.14 * 180;
      if (not std::isnan(tmp_yaw)) {
        yaw_ = tmp_yaw;
      }
      msg_diabolo_state.data.push_back(yaw_);

      // publish
      pub_diabolo_state_.publish(msg_diabolo_state);
      if (debug_print_) { std::cout << "[pitch] " << pitch_ << " [yaw] " << yaw_ << std::endl; }
    }

    { // publish diabolo state marker
      diabolo_state_marker_.header.frame_id = frame_id_;
      //diabolo_state_marker_.header.stamp = ros::Time::now();
      diabolo_state_marker_.pose.position.x = sum_diabolo_x / diabolo_points_num;
      diabolo_state_marker_.pose.position.y = sum_diabolo_y / diabolo_points_num;
      diabolo_state_marker_.pose.position.z = sum_diabolo_z / diabolo_points_num;
      tf::Quaternion diabolo_state_quat = tf::createQuaternionFromRPY(0, -pitch_ * 3.14 / 180 + 1.57, yaw_ * 3.14 / 180);
      quaternionTFToMsg(diabolo_state_quat, diabolo_state_marker_.pose.orientation);
      pub_diabolo_state_marker_.publish(diabolo_state_marker_);
    }

    { // publish diabolo cube marker
      diabolo_cube_marker_.header.frame_id = frame_id_;
      //diabolo_cube_marker_.header.stamp = ros::Time::now();
      pub_diabolo_cube_marker_.publish(diabolo_cube_marker_);
    }

    { // publish diabolo plane marker
      diabolo_plane_marker_.header.frame_id = frame_id_;
      //diabolo_plane_marker_.header.stamp = ros::Time::now();
      diabolo_plane_marker_.pose.position.x = mid_diabolo_x;
      diabolo_plane_marker_.pose.position.y = sum_diabolo_y / diabolo_points_num;
      diabolo_plane_marker_.pose.position.z = sum_diabolo_z / diabolo_points_num;
      pub_diabolo_plane_marker_.publish(diabolo_plane_marker_);
    }
  }

  std::string topic_name_, frame_id_;
  bool debug_print_;

  // ros params
  ros::NodeHandle nh_, pnh_;

  ros::Subscriber sub_pointcloud_;

  ros::Publisher pub_diabolo_state_, pub_diabolo_points_;
  ros::Publisher pub_diabolo_state_marker_, pub_diabolo_cube_marker_, pub_diabolo_plane_marker_;

  visualization_msgs::Marker diabolo_state_marker_, diabolo_cube_marker_, diabolo_plane_marker_;

  // cube params
  double min_cube_x_, max_cube_x_;
  double min_cube_y_, max_cube_y_;
  double min_cube_z_, max_cube_z_;

  // diabolo state
  double pitch_, yaw_;
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "estimate_diabolo_state");

  EstimateDiaboloStateNode n;
  ros::spin();

  return 0;
}
