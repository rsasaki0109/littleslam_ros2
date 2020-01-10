#if !defined LITTLESLAM_ROS2_NODE_HPP_
#define LITTLESLAM_ROS2_NODE_HPP_

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define LITTLESLAM_EXPORT __attribute__ ((dllexport))
    #define LITTLESLAM_IMPORT __attribute__ ((dllimport))
  #else
    #define LITTLESLAM_EXPORT __declspec(dllexport)
    #define LITTLESLAM_IMPORT __declspec(dllimport)
  #endif
  #ifdef LITTLESLAM_BUILDING_DLL
    #define LITTLESLAM_PUBLIC LITTLESLAM_EXPORT
  #else
    #define LITTLESLAM_PUBLIC LITTLESLAM_IMPORT
  #endif
  #define LITTLESLAM_PUBLIC_TYPE LITTLESLAM_PUBLIC
  #define LITTLESLAM_LOCAL
#else
  #define LITTLESLAM_EXPORT __attribute__ ((visibility("default")))
  #define LITTLESLAM_IMPORT
  #if __GNUC__ >= 4
    #define LITTLESLAM_PUBLIC __attribute__ ((visibility("default")))
    #define LITTLESLAM_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define LITTLESLAM_PUBLIC
    #define LITTLESLAM_LOCAL
  #endif
  #define LITTLESLAM_PUBLIC_TYPE
#endif

#if __cplusplus
} // extern "C"
#endif

#include <rclcpp/rclcpp.hpp>

#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/path.hpp"

#include "tf2/impl/utils.h"
#include "tf2/convert.h"
#include "tf2/impl/convert.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "SlamFrontEnd.h"
#include "FrameworkCustomizer.h"

namespace littleslam_ros2
{

class Littleslam : public rclcpp::Node
{
public:
  LITTLESLAM_PUBLIC Littleslam();

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr icp_map_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  SlamFrontEnd *sf_ = new SlamFrontEnd();
  FrameworkCustomizer fc_;

  bool make_scan2d(Scan2D &scan2d, const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);
  void broadcast_littleslam();

  PointCloudMap *map_;
  
  bool use_odom_; 
};

} // namespace littleslam_ros2

#endif // LITTLESLAM_ROS2_STYLE__LITTLESLAM_NODE_HPP_
