#pragma once 

#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.h"

#include "tf2_ros/buffer.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "sensor_msgs/msg/point_field.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>;

#include "common/buffer.hpp"
#include "common/client.hpp"
#include "common/threadpool.h"
#include "common/image_conversion.hpp"

enum CameraType
{
    MONO,
    DEPTH,
    STEREO
};

struct CameraConfig
{
    CameraType camera_type;
    std::string name;
    int device_id;
    float fx = 0.0, fy = 0.0, cx = 0.0, cy = 0.0, baseline = 0.0;
    int image_width = 0, image_height = 0;
    std::vector<double> arr_d, arr_k, arr_r, arr_p;
    bool is_master = false;
    std::string frame_id;
};

struct LaserConfig
{
    int down_sample_ratio = 1;
    float tolerance = 0.01;
    float min_height = 0.0;
    float max_height = 1.0;
    float angle_min = -1.5707963267948966;
    float angle_max = 1.5707963267948966;
    float angle_increment = 0.00315;
    float time_increment = 0.0;
    float scan_time = 0.1;
    float range_min = 0.05;
    float range_max = 150.0;
    bool use_inf = true;
    float inf_epsilon = 1.0;
};

struct RecordConfig
{
    int image_width = 0, image_height = 0;
    int fps = 0;
    std::string save_dir;
    std::string suffix;
    int fourcc = 0;
};
