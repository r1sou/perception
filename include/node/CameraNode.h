#pragma once 

#include "node/NodeType.h"

#include "model/base/type.hpp"

class CameraNode : public rclcpp::Node
{
public:
    CameraNode(std::string node_name) : Node(node_name)
    {
    }
    ~CameraNode()
    {
        stop();
    }
public:
    void configuration_camera(nlohmann::json &camera_config);
    void DualImageCallback(const sensor_msgs::msg::Image::SharedPtr msg1, const sensor_msgs::msg::Image::SharedPtr msg2);
    void SingleImageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void configuration_laser(nlohmann::json &laser_config);
    void configuration_client(nlohmann::json &client_config);
    void configuration_record(nlohmann::json &record_config);
    void start(){
        {
            websocket_client_->start();
        }
        // udp send
        worker_threads_.emplace_back(
            std::make_shared<std::thread>(
                [this](){
                    rclcpp::WallRate rate(publish_laserscan_fps_);
                    while(rclcpp::ok()){
                        publish_laserscan();
                        rate.sleep();
                    }
                }
            )
        );
        worker_threads_.emplace_back(
            std::make_shared<std::thread>(
                [this](){
                    rclcpp::WallRate rate(10);
                    while(rclcpp::ok()){
                        publish_camera_info();
                        rate.sleep();
                    }
                }
            )
        );
        // worker_threads_.emplace_back(
        //     std::make_shared<std::thread>(
        //         [this](){
        //             rclcpp::WallRate rate(10);
        //             while(rclcpp::ok()){
        //                 publish_image();
        //                 rate.sleep();
        //             }
        //         }
        //     )
        // );
        // websocket send
        worker_threads_.emplace_back(
            std::make_shared<std::thread>(
                [this](){
                    rclcpp::WallRate rate(1.0 / 2);
                    while(rclcpp::ok()){
                        send_camera_status();
                        rate.sleep();
                    }
                }
            )
        );
        worker_threads_.emplace_back(
            std::make_shared<std::thread>(
                [this](){
                    rclcpp::WallRate rate(10);
                    while(rclcpp::ok()){
                        record_thread();
                        rate.sleep();
                    }
                }
            )
        );
        // if(camera_config_.is_master){
        //     worker_threads_.emplace_back(
        //         std::make_shared<std::thread>(
        //             [this](){
        //                 rclcpp::WallRate rate(10);
        //                 while(rclcpp::ok()){
        //                     followme_thread();
        //                     rate.sleep();
        //                 }
        //             }
        //         )
        //     );
        // }
    }
    void send_camera_status();
    bool infer_common_process(std::shared_ptr<InferenceData_t> infer_data, std::string buffer_name);
    void record_thread();
    // void followme_thread();
    void publish_laserscan();
    void publish_camera_info();
    void publish_image();
    void stop(){
        for (auto &thread_ : worker_threads_)
        {
            if (thread_->joinable())
            {
                thread_->join();
            }
        }
        if(video_writer_){
            video_writer_->release();
            RCLCPP_INFO_STREAM(this->get_logger(), fmt::format("stop record, save path: {}, total frame: {}", save_path_, frame_count_).c_str());
        }
    }
public:
    int publish_laserscan_fps_ = 5;
    CameraConfig camera_config_;
    LaserConfig laser_config_;
    RecordConfig record_config_;
public:
    std::shared_ptr<WebSocketClient> websocket_client_;
    std::shared_ptr<UDPClient> udp_client_;
public:
    nlohmann::json laserscan_message_, camera_info_message_, image_message_;

public:
    std::map<std::string, std::shared_ptr<TripletBuffer<sensor_msgs::msg::Image::SharedPtr>>> buffers_;
private:
    // 深度或者双目
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> sub1_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> sub2_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> syncApproximate_;
private:
    // 单目
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
private:
    std::vector<std::shared_ptr<std::thread>> worker_threads_;
public:
    bool record_ = false, followme_ = false, show_ = false;
    std::atomic<bool> is_record_running_;
    std::atomic<bool> is_followme_running_;
    std::atomic<bool> is_recognize_running_;
    std::atomic<bool> is_obstacle_running_;
private:
    int frame_count_ = 0;
    std::string save_path_;
    std::shared_ptr<cv::VideoWriter> video_writer_;
private:
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::LaserScan>> laser_scan_pub_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> point_cloud_pub_;
};