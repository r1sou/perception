#pragma once

#include "node/CameraNode.h"

#include "model/engine.hpp"
#include "task/followme.h"
#include "task/obstacle.h"

class PerceptionNode : public rclcpp::Node
{
public:
    PerceptionNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) : Node("PerceptionNode", options){
        parse_declare_parameter();
        configuration();
    }
    ~PerceptionNode(){
        stop();
    }
public:
    void parse_declare_parameter(){
        this->declare_parameter("project_root", "");
        this->get_parameter("project_root", project_root_);
        RCLCPP_INFO(this->get_logger(), "project root: %s", project_root_.c_str());

        this->declare_parameter("camera_config_path", "");
        this->get_parameter("camera_config_path", camera_config_path_);
        RCLCPP_INFO_STREAM(this->get_logger(), "camera_config_path: " << camera_config_path_);

        this->declare_parameter("laser_config_path", "");
        this->get_parameter("laser_config_path", laser_config_path_);
        RCLCPP_INFO_STREAM(this->get_logger(), "laser_config_path: " << laser_config_path_);

        this->declare_parameter("client_config_path", "");
        this->get_parameter("client_config_path", client_config_path_);
        RCLCPP_INFO_STREAM(this->get_logger(), "client_config_path: " << client_config_path_);

        this->declare_parameter("model_config_path", "");
        this->get_parameter("model_config_path", model_config_path_);
        RCLCPP_INFO_STREAM(this->get_logger(), "model_config_path: " << model_config_path_);

        this->declare_parameter("record_config_path", "");
        this->get_parameter("record_config_path", record_config_path_);
        RCLCPP_INFO_STREAM(this->get_logger(), "record_config_path: " << record_config_path_);

        this->declare_parameter("show", false);
        this->get_parameter("show", show_);
        RCLCPP_INFO_STREAM(this->get_logger(), "show: " << show_);

        this->declare_parameter("debug", false);
        this->get_parameter("debug", debug_);
        RCLCPP_INFO_STREAM(this->get_logger(), "debug: " << debug_);

        this->declare_parameter("record", false);
        this->get_parameter("record", record_);
        RCLCPP_INFO_STREAM(this->get_logger(), "record: " << record_);

        this->declare_parameter("followme", false);
        this->get_parameter("followme", followme_);
        RCLCPP_INFO_STREAM(this->get_logger(), "followme: " << followme_);
    }
    void configuration(){
        {
            int ret;
            ret = std::system(
                fmt::format("python {}/scripts/camera.py --file {}/config/camera.json", project_root_, project_root_).c_str()
            ); // update camera config
            if (ret != 0)
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "\33[31mpython camera.py failed\33[0m");
            }
            ret = std::system(
                fmt::format("python {}/scripts/client.py --file {}/config/client.json", project_root_, project_root_).c_str()
            ); // auto find client
            if (ret != 0)
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "\33[31mpython client.py failed\33[0m");
            }
        }
        {
            camera_config_ = nlohmann::json::parse(std::ifstream(camera_config_path_));
            laser_config_ = nlohmann::json::parse(std::ifstream(laser_config_path_));
            client_config_ = nlohmann::json::parse(std::ifstream(client_config_path_));
            model_config_ = nlohmann::json::parse(std::ifstream(model_config_path_));
            record_config_ = nlohmann::json::parse(std::ifstream(record_config_path_));
        }
        {
            configuration_camera();
            configuration_client();
            configuration_task();
        }
    }
    void configuration_camera(){
        for(auto camera_config : camera_config_["cameras"]){
            auto camera_node = std::make_shared<CameraNode>(camera_config["name"].get<std::string>() + "_node");
            camera_node->configuration_camera(camera_config);
            camera_node->configuration_laser(laser_config_);
            camera_node->configuration_record(record_config_);
            camera_node->record_ = record_;
            camera_nodes_.push_back(camera_node);
        }
        RCLCPP_INFO_STREAM(this->get_logger(), "config camera Finish");
    }
    void configuration_client(){
        auto &config = client_config_["client"];
        {
            std::string uri = fmt::format("ws://{}:{}", config["ip"].get<std::string>(), config["WebSocket"]["port"].get<int>());
            websocket_client_ = std::make_shared<WebSocketClient>(uri, client_config_);
        }
        {
            udp_client_ = std::make_shared<UDPClient>(config["ip"].get<std::string>(), config["UDP"]["port"].get<int>());
        }
        for(auto &camera_node : camera_nodes_){
            camera_node->configuration_client(client_config_);
        }
        RCLCPP_INFO_STREAM(this->get_logger(), "config client Finish");
    }
    void configuration_task(){
        {
            engine_ = std::make_shared<Engine>();
            engine_->configuration(model_config_, project_root_);
        }
        {
            followme_task_ = std::make_shared<FollowMe>();
            followme_task_->init_task();
        }
        {
            obstacle_task_ = std::make_shared<Obstacle>();
            obstacle_task_->init_task();
        }
        RCLCPP_INFO_STREAM(this->get_logger(), "config task Finish");
    }
    void followme_thread(std::shared_ptr<CameraNode> camera_node){
        if(camera_node->websocket_client_->connected.load()){
            if((camera_node->websocket_client_->start_follow.load() && !camera_node->is_followme_running_.load()) || followme_){
                camera_node->is_followme_running_.store(true);
                if(!followme_){
                    if(camera_node->is_obstacle_running_.load()){
                        camera_node->is_obstacle_running_.store(false);
                        std::this_thread::sleep_for(std::chrono::milliseconds(200));
                    }
                    RCLCPP_INFO_STREAM(this->get_logger(), "start followme");
                }
            }
            if((!camera_node->websocket_client_->start_follow.load() && camera_node->is_followme_running_.load()) && !followme_){
                camera_node->is_followme_running_.store(false);
                engine_->reset_track();
                followme_task_->reset();
                if(!followme_){
                    RCLCPP_INFO_STREAM(this->get_logger(), "stop followme");
                    {
                        time_t timestamp = time(NULL);
                        std::filesystem::create_directory("/home/sunrise/Desktop/trajactory");
                        std::ofstream ofs(fmt::format("/home/sunrise/Desktop/trajactory/{}.txt", timestamp));
                        for(auto &trajactory : trajactory_){
                            ofs << trajactory[0] << " " << trajactory[1] << std::endl;
                        }
                    }
                    trajactory_.clear();
                    last_trajactory_ = 0.0;
                }
            }
            if(!camera_node->is_followme_running_.load()){
                cv::destroyAllWindows();
                return;
            }
            auto infer_data = std::make_shared<InferenceData_t>();
            if(!camera_node->infer_common_process(infer_data,"followme_buffer")){
                return;
            }
            {
                ScopeTimer t("follow me inference");
                followme_task_->run(infer_data, engine_);
            }
            {
                if(show_){
                    // track
                    if(infer_data->output.track_output.bboxes.size() > 0){
                        auto &track_output = infer_data->output.track_output;
                        image_render::draw_box(infer_data->input.render_image, track_output.bboxes, track_output.track_names);
                    }
                    // detect
                    else{
                        for(auto &detect_output: infer_data->output.detect_output){
                            image_render::draw_box(infer_data->input.render_image, detect_output.bboxes, detect_output.names);
                        }
                    }
                }
                if(show_){
                    cv::imshow("inference", infer_data->input.render_image);
                    cv::waitKey(1);
                }
            }
            {
                publish_thread_pool_.thread_pool_->detach_task(
                    [this, infer_data, camera_node](){
                        publish_followme_target(infer_data, camera_node);
                    }
                );
                // publish_followme_target(infer_data, camera_node);
            }
            {
                publish_thread_pool_.thread_pool_->detach_task(
                    [this, infer_data, camera_node](){
                        publish_obstacle_target(infer_data, camera_node);
                    }
                );
            }
        }
    }
    void publish_followme_target(std::shared_ptr<InferenceData_t> infer_data, std::shared_ptr<CameraNode> camera_node){
        if(!websocket_client_->connected.load()){
            RCLCPP_ERROR_STREAM(this->get_logger(), "websocket is not connected cannot publish followme target");
            return;
        }
        nlohmann::json message;
        auto &config = camera_node->camera_config_;
        {
            message["cmd_code"] = 0x12;
            message["device_id"] = config.device_id;
            time_t timestamp = time(NULL);
            message["time_stamp"] = timestamp;
            message["key"] = JWTGenerator::generate(client_config_["client"]["JWT"]["req_id"], client_config_["client"]["JWT"]["key"]);
        }
        {
            auto data = nlohmann::json::array();
            for(int i = 0; i < infer_data->output.track_output.bboxes.size(); i++){
                if(infer_data->output.track_output.track_ids[i] != followme_task_->target_id){
                    continue;
                }
                auto &track_output = infer_data->output.track_output;
                int box_center_x = static_cast<int>(track_output.bboxes[i][0] + (track_output.bboxes[i][2] - track_output.bboxes[i][0]) / 2.0f);
                int box_center_y = static_cast<int>(track_output.bboxes[i][1] + (track_output.bboxes[i][3] - track_output.bboxes[i][1]) / 2.0f);
                int box_w = static_cast<int>(track_output.bboxes[i][2] - track_output.bboxes[i][0]);
                int box_h = static_cast<int>(track_output.bboxes[i][3] - track_output.bboxes[i][1]);

                float X;
                if(infer_data->input.image_type == INPUT_IMAGE_TYPE::U16C1){
                    X = static_cast<double>(infer_data->input.images[1].at<uint16_t>(box_center_y, box_center_x)) / 1000.0;
                }
                else{
                    X = config.fx * config.baseline / infer_data->output.stereo_output.disparity.at<float>(box_center_y, box_center_x);
                }
                {
                    if(X <= 0.1 && last_trajactory_ != 0.0){
                        X = last_trajactory_;
                    }
                    else{
                        last_trajactory_ = X;
                    }
                }

                float Y = (config.cx - box_center_x) * X / config.fx;
                float Z = (config.cy - box_center_y) * X / config.fy;

                float H = 1.0 * box_h * X / config.fy;
                float W = 1.0 * box_w * X / config.fx;

                Z = 0.1, H = 0.1, W = 0.1;

                data.push_back(
                    {{"name", "person"},
                    {"obj_type", model_config_["detect"]["person"]["obj_type"].get<int>()},
                    {"obj_code", model_config_["detect"]["person"]["obj_code"].get<int>()},
                    {"loc", fmt::format("{:.2f},{:.2f},{:2f}", X, Y, Z)},
                    {"size", fmt::format("{:.2f},{:.2f}", W, H)}}
                );

                trajactory_.push_back({X, Y});
                break;
            }
            message["data"] = data;
        }
        if(message["data"].size() > 0){
            websocket_client_->send_message(message.dump());
        }
    }
    void obstacle_thread(std::shared_ptr<CameraNode> camera_node){
        if(camera_node->camera_config_.camera_type != CameraType::STEREO){
            RCLCPP_ERROR_STREAM(this->get_logger(), "master camera is not stereo, can not launch obstacle thread!!!");
            return;
        }
        if(camera_node->websocket_client_->connected.load()){
            if(camera_node->is_followme_running_.load() || camera_node->is_recognize_running_.load()){
                camera_node->is_obstacle_running_.store(false);
            }
            else{
                camera_node->is_obstacle_running_.store(true);
            }
            // if((websocket_client_->start_obstacle.load() && !is_obstacle_running_.load()) || obstacle_){
            //     is_obstacle_running_.store(true);
            //     if(!obstacle_){
            //         if(is_followme_running_.load()){
            //             is_followme_running_.store(false);
            //         }
            //         std::this_thread::sleep_for(std::chrono::milliseconds(500));
            //         RCLCPP_INFO_STREAM(this->get_logger(), "start obstacle");
            //     }
            // }
            // if((!websocket_client_->start_obstacle.load() && is_obstacle_running_.load()) && !obstacle_){
            //     is_obstacle_running_.store(false);
            //     engine_->reset_track();
            //     obstacle_task_->reset();
            //     if(!obstacle_){
            //         RCLCPP_INFO_STREAM(this->get_logger(), "stop obstacle");
            //     }
            // }
            // if(obstacle_){
            //     is_obstacle_running_.store(true);
            // }
            if(!camera_node->is_obstacle_running_.load()){
                return;
            }
            auto infer_data = std::make_shared<InferenceData_t>();
            if(!camera_node->infer_common_process(infer_data,"obstacle_buffer")){
                return;
            }
            {
                ScopeTimer t("obstacle inference");
                obstacle_task_->run(infer_data, engine_);
            }
            {
                publish_thread_pool_.thread_pool_->detach_task(
                    [this, infer_data, camera_node](){
                        publish_obstacle_target(infer_data, camera_node);
                    }
                );
            }
        }
    }
    void publish_obstacle_target(std::shared_ptr<InferenceData_t> infer_data, std::shared_ptr<CameraNode> camera_node){
        if(infer_data->output.stereo_output.disparity.empty()){
            return;
        }

        auto laserscan_message = camera_node->laserscan_message_;
        {
            time_t timestamp = time(NULL);
            laserscan_message["time_stamp"] = timestamp;
        }

        auto &camera_config = camera_node->camera_config_;
        auto &laser_config = camera_node->laser_config_;
        int down_sample_ratio = laser_config.down_sample_ratio;

        auto &disparity = infer_data->output.stereo_output.disparity;

        float fx       = camera_config.fx;
        float fy       = camera_config.fy;
        float cx       = camera_config.cx;
        float cy       = camera_config.cy;
        float baseline = camera_config.baseline;

        fx = disparity.rows / infer_data->input.image_H * fx;
        fy = disparity.cols / infer_data->input.image_W * fy;
        cx = disparity.rows / infer_data->input.image_H * cx;
        cy = disparity.cols / infer_data->input.image_W * cy;

        float x_ratio = infer_data->input.image_W / disparity.cols;
        float y_ratio = infer_data->input.image_H / disparity.rows;

        for (int v = 0; v < disparity.rows; v += down_sample_ratio){
            for (int u = 0; u < disparity.cols; u += down_sample_ratio){
                int real_u = u * x_ratio;
                int real_v = v * y_ratio;

                float disparity_value = disparity.at<float>(v, u);
                float z = baseline * fx / disparity_value;
                float x = z * (cx - real_u) / fx;
                float y = z * (cy - real_v) / fy;

                if (std::isnan(x) || std::isnan(y) || std::isnan(z))
                {
                    continue;
                }
                if (y > laser_config.max_height || y < laser_config.min_height)
                {
                    continue;
                }
                double range = hypot(x, z);
                if (range > laser_config.range_max || range < laser_config.range_min)
                {
                    continue;
                }
                double angle = std::atan2(x, z);
                if (angle > laser_config.angle_max || angle < laser_config.angle_min)
                {
                    continue;
                }
                int index = (angle - laser_config.angle_min) / laser_config.angle_increment;
                if (range < laserscan_message["data"]["ranges"][index])
                {
                    laserscan_message["data"]["ranges"][index] = range;
                }
            }
        }
        {
            camera_node->udp_client_->send_message(laserscan_message.dump());
        }
    }
    void start(){
        executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
        for(auto &camera_node : camera_nodes_){
            executor_->add_node(camera_node);
        }
        spin_thread_ = std::thread(
            [this](){
                executor_->spin(); 
            }
        );
        if(websocket_client_){
            websocket_client_->start();
        }
        for(auto &camera_node : camera_nodes_){
            camera_node->start();
        }
        worker_threads_.emplace_back(
            std::make_shared<std::thread>(
                [this](){
                    rclcpp::WallRate rate(10);
                    std::shared_ptr<CameraNode> camera_node;
                    for(int i = 0; i < camera_nodes_.size(); i++){
                        if(camera_nodes_[i]->camera_config_.is_master){
                            camera_node = camera_nodes_[i];
                        }
                    }
                    while(rclcpp::ok()){
                        followme_thread(camera_node);
                        rate.sleep();
                    }
                }
            )
        );
        worker_threads_.emplace_back(
            std::make_shared<std::thread>(
                [this](){
                    rclcpp::WallRate rate(10);
                    std::shared_ptr<CameraNode> camera_node;
                    for(int i = 0; i < camera_nodes_.size(); i++){
                        if(camera_nodes_[i]->camera_config_.is_master){
                            camera_node = camera_nodes_[i];
                        }
                    }
                    if(camera_node->camera_config_.camera_type != CameraType::STEREO){
                        RCLCPP_ERROR_STREAM(this->get_logger(), "master camera is not stereo, can not launch obstacle thread!!!");
                    }
                    else{
                        while(rclcpp::ok()){
                            obstacle_thread(camera_node);
                            rate.sleep();
                        }
                    }
                }
            )
        );

    }
    void stop(){
        if (executor_){
            executor_->cancel();
            RCLCPP_INFO_STREAM(this->get_logger(), "cancel executor Finish");
        }
        if (spin_thread_.joinable()){
            spin_thread_.join();
            RCLCPP_INFO_STREAM(this->get_logger(), "join spin thread Finish");
        }
        for(auto &t : worker_threads_){
            if(t->joinable()){
                t->join();
            }
        }
        worker_threads_.clear();
    }
public:
    void Display(){
        // if(!debug_){
        //     return;
        // }
        // for(auto &camera_node : camera_nodes_){
        //     auto element = camera_node->buffers_["display_buffer"]->read();
        //     if(element && element->msg1){
        //         cv::Mat image = cv_bridge::toCvShare(element->msg1, "bgr8")->image;
        //         cv::imshow(camera_node->camera_config_.name, image);
        //         cv::waitKey(1);
        //     }
        // }
    }
public:
    bool show_, debug_, record_, followme_;

    std::string project_root_;

    std::string camera_config_path_;
    std::string laser_config_path_;
    std::string client_config_path_;
    std::string model_config_path_;
    std::string record_config_path_;

    nlohmann::json camera_config_;
    nlohmann::json laser_config_;
    nlohmann::json client_config_;
    nlohmann::json model_config_;
    nlohmann::json record_config_;
private:
    std::vector<std::shared_ptr<CameraNode>> camera_nodes_;
private:
    std::shared_ptr<WebSocketClient> websocket_client_;
    std::shared_ptr<UDPClient> udp_client_;
private:
    std::vector<std::shared_ptr<std::thread>> worker_threads_;
public:
    CommonThreadPool publish_thread_pool_;
private:
    std::shared_ptr<Engine> engine_;
    std::shared_ptr<FollowMe> followme_task_;
    std::shared_ptr<Obstacle> obstacle_task_;
    
    std::atomic<bool> is_followme_running_;
    std::atomic<bool> is_recognize_running_;
    std::atomic<bool> is_obstacle_running_;
private:
    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
    std::thread spin_thread_;

private:
    float last_trajactory_;
    std::vector<std::vector<float>> trajactory_;
};
