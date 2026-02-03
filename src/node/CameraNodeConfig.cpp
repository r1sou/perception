#include "node/CameraNode.h"

void CameraNode::configuration_camera(nlohmann::json &camera_config){
    {
        // basic
        buffers_["publish_image_buffer"] = std::make_shared<TripletBuffer<sensor_msgs::msg::Image::SharedPtr>>();
        buffers_["publish_laserscan_buffer"] = std::make_shared<TripletBuffer<sensor_msgs::msg::Image::SharedPtr>>();
        // test
        buffers_["display_buffer"] = std::make_shared<TripletBuffer<sensor_msgs::msg::Image::SharedPtr>>();
        // task
        buffers_["followme_buffer"] = std::make_shared<TripletBuffer<sensor_msgs::msg::Image::SharedPtr>>();
        buffers_["recognize_buffer"] = std::make_shared<TripletBuffer<sensor_msgs::msg::Image::SharedPtr>>();
        buffers_["record_buffer"] = std::make_shared<TripletBuffer<sensor_msgs::msg::Image::SharedPtr>>();
        buffers_["obstacle_buffer"] = std::make_shared<TripletBuffer<sensor_msgs::msg::Image::SharedPtr>>();
    }
    {
        if(camera_config["type"] == "depth"){
            sub1_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
                shared_from_this(), camera_config["topic"]["image1"].get<std::string>());
            sub2_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
                shared_from_this(), camera_config["topic"]["image2"].get<std::string>());
            syncApproximate_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
                SyncPolicy(10), *sub1_, *sub2_);
            syncApproximate_->registerCallback(&CameraNode::DualImageCallback, this);
        }
        else if(camera_config["type"] == "mono"){
            sub_ = create_subscription<sensor_msgs::msg::Image>(
                camera_config["topic"]["image1"].get<std::string>(), 10,
                [this](const sensor_msgs::msg::Image::SharedPtr msg) {
                    SingleImageCallback(msg);
                }
            );
        }
        else if(camera_config["type"] == "stereo"){
            sub1_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
                shared_from_this(), camera_config["topic"]["image1"].get<std::string>());
            sub2_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
                shared_from_this(), camera_config["topic"]["image2"].get<std::string>());
            syncApproximate_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
                SyncPolicy(10), *sub1_, *sub2_);
            syncApproximate_->registerCallback(&CameraNode::DualImageCallback, this);
        }
    }
    {
        if(camera_config["type"] == "depth"){
            camera_config_.camera_type = CameraType::DEPTH;
        } else if(camera_config["type"] == "mono"){
            camera_config_.camera_type = CameraType::MONO;
        } else if(camera_config["type"] == "stereo"){
            camera_config_.camera_type = CameraType::STEREO;
        }
        camera_config_.device_id = camera_config["device_id"].get<int>();
        if(camera_config.contains("calib")){
            camera_config_.fx = camera_config["calib"]["fx"].get<float>();
            camera_config_.fy = camera_config["calib"]["fy"].get<float>();
            camera_config_.cx = camera_config["calib"]["cx"].get<float>();
            camera_config_.cy = camera_config["calib"]["cy"].get<float>();
            camera_config_.baseline = camera_config["calib"].value("baseline", 0.0);
        }
        if(camera_config.contains("shape")){
            std::vector<int> shape = camera_config["shape"];
            camera_config_.image_width = shape[0];
            camera_config_.image_height = shape[1];
        }
        if(camera_config.contains("distortion")){
            camera_config_.arr_d = camera_config["distortion"]["arr_d"].get<std::vector<double>>();
            camera_config_.arr_k = camera_config["distortion"]["arr_k"].get<std::vector<double>>();
            camera_config_.arr_r = camera_config["distortion"]["arr_r"].get<std::vector<double>>();
            camera_config_.arr_p = camera_config["distortion"]["arr_p"].get<std::vector<double>>();
        }
        camera_config_.name = camera_config["name"].get<std::string>();
        camera_config_.is_master = camera_config.value("is_master", false);
        camera_config_.frame_id = camera_config["topic"]["frame_id"].get<std::string>();

        publish_laserscan_fps_ = camera_config.value("publish_laserscan_fps", 5);

        configuration_static_tf();
    }
}

void CameraNode::DualImageCallback(const sensor_msgs::msg::Image::SharedPtr msg1, const sensor_msgs::msg::Image::SharedPtr msg2){
    for(auto &[buffer_name, buffer] : buffers_){
        buffer->update(
            [msg1, msg2](sensor_msgs::msg::Image::SharedPtr &msg1_, sensor_msgs::msg::Image::SharedPtr &msg2_){
                msg1_ = msg1;
                msg2_ = msg2;
            }
        );
    }
}

void CameraNode::SingleImageCallback(const sensor_msgs::msg::Image::SharedPtr msg){
    for(auto &[buffer_name, buffer] : buffers_){
        buffer->update(
            [msg](sensor_msgs::msg::Image::SharedPtr &msg1_, sensor_msgs::msg::Image::SharedPtr &msg2_){
                msg1_ = msg;
                // msg2_ = msg;
            }
        );
    }
}

void CameraNode::configuration_laser(nlohmann::json &laser_config){
    laser_config_.down_sample_ratio = laser_config["laserscan"].value("down_sample_ratio", 1);
    laser_config_.tolerance = laser_config["laserscan"].value("tolerance", 0.01);
    laser_config_.min_height = laser_config["laserscan"].value("min_height", 0.0);
    laser_config_.max_height = laser_config["laserscan"].value("max_height", 1.0);
    laser_config_.angle_min = laser_config["laserscan"].value("angle_min", -1.5707963267948966);
    laser_config_.angle_max = laser_config["laserscan"].value("angle_max", 1.5707963267948966);
    laser_config_.angle_increment = laser_config["laserscan"].value("angle_increment", 0.00315);
    laser_config_.time_increment = laser_config["laserscan"].value("time_increment", 0.0);
    laser_config_.scan_time = laser_config["laserscan"].value("scan_time", 0.1);
    laser_config_.range_min = laser_config["laserscan"].value("range_min", 0.05);
    laser_config_.range_max = laser_config["laserscan"].value("range_max", 150.0);
    laser_config_.use_inf = laser_config["laserscan"].value("use_inf", true);
    laser_config_.inf_epsilon = laser_config["laserscan"].value("inf_epsilon", 1.0);
}

void CameraNode::configuration_client(nlohmann::json &client_config){
    auto &config = client_config["client"];
    {
        std::string uri = fmt::format("ws://{}:{}", config["ip"].get<std::string>(), config["WebSocket"]["port"].get<int>());
        websocket_client_ = std::make_shared<WebSocketClient>(uri, client_config);
    }
    {
        udp_client_ = std::make_shared<UDPClient>(config["ip"].get<std::string>(), config["UDP"]["port"].get<int>());
    }
    {
        {
            // init laserscan message
            laserscan_message_["cmd_code"] = 0x01;
            laserscan_message_["device_id"] = camera_config_.device_id;
            laserscan_message_["data"] = nlohmann::json::object();
            laserscan_message_["data"]["angle_min"] = laser_config_.angle_min;
            laserscan_message_["data"]["angle_max"] = laser_config_.angle_max;
            laserscan_message_["data"]["angle_increment"] = laser_config_.angle_increment;
            laserscan_message_["data"]["time_increment"] = laser_config_.time_increment;
            laserscan_message_["data"]["scan_time"] = laser_config_.scan_time;
            laserscan_message_["data"]["range_min"] = laser_config_.range_min;
            laserscan_message_["data"]["range_max"] = laser_config_.range_max;
            std::vector<float> ranges;
            uint32_t ranges_size = std::ceil((laser_config_.angle_max - laser_config_.angle_min) / laser_config_.angle_increment);
            if (laser_config_.use_inf)
            {
                ranges.assign(ranges_size, std::numeric_limits<double>::infinity());
            }
            else
            {
                ranges.assign(ranges_size, laser_config_.range_max + laser_config_.inf_epsilon);
            }
            laserscan_message_["data"]["ranges"] = ranges;
        }
        {
            // init camera info message
            camera_info_message_["cmd_code"] = 0x02;
            camera_info_message_["device_id"] = camera_config_.device_id;
            camera_info_message_["data"] = nlohmann::json::object();
            camera_info_message_["data"]["height"] = camera_config_.image_height;
            camera_info_message_["data"]["width"] = camera_config_.image_width;
            camera_info_message_["data"]["arr_d"] = camera_config_.arr_d;
            camera_info_message_["data"]["arr_k"] = camera_config_.arr_k;
            camera_info_message_["data"]["arr_r"] = camera_config_.arr_r;
            camera_info_message_["data"]["arr_p"] = camera_config_.arr_p;
        }
        {
            // init image message
            image_message_["cmd_code"] = 0x03;
            image_message_["device_id"] = camera_config_.device_id;
        }
    }
}

void CameraNode::configuration_record(nlohmann::json &record_config){
    record_config_.image_width = camera_config_.image_width;
    record_config_.image_height = camera_config_.image_height;
    record_config_.fps = record_config["record"].value("fps", 10);
    record_config_.save_dir = record_config["record"].value("save_dir", "/home/sunrise/Desktop/dataset");
    record_config_.suffix = record_config["record"].value("suffix", ".avi");
    record_config_.fourcc = record_config["record"].value("fourcc", 0);

    if(record_config_.fourcc == 0){
        record_config_.fourcc = cv::VideoWriter::fourcc('X', 'V', 'I', 'D');
    }
    else if(record_config_.fourcc == 1){
        record_config_.fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
    }
    else if(record_config_.fourcc == 2){
        record_config_.fourcc = cv::VideoWriter::fourcc('a', 'v', 'c', '1');
    }
    else if(record_config_.fourcc == 3){
        record_config_.fourcc = cv::VideoWriter::fourcc('h', 'v', 'c', '1');
    }
}

void CameraNode::configuration_static_tf(){
    if(camera_config_.camera_type == CameraType::MONO){
        return;
    }

    {
        laser_scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
            fmt::format("{}/laserscan", camera_config_.name), 10
        );
        point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            fmt::format("{}/point_cloud", camera_config_.name), 10
        );
    }

    if(camera_config_.camera_type == CameraType::DEPTH){
        return;
    }

    static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    geometry_msgs::msg::TransformStamped transformStamped;

    transformStamped.header.stamp = this->now();
    transformStamped.header.frame_id = camera_config_.frame_id;
    transformStamped.child_frame_id = fmt::format("{}_child", camera_config_.frame_id);

    transformStamped.transform.translation.x = 0.0;
    transformStamped.transform.translation.y = 0.0;
    transformStamped.transform.translation.z = 0.0;

    static_broadcaster_->sendTransform(transformStamped);
}
