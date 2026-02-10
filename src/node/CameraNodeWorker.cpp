#include "node/CameraNode.h"

void CameraNode::send_camera_status()
{
    if (!websocket_client_->connected.load())
    {
        return;
    }
    nlohmann::json message;
    message["device_id"] = camera_config_.device_id;
    message["cmd_code"] = 0x16;
    time_t timestamp = time(NULL);
    message["time_stamp"] = timestamp;
    message["key"] = JWTGenerator::generate(
        websocket_client_->m_config["client"]["JWT"]["req_id"],
        websocket_client_->m_config["client"]["JWT"]["key"]);
    if (camera_config_.is_master)
    {
        message["data"]["follow_collect_status"] = websocket_client_->start_follow.load() ? 2 : 0;
    }
    else
    {
      
        message["data"]["follow_collect_status"] = 0;
    }
    message["data"]["identify_collect_status"] = websocket_client_->start_identify_collect.load() ? 1 : 0;
    message["data"]["cam_record_status"] = websocket_client_->start_cam_record.load() ? 1 : 0;
    websocket_client_->send_message(message.dump());
}

bool CameraNode::infer_common_process(std::shared_ptr<InferenceData_t> infer_data, std::string buffer_name)
{
    auto element = buffers_[buffer_name]->read();
    if (!element || !element->msg1)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        return false;
    }
    std::string encoding = element->msg2 ? element->msg2->encoding : element->msg1->encoding;
    if (encoding == "bgr8" || encoding == "BGR8")
    {
        infer_data->input.image_type = INPUT_IMAGE_TYPE::BGR;
        infer_data->input.images.push_back(cv_bridge::toCvShare(element->msg1, element->msg1->encoding)->image);
        if (element->msg2)
        {
            infer_data->input.images.push_back(cv_bridge::toCvShare(element->msg2, element->msg2->encoding)->image);
        }
        infer_data->input.image_H = element->msg1->height;
        infer_data->input.image_W = element->msg1->width;
    }
    else if (encoding == "rgb8" || encoding == "RGB8")
    {
        infer_data->input.image_type = INPUT_IMAGE_TYPE::RGB;
        infer_data->input.images.push_back(cv_bridge::toCvShare(element->msg1, element->msg1->encoding)->image);
        if (element->msg2)
        {
            infer_data->input.images.push_back(cv_bridge::toCvShare(element->msg2, element->msg2->encoding)->image);
        }
        infer_data->input.image_H = element->msg1->height;
        infer_data->input.image_W = element->msg1->width;
    }
    else if (encoding == "nv12" || encoding == "NV12")
    {
        infer_data->input.image_type = INPUT_IMAGE_TYPE::NV12;
        infer_data->input.images.push_back(cv::Mat(element->msg1->height * 3 / 2, element->msg1->width, CV_8UC1, element->msg1->data.data(), element->msg1->step));
        if (element->msg2)
        {
            infer_data->input.images.push_back(cv::Mat(element->msg2->height * 3 / 2, element->msg2->width, CV_8UC1, element->msg2->data.data(), element->msg2->step));
        }
        infer_data->input.image_H = element->msg1->height;
        infer_data->input.image_W = element->msg1->width;
    }
    else if (encoding == "16UC1")
    {
        infer_data->input.image_type = INPUT_IMAGE_TYPE::U16C1;
        infer_data->input.images.push_back(cv_bridge::toCvShare(element->msg1, element->msg1->encoding)->image);
        if (element->msg2)
        {
            infer_data->input.images.push_back(cv_bridge::toCvShare(element->msg2, element->msg2->encoding)->image);
        }
        infer_data->input.image_H = element->msg1->height;
        infer_data->input.image_W = element->msg1->width;
    }
    return true;
}

void CameraNode::record_thread()
{
    if (websocket_client_->connected.load())
    {
        if ((websocket_client_->start_cam_record.load() && !is_record_running_.load()) || record_)
        {
            is_record_running_.store(true);
            if (!record_)
            {
                RCLCPP_INFO_STREAM(this->get_logger(), fmt::format("camera {} start record", camera_config_.device_id).c_str());
            }
        }
        if ((!websocket_client_->start_cam_record.load() && is_record_running_.load()) && !record_)
        {
            is_record_running_.store(false);
            if (video_writer_)
            {
                video_writer_->release();
            }
            RCLCPP_INFO_STREAM(this->get_logger(), fmt::format("stop record, save path: {}, total frame: {}", save_path_, frame_count_).c_str());
            frame_count_ = 0;
        }
        if (!is_record_running_.load())
        {
            return;
        }
        auto infer_data = std::make_shared<InferenceData_t>();
        if (!infer_common_process(infer_data, "record_buffer"))
        {
            return;
        }
        {
            if (!video_writer_)
            {
                save_path_ = fmt::format("{}/camera-{}", record_config_.save_dir, camera_config_.device_id);
                std::filesystem::create_directories(save_path_);
                save_path_ += "/" + get_string_date(3) + record_config_.suffix;
                int width = camera_config_.camera_type == CameraType::STEREO ? record_config_.image_width * 2 : record_config_.image_width;
                video_writer_ = std::make_shared<cv::VideoWriter>(
                    save_path_, record_config_.fourcc, record_config_.fps,
                    cv::Size(width, record_config_.image_height));
            }
            if (camera_config_.camera_type != CameraType::STEREO)
            {
                cv::Mat write_image;
                if (infer_data->input.image_type != INPUT_IMAGE_TYPE::NV12)
                {
                    write_image = infer_data->input.images[0];
                }
                else
                {
                    image_conversion::nv12_to_bgr(infer_data->input.images[0], write_image);
                }
                video_writer_->write(write_image);
                frame_count_++;
            }
            else
            {
                cv::Mat write_image1, write_image2;
                if (infer_data->input.image_type != INPUT_IMAGE_TYPE::NV12)
                {
                    write_image1 = infer_data->input.images[0];
                    write_image2 = infer_data->input.images[1];
                }
                else
                {
                    image_conversion::nv12_to_bgr(infer_data->input.images[0], write_image1);
                    image_conversion::nv12_to_bgr(infer_data->input.images[1], write_image2);
                }
                cv::Mat concat_image;
                cv::hconcat(write_image1, write_image2, concat_image);
                video_writer_->write(concat_image);
                frame_count_++;
            }
        }
    }
}

// void CameraNode::followme_thread(){
//     if(websocket_client_->connected.load()){
//         if((websocket_client_->start_follow.load() && !is_followme_running_.load()) && followme_){
//             is_followme_running_.store(true);
//             RCLCPP_INFO_STREAM(this->get_logger(), fmt::format("camera {} start followme", camera_config_.device_id).c_str());
//         }
//         if((!websocket_client_->start_follow.load() && is_followme_running_.load()) && followme_){
//             is_followme_running_.store(false);
//             RCLCPP_INFO_STREAM(this->get_logger(), fmt::format("camera {} stop followme", camera_config_.device_id).c_str());
//         }
//         if(!is_followme_running_.load()){
//             return;
//         }
//         auto infer_data = std::make_shared<InferenceData_t>();
//         if(!infer_common_process(infer_data, "followme_buffer")){
//             return;
//         }

//     }
// }

void CameraNode::publish_laserscan()
{
    static int64_t last_publish_time = 0;

    if (camera_config_.camera_type != CameraType::DEPTH)
    {
        return;
    }
    if (!udp_client_)
    {
        return;
    }
    auto element = buffers_["publish_laserscan_buffer"]->read();
    if (!element || !element->msg1)
    {
        return;
    }

    int64_t publish_time = element->msg2->header.stamp.sec * 1000LL + element->msg2->header.stamp.nanosec / 1000000;
    if (last_publish_time > 0 && publish_time - last_publish_time < 10)
    {
        RCLCPP_DEBUG_STREAM(this->get_logger(), "depth camera too frequent");
        return;
    }
    last_publish_time = publish_time;

    // ScopeTimer timer(fmt::format("camera {} publish_laserscan", camera_config_.device_id));
    auto laserscan_message = laserscan_message_;
    {
        time_t timestamp = time(NULL);
        laserscan_message["time_stamp"] = timestamp;
    }

    auto scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>();
    auto pc_msg = std::make_unique<sensor_msgs::msg::PointCloud2>(
        rosidl_runtime_cpp::MessageInitialization::SKIP);
    sensor_msgs::msg::PointCloud2 &point_cloud_msg = *pc_msg;
    {
        scan_msg->header = element->msg2->header;
    }
    {
        scan_msg->angle_min = laser_config_.angle_min;
        scan_msg->angle_max = laser_config_.angle_max;
        scan_msg->angle_increment = laser_config_.angle_increment;
        scan_msg->time_increment = 0.0;
        scan_msg->scan_time = laser_config_.scan_time;
        scan_msg->range_min = laser_config_.range_min;
        scan_msg->range_max = laser_config_.range_max;
        uint32_t ranges_size = std::ceil(
            (scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment);
        if (laser_config_.use_inf)
        {
            scan_msg->ranges.assign(ranges_size, std::numeric_limits<double>::infinity());
        }
        else
        {
            scan_msg->ranges.assign(ranges_size, scan_msg->range_max + laser_config_.inf_epsilon);
        }
    }
    {
        point_cloud_msg.header = scan_msg->header;
        point_cloud_msg.is_dense = false;
        point_cloud_msg.fields.resize(4);
        point_cloud_msg.fields[0].name = "x";
        point_cloud_msg.fields[0].offset = 0;
        point_cloud_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
        point_cloud_msg.fields[0].count = 1;
        point_cloud_msg.fields[1].name = "y";
        point_cloud_msg.fields[1].offset = 4;
        point_cloud_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
        point_cloud_msg.fields[1].count = 1;
        point_cloud_msg.fields[2].name = "z";
        point_cloud_msg.fields[2].offset = 8;
        point_cloud_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
        point_cloud_msg.fields[2].count = 1;
        point_cloud_msg.fields[3].name = "rgb";
        point_cloud_msg.fields[3].offset = 12;
        point_cloud_msg.fields[3].datatype = sensor_msgs::msg::PointField::UINT32;
        point_cloud_msg.fields[3].count = 1;

        point_cloud_msg.height = 1;
        point_cloud_msg.point_step = 16;

        point_cloud_msg.data.resize((480 / laser_config_.down_sample_ratio) * (640 / laser_config_.down_sample_ratio) * point_cloud_msg.point_step * point_cloud_msg.height);
    }

    uint32_t point_size = 0;
    float *pcd_data_ptr = reinterpret_cast<float *>(point_cloud_msg.data.data());

    cv::Mat image = cv_bridge::toCvShare(element->msg1, "bgr8")->image;
    cv::Mat depth = cv_bridge::toCvShare(element->msg2, "16UC1")->image;
    {
        // ScopeTimer timer(fmt::format("camera {} pack_laserscan", camera_config_.device_id));
        for (int v = 0; v < depth.rows; v += laser_config_.down_sample_ratio)
        {
            for (int u = 0; u < depth.cols; u += laser_config_.down_sample_ratio)
            {
                double z = static_cast<double>(depth.at<uint16_t>(v, u)) / 1000.0;
                if (z > 0)
                {
                    double x = z * (camera_config_.cx - u) / camera_config_.fx;
                    double y = z * (camera_config_.cy - v) / camera_config_.fy;

                    if (std::isnan(x) || std::isnan(y) || std::isnan(z))
                    {
                        continue;
                    }
                    if (y < 5.0 && y > -5.0)
                    {
                        *pcd_data_ptr++ = z;
                        *pcd_data_ptr++ = x;
                        *pcd_data_ptr++ = y;
                        cv::Vec3b pixel = image.at<cv::Vec3b>(v, u);
                        *(uint32_t *)pcd_data_ptr++ = (pixel[2] << 16) | (pixel[1] << 8) | (pixel[0] << 0);
                        point_size++;
                    }
                    if (y > laser_config_.max_height || y < laser_config_.min_height)
                    {
                        continue;
                    }
                    double range = hypot(x, z);
                    if (range > laser_config_.range_max || range < laser_config_.range_min)
                    {
                        continue;
                    }

                    double angle = std::atan2(x, z);
                    if (angle > laser_config_.angle_max || angle < laser_config_.angle_min)
                    {
                        continue;
                    }
                    int index = (angle - laser_config_.angle_min) / laser_config_.angle_increment;
                    if (range < laserscan_message["data"]["ranges"][index])
                    {
                        laserscan_message["data"]["ranges"][index] = range;
                        scan_msg->ranges[index] = range;
                    }
                }
            }
        }
    }
    {
        point_cloud_msg.width = point_size;
        point_cloud_msg.row_step = point_cloud_msg.point_step * point_cloud_msg.width;
        point_cloud_msg.data.resize(point_size * point_cloud_msg.point_step *
                                    point_cloud_msg.height);
        point_cloud_pub_->publish(std::move(pc_msg));
    }
    {
        laser_scan_pub_->publish(std::move(scan_msg));
    }
    {
        // ScopeTimer timer(fmt::format("camera {} send_laserscan", camera_config_.device_id));
        udp_client_->send_message(laserscan_message.dump());
    }
    // debug
    {
        int64_t receive_time = element->timestamp.count();
        int64_t sendmsg_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        int64_t latency = sendmsg_time - publish_time;
        // RCLCPP_INFO_STREAM(
        //     this->get_logger(),
        //     fmt::format(
        //         "pub: \33[32m{:.3f}\33[0m, rec: \33[32m{:.3f}\33[0m, send: \33[32m{:.3f}\33[0m, latency: \33[32m{}\33[0m",
        //         publish_time / 1000.0, receive_time / 1000.0, sendmsg_time / 1000.0, latency)
        //         .c_str());
    }
}

void CameraNode::publish_camera_info()
{
    if (camera_config_.camera_type != CameraType::DEPTH)
    {
        return;
    }
    if (!udp_client_)
    {
        return;
    }
    return;
    // ScopeTimer timer(fmt::format("camera {} publish_camera_info", camera_config_.device_id));
    {
        time_t timestamp = time(NULL);
        camera_info_message_["time_stamp"] = timestamp;
    }
    {
        udp_client_->send_message(camera_info_message_.dump());
    }
}
void CameraNode::publish_image()
{
    // todo optimize
    if (!udp_client_)
    {
        return;
    }
    auto element = buffers_["publish_image_buffer"]->read();
    if (!element || !element->msg1)
    {
        return;
    }
    {
        time_t timestamp = time(NULL);
        image_message_["time_stamp"] = timestamp;
    }
    {
        // ScopeTimer timer(fmt::format("camera {} pack_image", camera_config_.device_id));
        image_message_["data"] = nlohmann::json::object();
        image_message_["data"]["height"] = element->msg1->height;
        image_message_["data"]["width"] = element->msg1->width;
        image_message_["data"]["encoding"] = element->msg1->encoding;
        image_message_["data"]["is_bigendian"] = element->msg1->is_bigendian;
        image_message_["data"]["step"] = element->msg1->step;

        image_message_["data"]["data"] = std::move(element->msg1->data);
        // image_message_["data"]["data"] = nlohmann::json::binary(std::move(element->msg1->data));
    }
    {
        // ScopeTimer timer(fmt::format("camera {} send_image", camera_config_.device_id));
        udp_client_->send_message(image_message_.dump());
        // std::vector<uint8_t> cbor_data = nlohmann::json::to_cbor(image_message_);
        // udp_client_->send_binary_message(cbor_data.data(), cbor_data.size());
    }
}