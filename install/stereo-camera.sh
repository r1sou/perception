source /opt/ros/humble/setup.bash
source /opt/tros/humble/setup.bash
if [ -d "/root/install" ]; then
    source /root/install/setup.bash
fi
ros2 launch mipi_cam mipi_cam_dual_channel.launch.py mipi_image_width:=544 mipi_image_height:=448 mipi_out_format:=nv12 mipi_frame_ts_type:=realtime

