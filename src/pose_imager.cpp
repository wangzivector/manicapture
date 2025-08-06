#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/CameraInfo.h>
#include <ros/package.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <sys/mman.h>
#include <string.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "pose_imager");
    ros::NodeHandle nh;

    ros::Publisher img_pub = nh.advertise<sensor_msgs::Image>("camera/image_raw", 1);
    ros::Publisher info_pub = nh.advertise<sensor_msgs::CameraInfo>("camera/camera_info", 1);

    std::string camera_id;
    nh.param<std::string>("camera_id", camera_id, "/dev/video0");
    // const char* dev_name = "/dev/video0";
    int fd = open(camera_id.c_str(), O_RDWR);
    if (fd == -1) {
        ROS_ERROR("Cannot open device");
        return 1;
    }
    
    // struct v4l2_streamparm streamparm;
    // memset(&streamparm, 0, sizeof(streamparm));
    // streamparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    // ioctl(fd, VIDIOC_G_PARM, &streamparm);

    // streamparm.parm.capture.timeperframe.numerator = 1;
    // streamparm.parm.capture.timeperframe.denominator = 120;
    // ioctl(fd, VIDIOC_S_PARM, &streamparm);

    // Set format
    struct v4l2_format fmt = {};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = 1280;
    fmt.fmt.pix.height = 720;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;
    ioctl(fd, VIDIOC_S_FMT, &fmt);

    // Mannual exposure
    struct v4l2_control control;
    control.id = V4L2_CID_EXPOSURE_AUTO;
    control.value = V4L2_EXPOSURE_MANUAL;
    ioctl(fd, VIDIOC_S_CTRL, &control);

    control.id = V4L2_CID_EXPOSURE_ABSOLUTE;
    control.value = 200; // exposure time in 100µs units
    ioctl(fd, VIDIOC_S_CTRL, &control);

    
    // Request buffers
    struct v4l2_requestbuffers req = {};
    req.count = 1;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    ioctl(fd, VIDIOC_REQBUFS, &req);

    // Query buffer
    struct v4l2_buffer buf = {};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = 0;
    ioctl(fd, VIDIOC_QUERYBUF, &buf);

    void* buffer_start = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf.m.offset);

    ioctl(fd, VIDIOC_QBUF, &buf);
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl(fd, VIDIOC_STREAMON, &type);

    std::string camera_info_path;
    if (!nh.getParam("camera_info", camera_info_path)) {
        camera_info_path = ros::package::getPath("manicapture") + "/cfg/caminfo.yaml";
    }
    camera_info_manager::CameraInfoManager cinfo(nh, "camera", camera_info_path);
    sensor_msgs::CameraInfo cam_info = cinfo.getCameraInfo();

    // ros::Rate loop_rate(30);  // 30 FPS
    static ros::Time last_time;
    static int frame_count = 0;
    static ros::Time start_time = ros::Time::now();
    
    while (ros::ok()) {
        ioctl(fd, VIDIOC_DQBUF, &buf);
    
        std::vector<uchar> mjpeg_data((uchar*)buffer_start, (uchar*)buffer_start + buf.bytesused);
        cv::Mat image = cv::imdecode(mjpeg_data, cv::IMREAD_GRAYSCALE);
        
        if (!image.empty()) {
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", image).toImageMsg();
            msg->header.stamp = ros::Time::now();
            img_pub.publish(msg);
            frame_count ++;
        } else {
            ROS_WARN("Empty frame skipped");
        }
        ioctl(fd, VIDIOC_QBUF, &buf);  // Requeue buffer
        ros::spinOnce();
        // loop_rate.sleep();
        
        ros::Time current_time = ros::Time::now();
        if ((current_time - start_time).toSec() >= 1.0) {
            double fps = frame_count / (current_time - start_time).toSec();
            ROS_INFO("Actual frame rate: %.2f FPS", fps);
            start_time = current_time;
            frame_count = 0;
            info_pub.publish(cam_info);
        }
    }

    ioctl(fd, VIDIOC_STREAMOFF, &type);
    munmap(buffer_start, buf.length);
    close(fd);

    return 0;
}
