#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream> // for converting the command line parameter to integer

#include <opencv2/opencv.hpp>
#include <flysense-jetson-cam/cam.h>
#include <flysense-viewer/viewer.h>

#include <thread>
#include <mutex>

std::mutex viewer_mutex;

std::unique_ptr<Camera> make_camera(const ros::NodeHandle& nh, size_t idx, size_t default_fps = 30, size_t default_width = 4032, size_t default_height = 3040)
{
    cv::Mat camera_matrix;
    cv::Mat distortion_coefficients;

    size_t cam_width, cam_height, cam_downscale_width, cam_downscale_height, cam_fps;

    std::stringstream cam_width_str, cam_height_str, cam_downscale_width_str, cam_downscale_height_str, cam_fps_str;
    cam_width_str << "cam_" << idx << "_width";
    cam_height_str << "cam_" << idx << "_height";
    cam_downscale_width_str << "cam_" << idx << "_downscale_width";
    cam_downscale_height_str << "cam_" << idx << "_downscale_height";
    cam_fps_str << "cam_" << idx << "_fps";

    if (!nh.getParam(cam_width_str, cam_width))
    {
        cam_width = default_width;
    }
    
    if (!nh.getParam(cam_height_str, cam_height))
    {
        cam_height = default_height;
    }

    if (!nh.getParam(cam_fps_str, cam_fps))
    {
        cam_fps = default_fps;
    }

    bool cam_1_downscale = nh.getParam(cam_downscale_width_str.str(), cam_downscale_width) && nh.getParam(cam_downscale_height_str.str(), cam_downscale_height);
    if (!cam_1_downscale)
    {
        cam_downscale_height = cam_1_height;
        cam_downscale_width = cam_1_width;
    }

    return std::make_shared<Camera>(cv::Size(cam_width, cam_height), cam_fps, 0, cv::Size(cam_downscale_width, cam_downscale_height), camera_matrix, distortion_coefficients);
}

void stream(size_t idx, ros::NodeHandle nh, std::shared_ptr<flysense::jetson::Viewer> viewer, std::shared_ptr<flysense::jetson::camera::Camera> camera)
{
    sensor_msgs::ImagePtr msg;
    cv::cuda::GpuMat gpu_img;
    cv::Mat cpu_img;
    uint64_t timestamp;

    std::stringstream image_topic_name, image_frame_id;
    image_topic_name << "camera_" << i << "/image";
    image_transport::Publisher publisher = it.advertise(image_topic_name.str(), 1);
    image_frame_id << "camera_" << i;

    while(nh.ok())
    {
        auto got_image = camera->getNextImageBGR(gpu_img, timestamp);
        if (!got_image)
        {
            ROS_INFO("Could not get image from camera %d", idx);
            continue;
        }

        // convert to ROS timestamp? Better: change lib to return a ROS timestamp with ROS time server. TODO
        auto image_time = ros::Time(0);

        viewer_mutex.lock();
        viewer->AddOverlayAndRender(gpu_img, idx);
        viewer_mutex.unlock();

        gpu_img.download(cpu_img);
        std_msgs::Header image_header;
        image_header.frame_id = image_frame_id.str();
        image_header.stamp = image_time;

        msg = cv_bridge::CvImage(image_header, "bgr8", cpu_img).toImageMsg();
        publisher.publish(msg);
    }
}

int main(int argc, char** argv)
{
    if(argv[1] == NULL) return 1;

    std::istringstream camera_idx_command(argv[1]);
    size_t camera_n;
    if(!(camera_idx_command >> camera_n)) return 1;

    std::vector<std::thread> camera_threads;
    camera_threads.reserve(camera_n);
 
    ros::init(argc, argv, "image_node");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    size_t viewer_width, viewer_height, viewer_fps, viewer_port;
    
    if (!nh.getParam("viewer_width", viewer_width))
    {
        viewer_width = 1920;
    }
    
    if (!nh.getParam("viewer_height", viewer_height))
    {
        viewer_height = 1200;
    }
    
    if (!nh.getParam("viewer_fps", viewer_fps))
    {
        viewer_fps = 30;
    }

    if (!nh.getParam("viewer_port", viewer_port))
    {
        viewer_port = 8080;
    }

    auto viewer = std::make_shared<flysense::jetson::Viewer>(cv::Size(viewer_width, viewer_height), viewer_fps, viewer_port);
    if (!viewer.StartWebServer())
    {
        std::cout << "not started webserver\n";
        return 1;
    }
    viewer.SelectCamera(0);

    for(size_t i = 0; i < camera_n - 1; i++)
    {
        camera_threads.emplace_back(&stream, i, nh, viewer, make_camera(nh, i));
    }

    ros::Rate loop_rate(30);
    while (nh.ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    for(auto& thread: camera_threads)
    {
        thread.join();
    }
}
