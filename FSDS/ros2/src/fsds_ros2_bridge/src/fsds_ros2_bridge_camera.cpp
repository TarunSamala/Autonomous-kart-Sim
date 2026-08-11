#include "common/common_utils/StrictMode.hpp"
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include "common/AirSimSettings.hpp"
#include "common/Common.hpp"
#include "vehicles/car/api/CarRpcLibClient.hpp"
#include "statistics.h"
#include "rpc/rpc_error.h"
#ifdef JAZZY
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif
#include <math.h>
#include <algorithm>
#include <cstring>

using dseconds = std::chrono::duration<double>;

typedef msr::airlib::ImageCaptureBase::ImageRequest ImageRequest;
typedef msr::airlib::ImageCaptureBase::ImageResponse ImageResponse;
typedef msr::airlib::ImageCaptureBase::ImageType ImageType;

// number of seconds to record frames before printing fps
const int FPS_WINDOW = 3;

msr::airlib::CarRpcLibClient *airsim_api;
rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub;
rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub;
rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_image_pub;
rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_pub;
ros_bridge::Statistics fps_statistic;

// settings
std::string camera_name = "";
std::string camera_frame_prefix = "";
std::string camera_frame_id = "";
std::string camera_topic_prefix = "";
std::string camera_topic_name = "";
std::string depth_camera_name = "";
std::string depth_camera_topic_name = "";
double framerate = 0.0;
double camera_fov_degrees = 90.0;
std::string host_ip = "localhost";
bool depthcamera = false;
bool rgbd_mode = false;

void publish_camera_info(
    const ImageResponse &img_response,
    const rclcpp::Time &stamp,
    const rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr &publisher);

rclcpp::Time make_ts(uint64_t unreal_ts)
{
    // unreal timestamp is a unix nanosecond timestamp just like ros.
    // We can do direct translation as long as ros is not running in simulated time mode.
    return rclcpp::Time(unreal_ts);
}

std::vector<ImageResponse> getImages(const std::vector<ImageRequest> &reqs) {
    // simGetImages returns image dimensions and can capture an RGB-D pair in
    // one RPC request, which is required for synchronized visual odometry.
    std::vector<ImageResponse> img_responses;
    try
    {
        img_responses = airsim_api->simGetImages(reqs, "FSCar");
    }
    catch (rpc::rpc_error &e)
    {
        std::cout << "error" << std::endl;
        std::string msg = e.what();
        std::cout << "Exception raised by the API while getting image:" << std::endl
                  << msg << std::endl;
    }
    return img_responses;
}

std::vector<ImageResponse> getImage(const ImageRequest &req) {
    return getImages({req});
}

bool valid_rgb_response(const ImageResponse &response)
{
    if (response.time_stamp == 0 || response.width <= 0 || response.height <= 0)
        return false;

    const size_t expected_size = static_cast<size_t>(response.width) *
                                 static_cast<size_t>(response.height) * 3U;
    return response.image_data_uint8.size() == expected_size;
}

bool valid_depth_response(const ImageResponse &response)
{
    if (response.time_stamp == 0 || response.width <= 0 || response.height <= 0)
        return false;

    const size_t expected_size = static_cast<size_t>(response.width) *
                                 static_cast<size_t>(response.height);
    return response.image_data_float.size() == expected_size;
}

void doImageUpdate()
{
    auto img_responses = getImage(ImageRequest(camera_name, ImageType::Scene, false, false));

    // if a render request failed for whatever reason, this img will be empty.
    if (img_responses.size() != 1 || !valid_rgb_response(img_responses[0]))
        return;

    ImageResponse img_response = img_responses[0];

    sensor_msgs::msg::Image::SharedPtr img_msg = std::make_shared<sensor_msgs::msg::Image>();

    img_msg->data = img_response.image_data_uint8;
    img_msg->height = img_response.height;
    img_msg->width = img_response.width;
    img_msg->step = img_response.width * 3; // image_width * num_bytes
    img_msg->encoding = "bgr8";
    img_msg->is_bigendian = 0;
    const auto stamp = make_ts(img_response.time_stamp);
    img_msg->header.stamp = stamp;
    img_msg->header.frame_id = camera_frame_id;

    image_pub->publish(*img_msg);

    publish_camera_info(img_response, stamp, info_pub);

    fps_statistic.addCount();
}

cv::Mat manual_decode_depth(const ImageResponse &img_response)
{
    cv::Mat mat(img_response.height, img_response.width, CV_32FC1);
    std::memcpy(
        mat.data,
        img_response.image_data_float.data(),
        img_response.image_data_float.size() * sizeof(float));
    return mat;
}

float roundUp(float numToRound, float multiple)
{
    assert(multiple);
    return ((numToRound + multiple - 1) / multiple) * multiple;
}

cv::Mat noisify_depthimage(cv::Mat in)
{
    cv::Mat out = in.clone();

    // Blur
    cv::Mat kernel = cv::Mat::ones(7, 7, CV_32F) / (float)(7 * 7);
    cv::filter2D(in, out, -1, kernel, cv::Point(-1, -1), 0, cv::BORDER_DEFAULT);

    // Decrease depth resolution
    for (int row = 0; row < in.rows; row++)
    {
        for (int col = 0; col < in.cols; col++)
        {
            float roundtarget = ceil(std::min(std::max(out.at<float>(row, col), 1.0f), 10.0f));
            out.at<float>(row, col) = roundUp(out.at<float>(row, col), roundtarget);
        }
    }

    return out;
}

void publish_camera_info(
    const ImageResponse &img_response,
    const rclcpp::Time &stamp,
    const rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr &publisher)
{
    sensor_msgs::msg::CameraInfo::SharedPtr info_msg =
        std::make_shared<sensor_msgs::msg::CameraInfo>();

    info_msg->header.stamp = stamp;
    info_msg->header.frame_id = camera_frame_id;
    info_msg->width = img_response.width;
    info_msg->height = img_response.height;
    info_msg->distortion_model = "plumb_bob";
    info_msg->d = {0, 0, 0, 0, 0};

    const double horizontal_fov_radians = camera_fov_degrees * M_PI / 180.0;
    double fx = static_cast<double>(img_response.width) /
                (2.0 * std::tan(horizontal_fov_radians / 2.0));
    double fy = fx;
    double cx = static_cast<double>(img_response.width) / 2.0;
    double cy = static_cast<double>(img_response.height) / 2.0;

    info_msg->k = {
        fx, 0, cx,
        0, fy, cy,
        0, 0, 1.0
    };

    info_msg->r = {
        1, 0, 0,
        0, 1, 0,
        0, 0, 1
    };

    info_msg->p = {
        fx, 0, cx, 0,
        0, fy, cy, 0,
        0, 0, 1.0, 0
    };

    publisher->publish(*info_msg);
}

void doDepthImageUpdate()
{
    auto img_responses = getImage(ImageRequest(camera_name, ImageType::DepthPerspective, true, false));

    // if a render request failed for whatever reason, this img will be empty.
    if (img_responses.size() != 1 || !valid_depth_response(img_responses[0]))
        return;

    ImageResponse img_response = img_responses[0];

    // AirSim already returns metric DepthPerspective values. Keep the raw
    // 32-bit depth for SLAM instead of applying the legacy blur/quantization.
    cv::Mat depth_img = manual_decode_depth(img_response);
    sensor_msgs::msg::Image::SharedPtr img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "32FC1", depth_img).toImageMsg();
    const auto stamp = make_ts(img_response.time_stamp);
    img_msg->header.stamp = stamp;
    img_msg->header.frame_id = camera_frame_id;
    publish_camera_info(img_response, stamp, info_pub);

    image_pub->publish(*img_msg);
    fps_statistic.addCount();
}

void doRgbdImageUpdate()
{
    const auto responses = getImages({
        ImageRequest(camera_name, ImageType::Scene, false, false),
        ImageRequest(depth_camera_name, ImageType::DepthPerspective, true, false)
    });

    if (responses.size() != 2 ||
        !valid_rgb_response(responses[0]) ||
        !valid_depth_response(responses[1])) {
        return;
    }

    const auto &rgb_response = responses[0];
    const auto &depth_response = responses[1];

    // AirSim captures requests in one simGetImages() batch. Use one timestamp
    // for the pair so exact-time ROS synchronizers cannot combine mismatched
    // exposures from different update cycles.
    const auto pair_stamp = make_ts(std::max(
        rgb_response.time_stamp, depth_response.time_stamp));

    auto rgb_msg = std::make_shared<sensor_msgs::msg::Image>();
    rgb_msg->data = rgb_response.image_data_uint8;
    rgb_msg->height = rgb_response.height;
    rgb_msg->width = rgb_response.width;
    rgb_msg->step = rgb_response.width * 3;
    rgb_msg->encoding = "bgr8";
    rgb_msg->is_bigendian = 0;
    rgb_msg->header.stamp = pair_stamp;
    rgb_msg->header.frame_id = camera_frame_id;

    const cv::Mat depth_mat = manual_decode_depth(depth_response);
    auto depth_msg = cv_bridge::CvImage(
        std_msgs::msg::Header(), "32FC1", depth_mat).toImageMsg();
    depth_msg->header.stamp = pair_stamp;
    depth_msg->header.frame_id = camera_frame_id;

    image_pub->publish(*rgb_msg);
    depth_image_pub->publish(*depth_msg);
    publish_camera_info(rgb_response, pair_stamp, info_pub);
    publish_camera_info(depth_response, pair_stamp, depth_info_pub);
    fps_statistic.addCount();
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> nh = rclcpp::Node::make_shared("fsds_ros2_bridge_camera");

    // load settings
    camera_name = nh->declare_parameter<std::string>("camera_name", "");
    camera_frame_prefix = nh->declare_parameter<std::string>("camera_frame_prefix", "fsds/");
    camera_frame_id = nh->declare_parameter<std::string>("frame_id", camera_frame_prefix + camera_name);
    camera_topic_prefix = nh->declare_parameter<std::string>("camera_topic_prefix", "/fsds/");
    camera_topic_name = camera_topic_prefix + camera_name;
    depth_camera_name = nh->declare_parameter<std::string>("depth_camera_name", "");
    depth_camera_topic_name = camera_topic_prefix + depth_camera_name;

    framerate = nh->declare_parameter<double>("framerate", 0.0);
    camera_fov_degrees = nh->declare_parameter<double>("fov_degrees", 90.0);
    host_ip = nh->declare_parameter<std::string>("host_ip", "localhost");
    depthcamera = nh->declare_parameter<bool>("depthcamera", false);
    rgbd_mode = nh->declare_parameter<bool>("rgbd_mode", false);

    if(camera_name == "") {
        RCLCPP_FATAL(nh->get_logger(), "camera_name unset.");
        return 1;
    }
    if(!std::isfinite(framerate) || framerate <= 0.0) {
        RCLCPP_FATAL(nh->get_logger(), "framerate must be greater than zero.");
        return 1;
    }
    if(!std::isfinite(camera_fov_degrees) ||
       camera_fov_degrees <= 0.0 || camera_fov_degrees >= 180.0) {
        RCLCPP_FATAL(nh->get_logger(), "fov_degrees must be within (0, 180).");
        return 1;
    }
    if(rgbd_mode && depth_camera_name.empty()) {
        RCLCPP_FATAL(nh->get_logger(), "depth_camera_name is required in rgbd_mode.");
        return 1;
    }

    // initialize fps counter
    fps_statistic = ros_bridge::Statistics("fps");

    // ready airsim connection
    msr::airlib::CarRpcLibClient client(host_ip, RpcLibPort, 5);
    airsim_api = &client;

    double timeout_sec = nh->declare_parameter<double>("timeout", 10.0);

    try {
        std::cout << "Waiting for connection - " << std::endl;
        airsim_api->confirmConnection(timeout_sec);
        std::cout << "Connected to the simulator!" << std::endl;
    } catch (const std::exception &e) {
        std::string msg = e.what();
        RCLCPP_ERROR(nh->get_logger(), "Exception raised by the API, something went wrong: %s\n", msg.c_str());
        return 1;
    }

    // ready topic
    const std::string image_topic = camera_topic_name +
        ((depthcamera && !rgbd_mode) ? "/image_depth" : "/image_color");

    image_pub = nh->create_publisher<sensor_msgs::msg::Image>(image_topic, 1);
    info_pub = nh->create_publisher<sensor_msgs::msg::CameraInfo>(camera_topic_name + "/camera_info", 1);
    if (rgbd_mode) {
        depth_image_pub = nh->create_publisher<sensor_msgs::msg::Image>(
            depth_camera_topic_name + "/image_depth", 1);
        depth_info_pub = nh->create_publisher<sensor_msgs::msg::CameraInfo>(
            depth_camera_topic_name + "/camera_info", 1);
    }

    // start the loop
    auto update_callback = rgbd_mode ? &doRgbdImageUpdate :
        (depthcamera ? &doDepthImageUpdate : &doImageUpdate);
    rclcpp::TimerBase::SharedPtr imageTimer = nh->create_wall_timer(
        dseconds { 1/framerate }, update_callback);
    rclcpp::TimerBase::SharedPtr fpsTimer = nh->create_wall_timer(dseconds { FPS_WINDOW }, [&nh](){
        RCLCPP_DEBUG(nh->get_logger(), "Average FPS: %d\n", fps_statistic.getCount()/FPS_WINDOW);
        fps_statistic.Reset();
    });
    rclcpp::spin(nh);
    return 0;
}
