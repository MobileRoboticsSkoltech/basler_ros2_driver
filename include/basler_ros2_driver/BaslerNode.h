//
// Created by devilox on 8/24/21.
//
//-----------------------------//
#ifndef BASLER_ROS2_BASLERNODE_H
#define BASLER_ROS2_BASLERNODE_H
//-----------------------------//
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <camera_calibration_parsers/parse.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
//-----------------------------//
#include <pylon/usb/BaslerUsbInstantCamera.h>
#include <pylon/usb/BaslerUsbInstantCamera.h>
#include <pylon/PylonIncludes.h>
//-----------------------------//
#include <yaml-cpp/yaml.h>
//-----------------------------//
class BaslerNode : public rclcpp::Node {
public:
    BaslerNode();
    ~BaslerNode();

    //----------//

    void publishFrame();
    void startLoop();
private:
    Pylon::CBaslerUsbInstantCamera*                                 mCamera                     = nullptr;
    Pylon::CImageFormatConverter                                    mFormatConverter;
    Pylon::CPylonImage                                              mPylonImage;
    Pylon::CGrabResultPtr                                           mGrabResultPrt;

    //----------//

    image_transport::CameraPublisher                                mPublisher;
    sensor_msgs::msg::CameraInfo                                    mCameraInfo;
    std::string                                                     mCameraName                 = "BaslerCamera";

    std::chrono::time_point <std::chrono::system_clock>             mStart;
    std::chrono::time_point <std::chrono::system_clock>             mEnd;
    uint16_t                                                        mFPS                        = 30;
};
//-----------------------------//
#endif //BASLER_ROS2_BASLERNODE_H
