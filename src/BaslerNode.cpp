//-----------------------------//
#include "basler_ros2_driver/BaslerNode.h"
//-----------------------------//
BaslerNode::BaslerNode() : Node("Basler") {
    Pylon::PylonInitialize();

    mCamera = new Pylon::CBaslerUsbInstantCamera(Pylon::CTlFactory::GetInstance().CreateFirstDevice());
    mCamera -> MaxNumBuffer = 5;
    mFormatConverter.OutputPixelFormat = Pylon::PixelType_RGB8packed;

    //----------//

    std::string PackagePath = ament_index_cpp::get_package_share_directory("basler_ros2");

    mPublisher = image_transport::create_camera_publisher(this, "basler/image");
    camera_calibration_parsers::readCalibration(PackagePath + "/config/basler_intrinsics_calibration.yaml", mCameraName, mCameraInfo);

    //----------//

    mCamera -> Open();
    mCamera -> StartGrabbing(Pylon::GrabStrategy_LatestImageOnly);

    //----------//

    YAML::Node Config = YAML::LoadFile(PackagePath + "/config/basler_params.yaml");
    mFPS = Config["fps"].as <int>();

    //----------//

    mStart = std::chrono::system_clock::now();
}
BaslerNode::~BaslerNode() {
    mCamera -> Close();

    if (mCamera) {
        delete(mCamera);
        mCamera = nullptr;
    }

    Pylon::PylonTerminate();
}
//-----------------------------//
void BaslerNode::publishFrame() {
    if (mCamera -> IsGrabbing()) {
        mCamera -> RetrieveResult(5000, mGrabResultPrt, Pylon::TimeoutHandling_ThrowException);

        if (mGrabResultPrt -> GrabSucceeded()) {
            mEnd = std::chrono::system_clock::now();
            std::chrono::duration <double> Dur(mEnd - mStart);

            if (Dur.count() > 1.0 / mFPS) {
                mStart = std::chrono::system_clock::now();
                mFormatConverter.Convert(mPylonImage, mGrabResultPrt);

                const uint8_t* Buffer = reinterpret_cast <uint8_t*> (mPylonImage.GetBuffer());
                const size_t Size = mGrabResultPrt -> GetImageSize() * 3;

                sensor_msgs::msg::Image RawImg;
                RawImg.header.stamp = this -> now();
                RawImg.header.frame_id = "/basler";
                RawImg.width = mGrabResultPrt -> GetWidth();
                RawImg.height = mGrabResultPrt -> GetHeight();
                RawImg.encoding = sensor_msgs::image_encodings::RGB8;
                RawImg.step = RawImg.width * 3;
                RawImg.data.assign(Buffer,Buffer + Size);

                mCameraInfo.header.stamp = RawImg.header.stamp;
                mCameraInfo.header.frame_id = RawImg.header.frame_id;

                mPublisher.publish(RawImg, mCameraInfo);
            }
        } else {
            std::cerr << "Runtime error" << std::endl;
        }
    }
}
void BaslerNode::startLoop() {
    while (rclcpp::ok()) {
        publishFrame();
    }
}
