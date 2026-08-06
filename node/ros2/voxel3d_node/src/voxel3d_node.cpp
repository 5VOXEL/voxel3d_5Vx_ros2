/****************************************************************************\
* Copyright (C) 2026 pmdtechnologies gmbh
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
* THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
* BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
* GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
* OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
\****************************************************************************/

#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>
#include <deque>
#include <chrono>
#include <unordered_map>
#include <sstream>
#include <iomanip>

#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

#include <opencv2/opencv.hpp>


#include "ParamHelper.hpp"
#include "FrameTimer.hpp"


#ifdef ROS1
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>

typedef ros::Publisher FramePublisher;
typedef ros::Publisher ImuPublisher;
typedef ros::Publisher PointCloudPublisher;
typedef ros::Publisher Voxel3dStrPublisher;

// ── Message type aliases for VOXEL3D_ADVERTISE ──────────────────────────
typedef sensor_msgs::Image        ImageMsg;
typedef sensor_msgs::Imu          ImuMsg;
typedef sensor_msgs::PointCloud2  PointCloud2Msg;
typedef std_msgs::String          StringMsg;
typedef std_msgs::Header          HeaderMsg;

#else
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_broadcaster.h>

typedef rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr FramePublisher;
typedef rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr ImuPublisher;
typedef rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr PointCloudPublisher;
typedef rclcpp::Publisher<std_msgs::msg::String>::SharedPtr Voxel3dStrPublisher;

// See the ROS1 branch above for why these aliases exist.
typedef sensor_msgs::msg::Image        ImageMsg;
typedef sensor_msgs::msg::Imu          ImuMsg;
typedef sensor_msgs::msg::PointCloud2  PointCloud2Msg;
typedef std_msgs::msg::String          StringMsg;
typedef std_msgs::msg::Header          HeaderMsg;
#endif

#include "SharedData.h"

static std::string formatFixed1(double value)
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(1) << value;
    return oss.str();
}


static std::string productNameFromSn(const char* dev_sn)
{
    std::string sn = dev_sn ? dev_sn : "";
    if (sn.find("ST03") != std::string::npos) return "ST03";
    if (sn.find("ST04") != std::string::npos) return "ST04";
    if (sn.find("5VHR") != std::string::npos) return "5HiRab";
    return "5HiRab";
}

// ── VOXEL3D_ADVERTISE ────────────────────────────────────────────────────
#ifdef ROS1
#define VOXEL3D_ADVERTISE(member, MsgType, topic) \
    member = nh.advertise<MsgType>(topic, 10)
#else
#define VOXEL3D_ADVERTISE(member, MsgType, topic) \
    member = node->create_publisher<MsgType>(topic, 10)
#endif

// ── Node/param-helper setup, spin loop, timestamps, TF, and publish calls ──
#ifdef ROS1
#define VOXEL3D_INIT_NODE() \
    ros::NodeHandle nh
#define VOXEL3D_PARAM_HELPER(varname) \
    ParamHelper varname(ros::NodeHandle("~"))
typedef ros::Rate RosRate;
#define VOXEL3D_OK()                ros::ok()
#define VOXEL3D_SPIN_SOME()         ros::spinOnce()
#define VOXEL3D_NOW()               ros::Time::now()
#define VOXEL3D_TIME_FROM_NSEC(ns)  ros::Time().fromNSec(ns)
#define VOXEL3D_PUBLISH(publisher, msg)        (publisher).publish(msg)
#define VOXEL3D_PUBLISH_IMG(publisher, msgPtr) (publisher).publish(msgPtr)
#define VOXEL3D_NUM_SUBSCRIBERS(publisher)     (publisher).getNumSubscribers()
// map -> voxel3d_frame TF, sent every publish_frames() call.
#define VOXEL3D_SEND_TF(originZ) \
    do { \
        tf::Transform transform; \
        transform.setOrigin(tf::Vector3(0.0, 0.0, (originZ))); \
        tf::Quaternion q(-0.7071068f, 0.0f, 0.0f, 0.7071068f); \
        transform.setRotation(q); \
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "voxel3d_frame")); \
    } while (0)
#else
#define VOXEL3D_INIT_NODE() \
    node = std::make_shared<rclcpp::Node>("voxel3d_node"); \
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node)
#define VOXEL3D_PARAM_HELPER(varname) \
    ParamHelper varname(node)
typedef rclcpp::Rate RosRate;
#define VOXEL3D_OK()                rclcpp::ok()
#define VOXEL3D_SPIN_SOME()         rclcpp::spin_some(node)
#define VOXEL3D_NOW()               node->get_clock()->now()
#define VOXEL3D_TIME_FROM_NSEC(ns)  rclcpp::Time(static_cast<int64_t>(ns))
#define VOXEL3D_PUBLISH(publisher, msg)        (publisher)->publish(msg)
#define VOXEL3D_PUBLISH_IMG(publisher, msgPtr) (publisher)->publish(*(msgPtr))
#define VOXEL3D_NUM_SUBSCRIBERS(publisher)     (publisher)->get_subscription_count()
#define VOXEL3D_SEND_TF(originZ) \
    do { \
        geometry_msgs::msg::TransformStamped t; \
        t.header.stamp    = VOXEL3D_NOW(); \
        t.header.frame_id = "map"; \
        t.child_frame_id  = "voxel3d_frame"; \
        t.transform.translation.x = 0.0; \
        t.transform.translation.y = 0.0; \
        t.transform.translation.z = (originZ); \
        t.transform.rotation.x = -0.7071068f; \
        t.transform.rotation.y =  0.0f; \
        t.transform.rotation.z =  0.0f; \
        t.transform.rotation.w =  0.7071068f; \
        tf_broadcaster_->sendTransform(t); \
    } while (0)
#endif

class CameraNode {
  public:
    CameraNode() {
        VOXEL3D_INIT_NODE();

        VOXEL3D_ADVERTISE(pc_ir_publisher, PointCloud2Msg, "camera/frame_pc_ir");
        VOXEL3D_ADVERTISE(pc_rgb_publisher, PointCloud2Msg, "camera/frame_pc_rgb");
        VOXEL3D_ADVERTISE(pc_thermal_publisher, PointCloud2Msg, "camera/frame_pc_thermal");

        VOXEL3D_ADVERTISE(depth_publisher,   ImageMsg, "camera/frame_depth");
        VOXEL3D_ADVERTISE(ir_publisher,      ImageMsg, "camera/frame_ir");
        VOXEL3D_ADVERTISE(rgb_publisher,     ImageMsg, "camera/frame_rgb");
        VOXEL3D_ADVERTISE(thermal_publisher, ImageMsg, "camera/frame_thermal");

        VOXEL3D_ADVERTISE(accel_publisher, ImuMsg, "imu/accel");
        VOXEL3D_ADVERTISE(gyro_publisher,  ImuMsg, "imu/gyro");

        VOXEL3D_ADVERTISE(sensorName_publisher,        StringMsg, "voxel3d/name");
        VOXEL3D_ADVERTISE(sensorSn_publisher,          StringMsg, "voxel3d/sn");
        VOXEL3D_ADVERTISE(sensorFwVersion_publisher,   StringMsg, "voxel3d/fw_version");
        VOXEL3D_ADVERTISE(sensorFwBuildDate_publisher, StringMsg, "voxel3d/fw_build_date");
        VOXEL3D_ADVERTISE(sensorLibVersion_publisher,  StringMsg, "voxel3d/lib_version");

        VOXEL3D_ADVERTISE(infoRgbRes_publisher, StringMsg, "voxel3d/info_rgb_res");
        VOXEL3D_ADVERTISE(infoRgbFov_publisher, StringMsg, "voxel3d/info_rgb_fov");
        VOXEL3D_ADVERTISE(infoRgbFps_publisher, StringMsg, "voxel3d/info_rgb_fps");

        VOXEL3D_ADVERTISE(infoDepthRes_publisher, StringMsg, "voxel3d/info_depth_res");
        VOXEL3D_ADVERTISE(infoDepthFov_publisher, StringMsg, "voxel3d/info_depth_fov");
        VOXEL3D_ADVERTISE(infoDepthFps_publisher, StringMsg, "voxel3d/info_depth_fps");

        VOXEL3D_ADVERTISE(infoThermalRes_publisher, StringMsg, "voxel3d/info_thermal_res");
        VOXEL3D_ADVERTISE(infoThermalFov_publisher, StringMsg, "voxel3d/info_thermal_fov");
        VOXEL3D_ADVERTISE(infoThermalFps_publisher, StringMsg, "voxel3d/info_thermal_fps");

// ── Load Acaas algorithm parameters (flat, read once at startup) ──
        VOXEL3D_PARAM_HELPER(voxel3d_params);
        flirDisMax = voxel3d_params.getParam<double>("voxel3d.thermal_max_temp", flirDisMax);
        flirDisMin = voxel3d_params.getParam<double>("voxel3d.thermal_min_temp", flirDisMin);
        int fusionMode = voxel3d_params.getParam<int>("voxel3d.fusion_mode", 0);
        depthMaxRange = voxel3d_params.getParam<double>("voxel3d.depth_max_range", depthMaxRange);

        m_undistort = false;

        // NONE     = 0,   /* Disable image rectification */
        // RGB2TOF  = 1,   /* Software RGB to Depth image rectification */
        // FLIR2TOF = 2,   /* Software FLIR to Depth image rectification */
        // TOF2RGB  = 3 
        if (fusionMode == 3)
            m_doRectify = RectifyType::TOF2RGB;
        else if (fusionMode == 2)
            m_doRectify = RectifyType::FLIR2TOF;
        else if (fusionMode == 1)
            m_doRectify = RectifyType::RGB2TOF;
        else
            m_doRectify = RectifyType::NONE;
    

        if (sensorScan(camInfo, dev_sn)) {
            if (sensorInit(dev_sn, _sensorInfo)) {
                m_isConnected.store(true);
                pixelCount = _sensorInfo.tofInfo.Width * _sensorInfo.tofInfo.Height;
                pointcloudData = new float[pixelCount * 3];
                pointcloudDataResized = new float[pixelCount * 3];
                return;
            }
            else
                std::cerr << "[ ERROR ][ 5VOXEL ] 5Voxel Device Initialization Failed. Please reconnect the sensor and restart the application." << std::endl;
        }
        else
            std::cerr << "[ ERROR ][ 5VOXEL ] No 5Voxel device found. Please ensure the sensor is securely connected." << std::endl;
        m_isConnected.store(false);
    }

#undef VOXEL3D_ADVERTISE
#undef VOXEL3D_INIT_NODE
#undef VOXEL3D_PARAM_HELPER


    ~CameraNode() {
        m_isRunning.store(false);
        m_captureCv.notify_all();
        if (m_captureThread.joinable()) {
            m_captureThread.join();
        }
        if (m_processThread.joinable()) {
            m_processThread.join();
        }

        if (_sensorInfo.tofInfo.IsExist || _sensorInfo.rgbInfo.IsExist || _sensorInfo.flirInfo.IsExist) {
            voxel3d_release(dev_sn);
        }
        m_isConnected.store(false);

        delete[] pointcloudData;
        delete[] pointcloudDataResized;
    }

    bool isConnected() const { return m_isConnected.load(); }


    // ────────────────────────────────────────────────────────────────────────
    // ROS spin loop
    // ────────────────────────────────────────────────────────────────────────
    void publish_loop() {
        m_isRunning.store(true);

        m_captureThread = std::thread(&CameraNode::captureLoop, this);
        m_processThread = std::thread(&CameraNode::processLoop, this);

        RosRate rate(10);
        while (VOXEL3D_OK()) {
            publish_frames();
            VOXEL3D_SPIN_SOME();
            rate.sleep();
        }
        m_isRunning.store(false);
        m_captureCv.notify_all();
        if (m_captureThread.joinable()) {
            m_captureThread.join();
        }
        if (m_processThread.joinable()) {
            m_processThread.join();
        }
    }

  private:

    // Voxed3d Scanner
    bool sensorScan(CamDevInfo& camInfo, char* dev_sn)
    {
        const std::string logTag = (dev_sn && dev_sn[0]) ? (productNameFromSn(dev_sn) + " " + dev_sn) : std::string("5HiRab");

        bool ret = voxel3d_scan(&camInfo);

        printf("\n[ INFO. ][ %s ] Found %d devices\n", logTag.c_str(), camInfo.num_of_devices);
        if (camInfo.num_of_devices > 0) {
            char dev_name[MAX_DEV_NAME_LEN];
            for (int ix = 0; ix < camInfo.num_of_devices; ix++) {

                sensorName = camInfo.dev_info[ix].dev_name;
                sensorSn = camInfo.dev_info[ix].product_sn;

                printf("\n%d: Name = %s, SN = %s\n", ix, camInfo.dev_info[ix].dev_name, camInfo.dev_info[ix].product_sn);
                printf("    Available frame format(s): %d\n", camInfo.dev_info[ix].frame_fmts.avail_frame_num);
                for (int iy = 0; iy < camInfo.dev_info[ix].frame_fmts.avail_frame_num; iy++) {
                    printf("        %d: width = %d, height = %d, fps = %.1f, format = %s %s\n",
                        iy,
                        camInfo.dev_info[ix].frame_fmts.fmt[iy].width,
                        camInfo.dev_info[ix].frame_fmts.fmt[iy].height,
                        camInfo.dev_info[ix].frame_fmts.fmt[iy].fps,
                        camInfo.dev_info[ix].frame_fmts.fmt[iy].fmt == VIDEO_MODE_YUY2 ? "YUY2" : "MJPG",
                        camInfo.dev_info[ix].frame_fmts.default_frame_index == iy ? "[Default]" : "");
                }
            }

            printf("\n[ INFO. ][ %s ] Embeded IMU: %s\n", logTag.c_str(), camInfo.imu_info.is_exist ? "Exist" : "Absent");
            if (camInfo.imu_info.is_exist) {
                printf("    Coordinate: %s\n",
                    camInfo.imu_info.imu_coordinate == IMU_COORDINATE_RIGHT_HAND ? "Right-hand" :
                    camInfo.imu_info.imu_coordinate == IMU_COORDINATE_LEFT_HAND ? "Left-hand" : "Unknown");
                printf("    Axis with Gravity: %s\n",
                    camInfo.imu_info.imu_axis_with_gravity == IMU_GRAVITY_ON_X ? "X" :
                    camInfo.imu_info.imu_axis_with_gravity == IMU_GRAVITY_ON_NEGATIVE_X ? "-X" :
                    camInfo.imu_info.imu_axis_with_gravity == IMU_GRAVITY_ON_Y ? "Y" :
                    camInfo.imu_info.imu_axis_with_gravity == IMU_GRAVITY_ON_NEGATIVE_Y ? "-Y" :
                    camInfo.imu_info.imu_axis_with_gravity == IMU_GRAVITY_ON_Z ? "Z" : "-Z"
                );
            }
        }
        else {
            printf("[ ERROR ] Can't find any 5Voxel device\n");
        }

        return ret;
    }

    /// <summary>
    /// 5Voxel Sensor initialization.
    /// </summary>
    /// <param name="dev_sn">Device serial number</param>
    /// <returns></returns>
    bool sensorInit(char* dev_sn, hiRabSensorInfo& _sensorInfo)
    {
        const std::string logTag = (dev_sn && dev_sn[0]) ? (productNameFromSn(dev_sn) + " " + dev_sn) : std::string("5HiRab");

        CamInitSettings tof_default_setting = { 0, 0, VIDEO_MODE_YUY2 }; //use default format

        if (!_sensorInfo.tofInfo.IsExist) {
            int result;
            result = voxel3d_tof_init(dev_sn, tof_default_setting);
            if (result > 0) {
                _sensorInfo.tofInfo.Width = voxel3d_tof_get_width(dev_sn);
                _sensorInfo.tofInfo.Height = voxel3d_tof_get_height(dev_sn);

                
                _sensorInfo.tofInfo.hfov = voxel3d_tof_get_depth_hfov(dev_sn);
                _sensorInfo.tofInfo.vfov = voxel3d_tof_get_depth_vfov(dev_sn);
              
                printf("\n[ INFO. ][ %s ] Get ToF Sensor, resolution: [ %d x %d ]\n", logTag.c_str(), _sensorInfo.tofInfo.Width, _sensorInfo.tofInfo.Height);

                memset(&_sensorInfo.tofInfo.Info, 0x0, sizeof(_sensorInfo.tofInfo.Info));
                voxel3d_tof_read_camera_info(dev_sn, &_sensorInfo.tofInfo.Info);

                // Lens parameters
                fillSensorParams(_sensorInfo.tofInfo.Params, _sensorInfo.tofInfo.Info);

                {
                    int ret;
                    char data[64];

                    memset(data, 0x0, sizeof(data));
                    ret = voxel3d_read_lib_version(data, sizeof(data));
                    if (ret < 0) {
                        printf("[ INFO. ][ %s ] Share library version read failed (err: %d)\n", logTag.c_str(), ret);
                    }
                    else {
                        printf("[ INFO. ][ %s ] Share library version  : %s\n", logTag.c_str(), data);
                        std::string s(data);
                        sensorLibVersion = s;
                    }

                    memset(data, 0x0, sizeof(data));
                    ret = voxel3d_read_fw_version(dev_sn, data, sizeof(data));
                    if (ret < 0) {
                        printf("[ INFO. ][ %s ] Device F/W version read failed (err: %d)\n", logTag.c_str(), ret);
                    }
                    else {
                        printf("[ INFO. ][ %s ] Device F/W version    : %s\n", logTag.c_str(), data);
                        std::string s(data);
                        sensorFwVersion = s;
                    }

                    memset(data, 0x0, sizeof(data));
                    ret = voxel3d_read_fw_build_date(dev_sn, data, sizeof(data));
                    if (ret < 0) {
                        printf("[ INFO. ][ %s ] Device F/W build date read failed (err: %d)\n", logTag.c_str(), ret);
                    }
                    else {
                        printf("[ INFO. ][ %s ] Device F/W build date : %s\n", logTag.c_str(), data);
                        std::string s(data);
                        sensorFwBuildDate = s;
                    }
                }
                _sensorInfo.tofInfo.IsExist = true;
            }
            //voxel3d_tof_release(dev_sn);
        }


        if (!_sensorInfo.rgbInfo.IsExist) {
            int result;
            result = voxel3d_rgb_init(dev_sn);
            if (result > 0) {
                int _rgbWidth = voxel3d_rgb_get_width(dev_sn);
                int _rgbHeight =  voxel3d_rgb_get_height(dev_sn);

                _sensorInfo.rgbInfo.hfov = voxel3d_rgb_get_hfov(dev_sn);
                _sensorInfo.rgbInfo.vfov = voxel3d_rgb_get_vfov(dev_sn);

                rgb = cv::Mat(1, _rgbHeight * _rgbWidth * 3, CV_8UC1, cv::Scalar(0)); // Linux

                _sensorInfo.rgbInfo.Width =  _rgbWidth;
                _sensorInfo.rgbInfo.Height = _rgbHeight;

                printf("[ INFO. ][ %s ] RGB Sensor, resolution: [ %d x %d ]\n", logTag.c_str(), _sensorInfo.rgbInfo.Width, _sensorInfo.rgbInfo.Height);

                memset(&_sensorInfo.rgbInfo.Info, 0x0, sizeof(_sensorInfo.rgbInfo.Info));
                voxel3d_rgb_read_camera_info(dev_sn, &_sensorInfo.rgbInfo.Info);

                // Lens parameters
                fillSensorParams(_sensorInfo.rgbInfo.Params, _sensorInfo.rgbInfo.Info);

                _sensorInfo.rgbInfo.IsExist = true;
            }
            //voxel3d_rgb_release(dev_sn);
        }

        if (!_sensorInfo.flirInfo.IsExist) {
            int result;
            result = voxel3d_lepton3_init(dev_sn);

            if (result > 0) {

                int _flirWidth = voxel3d_lepton3_get_width(dev_sn);
                int _flirHeight = voxel3d_lepton3_get_height(dev_sn);

                _sensorInfo.flirInfo.hfov = voxel3d_lepton3_get_hfov(dev_sn);
                _sensorInfo.flirInfo.vfov = voxel3d_lepton3_get_vfov(dev_sn);

                flir = cv::Mat(_flirHeight, _flirWidth, CV_32FC1, cv::Scalar(0));

                _sensorInfo.flirInfo.Width =  _flirWidth;
                _sensorInfo.flirInfo.Height = _flirHeight;


                printf("[ INFO. ][ %s ] Flir Sensor, resolution: [ %d x %d ]\n", logTag.c_str(), _sensorInfo.flirInfo.Width, _sensorInfo.flirInfo.Height);

                _sensorInfo.flirInfo.IsExist = true;
            }
        }

        for (int ix = 0; ix < camInfo.num_of_devices; ix++) {
            sensorName = camInfo.dev_info[ix].dev_name;
            sensorSn = camInfo.dev_info[ix].product_sn;

            if (strcmp(camInfo.dev_info[ix].dev_name, "FLIR") == 0)
                _sensorInfo.flirInfo.fps = camInfo.dev_info[ix].frame_fmts.fmt[0].fps;
            else if (strcmp(camInfo.dev_info[ix].dev_name, "HE-2 RGB Decoder") == 0 || strcmp(camInfo.dev_info[ix].dev_name, "5VSTDON RGB Camera") == 0)
                _sensorInfo.rgbInfo.fps = camInfo.dev_info[ix].frame_fmts.fmt[0].fps;
            else if (strcmp(camInfo.dev_info[ix].dev_name, "HE-2 ToF Decoder") == 0 || strcmp(camInfo.dev_info[ix].dev_name, "5VSTDON ToF Camera") == 0)
                _sensorInfo.tofInfo.fps = camInfo.dev_info[ix].frame_fmts.fmt[0].fps;
        }



        voxel3d_set_rectifyType(dev_sn, m_doRectify);
        if (m_doRectify == 3)
            printf("\n[ INFO. ][ %s ] Set rectifyType [ TOF2RGB ] COMPLETED.\n\n", logTag.c_str());
        else if (m_doRectify == 2)
            printf("\n[ INFO. ][ %s ] Set rectifyType [ FLIR2TOF ] COMPLETED.\n\n", logTag.c_str());
        else if (m_doRectify == 1)
            printf("\n[ INFO. ][ %s ] Set rectifyType [ RGB2TOF ] COMPLETED.\n\n", logTag.c_str());
        else
            printf("\n[ INFO. ][ %s ] Set rectifyType [ NONE ] COMPLETED.\n\n", logTag.c_str());


        printf("\n[ INFO. ][ %s ] Sensor initialization COMPLETED.\n\n", logTag.c_str());
        printf("\n[ INFO. ][ %s ] TOF [ %s ], RGB [ %s ], Flir [ %s ]\n",
            logTag.c_str(),
            _sensorInfo.tofInfo.IsExist ? "Active" : "Inactive",
            _sensorInfo.rgbInfo.IsExist ? "Active" : "Inactive",
            _sensorInfo.flirInfo.IsExist ? "Active" : "Inactive");

        return _sensorInfo.tofInfo.IsExist;
    }


    void captureLoop(){
        cv::Mat ironbowLUT = createIronLUT();

        float thermalRange = flirDisMax - flirDisMin;
        if (thermalRange < 1e-6f) thermalRange = 1.0f;
        float thermalAlpha = 255.0f / thermalRange;
        float thermalBeta = -flirDisMin * thermalAlpha;


        while (m_isRunning.load()) {
            auto t_captureStart = std::chrono::steady_clock::now();

            auto _sensorData = std::make_shared<SensorData>(
            _sensorInfo.tofInfo.Width,
            _sensorInfo.tofInfo.Height,
            _sensorInfo.rgbInfo.Width,
            _sensorInfo.rgbInfo.Height,
            _sensorInfo.flirInfo.Width,
            _sensorInfo.flirInfo.Height);
            
            unsigned int ret = voxel3d_tof_queryframe(dev_sn,
            _sensorData->depth.ptr<unsigned short>(0),
            _sensorData->conf.ptr<unsigned short>(0)); 
            
            if (ret > 0) {
                ret = voxel3d_tof_generatePointCloud(dev_sn,
                    _sensorData->depth.ptr<unsigned short>(0),
                    _sensorData->pointCloudXYZ.ptr<float>(0));

                voxel3d_read_imu_data(dev_sn, &_sensorData->imuData);
                if (_sensorInfo.flirInfo.IsExist)
                {
                    if (m_doRectify == RectifyType::FLIR2TOF)
                    {
                        retFlir = voxel3d_lepton3_queryframe(dev_sn, _sensorData->flir.ptr<float>(0));
                    }
                    else
                    {
                        retFlir = voxel3d_lepton3_queryframe(dev_sn, flir.ptr<float>(0));
                        
                        if (retFlir > 0) {
                            cv::resize(flir, _sensorData->flir, _sensorData->flir.size(), 0, 0, cv::INTER_LINEAR);
                            _sensorData->flir.copyTo(lastValidFlir);
                        }
                        else if (!lastValidFlir.empty())
                        {
                            lastValidFlir.copyTo(_sensorData->flir);
                        }
                    }
                }

                cv::Mat decodedRGB;
                if (_sensorInfo.rgbInfo.IsExist)
                {
                    if (m_doRectify == RectifyType::RGB2TOF)
                    {
                        /*
                        * Not supported. Suggest to use TOF2RGB for RGB-D fusion
                        */
                    }
                    else {
                        ret = voxel3d_rgb_queryframe(dev_sn, rgb.ptr<uchar>(0));
                        if (ret > 0) {
                            decodedRGB = cv::imdecode(rgb, cv::IMREAD_COLOR);
                            if (!decodedRGB.empty()) {
                                _sensorData->rgb = decodedRGB.clone();
                                _sensorData->rgb.copyTo(lastValidRgb);
                            }
                            else if (!lastValidRgb.empty())
                            {
                                lastValidRgb.copyTo(_sensorData->rgb);
                            }
                        }
                        else if (!lastValidRgb.empty())
                        {
                            lastValidRgb.copyTo(_sensorData->rgb);
                        }
                    }
                }

                auto t_captureEnd = std::chrono::steady_clock::now();

                auto captured = std::make_shared<CapturedFrame>();
                captured->t_captureStart = t_captureStart;
                captured->t_captureEnd = t_captureEnd;

                double effectiveMaxRange = (depthMaxRange > 0.0) ? depthMaxRange : 10000.0;
                cv::Mat depthClamped;
                cv::min(_sensorData->depth, effectiveMaxRange, depthClamped);

                cv::Mat depth8;
                double depthAlpha = 255.0 / effectiveMaxRange;
                cv::convertScaleAbs(depthClamped, depth8, depthAlpha);
                cv::applyColorMap(depth8, captured->depthRGB, cv::COLORMAP_JET);

                // Confidence map standing in for "IR" (see note above) -- same
                // saturation issue as depth above, same fix.
                cv::Mat conf8;
                cv::normalize(_sensorData->conf, conf8, 0, 255, cv::NORM_MINMAX, CV_8UC1);
                cv::cvtColor(conf8, captured->irBGR, cv::COLOR_GRAY2BGR);

                if (_sensorInfo.rgbInfo.IsExist)
                {
                    captured->rgbResized = _sensorData->rgb.clone();
                }


                if(_sensorInfo.flirInfo.IsExist)
                {
                    cv::Mat flirF;
                    _sensorData->flir.convertTo(flirF, CV_32F);

                    flirF.convertTo(flirF, CV_8UC1, thermalAlpha, thermalBeta);
                    cv::applyColorMap(flirF, captured->thermalResized, ironbowLUT);
                }

                captured->hasImu = true;
                captured->imuAcceleration[0] = _sensorData->imuData.imu_accel[0];
                captured->imuAcceleration[1] = _sensorData->imuData.imu_accel[1];
                captured->imuAcceleration[2] = _sensorData->imuData.imu_accel[2];
                captured->imuAngularVelocity[0] = _sensorData->imuData.imu_gyro[0];
                captured->imuAngularVelocity[1] = _sensorData->imuData.imu_gyro[1];
                captured->imuAngularVelocity[2] = _sensorData->imuData.imu_gyro[2];
                captured->imuAngularTimestamp = _sensorData->imuData.imu_ts;

                captured->hasPc = true;
                captured->pcWidth = _sensorInfo.tofInfo.Width;
                captured->pcHeight = _sensorInfo.tofInfo.Height;
                int numPoints = captured->pcWidth * captured->pcHeight;
                float* pcPtr = _sensorData->pointCloudXYZ.ptr<float>(0);
                captured->pointcloudRaw.assign(pcPtr, pcPtr + 3 * numPoints);

                captured->valid = true;
                {
                    std::lock_guard<std::mutex> lock(m_captureMutex);
                    m_latestCapturedFrame = captured;
                    m_captureFrameReady = true;
                }
                m_captureCv.notify_one();
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        m_captureCv.notify_all();
    }

    // ────────────────────────────────────────────────────────────────────────
    //
    // Processing thread: waits for whatever captureLoop() last produced
    //
    // ────────────────────────────────────────────────────────────────────────
    void processLoop(){
        while (m_isRunning.load()) {
            std::shared_ptr<CapturedFrame> frame;
            {
                std::unique_lock<std::mutex> lock(m_captureMutex);
                m_captureCv.wait(lock, [this] { return m_captureFrameReady || !m_isRunning.load(); });
                if (!m_captureFrameReady) {
                    continue; // woken up only because we're shutting down
                }
                frame = m_latestCapturedFrame;
                m_captureFrameReady = false;
            }
            if (!frame || !frame->valid) continue;

            cv::Mat zImageRGB     = frame->depthRGB;
            cv::Mat irImageBGR    = frame->irBGR;
            cv::Mat picBGRResized = frame->rgbResized;
            cv::Mat ThermalResized = frame->thermalResized;

    
            LatestImu imuCopy;
            if (frame->hasImu) {
                imuCopy.valid = true;
                imuCopy.acceleration[0] = frame->imuAcceleration[0];
                imuCopy.acceleration[1] = frame->imuAcceleration[1];
                imuCopy.acceleration[2] = frame->imuAcceleration[2];
                imuCopy.angularVelocity[0] = frame->imuAngularVelocity[0];
                imuCopy.angularVelocity[1] = frame->imuAngularVelocity[1];
                imuCopy.angularVelocity[2] = frame->imuAngularVelocity[2];
                imuCopy.angularTimestamp = frame->imuAngularTimestamp;

                ax = -frame->imuAcceleration[0];
                ay = frame->imuAcceleration[1];
                az = -frame->imuAcceleration[2];
                wx = -frame->imuAngularVelocity[0];
                wy = frame->imuAngularVelocity[1];
                wz = -frame->imuAngularVelocity[2];

                for (int ix = 0; ix < pixelCount; ix++)
                {
                    int pcl_idx = ix * 3;
                    pointcloudData[pcl_idx] = frame->pointcloudRaw[pcl_idx];
                    pointcloudData[pcl_idx + 1] = frame->pointcloudRaw[pcl_idx + 1];
                    pointcloudData[pcl_idx + 2] = frame->pointcloudRaw[pcl_idx + 2];
                }
            }

            LatestPointCloud pcCopy;

            if (frame->hasPc) {
                pcCopy.valid = true;
                pcCopy.width = frame->pcWidth;
                pcCopy.height = frame->pcHeight;
                int numPoints = frame->pcWidth * frame->pcHeight;
                if (numPoints > 0) {
                    pcCopy.data.resize(3 * numPoints);

                    float* srcDisPtr = pointcloudData;
                    float* dstDisPtr = pcCopy.data.data();
                    for (int ix = 0; ix < numPoints; ix++)
                    {
                        float x = *srcDisPtr++;
                        float y = *srcDisPtr++;
                        float z = *srcDisPtr++;

                        *dstDisPtr++ = x;
                        *dstDisPtr++ = y;
                        *dstDisPtr++ = z;
                    }
                }
            }

            {
                std::lock_guard<std::mutex> lock(m_latestFrameMutex);
                m_latestPc = pcCopy;
                m_latestImu = imuCopy;
                m_latestDepthRGB = zImageRGB;
                m_latestIrBGR = irImageBGR;
                m_latestRgbResized = picBGRResized;
                m_latestThermalResized = ThermalResized;
            }

            m_latestFrameReady.store(true);
        }
    }

    // ────────────────────────────────────────────────────────────────────────
    // ROS publish (main loop, 10 Hz)
    // ────────────────────────────────────────────────────────────────────────
    void publish_frames() {

        VOXEL3D_SEND_TF(sensorHeight * 0.001f);

        if (!m_latestFrameReady.load()) {
            return;
        }

        LatestPointCloud pc;
        LatestImu imu;
        cv::Mat zImageRGB, irImageBGR, picBGRResized, ThermalResized;

        {
            std::lock_guard<std::mutex> lock(m_latestFrameMutex);
            pc = m_latestPc;
            imu = m_latestImu;
            zImageRGB = m_latestDepthRGB;
            irImageBGR = m_latestIrBGR;
            picBGRResized = m_latestRgbResized;
            ThermalResized = m_latestThermalResized;
        }

        HeaderMsg header;
        header.stamp = VOXEL3D_NOW();
        header.frame_id = "voxel3d_frame"; // vital to foxy

        auto msg_depth = cv_bridge::CvImage(header, "bgr8", zImageRGB).toImageMsg();
        VOXEL3D_PUBLISH_IMG(depth_publisher, msg_depth);

        auto msg_ir = cv_bridge::CvImage(header, "bgr8", irImageBGR).toImageMsg();
        VOXEL3D_PUBLISH_IMG(ir_publisher, msg_ir);

        auto msg_rgb = cv_bridge::CvImage(header, "bgr8", picBGRResized).toImageMsg();
        VOXEL3D_PUBLISH_IMG(rgb_publisher, msg_rgb);

        auto msg_thermal = cv_bridge::CvImage(header, "bgr8", ThermalResized).toImageMsg();
        VOXEL3D_PUBLISH_IMG(thermal_publisher, msg_thermal);

        if (pc.valid) {
            if (VOXEL3D_NUM_SUBSCRIBERS(pc_ir_publisher) > 0)
                publishColoredPointCloud(pc, irImageBGR, pc_ir_publisher);
            if (VOXEL3D_NUM_SUBSCRIBERS(pc_rgb_publisher) > 0)
                publishColoredPointCloud(pc, picBGRResized, pc_rgb_publisher);
            if (VOXEL3D_NUM_SUBSCRIBERS(pc_thermal_publisher) > 0)
                publishColoredPointCloud(pc, ThermalResized, pc_thermal_publisher);
        }

        // IMU
        if (imu.valid) {
            ImuMsg msg_imu_accel;
            msg_imu_accel.header.stamp = VOXEL3D_NOW();
            msg_imu_accel.header.frame_id = "voxel3d_imu";
            msg_imu_accel.linear_acceleration.x = imu.acceleration[0];
            msg_imu_accel.linear_acceleration.y = imu.acceleration[1];
            msg_imu_accel.linear_acceleration.z = imu.acceleration[2];
            // orientation unknown
            msg_imu_accel.orientation_covariance[0] = -1.0;

            ImuMsg msg_imu_gyro;
            msg_imu_gyro.header.stamp = VOXEL3D_TIME_FROM_NSEC(imu.angularTimestamp);
            msg_imu_gyro.header.frame_id = "voxel3d_imu";
            msg_imu_gyro.angular_velocity.x = imu.angularVelocity[0];
            msg_imu_gyro.angular_velocity.y = imu.angularVelocity[1];
            msg_imu_gyro.angular_velocity.z = imu.angularVelocity[2];
            // orientation unknown
            msg_imu_gyro.orientation_covariance[0] = -1.0;

            VOXEL3D_PUBLISH(accel_publisher, msg_imu_accel);
            VOXEL3D_PUBLISH(gyro_publisher, msg_imu_gyro);
        }

        // Sensor Params
        StringMsg msg_voxel3dStr;

        // Sensor Information
        msg_voxel3dStr.data = std::to_string(_sensorInfo.rgbInfo.Width) + " x " + std::to_string(_sensorInfo.rgbInfo.Height);
        VOXEL3D_PUBLISH(infoRgbRes_publisher, msg_voxel3dStr);
        msg_voxel3dStr.data = formatFixed1(_sensorInfo.rgbInfo.hfov*57.29577952383886f) + " x " + formatFixed1(_sensorInfo.rgbInfo.vfov*57.29577952383886f);
        VOXEL3D_PUBLISH(infoRgbFov_publisher, msg_voxel3dStr);
        msg_voxel3dStr.data = formatFixed1(_sensorInfo.rgbInfo.fps);
        VOXEL3D_PUBLISH(infoRgbFps_publisher, msg_voxel3dStr);

        msg_voxel3dStr.data = std::to_string(_sensorInfo.tofInfo.Width) + " x " + std::to_string(_sensorInfo.tofInfo.Height);
        VOXEL3D_PUBLISH(infoDepthRes_publisher, msg_voxel3dStr);
        msg_voxel3dStr.data = formatFixed1(_sensorInfo.tofInfo.hfov*57.29577952383886f) + " x " + formatFixed1(_sensorInfo.tofInfo.vfov*57.29577952383886f);
        VOXEL3D_PUBLISH(infoDepthFov_publisher, msg_voxel3dStr);
        msg_voxel3dStr.data = formatFixed1(_sensorInfo.tofInfo.fps/2.0f);
        VOXEL3D_PUBLISH(infoDepthFps_publisher, msg_voxel3dStr);

        msg_voxel3dStr.data = std::to_string(_sensorInfo.flirInfo.Width) + " x " + std::to_string(_sensorInfo.flirInfo.Height);
        VOXEL3D_PUBLISH(infoThermalRes_publisher, msg_voxel3dStr);
        msg_voxel3dStr.data = formatFixed1(_sensorInfo.flirInfo.hfov*57.29577952383886f) + " x " + formatFixed1(_sensorInfo.flirInfo.vfov*57.29577952383886f);
        VOXEL3D_PUBLISH(infoThermalFov_publisher, msg_voxel3dStr);
        msg_voxel3dStr.data = formatFixed1(_sensorInfo.flirInfo.fps);
        VOXEL3D_PUBLISH(infoThermalFps_publisher, msg_voxel3dStr);

        // Sensor Params
        msg_voxel3dStr.data = sensorName;
        VOXEL3D_PUBLISH(sensorName_publisher, msg_voxel3dStr);

        msg_voxel3dStr.data = sensorSn;
        VOXEL3D_PUBLISH(sensorSn_publisher, msg_voxel3dStr);

        msg_voxel3dStr.data = sensorFwVersion;
        VOXEL3D_PUBLISH(sensorFwVersion_publisher, msg_voxel3dStr);

        msg_voxel3dStr.data = sensorFwBuildDate;
        VOXEL3D_PUBLISH(sensorFwBuildDate_publisher, msg_voxel3dStr);

        msg_voxel3dStr.data = sensorLibVersion;
        VOXEL3D_PUBLISH(sensorLibVersion_publisher, msg_voxel3dStr);
    }

    PointCloudPublisher pc_ir_publisher;
    PointCloudPublisher pc_rgb_publisher;
    PointCloudPublisher pc_thermal_publisher;

    FramePublisher depth_publisher;
    FramePublisher ir_publisher;
    FramePublisher rgb_publisher;
    FramePublisher thermal_publisher;

    ImuPublisher accel_publisher;
    ImuPublisher gyro_publisher;

    Voxel3dStrPublisher infoRgbRes_publisher;
    Voxel3dStrPublisher infoRgbFov_publisher;
    Voxel3dStrPublisher infoRgbFps_publisher;

    Voxel3dStrPublisher infoDepthRes_publisher;
    Voxel3dStrPublisher infoDepthFov_publisher;
    Voxel3dStrPublisher infoDepthFps_publisher;

    Voxel3dStrPublisher infoThermalRes_publisher;
    Voxel3dStrPublisher infoThermalFov_publisher;
    Voxel3dStrPublisher infoThermalFps_publisher;

    Voxel3dStrPublisher sensorName_publisher;
    Voxel3dStrPublisher sensorSn_publisher;
    Voxel3dStrPublisher sensorFwVersion_publisher;
    Voxel3dStrPublisher sensorFwBuildDate_publisher;
    Voxel3dStrPublisher sensorLibVersion_publisher;


    double depthMaxRange = 10000.0;

    // Device / sensor scan state
    CamDevInfo camInfo;
    char dev_sn[MAX_DEV_NAME_LEN] = {};
    hiRabSensorInfo _sensorInfo;

    // Rectification / distortion 
    RectifyType m_doRectify = RectifyType::NONE;
    bool m_undistort = false;

    // Working buffers for the non-rectified RGB / FLIR queries in captureLoop().
    cv::Mat rgb;
    cv::Mat flir;
    cv::Mat lastValidFlir;
    cv::Mat lastValidRgb;
    unsigned int retFlir = 0;

    float flirDisMax = 50.0f;
    float flirDisMin = 20.0f;

    // Frame data
    float* pointcloudData = nullptr;
    float* pointcloudDataResized = nullptr;

    // Imu data
    double ax, wx;
    double ay, wy;
    double az, wz;

    int pixelCount = 0;
    
    // Config value;
    float sensorHeight = 1050.0f; // sensor mounting height above ground, mm -- used for the map->voxel3d_frame TF
    std::string sensorName = "N/A";
    std::string sensorSn = "N/A";
    std::string sensorFwVersion = "N/A";
    std::string sensorFwBuildDate = "N/A";
    std::string sensorLibVersion = "N/A";

    std::thread m_processThread;
    std::atomic<bool> m_isRunning{ false };
    std::atomic<bool> m_isConnected{ false };


    struct CapturedFrame {
        bool valid = false;

        bool hasImu = false;
        float imuAcceleration[3] = {0.0f, 0.0f, 0.0f};
        float imuAngularVelocity[3] = {0.0f, 0.0f, 0.0f};
        uint64_t imuAngularTimestamp = 0;

        bool hasPc = false;
        int pcWidth = 0;
        int pcHeight = 0;
        std::vector<float> pointcloudRaw; // xyz triples, copied straight out of pc->m_data

        cv::Mat depthRGB;       // already colour-mapped + cloned, safe to keep
        cv::Mat irBGR;          // already converted + cloned
        cv::Mat rgbResized;     // already resized/converted + cloned
        cv::Mat thermalResized; // already resized/converted + cloned

        std::chrono::steady_clock::time_point t_captureStart;
        std::chrono::steady_clock::time_point t_captureEnd; // right after waitForFrames() returns, before extraction work
    };

    std::thread m_captureThread;
    std::mutex m_captureMutex;
    std::condition_variable m_captureCv;
    std::shared_ptr<CapturedFrame> m_latestCapturedFrame;
    bool m_captureFrameReady = false;

    // Latest processed frame data, shared between processLoop (writer) and publish_frames (reader)
    struct LatestPointCloud {
        bool valid = false;
        int width = 0;
        int height = 0;
        std::vector<float> data; // 4 floats per point: x, y, z, conf
    };
    struct LatestImu {
        bool valid = false;
        float acceleration[3] = {0.0f, 0.0f, 0.0f};
        float angularVelocity[3] = {0.0f, 0.0f, 0.0f};
        uint64_t angularTimestamp = 0;
    };

    std::mutex m_latestFrameMutex;
    std::atomic<bool> m_latestFrameReady{ false };
    LatestPointCloud m_latestPc;
    LatestImu m_latestImu;
    cv::Mat m_latestDepthRGB;
    cv::Mat m_latestIrBGR;
    cv::Mat m_latestRgbResized;
    cv::Mat m_latestThermalResized;

#ifdef ROS1
    tf::TransformBroadcaster br;
#else
    rclcpp::Node::SharedPtr node;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
#endif

    void publishColoredPointCloud(const LatestPointCloud& pc, const cv::Mat& colorBGR, PointCloudPublisher& publisher)
    {
        int numPoints = pc.width * pc.height;

        PointCloud2Msg msg_pc;
        msg_pc.header.stamp = VOXEL3D_NOW();
        msg_pc.header.frame_id = "voxel3d_frame";
        msg_pc.width = pc.width;
        msg_pc.height = pc.height;
        msg_pc.is_bigendian = false;
        msg_pc.is_dense = false;

        sensor_msgs::PointCloud2Modifier modifier(msg_pc);
        modifier.setPointCloud2FieldsByString(2, "xyz", "rgba");
        modifier.resize(numPoints);

        sensor_msgs::PointCloud2Iterator<float>    iter_x(msg_pc, "x");
        sensor_msgs::PointCloud2Iterator<float>    iter_y(msg_pc, "y");
        sensor_msgs::PointCloud2Iterator<float>    iter_z(msg_pc, "z");
        sensor_msgs::PointCloud2Iterator<uint32_t> iter_rgba(msg_pc, "rgba");

        cv::Mat colorResized;
        const cv::Mat* colorPtr = &colorBGR;
        if (!colorBGR.empty() && colorBGR.type() == CV_8UC3 && pc.width > 0 && pc.height > 0
            && (colorBGR.cols != pc.width || colorBGR.rows != pc.height)) {
            cv::resize(colorBGR, colorResized, cv::Size(pc.width, pc.height), 0, 0, cv::INTER_LINEAR);
            colorPtr = &colorResized;
        }

        bool has_image = !colorPtr->empty()
                       && colorPtr->type() == CV_8UC3
                       && colorPtr->isContinuous()
                       && colorPtr->cols == pc.width
                       && colorPtr->rows == pc.height;
        const uint8_t* img_ptr = has_image ? colorPtr->data : nullptr;
        for (size_t i = 0; i < (size_t)numPoints; ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_rgba) {
            int pcl_idx = i * 3;
            *iter_x = pc.data.data()[pcl_idx];
            *iter_y = pc.data.data()[pcl_idx + 1];
            *iter_z = pc.data.data()[pcl_idx + 2];

            uint8_t r = 255, g = 120, b = 0, a = 255;
            if (has_image) {
                b = img_ptr[i * 3 + 0];
                g = img_ptr[i * 3 + 1];
                r = img_ptr[i * 3 + 2];
            } else {
                r = g = b = 255;
            }
            *iter_rgba = ((uint32_t)a << 24) | ((uint32_t)r << 16) | ((uint32_t)g << 8) | (uint32_t)b;
        }

        VOXEL3D_PUBLISH(publisher, msg_pc);
    }

#undef VOXEL3D_OK
#undef VOXEL3D_SPIN_SOME
#undef VOXEL3D_NOW
#undef VOXEL3D_TIME_FROM_NSEC
#undef VOXEL3D_PUBLISH
#undef VOXEL3D_PUBLISH_IMG
#undef VOXEL3D_NUM_SUBSCRIBERS
#undef VOXEL3D_SEND_TF

    void fillSensorParams(std::vector<float>& params, CameraInfo& camInfo)
    {
        // Lens parameters
        // - Principal point cx, cy
        params[0] = camInfo.principalPointCx;
        params[1] = camInfo.principalPointCy;
        // - Focal length fx, fy
        params[2] = camInfo.focalLengthFx;
        params[3] = camInfo.focalLengthFy;
        // - Principal point p1, p2
        params[4] = camInfo.P1;
        params[5] = camInfo.P2;
        // - Distortion radial
        params[6] = camInfo.K1;
        params[7] = camInfo.K2;
        params[8] = camInfo.K3;
        params[9] = camInfo.K4;
        params[10] = camInfo.K5;
        params[11] = camInfo.K6;
    }

    /// <summary>
    /// Create a LUT for visualizing thermal data with an IRON color scheme.
    /// </summary>
    /// <returns>LUT</returns>
    cv::Mat createIronLUT()
    {
        cv::Mat lut(256, 1, CV_8UC3);

        for (int i = 0; i < 256; i++)
        {
            float t = i / 255.0f;
            cv::Vec3b color;
            if (t < 0.25f)
            {
                // Black to Purple
                float k = t / 0.25f;

                color[0] = uchar(128 * k);
                color[1] = 0;
                color[2] = uchar(64 * k);
            }
            else if (t < 0.5f)
            {
                // Purple to Red
                float k = (t - 0.25f) / 0.25f;

                color[0] = uchar(128 * (1.0f - k));
                color[1] = 0;
                color[2] = uchar(64 + 191 * k);
            }
            else if (t < 0.75f)
            {
                // Red to Yellow
                float k = (t - 0.5f) / 0.25f;

                color[0] = 0;
                color[1] = uchar(255 * k);
                color[2] = 255;
            }
            else
            {
                // Yellow to White
                float k = (t - 0.75f) / 0.25f;

                color[0] = uchar(255 * k);
                color[1] = 255;
                color[2] = 255;
            }

            lut.at<cv::Vec3b>(i, 0) = color;
        }
        return lut;
    }

};

#ifdef ROS1
#define VOXEL3D_ROS_INIT(argc, argv) ros::init(argc, argv, "camera_publisher")
#define VOXEL3D_LOG_ERROR(...)       ROS_ERROR(__VA_ARGS__)
#define VOXEL3D_ROS_SHUTDOWN()       ((void)0)
#else
#define VOXEL3D_ROS_INIT(argc, argv) rclcpp::init(argc, argv)
#define VOXEL3D_LOG_ERROR(...)       RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), __VA_ARGS__)
#define VOXEL3D_ROS_SHUTDOWN()       rclcpp::shutdown()
#endif

int main(int argc, char **argv) {
    VOXEL3D_ROS_INIT(argc, argv);
    try {
        CameraNode cam_node;
        cam_node.publish_loop();
    } catch (const std::exception &e) {
        VOXEL3D_LOG_ERROR("Error: %s", e.what());
        return 1;
    }

    VOXEL3D_ROS_SHUTDOWN();
    return 0;
}

#undef VOXEL3D_ROS_INIT
#undef VOXEL3D_LOG_ERROR
#undef VOXEL3D_ROS_SHUTDOWN