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

#ifndef VOXEL3D_PANEL_HPP
#define VOXEL3D_PANEL_HPP

#include <QObject>
#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTimer>
#include <QPushButton>
#include <QWidget>
#include <QScrollArea>
#include <QFrame>
#include <QGridLayout>
#include <QStringList>

#ifdef ROS1
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/UInt64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <rviz/panel.h>
#else
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <rviz_common/panel.hpp>
#endif

namespace voxel3d_node {

// ── ROS1/ROS2 type aliases ─────────────────────────────────────────────────
// ROS1 passes callbacks a boost::shared_ptr "ConstPtr" and stores every
// subscription in the same untyped ros::Subscriber. ROS2 passes a
// std::shared_ptr "SharedPtr" and stores each subscription in a
// Subscription<T>::SharedPtr templated on the message type. Aliasing both
// per message type here means every callback signature and subscriber member
// below is written once instead of being duplicated in a ROS1/ROS2 #ifdef.
#ifdef ROS1
typedef sensor_msgs::Image ImageMsg;
typedef sensor_msgs::Imu   ImuMsg;
typedef std_msgs::String   StringMsg;
typedef std_msgs::Float64  Float64Msg;

typedef sensor_msgs::Image::ConstPtr ImageMsgPtr;
typedef sensor_msgs::Imu::ConstPtr   ImuMsgPtr;
typedef std_msgs::String::ConstPtr   StringMsgPtr;
typedef std_msgs::Float64::ConstPtr  Float64MsgPtr;

typedef ros::Subscriber ImageSub;
typedef ros::Subscriber ImuSub;
typedef ros::Subscriber StringSub;
typedef ros::Subscriber Float64Sub;
#else
typedef sensor_msgs::msg::Image ImageMsg;
typedef sensor_msgs::msg::Imu   ImuMsg;
typedef std_msgs::msg::String   StringMsg;
typedef std_msgs::msg::Float64  Float64Msg;

typedef sensor_msgs::msg::Image::SharedPtr ImageMsgPtr;
typedef sensor_msgs::msg::Imu::SharedPtr   ImuMsgPtr;
typedef std_msgs::msg::String::SharedPtr   StringMsgPtr;
typedef std_msgs::msg::Float64::SharedPtr  Float64MsgPtr;

typedef rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr   ImageSub;
typedef rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr     ImuSub;
typedef rclcpp::Subscription<std_msgs::msg::String>::SharedPtr     StringSub;
typedef rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr    Float64Sub;
#endif

#ifdef ROS1
class Voxel3dRvizPanel : public rviz::Panel
#else
class Voxel3dRvizPanel : public rviz_common::Panel
#endif
{
    Q_OBJECT
  public:
    explicit Voxel3dRvizPanel(QWidget *parent = 0);
    ~Voxel3dRvizPanel() override;
    void onInitialize() override;

  private:
    // Frame
    // NOTE: these are passed BY VALUE (ImageMsgPtr, not const ImageMsgPtr&).
    // rclcpp's create_subscription() only accepts callbacks shaped like
    // void(SharedPtr) for a plain (non-const) message type -- there's no
    // void(const SharedPtr&) overload in its AnySubscriptionCallback variant,
    // so binding to `const ImageMsgPtr&` fails to compile with a wall of
    // "no known conversion for argument 1" / variant errors. Taking the
    // shared_ptr by value is cheap (one atomic refcount bump) and is what
    // ROS2 examples use; it also still works fine for ROS1's ConstPtr.
    void callbackRGB(ImageMsgPtr msg);
    void callbackDepth(ImageMsgPtr msg);
    void callbackIR(ImageMsgPtr msg);
    void callbackThermal(ImageMsgPtr msg);

    // Imu
    void callbackImuAccel(ImuMsgPtr msg);
    void callbackImuGyro(ImuMsgPtr msg);

    // Sensor Information
    // Fps fields are published as pre-formatted (1-decimal) strings now, not
    // Float64 -- see formatFixed1() in voxel3d_node.cpp -- so these take
    // StringMsgPtr like the res/fov callbacks above them.
    void callbackInfoRgbRes(StringMsgPtr msg);
    void callbackInfoRgbFov(StringMsgPtr msg);
    void callbackInfoRgbFps(StringMsgPtr msg);

    void callbackInfoDepthRes(StringMsgPtr msg);
    void callbackInfoDepthFov(StringMsgPtr msg);
    void callbackInfoDepthFps(StringMsgPtr msg);

    void callbackInfoThermalRes(StringMsgPtr msg);
    void callbackInfoThermalFov(StringMsgPtr msg);
    void callbackInfoThermalFps(StringMsgPtr msg);

    // Sensor Params
    void callbackSensorName(StringMsgPtr msg);
    void callbackSensorSN(StringMsgPtr msg);
    void callbackSensorFwVersion(StringMsgPtr msg);
    void callbackSensorFwBuildDate(StringMsgPtr msg);
    void callbackSensorLibVersion(StringMsgPtr msg);

    void updateLabels();

#ifndef ROS1
    rclcpp::Node::SharedPtr node;
#endif

    // Frame
    ImageSub SubRgbImage;
    ImageSub SubDepthImage;
    ImageSub SubIrImage;
    ImageSub SubThermalImage;

    // Imu
    ImuSub SubImuAccelSub;
    ImuSub SubImuGyroSub;

    // Sensor Information
    StringSub SubInfoRgbRes;
    StringSub SubInfoRgbFov;
    StringSub SubInfoRgbFps;

    StringSub SubInfoDepthRes;
    StringSub SubInfoDepthFov;
    StringSub SubInfoDepthFps;

    StringSub SubInfoThermalRes;
    StringSub SubInfoThermalFov;
    StringSub SubInfoThermalFps;

    // Sensor Params
    StringSub SubSensorName;
    StringSub SubSensorSn;
    StringSub SubSensorFwVersion;
    StringSub SubSensorFwBuildDate;
    StringSub SubSensorLibVersion;

    QLabel *lblRgb;
    QLabel *lblDepth;
    QLabel *lblIr;
    QLabel *lblThermal;

    QImage currentRgb;
    QImage currentIr;
    QImage currentDepth;
    QImage currentThermal;

    QTimer *timer;

    // Imu Pannel
    QGridLayout *layoutImu;

    QLabel *lblImuAccelHeader;
    QLabel *lblImuAccelXValue;
    QLabel *lblImuAccelYValue;
    QLabel *lblImuAccelZValue;

    QLabel *lblImuAngVelHeader;
    QLabel *lblImuAngVelXValue;
    QLabel *lblImuAngVelYValue;
    QLabel *lblImuAngVelZValue;

    QLabel *lblImuGyroHeader;
    QLabel *lblImuGyroXValue;
    QLabel *lblImuGyroYValue;
    QLabel *lblImuGyroZValue;

    QLabel *lblImuQuaternionHeader;
    QLabel *lblImuQuaternionXValue;
    QLabel *lblImuQuaternionYValue;
    QLabel *lblImuQuaternionZValue;

    QLabel *lblImuTimestampHeader;
    QLabel *lblImuTimestampValue;

    float accelVec[3];
    float angVelVec[3];
    uint64_t imuTimestamp;

    // Sensor Information
    struct SensorInfo
    {
        std::string res = "N/A";
        std::string fov = "N/A";
        std::string fps = "N/A"; // pre-formatted (1 decimal) by voxel3d_node
    };

    SensorInfo mRgb;
    SensorInfo mDepth;
    SensorInfo mThermal;

    QLabel *lblInfoRgbRes;
    QLabel *lblInfoRgbFov;
    QLabel *lblInfoRgbFps;

    QLabel *lblInfoDepthRes;
    QLabel *lblInfoDepthFov;
    QLabel *lblInfoDepthFps;

    QLabel *lblInfoThermalRes;
    QLabel *lblInfoThermalFov;
    QLabel *lblInfoThermalFps;


    // Voxel3d Pannel
    QGridLayout *layoutSensorInfo;

    // Sensor Params
    std::string sensorName;
    std::string sensorSn;
    std::string sensorFwVersion;
    std::string sensorFwBuildDate;
    std::string sensorLibVersion;

    QLabel *lblSensorName;
    QLabel *lblSensorSn;
    QLabel *lblSensorFwVersion;
    QLabel *lblSensorFwBuildDate;
    QLabel *lblSensorLibVersion;

    // Expander helpers
    QPushButton *rgbExpanderBtn;
    QPushButton *irExpanderBtn;
    QPushButton *depthExpanderBtn;
    QPushButton *thermalExpanderBtn;
    QPushButton *imuExpanderBtn;
    QPushButton *voxel3dExpanderBtn;

    QWidget     *rgbContainer;
    QWidget     *irContainer;
    QWidget     *depthContainer;
    QWidget     *thermalContainer;
    QWidget     *imuContainer;
    QWidget     *voxel3dContainer;

    QWidget     *imuAccelContainer;
    QWidget     *imuAngVelContainer;
};
} // namespace voxel3d_node

#endif // VOXEL3D_PANEL_HPP
