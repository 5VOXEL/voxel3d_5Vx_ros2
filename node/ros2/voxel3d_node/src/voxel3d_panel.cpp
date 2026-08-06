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

#ifdef ROS1
#include <pluginlib/class_list_macros.h>
#include <std_msgs/String.h>
#include <boost/bind.hpp>
#else
#include <pluginlib/class_list_macros.hpp>
#endif

#include <cv_bridge/cv_bridge.h>
#include <voxel3d_node/voxel3d_panel.hpp>

namespace voxel3d_node {

// ── Expander button factory ───────────────────────────────────────────────
static QPushButton* makeExpanderButton(const QString &title, QWidget *parent)
{
    auto *btn = new QPushButton("▼  " + title, parent);
    btn->setCheckable(true);
    btn->setChecked(true);
    btn->setFlat(true);
    btn->setStyleSheet(
        "QPushButton {"
        "  text-align: left; font-weight: bold;"
        "  padding: 4px 6px; border: none;"
        "  border-bottom: 1px solid palette(mid);"
        "  background: palette(window); }"
        "QPushButton:hover { background: palette(light); }"
    );
    return btn;
}

// ── Helper: connect one expander button to its container ─────────────────
static void connectExpander(QPushButton *btn, QWidget *container, const QString &title)
{
    QObject::connect(btn, &QPushButton::toggled, [btn, container, title](bool checked) {
        container->setVisible(checked);
        btn->setText(QString(checked ? "▼" : "▶") + "  " + title);
    });
}

// ── Helper: add one image expander section to gridMain ────────────────
static void addImageSection(QVBoxLayout *gridMain,
                            QPushButton *&btnOut, QWidget *&containerOut,
                            QLabel *&labelOut,
                            const QString &title)
{
    btnOut       = makeExpanderButton(title, nullptr);
    containerOut = new QWidget();
    auto *layout = new QHBoxLayout(containerOut);
    layout->setContentsMargins(0, 2, 0, 4);
    layout->setSpacing(4);
    labelOut = new QLabel();
    layout->addWidget(labelOut);
    gridMain->addWidget(btnOut);
    gridMain->addWidget(containerOut);
    connectExpander(btnOut, containerOut, title);
}

// ── Helper: add one titled group of "header / value" rows to gridVoxel3d ──
// (blue bold title row, then one row per header/value pair below it).
// `row` is the next free grid row and is advanced past everything this adds,
// so callers just chain calls with the same `row` variable.
static void addInfoSection(QGridLayout *grid, int &row,
                            const QString &title,
                            const QStringList &headers,
                            const std::initializer_list<QLabel **> &values)
{
    auto *lblTitle = new QLabel(title);
    lblTitle->setStyleSheet("color: #1d72f1; font-weight: bold; margin-top: 4px;");
    grid->addWidget(lblTitle, row++, 0);

    int i = 0;
    for (QLabel **value : values) {
        auto *lblHeader = new QLabel(headers[i]);
        lblHeader->setStyleSheet("margin-left: 2px;");
        *value = new QLabel("0.0");
        (*value)->setStyleSheet("color: #666666; font-weight: bold;");

        grid->addWidget(lblHeader, row, 0);
        grid->addWidget(*value, row, 1);
        ++row;
        ++i;
    }
}

// ── Constructor ───────────────────────────────────────────────────────────
Voxel3dRvizPanel::Voxel3dRvizPanel(QWidget *parent) : Panel(parent)
{
    auto *gridMain = new QVBoxLayout();
    gridMain->setContentsMargins(4, 4, 4, 4);
    gridMain->setSpacing(2);

    // 5Voxel sensor info panel
    voxel3dExpanderBtn = makeExpanderButton("Voxel3d", nullptr);
    voxel3dContainer   = new QWidget();
    auto *gridVoxel3d = new QGridLayout(voxel3dContainer);
    gridVoxel3d->setColumnStretch(0, 1);
    gridVoxel3d->setColumnStretch(1, 0);
    gridVoxel3d->setContentsMargins(4, 4, 4, 4);
    gridVoxel3d->setSpacing(2);
    gridVoxel3d->addWidget(voxel3dExpanderBtn);

    int gridCont = 0;

    addInfoSection(gridVoxel3d, gridCont, "Sensor Info.",
        {"Name", "SN", "FW Version", "FW Build Date", "Library Version"},
        {&lblSensorName, &lblSensorSn, &lblSensorFwVersion, &lblSensorFwBuildDate, &lblSensorLibVersion});

    addInfoSection(gridVoxel3d, gridCont, "RGB",
        {"Resolution", "FOV", "FPS"},
        {&lblInfoRgbRes, &lblInfoRgbFov, &lblInfoRgbFps});

    addInfoSection(gridVoxel3d, gridCont, "Depth",
        {"Resolution", "FOV", "FPS"},
        {&lblInfoDepthRes, &lblInfoDepthFov, &lblInfoDepthFps});

    addInfoSection(gridVoxel3d, gridCont, "Thermal",
        {"Resolution", "FOV", "FPS"},
        {&lblInfoThermalRes, &lblInfoThermalFov, &lblInfoThermalFps});

    // IMU section
    imuExpanderBtn = makeExpanderButton("IMU", nullptr);
    imuContainer   = new QWidget();
    layoutImu = new QGridLayout(imuContainer);
    layoutImu->setColumnStretch(0, 1);
    layoutImu->setColumnStretch(1, 1);
    layoutImu->setColumnStretch(2, 1);
    layoutImu->setContentsMargins(4, 4, 4, 4);
    layoutImu->setSpacing(0);

    lblImuAccelHeader = new QLabel("Acceleration (m/s²)");
    layoutImu->addWidget(lblImuAccelHeader, 0, 0, 1, 3);
    lblImuAngVelHeader = new QLabel("Angular Velocity (rad/s)");
    layoutImu->addWidget(lblImuAngVelHeader, 2, 0, 1, 3);

    QStringList axes = {"X:", "Y:", "Z:"};
    QLabel** labelValues[] = { &lblImuAccelXValue, &lblImuAccelYValue, &lblImuAccelZValue,
                              &lblImuAngVelXValue, &lblImuAngVelYValue, &lblImuAngVelZValue };

    // 6 entries in labelValues[] (accel X/Y/Z + angvel X/Y/Z) -- this was
    // `i < 5`, so lblImuAngVelZValue (index 5) never got `new QLabel(...)`
    // and stayed a wild pointer, dereferenced by updateLabels() every 33ms
    // via lblImuAngVelZValue->setText(...). Same class of bug as the
    // sensor-info loop above.
    for (int i = 0; i < 6; ++i) {
        auto *wdgElement = new QWidget();
        auto *layoutWdg = new QHBoxLayout(wdgElement);

        auto *lblElementHeader = new QLabel(axes[i%3]);
        *labelValues[i] = new QLabel("0.0");
        (*labelValues[i])->setStyleSheet("color: #666666; font-weight: bold;");

        layoutWdg->addWidget(lblElementHeader);
        layoutWdg->addWidget(*labelValues[i]);

        layoutWdg->setStretch(0, 1);
        layoutWdg->setStretch(1, 0);

        layoutImu->addWidget(wdgElement, 2 * (i/3) + 1, i%3);
    }

    lblImuTimestampHeader  = new QLabel("Timestamp");
    layoutImu->addWidget(lblImuTimestampHeader, 4, 0, 1, 2);
    lblImuTimestampValue = new QLabel("0");
    layoutImu->addWidget(lblImuTimestampValue, 4, 2, 1, 1);

    layoutImu->setRowMinimumHeight(2, 10);



    gridMain->addWidget(voxel3dExpanderBtn);
    gridMain->addWidget(voxel3dContainer);

    gridMain->addWidget(imuExpanderBtn);
    gridMain->addWidget(imuContainer);

    // Image sections (RGB / IR / Depth / Thermal)
    addImageSection(gridMain, rgbExpanderBtn,   rgbContainer,   lblRgb,   "RGB");
    addImageSection(gridMain, irExpanderBtn,    irContainer,    lblIr,    "IR");
    addImageSection(gridMain, depthExpanderBtn, depthContainer, lblDepth, "Depth");
    addImageSection(gridMain, thermalExpanderBtn, thermalContainer, lblThermal, "Thermal");

    gridMain->addStretch();

    connectExpander(imuExpanderBtn, imuContainer, "IMU");
    connectExpander(voxel3dExpanderBtn, voxel3dContainer, "Voxel3d");

    // ── ScrollArea ────────────────────────────────────────
    auto *scrollContent = new QWidget();
    scrollContent->setLayout(gridMain);

    auto *scrollArea = new QScrollArea(this);
    scrollArea->setWidget(scrollContent);
    scrollArea->setWidgetResizable(true);
    scrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    scrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    scrollArea->setFrameShape(QFrame::NoFrame);

    auto *gridOuter = new QVBoxLayout();
    gridOuter->setContentsMargins(0, 0, 0, 0);
    gridOuter->setSpacing(0);
    gridOuter->addWidget(scrollArea);
    setLayout(gridOuter);

    // Init data
    accelVec[0] = accelVec[1] = accelVec[2] = 0.0f;
    angVelVec[0] = angVelVec[1] = angVelVec[2] = 0.0f;
    imuTimestamp = 0;

    // Sensor Information
    mRgb.res = mRgb.fov = mRgb.fps = "N/A";
    mDepth.res = mDepth.fov = mDepth.fps = "N/A";
    mThermal.res = mThermal.fov = mThermal.fps = "N/A";

    // Sensor Params
    sensorSn = "N/A";
    sensorFwVersion = "N/A";
    sensorFwBuildDate = "N/A";
    sensorName = "N/A";
    sensorLibVersion = "N/A";

    timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &Voxel3dRvizPanel::updateLabels);
    timer->start(33);
}

Voxel3dRvizPanel::~Voxel3dRvizPanel() = default;

// ── onInitialize ─────────────────────────────────────────────────────────
// Subscription creation still differs in *shape* between ROS1 (member
// function pointer + separate NodeHandle) and ROS2 (std::bind + node->
// create_subscription<T>()), so it isn't a full one-liner unification like
// the callback signatures below -- but this macro collapses each individual
// subscription from a duplicated ROS1/ROS2 pair down to one line.
//
// `MsgType` is always one of the ImageMsg/ImuMsg/StringMsg/Float64Msg
// aliases from voxel3d_panel.hpp, not a raw sensor_msgs/std_msgs type: ROS2's
// create_subscription<T>() needs the type spelled out explicitly (can't be
// deduced from std::bind), and ROS1's nh.subscribe<M>() also needs it
// explicit -- with as many overloads as subscribe() has (raw function
// pointer, member-function pointer, boost::function, ...), deduction through
// a member-function-pointer callback isn't reliable, so both branches take
// it. Passing a raw ROS2-style type (e.g. sensor_msgs::msg::Image) here
// wouldn't even compile under ROS1, which is exactly why these aliases
// exist.
#ifdef ROS1
#define VOXEL3D_SUBSCRIBE(member, MsgType, topic, cb) \
    member = nh.subscribe<MsgType>(topic, 10, &Voxel3dRvizPanel::cb, this)
#else
#define VOXEL3D_SUBSCRIBE(member, MsgType, topic, cb) \
    member = node->create_subscription<MsgType>(topic, 10, std::bind(&Voxel3dRvizPanel::cb, this, _1))
#endif

void Voxel3dRvizPanel::onInitialize(){
#ifdef ROS1
    ros::NodeHandle nh;
#else
    using std::placeholders::_1;
    node = std::make_shared<rclcpp::Node>("voxel3d_rviz_panel");
#endif

    // Frame
    VOXEL3D_SUBSCRIBE(SubRgbImage,   ImageMsg, "/camera/frame_rgb",     callbackRGB);
    VOXEL3D_SUBSCRIBE(SubDepthImage, ImageMsg, "/camera/frame_depth",   callbackDepth);
    VOXEL3D_SUBSCRIBE(SubIrImage,    ImageMsg, "/camera/frame_ir",      callbackIR);
    VOXEL3D_SUBSCRIBE(SubThermalImage, ImageMsg, "/camera/frame_thermal", callbackThermal);

    // Imu
    VOXEL3D_SUBSCRIBE(SubImuAccelSub, ImuMsg, "/imu/accel", callbackImuAccel);
    VOXEL3D_SUBSCRIBE(SubImuGyroSub,  ImuMsg, "/imu/gyro",  callbackImuGyro);

    // Sensor Information
    VOXEL3D_SUBSCRIBE(SubInfoRgbRes, StringMsg, "voxel3d/info_rgb_res", callbackInfoRgbRes);
    VOXEL3D_SUBSCRIBE(SubInfoRgbFov, StringMsg, "voxel3d/info_rgb_fov", callbackInfoRgbFov);
    VOXEL3D_SUBSCRIBE(SubInfoRgbFps, StringMsg, "voxel3d/info_rgb_fps", callbackInfoRgbFps);

    VOXEL3D_SUBSCRIBE(SubInfoDepthRes, StringMsg, "voxel3d/info_depth_res", callbackInfoDepthRes);
    VOXEL3D_SUBSCRIBE(SubInfoDepthFov, StringMsg, "voxel3d/info_depth_fov", callbackInfoDepthFov);
    VOXEL3D_SUBSCRIBE(SubInfoDepthFps, StringMsg, "voxel3d/info_depth_fps", callbackInfoDepthFps);

    VOXEL3D_SUBSCRIBE(SubInfoThermalRes, StringMsg, "voxel3d/info_thermal_res", callbackInfoThermalRes);
    VOXEL3D_SUBSCRIBE(SubInfoThermalFov, StringMsg, "voxel3d/info_thermal_fov", callbackInfoThermalFov);
    VOXEL3D_SUBSCRIBE(SubInfoThermalFps, StringMsg, "voxel3d/info_thermal_fps", callbackInfoThermalFps);

    // Sensor Params
    VOXEL3D_SUBSCRIBE(SubSensorName,         StringMsg, "voxel3d/name",           callbackSensorName);
    VOXEL3D_SUBSCRIBE(SubSensorSn,           StringMsg, "voxel3d/sn",              callbackSensorSN);
    VOXEL3D_SUBSCRIBE(SubSensorFwVersion,    StringMsg, "voxel3d/fw_version",      callbackSensorFwVersion);
    VOXEL3D_SUBSCRIBE(SubSensorFwBuildDate,  StringMsg, "voxel3d/fw_build_date",   callbackSensorFwBuildDate);
    VOXEL3D_SUBSCRIBE(SubSensorLibVersion,   StringMsg, "voxel3d/lib_version",     callbackSensorLibVersion);
}

#undef VOXEL3D_SUBSCRIBE

// ── Image callback helper ─────────────────────────────────────────────────
static void decodeImage(const ImageMsgPtr &msg, QImage &out)
{
    if (!msg) return;
    auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    // voxel3d_node now publishes a 0x0 image on camera/frame_rgb or
    // camera/frame_thermal whenever that sensor doesn't exist on this
    // device (RGB/FLIR are optional per-variant). cv::cvtColor asserts
    // !src.empty(), so an empty frame here would crash the panel exactly
    // like the FLIR capture-side crash fixed earlier -- bail out instead
    // and leave `out` null; showImage() below already handles a null
    // QImage by logging "<name> is null!" instead of drawing anything.
    if (cv_ptr->image.empty()) {
        out = QImage();
        return;
    }
    cv::Mat rgb;
    cv::cvtColor(cv_ptr->image, rgb, cv::COLOR_BGR2RGB);
    out = QImage(rgb.data, rgb.cols, rgb.rows, rgb.step, QImage::Format_RGB888).copy();
}

void Voxel3dRvizPanel::callbackRGB    (ImageMsgPtr msg) { decodeImage(msg, currentRgb);     }
void Voxel3dRvizPanel::callbackDepth  (ImageMsgPtr msg) { decodeImage(msg, currentDepth);   }
void Voxel3dRvizPanel::callbackIR     (ImageMsgPtr msg) { decodeImage(msg, currentIr);      }
void Voxel3dRvizPanel::callbackThermal(ImageMsgPtr msg) { decodeImage(msg, currentThermal); }

// ── IMU callbacks ─────────────────────────────────────────────────────────
void Voxel3dRvizPanel::callbackImuAccel(ImuMsgPtr msg)
{
    accelVec[0] = static_cast<float>(msg->linear_acceleration.x);
    accelVec[1] = static_cast<float>(msg->linear_acceleration.y);
    accelVec[2] = static_cast<float>(msg->linear_acceleration.z);
#ifdef ROS1
    imuTimestamp = msg->header.stamp.toNSec();
#else
    imuTimestamp = static_cast<uint64_t>(msg->header.stamp.sec) * 1000000000ULL
                 + static_cast<uint64_t>(msg->header.stamp.nanosec);
#endif
}

void Voxel3dRvizPanel::callbackImuGyro(ImuMsgPtr msg)
{
    angVelVec[0] = static_cast<float>(msg->angular_velocity.x);
    angVelVec[1] = static_cast<float>(msg->angular_velocity.y);
    angVelVec[2] = static_cast<float>(msg->angular_velocity.z);
}


// ── Sensor Information callbacks ─────────────────────────────────────────
void Voxel3dRvizPanel::callbackInfoRgbRes(StringMsgPtr msg){ mRgb.res = msg->data; }
void Voxel3dRvizPanel::callbackInfoRgbFov(StringMsgPtr msg){ mRgb.fov = msg->data; }
void Voxel3dRvizPanel::callbackInfoRgbFps(StringMsgPtr msg){ mRgb.fps = msg->data; }

void Voxel3dRvizPanel::callbackInfoDepthRes(StringMsgPtr msg){ mDepth.res = msg->data; }
void Voxel3dRvizPanel::callbackInfoDepthFov(StringMsgPtr msg){ mDepth.fov = msg->data; }
void Voxel3dRvizPanel::callbackInfoDepthFps(StringMsgPtr msg){ mDepth.fps = msg->data; }

void Voxel3dRvizPanel::callbackInfoThermalRes(StringMsgPtr msg){ mThermal.res = msg->data; }
void Voxel3dRvizPanel::callbackInfoThermalFov(StringMsgPtr msg){ mThermal.fov = msg->data; }
void Voxel3dRvizPanel::callbackInfoThermalFps(StringMsgPtr msg){ mThermal.fps = msg->data; }

// ── Sensor Param callbacks ────────────────────────────────────────────────
void Voxel3dRvizPanel::callbackSensorName(StringMsgPtr msg){ sensorName = msg->data; }
void Voxel3dRvizPanel::callbackSensorSN(StringMsgPtr msg){ sensorSn = msg->data; }
void Voxel3dRvizPanel::callbackSensorFwVersion(StringMsgPtr msg){ sensorFwVersion = msg->data; }
void Voxel3dRvizPanel::callbackSensorFwBuildDate(StringMsgPtr msg){ sensorFwBuildDate = msg->data; }
void Voxel3dRvizPanel::callbackSensorLibVersion(StringMsgPtr msg){ sensorLibVersion = msg->data; }

// ── updateLabels ──────────────────────────────────────────────────────────
void Voxel3dRvizPanel::updateLabels()
{
#ifndef ROS1
    rclcpp::spin_some(node);
#endif

    // Helper lambda: set pixmap if we have one. A null image just means that
    // stream's sensor doesn't exist on this device (RGB/FLIR are optional)
    // or hasn't published its first frame yet -- normal/expected, not worth
    // logging every ~33ms update tick.
    auto showImage = [&](QLabel *label, const QImage &img, int w, int h, const char *name) {
        (void)name;
        if (!img.isNull()) {
            label->setPixmap(QPixmap::fromImage(img).scaled(w, h));
        }
    };

    showImage(lblRgb,   currentRgb,   320, 240, "RGB");
    showImage(lblDepth, currentDepth, 320, 240, "Depth");
    showImage(lblIr,    currentIr,    320, 240, "IR");
    showImage(lblThermal, currentThermal, 320, 240, "Thermal");

    lblImuAccelXValue->setText(QString("%1").arg(accelVec[0], 10, 'f', 4));
    lblImuAccelYValue->setText(QString("%1").arg(accelVec[1], 10, 'f', 4));
    lblImuAccelZValue->setText(QString("%1").arg(accelVec[2], 10, 'f', 4));
    lblImuAngVelXValue->setText(QString("%1").arg(angVelVec[0], 10, 'f', 4));
    lblImuAngVelYValue->setText(QString("%1").arg(angVelVec[1], 10, 'f', 4));
    lblImuAngVelZValue->setText(QString("%1").arg(angVelVec[2], 10, 'f', 4));

    lblImuTimestampValue->setText(QString("%1").arg(imuTimestamp));

    // Sensor Information
    lblInfoRgbRes->setText(QString("%1").arg(QString::fromStdString(mRgb.res)));
    lblInfoRgbFov->setText(QString("%1").arg(QString::fromStdString(mRgb.fov)));
    lblInfoRgbFps->setText(QString("%1").arg(QString::fromStdString(mRgb.fps)));

    lblInfoDepthRes->setText(QString("%1").arg(QString::fromStdString(mDepth.res)));
    lblInfoDepthFov->setText(QString("%1").arg(QString::fromStdString(mDepth.fov)));
    lblInfoDepthFps->setText(QString("%1").arg(QString::fromStdString(mDepth.fps)));

    lblInfoThermalRes->setText(QString("%1").arg(QString::fromStdString(mThermal.res)));
    lblInfoThermalFov->setText(QString("%1").arg(QString::fromStdString(mThermal.fov)));
    lblInfoThermalFps->setText(QString("%1").arg(QString::fromStdString(mThermal.fps)));

    // Sensor params
    lblSensorName->setText(QString("%1").arg(QString::fromStdString(sensorName)));
    lblSensorSn->setText(QString("%1").arg(QString::fromStdString(sensorSn)));
    lblSensorFwVersion->setText(QString("%1").arg(QString::fromStdString(sensorFwVersion)));
    lblSensorFwBuildDate->setText(QString("%1").arg(QString::fromStdString(sensorFwBuildDate)));
    lblSensorLibVersion->setText(QString("%1").arg(QString::fromStdString(sensorLibVersion)));
}

} // namespace voxel3d_node

#ifdef ROS1
PLUGINLIB_EXPORT_CLASS(voxel3d_node::Voxel3dRvizPanel, rviz::Panel)
#else
PLUGINLIB_EXPORT_CLASS(voxel3d_node::Voxel3dRvizPanel, rviz_common::Panel)
#endif
