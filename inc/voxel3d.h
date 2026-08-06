/**
 @file      voxel3d.h
 @brief     libvoxel3d APIs for 5Voxel 5VSTDON & 5VHiRab device
 @author    Jackie Lee
 @copyright Copyright (c) 2025 5Voxel Co., Ltd.
*/

#ifndef __VOXEL3d_H__
#define __VOXEL3d_H__

#ifdef PLAT_WINDOWS
#pragma once

#ifdef LIBVOXEL3D_EXPORTS
#define VOXEL3D_API_DLL __declspec(dllexport)
#elif LIBVOXEL3D_STATIC
#define VOXEL3D_API_DLL
#else
#define VOXEL3D_API_DLL __declspec(dllimport)
#endif

#else /* PLAT_LINUX */
#define VOXEL3D_API_DLL
#endif /* PLAT_WINDOWS */

#define MAX_SUPPORTED_CAMERA_MODULE   (12)
#define MAX_SUPPORTED_VIDEO_FRAME_FMT (12)
#define MAX_SUPPORTED_CAPABILITIES    (10)
#define MAX_PRODUCT_NAME_LEN          (128)
#define MAX_PRODUCT_SN_LEN            (128)
#define MAX_DEV_NAME_LEN              (128)

#define MAX_FW_VER_LEN                (16)
#define MAX_FW_BUILD_DATE_LEN         (16)
#define MAX_FLIR_PN_LEN               (32)

enum ImuOridnateType
{
    IMU_COORDINATE_UNKNOWN = 0,
    IMU_COORDINATE_RIGHT_HAND,
    IMU_COORDINATE_LEFT_HAND,
};

enum ImuAxisWithGravity
{
    IMU_GRAVITY_ON_X = 0,
    IMU_GRAVITY_ON_NEGATIVE_X,
    IMU_GRAVITY_ON_Y,
    IMU_GRAVITY_ON_NEGATIVE_Y,
    IMU_GRAVITY_ON_Z,
    IMU_GRAVITY_ON_NEGATIVE_Z,
};

enum VideoMode
{
    VIDEO_MODE_MJPG = 0,
    VIDEO_MODE_YUY2,
};

/**
 * @brief  List of ToF Capability code. 
 *         Different model of 5Voxel camera may support only partial or all.
 *         Use ToF capability APIs to get support list.
 */
enum ToFCapabilityCode
{
    TOF_CAPS_RESERVED = 0,
    TOF_CAPS_AUTO_EXPOSURE,   /* var1: On/Off; var2: Not used */
    TOF_CAPS_EXPOSURE_TIME,   /* var1: Freq1; var2: Freq2 */
    TOF_CAPS_CONF_THRESHOLD,  /* var1: global confidence threshold; var2: Not used */
    TOF_CAPS_FPR,             /* var1: On/Off; var2: Kernel Raidus */
    TOF_CAPS_BILATERAL,       /* var1: On/Off; var2: Kernel Width */
    TOF_CAPS_KALMAN,          /* var1: On/Off; var2: Weighting */
    TOF_CAPS_MEDIAN,          /* var1: On/Off; var2: Kernel Size */
    TOF_CAPS_ADAPTIVE_CONF,   /* var1: On/Off; var2: Not used */
    TOF_CAPS_FRAME_RATE,      /* var1: FPS; var2: Not used */
};

enum use_case {
    SINGLE_FREQ_200MHZ = 1,
    SINGLE_FREQ_80MHZ,
    DUAL_FREQ_80MHZ_60MHZ,
    DUAL_FREQ_80MHZ_50MHZ,
    DUAL_FREQ_200MHZ_166MHZ,
    INVALID_USE_CASE,
};

/**
 * @brief  Structure used in voxel3d_tof_read_camera_info() to read out
 *         camera info from device
 */
struct CameraInfo {
    float focalLengthFx;
    float focalLengthFy;
    float principalPointCx;
    float principalPointCy;
    float K1;
    float K2;
    float P1;
    float P2;
    float K3;
    float K4;
    float K5;
    float K6;
};

struct VideoFrameFormat
{
    unsigned int width;
    unsigned int height;
    unsigned int fmt;
    float fps;
};

struct VideoFormats
{
    int avail_frame_num;
    int default_frame_index;
    VideoFrameFormat fmt[MAX_SUPPORTED_VIDEO_FRAME_FMT];
};

/**
 * @brief  Structure used in voxel3d_scan() to read out supported
 *         ToF capabilities from device
 */
struct CapInfo
{
    unsigned int total_caps_num;
    unsigned char caps_list[MAX_SUPPORTED_CAPABILITIES];
};

/**
 * @brief  Some model(s) of ToF camera support multiple resolution.
 *         Structure used in voxel3d_tof_init() to specify target
 *         ToF resolution to open.
 */
struct CamInitSettings
{
    int width;
    int height;
    int format;
};

struct IMUInfo
{
    bool is_exist;
    unsigned int imu_coordinate;
    unsigned int imu_axis_with_gravity;
};

struct DevInfo
{
    char product_sn[MAX_PRODUCT_SN_LEN] = {};
    char dev_name[MAX_DEV_NAME_LEN] = {};
    VideoFormats frame_fmts = {};
    CapInfo caps = {};
};

/**
 * @brief  Structure used in voxel3d_tof_scan() to fill up with scanned device number
 *         and product serial number for each device
 */
struct CamDevInfo {
    int num_of_devices = 0;
    DevInfo dev_info[MAX_SUPPORTED_CAMERA_MODULE];
    IMUInfo imu_info;
};

/**
 * @brief  Structure used in voxel3d_set_rectifyType() to set alignment between depth and other devices
 */
enum RectifyType
{
    NONE = 0,       /* Disable image rectification */
    RGB2TOF = 1,    /* Software RGB to Depth image rectification */
    FLIR2TOF = 2,   /* Software FLIR to Depth image rectification */
    TOF2RGB = 3     /* Hardware Depth to RGB image rectification */
};

typedef struct
{
    unsigned int imu_ts;
    float imu_accel[3];
    float imu_gyro[3];
} IMU_DATA;

/**
 * @brief       Perform the scan of 5Voxel devices
 * @param[out]  CamDevInfo: structure to store the scanned result
 * @return      > 0: number of device(s) found
 * @return      others: can't find device
 */
extern "C" VOXEL3D_API_DLL int voxel3d_scan(CamDevInfo *cam_dev_info);


/**
 * @brief       Perform the initialization of ToF on sepcific 5Voxel device
 * @warning     This function has to be called before voxel3d_tof_queryframe(),
 *              otherwise the query will fail
 * @param[in]   dev_sn: device S/N, which can be read from the label on 5VHiRab device
 *                      or from result of voxel3d_scan(). Input S/N with NULL pointer
 *                      or empty string will initialize the 1st scanned device
 * @param[in]   tof_setting: target tof setting (width, height, fps, fmt). Library will
 *                           select default if target is not matched.
 * @return      true    found device and init successfully
 * @return      <= 0    can't find device or data error
 */
extern "C" VOXEL3D_API_DLL int voxel3d_tof_init(char *dev_sn, CamInitSettings tof_setting);


/**
 * @brief       Grab a depth & ir frame from 5Voxel 5VHiRab camera
 * @warning     Call voxel3d_tof_init() to initialize specific device before query
 * @param[in]   dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *                      initialize the 1st scanned device
 * @param[out]  depthmap: pointer of user-allocated buffer for undistorted Depth frame storage
 *                        Buffer size shall be TOF_DEPTH_ONLY_FRAME_SIZE (in bytes)
 * @param[out]  irmap: pointer of user-allocated buffer for undistorted IR frame storage
 *                     Buffer size shall be TOF_IR_ONLY_FRAME_SIZE (in bytes)
 * @return      > 0: current frame count (1 ~ UINT_MAX)
 * @return      = 0: no new frame from device
 */
extern "C" VOXEL3D_API_DLL unsigned int voxel3d_tof_queryframe(char* dev_sn,
                                                           unsigned short *depthmap,
                                                           unsigned short *irmap);


/**
 * @brief       Generate pointcloud data based on input deptpmap and the
 *              calibration parameters from 5Voxel 5VHiRab camera
 * @details     The unit of x/y/z is meter
 * @param[in]   dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *                      initialize the 1st scanned device
 * @param[in]   depthmap: pointer of Depth frame filled by voxel3d_tof_queryframe()
 * @param[out]  xyz: pointer of user-allocated buffer for pointcloud frame storage
 * @return      > 0: pixels of pointcloud xyz filled in xyz buffer
 * @return      <= 0: failed to generate pointcloud
 */
extern "C" VOXEL3D_API_DLL int voxel3d_tof_generatePointCloud(char* dev_sn,
                                                              unsigned short *depthmap,
                                                              float *xyz);


/**
 * @brief       Release the resource allocated for ToF on 5Voxel device
 * @warning     This function has to be called before program exit
 * @param[in]   dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *                      initialize the 1st scanned device
 */
extern "C" VOXEL3D_API_DLL void voxel3d_tof_release(char* dev_sn);


/**
 * @brief       Get ToF frame width associated with dev_sn
 * @warning     Call this function after voxel3d_tof_init() is completed and successfully,
 *              otherwise, it returns false
 * @param[in]   dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *                      initialize the 1st scanned device
 * @return      > 0: ToF frame width
 * @return      < 0: error code
 */
extern "C" VOXEL3D_API_DLL int voxel3d_tof_get_width(char* dev_sn);


/**
 * @brief       Get ToF frame height associated with dev_sn
 * @warning     Call this function after voxel3d_tof_init() is completed and successfully,
 *              otherwise, it returns false
 * @param[in]   dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *                      initialize the 1st scanned device
 * @return      > 0: ToF frame height
 * @return      < 0: error code
 */
extern "C" VOXEL3D_API_DLL int voxel3d_tof_get_height(char* dev_sn);


/**
 * @brief       Grab ToF camera info from 5Voxel device
 * @warning     Call this function after voxel3d_tof_init() is completed and successfully,
 *              otherwise, it returns false
 * @param[in]   dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *                      initialize the 1st scanned device
 * @param[out]  camera_info: pointer of uesr-allocated buffer to store camera info
 * @return      true: buffer shall be filled with related camera info
 * @return      < 0: failed to get camera info from library/device or error on inputa
 *                   parameter
 */
extern "C" VOXEL3D_API_DLL int voxel3d_tof_read_camera_info(char* dev_sn,
                                                            CameraInfo *cam_info);


/**
 * @brief       Get current temperature of ToF sensor in 5voxel device
 * @warning     This API supports only 5VSTDON
 * @warning     Call this function after voxel3d_tof_init() is completed and successfully,
 *              otherwise, it returns false
 * @param[in]   dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *                      initialize the 1st scanned device
 * @return      -273.15: error reading temperature
 * @return      Others: temperature in degree C
 */
extern "C" VOXEL3D_API_DLL float voxel3d_tof_get_sensor_temperature(char* dev_sn);


/**
 * @brief       Get current temperature of illumination VCSEL in 5voxel device
 * @warning     This API supports only 5VSTDON
 * @warning     Call this function after voxel3d_tof_init() is completed and successfully,
 *              otherwise, it returns false
 * @param[in]   dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *                      initialize the 1st scanned device
 * @return      -273.15: error reading temperature
 * @return      Others: temperature in degree C
 */
extern "C" VOXEL3D_API_DLL float voxel3d_tof_get_illum_temperature(char* dev_sn);


/**
 * @brief       Get total number of ToF capabilities associated with 5voxel device
 * @warning     Call this function after voxel3d_tof_init() is completed and successfully,
 *              otherwise, it returns false
 * @param[in]   dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *                      initialize the 1st scanned device
 * @return      >= 0: number of capabilities supported
 * @return      < 0: failed to get capability number
 */
extern "C" VOXEL3D_API_DLL int voxel3d_tof_get_total_caps_num(char* dev_sn);


/**
 * @brief       Get capability code of sepcific entry from capability list
 * @warning     Call this function after voxel3d_tof_init() is completed and successfully,
 *              otherwise, it returns false
 * @param[in]   dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *                      initialize the 1st scanned device
 * @param[in]   entry in capability list
 * @return      > 0: capability code
 * @return      < 0: failed to get capability code
 */
extern "C" VOXEL3D_API_DLL int voxel3d_tof_get_caps_code(char* dev_sn, 
                                                         unsigned int entry);


/**
 * @brief       Get capability name based on capability code
 * @warning     Call this function after voxel3d_tof_init() is completed and successfully,
 *              otherwise, it returns false
 * @param[in]   dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *                      initialize the 1st scanned device
 * @param[in]   caps_code: capability code
 * @param[out]  name: buffer to store capability name
 * @param[in]   len: length of buffer user allocated
 * @return      > 0: capability name is filled in buffer successfully
 * @return      < 0: failed to get name associated with selected capability code
 */
extern "C" VOXEL3D_API_DLL int voxel3d_tof_get_caps_name(char* dev_sn,
                                                         unsigned int caps_code,
                                                         char* name,
                                                         int len);


/**
 * @brief       Get maximum value of variable 1 & 2 for selected capability code
 * @warning     Call this function after voxel3d_tof_init() is completed and successfully,
 *              otherwise, it returns false
 * @param[in]   dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *                      initialize the 1st scanned device
 * @param[in]   caps_code: capability code
 * @param[out]  var1: pointer for library to fill in maximum value of variable 1 
 * @param[out]  var2: pointer for library to fill in maximum value of variable 2
 * @return      > 0: maximum values are filled in successfully
 * @return      < 0: failed to fill in maximum values
 */
extern "C" VOXEL3D_API_DLL int voxel3d_tof_get_caps_max(char* dev_sn,
                                                        unsigned int caps_code,
                                                        int* var1,
                                                        int* var2);


/**
 * @brief       Get minimum value of variable 1 & 2 for selected capability code
 * @warning     Call this function after voxel3d_tof_init() is completed and successfully,
 *              otherwise, it returns false
 * @param[in]   dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *                      initialize the 1st scanned device
 * @param[in]   caps_code: capability code
 * @param[out]  var1: pointer for library to fill in minimum value of variable 1 
 * @param[out]  var2: pointer for library to fill in minimum value of variable 2
 * @return      > 0: minimum values are filled in successfully
 * @return      < 0: failed to fill in minimum values
 */
extern "C" VOXEL3D_API_DLL int voxel3d_tof_get_caps_min(char* dev_sn,
                                                        unsigned int caps_code,
                                                        int* var1,
                                                        int* var2);


/**
 * @brief       Get current value of variable 1 & 2 for selected capability code
 * @warning     Call this function after voxel3d_tof_init() is completed and successfully,
 *              otherwise, it returns false
 * @param[in]   dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *                      initialize the 1st scanned device
 * @param[in]   caps_code: capability code
 * @param[out]  var1: pointer for library to fill in current value of variable 1 
 * @param[out]  var2: pointer for library to fill in current value of variable 2
 * @return      > 0: current values are filled in successfully
 * @return      < 0: failed to fill in current values
 */
extern "C" VOXEL3D_API_DLL int voxel3d_tof_get_caps_curr(char* dev_sn,
                                                         unsigned int caps_code,
                                                         int* var1,
                                                         int* var2);


/**
 * @brief       Set value of variable 1 & 2 for selected capability code
 * @warning     Call this function after voxel3d_tof_init() is completed and successfully,
 *              otherwise, it returns false
 * @param[in]   dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *                      initialize the 1st scanned device
 * @param[in]   caps_code: capability code
 * @param[in]   var1:      value of variable 1 to be set to device
 * @param[in]   var2:      value of variable 2 to be set to device
 * @return      > 0: set values successfully
 * @return      < 0: failed to set values
 */
extern "C" VOXEL3D_API_DLL int voxel3d_tof_set_caps(char* dev_sn,
                                                    unsigned int caps_code,
                                                    int var1,
                                                    int var2);


/**
 * @brief       Get ToF HFoV from 5voxel device
 * @warning     Call this function after voxel3d_tof_init() is completed and successfully,
 *              otherwise, it returns false
 * @param[in]   dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *                      initialize the 1st scanned device
 * @return      > 0: calculated ToF HFoV
 * @return      <=0: failed to get ToF HFoV
 */
extern "C" VOXEL3D_API_DLL float voxel3d_tof_get_depth_hfov(char* dev_sn);


/**
 * @brief       Get ToF VFoV from 5voxel device
 * @warning     Call this function after voxel3d_tof_init() is completed and successfully,
 *              otherwise, it returns false
 * @param[in]   dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *                      initialize the 1st scanned device
 * @return      > 0: calculated ToF VFoV
 * @return      <=0: failed to get ToF VFoV
 */
extern "C" VOXEL3D_API_DLL float voxel3d_tof_get_depth_vfov(char* dev_sn);


/**
 * @brief       Get current ToF modulation frequencies from 5voxel device
 * @warning     Call this function after voxel3d_tof_init() is completed and successfully,
 *              otherwise, it returns false
 * @param[in]   dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *                      initialize the 1st scanned device
 * @param[out]  mod_freq0: pointer of user-allocated buffer to store mod_freq0 value
 * @param[out]  mod_freq1: pointer of user-allocated buffer to store mod_freq1 value
 * @return      > 0: calculated ToF VFoV
 * @return      <=0: failed to get ToF VFoV
 */
extern "C" VOXEL3D_API_DLL int voxel3d_get_mod_freq(char* dev_sn, unsigned int* mod_freq0, unsigned int* mod_freq1);


/**
 * @brief       Get ToF debug info from 5voxel device
 * @warning     Call this function after voxel3d_tof_init() is completed and successfully,
 *              otherwise, it returns false
 * @param[in]   dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *                      initialize the 1st scanned device
 * @param[out]  data: pointer of user-allocated buffer for ToF debug info storage
 * @param[in]   data_len: length of user-allocated buffer
 * @return      > 0: get sensor info successfully
 * @return      <=0: failed to get sensor info
 */
extern "C" VOXEL3D_API_DLL int voxel3d_tof_get_debug_info(char* dev_sn, char *data, unsigned int data_len);

/**
 * @brief       Get ToF Calibration RAW data from 5voxel 5VSTDON device
 * @warning     This API supports only 5VSTDON
 * @warning     Call this function after voxel3d_tof_init() is completed and successfully,
 *              otherwise, it returns false.
 * @param[in]   dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *                      initialize the 1st scanned device
 * @param[out]  raw: pointer of user-allocated buffer for calibration data storage
 * @param[in]   raw_len: length of calibration data to be read (max: 8192 [8KBytes])
 * @return      > 0: read successfully
 * @return      <=0: failed to read data from device
 */
extern "C" VOXEL3D_API_DLL int voxel3d_tof_read_calibration_raw(char* dev_sn, char* raw, int raw_len);


/**
 * @brief       Perform the initialization of RGB on 5Voxel device
 * @warning     This function has to be called before voxel3d_rgb_queryframe(),
 *              otherwise the query will fail
 * @param[in]   dev_sn: device S/N, which can be read from the label on 5VHiRab device
 *                      or from result of voxel3d_scan(). Input S/N with NULL pointer
 *                      or empty string will initialize the 1st scanned device
 * @return      true: found device and init successfully
 * @return      < 0: can't find device or data error
 */
extern "C" VOXEL3D_API_DLL int voxel3d_rgb_init(char* dev_sn);


/**
 * @brief       Grab a rgb frame from 5Voxel device
 * @warning     Call voxel3d_rgb_init() to initialize specific device before query
 * @param[in]   dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *                      initialize the 1st scanned device
 * @param[out]  rgb_map: pointer of user-allocated buffer for Depth frame storage
 *                       data type: unsigned char
 *                       buffer size: RectifyType != RGB2TOF, RGB_WIDTH X RGB_HEIGHT
 *                                    RectifyType = RGB2TOF, TOF_DEPTH_WIDTH X TOF_DEPTH_HEIGHT
 * @param[in]   flipRedAndBlue: change RGB output to RGB output
 * @return      > 0: current frame count (1 ~ UINT_MAX)
 * @return      = 0: no new frame from device
 */
extern "C" VOXEL3D_API_DLL unsigned int voxel3d_rgb_queryframe(char* dev_sn, 
                                                               unsigned char* rgb_map,
                                                               bool flipRedAndBlue = false);


/**
  * @brief       Release the resource allocated for RGB on 5Voxel device
  * @warning     This function has to be called before program exit
  * @param[in]   dev_sn: device S/N. Input S/N with NULL pointer or empty string will
  *                      initialize the 1st scanned device
  */
extern "C" VOXEL3D_API_DLL void voxel3d_rgb_release(char* dev_sn);


/**
 * @brief       Get RGB frame width associated with dev_sn
 * @warning     Call this function after voxel3d_rgb_init() is completed and successfully,
 *              otherwise, it returns false
 * @param[in]   dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *                      initialize the 1st scanned device
 * @return      > 0: RGB frame width
 * @return      < 0: error code
 */
extern "C" VOXEL3D_API_DLL int voxel3d_rgb_get_width(char* dev_sn);


/**
 * @brief       Get RGB frame height associated with dev_sn
 * @warning     Call this function after voxel3d_rgb_init() is completed and successfully,
 *              otherwise, it returns false
 * @param[in]   dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *                      initialize the 1st scanned device
 * @return      > 0: RGB frame height
 * @return      < 0: error code
 */
extern "C" VOXEL3D_API_DLL int voxel3d_rgb_get_height(char* dev_sn);


/**
 * @brief       Get RGB HFoV from 5voxel device
 * @warning     Call this function after voxel3d_rgb_init() is completed and successfully,
 *              otherwise, it returns false
 * @param[in]   dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *                      initialize the 1st scanned device
 * @return      > 0: calculated RGB HFoV
 * @return      <=0: failed to get RGB HFoV
 */
extern "C" VOXEL3D_API_DLL float voxel3d_rgb_get_hfov(char* dev_sn);


/**
 * @brief       Get RGB VFoV from 5voxel device
 * @warning     Call this function after voxel3d_rgb_init() is completed and successfully,
 *              otherwise, it returns false
 * @param[in]   dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *                      initialize the 1st scanned device
 * @return      > 0: calculated RGB VFoV
 * @return      <=0: failed to get RGB VFoV
 */
extern "C" VOXEL3D_API_DLL float voxel3d_rgb_get_vfov(char* dev_sn);


/**
 * @brief       Get rgb camera intrinsic & distortion information from 5voxel device
 * @warning     This function has to be called after voxel3d_rgb_init()
 * @param[in]   dev_sn: device S/N, which can be read from the label on 5VHiRab device
 *                      or from result of voxel3d_scan(). Input S/N with NULL pointer
 *                      or empty string will initialize the 1st scanned device
 * @param[out]  cam_info: pointer of user-allocated buffer to store camera intrinsic &
                          distortion information
 * @return      true: get camera information successfully
 * @return      false: failed to read camera information from device
 */
extern "C" VOXEL3D_API_DLL int voxel3d_rgb_read_camera_info(char* dev_sn,
                                                            CameraInfo * cam_info);


/**
 * @brief       Perform the initialization of Lepton camera on 5Voxel device
 * @note        Support 160x120 FLIR Leption 3.5
 * @warning     This API supports only 5HiRab
 * @warning     This function has to be called before voxel3d_lepton3_queryframe(),
 *              otherwise the query will fail
 * @param[in]   dev_sn: device S/N, which can be read from the label on 5VHiRab device
 *                      or from result of voxel3d_scan(). Input S/N with NULL pointer
 *                      or empty string will initialize the 1st scanned device
 * @return      true: found device and init successfully
 * @return      < 0: can't find device or data error
 */
extern "C" VOXEL3D_API_DLL int voxel3d_lepton3_init(char* dev_sn);


/**
 * @brief       Grab a thermal image frame from 5Voxel device
 * @note        Support 160x120 FLIR Leption 3.5
 * @warning     This API supports only 5HiRab
 * @warning     Call voxel3d_lepton3_init() to initialize specific device before query
 * @param[in]   dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *                      initialize the 1st scanned device
 * @param[out]  thermal_img: pointer of user-allocated buffer for thermal image storage
 *                           data type: unsigned char
 *                           buffer size: RectifyType != FLIR2TOF, FLIR_WIDTH X FLIR_HEIGHT
 *                                        RectifyType = FLIR2TOF, TOF_DEPTH_WIDTH X TOF_DEPTH_HEIGHT
 * @return      > 0: current frame count (1 ~ UINT_MAX)
 * @return      = 0: no new frame from device
 */
extern "C" VOXEL3D_API_DLL unsigned int voxel3d_lepton3_queryframe(char* dev_sn,
                                                                   float* thremal_map);


/**
 * @brief       Get part number and shutter mode from FLIR
 * @note        Support 160x120 FLIR Leption 3.5
 * @warning     This API supports only 5HiRab
 * @warning     Call voxel3d_lepton3_init() to initialize specific device before query
 * @param[in]   dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *                      initialize the 1st scanned device
 * @param[out]  pn: pointer of user-allocated buffer for part number storage
 *                  data type: unsigned char
 *                  buffer size: 32 bytes
 * @param[out]  shutter_mode: pointer of user-allocated buffer for shutter mode storage
 *                            data type: int
 *                            0 -> Manual
 *                            1 -> Auto
 *                            2 -> External
 *                            Others -> Unknown
 * @return      true: found device and init successfully
 * @return      < 0: can't find device or data error
 */
extern "C" VOXEL3D_API_DLL int voxel3d_lepton3_get_pn_shutter_mode(char* dev_sn,
                                                                   char* pn,
                                                                   int* shutter_mode);

/**
 * @brief       reset Flir-Depth alignment rotation value, rotate follow x-y-z
 * @note        Support 160x120 FLIR Leption 
                    Default FLIR[500-0758-03, shutter mode: Auto]
                        wx = -1.5f * pi / 180.0f
                        wy = 0.1f * pi / 180.0f
                        wz = 0
                    Default FLIR Leption [500-0771-01, shutter mode: Auto]
                        wx = 0
                        wy = 2.5f * pi / 180.0f
                        wz = 0
                    Default FLIR Leption [500-0771-01, shutter mode: External]
                        wx = -6.0f * pi / 180.0f
                        wy = -1.0f * pi / 180.0f
                        wz = 0
 * @warning     This API supports only 5HiRab
 * @warning     Call voxel3d_lepton3_init() to initialize specific device before setting
 * @param[in]   dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *                      initialize the 1st scanned device
 * @param[in]   wx: rotate with x-axis
 * @param[in]   wy: rotate with y-axis
 * @param[in]   wz: rotate with z-axis
 */
extern "C" VOXEL3D_API_DLL void voxel3d_lepton3_reset_RotVector(char* dev_sn, float wx, float wy, float wz);


/**
 * @brief       reset Flir-Depth alignment translation value
 * @note        Support 160x120 FLIR Leption
                    Default FLIR[500-0758-03, shutter mode: Auto]
                        tx = 20
                        ty = 0
                        tz = 0
                    Default FLIR Leption [500-0771-01, shutter mode: Auto]
                        tx = 20
                        ty = 0
                        tz = 0
                    Default FLIR Leption [500-0771-01, shutter mode: External]
                        tx = 15
                        ty = 0
                        tz = 0
 * @warning     This API supports only 5HiRab
 * @warning     Call voxel3d_lepton3_init() to initialize specific device before setting
 * @param[in]   dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *                      initialize the 1st scanned device
 * @param[in]   tx: rotate with x-axis
 * @param[in]   ty: rotate with y-axis
 * @param[in]   tz: rotate with z-axis
 */
extern "C" VOXEL3D_API_DLL void voxel3d_lepton3_reset_Translate(char* dev_sn, float tx, float ty, float tz);


/**
 * @brief       Release the resource allocated for Lepton camera on 5Voxel device
 * @warning     This API supports only 5HiRab
 * @warning     This function has to be called before program exit
 * @param[in]   dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *                      initialize the 1st scanned device
 */
extern "C" VOXEL3D_API_DLL void voxel3d_lepton3_release(char* dev_sn);


/**
 * @brief       Get Lepton3 frame width associated with dev_sn
 * @warning     This API supports only 5HiRab
 * @warning     Call this function after voxel3d_lepton3_init() is completed and successfully,
 *              otherwise, it returns false
 * @param[in]   dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *                      initialize the 1st scanned device
 * @return      > 0: Lepton3 frame width
 * @return      < 0: error code
 */
extern "C" VOXEL3D_API_DLL int voxel3d_lepton3_get_width(char* dev_sn);


/**
 * @brief       Get Lepton3 frame height associated with dev_sn
 * @warning     This API supports only 5HiRab
 * @warning     Call this function after voxel3d_lepton3_init() is completed and successfully,
 *              otherwise, it returns false
 * @param[in]   dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *                      initialize the 1st scanned device
 * @return      > 0: Lepton3 frame height
 * @return      < 0: error code
 */
extern "C" VOXEL3D_API_DLL int voxel3d_lepton3_get_height(char* dev_sn);


/**
 * @brief       Get Lepton3 HFoV associated with dev_sn
 * @warning     Call this function after voxel3d_lepton3_init() is completed and successfully,
 *              otherwise, it returns false
 * @param[in]   dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *                      initialize the 1st scanned device
 * @return      > 0: calculated Lepton3 HFoV
 * @return      <=0: failed to get Lepton3 HFoV
 */
extern "C" VOXEL3D_API_DLL float voxel3d_lepton3_get_hfov(char* dev_sn);


/**
 * @brief       Get Lepton3 VFoV associated with dev_sn
 * @warning     Call this function after voxel3d_lepton3_init() is completed and successfully,
 *              otherwise, it returns false
 * @param[in]   dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *                      initialize the 1st scanned device
 * @return      > 0: calculated Lepton3 VFoV
 * @return      <=0: failed to get Lepton3 VFoV
 */
extern "C" VOXEL3D_API_DLL float voxel3d_lepton3_get_vfov(char* dev_sn);


/**
 * @brief       Grab thermal info from 5voxel device
 * @warning     This API supports only 5HiRab
 * @warning     Call this function after voxel3d_lepton3_init() is completed and successfully,
 *              otherwise, it returns false
 * @param[in]   dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *                      initialize the 1st scanned device
 * @param[out]  camera_params: pointer of uesr-allocated buffer to store camera info
 * @return      true: buffer shall be filled with related camera info
 * @return      < 0: failed to get camera info from library/device or error on inputa
 *                   parameter
 */
extern "C" VOXEL3D_API_DLL int voxel3d_lepton3_read_camera_info(char* dev_sn,
                                                                CameraInfo* cam_info);


/**
 * @brief       Release all cameras associated with the input device S/N
 * @note        User can either release single camera (tof, rgb, lepton3) by calling
 *              its release fucntion or call this to release all in one shot.
 *              Suggest to call this if user wants to release everything before program
 *              exit.
 * @param[in]   dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *                      initialize the 1st scanned device
 */
extern "C" VOXEL3D_API_DLL void voxel3d_release(char* dev_sn);


/**
 * @brief       Grab IMU data from 5Voxel device
 * @warning     Call this function after voxel3d_tof_init() is completed and successfully,
 *              otherwise, it returns false
 * @param[in]   dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *                      initialize the 1st scanned device
 * @param[out]  imu_data: pointer of uesr-allocated buffer to store imu data
 * @return      true: buffer shall be filled with related imu data
 * @return      0: no IMU data available
 */
extern "C" VOXEL3D_API_DLL int voxel3d_read_imu_data(char* dev_sn, IMU_DATA* imu_data);


/**
 * @brief       Set 5voxel device rectified mode
 * @param[in]   dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *                      initialize the 1st scanned device
 * @param[in]   inputType: 0 -> NONE, 
 *                         1 -> RGB - TOF (RGB output will become 640x480, aligned with ToF)
 *                         2 -> THERMAL(FLIR) - TOF (FLIR output will become 640x480, aligned with ToF)
 *                         3 -> TOF - RGB (HW, on camera side)
 * @return      true: set device alignment type successfully
 * @return      false: failed to set rectified mode
 */
extern "C" VOXEL3D_API_DLL int voxel3d_set_rectifyType(char* dev_sn, int inputType);


/**
 * @brief       Read out camera F/W version
 * @warning     Call this function after voxel3d_tof_init() is completed and successfully,
 *              otherwise, it returns false
 * @param[in]   dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *                      initialize the 1st scanned device
 * @param[out]  fw_ver: pointer of user-allocated buffer to store fw version string
 * @param[in]   max_len: length of user-allocated buffer
 * @return      true: buffer shall be filled with F/W version string
 * @return      < 0: failed to get F/W version from device or error on input parameters
 */
extern "C" VOXEL3D_API_DLL int voxel3d_read_fw_version(char* dev_sn,
                                                       char* fw_ver,
                                                       unsigned int max_len);


/**
 * @brief       Read out camera F/W build date
 * @warning     Call this function after voxel3d_tof_init() is completed and successfully,
 *              otherwise, it returns false
 * @param[in]   dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *                      initialize the 1st scanned device
 * @param[out]  fw_build_date: pointer of user-allocated buffer to store fw build date
 *                    string
 * @param[in]   max_len: length of user-allocated buffer
 * @return      true: buffer shall be filled with F/W build date string
 * @return      < 0: failed to get F/W build date from device or error on input parameters
 */
extern "C" VOXEL3D_API_DLL int voxel3d_read_fw_build_date(char* dev_sn,
                                                          char* fw_build_date,
                                                          unsigned int max_len);


/**
 * @brief       Read out library version
 * @param[out]  lib_version: pointer of user-allocated buffer to store library version
 *                           string
 * @param[in]   max_len: length of user-allocated buffer
 * @return      true: buffer shall be filled with library version string
 * @return      false: failed to get library version
 */
extern "C" VOXEL3D_API_DLL int voxel3d_read_lib_version(char* lib_version,
                                                        int max_len);


/**
 * @brief       Read out library build date
 * @param[out]  lib_build_date: pointer of user-allocated buffer to store library build date
 *                              string
 * @param[in]   max_len: length of user-allocated buffer
 * @return      true: buffer shall be filled with library build date string
 * @return      false: failed to get library build date
 */
extern "C" VOXEL3D_API_DLL int voxel3d_read_lib_build_date(char* lib_build_date,
                                                           int max_len);


/**
 * @brief       5voxel device firmware upgrade utility
 * @warning     Call this function after voxel3d_tof_init() is completed and successfully,
 *              otherwise, it returns false
 * @warning     Device firmware upgrade utility allows both upgrade to new version of firmware
 *              and also downgrade to older version. User can use voxel3d_read_fw_version() to
 *              confirm the firmware version running on device. It can also be used to confirm
 *              if the firmware version on device after upgrade.
 * @param[in]   dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *                      initialize the 1st scanned device
 * @param[in]   file_path: point to the location of the new firmware image string
 * @return      true: completed sending specific firmware to device for upgrade successfully
 * @return      < 0: upgrade failure
 */
extern "C" VOXEL3D_API_DLL int voxel3d_dev_fw_upgrade(char* dev_sn,
    char* file_path,
    unsigned char (*fw_upgrade_cb)(int state, unsigned int percent_complete));


/**
 * @brief       Function to poll the state and 5voxel device firmware upgrade utility
 * @warning     Call this function after voxel3d_tof_init() is completed and successfully,
 *              otherwise, it returns false
 * @param[in]   dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *                      initialize the 1st scanned device
 * @param[out]  state: reference of the state varaible for API to write current fw download state
 *              state: < 0 (error), 0: (initial), 1 (downloading), 2 (complete)
 * @param[out]  percent_complete: reference of the percentage varaible for API to write current percentage of completion
 *              percent_complete: 0 ~ 100
 * @return      true: In upgrade procedure
 * @return      < 0: Not in upgrade procedure
 *              Note: while output < 0, state & percent_complete can be used to know if the previous upgrade
 *                    had error or completed withtout failure
 */
extern "C" VOXEL3D_API_DLL int voxel3d_dev_fw_upgrade_state_poll(char* dev_sn, int& state,
    unsigned int& percent_complete);


/**
 * @brief       Function to get use case configuration from camera
 * @warning     Call this function after voxel3d_tof_init() is completed and successfully,
 *              otherwise, it returns false
 * @param[in]   dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *                      initialize the 1st scanned device
 * @return      > 0: use case number (range: 1 ~ 5)
 * @return      = 0: use case number from camera is invalid
 * @return      < 0: failed to get use case from camera
 */
extern "C" VOXEL3D_API_DLL int voxel3d_get_usecase(char* dev_sn);


/**
 * @brief       Function to set use case configuration to camera
 * @note        Valid use case range (1 ~ 5). Refer to enum use_case{} above
 *              Once set use case is done, camera will reboot itself to setup proper
 *              parameters based on selected use case. Don't power off camera during use
 *              case switch. User will need to call voxel3d_tof_release(), wait for 15
 *              seconds, then call voxel3d_tof_init() to check if camera is up
 * @warning     Call this function after voxel3d_tof_init() is completed and successfully,
 *              otherwise, it returns false
 * @param[in]   dev_sn: device S/N. Input S/N with NULL pointer or empty string will
 *                      initialize the 1st scanned device
 * @param[in]   target_usecase: target use case to set to camera
 * @return      true: set use case successfully
 * @return      <= 0: failed to set use case from camera
 */
extern "C" VOXEL3D_API_DLL int voxel3d_set_usecase(char* dev_sn, unsigned int target_usecase);


#endif /* __VOXEL3d_H__ */

