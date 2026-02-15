/**
 * @file hk_camera_node.cpp
 * @brief 海康威视 GigE 工业相机图像采集 ROS2 节点 (Jetson GPU 加速版)
 * @details 基于 MVS SDK 实现相机初始化、连续取流、图像格式转换，
 *          并通过 ROS2 话题发布 sensor_msgs::msg::Image 消息。
 *          相机参数：传感器全画幅采集后软件缩放至 640x480，帧率 60fps，Free-Run 连续采集模式。
 *          编译时定义 USE_CUDA 可启用 Jetson GPU 加速颜色空间转换及图像缩放。
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#ifdef USE_CUDA
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudawarping.hpp>
#endif

#include <thread>
#include <atomic>
#include <cstring>

#include "MvCameraControl.h"

/** @brief 目标图像宽度 */
static constexpr unsigned int TARGET_WIDTH = 640;
/** @brief 目标图像高度 */
static constexpr unsigned int TARGET_HEIGHT = 480;
/** @brief 目标帧率 (fps) */
static constexpr float TARGET_FPS = 60.0f;
/** @brief 目标曝光时间 (微秒) */
static constexpr float TARGET_EXPOSURE_TIME = 10000.0f;

/**
 * @class HkCameraNode
 * @brief 海康威视工业相机 ROS2 节点类 (Jetson GPU 加速版)
 * @details 负责相机设备的枚举、打开、参数配置、连续取流，
 *          将原始图像转换为 BGR8 格式后通过 ROS2 话题发布。
 *          编译时若定义 USE_CUDA 且运行时检测到 CUDA 设备，
 *          颜色空间转换将在 GPU 上完成；否则自动回退到 CPU 处理。
 */
class HkCameraNode : public rclcpp::Node
{
public:
    /**
     * @brief 构造函数，初始化相机并启动取流线程
     */
    HkCameraNode()
        : Node("hk_camera_node"),
          is_running_(false),
          handle_(nullptr),
          payload_size_(0),
          use_cuda_(false)
    {
        /* 检测并初始化 CUDA GPU (仅在编译时启用 CUDA 时有效) */
        initCuda();

        /* 创建图像发布者，使用 SensorDataQoS 以获得更低延迟 */
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "image_raw", rclcpp::SensorDataQoS());

        /* 创建 CameraInfo 发布者 */
        cam_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
            "camera_info", rclcpp::SensorDataQoS());

        /* 声明相机内参参数（标定值，可通过 launch 文件或命令行覆盖） */
        this->declare_parameter<double>("camera_info.fx", 877.4866);
        this->declare_parameter<double>("camera_info.fy", 875.7676);
        this->declare_parameter<double>("camera_info.cx", 297.0512);
        this->declare_parameter<double>("camera_info.cy", 233.9948);
        this->declare_parameter<std::vector<double>>("camera_info.distortion",
            std::vector<double>{-0.056809, 0.158542, -0.001542, -0.001099, -0.464843});

        /* 构建 CameraInfo 消息 */
        initCameraInfo();

        /* 初始化相机 */
        if (!initCamera())
        {
            RCLCPP_ERROR(this->get_logger(), "相机初始化失败，节点将退出");
            rclcpp::shutdown();
            return;
        }

        /* 启动取流线程 */
        is_running_ = true;
        grab_thread_ = std::thread(&HkCameraNode::grabLoop, this);

        RCLCPP_INFO(this->get_logger(),
                     "海康威视相机节点已启动，分辨率: %ux%u，帧率: %.0ffps，GPU加速: %s",
                     TARGET_WIDTH, TARGET_HEIGHT, TARGET_FPS,
                     use_cuda_ ? "已启用" : "未启用(CPU模式)");
    }

    /**
     * @brief 析构函数，停止取流并释放相机资源
     */
    ~HkCameraNode()
    {
        is_running_ = false;
        if (grab_thread_.joinable())
        {
            grab_thread_.join();
        }
        cleanupCamera();
        RCLCPP_INFO(this->get_logger(), "海康威视相机节点已关闭");
    }

private:
    /**
     * @brief 初始化 CameraInfo 消息
     * @details 从 ROS2 参数读取相机内参 (fx, fy, cx, cy) 和畸变系数，
     *          构建 sensor_msgs::msg::CameraInfo 消息。
     *          用户应通过标定获取实际参数后在 launch 文件中覆盖默认值。
     */
    void initCameraInfo()
    {
        double fx = this->get_parameter("camera_info.fx").as_double();
        double fy = this->get_parameter("camera_info.fy").as_double();
        double cx = this->get_parameter("camera_info.cx").as_double();
        double cy = this->get_parameter("camera_info.cy").as_double();
        auto dist = this->get_parameter("camera_info.distortion").as_double_array();

        cam_info_msg_.header.frame_id = "camera_optical_frame";
        cam_info_msg_.width = TARGET_WIDTH;
        cam_info_msg_.height = TARGET_HEIGHT;
        cam_info_msg_.distortion_model = "plumb_bob";

        /* 畸变系数 D: [k1, k2, p1, p2, k3] */
        cam_info_msg_.d.resize(5, 0.0);
        for (size_t i = 0; i < std::min(dist.size(), cam_info_msg_.d.size()); i++)
        {
            cam_info_msg_.d[i] = dist[i];
        }

        /* 相机内参矩阵 K (3x3, 行主序) */
        cam_info_msg_.k = {fx, 0.0, cx,
                           0.0, fy, cy,
                           0.0, 0.0, 1.0};

        /* 矫正矩阵 R (单目为单位阵) */
        cam_info_msg_.r = {1.0, 0.0, 0.0,
                           0.0, 1.0, 0.0,
                           0.0, 0.0, 1.0};

        /* 投影矩阵 P (3x4) */
        cam_info_msg_.p = {fx, 0.0, cx, 0.0,
                           0.0, fy, cy, 0.0,
                           0.0, 0.0, 1.0, 0.0};

        RCLCPP_INFO(this->get_logger(),
                     "CameraInfo 已初始化: fx=%.1f, fy=%.1f, cx=%.1f, cy=%.1f",
                     fx, fy, cx, cy);
    }

    /**
     * @brief 初始化 CUDA GPU
     * @details 编译时启用 USE_CUDA 后，检测 Jetson 上的 CUDA 设备并设置 GPU。
     *          未启用 USE_CUDA 或无 CUDA 设备时，use_cuda_ 保持 false。
     */
    void initCuda()
    {
#ifdef USE_CUDA
        int cuda_device_count = cv::cuda::getCudaEnabledDeviceCount();
        if (cuda_device_count > 0)
        {
            cv::cuda::setDevice(0);
            cv::cuda::DeviceInfo dev_info(0);
            use_cuda_ = true;
            RCLCPP_INFO(this->get_logger(),
                         "CUDA GPU 加速已启用: %s (计算能力 %d.%d, 显存 %zuMB)",
                         dev_info.name(),
                         dev_info.majorVersion(),
                         dev_info.minorVersion(),
                         dev_info.totalMemory() / (1024 * 1024));
        }
        else
        {
            RCLCPP_WARN(this->get_logger(),
                         "编译已启用 CUDA 但未检测到 CUDA 设备，回退到 CPU 模式");
        }
#else
        RCLCPP_INFO(this->get_logger(),
                     "编译时未启用 CUDA (USE_CUDA)，使用纯 CPU 模式。"
                     "安装 CUDA + 重编译 OpenCV with CUDA 后可启用 GPU 加速");
#endif
    }

    /**
     * @brief 初始化相机
     * @details 按顺序执行：枚举设备 -> 创建句柄 -> 打开设备 ->
     *          设置最优包大小 -> 关闭触发模式(Free-Run) ->
     *          设置全画幅分辨率 -> 设置帧率(60fps) ->
     *          开始取流 -> 获取 PayloadSize
     * @return true 初始化成功
     * @return false 初始化失败
     */
    bool initCamera()
    {
        int nRet = MV_OK;

        /* 1. 枚举设备 */
        MV_CC_DEVICE_INFO_LIST stDeviceList;
        memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
        nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
        if (MV_OK != nRet)
        {
            RCLCPP_ERROR(this->get_logger(), "枚举设备失败! nRet [0x%x]", nRet);
            return false;
        }

        if (stDeviceList.nDeviceNum == 0)
        {
            RCLCPP_ERROR(this->get_logger(), "未找到任何相机设备!");
            return false;
        }

        /* 打印设备信息 */
        for (unsigned int i = 0; i < stDeviceList.nDeviceNum; i++)
        {
            MV_CC_DEVICE_INFO *pDeviceInfo = stDeviceList.pDeviceInfo[i];
            if (pDeviceInfo && pDeviceInfo->nTLayerType == MV_GIGE_DEVICE)
            {
                int nIp1 = ((pDeviceInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
                int nIp2 = ((pDeviceInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
                int nIp3 = ((pDeviceInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
                int nIp4 = (pDeviceInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);
                RCLCPP_INFO(this->get_logger(), "[设备 %d] 型号: %s, IP: %d.%d.%d.%d",
                            i, pDeviceInfo->SpecialInfo.stGigEInfo.chModelName,
                            nIp1, nIp2, nIp3, nIp4);
            }
        }

        /* 默认选择第一个设备 */
        unsigned int nIndex = 0;
        RCLCPP_INFO(this->get_logger(), "选择设备 [%d]", nIndex);

        /* 2. 创建句柄 */
        nRet = MV_CC_CreateHandle(&handle_, stDeviceList.pDeviceInfo[nIndex]);
        if (MV_OK != nRet)
        {
            RCLCPP_ERROR(this->get_logger(), "创建句柄失败! nRet [0x%x]", nRet);
            return false;
        }

        /* 3. 打开设备 */
        nRet = MV_CC_OpenDevice(handle_);
        if (MV_OK != nRet)
        {
            RCLCPP_ERROR(this->get_logger(), "打开设备失败! nRet [0x%x]", nRet);
            return false;
        }

        /* 4. 设置 GigE 相机最优包大小 */
        if (stDeviceList.pDeviceInfo[nIndex]->nTLayerType == MV_GIGE_DEVICE)
        {
            int nPacketSize = MV_CC_GetOptimalPacketSize(handle_);
            if (nPacketSize > 0)
            {
                nRet = MV_CC_SetIntValue(handle_, "GevSCPSPacketSize", nPacketSize);
                if (nRet != MV_OK)
                {
                    RCLCPP_WARN(this->get_logger(), "设置包大小失败 nRet [0x%x]", nRet);
                }
                else
                {
                    RCLCPP_INFO(this->get_logger(), "GigE 最优包大小: %d", nPacketSize);
                }
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "获取最优包大小失败 nRet [0x%x]", nPacketSize);
            }
        }

        /* 5. 关闭触发模式，使用 Free-Run 连续采集模式 */
        nRet = MV_CC_SetEnumValue(handle_, "TriggerMode", 0);
        if (MV_OK != nRet)
        {
            RCLCPP_WARN(this->get_logger(), "设置触发模式失败! nRet [0x%x]", nRet);
        }

        /* 5.5 设置像素格式为 BayerRG8 (原始 Bayer，支持 GPU 解马赛克，带宽减半) */
        if (!setPixelFormat())
        {
            RCLCPP_WARN(this->get_logger(), "设置像素格式失败，将使用相机默认格式");
        }

        /* 6. 设置全画幅采集 (使用传感器最大分辨率，软件端缩放至 640x480) */
        if (!setResolution())
        {
            RCLCPP_WARN(this->get_logger(), "设置分辨率失败，将使用相机默认分辨率");
        }

        /* 7. 设置帧率为 60fps */
        if (!setFrameRate())
        {
            RCLCPP_WARN(this->get_logger(), "设置帧率失败，将使用相机默认帧率");
        }

        /* 7.5 设置曝光时间 */
        if (!setExposure())
        {
            RCLCPP_WARN(this->get_logger(), "设置曝光时间失败，将使用相机默认曝光");
        }

        /* 8. 开始取流 */
        nRet = MV_CC_StartGrabbing(handle_);
        if (MV_OK != nRet)
        {
            RCLCPP_ERROR(this->get_logger(), "开始取流失败! nRet [0x%x]", nRet);
            return false;
        }

        /* 9. 获取 PayloadSize */
        MVCC_INTVALUE stParam;
        memset(&stParam, 0, sizeof(MVCC_INTVALUE));
        nRet = MV_CC_GetIntValue(handle_, "PayloadSize", &stParam);
        if (MV_OK != nRet)
        {
            RCLCPP_ERROR(this->get_logger(), "获取 PayloadSize 失败! nRet [0x%x]", nRet);
            return false;
        }
        payload_size_ = stParam.nCurValue;
        RCLCPP_INFO(this->get_logger(), "PayloadSize: %u", payload_size_);

        return true;
    }

    /**
     * @brief 设置相机像素格式为 BayerRG8
     * @details 使用原始 Bayer 格式替代 YUV422，优势：
     *          1. 数据量减半 (8bpp vs 16bpp)，提升 GigE 传输帧率
     *          2. BayerRG8 直接支持 GPU 解马赛克 (cv::cuda::demosaicing)
     *          3. 避免相机内部 ISP 处理延迟
     *          如果 BayerRG8 不支持，依次尝试其他 Bayer 格式。
     * @return true 设置成功
     * @return false 设置失败
     */
    bool setPixelFormat()
    {
        int nRet = MV_OK;

        /**
         * @brief 按优先级尝试的像素格式列表
         * @details BayerRG8 最常见，其次是其他 Bayer 格式
         */
        struct PixelFormatCandidate
        {
            unsigned int value;
            const char *name;
        };

        const PixelFormatCandidate candidates[] = {
            {PixelType_Gvsp_BayerRG8, "BayerRG8"},
            {PixelType_Gvsp_BayerGR8, "BayerGR8"},
            {PixelType_Gvsp_BayerGB8, "BayerGB8"},
            {PixelType_Gvsp_BayerBG8, "BayerBG8"},
        };

        for (const auto &fmt : candidates)
        {
            nRet = MV_CC_SetEnumValue(handle_, "PixelFormat", fmt.value);
            if (MV_OK == nRet)
            {
                RCLCPP_INFO(this->get_logger(),
                             "像素格式已设置: %s (0x%x) — 支持 GPU 解马赛克",
                             fmt.name, fmt.value);
                return true;
            }
        }

        RCLCPP_WARN(this->get_logger(),
                     "所有 Bayer 格式均不支持，保持当前像素格式 (可能无法使用 GPU 加速)");
        return false;
    }

    /**
     * @brief 设置相机分辨率为传感器最大值（全画幅采集）
     * @details 使用传感器的最大分辨率采集完整画面，后续在软件端
     *          通过 cv::resize 缩放到 TARGET_WIDTH x TARGET_HEIGHT。
     *          相比 ROI 裁切，全画幅采集保留完整视野。
     * @return true 设置成功
     * @return false 设置失败
     */
    bool setResolution()
    {
        int nRet = MV_OK;

        /* 先将 Offset 归零，确保使用完整传感器区域 */
        MV_CC_SetIntValue(handle_, "OffsetX", 0);
        MV_CC_SetIntValue(handle_, "OffsetY", 0);

        /* 获取传感器最大分辨率 */
        MVCC_INTVALUE stWidthMax, stHeightMax;
        memset(&stWidthMax, 0, sizeof(MVCC_INTVALUE));
        memset(&stHeightMax, 0, sizeof(MVCC_INTVALUE));

        if (MV_OK != MV_CC_GetIntValue(handle_, "WidthMax", &stWidthMax) ||
            MV_OK != MV_CC_GetIntValue(handle_, "HeightMax", &stHeightMax))
        {
            RCLCPP_WARN(this->get_logger(), "获取传感器最大分辨率失败，将使用相机默认分辨率");
            return false;
        }

        unsigned int sensorWidth = stWidthMax.nCurValue;
        unsigned int sensorHeight = stHeightMax.nCurValue;

        /* 设置宽度为传感器最大值 */
        nRet = MV_CC_SetIntValue(handle_, "Width", sensorWidth);
        if (MV_OK != nRet)
        {
            RCLCPP_WARN(this->get_logger(), "设置宽度 %u 失败! nRet [0x%x]", sensorWidth, nRet);
            return false;
        }

        /* 设置高度为传感器最大值 */
        nRet = MV_CC_SetIntValue(handle_, "Height", sensorHeight);
        if (MV_OK != nRet)
        {
            RCLCPP_WARN(this->get_logger(), "设置高度 %u 失败! nRet [0x%x]", sensorHeight, nRet);
            return false;
        }

        RCLCPP_INFO(this->get_logger(),
                     "传感器全画幅采集: %ux%u -> 软件缩放至 %ux%u",
                     sensorWidth, sensorHeight, TARGET_WIDTH, TARGET_HEIGHT);

        return true;
    }

    /**
     * @brief 设置相机帧率为 TARGET_FPS
     * @details 先启用帧率控制，再设置目标帧率值
     * @return true 设置成功
     * @return false 设置失败
     */
    bool setFrameRate()
    {
        int nRet = MV_OK;

        /* 启用帧率控制 */
        nRet = MV_CC_SetBoolValue(handle_, "AcquisitionFrameRateEnable", true);
        if (MV_OK != nRet)
        {
            RCLCPP_WARN(this->get_logger(), "启用帧率控制失败! nRet [0x%x]", nRet);
            return false;
        }

        /* 设置帧率 */
        nRet = MV_CC_SetFloatValue(handle_, "AcquisitionFrameRate", TARGET_FPS);
        if (MV_OK != nRet)
        {
            RCLCPP_WARN(this->get_logger(), "设置帧率 %.0f 失败! nRet [0x%x]", TARGET_FPS, nRet);
            return false;
        }

        /* 读回实际帧率以确认 */
        MVCC_FLOATVALUE stFrameRate;
        memset(&stFrameRate, 0, sizeof(MVCC_FLOATVALUE));
        if (MV_OK == MV_CC_GetFloatValue(handle_, "ResultingFrameRate", &stFrameRate))
        {
            RCLCPP_INFO(this->get_logger(), "帧率已设置: 目标 %.0ffps, 实际 %.1ffps",
                         TARGET_FPS, stFrameRate.fCurValue);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "帧率已设置: %.0ffps", TARGET_FPS);
        }

        return true;
    }

    /**
     * @brief 设置相机曝光时间
     * @details 先关闭自动曝光（切换为手动模式），再设置目标曝光时间。
     *          曝光时间单位为微秒 (us)。
     * @return true 设置成功
     * @return false 设置失败
     */
    bool setExposure()
    {
        int nRet = MV_OK;

        /* 关闭自动曝光，切换为手动模式 (0 = Off) */
        nRet = MV_CC_SetEnumValue(handle_, "ExposureAuto", 0);
        if (MV_OK != nRet)
        {
            RCLCPP_WARN(this->get_logger(), "关闭自动曝光失败! nRet [0x%x]", nRet);
            return false;
        }

        /* 设置曝光时间 (单位: 微秒) */
        nRet = MV_CC_SetFloatValue(handle_, "ExposureTime", TARGET_EXPOSURE_TIME);
        if (MV_OK != nRet)
        {
            RCLCPP_WARN(this->get_logger(), "设置曝光时间 %.0fus 失败! nRet [0x%x]",
                         TARGET_EXPOSURE_TIME, nRet);
            return false;
        }

        /* 读回实际曝光时间以确认 */
        MVCC_FLOATVALUE stExposureTime;
        memset(&stExposureTime, 0, sizeof(MVCC_FLOATVALUE));
        if (MV_OK == MV_CC_GetFloatValue(handle_, "ExposureTime", &stExposureTime))
        {
            RCLCPP_INFO(this->get_logger(),
                         "曝光时间已设置: 目标 %.0fus, 实际 %.1fus (范围: %.1f ~ %.1fus)",
                         TARGET_EXPOSURE_TIME, stExposureTime.fCurValue,
                         stExposureTime.fMin, stExposureTime.fMax);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "曝光时间已设置: %.0fus", TARGET_EXPOSURE_TIME);
        }

        return true;
    }

    /**
     * @brief 取流循环，在独立线程中运行
     * @details 不断从相机获取帧数据，转换为 BGR8 格式，
     *          封装为 ROS2 Image 消息并发布。
     */
    void grabLoop()
    {
        int nRet = MV_OK;

        /* 分配原始数据缓冲区 */
        std::vector<unsigned char> raw_buffer(payload_size_);

        /* 分配 BGR 转换缓冲区 (CPU 路径使用) */
        std::vector<unsigned char> bgr_buffer;

        MV_FRAME_OUT_INFO_EX stImageInfo;
        bool bgr_buffer_allocated = false;

        RCLCPP_INFO(this->get_logger(), "取流线程已启动");

        while (is_running_ && rclcpp::ok())
        {
            memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));

            /* 获取一帧图像，超时 100ms (60fps -> ~16.7ms/帧，留足余量) */
            nRet = MV_CC_GetOneFrameTimeout(handle_, raw_buffer.data(),
                                            payload_size_, &stImageInfo, 100);
            if (MV_OK != nRet)
            {
                /* 超时是正常现象，不打印错误以避免刷屏 */
                continue;
            }

            /* 首次获取到帧后分配 BGR 缓冲区 */
            if (!bgr_buffer_allocated)
            {
                bgr_buffer.resize(stImageInfo.nWidth * stImageInfo.nHeight * 3);
                bgr_buffer_allocated = true;
                RCLCPP_INFO(this->get_logger(), "图像尺寸: %dx%d, 像素格式: 0x%x",
                            stImageInfo.nWidth, stImageInfo.nHeight,
                            stImageInfo.enPixelType);
            }

            /* 将原始图像转换为 BGR8 (有 CUDA 时优先用 GPU) */
            cv::Mat bgr_image;
            if (!convertToBGR(raw_buffer.data(), stImageInfo, bgr_buffer, bgr_image))
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                     "图像格式转换失败");
                continue;
            }

            /* 如果图像尺寸不是目标分辨率，则缩放到 TARGET_WIDTH x TARGET_HEIGHT */
            if (static_cast<unsigned int>(bgr_image.cols) != TARGET_WIDTH ||
                static_cast<unsigned int>(bgr_image.rows) != TARGET_HEIGHT)
            {
                resizeImage(bgr_image);
            }

            /* 构建 ROS2 Image 消息并发布 */
            auto msg = cv_bridge::CvImage(
                           std_msgs::msg::Header(), "bgr8", bgr_image)
                           .toImageMsg();
            msg->header.stamp = this->now();
            msg->header.frame_id = "camera_optical_frame";
            image_pub_->publish(*msg);

            /* 同步发布 CameraInfo（与图像使用相同的 header） */
            cam_info_msg_.header.stamp = msg->header.stamp;
            cam_info_pub_->publish(cam_info_msg_);
        }

        RCLCPP_INFO(this->get_logger(), "取流线程已退出");
    }

    /**
     * @brief 将全画幅 BGR 图像缩放到目标分辨率
     * @details 将传感器全分辨率采集的图像缩放到 TARGET_WIDTH x TARGET_HEIGHT。
     *          若 CUDA 可用，使用 GPU 加速缩放；否则使用 CPU cv::resize。
     *          使用 INTER_LINEAR 插值以平衡速度和质量。
     * @param[in,out] bgr_image 输入全分辨率 BGR 图像，输出缩放后的图像
     */
    void resizeImage(cv::Mat &bgr_image)
    {
#ifdef USE_CUDA
        if (use_cuda_)
        {
            cv::cuda::GpuMat gpu_full, gpu_resized;
            gpu_full.upload(bgr_image, cuda_stream_);
            cv::cuda::resize(gpu_full, gpu_resized,
                             cv::Size(TARGET_WIDTH, TARGET_HEIGHT),
                             0, 0, cv::INTER_LINEAR, cuda_stream_);
            gpu_resized.download(bgr_image, cuda_stream_);
            cuda_stream_.waitForCompletion();
            return;
        }
#endif
        cv::Mat resized;
        cv::resize(bgr_image, resized,
                   cv::Size(TARGET_WIDTH, TARGET_HEIGHT),
                   0, 0, cv::INTER_LINEAR);
        bgr_image = resized;
    }

    /**
     * @brief 将相机原始图像数据转换为 BGR8 格式的 cv::Mat
     * @details 若 USE_CUDA 已启用且运行时检测到 GPU，优先使用 CUDA 加速。
     *          支持的 GPU 加速格式：RGB8, Mono8, BayerGR8/RG8/GB8/BG8。
     *          其余格式或无 CUDA 时回退到 CPU。
     * @param[in]  pData       原始图像数据指针
     * @param[in]  stImageInfo 图像信息结构体
     * @param[out] bgr_buffer  BGR 转换缓冲区 (CPU 路径使用)
     * @param[out] bgr_image   输出的 BGR cv::Mat 图像
     * @return true 转换成功
     * @return false 转换失败
     */
    bool convertToBGR(unsigned char *pData,
                      const MV_FRAME_OUT_INFO_EX &stImageInfo,
                      std::vector<unsigned char> &bgr_buffer,
                      cv::Mat &bgr_image)
    {
#ifdef USE_CUDA
        unsigned int nWidth = stImageInfo.nWidth;
        unsigned int nHeight = stImageInfo.nHeight;
        MvGvspPixelType enPixelType = stImageInfo.enPixelType;

        /* ========== GPU 加速路径 ========== */
        if (use_cuda_)
        {
            /* BGR8: 已经是目标格式，直接使用 (无需 GPU) */
            if (enPixelType == PixelType_Gvsp_BGR8_Packed)
            {
                bgr_image = cv::Mat(nHeight, nWidth, CV_8UC3, pData).clone();
                return true;
            }

            /* RGB8 -> BGR8: GPU 通道交换 */
            if (enPixelType == PixelType_Gvsp_RGB8_Packed)
            {
                cv::Mat cpu_rgb(nHeight, nWidth, CV_8UC3, pData);
                gpu_src_.upload(cpu_rgb, cuda_stream_);
                cv::cuda::cvtColor(gpu_src_, gpu_dst_, cv::COLOR_RGB2BGR, 0, cuda_stream_);
                gpu_dst_.download(bgr_image, cuda_stream_);
                cuda_stream_.waitForCompletion();
                return true;
            }

            /* Mono8 -> BGR8: GPU 灰度转三通道 */
            if (enPixelType == PixelType_Gvsp_Mono8)
            {
                cv::Mat cpu_mono(nHeight, nWidth, CV_8UC1, pData);
                gpu_src_.upload(cpu_mono, cuda_stream_);
                cv::cuda::cvtColor(gpu_src_, gpu_dst_, cv::COLOR_GRAY2BGR, 0, cuda_stream_);
                gpu_dst_.download(bgr_image, cuda_stream_);
                cuda_stream_.waitForCompletion();
                return true;
            }

            /* Bayer 格式 -> BGR8: GPU 解马赛克 */
            int bayer_code = getBayerConversionCode(enPixelType);
            if (bayer_code >= 0)
            {
                cv::Mat cpu_bayer(nHeight, nWidth, CV_8UC1, pData);
                gpu_src_.upload(cpu_bayer, cuda_stream_);
                cv::cuda::demosaicing(gpu_src_, gpu_dst_, bayer_code, -1, cuda_stream_);
                gpu_dst_.download(bgr_image, cuda_stream_);
                cuda_stream_.waitForCompletion();
                return true;
            }

            /* GPU 不支持的格式，回退到 CPU */
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
                                 "像素格式 0x%x 不支持 GPU 加速，回退到 CPU", enPixelType);
        }
#endif

        /* ========== CPU 路径 ========== */
        return convertToBGR_CPU(pData, stImageInfo, bgr_buffer, bgr_image);
    }

#ifdef USE_CUDA
    /**
     * @brief 获取 Bayer 格式对应的 OpenCV 转换代码
     * @param[in] enPixelType MVS SDK 像素类型
     * @return 对应的 cv::ColorConversionCodes，不支持时返回 -1
     */
    int getBayerConversionCode(MvGvspPixelType enPixelType)
    {
        switch (enPixelType)
        {
        case PixelType_Gvsp_BayerGR8:
            return cv::COLOR_BayerGR2BGR;
        case PixelType_Gvsp_BayerRG8:
            return cv::COLOR_BayerRG2BGR;
        case PixelType_Gvsp_BayerGB8:
            return cv::COLOR_BayerGB2BGR;
        case PixelType_Gvsp_BayerBG8:
            return cv::COLOR_BayerBG2BGR;
        default:
            return -1;
        }
    }
#endif

    /**
     * @brief CPU 路径：将相机原始图像数据转换为 BGR8
     * @param[in]  pData       原始图像数据指针
     * @param[in]  stImageInfo 图像信息结构体
     * @param[out] bgr_buffer  BGR 转换缓冲区
     * @param[out] bgr_image   输出的 BGR cv::Mat 图像
     * @return true 转换成功
     * @return false 转换失败
     */
    bool convertToBGR_CPU(unsigned char *pData,
                          const MV_FRAME_OUT_INFO_EX &stImageInfo,
                          std::vector<unsigned char> &bgr_buffer,
                          cv::Mat &bgr_image)
    {
        unsigned int nWidth = stImageInfo.nWidth;
        unsigned int nHeight = stImageInfo.nHeight;
        MvGvspPixelType enPixelType = stImageInfo.enPixelType;

        /* 如果已经是 BGR8，直接使用 */
        if (enPixelType == PixelType_Gvsp_BGR8_Packed)
        {
            bgr_image = cv::Mat(nHeight, nWidth, CV_8UC3, pData).clone();
            return true;
        }

        /* 如果是 RGB8，转换通道顺序 */
        if (enPixelType == PixelType_Gvsp_RGB8_Packed)
        {
            cv::Mat rgb_image(nHeight, nWidth, CV_8UC3, pData);
            cv::cvtColor(rgb_image, bgr_image, cv::COLOR_RGB2BGR);
            return true;
        }

        /* 如果是 Mono8 灰度图，转换为三通道 */
        if (enPixelType == PixelType_Gvsp_Mono8)
        {
            cv::Mat mono_image(nHeight, nWidth, CV_8UC1, pData);
            cv::cvtColor(mono_image, bgr_image, cv::COLOR_GRAY2BGR);
            return true;
        }

        /* YUV422_Packed (YUYV) -> BGR8: 使用 OpenCV 转换 */
        if (enPixelType == PixelType_Gvsp_YUV422_Packed)
        {
            cv::Mat yuv_image(nHeight, nWidth, CV_8UC2, pData);
            cv::cvtColor(yuv_image, bgr_image, cv::COLOR_YUV2BGR_YUYV);
            return true;
        }

        /* YUV422_YUYV_Packed -> BGR8 */
        if (enPixelType == PixelType_Gvsp_YUV422_YUYV_Packed)
        {
            cv::Mat yuv_image(nHeight, nWidth, CV_8UC2, pData);
            cv::cvtColor(yuv_image, bgr_image, cv::COLOR_YUV2BGR_YUYV);
            return true;
        }

        /* 其他格式 (Bayer, YUV 等)，使用 SDK 进行转换 */
        MV_CC_PIXEL_CONVERT_PARAM stConvertParam;
        memset(&stConvertParam, 0, sizeof(MV_CC_PIXEL_CONVERT_PARAM));
        stConvertParam.nWidth = nWidth;
        stConvertParam.nHeight = nHeight;
        stConvertParam.pSrcData = pData;
        stConvertParam.nSrcDataLen = stImageInfo.nFrameLen;
        stConvertParam.enSrcPixelType = enPixelType;
        stConvertParam.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
        stConvertParam.pDstBuffer = bgr_buffer.data();
        stConvertParam.nDstBufferSize = bgr_buffer.size();

        int nRet = MV_CC_ConvertPixelType(handle_, &stConvertParam);
        if (MV_OK != nRet)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "像素格式转换失败! nRet [0x%x], 像素类型: 0x%x",
                                 nRet, enPixelType);
            return false;
        }

        bgr_image = cv::Mat(nHeight, nWidth, CV_8UC3, bgr_buffer.data()).clone();
        return true;
    }

    /**
     * @brief 清理相机资源
     * @details 按顺序执行：停止取流 -> 关闭设备 -> 销毁句柄
     */
    void cleanupCamera()
    {
        if (handle_ == nullptr)
        {
            return;
        }

        int nRet = MV_OK;

        /* 停止取流 */
        nRet = MV_CC_StopGrabbing(handle_);
        if (MV_OK != nRet)
        {
            RCLCPP_WARN(this->get_logger(), "停止取流失败! nRet [0x%x]", nRet);
        }

        /* 关闭设备 */
        nRet = MV_CC_CloseDevice(handle_);
        if (MV_OK != nRet)
        {
            RCLCPP_WARN(this->get_logger(), "关闭设备失败! nRet [0x%x]", nRet);
        }

        /* 销毁句柄 */
        nRet = MV_CC_DestroyHandle(handle_);
        if (MV_OK != nRet)
        {
            RCLCPP_WARN(this->get_logger(), "销毁句柄失败! nRet [0x%x]", nRet);
        }

        handle_ = nullptr;
        RCLCPP_INFO(this->get_logger(), "相机资源已释放");
    }

    /* ========== 成员变量 ========== */

    /** @brief 图像发布者 */
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    /** @brief CameraInfo 发布者 */
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_pub_;
    /** @brief CameraInfo 消息（预构建，每帧仅更新时间戳） */
    sensor_msgs::msg::CameraInfo cam_info_msg_;
    /** @brief 取流线程 */
    std::thread grab_thread_;
    /** @brief 线程运行标志 */
    std::atomic<bool> is_running_;
    /** @brief 相机设备句柄 */
    void *handle_;
    /** @brief 帧数据负载大小 */
    unsigned int payload_size_;
    /** @brief 是否使用 CUDA GPU 加速 */
    bool use_cuda_;

#ifdef USE_CUDA
    /* ========== CUDA GPU 加速相关 ========== */

    /** @brief GPU 源图像缓冲 */
    cv::cuda::GpuMat gpu_src_;
    /** @brief GPU 目标图像缓冲 */
    cv::cuda::GpuMat gpu_dst_;
    /** @brief CUDA 异步流，用于重叠 Upload/Compute/Download */
    cv::cuda::Stream cuda_stream_;
#endif
};

/**
 * @brief 主函数
 */
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HkCameraNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
