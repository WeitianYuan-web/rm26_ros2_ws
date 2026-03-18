#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <auto_aim_interfaces/msg/armors.hpp>
#include <auto_aim_interfaces/msg/armor.hpp>

#include <opencv2/opencv.hpp>
#include <NvInfer.h>
#include <cuda_runtime_api.h>

#include <memory>
#include <vector>
#include <string>
#include <fstream>
#include <iostream>

using std::placeholders::_1;
using namespace nvinfer1;

/**
 * @brief TensorRT Logger
 */
class TRTLogger : public ILogger {
public:
    void log(Severity severity, const char* msg) noexcept override {
        if (severity <= Severity::kWARNING) {
            RCLCPP_WARN(rclcpp::get_logger("TRTLogger"), "%s", msg);
        }
    }
} gLogger;

/**
 * @brief CUDA 错误检查宏
 */
#define checkCudaErrors(status)                                   \
{                                                                 \
    if (status != 0)                                              \
    {                                                             \
        std::cout << "CUDA failure: " << cudaGetErrorString(status) \
                  << " at line " << __LINE__ << std::endl;        \
        abort();                                                  \
    }                                                             \
}

/**
 * @brief 装甲板检测节点
 */
class ArmorDetectorNode : public rclcpp::Node {
public:
    ArmorDetectorNode() : Node("armor_detector_node"), engine_loaded_(false) {
        // 参数声明
        this->declare_parameter<std::string>("engine_path", "src/rm26_auto_aim/model/yolo26n_rm_500.engine");
        this->declare_parameter<double>("conf_threshold", 0.5);
        this->declare_parameter<double>("armor_real_width", 0.135); // 默认小装甲板宽度 135mm
        this->declare_parameter<bool>("show_image", true);          // 是否发布可视化图像
        this->declare_parameter<double>("aim_offset_x_px", 0.0);
        this->declare_parameter<double>("aim_offset_y_px", 0.0);

        std::string engine_path = this->get_parameter("engine_path").as_string();
        conf_threshold_ = this->get_parameter("conf_threshold").as_double();
        armor_real_width_ = this->get_parameter("armor_real_width").as_double();
        show_image_ = this->get_parameter("show_image").as_bool();
        aim_offset_x_px_ = this->get_parameter("aim_offset_x_px").as_double();
        aim_offset_y_px_ = this->get_parameter("aim_offset_y_px").as_double();

        // 加载 TensorRT Engine
        if (!loadEngine(engine_path)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load TensorRT engine: %s", engine_path.c_str());
            return;
        }

        // QoS 配置为 SensorData (Best Effort)，与相机节点匹配
        auto qos = rclcpp::SensorDataQoS();

        // 订阅者
        cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "camera_info", qos, std::bind(&ArmorDetectorNode::camInfoCallback, this, _1));

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "image_raw", qos, std::bind(&ArmorDetectorNode::imageCallback, this, _1));

        // 发布者
        armors_pub_ = this->create_publisher<auto_aim_interfaces::msg::Armors>("detector/armors", 10);
        if (show_image_) {
            result_img_pub_ = this->create_publisher<sensor_msgs::msg::Image>("detector/result_img", 10);
        }

        RCLCPP_INFO(this->get_logger(), "ArmorDetectorNode started.");
    }

    ~ArmorDetectorNode() {
        if (context_) context_->destroy();
        if (engine_) engine_->destroy();
        if (runtime_) runtime_->destroy();
        cudaFree(buffers_[0]);
        cudaFree(buffers_[1]);
    }

private:
    bool loadEngine(const std::string& path) {
        std::ifstream file(path, std::ios::binary);
        if (!file.good()) return false;

        file.seekg(0, file.end);
        size_t size = file.tellg();
        file.seekg(0, file.beg);
        std::vector<char> trtModelStream(size);
        file.read(trtModelStream.data(), size);
        file.close();

        // 处理 Ultralytics 添加的 JSON 头部
        size_t offset = 0;
        if (trtModelStream[0] == '/' || trtModelStream[4] == '{') {
            uint32_t meta_len;
            memcpy(&meta_len, trtModelStream.data(), sizeof(uint32_t));
            offset = 4 + meta_len;
            RCLCPP_INFO(this->get_logger(), "Skipped Ultralytics metadata header (size: %u)", meta_len);
        }

        runtime_ = createInferRuntime(gLogger);
        engine_ = runtime_->deserializeCudaEngine(trtModelStream.data() + offset, size - offset);
        if (!engine_) return false;

        context_ = engine_->createExecutionContext();
        if (!context_) return false;

        // 获取输入输出形状
        auto input_dims = engine_->getBindingDimensions(0);
        input_w_ = input_dims.d[3];
        input_h_ = input_dims.d[2];
        input_size_ = input_w_ * input_h_ * 3 * sizeof(float);

        auto output_dims = engine_->getBindingDimensions(1);
        num_preds_ = output_dims.d[1];
        num_attrs_ = output_dims.d[2];
        output_size_ = num_preds_ * num_attrs_ * sizeof(float);

        // 分配 GPU 内存
        checkCudaErrors(cudaMalloc(&buffers_[0], input_size_));
        checkCudaErrors(cudaMalloc(&buffers_[1], output_size_));

        // 分配 CPU 内存用于拷贝输出
        output_data_.resize(num_preds_ * num_attrs_);

        engine_loaded_ = true;
        return true;
    }

    void camInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        camera_info_ = msg;
    }

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        if (!engine_loaded_) return;

        // 获取图像
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat img = cv_ptr->image;
        int orig_w = img.cols;
        int orig_h = img.rows;

        // 1. 预处理 (CPU)
        cv::Mat resized_img;
        cv::resize(img, resized_img, cv::Size(input_w_, input_h_));
        
        cv::Mat float_img;
        resized_img.convertTo(float_img, CV_32FC3, 1.0f / 255.0f);

        // HWC to CHW
        std::vector<cv::Mat> chw_channels(3);
        cv::split(float_img, chw_channels);
        
        std::vector<float> input_data(input_w_ * input_h_ * 3);
        int channel_size = input_w_ * input_h_;
        memcpy(input_data.data() + 0 * channel_size, chw_channels[2].data, channel_size * sizeof(float)); // R
        memcpy(input_data.data() + 1 * channel_size, chw_channels[1].data, channel_size * sizeof(float)); // G
        memcpy(input_data.data() + 2 * channel_size, chw_channels[0].data, channel_size * sizeof(float)); // B

        // 2. 推理 (GPU)
        checkCudaErrors(cudaMemcpyAsync(buffers_[0], input_data.data(), input_size_, cudaMemcpyHostToDevice, 0));
        context_->enqueueV2(buffers_, 0, nullptr);
        checkCudaErrors(cudaMemcpyAsync(output_data_.data(), buffers_[1], output_size_, cudaMemcpyDeviceToHost, 0));
        cudaStreamSynchronize(0);

        // 3. 后处理
        auto_aim_interfaces::msg::Armors armors_msg;
        armors_msg.header = msg->header;

        float scale_w = static_cast<float>(orig_w) / input_w_;
        float scale_h = static_cast<float>(orig_h) / input_h_;

        // Camera intrinsics
        double fx = 1500.0, fy = 1500.0, cx = orig_w / 2.0, cy = orig_h / 2.0;
        if (camera_info_) {
            fx = camera_info_->k[0];
            cx = camera_info_->k[2];
            fy = camera_info_->k[4];
            cy = camera_info_->k[5];
        }

        // YOLOv8 output: [num_preds, num_attrs] -> attr: [cx, cy, w, h, conf, cls] or [x1, y1, x2, y2, conf, cls]
        // From test script: shape (1, 300, 6), valid = dets[dets[:, 4] > conf]
        // Usually index 0,1,2,3 are x1, y1, x2, y2. Let's assume xyxy format based on python script.
        
        for (int i = 0; i < num_preds_; ++i) {
            float* det = output_data_.data() + i * num_attrs_;
            float conf = det[4];
            if (conf > conf_threshold_) {
                float x1 = det[0] * scale_w;
                float y1 = det[1] * scale_h;
                float x2 = det[2] * scale_w;
                float y2 = det[3] * scale_h;
                int cls = static_cast<int>(det[5]);

                // 类别映射: 0='lan'(蓝), 1='hong'(红)
                std::string color_str = (cls == 0) ? "blue" : "red";

                float bbox_cx = (x1 + x2) / 2.0f;
                float bbox_cy = (y1 + y2) / 2.0f;
                float bbox_w = std::max(1.0f, x2 - x1);
                float aim_cx = bbox_cx + static_cast<float>(aim_offset_x_px_);
                float aim_cy = bbox_cy + static_cast<float>(aim_offset_y_px_);
                
                // 计算相机坐标系下的 3D 位姿 (Pinhole模型简单估算)
                double z = (fx * armor_real_width_) / bbox_w;
                double x = (aim_cx - cx) * z / fx;
                double y = (aim_cy - cy) * z / fy;

                auto_aim_interfaces::msg::Armor armor;
                armor.type = color_str;
                armor.number = "1"; // 默认数字
                armor.distance_to_image_center = std::sqrt(std::pow(aim_cx - cx, 2) + std::pow(aim_cy - cy, 2));
                
                armor.pose.position.x = x;
                armor.pose.position.y = y;
                armor.pose.position.z = z;
                // 暂时不估计旋转，使用单位四元数
                armor.pose.orientation.w = 1.0;

                armors_msg.armors.push_back(armor);

                // 可视化绘制
                if (show_image_) {
                    cv::Rect box(x1, y1, x2 - x1, y2 - y1);
                    cv::Scalar color = (cls == 0) ? cv::Scalar(255, 0, 0) : cv::Scalar(0, 0, 255);
                    cv::rectangle(img, box, color, 2);
                    std::string label = color_str + " " + std::to_string(conf).substr(0, 4);
                    cv::putText(img, label, cv::Point(x1, y1 - 5), cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 2);
                    // 画中心点
                    cv::circle(img, cv::Point(bbox_cx, bbox_cy), 3, cv::Scalar(0, 255, 0), -1);
                    cv::circle(img, cv::Point(aim_cx, aim_cy), 3, cv::Scalar(0, 255, 255), -1);
                }
            }
        }

        armors_pub_->publish(armors_msg);

        if (show_image_) {
            sensor_msgs::msg::Image::SharedPtr result_msg = 
                cv_bridge::CvImage(msg->header, "bgr8", img).toImageMsg();
            result_img_pub_->publish(*result_msg);
        }
    }

    IRuntime* runtime_ = nullptr;
    ICudaEngine* engine_ = nullptr;
    IExecutionContext* context_ = nullptr;
    void* buffers_[2];
    std::vector<float> output_data_;
    bool engine_loaded_;

    int input_w_, input_h_, input_size_;
    int num_preds_, num_attrs_, output_size_;
    double conf_threshold_;
    double armor_real_width_;
    bool show_image_;
    double aim_offset_x_px_{0.0};
    double aim_offset_y_px_{0.0};

    sensor_msgs::msg::CameraInfo::SharedPtr camera_info_;

    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<auto_aim_interfaces::msg::Armors>::SharedPtr armors_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr result_img_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmorDetectorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
