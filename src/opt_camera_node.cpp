

// optsdk include
#include <OPTApi.h>

// ros2 include
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

// c++ system
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <memory>
#include <thread>
#include <vector>

#define ImageMaxHeight 1200
#define ImageMaxWidth 1920
namespace opt_camera
{
  class OPTCameraNode : public rclcpp::Node
  {
  public:
    explicit OPTCameraNode(const rclcpp::NodeOptions &options) : Node("opt_camera", options)
    {
      /*****************************************/
      /***********CAMERA******INIT**************/
      /*****************************************/
      // 发现设备
      // discover camera
      ret = OPT_EnumDevices(&deviceInfoList, interfaceTypeAll);

      if (OPT_OK != ret)
      {
        RCLCPP_ERROR(this->get_logger(), "Enumeration devices failed! ErrorCode[%d]\n", ret);
        return;
      }

      while (deviceInfoList.nDevNum < 1 && rclcpp::ok())
      {
        RCLCPP_ERROR(this->get_logger(), "no camera\n");
        std::this_thread::sleep_for(std::chrono::seconds(1));
        ret = OPT_EnumDevices(&deviceInfoList, interfaceTypeAll);
        if (OPT_OK != ret)
        {
          RCLCPP_ERROR(this->get_logger(), "Enumeration devices failed! ErrorCode[%d]\n", ret);
          return;
        }
      }
      RCLCPP_INFO(this->get_logger(), "detect camera!!!\n");
      // TODO:Now default choose camera Index 1,future need to change.
      //  选择需要连接的相机
      // Select one camera to connect to
      cameraIndex = 0;
      // 创建设备句柄
      // Create Device Handle
      ret = OPT_CreateHandle(&devHandle, modeByIndex, (void *)&cameraIndex);
      if (OPT_OK != ret)
      {
        RCLCPP_ERROR(this->get_logger(), "Create devHandle failed! ErrorCode[%d]\n", ret);
        return;
      }

      // 打开相机
      // Open camera
      ret = OPT_Open(devHandle);
      if (OPT_OK != ret)
      {
        RCLCPP_ERROR(this->get_logger(), "Open camera failed! ErrorCode[%d]\n", ret);
        return;
      }

      // Create camera publisher
      // rqt_image_view can't subscribe image msg with sensor_data QoS
      // https://github.com/ros-visualization/rqt/issues/187
      bool use_sensor_data_qos = this->declare_parameter("use_sensor_data_qos", false);
      auto qos = use_sensor_data_qos ? rmw_qos_profile_sensor_data : rmw_qos_profile_default;
      camera_pub_ = image_transport::create_camera_publisher(this, "image_raw", qos);
      declareParameters();
      ret = OPT_StartGrabbing(devHandle);
      if (OPT_OK != ret)
      {
        RCLCPP_INFO(this->get_logger(), "Start grabbing failed! ErrorCode[%d]\n", ret);
        return;
      }
      // Load camera info
      camera_name_ = this->declare_parameter("camera_name", "opt_camera");
      camera_info_manager_ =
          std::make_unique<camera_info_manager::CameraInfoManager>(this, camera_name_);
      auto camera_info_url =
          this->declare_parameter("camera_info_url", "package://opt_camera/config/camera_info.yaml");
      if (camera_info_manager_->validateURL(camera_info_url))
      {
        camera_info_manager_->loadCameraInfo(camera_info_url);
        camera_info_msg_ = camera_info_manager_->getCameraInfo();
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "Invalid camera info URL: %s", camera_info_url.c_str());
      }
      params_callback_handle_ = this->add_on_set_parameters_callback(
          std::bind(&OPTCameraNode::parametersCallback, this, std::placeholders::_1));

      capture_thread_ = std::thread{
          [this]() -> void
          {
            // the son thread :capture and publish image
            //  开始拉流
            //  Start grabbing
            OPT_Frame frame;
            RCLCPP_INFO(this->get_logger(), "Publishing image!");

            image_msg_.header.frame_id = "camera_optical_frame";
            image_msg_.encoding = "rgb8";
            image_msg_.data.reserve(ImageMaxHeight * ImageMaxWidth * 3);
            while (rclcpp::ok())
            {
              // 获取一帧图像
              // Get a frame image
              ret = OPT_GetFrame(devHandle, &frame, 200);
              if (OPT_OK == ret)
              {
                fail_count_ = 0;
                /*********************************************************************/
                /*tranfer image format from OPT_Frame to ros2 sensor_msgs::msg::Image*/
                /*********************************************************************/
                // TODO:question??:Is gvspPixelRGB8 format equal to RGB8 format?
                pConvertFormatStr = (const char *)"RGB8";
                pDstBuf = image_msg_.data.data();
                // TODO:error in the following code.
                // nDstBufSize = (unsigned int)image_msg_.data.size();
                nDstBufSize = sizeof(unsigned char) * frame.frameInfo.width * frame.frameInfo.height * 3;

                memset(&stPixelConvertParam, 0, sizeof(stPixelConvertParam));
                stPixelConvertParam.nWidth = frame.frameInfo.width;
                stPixelConvertParam.nHeight = frame.frameInfo.height;
                stPixelConvertParam.ePixelFormat = frame.frameInfo.pixelFormat;
                stPixelConvertParam.pSrcData = frame.pData;
                stPixelConvertParam.nSrcDataLen = frame.frameInfo.size;
                stPixelConvertParam.nPaddingX = frame.frameInfo.paddingX;
                stPixelConvertParam.nPaddingY = frame.frameInfo.paddingY;
                stPixelConvertParam.eBayerDemosaic = demosaicNearestNeighbor;
                stPixelConvertParam.eDstPixelFormat = convertFormat;
                stPixelConvertParam.pDstBuf = pDstBuf;
                stPixelConvertParam.nDstBufSize = nDstBufSize;

                ret = OPT_PixelConvert(devHandle, &stPixelConvertParam);
                if (OPT_OK != ret)
                {
                  RCLCPP_ERROR(this->get_logger(), "image convert to %s failed! ErrorCode[%d]\n", pConvertFormatStr, ret);
                  return;
                }
                image_msg_.header.stamp = this->now();
                image_msg_.height = frame.frameInfo.height;
                image_msg_.width = frame.frameInfo.width;
                image_msg_.step = frame.frameInfo.width * 3;
                image_msg_.data.resize(image_msg_.width * image_msg_.height * 3);
                camera_info_msg_.header = image_msg_.header;
                camera_pub_.publish(image_msg_, camera_info_msg_);
                // 释放图像缓存
                // Free image buffer
                ret = OPT_ReleaseFrame(devHandle, &frame);
                if (OPT_OK != ret)
                {
                  RCLCPP_ERROR(this->get_logger(), "Release frame failed! ErrorCode[%d]\n", ret);
                }
              }
              else
              {
                RCLCPP_INFO(this->get_logger(), "Get frame failed! ErrorCode[%d]\n", ret);
                fail_count_++;
              }
              if (fail_count_ > 5)
              {
                RCLCPP_FATAL(this->get_logger(), "Camera failed!");
                rclcpp::shutdown();
              }
            }
          }};
    }

    ~OPTCameraNode() override
    {
      if (capture_thread_.joinable())
      {
        capture_thread_.join();
      }
      // 关闭相机
      // Close camera
      ret = OPT_Close(devHandle);
      if (OPT_OK != ret)
      {
        RCLCPP_ERROR(this->get_logger(), "Close camera failed! ErrorCode[%d]\n", ret);
        return;
      }
      if (devHandle != NULL)
      {
        // 销毁设备句柄
        // Destroy Device Handle
        OPT_DestroyHandle(devHandle);
      }
      RCLCPP_INFO(this->get_logger(), "Camera node destroyed!");
    }

  private:
    void declareParameters()
    {
      double double_minvalue = 0.0;
      double double_maxvalue = 0.0;
      int64_t int64_minvalue = 0;
      int64_t int64_maxvalue = 0;
      rcl_interfaces::msg::ParameterDescriptor param_desc;
      param_desc.integer_range.resize(1);
      param_desc.integer_range[0].step = 1;
      // Exposure time  :max:1000000.0  min:1.0
      double exposureTimeValue = 0;
      param_desc.description = "Exposure time in microseconds";
      ret = OPT_GetDoubleFeatureValue(devHandle, "ExposureTime", &exposureTimeValue);
      ret = OPT_GetDoubleFeatureMin(devHandle, "ExposureTime", &double_minvalue);
      ret = OPT_GetDoubleFeatureMax(devHandle, "ExposureTime", &double_maxvalue);
      param_desc.integer_range[0].from_value = double_minvalue;
      param_desc.integer_range[0].to_value = double_maxvalue;
      exposureTimeValue = this->declare_parameter("exposure_time", exposureTimeValue, param_desc);
      ret = OPT_SetDoubleFeatureValue(devHandle, "ExposureTime", exposureTimeValue);
      if (OPT_OK != ret)
      {
        RCLCPP_ERROR(this->get_logger(), "set exposure time error");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Exposure time: %f", exposureTimeValue);
      // Gain  min:1.0 max:32.0
      double gainValue = 0.0;
      param_desc.description = "Gain";
      ret = OPT_GetDoubleFeatureValue(devHandle, "GainRaw", &gainValue);
      ret = OPT_GetDoubleFeatureMin(devHandle, "GainRaw", &double_minvalue);
      ret = OPT_GetDoubleFeatureMax(devHandle, "GainRaw", &double_maxvalue);
      param_desc.integer_range[0].from_value = double_minvalue;
      param_desc.integer_range[0].to_value = double_maxvalue;
      gainValue = this->declare_parameter("gain", gainValue, param_desc);
      ret = OPT_SetDoubleFeatureValue(devHandle, "GainRaw", gainValue);
      if (OPT_OK != ret)
      {
        RCLCPP_ERROR(this->get_logger(), "set gain error");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Gain: %f", gainValue);
      // Gamma:
      double gammaValue = 0.0;
      param_desc.description = "Gamma";
      ret = OPT_GetDoubleFeatureValue(devHandle, "Gamma", &gammaValue);
      ret = OPT_GetDoubleFeatureMin(devHandle, "Gamma", &double_minvalue);
      ret = OPT_GetDoubleFeatureMax(devHandle, "Gamma", &double_maxvalue);
      param_desc.integer_range[0].from_value = double_minvalue;
      param_desc.integer_range[0].to_value = double_maxvalue;
      gammaValue = this->declare_parameter("gamma", gammaValue, param_desc);
      ret = OPT_SetDoubleFeatureValue(devHandle, "Gamma", gammaValue);
      if (OPT_OK != ret)
      {
        RCLCPP_ERROR(this->get_logger(), "set gamma error");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "gamma: %f", gammaValue);
      // width 128~1920
      int64_t widthValue = 1920;
      // param_desc.description = "Image Width";
      // ret = OPT_GetIntFeatureValue(devHandle, "Width", &widthValue);
      ret = OPT_GetIntFeatureMin(devHandle, "Width", &int64_minvalue);
      ret = OPT_GetIntFeatureMax(devHandle, "Width", &int64_maxvalue);
      // param_desc.integer_range[0].from_value = int64_minvalue;
      // param_desc.integer_range[0].to_value = int64_maxvalue;
      //widthValue = this->declare_parameter("width", widthValue, param_desc);
      ret = OPT_SetIntFeatureValue(devHandle, "Width", widthValue);
      if (OPT_OK != ret)
      {
        RCLCPP_ERROR(this->get_logger(), "set width error");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Width: %ld", widthValue);
      // Height 64~1200 note:步长
      int64_t heightValue = 1200;
      // param_desc.description = "Image Height";
      //ret = OPT_GetIntFeatureValue(devHandle, "Height", &heightValue);
      ret = OPT_GetIntFeatureMin(devHandle, "Height", &int64_minvalue);
      ret = OPT_GetIntFeatureMax(devHandle, "Height", &int64_maxvalue);
      // param_desc.integer_range[0].from_value = int64_minvalue;
      // param_desc.integer_range[0].to_value = int64_maxvalue;
      // heightValue = this->declare_parameter("height", heightValue, param_desc);
      ret = OPT_SetIntFeatureValue(devHandle, "Height", heightValue);
      if (OPT_OK != ret)
      {
        RCLCPP_ERROR(this->get_logger(), "set height error");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Height: %ld", heightValue);
      // ExposureAuto 0:Off 1:Once 2:Continuous
      int64_t autoValue = 0;
      param_desc.description = "sets the automatic exposure mode when ExposureMode isimed.The exact algorithm used to implement this control is device specific ";
      OPT_String enumSymbolValue;
      ret = OPT_GetEnumFeatureSymbol(devHandle, "ExposureAuto", &enumSymbolValue);
      if (enumSymbolValue.str[1] == 'f')
        autoValue = 0;
      else if (enumSymbolValue.str[1] == 'n')
        autoValue = 1;
      else
        autoValue = 2;
      param_desc.integer_range[0].from_value = 0;
      param_desc.integer_range[0].to_value = 2;
      autoValue = this->declare_parameter("exposureAuto", autoValue, param_desc);
      ret = OPT_SetEnumFeatureSymbol(devHandle, "ExposureAuto", enumSymbolValue.str);
      if (OPT_OK != ret)
      {
        RCLCPP_ERROR(this->get_logger(), "set exposureAuto error");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "exposureAuto: %s", enumSymbolValue.str);
    }

    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter> &parameters)
    {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      for (const auto &param : parameters)
      {
        if (param.get_name() == "exposure_time")
        {
          ret = OPT_SetDoubleFeatureValue(devHandle, "ExposureTime", param.as_double());

          if (OPT_OK != ret)
          {
            result.successful = false;
            RCLCPP_ERROR(this->get_logger(), "Failed to set exposureTime into %lf \n", param.as_double());
            result.reason = "Failed to set exposure time, error code = " + std::to_string(ret);
          }
          else
            RCLCPP_INFO(this->get_logger(), "Success to set exposureTime into %lf \n", param.as_double());
        }
        else if (param.get_name() == "gain")
        {
          ret = OPT_SetDoubleFeatureValue(devHandle, "GainRaw", param.as_double());
          if (OPT_OK != ret)
          {
            result.successful = false;
            RCLCPP_ERROR(this->get_logger(), "Failed to set gain into %lf \n", param.as_double());
            result.reason = "Failed to set gain, error code  = " + std::to_string(ret);
          }
          else
            RCLCPP_INFO(this->get_logger(), "Success to set gain into %lf \n", param.as_double());
        }
        else if (param.get_name() == "gamma")
        {
          ret = OPT_SetDoubleFeatureValue(devHandle, "gamma", param.as_double());
          if (OPT_OK != ret)
          {
            result.successful = false;
            RCLCPP_ERROR(this->get_logger(), "Failed to set Gamma into %lf \n", param.as_double());
            result.reason = "Failed to set gamma, error code  = " + std::to_string(ret);
          }
          else
            RCLCPP_INFO(this->get_logger(), "Success to set gamma into %lf \n", param.as_double());
        }
        // else if (param.get_name() == "width")
        // {
        //   ret = OPT_SetIntFeatureValue(devHandle, "Width", param.as_int());
        //   if (OPT_OK != ret)
        //   {
        //     result.successful = false;
        //     RCLCPP_ERROR(this->get_logger(), "Failed to set width into %ld \n", param.as_int());
        //     result.reason = "Failed to set width, error code  = " + std::to_string(ret);
        //   }
        //   else
        //     RCLCPP_INFO(this->get_logger(), "Success to set width into %ld \n", param.as_int());
        // }
        // else if (param.get_name() == "height")
        // {
        //   ret = OPT_SetIntFeatureValue(devHandle, "Height", param.as_int());
        //   if (OPT_OK != ret)
        //   {
        //     result.successful = false;
        //     RCLCPP_ERROR(this->get_logger(), "Failed to set height into %ld \n", param.as_int());
        //     result.reason = "Failed to set height, error code  = " + std::to_string(ret);
        //   }
        //   else
        //     RCLCPP_INFO(this->get_logger(), "Success to set height into %ld \n", param.as_int());
        // }
        else if (param.get_name() == "exposureAuto")
        {
          int autoNum = param.as_int();
          char auto_name[3][20] = {"Off", "Once", "Continuous"};
          ret = OPT_SetEnumFeatureSymbol(devHandle, "ExposureAuto", auto_name[autoNum]);
          if (OPT_OK != ret)
          {
            result.successful = false;
            RCLCPP_ERROR(this->get_logger(), "Failed to set exposureAuto into %s \n", auto_name[autoNum]);
            result.reason = "Failed to set exporesureAuto, error code  = " + std::to_string(ret);
          }
          else
            RCLCPP_INFO(this->get_logger(), "Success to set exposureAuto into %s \n", auto_name[autoNum]);
        }
        else
        {
          result.successful = false;
          result.reason = "Unknown parameter: " + param.get_name();
        }
      }
      return result;
    }

    int ret = OPT_OK;              // the flag to detect whether error about opt happen or not
    unsigned int cameraIndex = 0;  // the camera you choose to connect
    OPT_HANDLE devHandle = NULL;   // camera handle
    OPT_DeviceList deviceInfoList; // the device list you find .
    std::thread capture_thread_;   // the thread of capture and publish image
    int fail_count_ = 0;           // record the number of frame that you fail to capture continuely.
    // image convert variable
    OPT_PixelConvertParam stPixelConvertParam; // Image convert param
    unsigned char *pDstBuf = NULL;             // point to the image data with format rgb8
    unsigned int nDstBufSize = 0;
    const char *pConvertFormatStr = NULL;
    OPT_EPixelType convertFormat = gvspPixelRGB8;
    // ros2 variable
    image_transport::CameraPublisher camera_pub_;
    OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;
    std::string camera_name_;
    std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
    sensor_msgs::msg::CameraInfo camera_info_msg_;
    sensor_msgs::msg::Image image_msg_; // ros2 standard image msg ,will be published in son thread
    // camera property
  };

} // namespace opt_camera

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(opt_camera::OPTCameraNode)
