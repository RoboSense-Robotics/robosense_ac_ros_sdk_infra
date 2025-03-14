#ifndef SENSORMANAGER_H
#define SENSORMANAGER_H

// #include "hyper_vision/common/common.h"
#include "libusb/libusb.h"
#include "rs_driver/api/lidar_driver.hpp"
#include "rs_driver/msg/pcl_point_cloud_msg.hpp"
#include <functional>
#include <queue>
#include <set>
#include <string>

// 定义点云类型
using RsPointXYZIRT = PointXYZIRT;

namespace robosense {
namespace device {

typedef enum DeviceEventType {
  DEVICE_EVENT_DETACH = 0,
  DEVICE_EVENT_ATTACH,

  DEVICE_EVENT_UNKNOWN = 255,
} DeviceEventType_t;

typedef struct DeviceEvent {
  DeviceEventType_t event_type;
  uint32_t uuid_size;
  char uuid[128];
} DeviceEvent_t;

class DeviceManager : public std::enable_shared_from_this<DeviceManager> {
public:
  using Ptr = std::shared_ptr<DeviceManager>;
  using ConstPtr = std::shared_ptr<const DeviceManager>;

public:
  using RS_DEVICE_EVENT_CALLBACK =
      std::function<void(const DeviceEvent &deviceEvent)>;

  using RS_DEVICE_POINTCLOUD_CALLBACK =
      std::function<void(const std::shared_ptr<PointCloudT<RsPointXYZIRT>> &,
                         const std::string &)>;

  using RS_DEVICE_IMAGE_CALLBACK =
      std::function<void(const std::shared_ptr<robosense::lidar::ImageData> &,
                         const std::string &)>;

  using RS_DEVICE_IMU_CALLBACK = std::function<void(
      const std::shared_ptr<robosense::lidar::ImuData> &, const std::string &)>;

private:
  using RS_DEVICE_DRIVER =
      robosense::lidar::LidarDriver<PointCloudT<RsPointXYZIRT>>;
  using RS_DEVICE_DRIVER_PTR = std::shared_ptr<RS_DEVICE_DRIVER>;

public:
  class DeviceInfoItem {
  public:
    using Ptr = std::shared_ptr<DeviceInfoItem>;
    using ConstPtr = std::shared_ptr<const DeviceInfoItem>;

  public:
    DeviceInfoItem() {
      uuid.clear();
      is_attach = false;
      is_usb_mode = false;
      i_sn = 0;
      driver_ptr = nullptr;
      is_pause = true;
    }

    ~DeviceInfoItem() = default;

  public:
    std::string uuid;
    bool is_attach;
    bool is_usb_mode;
    uint8_t i_sn;
    RS_DEVICE_DRIVER_PTR driver_ptr;
    bool is_pause;
  };

public:
  DeviceManager();

  ~DeviceManager();

  bool init(const bool isEnableDebug = false);

  int stop();

  int openDevice(const std::string &device_uuid);

  int closeDevice(const std::string &device_uuid, const bool isMetux = true);

  int pauseDevice(const std::string &device_uuid, const bool isPauseOp);

  int closeDevices();

  std::set<std::string> getDevices();

  bool getDeviceType(std::string &uuid, bool &is_usb_mode);

  void regDeviceEventCallback(const RS_DEVICE_EVENT_CALLBACK &event_cb);

  void regPointCloudCallback(const RS_DEVICE_POINTCLOUD_CALLBACK &pc_cb);

  void regImageDataCallback(const RS_DEVICE_IMAGE_CALLBACK &image_cb);

  void regImuDataCallback(const RS_DEVICE_IMU_CALLBACK &imu_cb);

private:
  int findDevices(std::map<std::string, DeviceInfoItem> &devices_map);
  std::shared_ptr<PointCloudT<RsPointXYZIRT>> localGetPointCloudCallback();
  std::shared_ptr<robosense::lidar::ImageData> localGetImageDataCallback();
  std::shared_ptr<robosense::lidar::ImuData> localGetImuDataCallback();
  void localRunPointCloudCallback(
      const std::shared_ptr<PointCloudT<RsPointXYZIRT>> &msgPtr,
      const std::string &uuid);
  void localRunImageCallback(
      const std::shared_ptr<robosense::lidar::ImageData> &msgPtr,
      const std::string &uuid);
  void
  localRunImuCallback(const std::shared_ptr<robosense::lidar::ImuData> &msgPtr,
                      const std::string &uuid);
  void hotplugWorkThread();

private:
  std::thread _m_thread;
  std::thread _usb_thread;
  libusb_context *_usb_ctx;
  int _kill_handler_thread;
  bool _start;
  bool _inited;
  RS_DEVICE_EVENT_CALLBACK _event_cb;
  RS_DEVICE_POINTCLOUD_CALLBACK _pc_cb;
  RS_DEVICE_IMAGE_CALLBACK _image_cb;
  RS_DEVICE_IMU_CALLBACK _imu_cb;

  std::mutex _devices_map_mtx;
  std::map<std::string, DeviceInfoItem> _devices_map;

  bool _enable_debug;
  std::atomic_bool _is_stoping_;

private:
  const uint32_t VENDOR_ID = 0x3840;
  const uint32_t PRODUCT_ID = 0x1010;
};

} // namespace device
} // namespace robosense

#endif // SENSORMANAGER_H