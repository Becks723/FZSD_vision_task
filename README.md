# FZSD_vision_task

# 项目简述

简单实现了rm的自瞄功能。<small>关键词：Robomaster、计算机视觉、OpenCV、ROS2、神经网络、卡尔曼滤波</small>

实现思路：

  订阅上游发布的`/image_raw`和`/camera_info`话题 > 识别装甲板 > PnP解算位姿、坐标变换 > 将装甲板信息、tf2坐标变换发布给下游

| 节点名           | 描述                                       |
| ---------------- | ------------------------------------------ |
| `armor_detector` | 唯一节点，负责整个从订阅、识别到发布的流程 |

| 话题名             | 类型                                       | 描述               |
| ------------------ | ------------------------------------------ | ------------------ |
| `/image_raw`       | `sensor_msgs/Image`                        | 订阅上游的图像信息 |
| `/camera_info`     | `sensor_msgs/CameraInfo`                   | 订阅上游的相机信息 |
| `/detector/armors` | `armor_interface/Armors`（自定义消息类型） | 发布装甲板位姿信息 |



# 构建 & 运行

- 虚拟机环境：vmware Ubuntu22.04.5

- 编译环境：ros2 humble, vscode 1.99.3, g++ (Ubuntu 11.4.0-1ubuntu1~22.04) 11.4.0, OpenCV 4.5.4 

```bash
# 编译
colcon build
# 部署
source install/setup.bash

# 运行海康相机包
ros2 launch hik_camera hik_camera.launch.py
# 运行armor_detector包
ros2 run armor_detector armor_detector
```



# 项目结构

```
.vscode
armor_detector/           # armor_detector包
  include/
  src/
    armordetectornode.cpp # armor_detector节点
    detector.cpp          # 装甲板识别逻辑
    pnpsolver.cpp         # 解算PnP逻辑
    ...
  CMakeLists.txt
  ...
armor_interface/          # 装甲板消息接口包
  msg/
    Armor.msg             # 自定义单个装甲板消息类型
    Armors.msg            # 自定义装甲板消息类型
  ...
ros2_hik_camera/          # 海康工业相机驱动包
  ...
...
```

有关`armor_interface`自定义消息类型见[这里](https://becks723.github.io/2025/06/11/ros2/#自定义消息类型XXX-msg)。