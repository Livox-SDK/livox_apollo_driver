## Livox_Apollo_Driver
Livox_Apollo_Driver 可以将 Livox LiDAR 原始数据转换为标准点云数据格式,并输出到点云 channels.

### 点云输出 channels

每个 LiDAR 都对应一个点云通道
 ```
channel: /apollo/sensor/livox/front/center/PointCloud2
type: apollo::drivers::PointCloud
proto: [modules/drivers/proto/pointcloud.proto]https://github.com/ApolloAuto/apollo/blob/master/modules/drivers/proto/pointcloud.proto

channel: /apollo/sensor/livox/front/left/PointCloud2
type: apollo::drivers::PointCloud
proto: [modules/drivers/proto/pointcloud.proto]https://github.com/ApolloAuto/apollo/blob/master/modules/drivers/proto/pointcloud.proto

...
```

### 配置文件
```
use_hub: true
hub_sn: "13UUG1R00400170"
lidars_conf {
  lidar_conf {
    frame_id: "livox_horizon_front_center"
    sn: "1HDDG8M00100191"
    convert_channel_name: "/apollo/sensor/livox/front/center/PointCloud2"
    fan_status : true
    return_mode : DualReturn
  }
  lidar_conf {
    frame_id: "livox_horizon_front_left"
    sn: "1HDDGAU00100351"
    convert_channel_name: "/apollo/sensor/livox/front/left/PointCloud2"
    fan_status : true
    return_mode : DualReturn
  }
  ...
}
```
`use_hub` 表明是否使用 Hub 设备，`hub_sn` 表示 Hub 设备的广播码. `lidars_conf` 包含所有雷达的配置参数,
每新增一个 LiDAR 设备，都需要增加对应的`lidar_conf`. `sn` 表示 LiDAR 的广播码. `convert_channel_name` 表示 LiDAR 的输出点云通道. `fan_status` 表示风扇的开关. `return_mode` 表示点云的回波模式，可以配置为：(1) FirstReturn; (2) StrongestReturn; (3) DualReutrn.

**注意**

    1. Mid40/100 不支持回波模式和风扇开关的设置.
    2. 广播码包含了 LiDAR 序列号和一个额外数字. 序列号可以在 LiDAR 的机身找到(QR 码的下面). Hub 的额外数字为 0. MID-40, Horizon 和 Tele 的额外数字为 1. MID-100 的额外数字为 1,2,3, 分别对应MID-100设备的左，中和右. 例如, 如果 Hub 的序列号为 "13UUG1R0040017", 则 Hub 的广播码为 "13UUG1R00400170".


### 如何加入 Apollo Project

Livox_Apollo_Driver 和 Apollo5.0 的目录一致

1. 将 livox_apollo_driver 代码拷贝到 apollo 工程的同级目录中:
  ```bash
  cp -r livox_apollo_driver/apollo/modules/drivers/livox  your_apollo_project/apollo/modules/drivers/
  cp livox_apollo_driver/apollo/docker/build/installers/install_livox_sdk.sh  your_apollo_project/apollo/docker/build/installers/
  ```
2. 启动和进入 apollo docker:
  ```bash
  bash /apollo/docker/scripts/dev_start.sh
  bash /apollo/docker/scripts/dev_into.sh
  ```
3. 安装 livox_sdk 动态链接库:
  ```bash
  sudo bash /apollo/docker/build/installers/install_livox_sdk.sh
  ```
4. 构建项目:
  ```bash
  bash apollo.sh build
  ```

### 启动 Livox_Apollo_Driver

```bash
#in docker
cd /apollo && cyber_launch start modules/drivers/livox/launch/livox.launch
```

### Livox_Apollo_Driver 测试

```bash
#in docker
cd /apollo && ./bazel-bin/modules/drivers/livox/driver/driver_test
```

