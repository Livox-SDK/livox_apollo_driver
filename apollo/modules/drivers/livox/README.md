## Livox_Apollo_Driver
Livox_Apollo_Driver can convert Livox LiDAR's raw data to standard format of point cloud, and write to point cloud channels.

### Point cloud output channels

Each LiDAR corresponds to a point cloud channel.
 ```
channel: /apollo/sensor/livox/front/center/PointCloud2
type: apollo::drivers::PointCloud
proto: [modules/drivers/proto/pointcloud.proto]https://github.com/ApolloAuto/apollo/blob/master/modules/drivers/proto/pointcloud.proto

channel: /apollo/sensor/livox/front/left/PointCloud2
type: apollo::drivers::PointCloud
proto: [modules/drivers/proto/pointcloud.proto]https://github.com/ApolloAuto/apollo/blob/master/modules/drivers/proto/pointcloud.proto

...
```

### Config file
```
use_hub: true
lidars_conf {
  lidar_conf {
    frame_id: "livox_horizon_front_center"
    hub_port: 1
    convert_channel_name: "/apollo/sensor/livox/front/center/PointCloud2"
    fan_status : true
    return_mode : DualReturn
  }
  lidar_conf {
    frame_id: "livox_horizon_front_left"
    hub_port: 2
    convert_channel_name: "/apollo/sensor/livox/front/left/PointCloud2"
    fan_status : true
    return_mode : DualReturn
  }
  ...
}
```
`use_hub` represents wether to use Livox Hub device，currently only support use Livox Hub. `hub_port` represents Livox Hub device's hardware port number. `lidars_conf` contains each LiDAR's configuration parameters, each LiDAR corresponds to a `lidar_conf`. `sn` represents LiDAR's broadcast code. `convert_channel_name` represents LiDAR's point cloud output channel. `fan_status` represents turn on/off fan. `return_mode` represents return mode of point cloud，it can be configured to：(1) FirstReturn; (2) StrongestReturn; (3) DualReutrn.

**Notice**

    1. MID40/100 don't support config `return_mode` and `fan_status`.


### How to add to Apollo Project

Livox_Apollo_Driver has the same directory structure as Apollo5.0

1. copy livox_apollo_driver to the same folder of your apollo project：
  ```bash
  cp -r livox_apollo_driver/apollo/modules/drivers/livox  your_apollo_project/apollo/modules/drivers/
  cp livox_apollo_driver/apollo/docker/build/installers/install_livox_sdk.sh  your_apollo_project/apollo/docker/build/installers/
  ```
2. start and enter apollo docker:
  ```bash
  bash /apollo/docker/scripts/dev_start.sh
  bash /apollo/docker/scripts/dev_into.sh
  ```
3. install livox_sdk dynamic link library:
  ```bash
  sudo bash /apollo/docker/build/installers/install_livox_sdk.sh
  ```
4. build project:
  ```bash
  bash apollo.sh build
  ```

### Start Livox_Apollo_Driver

```bash
#in docker
cd /apollo && cyber_launch start modules/drivers/livox/launch/livox.launch
```

### Livox_Apollo_Driver test

```bash
#in docker
cd /apollo && ./bazel-bin/modules/drivers/livox/driver/driver_test
```

