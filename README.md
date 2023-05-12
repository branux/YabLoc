# YabLoc

**YabLoc**  é uma localização baseada em visão com mapa vetorial. [https://youtu.be/Eaf6r_BNFfk](https://youtu.be/Eaf6r_BNFfk)

Yabloc foi desenvolvido como uma nova pilha de localização para [Autoware](https://github.com/autowarefoundation/autoware).

[![thumbnail](docs/yabloc_thumbnail.jpg)](https://youtu.be/Eaf6r_BNFfk)


## Instalação

### Pré-requisito

**supporting `Ubuntu 22.04` + `ROS2 humble` now.**

**NOTA:** Atualmente, presume-se que este software seja criado em um espaço de trabalho separado para não contaminar o espaço de trabalho do autoware. Algum dia isso estará localizado no espaço de trabalho onde o Autoware blongs. Os seguintes submódulos serão removidos no momento.

Branches

* [main](https://github.com/tier4/YabLoc/tree/main) é um ramo para trabalhar com dependência mínima de autoware.
  * Se você quiser experimentar a demonstração do YabLoc, use `main` branch.
* [autoware(under construction)](https://github.com/tier4/YabLoc/tree/autoware) é uma ramificação feita para rodar como parte da Autoware.

Submódulos

* [external/autoware_auto_msgs](https://github.com/tier4/autoware_auto_msgs)
* [external/autoware_msgs](https://github.com/autowarefoundation/autoware_msgs.git)
* [external/septentrio_gnss_driver](https://github.com/tier4/septentrio_gnss_driver.git)
* [external/tier4_autoware_msgs](https://github.com/tier4/tier4_autoware_msgs.git)

### Como build

```shell
mkdir yabloc_ws/src -p
cd yabloc_ws
git clone git@github.com:tier4/YabLoc.git src/YabLoc --recursive
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

<details><summary>O autor costuma usar este comando de construção</summary><div>

```shell
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache --continue-on-error
```

* (optional) ccache `(--cmake-args) -DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache`

* (optional) clang-tidy `(--cmake-args) -DCMAKE_EXPORT_COMPILE_COMMANDS=ON`

* (optional) test `(--cmake-args) -DBUILD_TESTING=ON`

</div></details>

## Demonstração de início rápido

![how_to_launch_with_rosbag](docs/how_to_launch_quick_start_demo.drawio.svg)

* amostra de rosbag: [Google Drive link](https://drive.google.com/file/d/1UqULyfidxcA5JidfHWAsSqNy8itampAX/view?usp=share_link)

```shell
# terminal 1
source install/setup.bash
ros2 launch yabloc_launch sample_launch.xml

# terminal 2
source install/setup.bash
ros2 launch yabloc_launch rviz.launch.xml

# terminal 3
source install/setup.bash
ros2 bag play awsim_yabloc_rosbag_sample_0.db3 -s sqlite3
```

Se o YabLoc for executado com sucesso, você verá uma tela como a seguinte.

<img src="docs/quick_start_demo_screen.png" width="600">


## Demonstração com Autoware

**NOTE:** `use_sim_time` is TRUE as default.

### Run com rosbag 

<details><summary>clique para abrir </summary><div>

**Este é um procedimento para autolocalização, os componentes de planejamento/controle do Autoware não funcionariam dessa maneira.**

![how_to_launch_with_rosbag](docs/how_to_launch_with_rosbag.drawio.svg)

```shell
ros2 launch yabloc_launch sample_launch.xml
ros2 launch yabloc_launch rviz.launch.xml
ros2 launch autoware_launch logging_simulator.launch.xml \
  system:=false \
  localizaton:=false \
  sensing:=false \
  perception:=false \
  planning:=false \
  control:=false \
  rviz:=false \
  vehicle_model:=sample_vehicle \ 
  sensor_model:=sample_sensor_kit \
  vehicle_id:=default \
  map_path:=$HOME/Maps/odaiba

ros2 bag play your_rosbag --clock 100
```

</div></details>

### Run no mundo real

<details><summary>clique para abrir </summary><div>

**Este é um procedimento para autolocalização, os componentes de planejamento/controle do Autoware não funcionariam dessa maneira.**

![how_to_launch_with_rosbag](docs/how_to_launch_in_real.drawio.svg)

```shell
ros2 launch yabloc_launch sample_launch.xml use_sim_time:=false
ros2 launch yabloc_launch rviz.launch.xml
ros2 launch autoware_launch autoware.launch.xml \
  rviz:=false
```

</div></details>

### Execute com [AWSIM](https://github.com/tier4/AWSIM) <ins>(EM CONSTRAÇÃO)</ins>

<details><summary>clique para abrir </summary><div>

**Você tem que mudar o branch autoware.universe **

```shell
```

</div></details>

## Como definir a pose inicial

### 1. Quando o YabLoc é executado `standalone:=true`(default)  (without Autoware's pose_initializer)

1. `2D Pose Estimate` in Rviz

Você pode indicar x, y manualmente no rviz.

2. Inicialização GNSS Doppler

Se o (`ublox_msgs/msg/navpvt`) estiver disponível e o veículo se mover rápido o suficiente, o YabLoc estimará a pose inicial automaticamente.

### 2. Quando o Yabloc é executado `standalone:=false` (through Autoware's pose_initializer)

<ins>EM CONSTRUÇÃO</ins>

## Arquitetura

![node_diagram](docs/yabloc_abstruct_architecture.drawio.svg)

<details><summary>clique para mais detalhes</summary><div>

![node_diagram](docs/yabloc_architecture.drawio.svg)

</div></details>

### Tópicos de entrada

de sensores

| topic name                                           | msg type                                         | description           |
|------------------------------------------------------|--------------------------------------------------|-----------------------|
| `/sensing/imu/tamagawa/imu_raw`                      | `sensor_msgs/msg/Imu`                            |                       |
| `/sensing/camera/traffic_light/image_raw/compressed` | `sensor_msgs/msg/CompressedImage`                |                       |
| `/sensing/camera/traffic_light/camera_info`          | `sensor_msgs/msg/CameraInfo`                     |                       |
| `/sensing/gnss/ublox/navpvt`                         | `ublox_msgs/msg/NavPVT`                          | If you use ublox      |
| `/sensing/gnss/septentrio/poscovgeodetic`            | `septentrio_gnss_driver_msgs/msg/PosCovGeodetic` | If you use Septentrio |
| `/vehicle/status/velocity_status`                    | `autoware_auto_vehicle_msgs/msg/VelocityReport`  |                       |

de autoware
| nome do tópico    | tipo de msg                                | descrição                                 |
|-------------------|--------------------------------------------|-------------------------------------------|
| `/tf_static`      | `tf2_msgs/msg/TFMessage`                   | published from `sensor_kit`               |
| `/map/vector_map` | `autoware_auto_mapping_msgs/msg/HADMapBin` | published from `/map/lanelet2_map_loader` |

#### about tf_static

<details><summary>clique para abrir</summary><div>

Alguns nós requerem `/tf_static` from `/base_link` para o frame_id of `/sensing/camera/traffic_light/image_raw/compressed` (e.g. `/traffic_light_left_camera/camera_optical_link`).
You can verify that the tf_static is correct with the following command.

```shell
ros2 run tf2_ros tf2_echo base_link traffic_light_left_camera/camera_optical_link
```

If the wrong `/tf_static` are broadcasted due to using a prototype vehicle, not having accurate calibration data, or some other unavoidable reason, it is useful to give the frame_id in `override_camera_frame_id`.
If you give it a non-empty string, `/imgproc/undistort_node` will rewrite the frame_id in camera_info.
For example, you can give a different tf_static as follows.

```shell
ros2 launch yabloc_launch sample_launch.xml override_camera_frame_id:=fake_camera_optical_link
ros2 run tf2_ros static_transform_publisher \
  --frame-id base_link \
  --child-frame-id fake_camera_optical_link \
  --roll -1.57 \
  --yaw -1.570
```

</div></details>

### Output topics about pose


| topic name                                            | msg type                             | description                                |
|-------------------------------------------------------|--------------------------------------|--------------------------------------------|
| `/localicazation/pf/pose`                             | `geometry_msgs/msg/PoseStamped`      | estimated pose                             |
| `/localicazation/pose_estimator/pose_with_covariance` | `geometry_msgs/msg/PoseStamped`      | estimated pose with covariance             |

### Output topics for visualization

This project contains original rviz plugins. [rviz2_overlay_plugins](./rviz2_plugins/rviz2_overlay_plugins/README.md)

![rviz](docs/rviz_description.png)

| index | topic name                                         | description                                                                                                                                                               |
|-------|----------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 1     | `/localicazation/imgproc/lanelet2_overlay_image`   | Projection of lanelet2 (yellow lines) onto image based on estimated pose. If they match well with the actual road markings, it means that the localization performs well. |
| 2     | `/localicazation/imgproc/segmented_image`          | result of graph-based segmetation. yellow area is identified as the road surface.                                                                                         |
| 3     | `/localicazation/pf/cost_map_image`                | cost map generated from lanelet2.                                                                                                                                         |
| 4     | `/localicazation/imgproc/image_with_line_segments` | detected line segments                                                                                                                                                    |
| 5     | `/localicazation/map/ground_status`                | ground height and tilt estimatation status                                                                                                                                |
| 6     | `/localicazation/twist/kalman/status`              | twist estimation status                                                                                                                                                   |
| 7     | `/localicazation/pf/predicted_particle_marker`     | particle distribution of particle fitler (red means a probable candidate)                                                                                                 |
| 8     | `/localicazation/pf/gnss/range_marker`             | particle weight distribution by GNSS                                                                                                                                      |
| 9     | `/localicazation/pf/scored_cloud`                  | 3D projected line segments. the color means the how match they are                                                                                                        |

## License

YabLoc is licensed under Apache License 2.0.
