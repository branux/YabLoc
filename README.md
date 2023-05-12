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

| nome do tópico                                       | tipo de msg                                      | descrição           |
|------------------------------------------------------|--------------------------------------------------|-----------------------|
| `/sensing/imu/tamagawa/imu_raw`                      | `sensor_msgs/msg/Imu`                            |                       |
| `/sensing/camera/traffic_light/image_raw/compressed` | `sensor_msgs/msg/CompressedImage`                |                       |
| `/sensing/camera/traffic_light/camera_info`          | `sensor_msgs/msg/CameraInfo`                     |                       |
| `/sensing/gnss/ublox/navpvt`                         | `ublox_msgs/msg/NavPVT`                          | Se você usa ublox      |
| `/sensing/gnss/septentrio/poscovgeodetic`            | `septentrio_gnss_driver_msgs/msg/PosCovGeodetic` | Se você usa Septentrio |
| `/vehicle/status/velocity_status`                    | `autoware_auto_vehicle_msgs/msg/VelocityReport`  |                       |

de autoware
| nome do tópico    | tipo de msg                                | descrição                                 |
|-------------------|--------------------------------------------|-------------------------------------------|
| `/tf_static`      | `tf2_msgs/msg/TFMessage`                   | published from `sensor_kit`               |
| `/map/vector_map` | `autoware_auto_mapping_msgs/msg/HADMapBin` | published from `/map/lanelet2_map_loader` |

#### about tf_static

<details><summary>clique para abrir</summary><div>

Alguns nós requerem `/tf_static` from `/base_link` para o frame_id de `/sensing/camera/traffic_light/image_raw/compressed` (Ex: `/traffic_light_left_camera/camera_optical_link`).
Você pode verificar se o tf_static está correto com o seguinte comando.

```shell
ros2 run tf2_ros tf2_echo base_link traffic_light_left_camera/camera_optical_link
```

Se os erros `/tf_static` forem transmitidos devido ao uso de um protótipo de veículo, falta de dados de calibração precisos ou algum outro motivo inevitável, é útil fornecer o frame_id em `override_camera_frame_id`.
Se você fornecer uma string não vazia, `/imgproc/undistort_node`irá reescrever o frame_id em camera_info. Por exemplo, você pode fornecer um tf_static diferente da seguinte maneira.

```shell
ros2 launch yabloc_launch sample_launch.xml override_camera_frame_id:=fake_camera_optical_link
ros2 run tf2_ros static_transform_publisher \
  --frame-id base_link \
  --child-frame-id fake_camera_optical_link \
  --roll -1.57 \
  --yaw -1.570
```

</div></details>

### Tópicos de saída sobre pose


| nome do tópico                                        | tipo de msg                          | descrição                                  |
|-------------------------------------------------------|--------------------------------------|--------------------------------------------|
| `/localicazation/pf/pose`                             | `geometry_msgs/msg/PoseStamped`      | pose estimada                             |
| `/localicazation/pose_estimator/pose_with_covariance` | `geometry_msgs/msg/PoseStamped`      | pose estimada com covariância             |

### Tópicos de saída para visualização

Este projeto contém plugins rviz originais. [rviz2_overlay_plugins](./rviz2_plugins/rviz2_overlay_plugins/README.md)

![rviz](docs/rviz_description.png)

| index | nome do tópico                                     | descrição                                                                                                                                                               |
|-------|----------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 1     | `/localicazation/imgproc/lanelet2_overlay_image`   | Projeção de lanelet2 (linhas amarelas) na imagem com base na pose estimada. Se eles combinarem bem com as marcações reais da estrada, isso significa que a localização funciona bem. |
| 2     | `/localicazation/imgproc/segmented_image`          | resultado da segmentação baseada em gráfico. a área amarela é identificada como a superfície da estrada.                                                                                         |
| 3     | `/localicazation/pf/cost_map_image`                | mapa de custo gerado a partir de lanelet2.                                                                                                                                       |
| 4     | `/localicazation/imgproc/image_with_line_segments` | segmentos de linha detectados                                                                                                                                                    |
| 5     | `/localicazation/map/ground_status`                | status de estimativa de altura e inclinação do solo                                                                                                                                |
| 6     | `/localicazation/twist/kalman/status`              | status de estimativa de torção                                                                                                                                                   |
| 7     | `/localicazation/pf/predicted_particle_marker`     | distribuição de partículas do filtro de partículas (vermelho significa um provável candidato)                                                                                                 |
| 8     | `/localicazation/pf/gnss/range_marker`             | distribuição de peso de partícula por GNSS                                                                                                                                      |
| 9     | `/localicazation/pf/scored_cloud`                  | Segmentos de linha projetados em 3D. a cor significa como eles combinam                                                                                                        |

## Licença

YabLoc é licenciado sob Apache License 2.0.
