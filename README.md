# :ship: ROS2 Packages for Mechasolution Autoship Project
메카솔루션 자율운행선박 프로젝트의 ROS2 패키지입니다.

* 메카솔루션 공식 홈페이지: [바로가기](https://mechasolution.com)
* 자율운행선박 개시판: [바로가기](https://cafe.naver.com/mechawiki?iframe_url=/ArticleList.nhn%3Fsearch.clubid=29397234%26search.menuid=55)
* MCU 펌웨어 및 하드웨어 결선: [바로가기](https://github.com/mechasolution/mecha_autoship_2022-FW)
---
## :heavy_check_mark: 사전 준비
mecha_autoship 패키지를 사용하기 위해서는 아래와 같은 환경이 구성되어야 합니다.
- [Ubuntu 20.04 LTS](https://releases.ubuntu.com/20.04/)가 설치된 PC
- [Ubuntu 20.04 LTS](https://releases.ubuntu.com/20.04/) 또는 [Xubuntu 20.04](https://forums.developer.nvidia.com/t/xubuntu-20-04-focal-fossa-l4t-r32-3-1-custom-image-for-the-jetson-nano/121768)가 설치된 [Jetson NANO 4GB](https://mechasolution.com/shop/goods/goods_view.php?goodsno=590875&category=), [64gb uSD](https://mechasolution.com/shop/goods/goods_view.php?goodsno=330690&category=)
- Jetson과 USB로 연결된 조이스틱, [YDLiDAR X4](https://mechasolution.com/shop/goods/goods_view.php?goodsno=592131&category=), [Arduino NANO 33 IoT](https://mechasolution.com/shop/goods/goods_view.php?goodsno=585119&category=), [RPi Camera V2](https://mechasolution.com/shop/goods/goods_view.php?goodsno=537776&category=), USB 무선 랜카드
- Arduino와 연결된 [QMC5883](https://mechasolution.com/shop/goods/goods_view.php?goodsno=586260&category=), [NEO-6M GPS 모듈](https://mechasolution.com/shop/goods/goods_view.php?goodsno=539611&category=), [서보모터](https://mechasolution.com/shop/goods/goods_view.php?goodsno=594268&category=), [WS2812 x 8 LED Ring](https://mechasolution.com/shop/goods/goods_view.php?goodsno=543487&category=), ESC, BLDC 추진기
---
## :rocket: ROS 패키지 설치 과정 (PC, Jetson 공통)
본 패키지는 Ubuntu 20.04의 ROS2 Foxy 버전을 지원합니다. `PC 및 로봇 모두` 동일한 설치 과정을 진행해야 합니다.
### 1. ROS2 설치 & 워크스페이스 설정
- ### 방법1: 공식 설치 과정
  아래 링크를 참고해 Ubuntu 20.04에 ROS2 Foxy를 설치하고 워크스페이스를 설정합니다.

  [ROS2 Foxy Ubuntu 20.04 설치 과정](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)

  [ROS2 워크스페이스 설정 과정](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)
- ### 방법2: 아래 과정을 따라합니다.
  * Locale 설정
    ``` bash
    $ sudo apt update && sudo apt install locales
    $ sudo locale-gen en_US en_US.UTF-8
    $ sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    $ export LANG=en_US.UTF-8
    ```
  * ROS2 Repository 추가
    ``` bash
    $ sudo apt update && sudo apt install curl gnupg2 lsb-release
    $ curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    $ echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    ```
  * ROS2 Foxy 설치, 적용
    ``` bash
    $ sudo apt update
    $ sudo apt install ros-foxy-desktop
    $ source /opt/ros/foxy/setup.bash
    $ echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
    ```
  * colcon 빌드툴 및 기타 소프트웨어 설치
    ``` bash
    $ sudo apt install -y python3-pip
    $ pip3 install -U argcomplete
    $ sudo apt install python3-colcon-common-extensions
    ```
  * 워크스페이스 설정
    ``` bash
    $ mkdir -p ~/ros2_ws/src
    $ cd ~/ros2_ws
    $ colcon build
    $ . install/local_setup.bash
    $ echo "source ~/ros2_ws/install/setup.bash">>~/.bashrc
    ```
### 2. mecha_autoship 패키지 다운로드
앞서 설정한 워크스페이스 폴더로 이동해 다음 작업을 수행합니다. 이후부터 워크스페이스 디렉토리는 `~/ros2_ws`로 가정하고 진행하겠습니다.
``` bash
$ cd ~/ros2_ws/src
$ git clone https://github.com/mechasolution/mecha_autoship_2022.git
$ git clone https://github.com/YDLIDAR/YDLidar-SDK.git
$ git clone https://github.com/YDLIDAR/ydlidar_ros2_driver.git
```
### 3. 의존성 패키지 설치
mecha_autoship 패키지 사용을 위해서는 아래의 의존성 패키지가 필요합니다.

- [imu_filter_madgwick](https://index.ros.org/p/imu_filter_madgwick/) - IMU 필터
- [joy](https://index.ros.org/p/joy/) - 조이스틱 드라이버

또한 아래 파이썬 모듈이 필요합니다.
- [pyserial](https://pypi.org/project/pyserial/) - Arduino와의 통신

``` bash
$ sudo apt install ros-foxy-imu-filter-madgwick ros-foxy-joy
$ pip3 install pyserial
```
### 4. YD-LiDAR SDK 빌드
YD-LiDAR ROS2 드라이버 패키지를 사용하기 위해서는 YD-LiDAR SDK를 빌드해야합니다.
``` bash
$ cd ~/ros2_ws/src/YDLidar-SDK
$ mkdir build
$ cd build
$ cmake ..
$ make
$ sudo make install
```
### 5. mecha_autoship 패키지 빌드
``` bash
$ cd ~/ros2_ws/
$ colcon build --symlink-install
```
<p align="center"><img src="https://mechaimage.godohosting.com/mecha_git_image/mecha_autoship/img1.png"/></p>
위 사진과 같이 warning이 발생할 수 있습니다.
<br>
<br>

### 6. Namespace 설정
ROS2에서 한 네트워크에 연결된 모든 로봇은 같은 토픽을 공유합니다. 즉, `/scan`이라는 토픽을 모든 로봇들이 공유해 LiDAR를 켜지 않은 로롯이 옆의 다른 로봇의 LiDAR 데이터를 읽을 수 있습니다. 이러한 문제를 해결하기 위해 Namespace 설정이 필요하며, mecha_autoship 패키지에서는 `ROS_NAMESPACE`환경변수로 이를 설정합니다.

홈 디렉토리의 .bashrc에 `ROS_NAMESPACE` 환경변수를 팀 이름과 함께 추가해줍니다.
``` bash
$ nano ~/.bashrc
```
`.bashrc` 파일의 최하단에 다음 줄을 추가합니다. 이때 `{팀 이름}`은 팀의 영문 이름으로 합니다.
``` bash
export ROS_NAMESPACE={팀 이름}
```
예시:

``` bash
export ROS_NAMESPACE=team_mecha
```
Ctrl + x 를 입력한 후 y, Enter을 차례대로 입력해 저장한 후 빠져나옵니다. 그 후 아래 명령을 입력해 변경된 내용을 현재 터미널에 적용합니다.
``` bash
$ source ~/.bashrc
```
현재 터미널 또는 이후로 여는 모든 터미널에서 아래 명령을 입력하면 Namespace로 설정한 설정한 팀의 이름이 출력됩니다.
``` bash
$ echo $ROS_NAMESPACE
```
### 7. 시리얼 포트 접근 권한 부여
리눅스에서는 기본적으로 일반 사용자에게 시리얼 포트 접근 권한이 부여되어 있지 않습니다. 그렇기 때문에 `1`시리얼 포트의 요구 권한을 낮추거나 `2`사용자에게 시리얼 포트 접근 권한을 주는 두 방법을 사용할 수 있습니다.

`1`번 방법인 시리얼 포트의 권한을 낮추는 경우 장치를 재연결하거나 PC를 재시작하는 경우 다시 권한을 보여해야 하기 때문에 여기서는 `2`번 방법을 사용합니다.

``` bash
$ sudo adduser $USER dialout 
```
이후 재시작합니다.
``` bash
$ sudo reboot now
```
---
## :camera: OpenCV 설치(Jetson)
Jetson은 OpenCV 설치 과정이 달라 PC와 서로 다른 방법을 사용해야합니다.
### 1. CUDA 설치하기
- ### 현재 설치된 CUDA 버전 확인
  CUDA가 설치되어 있다면 CUDA 버전, 설치되어 있지 않다면 오류 메세지를 출력합니다.
  CUDA가 이미 설치되어 있다면 해당 과정을 생략하셔도 괜찮습니다.
  ```shell
  $ nvcc --version
  ```
- ### CUDA 10.0 설치
  ```shell
  $ sudo apt install -y cuda-core-10-0 \
  cuda-cublas-dev-10-0 \
  cuda-cudart-dev-10-0 \
  cuda-libraries-dev-10-0 \
  cuda-toolkit-10-0
  ```
- ### cuDNN 설치
  ```shell
  $ sudo apt install libcudnn7-dev
  ```
- ### CUDA 설치 확인
  정상적으로 설치되었다면 /usr/local/ 경로에 cuda-10.0 폴더가 생성됩니다.
  ```shell
  $ ls /usr/local/cuda*
  ```
- ### 환경변수 설정
  ```shell
  $ export PATH=/usr/local/cuda-10.0/bin${PATH:+:${PATH}}
  $ export LD_LIBRARY_PATH=/usr/local/cuda/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}
  ```
- ### gcc version 변경
  OpenCV는 Jetson에 내장된 gcc 버전과 다른 7버전을 필요로 하기때문에 gcc-7을 별도로 설치한 후 링크해줍니다.
  ```shell
  $ sudo apt install gcc-7 g++-7
  $ sudo ln -s /usr/bin/gcc-7 /usr/local/cuda-10.0/bin/gcc
  $ sudo ln -s /usr/bin/g++-7 /usr/local/cuda-10.0/bin/g++
  ```
### 2. OpenCV 설치하기
- ### OpenCV 설치 스크립트 다운로드
  스크립트 다운로드를 위해 git을 설치합니다.
  ```shell
  $ sudo apt install git
  $ git clone https://github.com/kyuhyong/buildOpenCV.git
  ```
- ### OpenCV 빌드 옵션 설정
  다운로드한 스크립트를 실행해 OpenCV 빌드 옵션을 설정합니다.
  시간이 오래 소요될 수 있습니다.
  ```shell
  $ cd buildOpenCV
  $ ./buildOpenCV.sh |& tee OpenCV_build.log
  ```
- ### OpenCV 빌드
  build 폴더로 이동하여 OpenCV를 빌드합니다.
  ```shell
  $ cd ~/opencv/build
  $ make -j4
  ```
- ### OpenCV 설치 확인
  pkg-config 명령어로 OpenCV가 설치되었는지 확인합니다.
  설치가 되었을 경우 opencv_version 함수로 설치된 OpenCV의 버전을 확인할 수 있습니다.
  ```shell
  $ pkg-config --modversion opencv4
  $ opencv_version
  ```
- ### OpenCV 설치 옵션 확인
  설치가 확인되었을 경우 파이썬에서 OpenCV를 호출하여 상세 옵션을 확인합니다. OpenCV 옵션에서 Gstreamer가 허용되어 있는지 확인합니다.
  ```shell
  $ python3
  >> import cv2
  >> print(cv2.getBuildInformation())
  ```
  만약 pkg-config 명령어로 OpenCV 설치가 확인되지만, 파이썬에서 import 오류가 발생할 경우 아래의 라이브러리를 추가로 설치합니다.
  ```shell
  $ sudo apt install libopencv-dev python3-opencv
  ```
## :desktop_computer: OpenCV 설치(PC)
OpenCV는 사용하는 PC에 따라 설치 방법이 조금씩 달라질 수 있습니다.

특히, GPU 가속을 돕는 CUDA는 GPU의 유무와 사용중인 CPU가 지원하는 CUDA 버전에 맞춰 설치를 진행해야 합니다.

여기선 GPU를 사용하지 않는 PC에서 OpenCV를 설치하는 방법을 다루겠습니다.
- ### apt 명령어로 OpenCV 설치
  ```shell
  $ sudo apt update
  $ sudo apt install libopencv-dev python3-opencv
  ```
- ### OpenCV 설치 확인
  OpenCV가 성공적으로 설치되었을 경우 OpenCV의 버전을 출력합니다.
  ```shell
  $ python3
  >> import cv2
  >> print(cv2.__version__)
  ```
---
## :package: mecha_autoship 패키지 구성
mecha_autoship은 아래 패키지로 구성되어 있습니다.
  - ### mecha_autoship_bringup
    - #### <span style="color:#c3e88d">mecha_autoship_mcu_node.py \<Node\></span>
      USB로 연결된 Arduino와 통신해 IMU, 지자기, GPS 데이터를 Publishing하고 쓰로틀, 키, RGB LED를 위한 Service의 Server 역할을 합니다.
    - #### <span style="color:#c3e88d">mecha_autoship_lidar_node.py \<Node\></span>
      YD-LiDAR의 laser 타입 데이터를 PointCloud로 변환해 Publishing합니다.
    - #### <span style="color:#d7ba7d">mecha_autoship_bringup.launch.py \<Launch\></span>
      로봇의 모든 센서와 액추에이터의 기능을 켭니다.
  - ### mecha_autoship_camera
    - #### <span style="color:#c3e88d">mecha_autoship_filtered_image_sub_node.py \<Node\></span>
      Image 타입의 사진과 ROI 타입의 좌표를 Subscribe합니다. 사진에 좌표를 그린 후 화면에 출력합니다.
    - #### <span style="color:#c3e88d">mecha_autoship_image_color_filter_node.py \<Node\></span>
      Image 타입의 사진을 Subscribe한 후 색체탐지로 ROI 타입의 사각형 좌표를 생성해 Publishing합니다.
    - #### <span style="color:#c3e88d">mecha_autoship_image_pub_node.py \<Node\></span>
      카메라에서 촬영한 사진을 Image 타입으로 변환해 Publishing합니다.
    - #### <span style="color:#c3e88d">mecha_autoship_image_sub_node.py \<Node\></span>
      Image 타입의 사진을 Subscribe한 후 화면에 출력합니다.
    - #### <span style="color:#d7ba7d">mecha_autoship_camera_publisher.launch.py \<Launch\></span>
      카메라와 색체탐지 기능을 실행합니다.
    - #### <span style="color:#d7ba7d">mecha_autoship_camera_subscriber.launch.py \<Launch\></span>
      카메라와 색체탐지 데이터를 시각화합니다.
  - ### mecha_autoship_teleop
    - #### <span style="color:#c3e88d">mecha_autoship_joystick_node.py \<Node\></span>
      조이스틱 데이터를 받아 쓰로틀, 키, RGB LED Service에 Call합니다.
    - #### <span style="color:#d7ba7d">mecha_autoship_joystick.launch.py \<Launch\></span>
      조이스틱으로 로봇을 원격 제어할 수 있도록 합니다.
  - ### mecha_autoship_interfaces
    - #### <span style="color:#eccdf4">Actuator.srv \<Interface\></span>
      쓰로틀과 키의 데이터를 표현하는 인터페이스입니다.
    - #### <span style="color:#eccdf4">Battery.srv \<Interface\></span>
      배터리의 잔량을 표현하는 인터페이스입니다.
    - #### <span style="color:#eccdf4">Color.srv \<Interface\></span>
      RGB 스트립의 색상 데이터를 표현하는 인터페이스입니다.
  - ### mecha_autoship_total
    - #### <span style="color:#c3e88d">mecha_autoship_example_node.py \<Node\></span>
      간단한 센서 데이터를 수신하고 모터와 LED 링을 제어하는 예시 노드입니다.
    - #### <span style="color:#d7ba7d">mecha_autoship_example_with_camera.launch.py \<Launch\></span>
      mecha_autoship 패키지의 모든 기능을 실행합니다. 젯슨에서만 실행 가능하며 아두이노, 라이다, 카메라가 연결되어있어야 합니다. mecha_autoship_camera 패키지의 mecha_autoship_camera_subscriber.launch.py 파일을 실행해 카메라 데이터를 확인할 수 있습니다.
      ``` bash
      $ ros2 launch mecha_autoship_camera mecha_autoship_camera_subscriber.launch.py
      ```
    - #### <span style="color:#d7ba7d">mecha_autoship_example_without_camera.launch.py \<Launch\></span>
      카메라를 제외한 mecha_autoship 패키지의 모든 기능을 실행합니다. PC에 아두이노, 라이다를 연결한 후 센서 데이터 디버깅용으로 활용할 수 있습니다.
---
## :label: 토픽 구성
mecha_autoship 패키지에서 출력하는 토픽입니다. 네임스페이스는 생략하였습니다.
- ### /Image
- ### /ROI
- ### /gps/data
- ### /imu/data
- ### /imu/data_raw
- ### /imu/mag
- ### /scan
- ### /scan_points
- ### /joy
---
## :boomerang: 서비스 구성
mecha_autoship 패키지에서 제공하는 서비스입니다. 네임스페이스는 생략하였습니다.
- ### /set_actuator
- ### /set_color