## IsaacSim projects (Simulator : Unity 3D)

  * Publishing rgb, depth image with ROS(Using multi realsense D435i)
  * Pose control with ROS tf message

## Publishing RGBD image with ROS(Using multi realsense D435i)

* Scene : 연구실
* IsaacSim + ROS(melodic)
* Unity 3D Scene에 n개 Realsense D435i 센서 부착 후 RGB, Depth image들을 각자 다른 topic으로 publish

<p align="center">
  <img width="800" height="500" src="https://user-images.githubusercontent.com/80872528/127097749-ae2e3e4c-4f88-43bc-81ac-258a55e747f0.png">
</p>

### 개발 환경 구성

* 개발 환경
  * Ubuntu 18.04 (필수)
  * RTX 2080 Ti 사용 - Cuda 10.2 (필수), cuDNN 8.0.3
  * NVIDIA graphics card drivers version 440 (version 440 이상 필요)  
  * Bazel 3.1.0 (필수)
  * Isaac SDK(2020.2), IsaacSim Unity3D (2020.2)
  * Python virtual environment (python 3.6, requirements.txt 내 명시된 모듈) 
  * Unity 2019.03.0f6 (필수, 다른 버전 사용시 오류 발생했음)
  * ROS Melodic
  
  
* [개발 환경 상세 설명](https://docs.nvidia.com/isaac/isaac/doc/setup.html)

  * GPU는 compute capability 6.1 이상 필요하며 Readme 작성일 기준(2021.07.28) [Cuda 10.2](https://developer.nvidia.com/cuda-10.2-download-archive) 설치 요구

  * [cuDNN 8.0.3 설치](https://developer.nvidia.com/rdp/cudnn-archive)

  * [Nvidia gpu driver 440 설치](https://docs.nvidia.com/isaac/isaac/doc/setup.html#nvidia-gpu-drivernvidia-gpu-driver) 

    <pre>
    bob@desktop:~/isaac/sdk$ sudo add-apt-repository ppa:graphics-drivers/ppa
    bob@desktop:~/isaac/sdk$ sudo apt-get update
    bob@desktop:~/isaac/sdk$ sudo apt-get install nvidia-driver-440
    </pre>

  * Isaac SDK, IsaacSim Unity 3D 설치

    1. https://developer.nvidia.com/isaac/downloads 들어가서 하단 ARCHIVE (Click to toggle open/close) 클릭
    2. ISAAC 2020.2의 SDK, SIM(Unity) 다운로드
    3. 둘 다 unzip 후 isaac_sim_unity3d-20201123-197b4c38 -> isaac_sim_unity3d로, isaac-sdk-20201201-427971df2 -> isaac 으로 이름 수정
    4. isaac_sim_unity3d 안에 isaac 폴더 삽입

  * [Bazel 3.1.0 설치](https://docs.bazel.build/versions/main/install-ubuntu.html)

    3.1.0 버전으로 수정 후 설치 주의
    
  * Python virtual environment 구성 (Anaconda 이용했으며 동일하게 세팅시 별도 설치 필요)
    
    1. [Anaconda 설치](https://www.anaconda.com/products/individual#Downloads)
    2. requirements.txt(업로드된 파일) 다운로드
    3. 가상 환경 세팅
       <pre>
       $ conda create -n isaac_test python=3.6
       $ conda activate isaac_test
       $ pip install requirements.txt
       </pre>
       
    * requirements.txt는 필요 모듈과 각 버전 열거한 리스트
    * 앞으로 bazel run 등을 이용하여 application 실행시 isaac_test 가상환경 안에서 진행할 것

  * [Dependencies 설치](https://docs.nvidia.com/isaac/isaac/doc/setup.html#installing-dependencies-on-the-desktop)
    
    <pre>
    bob@desktop:~/isaac/engine/$ ./engine/build/scripts/install_dependencies.sh
    </pre>Creating scene and packages

  * Unity 2019.03.0f6 설치 (Unity Editor 모드)

    1. [UnityHub.AppImage](https://forum.unity.com/threads/unity-hub-v-1-3-2-is-now-available.594139/) 다운로드
    2. 2019.3.0f6 버전 설치
       <pre>
       ./UnityHub.AppImage unityhub://2019.3.0f6/27ab2135bccf
       </pre>
    
    * 위와 같은 버전 링크는 [https://unity3d.com/get-unity/download/archive]에서 원하는 버전의 "Unity Hub" 우클릭 후 "copy link address" 클릭하여 얻을 수 있음
    
    
* Publishing RGBD image with ROS 준비 사항
  
  * /isaac_sim_unity3d/isaac/sdk/apps/tutorials/에 MultiSensor_Simulation/multisensor_unity3d (업로드 된 폴더) 다운로드
  * /isaac_sim_unity3d/isaac/sdk/packages/ros_bridge/components에 MultiSensor_Simulation/DepthToRos.cpp, hpp (업로드 된 파일) 다운로드
  * /isaac_sim_unity3d/isaac/sdk/packages/ros_bridge/components/BUILD 에 아래 코드 추가 
  
    <pre>
    isaac_component(
    name = "depth_to_ros",
    tags = ["manual"],
    visibility = ["//visibility:public"],
    deps = [
        ":proto_to_ros_converter",
        "//packages/ros_bridge/gems",
        "//third_party:ros",
    ],
    )
    </pre>
    
  * /isaac_sim_unity3d/isaac/sdk/packages/ros_bridge/BUILD 에 아래 코드 추가
    
    <pre>
    "//packages/ros_bridge/components:depth_to_ros",
    </pre>


### Scene, Project 구성

<pre>
~/isaac_sim_unity3d$ mkdir projects/test
~/isaac_sim_unity3d$ cp -r projects/sample/Assets projects/sample/Packages projects/sample/ProjectSettings projects/test/
</pre>

 1. /isaac_sim_unity3d/projects/test/Assets 에 rgbdROS.unity, rgbdROS.unity.meta (업로드 된 파일) 다운로드
 2. project 열기
   <pre>
   ~/isaac_sim_unity3d$ ~/Unity/Hub/Editor/2019.3.0f6/Editor/Unity -projectPath projects/test -logfile
   </pre>
   
 3. File - Open Scene - /isaac_sim_unity3d/projects/test/Assets/rgbdROS.unity 클릭
 
 4. [Asset 다운로드](https://youtu.be/GFcA9U45poQ)

    아래의 Asset 사용 (모두 Download 후 Unity 내에서 import)

    <p align="center">
      <img width="600" height="500" src="https://user-images.githubusercontent.com/80872528/127104398-7bef679c-a172-41ca-99a3-d9367e362da6.png">
    </p>

### 실행

1. Unity3D에서 Scene 열고 시뮬레이션 시작
2. Applicaiton 실행

<pre>
terminal 1 - (isaac_test) ~/isaac_sim_unity3d/isaac/sdk$ bazel run //apps/tutorials/multisensor_unity3d:multisensor_unity3d 
terminal 2 - $ rviz
</pre>

<p align="center">
  <img width="800" height="500" src="https://user-images.githubusercontent.com/80872528/127104830-4692edbf-c08b-453f-93a7-c5bf77b0be49.png">
</p>







### Sensor description (Unity 3D)
(카메라 기존 2개보다 추가시 참고)

* 카메라 가져오기

  [Menu] GameObject > Camera

* 카메라에 센서 스크립트 붙이기

  * Color, depth camera scripts 위치 : Packages/com.nvidia.isaac_sim_core/Scripts/Runtime/Sensor/.
  * 두 스트립트를 카메라에 드래그(color, depth 순서로)
  * 카메라 클릭하면 나오는 inspector에서 intrinsic, extrinsic parameter 수정 가능 (본인은 [Realsense D435i](https://www.intelrealsense.com/depth-camera-d435i/) 스펙에 맞춤)

  <p align="center">
    <img width="400" height="700" src="https://user-images.githubusercontent.com/80872528/127105690-9d018204-443a-44ec-8a5e-bf9ff7c64ec8.png">
  </p>

* Isaac Sim과 unity simulation 연결

### [Scene description](https://docs.nvidia.com/isaac/isaac/doc/simulation/unity3d.html) (Unity 3D)
(새로 project, scene 구성할 때 참고)

* isaac sdk와 Unity를 연결하려면 [isaac.alice, isaac.minisight를 scene에 드래그](https://docs.nvidia.com/isaac/isaac/doc/simulation/unity3d.html#add-carter-and-isaac-applications) (NVIDIA IsaacSim for Unity3D(Core) > Prefabs에 위치)

### Application description - multisensor_unity3d.app.json (Isaac)
   * Isaac sdk에서는 json 형식을 이용해 전체 연결 시스템을 구성하며 그래프로 구성해야 한다. 크게 node, edge, Config로 이루어져 있다. 
   * [일반적으로 Unity Simulation과 Sight(Viewer) 및 기타 동작들을 연결시켜주는데 이용된다](https://docs.nvidia.com/isaac/isaac/doc/tutorials/building_apps.html?highlight=edge%20detection#processing-input-from-simulation) 
   * multisensor_unity3d.app.json 파일은 Isaac sdk로서 Unity simulation과 ROS를 이어주는 중간 매개체 역할을 해준다. 
   * Json 파일 구성시에는 동일 디렉토리 내에 BUILD 파일도 있어야 한다.

   * BUILD 파일
   
     <pre>
     name = "multisensor_unity3d" 
     </pre>
     
     name은 application의 명칭
     
     <pre>
     data = [
          "//packages/navsim/apps:navsim_navigation_subgraph"
     ]
     </pre>
    
     data에는 어떤 subgraph가 들어가는지 표기한다. 외부에 있는 다른 json 형식 파일을 가져올 때 subgraph로 지정하며 path도 함께 표기해줘야 한다.
     
     subgraph를 함수라고 생각해도 되며 반복적으로 쓰이는 부분만 외부 json 파일로 저장 후 갖다 쓰는 방식으로 써도 된다.
     
     <pre>
     modules = [
               "rgbd_processing",
                      "sight",
                      "viewers",
               "ros_bridge",
               "behavior_tree"
     ]
      </pre>
      
      modules에는 json 파일 구성에 있어서 필요한 외부 cpp, hpp 패키지 이름을 쓴다. 
      
      /isaac_sim_unity3d/isaac/sdk/packages/에 Isaac SDK 설치시 내장된 모듈들을 볼 수 있다.
      
  * multisensor_unity3d.app.json

    * name과 modules은 BUILD 파일과 동일하게 써준다.
    
    * simulation
      
      BUILD 파일에서 명시한 simulation subgraph를 써준다. simulation에서 출력되는 camera frame들을 다른 node로 이동시킬 수 있다.
      
      ```json
      {
        "name": "simulation",
        "subgraph": "packages/navsim/apps/navsim_navigation.subgraph.json"
      }
      ```
      
    * camera_viewer 1, 2
      
      ```json
      { 
        "name": "camera_viewer",
        "components": [
          {
            "name": "ledger",
            "type": "isaac::alice::MessageLedger"
          },
          {
            "name": "ImageViewer",
            "type": "isaac::viewers::ImageViewer"
          },
          {
            "name": "DepthViewer",
            "type": "isaac::viewers::DepthCameraViewer"
          }          
        ]
      }
      ```
      
      name에는 이 json 파일에서 해당 노드를 어떻게 부를건지 써주면 되고, components에는 이 노드가 무엇들로 구성되어 있는지 써준다.
      
      서로 다른 패키지에 있는 component를 이렇게 하나의 노드로 묶을 수 있다.
      
      simulation 카메라가 촬영하는 영상을 localhost로 열리는 웹페이지(Sight)에서 보여주는 역할을 한다.
      
      Scene에서 두 개의 카메라를 이용했기 때문에 Sight의 camera viewer 노드도 총 2개 만들었다.
      
    * point cloud
    
      ```json
      {
        "name": "point_cloud",
        "components": [
          {
            "name": "message_ledger",
            "type": "isaac::alice::MessageLedger"
          },
          {
            "name": "depth_to_pointcloud",
            "type": "isaac::rgbd_processing::DepthImageToPointCloud"
          },
          {
            "name": "viewer",
            "type": "isaac::viewers::PointCloudViewer"
          }
        ]
      }
      ```
      
      Simulation에서 출력되는 depth image를 Sight에서 point cloud로 볼 수 있게 해준다.
      
    * ros converter
      ```json
      {
        "name": "ros_converters",
        "components": [
          {
            "name": "MessageLedger",
            "type": "isaac::alice::MessageLedger"
          },
          {
            "name": "ImageToRos",
            "type": "isaac::ros_bridge::ImageToRos"
          },
          {
            "name": "DepthToRos",
            "type": "isaac::ros_bridge::DepthToRos"
          },
          {
            "name": "CameraIntrinsicsToRos_color",
            "type": "isaac::ros_bridge::CameraIntrinsicsToRos"
          },
          {
            "name": "ImageToRos2",
            "type": "isaac::ros_bridge::ImageToRos"
          },
          {
            "name": "DepthToRos2",
            "type": "isaac::ros_bridge::DepthToRos"
          },
          {
            "name": "CameraIntrinsicsToRos_depth",
            "type": "isaac::ros_bridge::CameraIntrinsicsToRos"
          }
        ],
        "disable_automatic_start": true
      }
      ```
      
      Simulation 카메라에서 출력되는 rgb, depth 영상을 ros로 publish 할 수 있게 해준다.
   
    * Edge
   
      ```json
      {
        "source": "simulation.interface/output/depth",
        "target": "camera_viewer/DepthViewer/depth"
      },
      {
        "source": "simulation.interface/output/depth_intrinsics",
        "target": "camera_viewer/DepthViewer/intrinsics"
      },
      {
        "source": "simulation.interface/output/color",
        "target": "camera_viewer/ImageViewer/image"
      }
      ```
      simulation rgb frame <-> sight edge
      
      ex) target : "camera_viewer/DepthViewer/depth" ==> camera_viewer 노드의 DepthViewer의 depth proto로 보낸다 
      
      <p align="center">
        <img width="700" height="500" src="https://user-images.githubusercontent.com/80872528/127318219-f714a308-9e3d-4afc-bf08-f92d247b5b2c.png">
      </p>
      
      * Viewer 여는 방법 

        1. application, simulation 실행 후 http://localhost:3000/ 에 들어간다.        
        2. 좌측 Channels에서 multisensor_unity3d > camera_viewer > ImageViewer 우클릭 > Create a new 2d Renderer > 이름 : ImageViewer - 2d
        3. 모든 Viewer 동일하게 열면 되고 각 viewer 이름은 다르게 해줘야 한다.
        
      
      ```json
      {
        "source": "simulation.interface/output/depth",
        "target": "point_cloud/depth_to_pointcloud/depth"
      },
      {
        "source": "simulation.interface/output/depth_intrinsics",
        "target": "point_cloud/depth_to_pointcloud/intrinsics"
      },
      {
        "source": "simulation.interface/output/color",
        "target": "point_cloud/depth_to_pointcloud/color"
      },
      {
        "source": "point_cloud/depth_to_pointcloud/cloud",
        "target": "point_cloud/viewer/cloud"
      }
      ```
      <p align="center">
        <img width="700" height="500" src="https://user-images.githubusercontent.com/80872528/127318521-6dffafeb-2de3-43da-a7e4-8c69215af48b.png">
      </p>
      
      * 3D Viewer 여는 방법
        
        point_cloud 우클릭 후 Create a new 3d Renderer
      
      
      Simulation의 depth frame을 pointcloud로 만들어주고 viewr로 전송
      
      ```json
      {
        "source": "simulation.interface/output/color",
        "target": "ros_converters/ImageToRos/proto"
      },
      {
        "source": "simulation.interface/output/depth",
        "target": "ros_converters/DepthToRos/proto"
      },
      {
        "source": "simulation.interface/output/intrinsics",
        "target": "ros_converters/CameraIntrinsicsToRos_color/proto"
      }
      ```
      
      Simulation의 rgb, depth image를 ros converter로 전송하면 ros converter가 자동으로 publish

      각 카메라의 rgb frame, depth frame 모두 ros topic이 다르므로 ros converters 내에서도 componet 이름을 다르게 해줘야 한다. 
      
      ex) ImageToRos, ImageToRos2, DepthToRos, DepthToRos2
      
      Config에서 각 component 상세 구성을 해주게 되는데 이 때 하나의 componet 당 하나의 topic name을 가지기 때문이다.
      
      
      
    * Config
    
      camera_viewer 1, 2에서 ImageViewer와 DepthViewer의 parameter를 각자 구성해준다. 
      
      ```json
      "camera_viewer": {
      "ImageViewer": {
        "target_fps": 20.0,
        "reduce_scale": 4
      },
      "DepthViewer": {
        "colormap": [
          [ 128, 0, 0 ],
          [ 255, 0, 0 ],
          [ 255, 255, 0 ],
          [ 0, 255, 255 ],
          [ 0, 0, 255 ],
          [ 0, 0, 128 ]
        ],
        "min_visualization_depth": 0.0,
        "max_visualization_depth": 20.0,
        "target_fps": 20,
        "reduce_scale": 4
      }
      }
      ```
      
      behavior_bridge와 ros_node를 구성해줘야 rosnode를 만들어 publish가 가능해진다.
      
      ```json
      "behavior_bridge": {
      "tick_period": "30Hz",
      "NodeGroup": {
        "node_names": [
          "$(fullname ros_node)",
          "$(fullname ros_converters)"
        ]
      }
      },

      "ros_node": {
        "RosNode": {
          "tick_period": "30Hz"
        }
      }
      ```
      
      ros_converters에서 channel_name(topic)을 중복되지 않게 구성해준다.
      
      ```json
      "ros_converters": {
        "ImageToRos": {
          "ros_node": "$(fullname ros_node)",
          "channel_name": "/camera_rect/image_rect/color"
        },
        "DepthToRos": {
          "ros_node": "$(fullname ros_node)",
          "channel_name": "/camera_rect/image_rect/depth"
        },
        "CameraIntrinsicsToRos_color": {
          "ros_node": "$(fullname ros_node)",
          "channel_name": "/camera_rect/camera_info/color"
        },
        "ImageToRos2": {
          "ros_node": "$(fullname ros_node)",
          "channel_name": "/camera_rect/image_rect/color2"
        },
        "DepthToRos2": {
          "ros_node": "$(fullname ros_node)",
          "channel_name": "/camera_rect/image_rect/depth2"
        },
        "CameraIntrinsicsToRos_depth": {
          "ros_node": "$(fullname ros_node)",
          "channel_name": "/camera_rect/camera_info/depth"
        }
      }
    
      ```
      
      기타 API 설명 - https://docs.nvidia.com/isaac/archive/2020.1/doc/doc/component_api.html


### RGB, Depth Sensor 추가하는 방법

- Unity 3D 수정

  * Unity3D의 Sensor를 복사하여 원하는 위치로 이동 후 Inspector에서 color camera, depth camera script의 channel name을 color 3,...으로 바꾼다.

- multisensor_unity3d.app.json 수정

  Node
  
  * Isaac sight가 추가적으로 필요할 경우, node 파트에서 camera_viewer 3,... 추가해준다
  * ros_converters 노드에 ImageToRos3, DepthToRos3, ... 등의 component들을 추가해준다.
  
  Edge
  
  * edges 부분에서는 위에서 추가된 노드와 simulation에서 추가된 센서의 channel에 맞게 source와 target을 추가해 써준다.
  * simulation의 "simulation.interface/output/color" 채널의 color 부분에 unity에서 추가한 channel name을 써주며 depth 채널에서도 마찬가지로 써준다.
  * sight와 연결시에도 동일하게 적용해준다.
  
  Config
  
  * Config에서도 sight 추가할 경우 camera_viewer3, ... 를 만들어주고 ros_converters에도 ImageToRos3, DepthToRos3,... 하나씩 추가해준다. 
  * config - ros converters의 channel_name은 ros topic이므로 중복되지 않게 추가해준다.



### 기타 참고 예제

ros bridge - https://docs.nvidia.com/isaac/isaac/packages/ros_bridge/doc/ros_bridge.html

creating new project - https://docs.nvidia.com/isaac/isaac/doc/simulation/unity3d.html#creating-a-unity-project-with-isaac-sim

opencv edge detection - https://docs.nvidia.com/isaac/isaac/doc/tutorials/building_apps.html 



## Pose control with ROS tf message

### 시뮬레이터 환경 구성

  * [개발 환경 구성](https://github.com/SungjoonCho/IsaacSim_Simulation#user-content-%EA%B0%9C%EB%B0%9C-%ED%99%98%EA%B2%BD-%EA%B5%AC%EC%84%B1)
  * [Scene,Project 구성](https://github.com/SungjoonCho/IsaacSim_Simulation#user-content-scene-project-%EA%B5%AC%EC%84%B1)

### Unity3D - Pose control 할 대상에 script 붙이기
  
  
  <p align="center">
    <img width="1000" height="500" src="https://user-images.githubusercontent.com/80872528/129162341-2e353914-3e71-45e7-91db-44abf0a4c467.png">
  </p>

  1. chair_ros로 이름 변경
  2. NVIDIA IsaacSim for Unity3D (Core) > Scripts > Runtime > Actuator 에서 Teleport.cs 드래그하여 inspector에 script 붙이기
  3. Teleport(Script) > Objects > Size 1로 수정 후 Element 0 을 chair_ros 선택
    
### ros_bridge package에 RosToRigid 추가

  1. 업로드된 PoseControl/RosToRigid 디렉토리 내부 RosToRigid.cpp, RosToRigid.hpp 다운로드 후 isaac/sdk/packages/ros_bridge/components/ 로 이동
  2. /isaac/sdk/packages/ros_bridge/components/BUILD 파일 하단에 아래 코드 추가
  
     ```json
     <pre>
      isaac_component(
           name = "ros_to_rigid",
           tags = ["manual"],
           visibility = ["//visibility:public"],
           deps = [
               ":ros_node",
               "//packages/ros_bridge/gems",
               "//third_party:ros",
           ],
       )
     </pre>

### posecontrol.app.json 
 
  * PoseControl/posecontrol 디렉토리 다운로드 후 /isaac/sdk/apps/tutorials 로 이동
  
### talker (ROS message publisher)

  * talker/talker.cpp, talker/CMakeLists.txt 다운로드 후 rosrun 하여 tf 정보 ROS publish

### 실행

  * terminal 1
    <pre>
      ~/tf_talker$ rosrun tf_talker talker
    </pre>
    
  * terminal 2
    <pre>
      (isaac_test) ~/isaac_sim_unity3d/isaac/sdk$ bazel run //apps/tutorials/posecontrol:posecontrol
    </pre>
    
  * Unity 3D scene 시뮬레이션 실행

    <p align="center">
      <img width="1000" height="500" src="https://user-images.githubusercontent.com/80872528/129161873-3bccad63-ef3c-4eee-b7d8-47e342905490.png">
    </p>
        
        
### Description

   * 전체 흐름
     
     talker에서 ROS tf publish -> RosToRigid에서 subscribe 후 pose 정보를 RigidBody3Group proto 형식에 맞게 저장, simulator에 publish(ROS X) 
   
     -> chair_ros script에서 pose 반영하여 teleport

   * RosToRigid.cpp, hpp

     RosToRigid는 RosToPose와 RigidBody3GroupGenerator 병합하기 위해 새로 cutomizing 하였으며, ROS로 받아온 정보 중 필요한 정보만 Rigid 객체 형식에 맞춰 저장 후 proto로 만드는 것이 목적
   
     1. ROS /tf topic으로 메시지 subscribe하여(tf2_ros::TransformListener에서 topic 확인 가능) rotation, translation을 Pose3d에 저장
     2. RigidBody3Group에 pose 대입, 그 외 velocity, acceleration은 0으로 초기화
     3. posecontrol.app.json의 config > RosToRigid에서 body_name 및 기타 parameter (ex. velocity, acceleration) 설정 가능(값들은 RigidBody3Group에 저장됨)
     4. 생성된 bodies proto는 시뮬레이션의 chair_ros teleport channel로 이동

     
     talker에서 pose data publish 할 때 target frame, source frame은 map, base_link로 초기화 해놓았고, RosToRigid에서도 동일한 frame 쓰일 수 있게 hpp에서 미리 초기화 
     
     
   * posecontrol.app.json

     1. Edge의 RosToRigid 결과물인 bodies proto를 시뮬레이션의 chair_ros teleport channel과 연결
     2. Config의 RosToRigid에서 ROS 연결 위한 parameter와 Pose control 할 대상 이름 설정

     
