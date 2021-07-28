## IsaacSim projects (Simulator : Unity 3D)

  * Publishing rgb, depth image with ROS(Using multi realsense D435i)

## Publishing RGBD image with ROS(Using multi realsense D435i)

* Scene : 연구실
* IsaacSim + ROS(melodic)
* Unity 3D Scene에 n개 Realsense D435i 센서 부착 후 RGB, Depth image들을 각자 다른 topic으로 publish

<p align="center">
  <img width="800" height="500" src="https://user-images.githubusercontent.com/80872528/127097749-ae2e3e4c-4f88-43bc-81ac-258a55e747f0.png">
</p>

## 개발 환경 구성

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
    </pre>

  * Unity 2019.03.0f6 설치 (Unity Editor 모드)

    1. [UnityHub.AppImage](https://forum.unity.com/threads/unity-hub-v-1-3-2-is-now-available.594139/) 다운로드
    2. 2019.3.0f6 버전 설치
       <pre>
       ./UnityHub.AppImage unityhub://2019.3.0f6/27ab2135bccf
       </pre>
    
    * 위와 같은 버전 링크는 [https://unity3d.com/get-unity/download/archive]에서 원하는 버전의 "Unity Hub" 우클릭 후 "copy link address" 클릭하여 얻을 수 있음
    
* 기타 준비 사항
  
  * MultiSensor_Simulation/multisensor_unity3d 다운로드 후 








    
    
## Other preparations

* Download multisensor_unity3d directory from my github and add in /isaac_sim_unity3d/isaac/sdk/apps/tutorials/

* Download DepthToRos.cpp, hpp from my github and add in /isaac_sim_unity3d/isaac/sdk/packages/ros_bridge/components.

* Download camera_to_ros.subgraph.json from my github and replace with /isaac_sim_unity3d/isaac/sdk/packages/ros_bridge/apps/camera_to_ros.subgraph.json.

## Creating scene and packages

<pre>
~/isaac_sim_unity3d$ mkdir projects/test
~/isaac_sim_unity3d$ cp -r projects/sample/Assets projects/sample/Packages projects/sample/ProjectSettings projects/test/
</pre>

Download rgbdROS.unity.meta from my github and add in /isaac_sim_unity3d/projects/test/Assets. 

rgbdROS.unity.meta is scene that I made. 

Open the project
<pre>
~/isaac_sim_unity3d$ ~/Unity/Hub/Editor/2019.3.0f6/Editor/Unity -projectPath projects/test -logfile
</pre>

[You have to download and import additional asset from asset store.](https://youtu.be/GFcA9U45poQ)

<p align="center">
  <img width="800" height="500" src="https://user-images.githubusercontent.com/80872528/127104398-7bef679c-a172-41ca-99a3-d9367e362da6.png">
</p>

I used above assets.

## Execute

Run simulation in Unity 3D.
<pre>
terminal 1 - (isaac1) ~/isaac_sim_unity3d/isaac/sdk$ bazel run //apps/tutorials/multisensor_unity3d:multisensor_unity3d 
terminal 2 - $ rviz
</pre>
<p align="center">
  <img width="800" height="500" src="https://user-images.githubusercontent.com/80872528/127104830-4692edbf-c08b-453f-93a7-c5bf77b0be49.png">
</p>


## Sensor description

* 카메라 가져오기

  GameObject > Camera

* 카메라에 센서 스크립트 붙이기

  There are color camera and depth camera scripts in Packages/com.nvidia.isaac_sim_core/Scripts/Runtime/Sensor/.
  
  Drag the color camera script and depth camera script to the camera. You can edit intrinsic, extrinsic parameter in the inspector.
  
  <p align="center">
    <img width="200" height="500" src="https://user-images.githubusercontent.com/80872528/127105690-9d018204-443a-44ec-8a5e-bf9ff7c64ec8.png">
  </p>


여기서부터 이어서 쓰기

* Isaac Sim과 unity simulation 연결

  multisensor_unity3d.app.json helps isaac and simulation be connected.

  * Sight와 연결
  
  * ROS와 연결






