## IsaacSim projects (Used Unity 3D)

  * Publishing rgb, depth image with ROS(Using multi realsense camera)

## IsaacSim with multicamera (publish rgb, depth image) 

* Scene : Laboratory
* IsaacSim + ROS(melodic)
* After installing n Realsense cameras to obtain rgb, depth frames from each camera and publishing with different topics


<p align="center">
  <img width="800" height="500" src="https://user-images.githubusercontent.com/80872528/127097749-ae2e3e4c-4f88-43bc-81ac-258a55e747f0.png">
</p>

## Installing IsaacSim

* [Environment](https://docs.nvidia.com/isaac/isaac/doc/setup.html)
  * Ubuntu 18.04
  * Cuda 10.2 
  * RTX 2080 Ti
  * NVIDIA graphics card drivers version 440
  * Unity 2019.03.0f6 (must be used)
  * Python virtual environment (python 3.6)
 
* Installing Isaac SDK and IsaacSim Unity 3D (https://docs.nvidia.com/isaac/isaac/doc/setup.html)

* Isaac path should be made like this. -> ./isaac_sim_unity3d/isaac/sdk/...

  isaac(Unzip isaac-sdk and rename to isaac) must be in isaac_sim_unity3d.
  

## Installing Unity 2019.03.0f6

  Download UnityHub.AppImage, then run the following commands
  
  <pre>
  ./UnityHub.AppImage unityhub://2019.3.0f6/27ab2135bccf
  </pre>
  
  The link above can be copied from [https://unity3d.com/get-unity/download/archive] after right-clicking the version you want and clicking "copy link"
  
  You can execute play mode & editor mode (https://docs.nvidia.com/isaac/isaac/doc/simulation/unity3d.html)
  
## Setting python virtual environment (python 3.6)

  First, Install Anacdona
  
  <pre>
  $ conda create -n isaac_test python=3.6
  $ conda activate isaac_test
  $ pip install requirements.txt
  </pre>
  
  You can download requirements.txt from my github code directory.
  
  Run command such as [bazel run~] in virtual environment. IsaacSim needs specific python version and modules.
  
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

terminal 1 - (isaac1) ~/isaac_sim_unity3d/isaac/sdk$ bazel run //apps/tutorials/multisensor_unity3d:multisensor_unity3d 

terminal 2 - $ rviz

<p align="center">
  <img width="800" height="500" src="https://user-images.githubusercontent.com/80872528/127104830-4692edbf-c08b-453f-93a7-c5bf77b0be49.png">
</p>


## 
