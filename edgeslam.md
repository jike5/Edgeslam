# Edge-SLAM

* **Authors**

[Ali J. Ben Ali](https://github.com/benaliny), [Zakieh Sadat Hashemifar](https://github.com/Zakieh), [Karthik Dantu](https://github.com/dkkarthik)

* **Description**

Edge-SLAM is an edge-assisted visual simultaneous localization and mapping.
Edge-SLAM adapts Visual-SLAM into edge computing architecture to enable long
operation of Visual-SLAM on mobile devices. This is achieved by offloading the
computation-intensive modules to the edge. Thus, Edge-SLAM reduces resource
usage on the mobile device and keeps it constant. Edge-SLAM is implemented on
top of [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2).

* **Presentation Video**

[![Presentation Video](https://img.youtube.com/vi/lCDf07iQLQs/0.jpg)](https://youtu.be/lCDf07iQLQs)

* **Demonstration Videos**

[![Demonstration Elevator Pitch Video](https://img.youtube.com/vi/LLsbALyANA8/0.jpg)](https://youtu.be/LLsbALyANA8)

[![Demonstration Full Video](https://img.youtube.com/vi/AeTK4EyfRZ0/0.jpg)](https://youtu.be/AeTK4EyfRZ0)

## Publications

Ali J. Ben Ali, Zakieh Sadat Hashemifar, and Karthik Dantu. 2020. Edge-SLAM:
edge-assisted visual simultaneous localization and mapping. In Proceedings of
the 18th International Conference on Mobile Systems, Applications, and Services
(MobiSys ’20). Association for Computing Machinery, New York, NY, USA, 325–337.
[DOI](https://doi.org/10.1145/3386901.3389033)

## License

Edge-SLAM is released under the same ORB-SLAM2 license, i.e., [GPLv3
license](https://github.com/droneslab/edgeslam/blob/master/License-gpl.txt).

For a list of all code/library dependencies (and associated licenses), please
see
[Dependencies.md](https://github.com/droneslab/edgeslam/blob/master/Dependencies.md).

If you use Edge-SLAM in an academic work, please cite:

```
@inproceedings{10.1145/3386901.3389033,
author = {Ben Ali, Ali J. and Hashemifar, Zakieh Sadat and Dantu, Karthik},
title = {Edge-SLAM: Edge-Assisted Visual Simultaneous Localization and Mapping},
year = {2020},
isbn = {9781450379540},
publisher = {Association for Computing Machinery},
address = {New York, NY, USA},
url = {https://doi.org/10.1145/3386901.3389033},
doi = {10.1145/3386901.3389033},
booktitle = {Proceedings of the 18th International Conference on Mobile Systems, Applications, and Services},
pages = {325–337},
numpages = {13},
keywords = {mobile systems, mapping, visual simultaneous localization and mapping, split architecture, localization, edge computing},
location = {Toronto, Ontario, Canada},
series = {MobiSys '20}
}
```

## Usage

### Environment Setup

* Setup the
  [ORB-SLAM2](https://github.com/droneslab/edgeslam/blob/master/ORB-SLAM2.md)
  prerequisites in section 2 on mobile and edge devices.

* Setup [Boost C++ Libraries](https://www.boost.org/) on mobile and edge
  devices and make sure both devices have the same Boost version.

* Our testing setup:
  * We run Edge-SLAM using the RGB-D/Stereo dataset using the ROS examples.
  * Ubuntu 18.04 LTS.
  * OpenCV 3.4.2.
  * Boost 1.65.1.
  * ROS Melodic Morenia.
  * Eigen3 3.3.4.

### Building Edge-SLAM

* Clone the repository

  ```
  git clone https://github.com/droneslab/edgeslam.git
  ```
    
* Run the build script

  ```
  cd edgeslam
  chmod +x build.sh
  ./build.sh
  ```

This will create **libEdge_SLAM.so** at **lib** folder and the executables
**mono_tum**, **mono_kitti**, **mono_euroc**, **rgbd_tum**,
**stereo_kitti**, and **stereo_euroc** in **Examples** folder.

* To build ROS examples
  * Add the path including **Examples/ROS/Edge_SLAM** to the
    `ROS_PACKAGE_PATH` environment variable. Open `.bashrc` file and add at the
    end the following line. Replace `PATH` by the folder where you cloned
    Edge-SLAM
    
    ```
    export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:PATH/edgeslam/Examples/ROS
    ```
    已经将该句话添加到 `build_ros.sh` 中
        
  * Run the ROS build script

    ```
    chmod +x build_ros.sh
    ./build_ros.sh
    ```

### Running Edge-SLAM

* Download the following TUM RGB-D dataset bag file
  [freiburg2_desk](https://vision.in.tum.de/rgbd/dataset/freiburg2/rgbd_dataset_freiburg2_desk.bag), and adjust the `DepthMapFactor` in `Examples/RGB-D/TUM2.yaml` to `1.0`.
* On mobile device
  * Open a new terminal window, navigate to project root directory, and run

    ```
    roscore
    ```

  * Open a second terminal window, navigate to project root directory, and run

    ```
    cd Examples/ROS/Edge_SLAM/
    rosrun Edge_SLAM RGBD ../../../Vocabulary/ORBvoc.txt ../../RGB-D/TUM2.yaml client
    ```

* On edge device
  * Open a new terminal window, navigate to project root directory, and run

    ```
    roscore
    ```

  * Open a second terminal window, navigate to project root directory, and run

    ```
    cd Examples/ROS/Edge_SLAM/
    rosrun Edge_SLAM RGBD ../../../Vocabulary/ORBvoc.txt ../../RGB-D/TUM2.yaml server
    ```

* When Edge-SLAM is running on both devices, you will be asked in the Edge-SLAM
  terminal to enter IP and port numbers to setup the network connections.
  Edge-SLAM uses three TCP connections. First connection transmits keyframes,
  second connection transmits frames, and third connection transmits map
  updates. Follow the below steps in the order presented to setup the network
  * On the edge device
    * Enter the edge device IP address
    * Enter the port number to use for keyframe connection on the edge
  * On the mobile device
    * Enter the mobile device IP address
    * Enter the edge device IP address
    * Enter the port number to use for keyframe connection on the mobile
    * Enter the port number used for keyframe connection on the edge
    * If keyframe connection is established, the system will continue asking to
      setup the second connection
  * On the edge device
    * Enter the port number to use for frame connection on the edge
  * On the mobile device
    * Enter the port number to use for frame connection on the mobile
    * Enter the port number used for frame connection on the edge
    * If frame connection is established, the system will continue asking to
      setup the third connection
  * On the edge device
    * Enter the port number to use for map update connection on the edge
  * On the mobile device
    * Enter the port number to use for map update connection on the mobile
    * Enter the port number used for map update connection on the edge
    * If map update connection is established, the system on the mobile device
      should open the viewer windows, and the edge device should wait to
      receive data.
* On the mobile device, open a third terminal window, navigate to project root
  directory, and run the bag file
  ```
  rosbag play ./rgbd_dataset_freiburg2_desk.bag /camera/rgb/image_color:=/camera/rgb/image_raw /camera/depth/image:=/camera/depth_registered/image_raw
  ```
