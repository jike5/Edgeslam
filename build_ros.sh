echo "Building ROS nodes"
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/workspace/edgeslam/Examples/ROS
cd Examples/ROS/Edge_SLAM
rm -rf build && mkdir build
cd build
cmake .. -DROS_BUILD_TYPE=Release
make -j
