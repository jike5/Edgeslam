# Copyright 2019 The KubeEdge Authors.
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#     http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


# pwd: /workspace/edgeslam/Examples
/workspace/edgeslam/Examples/RGB-D/rgbd_tum /workspace/edgeslam/Vocabulary/ORBvoc.txt /workspace/edgeslam/Examples/RGB-D/TUM3.yaml client websocket "$PATH_TO_SEQUENCE_FOLDER" "$ASSOCIATIONS_FILE"

# You can also use ROS:
# roslaunch Edge_SLAM  edge.launch tum_name:=TUM3
# please notes that the topic subscribed in code are :/camera/rgb/image_color, /camera/depth/image. 

# The parameter a represents the name of the configuration file, please refer to https://github.com/raulmur/ORB_SLAM2#tum-dataset-1