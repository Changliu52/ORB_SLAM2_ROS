# install Pangolin: https://github.com/stevenlovegrove/Pangolin
sudo apt-get install -y libglew-dev ros-kinetic-octomap-mapping &&
cd
git clone https://github.com/stevenlovegrove/Pangolin.git &&
cd Pangolin
mkdir build
cd build
cmake .. &&
cmake --build . &&

# Build ORB_SLAM2 https://github.com/Changliu52/ORB_SLAM2_ROS.git
cd
git clone https://github.com/Changliu52/ORB_SLAM2_ROS.git &&
cd ORB_SLAM2
chmod +x build.sh
./build.sh &&

# ROS node
echo "export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:~/ORB_SLAM2_ROS/orb_slam2_ros:~/ORB_SLAM2_ROS/orb_slam2_lib" >> ~/.bashrc # this needs manual copy into ~/.bashrc!!
source ~/.bashrc
chmod +x build_ros.sh
./build_ros.sh

#test 
# cd testing
# cp taracali_orb.yaml ~/ORB_SLAM2_ROS/orb_slam2_ros/settings
# sudo test_tara_orb.sh
