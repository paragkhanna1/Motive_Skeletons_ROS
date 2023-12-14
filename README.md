# Motive_Skeletons_ROS
Repo with a ROS1-package for getting skeleton data from Motive-Optotrack mocap application to ROS

Compatible with Motive 4.0.0 , download here: https://optitrack.com/support/downloads/developer-tools.html

Follow these steps to setup Natnet:
1. $ mkdir ~/NatNet
2. Extract the downloaded file here and remane it to NatNet_SDK_4.0
3. $ cd ~/NatNet/NatNet_SDK_4.0/samples/SampleClient
4. $ make clean
5. $ make -f makefile
6. $ cd build
7. $ export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/NatNet/NatNet_SDK_4.0/lib
   or to add it in bashrc 
   $ echo -n "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/NatNet/NatNet_SDK_3.0/lib" >> ~/.bashrc
8. Verfiy that you can listen to data
   $ ./SampleClient

Remember in the CMakelists.txt, library linked via: 
_add_executable(mocap_client_global_coordinate src/mocap_client_global_coordinate.cpp)
target_link_libraries(mocap_client_global_coordinate ${catkin_LIBRARIES} ~/NatNet/NatNet_SDK_4.0/lib/libNatNet.so)
_
(Change if needed)
