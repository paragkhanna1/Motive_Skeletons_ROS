# Motive_Skeletons_ROS
Repo with a ROS1-package for getting skeleton data from Motive-Optitrack mocap application to ROS-1

Compatible with Natnet 4.0.0 , download here: https://optitrack.com/support/downloads/developer-tools.html
In the Motive application, change transmission type to "Unicast" in View:Data Streaming Pane

Follow these steps to setup Natnet:
1. $ mkdir ~/NatNet
2. Extract the downloaded file here and remane it to NatNet_SDK_4.0
3. $ cd ~/NatNet/NatNet_SDK_4.0/samples/SampleClient
4. $ make clean
5. $ make -f makefile
6. $ cd build
7. $ export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/NatNet/NatNet_SDK_4.0/lib
8. $ echo -n "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/NatNet/NatNet_SDK_3.0/lib" >> ~/.bashrc
9. Verfiy that you can listen to data
   $ ./SampleClient

Remember in the CMakelists.txt, library linked via: <br/>
_add_executable(mocap_client_global_coordinate src/mocap_client_global_coordinate.cpp)
target_link_libraries(mocap_client_global_coordinate ${catkin_LIBRARIES} ~/NatNet/NatNet_SDK_4.0/lib/libNatNet.so)


(Change above if needed)

<br/>

The code assumes 2 skeletons and a rigid body (baton) being tracked in the MoCap room

For global skeleton coordinates in the Motive application use <br/>
$ rosrun mocap_skeleton mocap_client_global_coordinate

For local skeleton coordinates in the Motive application use <br/>
$ rosrun mocap_skeleton mocap_client_local_coordinate




Tested on ROS melodic and Ros Noetic

Tested with Motivebody 2.3.0
