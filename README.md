# Unity-ROS Simulator
Meant to be used with HiPeRLab General Code.

## Install Unity

The instructions from this point onward is about the Unity simulator.

* First, [install Unity Hub](https://docs.unity3d.com/hub/manual/InstallHub.html#install-hub-linux) on your computer.
`wget -qO - https://hub.unity3d.com/linux/keys/public | gpg --dearmor | sudo tee /usr/share/keyrings/Unity_Technologies_ApS.gpg > /dev/null`
`sudo sh -c 'echo "deb [signed-by=/usr/share/keyrings/Unity_Technologies_ApS.gpg] https://hub.unity3d.com/linux/repos/deb stable main" > /etc/apt/sources.list.d/unityhub.list'`
`sudo apt update`
`sudo apt-get install unityhub`

* Open up Unity Hub, but don't install the recommended Unity Editor. The default project was created using 2021.3.16f1. Go to Install Editor and choose the one that starts with "2021.3" to avoid too much compatibility issue. (Any version after 2020 should in theory work, but this has not been tested)

* Once the installation completes, open the Unity project from repo. In Unity Hub, click Add -> Add project from desk, and select the "Unity Simulator" folder and click "Add Project".

* Open the project with your Unity Hub. You might get some warnings if the versions are not exactly compatible, but this should be okay.
![image](https://hackmd.io/_uploads/r1gvVWhM1e.png)

* In the Assets panel below, double-click on "Scenes" and **"Simulator Example"** (The other scenes will be soon removed). On the right, you should see these objects in the project.
![image](https://hackmd.io/_uploads/r1pzr-3zke.png)

* Under the "Drone" object, you should see a few cameras. Make sure only the depth camera is enabled for running RAPPIDS only. Verify that a script called "Depth Image Publisher" is attached to the drone. Also click on "Drone" and verify that "Sim Pose Subscriber" is attached. Note that the vehicle ID is by default set to 110, so make sure this is the ID you use in the simulator node.
![image](https://hackmd.io/_uploads/Hy8XcU2MJx.png)
![image](https://hackmd.io/_uploads/B18OKP2Myl.png)
![image](https://hackmd.io/_uploads/SkBQj82MJx.png)


## Check connection

Open your workspace a couple terminals again.

* In all terminals:
`cd catkin_ws`
`source devel/setup.bash`

* Instead of running `roscore`, do:
    `roslaunch ros_tcp_endpoint endpoint.launch`
    * If you run into some error related to python, try something like
`sudo ln -sf /usr/bin/python3 /usr/local/bin/python`

* Once the endpoint node launches successfully, click the start button in Unity. ![image](https://hackmd.io/_uploads/SJAZHDnfkx.png)
* You should see blue arrows on the top left corner of the Unity display if the connection is successful: 
![image](https://hackmd.io/_uploads/HyQBBDhM1g.png)

* Sync simulator node:
`rosrun hiperlab_rostools sync_simulator 110`

* Run the RAPPIDS control node:
`rosrun hiperlab_rostools quad_rappids_rates_control 110`
    * Note that the -x direction is forward on this drone, unfortunately :P
    ![image](https://hackmd.io/_uploads/B1wqeFnzJg.png)


* Finally, keyboard node:
`rosrun hiperlab_hardware keyboard_control`
    * Note that in this example, you need to press the green button after take off when the node prints out the current estimate. This is written into the logic so that we can terminate the mission at this point in case VIO goes wrong.

* You should now see the vehicle fly forward 10m and then landing.
