### Automation of the UR5 pick, place, and sort in Physical Environment with a RealSense camera and vacuum suction gripper. 

- Video demos:
  [`Simulation video`](https://youtu.be/WiZtcR9hqvM)
  
- Check our blogpost for comprehensive documentation and the scrum development of this repository.
  https://blog.uta.edu/cseseniordesign/2021/12/10/project-identify-classify-idc/

- Citation of this repository: 
  ```
    Nguyen, H., Yi, J., Vu, T., Wellington, M., Safder, M., Automation of the UR5 pick, place, and sort in Physical Environment with a RealSense camera and vacuum suction gripper. (Bionic Distribution), (2021), Github repository, https://github.com/hxn8439/idk_planning.git
  ```
#### Python Unit Test

![Screenshot](/UnitTestpython.png)

#### How to use this repository
- This project was executed in Ubuntu 20.04 with ROS Bionic distribution.
- Make sure that you installed docker on Ubuntu 20.04 and create an image file using Bionic distribution. 
- Refer to dockerHub link https://hub.docker.com/r/capstoneidk/senior-design-demo for the exact copy of the docker image file. 
- Be familiar with git commands. 

1. Execute in ubuntu terminal CLI.
~$ 
```
sudo git clone  https://github.com/hxn8439/Ubuntu_18.04_Docker_Script
```

2. Create a docker image and container -refer to the docker flowchart for the following commands in that repository to build a catkin workspace using ROS.

3. Navigate to the root folder of the catkin workspace and execute."catkin clean" is to wipe and clean catkin workspace. make sure that when executing this command, it has to be at the root of the workspace folder e.g. /catkin_ws/ 
root@3id89cj5:~/catkin_ws#
```
 catkin clean
```

4. Navigate to src folder in catkin workspace and execute the following CLI commands:
root@3id89cj5:~/catkin_ws/src#
 ```
  git clone https://github.com/hxn8439/idk_planning.git 
  git clone https://github.com/hxn8439/fmauch_universal_robot.git
  git clone https://github.com/hxn8439/realsense-ros.git
  git clone https://github.com/hxn8439/Universal_Robots_ROS_Driver.git
 ```
 5. Navigate back to the root folder of the catkin workspace and build the ROS packages under directory using CLI commands:
root@3id89cj5:~/catkin_ws#
  ```
   catkin_make
   source devel/setup.bash  
  ```
 6. After that, execute the following CLI command:
root@3id89cj5:~/catkin_ws#
  ```
  connect_robot
  ``` 
  
  7. After that, execute the following command on ROS Tablet:
  ```
  connect_idk
  ``` 
  8. After that, execute the following CLI command:
root@3id89cj5:~/catkin_ws#
  ```
  connect_moveit
  ``` 
   9. After that, execute the following CLI command:
root@3id89cj5:~/catkin_ws#
  ```
  connect_realsense
  ``` 
   10. After that, execute the following CLI command:
root@3id89cj5:~/catkin_ws#
  ```
  rosrun idk_planning main.py
  ``` 
   11. UR5 robotic arm should be executing and a camera feed GUI shall be display on screen.
   
#### . References 

__######## Attention: This repository is closed for the senior design project and will discontinue the development as of 12/13/2021. H. Nguyen 12/13/2021###########__
