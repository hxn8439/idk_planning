### Automation of the UR5 pick, place, and sort in ROS-Gazebo (virtual Environment) with a virtual USB cam and vacuum grippers. 

This repository demonstrates UR5 pick, place, and sort in ROS and Gazebo. The UR5 uses a USB cam to detect a color object (e.g. red box) on a conveyor belt ([`ur5_vision.py`]), and it publish the position. Then, the UR5 plans its motion ([`ur5_mp.py`]) to follow the object. Once the end-effector is close to the object, it picks up the object with the vacuum grippers by turning on ([`ur5_gripper.py`]). Note that the vacuum gripper has only one source of suction. 

- Video demos:
  [`Simulation video`](https://www.youtube.com/watch?v=Yj5DEocFa48)

- Citation of this repository: 
  ```
    Nguyen, H., Yi, J., Vu, T., Wellington, M., Safder, M., Automation of the UR5 pick, place, and sort in ROS-Gazebo and real environment with a USB cam and vacuum grippers (Melodic Distribution), (2021), Github repository, https://github.com/hxn8439/computer_system_design_project_Virtual.git
  ```
#### How to use this repository
- This project was executed in Ubuntu 20.04 with ROS Melodic.
- make sure that you installed docker on Ubuntu 20.04 and create an image file using Melodic distribution. 
- Refer to dockerHub link https://hub.docker.com/repository/docker/hxn8439/ros_melodic for the exact copy of the docker image file. 
- be familiar with git commands. 

1. Execute in ubuntu terminal CLI.
~$ 
```
sudo git clone  https://github.com/hxn8439/Ubuntu_18.04_Docker_Script
```
2. Create a docker image and container -refer to the docker flowchart for the following commands in that repository to build a catkin workspace using ROS.
3. Within the docker container, make sure to delete a pre-installed folder name universal_robot in the src folder from the catkin_ws. 
root@3id89cj5:~/catkin_ws/src#
```
rm -rf universal_robot
```
4. Navigate to the root folder of the catkin workspace and execute."catkin clean" is to wipe and clean catkin workspace. make sure that when executing this command, it has to be at the root of the workspace folder e.g. /catkin_ws/ 
root@3id89cj5:~/catkin_ws#
```
 catkin clean
```
5. Navigate to src folder in catkin workspace and execute the following CLI commands:
root@3id89cj5:~/catkin_ws/src#
 ```
  git clone https://github.com/hxn8439/computer_system_design_project.git 
  git clone https://github.com/hxn8439/universal_robot.git
 ```
6. Navigate to computer_system_design_project folder and grant super user access for the python files by execuitng the following CLI commands:
root@3id89cj5:~/catkin_ws/src/computer_system_design_project# 
```
 sudo chmod +x*.py
```
7. Navigate back to the root folder of the catkin workspace and build the ROS packages under directory using CLI commands:
root@3id89cj5:~/catkin_ws#
  ```
   catkin_make
   source devel/setup.bash  
  ```
8. Execute the ROS packages with ROS and the Gazebo and RVIZ GUI will display the virtual simulation environment. 
root@3id89cj5:~/catkin_ws# 
  ```
  roslaunch ur5_notebook initialize.launch 
  ```
#### . References 
  ```
    Huang, L., Zhao, H., Implementation of UR5 pick and place in ROS-Gazebo with a USB cam and vacuum grippers, (2018), GitHub repository, https://github.com/lihuang3/ur5_ROS-Gazebo.git
  ```

__######## Attention: This repository is closed for the senior design project and will discontinue the development as of 12/13/2021. H. Nguyen 12/13/2021###########__
