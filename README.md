# **Pick and Place Operation by LEGO EV3 Robot Manipulator**
---

- `Robot5_v3_10.m`: Includes the whole script needed to operate the robot manipulator in all of its activities.
- `LEGO_EV3_MANIPULATOR_REPORT.pdf`: Contains useful information for operation of Robot
- `Actual_working_of_robot.mp4`: Working of robot in video format(mp4).
- `inverse_kinematics.png`: Diagram of robot links and its crucial angles


# **Introduction**
<p align="justify"> 
A robotic system constructed with the LEGO Mindstorms EV3 platform is called the LEGO EV3 Robot Manipulator. One well-liked robotics kit for educators and hobbyists that lets users construct and operate their own robots is the LEGO Mindstorms EV3. Robot manipulators and other robotic projects can be made with the EV3 kit, which comes with a variety of sensors, motors, and programmable bricks.  
<img src="https://github.com/shreyaskorde16/LEGO-EV3-Manipulator/blob/main/robot.png" width="500" height="500" align="right"/>  
A robot manipulator is a type of robot that is designed to manipulate objects, often with a specific end-effector, such as a gripper or an arm with various joints. The LEGO EV3 Robot Manipulator typically consists of the following key components:
<p align="justify"> 
  
- <p align="justify">EV3 Brick: The central control unit of the robot, which contains the programmable microcontroller. Users can create custom programs to control the manipulator's movements and actions.
<p align="justify"> 
  
- <p align="justify"> Motors: The EV3 kit includes two or more motors that can be used to drive the robot's various joints and the end-effector (such as a gripper or a claw). These motors provide the manipulator with the ability to move and grasp objects.
<p align="justify"> 
  
- <p align="justify">Sensors: The EV3 kit includes various sensors, such as touch sensors, color sensors, ultrasonic sensors, and gyro sensors. These sensors can be used to provide feedback and enable the robot manipulator to interact with its environment and objects.
<p align="justify"> 
  
- <p align="justify"> LEGO Technic Elements: LEGO Technic parts, including beams, gears, axles, and connectors, are used to build the mechanical structure of the robot manipulator. These components allow for the creation of complex joints and linkages, which give the robot its ability to perform precise and coordinated movements.
<p align="justify"> 
  
- <p align="justify">End-Effector: The manipulator's end-effector is the tool or device at the end of the robot arm that is responsible for interacting with objects. It can be a gripper, a claw, or any other attachment that suits the intended task. With the LEGO EV3 Robot Manipulator, you can design and build a robotic arm capable of picking up objects, sorting items, or performing other manipulation tasks. It's a great way to introduce robotics and programming concepts
</p>

# **Dimensions**
<img src="https://github.com/shreyaskorde16/LEGO-EV3-Manipulator/blob/09a2a57ab95224427c76cefb4f220ca3a2b7aade/links.png" width="350" height="350" align="right"/>

`link 1`: 50mm  

`link 2`: 95mm  

`link 3`: 185mm  

`link 4`: 110mm  

`link 0`: 70mm  

`Gear ratio of motor C `: 3  

`Gear ratio of motor A`: 5  

# **Working**

Explanation of various functions used in the script are as follows:

 <div style="text-align: center">
   
  - `home`: This function is used to raise the robot arm to position B, leaving the gripper open and the arm raised.
  - `armDown`: In order to lower the robot arm downward.
  - `armup`: In order to lower the robot arm upward.
  - `gripper_operations`: Use to `OPEN` and `CLOSE` the gripper of the Robot.
  - `inv_kine`: This function uses an inverse kinematics solution to find out the angles theta1 and theta2.
  - `turnC`: To set the gripper at position C.
  - `turn_b_fromC`: to turn the gripper from position `B` to `C`.
  - `turn_a`: To set the gripper at position `A`.
  - `turn_c_fromA`:To set the gripper from position `C` to `A`
  
 </div>




Video link: __[Watch the working of LEGO EV3 Manipulator](https://youtu.be/QXRAOuqKnGk)__
