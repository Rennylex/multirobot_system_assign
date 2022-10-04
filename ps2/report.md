
# COSC.69.13, 2022 fall, PA2, Ziang Ren(Graduate student)

## 1. Method Explanation

## 1.1 General Description
  Overall speaking, the project aims to let us develop a node that can make the robot to do the flocking behavior
  containing 3 movements:
  1.  Align: robots group trying to move in the same directions
  2.  Cohesion: robots that are relatively far trying to be togather and moving 
  3.  Seperation: robots that are relatively close trying to spread out a little bit.
  
  In order to let the robot implement the aforementioned behaviors, we first need to decide how to pass down the instructions--to be more specific, what parameters we should set up. My idea is that, the robot should receive a 
 target direction--`target_yaw`-- dynamically given by our algorithm. This `target_yaw` will be changing the whole time according to the local situational awarenss of the robots. It is also a parameter calculated by weighting the yaw angle 
 given by `self.align()`, `self.cohesion()`, and `self.seperation()`.
  
  
  ## 1.2. Code implementation
  
  What we're gonna do is to implement the code.
 ### 1.2.1. Launch File
  Different from the previous task, in this task, we need to spawn multiple robots and control them. For the Gazebo simulator, we can do so buy declaring multiple robots in the launch file, and clarify their respective namespace:
  ```launch
  <launch>
  <arg name="model" default="waffle_pi"/>
  <arg name="first_tb3"  default="robot_0"/>
  <arg name="second_tb3" default="robot_1"/>
  <arg name="third_tb3"  default="robot_2"/>

  <rosparam param="namespaces">
  ["robot_0", "robot_1", "robot_2"]
  </rosparam>
  ```
  For each robot, we need to initialize its spawn location with the following lines:
  ```launch
  <arg name="third_tb3_x_pos" default=" -5.5"/>
  <arg name="third_tb3_y_pos" default=" 0.0"/>
  <arg name="third_tb3_z_pos" default=" 0.0"/>
  <arg name="third_tb3_yaw"   default=" 0.0"/>
  ```
    
  At last, we call the `spawn_urdf` node to spawn the robots, and our own node to let the robot follow our instructions.
  ```launch
   <group ns = "$(arg first_tb3)">
    <param name="robot_description" command="$(find xacro)/xacro --inorder $(find turtlebot3_description)/urdf/turtlebot3_$(arg model).urdf.xacro" />

    <node pkg="robot_state_publisher" type="robot_state_publisher" name="robot_state_publisher" output="screen">
      <param name="publish_frequency" type="double" value="50.0" />
      <param name="tf_prefix" value="$(arg first_tb3)" />
    </node>
    <node name="spawn_urdf" pkg="gazebo_ros" type="spawn_model" args="-urdf -model $(arg first_tb3) -x $(arg first_tb3_x_pos) -y $(arg first_tb3_y_pos) -z $(arg first_tb3_z_pos) -Y $(arg first_tb3_yaw) -param robot_description" />
      <node pkg="simple_shape" type="simple_shape" name="simple_shape" output="screen"/>
  </group>
  ```
  
  When the world is initialized, it should look like this.
  ![avatar](https://github.com/Rennylex/multirobot_system_assign/blob/main/ps2/ini_gazebo.png)
  
  #### world file modification
  If we want to test the robots in the StageROS, we need to generate our own world file that defines the robots objects and floor plan. Here is how I spawn 10 robots in the `.world` file.
  ```world
  # throw in 10 robots
erratic( pose [ -10.277 23.266 0.000 75.000 ] name "era" color "blue")
erratic( pose [ -14.277 19.266 0.000 10.000 ] name "era2" color "blue")
erratic( pose [ -13.277 18.266 0.000 160.000 ] name "era3" color "blue")
erratic( pose [ -6.277 25.266 0 150.000 ] name "era4" color "blue")
erratic( pose [ -11.277 18.266 0 180.000 ] name "era5" color "blue")
erratic( pose [ -10.277 26.266 0 80.000 ] name "era6" color "blue")
erratic( pose [ -13.277 25.266 0 120.000 ] name "era7" color "blue")
erratic( pose [ -8.277 27.266 0 130.000 ] name "era8" color "blue")
erratic( pose [ -7.277 23.266 0 140.000 ] name "era9" color "blue")
erratic( pose [ -3.277 25.266 0 150.000 ] name "era10" color "blue")
  ```
  I use the robot class `erratic`, and I need to specify the location and the yaw angle of the robots when defining them.
  The names I give them are not their namespaces. They should have the namespaces `robot_0` to `robot_9`
  
  In order to get the namespace of the robot currently running the node, we can use `self.robot_name=rospy.get_namespace()`
  The index for the robot should be the second one to the end. Use `self.robot_no=self.robot_name[-2]` to get that digit.
  When the world is initialized, it should look like this.
  ![avatar](https://github.com/Rennylex/multirobot_system_assign/blob/main/ps2/ini_world.png)
  
   
  ### 1.2.2. Python file
  
  #### Structure
  
  Since the ultimate message we need to give to the robots is the direction--which indicate where the velocity vector should be pointing, my implementation is to let the robots get moving the whole time, but also have them constantly check
 if their `self.yaw` equals to the `target_yaw`.
 
 ```python
     def move_forward(self):
        """Function to move_forward"""
        # Rate at which to operate the while loop.
        rate = rospy.Rate(FREQUENCY)

        # Setting velocities. 
        twist_msg = Twist()
        while not rospy.is_shutdown():
            alig_yaw=self.align()#align to the direction of the goal
            sep_yaw=self.seperation()#seperate from other robots
            coh_yaw=self.cohesion()#move to the center of the group
            counter=1#counter for the number of robots that are in the range
            if(sep_yaw!=0):
                counter+=1
            if(coh_yaw!=0):
                counter+=1
            target_yaw=alig_yaw+sep_yaw+coh_yaw/counter#calculate the target yaw
            dif_yaw=target_yaw-self.yaw#calculate the difference between the target yaw and the current yaw            

            if(dif_yaw>0.1):#if the target yaw is to the left of the current yaw
                twist_msg.angular.z=math.pi/4#turn to the target yaw
            elif(dif_yaw<-0.1):#if the target yaw is to the right of the current yaw
                twist_msg.angular.z=-math.pi/4#turn to the target yaw
            else:
                twist_msg.angular.z=0#stop turning

            twist_msg.linear.x = self.linear_velocity
            if self._close_obstacle:#when in front of obstacle, turn 180 degrees.
                self.rotation_in_place(math.pi)
            else:
                self._cmd_pub.publish(twist_msg)

            # Sleep to keep the set publishing frequency.
            rate.sleep()

        # Traveled the required distance, stop.
        self.stop()
        
 ```
  The above code is modified from the original `move_forward()`. Here, we let the robot move forward all the time--unless it detects a obstacles(for obstacles detection, the Gazebo simulator allows for `laser_scan` topic, while for the StageROS, we can use `base_scan` topic). in the while loop, we first calculate the yaw angle for alignment, seperation and cohesion, and then we combine them altogether to get the ultimate `target_yaw`. If `dif_yaw`, which is the difference between current yaw and the `target_yaw` is within the `MAX_ERROR`, the robot will maintain its current direction.
  Also, for the obstacle avoidance, the robot will turn 180 degrees when it find itself in front of an obstacle.
  
  
  
  
  #### Subscribe topics
  
  Since we need to know the location and orientation information of other robots, we need to subscribe to those topics in odom.
  ```python
  #subscribing to a robot's odom information
  self.robo_0_odom=rospy.Subscriber('/robot_'+str(0)+'/odom', Odometry,  self._odom_callback_2, queue_size=4)
  ```
  This subscribing process should be repeated until all robots' `odom` msgs are subscribed. In the callback function
  `self._odom_callback_2`, the info will be stored in the numpy arrays where the key is the index of the robot, and the value is the corresponding information like x coordinate and y coordinate.
 


#### self.align()

Aligment means that robots should be move in the same direction. The alignment function loop through all neighboring robots, and then get their velocity vectors, and ultimately average the velocity vectors to get the final direction. Since all robots here are initialized with the same speed magnitude, we can directely average the yaw angles of different neighboring robots.

```python
    def align(self):
        """this function return the desired speed, which is the average speed of two other robots(in a vector)"""
        cnt=0#count the number of robots within the range
        avg_yaw=0#average yaw
        for i in range(0,ROBOT_NUM):#loop through all robots
            if(self.within_dis(i)):#if the robot is within the range
                if(self.within_dis(i) and i!=self.robot_no):#if the robot is not itself
                    ori_list=[self.ori_x[i],self.ori_y[i],self.ori_z[i],self.ori_w[i]]#get the orientation of the robot
                    (roll, pitch, yaw)=tf.transformations.euler_from_quaternion(ori_list)#convert the orientation to yaw
                    avg_yaw+=yaw
                    cnt+=1
        if(cnt!=0):
            avg_yaw/=cnt   
        print(avg_yaw)
        return avg_yaw
```

Since the orientation info stored in the `odom` topic is expressed in quaternio [x,y,z,w], we have to convert it to Euler
Angle to get the yaw angle. This can be done by calling `tf.transformations.euler_from_quaternion()`. 


#### self.cohesion()
 Cohesion means that robots should be moving towards their geographical center--which is calculated by first averaging the x and y coordinates of the neighboring robot and then substracting it from the robot's own location. The difference is actually a vector pointing from the robot to the center of the neighboring robots.
```python
    def cohesion(self):
        """return the desired yaw, pointing to the center of the robot"""
        #calculated by averaging the pos.x and pos.y of neighboring robots
        cnt=0#count the number of robots within the range
        avg_x=0#  average x
        avg_y=0#  average y
        avg_yaw=0#  average yaw
        for i in range(0,ROBOT_NUM):#loop through all robots
            if(self.within_dis(i)):#if the robot is within the range
                if(self.within_dis(i) and i!=self.robot_no):#if the robot is within the range and not itself
                    avg_x+=self.pos_x[i]
                    avg_y+=self.pos_y[i]
                    cnt+=1

        if(cnt!=0):
            avg_x=avg_x/cnt
            avg_y=avg_y/cnt
        #calculate the desired yaw
        avg_yaw=math.atan2(avg_y-self.y,avg_x-self.x)
        
        if(self.dis(avg_x,avg_y)>DIS_UPPER):#if the distance is too far
            return avg_yaw
        else:
            return 0
```
 In the above codes, after getting the `avg_x` and `avg_y`, we need to use inverse_tan `math.atan2()` to get the correct
 yaw angle information. Also, be advised that if the robots are too close to each other(smaller than the `DIS_UPPER`), we don't want them to gather--that might lead to an collison. Under that circumstance, the function will return 0.
 
####  self.seperation()
Seperation means that robots should disperse when they are too close to each other. `self.seperation()` applies the same logic of `self.cohesion`--but in an opposite direction. After calculating the averaged x and y, we should substract the current coordinate of the robot from the averaged x and y respectively.

```python
    def seperation(self):
        """return the desired yaw, pointing out of the center of the robot"""
        #calculated by averaging the pos.x and pos.y of neighboring robots
        cnt=0#number of robots within the range
        avg_x=0#average x of robots within the range
        avg_y=0#average y of robots within the range
        avg_yaw=0#average yaw of robots within the range

        for i in range(0,ROBOT_NUM):#loop through all robots
            if(self.within_dis(i)):#if the robot is within the range
                if(self.within_dis(i) and i!=self.robot_no):#if the robot is within the range and not itself
                    avg_x+=self.pos_x[i]
                    avg_y+=self.pos_y[i]
                    cnt+=1

        if(cnt!=0):
            avg_x=avg_x/cnt
            avg_y=avg_y/cnt

        avg_yaw=-self.cohesion()#calculate the desired yaw


        if(self.dis(avg_x,avg_y)<DIS_LOWER):
            return avg_yaw+math.pi
        else:
            return 0
```

Additionally, if the robots are too far from each other (larger than `DIS_LOWER`), we don't want them to seperate from each other, because they are already seperated enough.





## 2. Results and Evaluation

The robots were tested for alignment, cohesion, seperation, and obstacle avoiding. Videos were recorded.

### 2.1. Overall performance

According to the video, the robots behaved correctly for all of these movements. Also, when I combine all these behaviors together, the robots will also following the correct rules. They also have good grouping behavior--only focus on the local situation.

### 2.2. With higher angular velocity, the bigger chance to move in zig-zag?
In my codes, the angular velocity is a constant. However, when I increase its values, I find that the robots will behave correctly--but their movement is less smooth. In fact, when they try to go straight, they move in a zig-zag manner. 
This phenomenon is easy to understand--higher angular velocity requires the robot to spend more time in acceleration and slowing down, and the "zig-zag" will happen when its current yaw is close to the `target_yaw`--but still not close enough. Under this circumstance, the robot can't get aligned with the `target_yaw` because high speed will cause overhead easily.


## 3. Debugging & misc

### 3.1. How to assign executable permission to the .sh file?

Initially, I opened 3 terminals for letting 3 robots to execute the node file. Then I decided to use a .sh script (run3.sh) to do the trick. In this file, I have to designate the namespaces for different robots,and the same for the nodes and launch file we want them to run. The script is as follows

```bash
#!/bin/sh
ROS_NAMESPACE=robot_0 roslaunch simple_shape simple_shape.launch &
ROS_NAMESPACE=robot_1 roslaunch simple_shape simple_shape.launch &
ROS_NAMESPACE=robot_2 roslaunch simple_shape simple_shape.launch &
```
But when I used the command line `./run3.sh` to execute it, the terminal returns a permission denied error. It turns out that to make a shell script file executable, we have to first run `chmod +x filename.sh` to mark it as executable.

The same should be applied for the scripts killing the process.

```bash
#! /bin/bash

#chmod +x run4.sh
PID=$(ps -A | grep "python" | awk '{print $1}')
for pid in $PID
do
    kill  "$pid"
done
```

### 3.2. How to avoid the zig-zag motion?

As I mentioned before, when I increased the angular velocity, the robot will do the zig-zag motion because of the overhead. However, this issue is solved by setting up threshold. The robots are allowed to have slightly difference between its current yaw and the target yaw.





  
  
  

