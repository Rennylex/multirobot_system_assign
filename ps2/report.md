
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
  
   
  ### 1.2.2. Python file: simple_shape
  
  #### Structure
  
  Since the ultimate message we need to give to the robots is the direction--which indicate where the velocity vector should be pointing, my implementation is to let the robots get moving the whole time, but also have them constantly check
 if their `self.yaw` equals to the `target_yaw`.
 
 ```python
     def move_forward(self):
        """Function to move_forward for a given distance."""
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
            if self._close_obstacle:
                self.stop()
            else:
                self._cmd_pub.publish(twist_msg)

            # Sleep to keep the set publishing frequency.
            rate.sleep()

        # Traveled the required distance, stop.
        self.stop()
        
 ```
 
 
 
  In the above codes, we call the `calculate_distance()` function to acquire the error measured by Eucledean Distance. We also
  call the `error()` function to publish the error topic. These steps will be covered in the following sections.
  
  #### Subscribe topic
  
  In order to calculate the error, knowing the position of our robot is necessary--luckily, subscribing the Odometry message in
  nav_msgs can satisfy this demand.
  
  ```python
        # Setting up subscriber.
        self._odom_sub  = rospy.Subscriber(DEFAULT_ODOM_TOPIC, Odometry,  self._odom_callback, queue_size=1)
  ```
  
 The `self._odom_callback` is actually a callback function that's going to be called every time a message of this topic gets
 published. I define this callback function to acquire the position of the robot and assign the coordinates to instance
 variables `self.x` and `self.y`. Here the msg is actually a instanciated Odometry object.
 
 ```python
     def _odom_callback(self,msg):
        xx=msg.pose.pose.position.x #get x
        yy=msg.pose.pose.position.y #get y
        self.x=xx
        self.y=yy
 ```
  
 #### Publish topic
  

After having the position of the robot, we can calculate the error with the following function:

```python
   def calculate_distance(self,i,deg_inc,edge_len):
        print("Self x and y")
        print(self.x,self.y)
        
        ##getting the correct coordinate for each vertice
        self.correct_x+=edge_len*cos(deg_inc*i).real
        self.correct_y+=edge_len*sin(deg_inc*i).real #in case that the calculation returns a complex, we need to use .real here
        print("correct x and y")
        print(self.correct_x,self.correct_y)

        return sqrt((self.x-self.correct_x)**2+(self.y-self.correct_y)**2).real

```
The ideal position of each vertice (at one end of each edge) is determined by three variables: the length of each edge `edge_len`, the index of edge `i` and the rotation angle for each
turn `deg_inc`, as can be seen in line 4 and line 5 of the above codes.



Then, we define the function `error` to publish the error data in Float32 format.

```python
    def error(self,dist):
        error_msg=Float32()
        error_msg.data=dist
        self._error_pub.publish(error_msg)
```

## 2. Results and Evaluation

The robot was tested under 3 polygon shape: triangle, square and pentagon. For each polygon, video and erro at each vertice was recorded. 

    here is the screen shot for error...
![avatar](https://github.com/Rennylex/multirobot_system_assign/blob/main/3_er.png)
![avatar](https://github.com/Rennylex/multirobot_system_assign/blob/main/4_er.png)
![avatar](https://github.com/Rennylex/multirobot_system_assign/blob/main/5_er.png)

### 2.1. Overall performance

According to the video, the error for triangle and square is fairly acceptable. The same for the first 4 vertices of the polygon. However, when it comes to the 5th vertice, the error increase drastically. The reason for causing this immense
error might be the accumulation of errors, which ultimately leads to an fall down.

### 2.2. With higher velocity, comes the bigger error?
  The robot is also tested with different linear velocity and the results are recorded in the table as follows. The relationship between velocity and error can be concluded as: with a higher speed, the greater the error would be. To be more specific, under the same `edge_len`, the robot will travel a shorter distance if the velocity is high
  
|  Velocity (m/s)   | Error at Vertice 1 (m)  | Error at Vertice 2 (m) | Error at Vertice 3 (m) | Average Error (m)|
|  ----       | ----                  |                     ---- |         ---- | ----   |
| 0.2 | 0.0341282788309 |0.018326361481 |0.114480155123 |0.055644931811 |
| 0.8 | 0.311147796769 |0.260894322987 |0.42401567887 | 0.332019266208 |
| 1.4 | 0.652238059437 |0.625472860458 |0.33535084758 | 0.537687255825|
  
  The reason for this phenomenon is that the robot needs more time to accelerate and in order reach the high speed, and therefore more time to deaccelerate to stop. Take a look at the function `move_forward()`, the robot uses the travelling time to determine whether it has reached the destination, and the travelling time is calculated by assuming the robot travels in a constant speed. Therefore, the actual travel distance is always shorter than `edge_len`,
 and the higher the desired velocity is, the shorter the actual travel distance will be.
 
 ### 2.3 With higher velocity, the more likely to travel in curve?
 
 Another interesting phenomenon is that, when the speed is high, the robot is more likely to travel in curve. As can be seen in the following pictures:
    `pic1 & 2`
A plausible explanation would be, when the robot aims to accelarate to a higher speed, it requires larger force to drive
its wheel. And if there's a difference in the sleeping times of two wheels, the yaw angle of the robot will be changed 
dramatically, and result in a curve-like orbit.

## 3. Debugging & misc

### 3.1. How to make the robot turn clockwise?

My direction parameters `rotate_direction` is an integer, and I set it to -1 to let the robot turn in a clockwise direction--by multiplying it to the `deg_inc`. However, this didn't work out at first: instead of turning at each vertice, the robot just went straight ahead.

Therefore, something must be wrong with the `rotate_in_place()` function. After taking a closer look, I found that if I simply pass in a negative `rotation_angle`, the `duration` calculated by the `rotation_angle` will also be a negative number, which means the while loop will break immediately.

My solution for this is to set the `simple_shape.angular_velocity` as negative if `rotation_direction` equals to -1. Then,
I make sure the duration time is a possitive number. Codes are as follows.
```python
        def rotate_in_place(self, rotation_angle):
        """
        Rotate in place the robot of rotation_angle (rad) based on fixed velocity.
        Assumption: Counterclockwise rotation.
        """
        twist_msg = Twist()
        twist_msg.angular.z = self.angular_velocity
        duration = rotation_angle / twist_msg.angular.z
        if(duration<0): duration=-duration
        start_time = rospy.get_rostime()
        rate = rospy.Rate(FREQUENCY)
        while not rospy.is_shutdown():
            # Check if done
            if rospy.get_rostime() - start_time >= rospy.Duration(duration):
                break
            # Publish message.
            self._cmd_pub.publish(twist_msg)
            # Sleep to keep the set frequency.
            rate.sleep()
```





### 3.2. How to publish the data and read it from the terminal?

A useful command line for reading the topic will be `rostopic echo '/error'`. However, when I first use this method,
the process was killed because there's an error:
![avatar](https://github.com/Rennylex/multirobot_system_assign/blob/main/last_er.png)
I then printed the `error_msg` and found that the results are given in complex form. After double checking the program, I
found that it's because when calculating `correct_x` and `correct_y` using `cmath.sin` and `cmath.cos`, the results are also presented into a complex form in my case. The same thing happened when using `sqrt` to calculate the distance.

Therefore, I use `.real` to acquire the real part of these complex results, which solved the problem.




  
  
  

