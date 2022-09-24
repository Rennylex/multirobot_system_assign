# Testing

## 1. Method Explanation
  Overall speaking, the project aims to instruct the robot to travel along a designated normal polygon shape. This can actually be broken down
  into 3 steps:
  1.  tralvel the designated distance
  2.  rotate the required angle following a given direction(clockwise or counterclockwise).
  3.  repeat the above steps by designated times
  
  Therefore, we need to determine the parameters set that can actually allow the formentioned steps to be executed. 
  
  First, for the designated distance, it's actually the length of the edge of a polygon, we can call it as `edge_len`. 
  
  Second, for the required angle, it's actually the value of rotate_dirction*(Pi-inner_angle)--the rotate_direction can either be 1 or -1,
  and the inner_angle can be calculated with the number of edges `edge_num`. So the required basic parameters here are the `rotate_direction` and `edge_num`.
  
  Third, the value of designated times is actually `edge_num`. 
  
  
  ## 1.2. Code implementation
  
  ### 1.2.1. Launch File
  
  What we're gonna do is to implement the code. The node we will be developing is `simple_shape`, which is modified from `simple_motion`.
  
  First, we need to address the parameters `edge_num`, `edge_len`, `rotation_direction` in the launch file `simple_shape.launch`. The data type for
  these 3 parameters is `int`, which can be declared as follows:
  ```launch
    <param name="edge_num" type="int" value="4"/>
    <param name="edge_len" type="int" value="1"/>
    <param name="travel_dir" type="int" value="1"/>
  ```
    
  ### 1.2.2. Python file: simple_shape
  
  Move the object
  
  the movement of the object is the combination of moving forward and rotation, which can be realized by the function `move_forward(self, distance)`
  and `rotate_in_place(self, rotation_angle)`. The distance for moving is `edge_len`, and the angles for turning each time `deg_inc` can be calculated
  
  ```python
      deg_inc=(math.pi-(edge_num-2)*math.pi/edge_num)*travel_dir
  ```
  
  In order to make sure the robot move and turn for each edge, I used the while loop. The loop will be executed by `edge_num` times. Every time it is 
  executed, we will call move_forward() and rotate_in_place() by order.
  ```python
     
          while(i<edge_num):
            simple_shape.move_forward(edge_length)
            simple_shape.rotate_in_place(deg_inc) # rotate the robot everytime it reaches a vertice
            
            dist=simple_shape.calculate_distance(i,deg_inc,edge_length) #calculate the error distance
            simple_shape.error(dist) #publish the error distance
            print(dist) 
            i+=1 

  ```
  In the above codes, we call the `calculate_distance()` function to acquire the error measured by Eucledean Distance. We also
  call the `error()` function to publish the error topic. These steps will be covered in the following sections.
  
  Subscribe topic
  
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
  
 Publish topic
  
Calculating the distance...

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

### 2.1. Analysis

#### Overall performance

According to the video, the error for triangle and square is fairly acceptable. The same for the first 4 vertices of the polygon. However, when it comes to the 5th vertice, the error increase drastically. The reason for causing this immense
error might be the accumulation of errors, which ultimately leads to an fall down.

#### With higher velocity, comes the bigger error?
  The robot is also tested with different linear velocity and the results are recorded in the table as follows. The relationship between velocity and error can be concluded as: with a higher speed, the greater the error would be. To be more specific, under the same `edge_len`, the robot will travel a shorter distance if the velocity is high
  
|  Velocity (m/s)   | Error at Vertice 1 (m)  | Error at Vertice 2 (m) | Error at Vertice 3 (m) | Average Error (m)|
|  ----       | ----                  |                     ---- |         ---- | ----   |
| 0.2 | 0.0341282788309 |0.018326361481 |0.114480155123 |0.055644931811 |
| 0.8 | 0.311147796769 |0.260894322987 |0.42401567887 | 0.332019266208 |
| 1.4 | 0.652238059437 |0.625472860458 |0.33535084758 | 0.537687255825|
  
  The reason for this phenomenon is that the robot needs more time to accelerate and in order reach the high speed, and therefore more time to deaccelerate to stop. Take a look at the function `move_forward()`, the robot uses the travelling time to determine whether it has reached the destination, and the travelling time is calculated by assuming the robot travels in a constant speed. Therefore, the actual travel distance is always shorter than `edge_len`,
 and the higher the desired velocity is, the shorter the actual travel distance will be.
 
 #### With higher velocity, the more likely to travel in curve?
 
 Another interesting phenomenon is that, when the speed is high, the robot is more likely to travel in curve. As can be seen in the following pictures:
    `pic1 & 2`
A plausible explanation would be, when the robot aims to accelarate to a higher speed, it requires larger force to drive
its wheel. And if there's a difference in the sleeping times of two wheels, the yaw angle of the robot will be changed 
dramatically, and result in a curve-like orbit.

## 3. Debugging & misc

### How to make the robot turn clockwise?

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

### How to publish the data and read it from the terminal?

A useful command line for reading the topic will be `rostopic echo '/error'`. However, when I first use this method,
the process was killed because there's an error:
`error pic`
I then printed the `error_msg` and found that the results are given in complex form. After double checking the program, I
found that it's because when calculating `correct_x` and `correct_y` using `cmath.sin` and `cmath.cos`, the results are also presented into a complex form in my case. The same thing happened when using `sqrt` to calculate the distance.

Therefore, I use `.real` to acquire the real part of these complex results, which solved the problem.




  
  
  

