# System Integration
This is the final project of the Udacity Self-Driving Car Engineer Nanodegree.

The project focuses on integrating different parts of the Car using ROS. We focus on building ROS nodes to implement waypoint updater, trafficlight detection and control. This means that we are creating a path for the car to follow, detecting the traffic light and deciding to stop or go depending on traffic light.

The architecture of the car system is as shown below.

![](./ros-architecture.png)

It has a perception, planning and control module. The interaction and the message architecture are shown in the picture. 

## Waypoint Updater
This node publishes the next 40 waypoints that are closest to the vehicle location. This node subscribes to /current_pose, /base_waypoints and /traffic_waypoint.
### Current Pose:
This gives the current position of the vehicle

### Base Waypoint:
These are the waypoints for the complete track. These waypoints are received once and stored. We then use the next 40 waypoints from the current pose and visualize it using the green dots.

### Traffic Waypoints:
This merely gives the index in the base_waypoint list which is close to a traffic light. We use this to calculate the distance of the vehicle from a traffic light.

## Waypoint_Updater
We use pose_cb function to receive the current position of the car. We then determine which is the waypoint which is closest to the car and ensure that we consider the waypoint ahead of the car in the get_closest_waypoint_idx function. We determine if we wish to decelerate and stop for a Traffic light red and close or linear velocity drop if the traffic light is red and far away or to continue with current velocity if the light is green or no light is present using the decelerate_waypoints function. This function is called in the generate_lane function which generates the lanes for driving between the closest and the farthest waypoints and then calls the appropriate function to decelerate or stop. Finally the waypoints are published in the final_waypoints topic


## DBW node and Twist Controller:
The Drive by wire and twist work in conjunction to control the movement of the car. This is responsible for the vehicle steering, acceleration and braking. This node subscribes to the dbw_enabled, current_velocity and twist_cmd topics. It publishes steering_cmd, brake_cmd and throttle_cmd.

This uses the Controller class defined in the twist_controller. The throttle is based on current velocity and target velocity and uses a PID controller provided in the workspace. The Kp, Ki, Kd values are chosen based on some trail and error.
        kp = 0.3
        ki = 0.1
        kd = 0.0
        
For the steering angle control, a Yaw Controller is also provided by the workspace and it uses linear velocity and angular velocity.

The brake command is calculated using some physics since multiple vehicle values are provided in this node.

            decel = max(vel_error, self.decel_limit)
            brake = abs(decel) * self.vehicle_mass * self.wheel_radius
            
## TL Detector Node:
The TL detector node subscribes to the base_waypoints, current_pose, traffic_lights, img_color. This publishes the traffic_waypoint message. The closest traffic waypoint is calculated using the shortest distance. In the TL detector node the process_traffic)lights function performs the majority of the work where the waypoint position of each traffic light is stored and compared to the current base_waypoint based on the current position of the vehicle. If a traffic light is observed, the waypoint of the light and the stopposition (which is 2 waypoints before the light position) is calculated. That is transmitted on the /traffic_waypoint. 
The image_cb detects if the light is red or green from camera images.

The video of the final car drive is below:
[video](./Red_light.mov)
