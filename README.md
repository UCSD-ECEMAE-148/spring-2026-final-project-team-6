# UCSD ECE/MAE 148 Team 6 Final Project 

![Autonomous Patrol Robot](Assets/Car_Image_1.png)
# Doom Patrol: An Autonomous Security Guard 
![UCSD Logo](Assets/UCSDLogo.png)





## Table of Contents

1. [Team Members](#team-members)
2. [Project Overview](#project-overview)
3. [Goals](#goals)
4. [Robot Hardware](#robot-hardware)
5. [Mechanical Design](#mechanical-design)
6. [Accomplishments](#accomplishments)
7. [Lessons Learned](#lessons-learned)
8. [Next Steps](#next-steps)
9. [Gantt Chart](#gantt-chart)
10. [Project Reconstruction](#project-reconstruction)


## Team Members: 

| Name | Major | Year |
|---------------------|---------------------|---------------------|
| Ahnaf H. | Mechanical Engineering| 2027 |
| Daniel O. | Computer Engineering | 2026 |
| Tracy T. | Mechanical Engineering | 2027 |
| Vincent S. | Computer Engineering | 2029 |

## Project Overview 
Thermal Patrol is an autonomous indoor security robot built ontop of UCSD's robocar platoform. The robot patrols a hallway using LiDAR based closed loop PID control, detects people using a thermal camera and triggers an alarm in response. 

## Goals 

### Original Goals
- Connect an ESP-32 based thermal camera with a Raspberri Pi using ROS2
- Detect people using the thermal camera, stop, and sound the alarm 
- Autonomously patrol hallways and avoid obsticles using a LiDAR

### Goals We Met 
- ESP-32 thermal camera integrated with ROS2 via a MQTT bridge
- LiDAR based hallway patroling 
- Obsticle detection and avoidance 
- Heat signature detection using a ML model running on the ESP-32
- Got SLAM to map a hallway although inconsistently

### Stretch Goals 
- full SLAM based mapping and autonomous navigation with waypoints 
- Web application to notify the user of an intruder 
- YOLO model integration to verify if a heat signature is infact a person 


## Robot Hardware 
| Component | Description |
|---------------------|---------------------|
| LD19 LiDAR | Used for autonomous navigation including obsticle avoidance and SLAM mapping |
| Thermal Camera | Low resolution thermal sensor used for detecting human heat signatures |
| ESP-32 | Reads thermal camera data, runs a custom ML model and publishes the results over MQTT |
| Raspberry Pi | Our main computer running ROS2 Jazzy |

## Mechanical Design 

<p float="left">
  <img src="Assets/CAD_Image_Front.png" height="300" />
  <img src="Assets/CAD_Image_Side.png" height="300" />
</p>



## Accomplishments 

### Thermal Detection & MQTT
- An ESP-32 reads ther thermal camera's measurments and runs an on device ML model to detect heat signatures 
- The ESP-32 publishes detections over MQTT through WiFi, allows for the reuse and scaling of the thermal detection module 
- A ROS2 bridge node subscribes to the ESP-32's MQTT topic and converts that signal into standard ROS2 topics 

### Autonomous Navigation 
#### PID Based Hallway Centering 
- Our LD19 LiDAR measures the distance from the left and right wall 
- The controller averages the two measurments to determine the center of the hallway 
- Using closed loop PID control the car continously aims for the center of the hallway 

#### Obsticle Avoidance 
- LiDAR detects objects infront of the robot 
- The system checks left and right to check for obsticles and turns in the aproriate direction 
- The robot waits until the path infront of it is clear again before turning back and returning to its original path 

#### SLAM Mapping 
- Got SLAM mapping to work but had trouble with odometry drift and issues pattern matching in a symetrical hallway 

## Challenges & Solutions

| System | Challenge | Solution |
|---|---|---|
| GPS | Unreliable signal and inaccurate readings due to a lack of corrections | Determined it was a hardware issue and pivoted to an indoor LiDAR based solution |
| Odometry | Significant drift in odometry readings made it unreliable and impacted SLAM mapping | Relied more heavily on LiDAR and implemented closed loop control based on LiDAR measurements |
| SLAM | Generated noisy and inconsistent maps; unable to consistently map a symmetrical hallway | Tried filtering out bad poses; next step would be RTAB-Map with an OAK-D camera for visual anchors |


<table align="center" width="90%">
  <tr>
    <td align="center"><img src="Assets/Odometry_Drift_Image.png" height="200" /></td>
    <td align="center"><img src="Assets/LiDAR_Issues.png" height="200" /></td>
  </tr>
  <tr>
    <td align="center">Display of odometry drift: red dots are the actual percived path and purple is our actual odometry data</td>
    <td align="center">Display of our SLAM map faliures:
     1. The mapping becomes off at an angle due to odometry drift 
     2. The end of the map shifts due to the symetry and difficulties pattern matching in that cast</td>
  </tr>
</table>

## Lessons Learned 
- **Start early and test frequently:** hardware and software issues come up frequently even the smallest things break
- **Expect things to break:** leave time for debugging and unexpected faliures 
- **Research before committing to a solution:** the SLAM toolbox was a reaonable initial choice but RTAB mapping would have better fitted for mapping symmetric hallways from the start. 
- **Pivot decisively:** when we were unable to get our GPS working after about a week pivoting to an indoor solution was the right call inorder to get things up and running. 

## Next Steps
If we had one more week we would have implemented... 

+ RTAB Mapping: 
    + Combine our existing LiDAR with an OAK-D camera to improve mapping in symmetrical hallways using visual landmarks 
    + Corrects for drift and current SLAM failure modes 
+ Web Application: 
    + Send notifications to users to create a more active autonomous ‘security guard’
    + Allow them to use the web app to control the robot, draw maps, start patrol ect 
+ Higher Resolution Thermal Camera:
    + Our current thermal camera has very low resolution allowing for misidentification of objects or animals as people
+ YOLO Model Verification: 
    + Add a camera and light to run a YOLO model to detect to verify that an object is infact a person and send a notification to the user 
 

## Gantt Chart 

<table>
  <tr>
    <td><img src="Assets/Original_Gantt.png" height="200" /></td>
    <td><img src="Assets/New_Gantt.png" height="200" /></td>
  </tr>
  <tr>
    <td align="center">Origional Gantt Chart</td>
    <td align="center">New Gantt Chart</td>
  </tr>
</table>

**Challenges & Changes :** 
- Around Week 8 or so we decided that it was necessary to expand the scope of our project from using GPS to LiDAR for indoor navigation. 
- We spent a lot more time debugging and had far more hardware issues than we thought for example debugging the ROS2 laps. 
- SLAM localization proved much harder to implement that we expected and forced us to focus on more alternative options. 

## Videos 




## Project Reconstruction 

Here are a few steps on how to recreate our project: 

### Prerequisites: 


