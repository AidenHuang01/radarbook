# radarbook
ROS driver for RadarBook 2.
ROS version: ROS Melodic
## Install:
```
mkdir catkin_ws
cd catkin_ws
mkdir src
cd src
git clone git@github.com:AidenHuang01/radarbook.git
cd ..
catkin_make
```

## Run
Connect the Radar to PC using ethernet cable and power up the radar.
```
rosrun radarbook <script>
```
