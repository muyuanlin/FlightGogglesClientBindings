## unity_sim
#### Generate a simulated trajectory (e.g. a circle), publish through /trajectory topic;
#### Publish groundtruth poses of vehicle;
#### Request synthetic images from flight_goggles;
#### Publish simulated imu measurements.

## Usage
1. Download flight_goggles from https://github.mit.edu/FAST-LL-FlightGoggles/FlightGoggles/releases, extract files and place under Thirdparty/flight_goggles/

2. Install ```imusim``` package as instructed in the following page
```
https://github.com/martinling/imusim/blob/master/docs/installation.rst
```

2. Switch to ROS project folder, run
```
catkin_make
``` 

3. Change the access permissions to file "launch/simulation.launch" and "Thirdparty/flight_goggles/run_goggle.sh"
```
chmod +x ./src/folklift/launch/simulation.launch
chmod +x ./src/folklift/Thirdparty/flight_goggles/run_goggle.sh
```

4. Run the following command
```
source ./devel/setup.bash
roslaunch ./src/folklift/launch/simulation.launch
```
In another terminal tab
```
./src/folklift/Thirdparty/flight_goggles/run_goggle.sh
```
