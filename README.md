## Kidnapped Vehicle Project
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

This repo contains the written code to complete the project **Kidnapped Vehicle Project** on Udacity Self-Driving Car Nanodegree. The goal is to predict vehicle position and orientation from measures distance to landmarks using a [Particle Filter](https://en.wikipedia.org/wiki/Particle_filter). 

Prerequisites
---
To run this project, it is necessary to have a **c++11** compiler and **cmake** (3.5 minimum version).

Installation
---
First, clone the repository:
```
git clone https://github.com/shohne/CarND-Kidnapped-Vehicle-Project.git
```
Change current directory:
```
cd CarND-Kidnapped-Vehicle-Project
```
Execute script *build.sh* to build the executable:
```
./build.sh
```
If everything was accordingly, there should be an executable file **particle_filter** on *build* directory.

Running the Application
---
First, start the executable:
```
./build/particle_filter
```
Now, the program shows in console:
```
Listening to port 4567
```
It is waiting for tcp/ip connection on port to 4567 to receive update measures from the car simulator.

Finally, run the simulator. An example for execution:

[pf.m4v](pf.m4v)

We could obtain a **error** as:

|Data|Error|
|----|-----|
|x   |0.120|
|y   |0.112|
|yaw |0.004|

An the simulator could execute the test in less than 60 seconds.

Implementation Details
---
Please visit the [report.md](report.md) for more information about the algorithm pipeline.
