# **Kidnapped Vehicle Projectr**

The goals / steps of this project are the following:
* Predict position and orientation to a vehicle;
* Receive distance between vehicle and lankmarks vehicle and update its state using a [Particle Filter](https://en.wikipedia.org/wiki/Particle_filter).

### Inplementation Details

There are 3 main steps in this code:
1. Initialization;
2. Prediction;
3. Update state;

All these 3 steps are in the file **src/particle_filter.cpp**. The *initialization* can be found in method **init**. There, the number of particles is set in **100** and equal weights for each particle is given. Random gaussian generators are used for sample (x,y) and theta variables for each particle filter.

The **prediction** method predicts the next position (x,y) and theta for a given elapse time (since last update). In essence, the movement model is applied to each particle filter considering gaussian noise.

The last step takes place in **updateWeights** and **resample** methods. The ideia is to verify the probability for each particle filter given a set of observations (distance between vehicle and landmarks). In **resample**, particles with low weight (low probability) are rejected and replaced by particles with high weights (high probability).


