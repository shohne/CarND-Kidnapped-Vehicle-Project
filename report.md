# **Kidnapped Vehicle Projectr**

The goals / steps of this project are the following:
* Predict position and orientation to a vehicle;
* Receive distance between vehicle and lankmarks vehicle and update its state using a [Particle Filter](https://en.wikipedia.org/wiki/Particle_filter).

### Inplementation Details

There are 3 main steps in this code:
1. Initialization;
2. Prediction;
3. Update state;

All these 3 steps are in the file **src/particle_filter.cpp**. The *initialization* is on method **init**. There, we set the number of particle filters in **100** and equals weights for each particle. We set random gaussian generators used to sample (x,y) and theta variables for each particle filter.

The **prediction** method predicts the next position (x,y) and theta for a given elapse time (since last update). In essence, We apply the movement model to each particle filter considering gaussian noise.

The last step takes place in **updateWeights** and **resample** methods. The ideia is verify probability for each particle filter given a set of observations (distance between vehicle and landmarks). In **resample**, We discard particles with low weight (low probability) replacing it for particles with high weights (high probability).


