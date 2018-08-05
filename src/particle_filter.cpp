/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

    num_particles = 100;
    is_initialized = true;
    weights = std::vector<double>(num_particles, 1.0);
    particles = std::vector<Particle>(num_particles);
    
    std::default_random_engine generator;
    std::normal_distribution<double> dist_x(x, std[0]);
    std::normal_distribution<double> dist_y(y, std[1]);
    std::normal_distribution<double> dist_theta(theta, std[2]);

    for (int i = 0; i < num_particles; i++) {
        particles[i].id = i;
        particles[i].x = dist_x(generator);
        particles[i].y = dist_y(generator);
        particles[i].theta = dist_theta(generator);
        particles[i].weight = 1.0 / num_particles;
    }
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
    
    std::default_random_engine generator;

    for (int i = 0; i < num_particles; i++) {
        double x_f;
        double y_f;
        if (fabs(yaw_rate) < 0.00001) {
            x_f = particles[i].x + velocity * delta_t * cos(particles[i].theta);
            y_f = particles[i].y + velocity * delta_t * sin(particles[i].theta);
        }
        else {
            x_f = particles[i].x + (velocity/yaw_rate) * ( sin(particles[i].theta + yaw_rate*delta_t) - sin(particles[i].theta));
            y_f = particles[i].y + (velocity/yaw_rate) * (-cos(particles[i].theta + yaw_rate*delta_t) + cos(particles[i].theta));
        }
        double theta_f = particles[i].theta + yaw_rate*delta_t;
        
        particles[i].x     = x_f;
        particles[i].y     = y_f;
        particles[i].theta = theta_f;
        
        std::normal_distribution<double> dist_x(particles[i].x, std_pos[0]);
        std::normal_distribution<double> dist_y(particles[i].y, std_pos[1]);
        std::normal_distribution<double> dist_theta(particles[i].theta, std_pos[2]);

        particles[i].x = dist_x(generator);
        particles[i].y = dist_y(generator);
        particles[i].theta = dist_theta(generator);
    }

}

//void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

 //   cout << "ParticleFilter::dataAssociation" << endl;

//}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html
    
    vector<vector<LandmarkObs>> particlePredictionForLandmark(num_particles);
    for (int z=0; z<num_particles; z++) {
        vector<LandmarkObs> predictionForLandmark(observations.size());
        
        for (int i=0; i<predictionForLandmark.size(); i++) {
            predictionForLandmark[i].x =  observations[i].x * cos(particles[z].theta)
                                        - observations[i].y * sin(particles[z].theta) + particles[z].x;
            predictionForLandmark[i].y =  observations[i].x * sin(particles[z].theta)
                                        + observations[i].y * cos(particles[z].theta) + particles[z].y;

            int idxNearestLandmark = -1;
            double distNearestLandmard2 = 999999;
            for (int k=0; k<map_landmarks.landmark_list.size(); k++) {
                double d2 = pow(predictionForLandmark[i].x - map_landmarks.landmark_list[k].x_f, 2) +
                            pow(predictionForLandmark[i].y - map_landmarks.landmark_list[k].y_f, 2);

                if (d2 < distNearestLandmard2) {
                    idxNearestLandmark = k;
                    distNearestLandmard2 = d2;
                }
            }
            predictionForLandmark[i].id = idxNearestLandmark;
        }
        particlePredictionForLandmark[z] = predictionForLandmark;
    }
    
    double totalPropability = 0.0;
    vector<double> probabilityParticle(num_particles);
    for (int z=0; z<num_particles; z++) {
        double p = 1;
        for (int i=0; i<observations.size(); i++) {
            double dx = particlePredictionForLandmark[z][i].x - map_landmarks.landmark_list[particlePredictionForLandmark[z][i].id].x_f;
            double dy = particlePredictionForLandmark[z][i].y - map_landmarks.landmark_list[particlePredictionForLandmark[z][i].id].y_f;
            double dx2 = dx*dx;
            double dy2 = dy*dy;
            p *= exp2(-0.5 * (dx2/std_landmark[0] + dy2/std_landmark[1]));
        }
        particles[z].weight = p;
        totalPropability += p;
    }
    
    for_each (particles.begin(), particles.end(), [totalPropability](Particle p){p.weight /= totalPropability;});
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
    
    vector<Particle> newParticles(num_particles);

    random_device rd;
    mt19937 gen(rd());
    vector<double> p(num_particles);
    for(int z=0; z<num_particles; z++) p[z] = particles[z].weight;
    discrete_distribution<int> d(p.begin(), p.end());
    for(int z=0; z<num_particles; z++) {
        newParticles[z] = particles[d(gen)];
    }
    particles = newParticles;
}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
    return particle;
 
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}


ostream& operator<<(ostream& os, const Particle& particle) {
    os << "particle: [" << particle.id << ", " << particle.x << ", " << particle.y << ", " << particle.theta << "]" << endl;
    return os;
}


