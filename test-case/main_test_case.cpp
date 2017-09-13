#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include <math.h>
#include "particle_filter.h"

using namespace std;


int main()
{
  //Set up parameters here
  double delta_t = 0.1; // Time elapsed between measurements [sec]
  double sensor_range = 5; // Sensor range [m]

  double sigma_pos [3] = {0.3, 0.3, 0.01}; // GPS measurement uncertainty [x [m], y [m], theta [rad]]
  double sigma_landmark [2] = {0.3, 0.3}; // Landmark measurement uncertainty [x [m], y [m]]

  // Read map data
  Map map;
  if (!read_map_data("../data/map_data.txt", map)) {
	  cout << "Error: Could not open map file" << endl;
	  return -1;
  }

  // Create particle filter
  ParticleFilter pf;
  bool terminate = false;
  
  while(!terminate) {
    if (!pf.initialized()) {

    // Sense noisy position data from the simulator
		double sense_x = 2;
		double sense_y = 2;
		double sense_theta = M_PI/2;

		pf.init(sense_x, sense_y, sense_theta, sigma_pos);
	  }
	  else {
		// Predict the vehicle's next state from previous (noiseless control) data.
	  	double previous_velocity = 1.5;
		  double previous_yawrate = 2;

		  //pf.prediction(delta_t, sigma_pos, previous_velocity, previous_yawrate);
		  
		  vector<LandmarkObs> noisy_observations;
		  LandmarkObs obs_ref1, obs_ref2, obs_ref3;
		  obs_ref1.x = 2;
		  obs_ref1.y = 2;
		  obs_ref2.x = 3;
		  obs_ref2.y = -2;
		  obs_ref3.x = 0;
		  obs_ref3.y = -4;
		  noisy_observations.push_back(obs_ref1);
		  noisy_observations.push_back(obs_ref2);
		  noisy_observations.push_back(obs_ref3);
      // Update the weights and resample
	    pf.updateWeights(sensor_range, sigma_landmark, noisy_observations, map);
	    pf.resample();
	    terminate = true;
	  }
  }
  
}























































































