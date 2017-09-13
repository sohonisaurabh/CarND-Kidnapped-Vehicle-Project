#include <random>
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
	num_particles = 10;
	default_random_engine gen;
	
	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]);
	// cout<<"Particles initialized are as follows: "<<endl;
	// particles.push_back(Particle {6, 4, 5, -M_PI/2});
	// cout<<"Particle: "<<6<<" - "<<particles[0].x<<" "<<particles[0].y<<" "<<particles[0].theta<<endl;
	
	int i;
	for (i = 0; i < num_particles; i++) {
	  Particle current_particle;
	  current_particle.id = i;
	  current_particle.x = dist_x(gen);
	  current_particle.y = dist_y(gen);
	  current_particle.theta = dist_theta(gen);
	  //current_particle.x = x;
	  //current_particle.y = y;
	  //current_particle.theta = theta;
	  current_particle.weight = 1.0;
	  
	  particles.push_back(current_particle);
	  weights.push_back(current_particle.weight);
	  // cout<<"Particle: "<<i+1<<" - "<<current_particle.x<<" "<<current_particle.y<<" "<<current_particle.theta<<endl;
	}
	is_initialized = true;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	
	default_random_engine gen;
	// cout<<"Particles predicted after prediction step are as follows: "<<endl;
	
	int i;
	for (i = 0; i < num_particles; i++) {
	  double particle_x = particles[i].x;
	  double particle_y = particles[i].y;
	  double particle_theta = particles[i].theta;
	 
	  double pred_x;
	  double pred_y;
	  double pred_theta;
	  //cout<<"Yaw rate is: "<<yaw_rate<<endl;
	  //cout<<"Fabs of yaw rate is: "<<fabs(yaw_rate)<<endl;
	  //Instead of a hard check of 0, adding a check for very low value
	  if (fabs(yaw_rate) < 0.0001) {
	     //cout<<"Yaw rate is zero, so entered IF"<<endl;
	    pred_x = particle_x + velocity * cos(particle_theta) * delta_t;
	    pred_y = particle_y + velocity * sin(particle_theta) * delta_t;
	    pred_theta = particle_theta;
	  } else {
	  	// cout<<"Yaw rate is not zero, so entered ELSE"<<endl;
	    pred_x = particle_x + (velocity/yaw_rate) * (sin(particle_theta + (yaw_rate * delta_t)) - sin(particle_theta));
	    pred_y = particle_y + (velocity/yaw_rate) * (cos(particle_theta) - cos(particle_theta + (yaw_rate * delta_t)));
	    pred_theta = particle_theta + (yaw_rate * delta_t);
	  }
	  
	  normal_distribution<double> dist_x(pred_x, std_pos[0]);
	  normal_distribution<double> dist_y(pred_y, std_pos[1]);
	  normal_distribution<double> dist_theta(pred_theta, std_pos[2]);
	  
	  particles[i].x = dist_x(gen);
	  particles[i].y = dist_y(gen);
	  particles[i].theta = dist_theta(gen);
	  //particles[i].x = pred_x;
	  //particles[i].y = pred_y;
	  //particles[i].theta = pred_theta;
	  // cout<<"Particle predicted: "<<" - "<<particles[i].x<<" "<<particles[i].y<<" "<<particles[i].theta<<endl;
	}
	

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations, double sensor_range) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
	
	/*Associate observations in map co-ordinates to predicted landmarks using nearest neighbor algorithm. Here, the number of observations may
    be less than the total number of landmarks as some of the landmarks may be outside the range of vehicle's sensor.*/
  
  int i, j;
  // cout<<"Data associations are as follows: "<<endl;
  for (i = 0; i < observations.size(); i++) {
    //Maximum distance can be square root of 2 times the range of sensor.
    double lowest_dist = sensor_range * sqrt(2);
    // cout<<"Max possible distance is: "<<lowest_dist<<endl;
    int closest_landmark_id = -1;
    double obs_x = observations[i].x;
    double obs_y = observations[i].y;
    
    for (j = 0; j < predicted.size(); j++) {
      double pred_x = predicted[j].x;
      double pred_y = predicted[j].y;
      int pred_id = predicted[j].id;
      double current_dist = dist(obs_x, obs_y, pred_x, pred_y);
      
      if (current_dist < lowest_dist) {
        lowest_dist = current_dist;
        closest_landmark_id = pred_id;
      }
    }
     // cout<<"Lowest distance is: "<<lowest_dist<<endl;
    observations[i].id = closest_landmark_id;
    // cout<<"Closest landmark ID is: "<<closest_landmark_id<<endl;
    // cout<<"Current Observation: "<<" - "<<obs_x<<" "<<obs_y<<" "<<endl;
    for (int k = 0; k < predicted.size(); k++) {
      LandmarkObs current_pred = predicted[k];
      if (current_pred.id == closest_landmark_id) {
        // cout<<"Current Association: "<<" - "<<current_pred.x<<" "<<current_pred.y<<endl;
      }
    }
  }  

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {
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
  
  // cout<<"Transformed observations are as follows: "<<endl;
  //return;
  int i, j;
  double weight_normalizer = 0.0;
  for (i = 0; i < num_particles; i++) {
    double particle_x = particles[i].x;
    double particle_y = particles[i].y;
    double particle_theta = particles[i].theta;
    
    //Vector containing observations transformed to map co-ordinates w.r.t. current particle.
    vector<LandmarkObs> transformed_observations;
    
    //Transform observations from vehicle's co-ordinates to map co-ordinates.
    for (j = 0; j < observations.size(); j++) {
      LandmarkObs transformed_obs;
      transformed_obs.id = j;
      transformed_obs.x = particle_x + (cos(particle_theta) * observations[j].x) - (sin(particle_theta) * observations[j].y);
      transformed_obs.y = particle_y + (sin(particle_theta) * observations[j].x) + (cos(particle_theta) * observations[j].y);
      transformed_observations.push_back(transformed_obs);
      // cout<<"Observation in vehicle co-ordinates: "<<j<<" - "<<observations[j].x<<" "<<observations[j].y<<" "<<endl;
      // cout<<"Transformed Observation: "<<j<<" - "<<transformed_obs.x<<" "<<transformed_obs.y<<" "<<endl;
    }
    
    // cout<<"Landmarks inside sensor's range are as follows: "<<endl;
    /*Filter map landmarks to keep only those which are in the sensor_range of current particle. Push them to predictions vector.*/
    vector<LandmarkObs> predicted_landmarks;
    for (j = 0; j < map_landmarks.landmark_list.size(); j++) {
      Map::single_landmark_s current_landmark = map_landmarks.landmark_list[j];
      if ((fabs((particle_x - current_landmark.x_f)) <= sensor_range) && (fabs((particle_y - current_landmark.y_f)) <= sensor_range)) {
        predicted_landmarks.push_back(LandmarkObs {current_landmark.id_i, current_landmark.x_f, current_landmark.y_f});
        // cout<<"Landmark: "<<j<<" - "<<current_landmark.x_f<<" "<<current_landmark.y_f<<" "<<endl;
      }
    }
    
    //Associate observations with predicted landmarks
    dataAssociation(predicted_landmarks, transformed_observations, sensor_range);
    
    //Reset the weight of particle to 1.
    particles[i].weight = 1.0;
    double sigma_x = std_landmark[0];
    double sigma_y = std_landmark[1];
    double sigma_x_2 = pow(sigma_x, 2);
    // cout<<"Sigmax2 is: "<<sigma_x_2<<endl;
    double sigma_y_2 = pow(sigma_y, 2);
    // cout<<"Sigmay2 is: "<<sigma_y_2<<endl;
    double normalizer = (1.0/(2.0 * M_PI * sigma_x * sigma_y));
    // cout<<"Normalizer is: "<<normalizer<<endl;
    int k, l;
    
    // cout<<"Multi-variate Gaussion probabilities are as follows: "<<endl;
    /*Calculate the weight of particle based on the multivariate Gaussian probability function*/
    for (k = 0; k < transformed_observations.size(); k++) {
      double trans_obs_x = transformed_observations[k].x;
      double trans_obs_y = transformed_observations[k].y;
      double trans_obs_id = transformed_observations[k].id;
      double multi_prob = 1.0;
      // cout<<"Observations to association tagging ID is: "<<current_obs.id<<endl;
      
      for (l = 0; l < predicted_landmarks.size(); l++) {
        double pred_landmark_x = predicted_landmarks[l].x;
        double pred_landmark_y = predicted_landmarks[l].y;
        double pred_landmark_id = predicted_landmarks[l].id;
        // cout<<"Association ID is: "<<trans_obs_id<<endl;
        
        if (trans_obs_id == pred_landmark_id) {
          multi_prob = normalizer * exp(-1.0 * ((pow((trans_obs_x - pred_landmark_x), 2)/(2.0 * sigma_x_2)) + (pow((trans_obs_y - pred_landmark_y), 2)/(2.0 * sigma_y_2))));
          // multi_prob = normalizer;
          /*if (multi_prob == 0) {
            cout<<"Current obs x: "<<current_obs.x<<endl;
            cout<<"Current obs y: "<<current_obs.y<<endl;
            cout<<"Current pred x: "<<current_pred.x<<endl;
            cout<<"Current pred y: "<<current_pred.y<<endl;
          }*/
          // cout<<"Current prob is: "<<multi_prob<<endl;
          particles[i].weight *= multi_prob;
        }
      }
    }
    // cout<<"Particle weight is: "<<particles[i].weight<<endl;
    weight_normalizer += particles[i].weight;
    // cout<<"Weight is: "<<current_particle.weight<<endl;
  }
   // cout<<"Weight normalizer is: "<<weight_normalizer<<endl;
  for (int i = 0; i < particles.size(); i++) {
    particles[i].weight /= weight_normalizer;
    weights[i] = particles[i].weight;
  }
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
	
	//return;
	vector<Particle> resampled_particles;
	
	// cout<<"Weights finally are: "<<endl;
	
	// Create a generator to be used for generating random particle index and beta value
	default_random_engine gen;
	
	//Generate random particle index
	uniform_int_distribution<int> particle_index(0, num_particles - 1);
	
	int current_index = particle_index(gen);
	// cout<<"Random index is: "<<current_index<<endl;
	
	double beta = 0.0;
	
	double max_weight_2 = 2.0 * *max_element(weights.begin(), weights.end());
	// cout<<"Max weight is: "<<max_weight_2;
	
	for (int i = 0; i < particles.size(); i++) {
	 uniform_real_distribution<double> random_weight(0.0, max_weight_2);
	 // cout<< "Random weight is: "<<random_weight(gen)<<endl;
	 beta += random_weight(gen);
	  
	  while (beta > weights[current_index]) {
	    beta -= weights[current_index];
	    current_index = (current_index + 1) % num_particles;
	  }
	  resampled_particles.push_back(particles[current_index]);
	}
	particles = resampled_particles;
}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	//Clear the previous associations
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

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
