# CarND-Kidnapped-Vehicle-Project
This repository contains C++ code for implementation of Particle Filter to localize a vehicle kidnapped in a closed environment. This task was implemented to partially fulfill Term-II goals of Udacity's self driving car nanodegree program.

## Background

A critical module in the working of a self driving car is the localizer or localization module. Localization can be defined as predicting the location of car with an accuracy of 3-10 cm. This location is in reference to a global map of the locality in which the self driving car is either stationery or moving.

One way to localize a car is by using Global Positioning System (GPS), which makes use of triangulation to predict the position of an object detected by multiple satellites. But GPS doesn't always provide high accuracy data. For e.g.: In case of strong GPS signal, the accuracy in location is in the range of 1-3 m. Whereas in the case of a weak GPS signal, the accuracy drops to a range of 10-50 m. Hence the use of only GPS is not desirable and reliable.

To achieve an accuracy of 3-10 cm, sensor information from Laser sensor (LIDAR) and/or Radial distance and angle sensor (RADAR) is used and fused together using a Particle Filters. This process is demonstrated in the following sections.


## Localization Algorithm:

Localization in case of self driving car makes use of GPS, range sensors, landmark information and a global map based on the following algorithm given below:

  1. Construct a global map of different areas in which the self driving car is to be deployed. This map contains information about different location of 'landmarks'. Landmarks are nothing but major features present in the locality, which are not subject to change for a longer period of time. Examples can be buildings, signal posts, intersections, etc. These landmarks are used in later steps to predict the relative location of car. These maps need to be updated often so as to add new features and update the locations of existing features.
  
  2. Once a map is constructed, GPS sensor installed inside car is used to predict the locality of in which the car is present. On basis of this locality, only a portion of global map is selected to avoid a large number of real time calculations as the algorithms must run in real time. As stated earlier, GPS sensor provides noisy measurement and hence cannot be used alone for localization.
  
  3. LIDAR and/or RADAR sensor installed on car then measure the distance between the car and the landmarks in the map. This helps in further pinning down location of car as it is now relative to landmarks in the global map constructed earlier. However, LIDAR and RADAR information is also not accurate and prone to noise. Hence, a sophisticated technique like Particle Filter is used.
  
  4. Particle Filter is used to combine the information gained from all above steps and predict the location of car with high accuracy of 3-10 cm.
  
The whole algorithm repeats at run time when the car is moving and new location of car is predicted.

