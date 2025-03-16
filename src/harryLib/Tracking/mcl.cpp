#include "../include/main.h"
#include "../include/harryLibHeader/globals.h"
#include "../include/harryLibHeader/odom.hpp"


namespace Odometery
{
    // Particle::Particle()
    // {
    //     this->x = 0;
    //     this->y = 0;
    //     this->weight = 0;
    // }
    // Particle::Particle(double x, double y, double weight)
    // {
    //     this->x = x;
    //     this->y = y;
    //     this->weight = weight;
    // }



    // std::vector<double> getDistSensReadings()
    // {
    //     return {double(distanceLeft.get_distance() / 10), double(distanceBack.get_distance() / 10), double(distanceRight.get_distance() / 10)};
    // }

    // double getDistSensRead(int portConstant)
    // {
    //     switch (portConstant)
    //     {
    //         case DIST_LEFT: distanceLeft.get_distance() / 10;
    //         case DIST_BACK: distanceBack.get_distance() / 10;
    //         case DIST_RIGHT: distanceRight.get_distance() / 10;
    //     }
    // }



    void MCLGenerateParticles(Point meanPoint, double std, double particleAmount)
    {
        //Generate particles with standard distribution on x and y
    }
    
    void MCLResample()
    {
        //Create cumulative sum of weights as an array
        //Generate N(number of particles) random numbers between 0 and 1
        //Check every number, and see what cumulative sum index it is closest below
        //bigger wiehgts mean bigger cumsum gaps mean more chance for an index to be selected
        //Resample all particles to the particles in these indexes
        //Set weights to equal for every particle again
    }


    void MCLPredict(Point deltaPos, std::pair<double, double> std)
    {
        // Move particles based on delta x and delta y
    }

    void MCLUpdate(double std)
    {
        //Loop through every particle, calculate the reading each particle should have for all three sensors
        //Multiply the weight of that particle by the probabilty that its sensor matches the actual sensor reading, for all three sensors
        //double prob = exp(-0.5 * pow((sensorReading - particleReading) / std, 2)) / (sqrt(2 * M_PI) * std);
        //particleWeight *= probLeft * probBack * probRight;
        //Also add 1e-300 to each particle to prevent numerical underflow

        //Divide each particles weight by the sum of all the particles weights to normalize the total weight to 1

    }

    void MCLEstimate()
    {
        //Get the mean x and mean y values from each particle and set the robots position to those values
    }
}