#include "../include/main.h"
#include "../include/harryLibHeader/globals.h"
#include "../include/harryLibHeader/odom.hpp"
#include <random>


namespace Odometery
{
    pros::Distance distanceLeft(DIST_LEFT); 
    pros::Distance distanceFront(DIST_FRONT);
    pros::Distance distanceRight(DIST_RIGHT);
    std::vector<Particle> particles;


    Particle::Particle()
    {
        this->x = 0;
        this->y = 0;
        this->weight = 0;
    }
    Particle::Particle(double x, double y, double weight)
    {
        this->x = x;
        this->y = y;
        this->weight = weight;
    }



    std::vector<double> getDistSensReadings()
    {
        return {double(distanceLeft.get_distance() / 10), double(distanceFront.get_distance() / 10), double(distanceRight.get_distance() / 10)};
    }

    double getDistSensRead(int portConstant)
    {
        switch (portConstant)
        {
            case DIST_LEFT: return distanceLeft.get_distance() / 10;
            case DIST_FRONT: return distanceFront.get_distance() / 10;
            case DIST_RIGHT: return distanceRight.get_distance() / 10;

            default: return -1;
        }
    }



    void MCLGenerateParticles(Point meanPoint, double std, double particleAmount)
    {
        //Generate particles with gaussian distribution on x and y

        //Creating random numebr generator
        std::random_device rd;
        std::mt19937 gen(rd());

        //Creating distributions
        std::normal_distribution<double> xDistro(meanPoint.x, std);
        std::normal_distribution<double> yDistro(meanPoint.y, std);

        //Creating particles
        double weight = 1 / particleAmount;//Normalizing weights for each particle
        particles = {Particle(xDistro(gen), yDistro(gen), weight)};
        for(int i = 1; i <= particleAmount; i++)
            particles.push_back(Particle(xDistro(gen), yDistro(gen), weight));
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

        //Creating random numebr generator
        std::random_device rd;
        std::mt19937 gen(rd());

        //Creating distributions
        std::normal_distribution<double> xDistro(deltaPos.x, std.first);
        std::normal_distribution<double> yDistro(deltaPos.y, std.second);

        //Moving Each point
        for(int i = 0; i < particles.size(); i++)
        {
            particles.at(i).x += xDistro(gen);
            particles.at(i).y += yDistro(gen);
        }
    }

    void MCLUpdate(double std)
    {
        //Loop through every particle, calculate the reading each particle should have for all three sensors
        //Multiply the weight of that particle by the probabilty that its sensor matches the actual sensor reading, for all three sensors
        //double prob = exp(-0.5 * pow((sensorReading - particleReading) / std, 2)) / (sqrt(2 * M_PI) * std);
        //particleWeight *= probLeft * probBack * probRight;
        //Also add 1e-300 to each particle to prevent numerical underflow

        //Divide each particles weight by the sum of all the particles weights to normalize the total weight to 1


        //Wall choosing
        //if looking top right(0deg to 90deg), only look at top and right wall
        //if looking botome right(90deg to 180deg), only look at bottom and right wall
        //etc
        //If looking perfectly 0, 90, 180, 270, then only 1 wall

        //Distance caluclation
        //Walls at x=0, x=Width, y=0, y = Height
        //Distance = (wallx - sensorx)/sin(sensorHeading)
        //Distance = (wally - sensory)/cos(sensorHeading)

    }

    void MCLEstimate()
    {
        //Get the mean x and mean y values from each particle and set the robots position to those values
    }
}