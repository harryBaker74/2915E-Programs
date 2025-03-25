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
        
        //Creating random number generator
        std::random_device rd;
        std::mt19937 gen(rd());
        
        //Cumalitve sum stuff
        std::vector<double> cumsum(particles.size());
        double total = 0;
        double squareTotal = 0;
        for(int i = 0; i < particles.size(); i++)
        {
            total += particles.at(i).weight;
            squareTotal += pow(particles.at(i).weight, 2);
            cumsum.at(i) = total;
        }

        //
        for(int i = 0; i < particles.size(); i++)
        {

        }
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

    void MCLUpdate(Pose robotPose, double std)
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


        //Calculate sensor global positions
        Point leftPosOffset = Point(  MCL_LEFT_X_OFFSET * cos(robotPose.heading) + MCL_LEFT_Y_OFFSET * sin(robotPose.heading), 
                                MCL_LEFT_X_OFFSET * -sin(robotPose.heading) + MCL_LEFT_Y_OFFSET * cos(robotPose.heading));
        Point frontPosOffset = Point( MCL_FRONT_X_OFFSET * cos(robotPose.heading) + MCL_FRONT_Y_OFFSET * sin(robotPose.heading), 
                                MCL_FRONT_X_OFFSET * -sin(robotPose.heading) + MCL_FRONT_Y_OFFSET * cos(robotPose.heading));
        Point rightPosOffset = Point( MCL_RIGHT_X_OFFSET * cos(robotPose.heading) + MCL_RIGHT_Y_OFFSET * sin(robotPose.heading), 
                                MCL_RIGHT_X_OFFSET * -sin(robotPose.heading) + MCL_RIGHT_Y_OFFSET * cos(robotPose.heading));

        double weightTotal = 0;

        for(int i = 0; i < particles.size(); i++)
        {
            Point leftPos = leftPosOffset + Point(particles.at(i).x, particles.at(i).y);
            Point frontPos = frontPosOffset + Point(particles.at(i).x, particles.at(i).y);
            Point rightPos = rightPosOffset + Point(particles.at(i).x, particles.at(i).y);


            //Calculate particles sensor readings
            double leftRead = 0;
            double frontRead = 0;
            double rightRead = 0;

                if((0 < robotPose.heading) && (robotPose.heading < M_PI_2))
                {
                    // printf("Case 1");

                    //Top right quadrant
                    //Then:
                    //90 < leftSensHead < 180, bottom, right
                    //0 < frontSensHead < 90, top, right
                    //-90 < rightSensHead < 0, top, left

                    //top, left
                    //top, right
                    //bottom right

                    // leftRead = fmin(fabs(((FIELD_WIDTH - leftPos.x) / sin(robotPose.rotation + M_PI_2))), fabs(((0 - leftPos.y) / cos(robotPose.rotation + M_PI_2))));
                    // frontRead = fmin(fabs(((FIELD_WIDTH - frontPos.x) / sin(robotPose.rotation))), fabs(((FIELD_HEIGHT - frontPos.y) / cos(robotPose.rotation))));
                    // rightRead = fmin(fabs(((0 - rightPos.x) / sin(robotPose.rotation - M_PI_2))), fabs(((FIELD_HEIGHT - rightPos.y) / cos(robotPose.rotation - M_PI_2))));

                    leftRead = fmin(fabs(((0 - leftPos.x) / sin(robotPose.rotation - M_PI_2))), fabs(((FIELD_HEIGHT - leftPos.y) / cos(robotPose.rotation - M_PI_2))));
                    frontRead = fmin(fabs(((FIELD_WIDTH - frontPos.x) / sin(robotPose.rotation))), fabs(((FIELD_HEIGHT - frontPos.y) / cos(robotPose.rotation))));
                    rightRead = fmin(fabs(((FIELD_WIDTH - rightPos.x) / sin(robotPose.rotation + M_PI_2))), fabs(((0 - rightPos.y) / cos(robotPose.rotation + M_PI_2))));
                }
                else if((M_PI_2 < robotPose.heading) && (robotPose.heading < M_PI))
                {
                    // printf("Case 2");

                    //Bottom right quadrant
                    //Then:
                    //180 < leftSensHead < 270, bottom, left
                    //90 < frontSensHead < 180, bottom, right
                    //0 < rightSensHead < 90, top, right

                    //top, right
                    //bottom, right
                    //bottom, left

                    // leftRead = fmin(fabs(((0 - leftPos.x) / sin(robotPose.rotation + M_PI_2))), fabs(((0 - leftPos.y) / cos(robotPose.rotation + M_PI_2))));
                    // frontRead = fmin(fabs(((FIELD_WIDTH - frontPos.x) / sin(robotPose.rotation))), fabs(((0 - frontPos.y) / cos(robotPose.rotation))));
                    // rightRead = fmin(fabs(((FIELD_WIDTH - rightPos.x) / sin(robotPose.rotation - M_PI_2))), fabs(((FIELD_HEIGHT - rightPos.y) / cos(robotPose.rotation - M_PI_2))));

                    leftRead = fmin(fabs(((FIELD_WIDTH - leftPos.x) / sin(robotPose.rotation - M_PI_2))), fabs(((FIELD_HEIGHT - leftPos.y) / cos(robotPose.rotation - M_PI_2))));
                    frontRead = fmin(fabs(((FIELD_WIDTH - frontPos.x) / sin(robotPose.rotation))), fabs(((0 - frontPos.y) / cos(robotPose.rotation))));
                    rightRead = fmin(fabs(((0 - rightPos.x) / sin(robotPose.rotation + M_PI_2))), fabs(((0 - rightPos.y) / cos(robotPose.rotation + M_PI_2))));
                }
                else if((-M_PI_2 > robotPose.heading) && (robotPose.heading > -M_PI))
                {
                    // printf("Case 3");

                    //Bottom left quadrant
                    //Then:
                    //270 < leftSensHead < 360, top, left
                    //180 < frontSensHead < 270, bottom, left
                    //90 < rightSensHead < 180, bottom, right

                    //bottom, right
                    //bottom, left
                    //Top, left

                    // leftRead = fmin(fabs(((0 - leftPos.x) / sin(robotPose.rotation + M_PI_2))), fabs(((FIELD_HEIGHT - leftPos.y) / cos(robotPose.rotation + M_PI_2))));
                    // frontRead = fmin(fabs(((0 - frontPos.x) / sin(robotPose.rotation))), fabs(((0 - frontPos.y) / cos(robotPose.rotation))));
                    // rightRead = fmin(fabs(((FIELD_WIDTH - rightPos.x) / sin(robotPose.rotation - M_PI_2))), fabs(((0 - rightPos.y) / cos(robotPose.rotation - M_PI_2))));

                    leftRead = fmin(fabs(((FIELD_WIDTH - leftPos.x) / sin(robotPose.rotation - M_PI_2))), fabs(((0 - leftPos.y) / cos(robotPose.rotation - M_PI_2))));
                    frontRead = fmin(fabs(((0 - frontPos.x) / sin(robotPose.rotation))), fabs(((0 - frontPos.y) / cos(robotPose.rotation))));
                    rightRead = fmin(fabs(((0 - rightPos.x) / sin(robotPose.rotation + M_PI_2))), fabs(((FIELD_HEIGHT - rightPos.y) / cos(robotPose.rotation + M_PI_2))));
                }
                else if((0 > robotPose.heading) && (robotPose.heading > -M_PI))
                {
                    // printf("Case 4");

                    //Top left quadrant
                    //Then:
                    //0 < leftSensHead < 90, top, right
                    //270 < frontSensHead < 360, top, left
                    //180 < rightSensHead < 270, bottom, left

                    //bottom left
                    //top left
                    //top right

                    // leftRead = fmin(fabs(((FIELD_WIDTH - leftPos.x) / sin(robotPose.rotation + M_PI_2))), fabs(((FIELD_HEIGHT - leftPos.y) / cos(robotPose.rotation + M_PI_2))));
                    // frontRead = fmin(fabs(((0 - frontPos.x) / sin(robotPose.rotation))), fabs(((FIELD_HEIGHT - frontPos.y) / cos(robotPose.rotation))));
                    // rightRead = fmin(fabs(((0 - rightPos.x) / sin(robotPose.rotation - M_PI_2))), fabs(((0 - rightPos.y) / cos(robotPose.rotation - M_PI_2))));

                    leftRead = fmin(fabs(((0 - leftPos.x) / sin(robotPose.rotation - M_PI_2))), fabs(((0 - leftPos.y) / cos(robotPose.rotation - M_PI_2))));
                    frontRead = fmin(fabs(((0 - frontPos.x) / sin(robotPose.rotation))), fabs(((FIELD_HEIGHT - frontPos.y) / cos(robotPose.rotation))));
                    rightRead = fmin(fabs(((FIELD_WIDTH - rightPos.x) / sin(robotPose.rotation + M_PI_2))), fabs(((FIELD_HEIGHT - rightPos.y) / cos(robotPose.rotation + M_PI_2))));
                }
                else if(robotPose.heading == 0)
                {
                    // printf("Case 5");

                    //Looking Straight up
                    leftRead = leftPos.x;
                    frontRead = FIELD_HEIGHT - frontPos.y;
                    rightRead = FIELD_WIDTH - rightPos.x;
                }
                else if(robotPose.heading == M_PI_2)
                {
                    // printf("Case 6");

                    //looking straight right
                    leftRead = FIELD_HEIGHT - leftPos.y;
                    frontRead = FIELD_WIDTH - frontPos.x;
                    rightRead = rightPos.y;
                }
                else if((robotPose.heading == M_PI) || (robotPose.heading == -M_PI))
                {
                    // printf("Case 7");

                    //Looking straight down
                    leftRead = FIELD_WIDTH - leftPos.x;
                    frontRead = frontPos.y;
                    rightRead = rightPos.x;
                }
                else if(robotPose.heading == -M_PI_2)
                {
                    // printf("Case 8");

                    //Looking stright left!!
                    leftRead = leftPos.y;
                    frontRead = frontPos.x;
                    rightRead = FIELD_HEIGHT - rightPos.y;
                }
                else
                {
                    // printf("Case 9");

                    //defualt case for debugging
                    leftRead = 0;
                    frontRead = 0;
                    rightRead = 0;
                }

            // printf("Left Read:%f, Front Read:%f, Right Read:%f\n", leftRead, frontRead, rightRead);
            // printf("Left Pos:(%.3f, %.3f), Front Pos:(%.3f, %.3f), Right Pos:(%.3f, %.3f)\n", leftPos.x, leftPos.y, frontPos.x, frontPos.y, rightPos.x, rightPos.y);
            
            double leftActual = distanceLeft.get_distance() / 10;
            double frontActual = distanceLeft.get_distance() / 10;
            double rightActual = distanceLeft.get_distance() / 10;

            //Calculate probability that each sensor on the particles is reprensentitive of the actual sensors reading
            double probLeft = exp(-0.5 * pow(((leftActual) - leftRead) / std, 2)) / (sqrt(2 * M_PI) * std);
            double probFront = exp(-0.5 * pow(((frontActual) - frontRead) / std, 2)) / (sqrt(2 * M_PI) * std);
            double probRight = exp(-0.5 * pow(((rightActual) - rightRead) / std, 2)) / (sqrt(2 * M_PI) * std);

            //Recalculating weight for particle
            particles.at(i).weight *= probLeft * probFront * probRight;
            particles.at(i).weight += 1e-300;

            // printf("ProbLeft:%f, ProbFront:%f, ProbRight:%f", probLeft, probFront, probRight);
                
            //Storing weight total
            weightTotal += particles.at(i).weight;
        }

        //Normalizing weights back down(or up) to 1 total
        for(int i = 0; i < particles.size(); i++)
        {
            particles.at(i).weight /= weightTotal;
            // printf("Prob:%f\n", particles.at(i).weight);
        }
    }

    Point MCLEstimate()
    {
        //Get the mean x and mean y values from each particle and set the robots position to those values
        double estimateX = 0;
        double estimateY = 0;
        for(int i = 0; i < particles.size(); i++)
        {
            estimateX += (particles.at(i).x * particles.at(i).weight);
            estimateY += (particles.at(i).y * particles.at(i).weight);
        }

        return Point(estimateX, estimateY);
    }
}