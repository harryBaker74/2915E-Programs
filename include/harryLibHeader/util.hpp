#pragma once

#define CENTIDEGREES_TO_ENCODER 1/100
#define MAX_RPMPS 679.58858174586268543615659672366

//1.988387815159469 m/s/s
//11.326476362431044757269276612061 r/s/s
//


int sign(double num);

double atan3(double y, double x);

double linearToCubed(double input, double maxInput, double k);

double boundAngle(double angle, bool raidians);

double inToCm(double inch);

double cmToIn(double centimeter);

double slew(double targetAmount, double currentAmount, double rateOfChange_ms, double timeStep_ms);

//Point Struct
struct Point
{
    double x;
    double y;

    Point(double x, double y);

    void set(double x, double y);
};

//Pose struct
    struct Pose
    {
        double x;
        double y;
        double heading;
        double rotation;

        public:
        //Constructor
        Pose(double x, double y, double heading);

        /**
         * @brief Function to set this pose to a deisred pose
         * 
         * @param x What to set the x value
         * @param y What to set the y value
         * @param heading What to set the heading
         */
        void set(double x, double y, double heading);

        /**
         * @brief Function to set this pose to a deisred pose
         * 
         * @param pose The pose to set this pose to
         */
        void set(Pose pose);
    };