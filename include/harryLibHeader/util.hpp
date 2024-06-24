#pragma once

#define CENTIDEGREES_TO_ENCODER 1/100

int sign(double num);

double linearToCubed(double input, double maxInput, double k);

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