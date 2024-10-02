#pragma once

#define CENTIDEGREES_TO_ENCODER 0.01
#define MAX_RPMPS 679.58858174586268543615659672366

//1.988387815159469 m/s/s
//11.326476362431044757269276612061 r/s/s
//


int sign(double num);

double atan3(double y, double x);

double linearToCubed(double input, double maxInput, double k);

double lineartoSquared(double input, double maxInput, double k);

double boundAngle(double angle, bool radians);

double inToCm(double inch);

double cmToIn(double centimeter);

double slew(double targetAmount, double currentAmount, double rateOfChange_ms, double timeStep_ms);

/**
 * @brief Returns the weighted average between 2 numbers
 * 
 * @param numA The first number, a weight of 1 would be fully favouring this number
 * @param numB The second number, a weight of 0 would be fully favouring this number
 * @param decimalWeight A number between {1, 0} that decides which number to favour more
 */
double getWeightedAverage(double numA, double numB, double decimalWeight);

//Point Struct
struct Point
{
    double x;
    double y;

    Point(double x, double y);

    void set(double x, double y);

    Point operator*(float scalar) const
    {
        return {x * scalar, y * scalar};
    }
    double operator*(Point point) const
    {
        return {x * point.x + y * point.y};
    }
    Point operator+(const Point& point) const
    {
        return {x + point.x, y + point.y};
    }
    Point operator-(const Point& point) const
    {
        return {x - point.x, y - point.y};
    }

    double cross(Point point)
    {
        return (x * point.y - y * point.x);
    }
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

double pointToPointDistance(Point p1, Point p2);