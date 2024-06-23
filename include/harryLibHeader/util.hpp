#pragma once

int sign(double num);

double linearToCubed(double input, double maxInput, double k);

struct Point
{
    double x;
    double y;

    Point(double x, double y);

    void set(double x, double y);
};