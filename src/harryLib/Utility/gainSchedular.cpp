#include "harryLibHeader/gainSchedular.hpp"
#include "main.h"

gainSchedular::gainSchedular(double initial, double final, double sharpness, double width)
{
    this->i = initial;
    this->f = final;
    this->p = sharpness;
    this->k = width;
}

double gainSchedular::getGain(double input)
{
    return ((-1 * (i - f) * (std::pow(fabs(input), p))) / (pow(fabs(input), p) + pow(k, p))) + i;
}