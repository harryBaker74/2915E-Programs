class gainSchedular
{
    double i;
    double f;
    double p;
    double k;

    public:
    /**
     * @brief Gain shcedular for use with single gains
     * @param initial Initial gain at the theoretical lowest movement
     * @param final Final gain for the theoretical largest movement, or the smallest movement to reach maximum velocity
     * @param sharpness The sharpness of the curve. A more sharp curve will gain amount quicker over a smaller distance
     * @param width Proportianl to the width of the curve
     */
    gainSchedular(double initial, double final, double sharpness, double width);

    /**
     * @brief Returns a gain amount for an input
     */
    double getGain(double input);
};