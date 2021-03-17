#define Nsta 1
#define Mobs 2
#include "EKFImplement/src/TinyEKF.h"

class AltitudeEKF : public TinyEKF
{
public:
    AltitudeEKF()
    {
        this->setQ(0, 0, .0005);

        this->setR(0, 0, .5);
        this->setR(1, 1, .02);
    }

protected:
    void model(double fx[Nsta], double F[Nsta][Nsta], double hx[Mobs], double H[Mobs][Nsta])
    {
        fx[0] = this->x[0];

        F[0][0] = 1;

        hx[0] = this->x[0];
        hx[1] = this->x[0];

        H[0][0] = 0.993;
        H[1][0] = 0.120131 * pow((1 - 2.2577e-7 * x[0]), 4.25588);
    }
};