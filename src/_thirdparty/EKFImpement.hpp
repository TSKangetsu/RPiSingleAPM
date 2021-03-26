#define Nsta 2
#define Mobs 4
#include "EKFImplement/src/TinyEKF.h"

class TotalEKF : public TinyEKF
{
public:
    TotalEKF()
    {
        this->setQ(0, 0, .02);
        this->setQ(1, 1, .2);
        this->setQ(2, 2, .02);
        this->setQ(3, 3, .2);

        this->setR(0, 0, .5);
        this->setR(1, 1, .8);
        this->setR(2, 2, .5);
        this->setR(3, 3, .8);
    }

protected:
    void model(double fx[Nsta], double F[Nsta][Nsta], double hx[Mobs], double H[Mobs][Nsta])
    {
        fx[0] = this->x[0];
        fx[1] = this->x[1];

        F[0][0] = 1;
        F[1][1] = 1;

        hx[0] = this->x[0];
        hx[1] = this->x[0];
        hx[2] = this->x[1];
        hx[3] = this->x[1];

        H[0][0] = 1;
        H[1][0] = 1;
        H[2][1] = 1;
        H[3][1] = 1;
    }
};