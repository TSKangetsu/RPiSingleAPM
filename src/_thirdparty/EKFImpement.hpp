#define Nsta 2
#define Mobs 6
#include "EKFImplement/src/TinyEKF.h"

class TotalEKF : public TinyEKF
{
public:
    TotalEKF()
    {
        this->setQ(0, 0, .0001);
        this->setR(0, 0, 5);
        this->setQ(1, 1, .0001);
        this->setR(1, 1, .5);
        this->setQ(2, 2, .08);
        this->setR(2, 2, .5);

        this->setQ(3, 3, .0001);
        this->setR(3, 3, .9);
        this->setQ(4, 4, .001);
        this->setR(4, 4, .5);
        this->setQ(5, 5, .005);
        this->setR(5, 5, .1);
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
        hx[2] = this->x[0];

        hx[3] = this->x[1];
        hx[4] = this->x[1];
        hx[5] = this->x[1];

        H[0][0] = 1;
        H[1][0] = 1;
        H[2][0] = 1;

        H[3][1] = 1;
        H[4][1] = 1;
        H[5][1] = 1;
    }
};