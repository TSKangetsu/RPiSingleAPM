#define Nsta 1
#define Mobs 3
#include "EKFImplement/src/TinyEKF.h"

class TotalEKF : public TinyEKF
{
public:
    TotalEKF()
    {
        this->setQ(0, 0, 20.0);
        this->setR(0, 0, 70.0);

        this->setQ(1, 1, 0.001);
        this->setR(1, 1, 45.0);

        this->setQ(2, 2, 20.0);
        this->setR(2, 2, 70.0);
    }

protected:
    void model(double fx[Nsta], double F[Nsta][Nsta], double hx[Mobs], double H[Mobs][Nsta])
    {
        fx[0] = this->x[0];

        F[0][0] = 1;

        hx[0] = this->x[0];
        hx[1] = this->x[0];
        hx[2] = this->x[0];

        H[0][0] = 1;
        H[1][0] = 1;
        H[2][0] = 1;
    }
};