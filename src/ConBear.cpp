#include "ConBear.h"
#include <cmath>

double ConBear::conBear(double oldBear)
{
    while (oldBear < -M_PI)
    {
        oldBear += 2 * M_PI;
    }
    while (oldBear > M_PI)
    {
        oldBear -= 2 * M_PI;
    }
    return oldBear;
}