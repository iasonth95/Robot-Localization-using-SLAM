#ifndef CONBEAR_H
#define CONBEAR_H

class ConBear {
public:
    // Function to normalize bearing angle to [-pi, pi]
    static double conBear(double oldBear);
};

#endif // CONBEAR_H