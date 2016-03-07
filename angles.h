#ifndef COMPASS_ANGLES_H
#define COMPASS_ANGLES_H

#include <math.h>
#include <Eigen/Dense>

typedef Eigen::Vector2f vec2;

float angleTheta (vec2 v) {
    float angle = atan2(v[1], v[0]);
    return (angle < 0 ? angle + M_2_PI : angle) / M_2_PI;
}

#endif //COMPASS_ANGLES_H
