#ifndef COMPASS_ANGLES_H
#define COMPASS_ANGLES_H

#include <math.h>
#include <Eigen/Dense>

typedef Eigen::Vector2f vec2;

float angleTheta (vec2 v) {
    float angle = std::atan2(v[1], v[0]);
    angle = std::fmod(angle, 2 * M_PI);
    return (angle < 0 ? angle + 2 * M_PI : angle) / (2 * M_PI);
}

#endif //COMPASS_ANGLES_H
