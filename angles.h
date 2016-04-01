#ifndef COMPASS_ANGLES_H
#define COMPASS_ANGLES_H

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <math.h>

typedef Eigen::Vector2f vec2;

float angleBetween (vec2 a, vec2 b) {
    float theta = a.dot(b) / (a.norm() * b.norm());
    theta = std::min(1.0f, std::max(-1.0f, theta));
    return std::acos(theta);
}

float angleBetweenWithDirection (vec2 a, vec2 aDirection, vec2 b) {
    float simpleAngle = angleBetween(a, b);
    vec2 linearDirection = (b - a).normalized();

    if (aDirection.dot(linearDirection) >= 0) {
        return simpleAngle;
    } else {
        return 2 * M_PI - simpleAngle;
    }
}

#endif //COMPASS_ANGLES_H
