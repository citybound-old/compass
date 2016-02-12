//
// Created by Anselm Eickhoff on 28/01/16.
//

#ifndef COMPASS_PRIMITIVES_H
#define COMPASS_PRIMITIVES_H
#include <Eigen/Dense>

typedef Eigen::Vector2f vec2;

enum curveOrientation {LEFTWARDS, RIGHTWARDS};

const float thickness = 0.0;

class Segment {
    float _lengthAndStraightInfo;

    // TODO: check this
    curveOrientation orientation () {
        if (direction.cross2(end - start) > 0) {
            return LEFTWARDS;
        } else {
            return RIGHTWARDS;
        }
    }

public:
    vec2 start;
    vec2 end;
    vec2 direction;

    // LineSegment
    Segment (vec2 start, vec2 end)
        :start(start), direction((end - start).normalized()), end(end)
    {
        _lengthAndStraightInfo = (end - start).norm();
    }

    // CircleSegment
    Segment (vec2 start, vec2 direction, vec2 end)
        :start(start), direction(direction), end(end)
    {
        bool isStraight = (end - start).normalized() == direction;
        if (isStraight) {
            _lengthAndStraightInfo = (end - start).norm();
        } else {

        }
    }

    float norm() {
        return std::abs(_lengthAndStraightInfo);
    }

    bool isStraight() {
        return _lengthAndStraightInfo > 0;
    }

    vec2 radialCenter () {

    }

    float radius () {
        return (radialCenter() - start).norm();
    }

    vec2 midpoint () {
        auto linearMidpoint = (end + start) / 2;
        if (isStraight()) return linearMidpoint;
        else {
            //center + ((linearMidpoint - center).normalized() * radius());
        }
    }

    vec2 endDirection () {
        if (isStraight()) return direction;
            // TODO check this
        else return (orientation() == LEFTWARDS ? 1 : -1) * (radialCenter() - end).unitOrthogonal();
    }

    vec2 directionOf (float offset) {

    }

    Segment reverse() {}
};

class Circle {
public:
    vec2 center;
    float radius;

    Circle (vec2 center, float radius) : center(center), radius(radius) {};

    bool contains(vec2 point) {
        return (center - point).norm() <= this->radius + thickness/2;
    }
};

class Line {
    vec2 middle;
    vec2 direction;

    Line (vec2 middle, vec2 direction) : middle(middle), direction(direction) {};
};

class Ray {
    vec2 start;
    vec2 direction;

    Ray (vec2 start, vec2 direction) : start(start), direction(direction) {};
};

#endif //COMPASS_PRIMITIVES_H
