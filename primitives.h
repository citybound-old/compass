//
// Created by Anselm Eickhoff on 28/01/16.
//

#ifndef COMPASS_PRIMITIVES_H
#define COMPASS_PRIMITIVES_H

struct vec2 {
    float x;
    float y;
};

enum curveOrientation {LEFTWARDS, RIGHTWARDS};

const float thickness;

class Segment {
    float _lengthAndStraightInfo;

    // TODO: check this
    curveOrientation orientation () {
        if (crossz(direction, end - start) > 0) {
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
        :start(start), direction((end - start).normalize()), end(end)
    {
        _lengthAndStraightInfo = (end - start).length();
    }

    // CircleSegment
    Segment (vec2 start, vec2 direction, vec2 end)
        :start(start), direction(direction), end(end)
    {
        bool isStraight = (end - start).normalize() == direction;
        if (isStraight()) {
            _lengthAndStraightInfo = (end - start).length();
        } else {

        }
    }

    float length() {
        return abs(_lengthAndStraightInfo);
    }

    bool isStraight() {
        return _lengthAndStraightInfo > 0;
    }

    vec2 radialCenter () {

    }

    float radius () {
        return (radialCenter() - start).length();
    }

    vec2 midpoint () {
        auto linearMidpoint = (end + start) / 2;
        if (isStraight()) return linearMidPoint;
        else {
            center + ((linearMidpoint - center).normalize() * radius());
        }
    }

    vec2 endDirection () {
        if (isStraight()) return direction;
            // TODO check this
        else return (orientation() == LEFTWARDS ? 1 : -1) * (radialCenter() - end).perpendicular().normalize();
    }

    vec2 directionOf (offset) {

    }

    Segment reverse() {}
};

class Circle {
public:
    vec2 center;
    float radius;

    Circle (center, radius) : center(center), radius(radius) {};

    bool contains(vec2 point) {
        return (center - point).length <= this->radius + thickness/2;
    }
};

class Line {
    vec2 middle;
    vec2 direction;

    Line (middle, direction) : middle(middle), direction(direction) {};
};

class Ray {
    vec2 start;
    vec2 direction;

    Ray (start, direction) : start(start), direction(direction) {};
};

struct Intersection {
    float u;
    float v;
    vec2 position;

    Intersection (float u, float v, vec2 position)
            : u(u), v(v), position(position) {};

    Intersection swapped () {
        return Intersection(v, u, position);
    }
};

#endif //COMPASS_PRIMITIVES_H
