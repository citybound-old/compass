//
// Created by Anselm Eickhoff on 12/02/16.
//

#ifndef COMPASS_INTERSECTIONS_H
#define COMPASS_INTERSECTIONS_H

#include "primitives.h"
#include "at-most.h"
#include "angles.h"
#include "lzy/lzy.h"

using namespace lzy;

const float ROUGH_TOLERANCE = 0.0000001;

template<typename N1, typename N2, typename N3>
bool roughlyEqual(N1 a, N2 b, N3 tolerance) {
    return std::abs(b - a) < tolerance;
}

template <typename N3>
bool roughlyEqual(vec2 a, vec2 b, N3 tolerance) {
    return (b - a).norm() < tolerance;
}

template<typename N1, typename N2>
bool roughlyEqual(N1 a, N2 b) {
    return roughlyEqual(a, b, ROUGH_TOLERANCE);
}

float pointToLineDistance (vec2 point, vec2 start, vec2 direction) {
    return std::abs((point - start).dot(direction.unitOrthogonal()));
}

struct Intersection {
    float u;
    float v;
    vec2 position;

    Intersection (float u, float v, vec2 position)
            : u(u), v(v), position(position) {};

    Intersection (Intersection&& other)
            : u(other.u), v(other.v), position(std::move(other.position)) {};

    Intersection (const Intersection&& other)
            : u(other.u), v(other.v), position(std::move(other.position)) {};

    Intersection swapped () const {
        return Intersection(v, u, position);
    }
};

AtMost<2, Intersection> intersect (Ray a, Ray b) {
    float determinant = b.direction[0] * a.direction[1] - b.direction[1] * a.direction[0];
    bool isParallel = roughlyEqual(determinant, 0);

    if (isParallel) {
        bool sameStart = roughlyEqual(a.start, b.start, thickness);
        if (sameStart) return {Intersection(0, 0, a.start)};

        bool facingOpposite = !roughlyEqual(a.direction, b.direction);
        bool tooFarApart = pointToLineDistance(a.start, b.start, b.direction) > thickness/2;
        if (facingOpposite || tooFarApart) return {};

        // a contains b or b contains a
        if (roughlyEqual(a.direction, (b.start - a.start).normalized()))
            return {Intersection(0, 0, b.start)}; // TODO: u or v wrong here!!
        else
            return {Intersection(0, 0, a.start)}; // TODO: u or v wrong here!!
    }

    auto dx = b.start[0] - a.start[0];
    auto dy = b.start[1] - a.start[1];

    auto u = (dy * b.direction[0] - dx * b.direction[1]) / determinant;
    auto v = (dy * a.direction[0] - dx * a.direction[1]) / determinant;

    bool notOnRay = u < -thickness/2 || v < -thickness/2;
    if (notOnRay) return {};

    u = std::max(u, 0.0f);
    v = std::max(v, 0.0f);

    return {Intersection(u, v, a.start + u * a.direction)};
}

AtMost<2, Intersection> intersect (Line a, Line b) {
    float determinant = b.direction[0] * a.direction[1] - b.direction[1] * a.direction[0];
    bool isParallel = roughlyEqual(determinant, 0);

    if (isParallel) {
        bool tooFarApart = pointToLineDistance(a.middle, b.middle, b.direction) > thickness/2;
        if (tooFarApart) return {};

        // a == b
        return {Intersection(0, 0, a.middle)};
    }

    auto dx = b.middle[0] - a.middle[0];
    auto dy = b.middle[1] - a.middle[1];

    auto u = (dy * b.direction[0] - dx * b.direction[1]) / determinant;
    auto v = (dy * a.direction[0] - dx * a.direction[1]) / determinant;

    return {Intersection(u, v, a.middle + u * a.direction)};
}

AtMost<2, Intersection> intersect (Ray a, Line b) {
    float determinant = b.direction[0] * a.direction[1] - b.direction[1] * a.direction[0];
    bool isParallel = roughlyEqual(determinant, 0);

    if (isParallel) {
        bool tooFarApart = pointToLineDistance(a.start, b.middle, b.direction) > thickness/2;
        if (tooFarApart) return {};

        // b contains a
        return {Intersection(0, 0, a.start)}; // TODO: u or v wrong here!!
    }

    auto dx = b.middle[0] - a.start[0];
    auto dy = b.middle[1] - a.start[1];

    auto u = (dy * b.direction[0] - dx * b.direction[1]) / determinant;
    auto v = (dy * a.direction[0] - dx * a.direction[1]) / determinant;

    bool notOnRay = u < -thickness/2;
    if (notOnRay) return {};

    u = std::max(u, 0.0f);

    return {Intersection(u, v, a.start + u * a.direction)};
}
AtMost<2, Intersection> intersect (Line a, Ray b) {
    return from(intersect(b, a)) >> map(&Intersection::swapped) >> to<AtMost<2, Intersection>>();
}

AtMost<2, Intersection> intersect (Ray a, Segment b) {
    if (b.isStraight()) {
        auto potentials = from(intersect(a, Ray(b.start, b.direction))) >> (
            filter([&](const Intersection& i) {return i.v <= b.length() + thickness/2;})
            | map([](const Intersection& i) {return Intersection(i.u, std::min(i.v, 1.0f), i.position);})
        ) >> to<AtMost<2, Intersection>>();

        return potentials;
    } else {

    }
}

AtMost<2, Intersection> intersect (Segment a, Ray b) {
    return from(intersect(b, a)) >> map(&Intersection::swapped) >> to<AtMost<2, Intersection>>();
}

AtMost<2, Intersection> intersect (Segment a, Segment b) {

    if (a.isStraight() && b.isStraight()) {

        auto da = a.end - a.start;
        auto db = b.end - b.start;
        auto orientation = a.direction.cross2(b.direction);

        bool isParallel = roughlyEqual(orientation, 0);
        if (isParallel) {
            bool tooFarApart = pointToLineDistance(a.start, b.start, b.direction) > thickness/2;
            bool bothZeroLength = roughlyEqual(a.length(), 0) && roughlyEqual(b.length(), 0);
            if (tooFarApart || bothZeroLength) return {};

            // TODO: properly (a on b, b on a, ...)
            throw "not implemented";
        }

        auto determinant = db.cross2(da);

    } else {
        throw "not implemented";
    }
};

AtMost<2, Intersection> intersect (Circle a, Circle b) {
    auto aToB = (b.center - a.center);
    auto aToBDist = aToB.norm();

    if (roughlyEqual(aToBDist, 0) && roughlyEqual(a.radius, b.radius)) return {};
    if (aToBDist > (a.radius + b.radius + ROUGH_TOLERANCE)) return {};
    if (aToBDist < std::abs(a.radius - b.radius) - ROUGH_TOLERANCE) return {};

    auto aToCentroidDist = (pow(a.radius, 2) - pow(b.radius, 2) + pow(aToBDist, 2)) / (2 * aToBDist);
    auto intersectionToCentroidDist = sqrt(pow(a.radius, 2) - pow(aToCentroidDist, 2));

    auto centroid = a.center + (aToB * aToCentroidDist / aToBDist);

    auto centroidToIntersection = aToB.unitOrthogonal() * intersectionToCentroidDist;

    // solution 1
    auto solution1Position = centroid + centroidToIntersection;
    auto&& solution1 = Intersection(
            angleTheta(solution1Position - a.center),
            angleTheta(solution1Position - b.center),
            solution1Position
    );

    if (roughlyEqual(intersectionToCentroidDist, 0)) return {std::move(solution1)};

    // solution 2
    auto solution2Position = centroid - centroidToIntersection;
    auto&& solution2 = Intersection(
            angleTheta(solution2Position - a.center),
            angleTheta(solution2Position - b.center),
            solution2Position
    );

    return {std::move(solution1), std::move(solution2)};
};

#endif //COMPASS_INTERSECTIONS_H
