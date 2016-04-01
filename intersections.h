//
// Created by Anselm Eickhoff on 12/02/16.
//

#ifndef COMPASS_INTERSECTIONS_H
#define COMPASS_INTERSECTIONS_H

#include "primitives.h"
#include "at-most.h"
#include "lzy/lzy.h"

using namespace lzy;

const float ROUGH_TOLERANCE = 0.0000001;

template<typename N1, typename N2, typename N3>
bool roughlyEqual(N1 a, N2 b, N3 tolerance) {
    return std::abs(b - a) <= tolerance;
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
    float alongA;
    float alongB;
    vec2 position;

    Intersection (float alongA, float alongB, vec2 position)
            : alongA(alongA), alongB(alongB), position(position) {};

    Intersection (Intersection&& other)
            : alongA(other.alongA), alongB(other.alongB), position(std::move(other.position)) {};

    Intersection (const Intersection&& other)
            : alongA(other.alongA), alongB(other.alongB), position(std::move(other.position)) {};

    Intersection swapped () const {
        return Intersection(alongB, alongA, position);
    }
};

// FUNDAMENTAL INTERSECTIONS

AtMost<1, Intersection> intersect (Line a, Line b) {
    auto det = b.direction[0] * a.direction[1] - b.direction[1] * a.direction[0];

    if (roughlyEqual(0, det)) return {};

    auto delta = b.start - a.start;
    auto alongA = (delta[1] * b.direction[0] - delta[0] * b.direction[1]) / det;
    auto alongB = (delta[1] * a.direction[0] - delta[0] * a.direction[1]) / det;

    return {Intersection(alongA, alongB, a.start + alongA * a.direction)};
};

AtMost<2, Intersection> intersect (Circle& a, Circle& b) {
    auto aToB = (b.center - a.center);
    auto aToBDist = aToB.norm();

    if ((roughlyEqual(aToBDist, 0, thickness) && roughlyEqual(a.radius, b.radius, thickness))
        || aToBDist > (a.radius + b.radius + thickness)
        || aToBDist < std::abs(a.radius - b.radius) - thickness)
        return {};

    auto aToCentroidDist = (pow(a.radius, 2) - pow(b.radius, 2) + pow(aToBDist, 2)) / (2 * aToBDist);
    auto intersectionToCentroidDist = sqrt(pow(a.radius, 2) - pow(aToCentroidDist, 2));

    auto centroid = a.center + (aToB * aToCentroidDist / aToBDist);

    auto centroidToIntersection = aToB.unitOrthogonal() * intersectionToCentroidDist;

    // solution 1P
    auto solution1Position = centroid + centroidToIntersection;
    auto&& solution1 = Intersection(
            a.offsetAt(solution1Position),
            b.offsetAt(solution1Position),
            solution1Position
    );

    if (roughlyEqual((centroid - a.center).norm() - a.radius, 0, thickness)) return {std::move(solution1)};

    // solution 2
    auto solution2Position = centroid - centroidToIntersection;
    auto&& solution2 = Intersection(
            a.offsetAt(solution2Position),
            b.offsetAt(solution2Position),
            solution2Position
    );

    return {std::move(solution1), std::move(solution2)};
};

AtMost<2, Intersection> intersect (Line& a, Circle& b) {
    // TODO: tolerance: make radius always thickness bigger
    // then check if two solutions are close enough together to be one
    // if (((solution1Position + solution2Position)/2 - b.center).norm() > radius - thickness) ...
    auto delta = a.start - b.center;
    auto directionDotDelta = a.direction.dot(delta);
    auto det = std::pow(directionDotDelta, 2.0) - (delta.squaredNorm() - std::pow(b.radius, 2.0));

    if (det < 0) return {};

    auto t1 = (-directionDotDelta - std::sqrt(det));
    auto solution1Position = a.start + t1 * a.direction;
    auto&& solution1 = Intersection(
            t1,
            b.offsetAt(solution1Position),
            solution1Position
    );

    if (det == 0) return {std::move(solution1)};

    auto t2 = (-directionDotDelta + std::sqrt(det));
    auto solution2Position = a.start + t2 * a.direction;
    auto&& solution2 = Intersection(
            t2,
            b.offsetAt(solution2Position),
            solution2Position
    );

    return {std::move(solution1), std::move(solution2)};
};

AtMost<2, Intersection> intersect (Circle& a, Line& b) {
    return from(intersect(b, a)) >> map([](const Intersection& i) {return i.swapped();}) >> to<AtMost<2, Intersection>>();
};

// CONSTRAINED INTERSECTIONS

template <typename OtherPrimitive, typename std::enable_if<
        !std::is_same<OtherPrimitive, Segment>::value>::type* = nullptr>
AtMost<2, Intersection> intersect (Ray& a, OtherPrimitive& b) {
    return from(intersect(reinterpret_cast<Line&>(a), b)) >> (filter([](const Intersection& i) {
        return i.alongA > -thickness/2;
        // TODO: handle more exotic case where angles between a and b are pointy
        // TODO: and the intersection point is far but the touch point close
    }) | map([](const Intersection& i) -> Intersection {
        if (i.alongA >= 0) return std::move(i);
        else return Intersection(0, i.alongB, i.position);
    })) >> to<AtMost<2, Intersection>>();
};

template <typename OtherPrimitive, typename std::enable_if<
        !std::is_same<OtherPrimitive, Ray>::value && !std::is_same<OtherPrimitive, Segment>::value>::type* = nullptr>
AtMost<2, Intersection> intersect (OtherPrimitive& a, Ray& b) {
    return from(intersect(b, a)) >> map([](const Intersection& i) {return i.swapped();}) >> to<AtMost<2, Intersection>>();
};

template <typename OtherPrimitive>
AtMost<2, Intersection> intersect (Segment& a, OtherPrimitive& b) {
    if (a.isStraight()) {
        auto segmentAsRay = Ray(a.start, a.direction);
        return from(intersect(segmentAsRay, b)) >> (filter([&](const Intersection& i) {
            return i.alongA < a.length() + thickness/2;
            // TODO: handle more exotic case where angles between a and b are pointy
            // TODO: and the intersection point is far but the touch point close
        }) | map([&](const Intersection& i) -> Intersection {
            if (i.alongA <= a.length()) return std::move(i);
            else return Intersection(a.length(), i.alongB, i.position);
        })) >> to<AtMost<2, Intersection>>();
    } else {
        auto segmentAsCircle = Circle(a.radialCenter(), a.radius());
        return from(intersect(segmentAsCircle, b)) >> (filter([&](const Intersection& i) {
            return a.contains(i.position);
            // TODO: handle more exotic case where angles between a and b are pointy
            // TODO: and the intersection point is far but the touch point close
        }) | map([&](const Intersection& i) -> Intersection {
            float alongA = a.offsetAt(i.position);
            if (alongA < 0) return Intersection(0, i.alongB, i.position);
            else if (alongA > a.length()) return Intersection(a.length(), i.alongB, i.position);
            else return Intersection(alongA, i.alongB, i.position);
        })) >> to<AtMost<2, Intersection>>();
    }
};

template <typename OtherPrimitive, typename std::enable_if<
        !std::is_same<OtherPrimitive, Segment>::value>::type* = nullptr>
AtMost<2, Intersection> intersect (OtherPrimitive& a, Segment& b) {
    return from(intersect(b, a)) >> map([](const Intersection& i) {return i.swapped();}) >> to<AtMost<2, Intersection>>();
};

#endif //COMPASS_INTERSECTIONS_H
