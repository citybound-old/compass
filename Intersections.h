//
// Created by Anselm Eickhoff on 12/02/16.
//

#ifndef COMPASS_INTERSECTIONS_H
#define COMPASS_INTERSECTIONS_H

#include "primitives.h"

// TODO: also make this a view https://github.com/ericniebler/range-v3
template <int max_n, typename T>
class AtMost : public view_facade<AtMost> {
private:
    size_t actualElements;
    size_t current = 0;
    T elements[max_n];
public:
    AtMost = default;
    explicit AtMost (std::initializer_list<T> a_args) {
        elements = a_args;
        actualElements = a_args.size();
    }

    size_t size () {
        return actualElements;
    }

    T const & get () const {
        return elements[current];
    }

    bool done () const {
        return current == actualElements;
    }

    void next () {++current;}
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
        if (roughlyEqual(a.direction, (b.start - a.start).normalize()))
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

    u = std::max(u, 0);
    v = std::max(v, 0);

    return {Intersection(u, v, a.start + u * a.direction)};
}

AtMost<2, Intersection> intersect (Line a, Line b) {
    float determinant = b.direction[0] * a.direction[1] - b.direction[1] * a.direction[0];
    bool isParallel = roughlyEqual(determinant, 0);

    if (isParallel) {
        bool tooFarApart = pointToLineDistance(a.start, b.start, b.direction) > thickness/2;
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
        bool tooFarApart = pointToLineDistance(a.start, b.start, b.direction) > thickness/2;
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

    u = std::max(u, 0);

    return {Intersection(u, v, a.start + u * a.direction)};
}
AtMost<2, Intersection> intersect (Line a, Ray b) {
    return intersect(b, a) | view::transform(&Intersection::swapped);
}

AtMost<2, Intersection> intersect (Ray a, Segment b) {
    if (b.isStraight()) {
        auto potentials = intersect(a, Ray(b.start, b.direction));
        return potentials | view::remove_if([](Intersection i){
            return i.v <= b.length() + thickness/2;
        }) | view::transform([](Intersection i){
            return Intersection(i.u, std::min(i.v, 1), i.position);
        });
    } else {
        throw std::exception("not implemented");
    }
}
AtMost<2, Intersection> intersect (Segment a, Ray b) {
    return intersect(b, a) | view::transform(&Intersection::swapped);
}

AtMost<2, Intersection> intersect (Segment a, Segment b) {

    if (a.isStraight() && b.isStraight()) {

        auto da = a.end - a.start;
        auto db = b.end - b.start;
        auto orientation = crossz(a.direction, b.direction);

        bool isParallel = roughlyEqual(orientation, 0);
        if (isParallel) {
            bool tooFarApart = pointToLineDistance(a.start, b.start, b.direction) > thickness/2;
            bool bothZeroLength = roughlyEqual(a.length(), 0) && roughlyEqual(b.length(), 0);
            if (tooFarApart || bothZeroLength) return {};

            // TODO: properly (a on b, b on a, ...)
            throw std::exception("not implemented");
        }

        auto determinant = crossz(db, da);

    } else {
        throw std::exception("not implemented");
    }
};

#endif //COMPASS_INTERSECTIONS_H
