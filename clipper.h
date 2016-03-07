/*

    This module implements polygon clippings, inspired by:

 An Extension of Polygon Clipping To Resolve Degenerate Cases
 ============================================================

                              by

                 Dae Hyun Kim & Myoung-Jun Kim

            (a paper that I paid fucking 42EUR for)

 */

#ifndef COMPASS_CLIPPER_H
#define COMPASS_CLIPPER_H

#include "./primitives.h"
#include "range/v3/all.hpp"

enum Mode {INTERSECTION, UNION, DIFFERENCE, NOT};
enum Location {INSIDE, OUTSIDE, ON_EDGE};
enum Role {NONE, ENTRY, EXIT, ENTRY_EXIT, EXIT_ENTRY};
enum Direction {FORWARD_STAY, FORWARD_SWITCH, BACKWARD_STAY, BACKWARD_SWITCH};

struct ClipperVertex {
    Segment forwardEdge;
    Location forwardEdgeLocation;
    Location location;
    Role role;
    bool visited;
    ClipperVertex* neighbor;
    ClipperVertex* partner;
    bool isFirstPartner;

    vec2 position () { return forwardEdge.start; };
    bool isIntersection () {return neighbor && role != NONE; };
    bool isEntry () {return role == ENTRY || role == ENTRY_EXIT; };
    bool isExit () {return role == EXIT || role == EXIT_ENTRY; };

    ClipperVertex (Segment segment) : forwardEdge(segment) {};

    ClipperVertex split (vec2 position) {
        auto parts = forwardEdge.subdivide(position);
        forwardEdge = parts[0];
        auto newVertex = ClipperVertex(parts[1]);

        newVertex.next = next;
        next->previous = newVertex;
        newVertex.previous = this;
        next = &newVertex;

        return newVertex;
    }
};

std::vector<Path> clip (Mode mode, Path& subjectPath, clipPath) {
    if (subjectPath.isClockwise()) return clip(mode, subjectPath.reversed(), clipPath);
    if (clipPath.isClockwise()) return clip(mode, subjectPath, clipPath.reversed());

    ClipperVertex* subjectChain = createVertexChain(subjectPath, clipPath);
    ClipperVertex* clipChain = createVertexChain(clipPath, subjectPath);

    auto intersectionGroups = findIntersections(subjectChain, clipChain);
    splitAtIntersections(subjectChain, clipChain, intersectionGroups);
    determineEdgeLocations(subjectChain, clipChain, subjectPath, clipPath);

    bool allEdgesOnAnotherEdge = false;
    for (auto vertex : subjectChain) {
        if (vertex.forwardEdgeLocation != ON_EDGE) {
            allEdgesOnAnotherEdge = false;
            break;
        }
    }

    if (allEdgesOnAnotherEdge) {
        if (mode == UNION || mode == INTERSECTION) {
            return {subjectChain.toPath()};
        } else return {};
    }

    setRoles(subjectChain, clipChain);
    markCouples(subjectChain, clipChain);

    if (mode == UNION || mode == DIFFERENCE) {
        reverseRoles(subjectChain);
    }

    if (mode == UNION || mode == NOT) {
        reverseRoles(clipChain);
    }

    std::vector<Path> resultPaths = traverse(subjectChain, clipChain, mode);
    resultPaths |= range::actions::transform([&](Segment segment){return segment.weld(thickness);});

    // TODO: free chains?

    return resultPaths;
}


#endif //COMPASS_CLIPPER_H
