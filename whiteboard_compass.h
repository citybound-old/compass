#ifndef COMPASS_WHITEBOARD_COMPASS_H
#define COMPASS_WHITEBOARD_COMPASS_H

#include "primitives.h"
typedef Eigen::Vector2f vec2;

wb::draw_stream& operator<< (wb::draw_stream& drawStream, vec2 point) {
    drawStream.startOutput() << "dot " << point[0] << " " << point[1];
    drawStream.lineDone();
    return drawStream;
}

wb::draw_stream& operator<< (wb::draw_stream& drawStream, Segment segment) {
    if (segment.isStraight()) {
        drawStream.startOutput() << "line "
            << segment.start[0] << " " << segment.start[1] << " "
            << segment.end[0] << " " << segment.end[1];
        drawStream.lineDone();
    } else {
        drawStream.startOutput() << "arc "
            << segment.start[0] << " " << segment.start[1] << " "
            << segment.direction[0] << " " << segment.direction[1] << " "
            << segment.end[0] << " " << segment.end[1];
        drawStream.lineDone();
    }
    return drawStream;
}

wb::draw_stream& operator<< (wb::draw_stream& drawStream, Circle circle) {
    return drawStream << Segment(circle.center + vec2(0, -circle.radius), vec2(1, 0), circle.center + vec2(0, circle.radius))
               << Segment(circle.center + vec2(0, circle.radius), vec2(-1, 0), circle.center + vec2(0, -circle.radius));
}

wb::draw_stream& operator<< (wb::draw_stream& drawStream, Line line) {
    return drawStream << Segment(line.start - 1000 * line.direction, line.start + 1000 * line.direction);
}

wb::draw_stream& operator<< (wb::draw_stream& drawStream, Ray ray) {
    return drawStream << Segment(ray.start, ray.start + 1000 * ray.direction);
}

#endif //COMPASS_WHITEBOARD_COMPASS_H
