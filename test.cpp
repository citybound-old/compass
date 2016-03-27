//
// Created by Anselm Eickhoff on 12/02/16.
//

#include <gtest/gtest.h>
#define EIGEN_MATRIXBASE_PLUGIN "../../../../../eigen_2d_extensions.h"
#include "primitives.h"
#include "intersections.h"
#include "whiteboard/whiteboard.h"
#include "whiteboard_compass.h"

typedef Eigen::Vector2f vec2;

#define PRECISION 0.001

#define TEST_CALL(statement) \
   do { \
     SCOPED_TRACE(""); \
     statement; \
   } while(0)

#define EXPECT_VECTOR_ROUGHLY_EQUAL(...) \
   TEST_CALL(test_vectors_roughly_equal(__VA_ARGS__))

void test_vectors_roughly_equal(vec2 const& expected, vec2 const& actual)
{
    EXPECT_NEAR(expected[0], actual[0], PRECISION) << " at " << 0;
    EXPECT_NEAR(expected[1], actual[1], PRECISION) << " at " << 1;
}

auto whiteboard = wb::draw(std::cout);

// PRIMITIVES: SEGMENTS

//TEST(CompassPrimitives, LineSegmentSubdivide) {
//    whiteboard << wb::clear;
//    auto segment = Segment({0, 0}, {1, 1});
//    auto pieces = segment.subdivide({0.25, 0.25});
//    whiteboard << wb::color{255, 0, 0, 255} << pieces[0];
//    whiteboard << wb::color{0, 0, 255, 255} << pieces[1];
//
//    EXPECT_VECTOR_ROUGHLY_EQUAL(vec2(0.0, 0.0), pieces[0].start);
//    EXPECT_VECTOR_ROUGHLY_EQUAL(vec2(0.25, 0.25), pieces[0].end);
//    EXPECT_VECTOR_ROUGHLY_EQUAL(vec2(0.25, 0.25), pieces[1].start);
//    EXPECT_VECTOR_ROUGHLY_EQUAL(vec2(1.0, 1.0), pieces[1].end);
//}
//
//TEST(CompassPrimitives, ArcSegmentProperties) {
//    whiteboard << wb::clear;
//    auto a = Segment({0, 0}, {0, 1}, {0.4, 0});
//    auto b = Segment({0.2, 0}, {0, 1}, {0.6, 0});
//    whiteboard << a << b;
//
//    EXPECT_FALSE(a.isStraight());
//    EXPECT_FALSE(b.isStraight());
//
//    EXPECT_NEAR(0.2, a.radius(), PRECISION);
//    EXPECT_NEAR(0.2, b.radius(), PRECISION);
//
//    whiteboard << a.radialCenter() << b.radialCenter();
//
//    EXPECT_VECTOR_ROUGHLY_EQUAL(vec2(0.2, 0), a.radialCenter());
//    EXPECT_VECTOR_ROUGHLY_EQUAL(vec2(0.4, 0), b.radialCenter());
//}
//
//TEST(CompassPrimitives, ArcSegmentToLineSegment) {
//    auto a = Segment({0, 0}, {0, 1}, {0, 1});
//    EXPECT_TRUE(a.isStraight());
//}
//
//TEST(CompassPrimitives, ArcSegmentSubdivide) {
//    whiteboard << wb::clear;
//    auto segment = Segment({0.5, 0}, {1, 0}, {0.5, 1});
//    auto pieces = segment.subdivide({1.0, 0.5});
//    whiteboard << wb::color{255, 0, 0, 255} << pieces[0];
//    whiteboard << wb::color{0, 0, 255, 255} << pieces[1];
//
//    EXPECT_VECTOR_ROUGHLY_EQUAL(vec2(0.5, 0.0), pieces[0].start);
//    EXPECT_VECTOR_ROUGHLY_EQUAL(vec2(1.0, 0.0), pieces[0].direction);
//    EXPECT_VECTOR_ROUGHLY_EQUAL(vec2(1.0, 0.5), pieces[0].end);
//    EXPECT_VECTOR_ROUGHLY_EQUAL(vec2(1.0, 0.5), pieces[1].start);
//    EXPECT_VECTOR_ROUGHLY_EQUAL(vec2(0.0, 1.0), pieces[1].direction);
//    EXPECT_VECTOR_ROUGHLY_EQUAL(vec2(0.5, 1.0), pieces[1].end);
//}

// CIRCLE-CIRCLE

TEST(CompassIntersections, CircleCircleIntersection) {
    whiteboard << wb::clear;
    auto a = Circle({0.25, 0.5}, 0.25);
    auto b = Circle({0.5, 0.5}, 0.25);
    auto i = intersect(a, b);
    whiteboard << a << b;

    EXPECT_EQ(2, i.size());

    whiteboard << wb::color{0, 0, 0, 64}
               << Segment(a.center, a.center + 0.3 * vec2(std::cos(i[0].alongA / a.radius), std::sin(i[0].alongA / a.radius)))
               << Segment(a.center, a.center + 0.3 * vec2(std::cos(i[1].alongA / a.radius), std::sin(i[1].alongA / a.radius)))
               << Segment(b.center, b.center + 0.3 * vec2(std::cos(i[0].alongB / b.radius), std::sin(i[0].alongB / b.radius)))
               << Segment(b.center, b.center + 0.3 * vec2(std::cos(i[1].alongB / b.radius), std::sin(i[1].alongB / b.radius)));

    EXPECT_NEAR(i[0].alongA, 1.0/6.0 * 2 * M_PI * a.radius, PRECISION);
    EXPECT_NEAR(i[0].alongB, 2.0/6.0 * 2 * M_PI *  b.radius, PRECISION);
    EXPECT_NEAR(i[1].alongA, 5.0/6.0 * 2 * M_PI *  a.radius, PRECISION);
    EXPECT_NEAR(i[1].alongB, 4.0/6.0 * 2 * M_PI *  b.radius, PRECISION);

    whiteboard << wb::color{255, 0, 0, 255} << i[0].position << i[1].position;

    EXPECT_VECTOR_ROUGHLY_EQUAL(vec2(0.375, 1 - 0.28349363803863525), i[0].position);
    EXPECT_VECTOR_ROUGHLY_EQUAL(vec2(0.375, 0.28349363803863525), i[1].position);
}

TEST(CompassIntersections, CircleCircleTip) {
    whiteboard << wb::clear;
    auto a = Circle({0.25, 0.5}, 0.25);
    auto b = Circle({0.75, 0.5}, 0.25);
    auto i = intersect(a, b);
    whiteboard << a << b;

    EXPECT_EQ(1, i.size());

    whiteboard << wb::color{0, 0, 0, 64}
        << Segment(a.center, a.center + 0.3 * vec2(std::cos(i[0].alongA / a.radius), std::sin(i[0].alongA / a.radius)))
        << Segment(b.center, b.center + 0.3 * vec2(std::cos(i[0].alongB / b.radius), std::sin(i[0].alongB / b.radius)));

    EXPECT_NEAR(i[0].alongA, 0, PRECISION);
    EXPECT_NEAR(i[0].alongB, 0.5 * 2.0 * M_PI * b.radius, PRECISION);

    whiteboard << wb::color{255, 0, 0, 255} << i[0].position;

    EXPECT_VECTOR_ROUGHLY_EQUAL(vec2(0.5, 0.5), i[0].position);
}

TEST(CompassIntersections, CircleCircleInsideOutside) {
    auto a = Circle({0.5, 0.5}, 0.5);
    auto b = Circle({0.5, 0.5}, 0.25);
    auto i = intersect(a, b);

    EXPECT_EQ(0, i.size());
}

TEST(CompassIntersections, CircleCircleSame) {
    auto a = Circle({0.5, 0.5}, 0.5);
    auto b = Circle({0.5, 0.5}, 0.5);
    auto i = intersect(a, b);

    EXPECT_EQ(0, i.size());
}

// LINE-LINE

TEST(CompassIntersections, LineLineIntersection) {
    whiteboard << wb::clear;
    auto a = Line({0.5, 0.5}, {-1, -1});
    auto b = Line({0.5, 1}, {-1, 0});
    auto i = intersect(a, b);
    whiteboard << a << b;

    EXPECT_EQ(1, i.size());

    whiteboard << wb::color{255, 0, 0, 255} << i[0].position;

    EXPECT_VECTOR_ROUGHLY_EQUAL(vec2(1, 1), i[0].position);
}

TEST(CompassIntersections, LineLineIntersection2) {
    whiteboard << wb::clear;
    auto a = Line({0, 0}, {1, 1});
    auto b = Line({0, 1}, {1, 0});
    auto i = intersect(a, b);
    whiteboard << a << b;

    EXPECT_EQ(1, i.size());

    whiteboard << wb::color{255, 0, 0, 255} << i[0].position;

    EXPECT_VECTOR_ROUGHLY_EQUAL(vec2(1, 1), i[0].position);
}

TEST(CompassIntersections, LineLineParallel) {
    auto a = Line({0, 0.1}, {1, 0});
    auto b = Line({0, 0.9}, {1, 0});
    auto i = intersect(a, b);

    EXPECT_EQ(0, i.size());
}

TEST(CompassIntersections, LineLineContained) {
    auto a = Line({0, 0.5}, {1, 0});
    auto b = Line({0.5, 0.5}, {1, 0});
    auto i = intersect(a, b);

    EXPECT_EQ(0, i.size());
}

// RAY-RAY

TEST(CompassIntersections, RayRayIntersection) {
    whiteboard << wb::clear;
    auto a = Ray({0, 0}, {1, 1});
    auto b = Ray({0, 1}, {1, 0});
    auto i = intersect(a, b);
    whiteboard << a << b;

    EXPECT_EQ(1, i.size());

    whiteboard << wb::color{255, 0, 0, 255} << i[0].position;

    EXPECT_VECTOR_ROUGHLY_EQUAL(vec2(1, 1), i[0].position);
}

TEST(CompassIntersections, RayRayTip) {
    whiteboard << wb::clear;
    auto a = Ray({0, 0}, {0, 1});
    auto b = Ray({0, 0}, {1, 0});
    auto i = intersect(a, b);
    whiteboard << a << b;

    EXPECT_EQ(1, i.size());

    whiteboard << wb::color{255, 0, 0, 255} << i[0].position;

    EXPECT_VECTOR_ROUGHLY_EQUAL(vec2(0, 0), i[0].position);
}

TEST(CompassIntersections, RayRayContained) {
    auto a = Ray({0, 0.5}, {1, 0});
    auto b = Ray({0.5, 0.5}, {1, 0});
    auto i = intersect(a, b);

    EXPECT_EQ(0, i.size());
}

TEST(CompassIntersections, RayRayContained2) {
    auto a = Ray({0.5, 0.5}, {1, 0});
    auto b = Ray({0, 0.5}, {1, 0});
    auto i = intersect(a, b);

    EXPECT_EQ(0, i.size());
}

TEST(CompassIntersections, RayRayNoIntersection) {
    auto a = Ray({0.33, 1}, {-0.1, 1});
    auto b = Ray({0.66, 1}, {-0.1, -1});
    auto i = intersect(a, b);

    EXPECT_EQ(0, i.size());
}

TEST(CompassIntersections, RayRayIntersectionUp) {
    auto a = Ray({0.33, 1}, {0.1, -1});
    auto b = Ray({0.66, 1}, {-0.1, -1});
    auto i = intersect(a, b);

    EXPECT_EQ(1, i.size());
}

TEST(CompassIntersections, RayRayIntersectionDown) {
    auto a = Ray({0.33, 0}, {0.1, 1});
    auto b = Ray({0.66, 0}, {-0.1, 1});
    auto i = intersect(a, b);

    EXPECT_EQ(1, i.size());
}

// RAY-LINE

TEST(CompassIntersections, RayLineIntersection) {
    whiteboard << wb::clear;
    auto a = Ray({0, 0}, {1, 1});
    auto b = Line({0, 1}, {1, 0});
    auto i = intersect(a, b);
    whiteboard << a << b;

    EXPECT_EQ(1, i.size());

    whiteboard << wb::color{255, 0, 0, 255} << i[0].position;

    EXPECT_VECTOR_ROUGHLY_EQUAL(vec2(1, 1), i[0].position);
}

TEST(CompassIntersections, RayLineNoIntersection) {
    auto a = Ray({0.5, 1}, {-1, 0});
    auto b = Line({0.5, 0.5}, {1, 1});
    auto i = intersect(a, b);

    EXPECT_EQ(0, i.size());
}

// RAY-LINESEGMENT

TEST(CompassIntersections, RayLineSegmentIntersection) {
    whiteboard << wb::clear;
    auto a = Ray({0, 0.5}, {1, 0});
    auto b = Segment({1, 0.25}, {1, 0.75});
    auto i = intersect(a, b);
    whiteboard << a << b;

    EXPECT_EQ(1, i.size());

    whiteboard << wb::color{255, 0, 0, 255} << i[0].position;

    EXPECT_VECTOR_ROUGHLY_EQUAL(vec2(1, 0.5), i[0].position);
}

TEST(CompassIntersections, RayLineSegmentNoIntersectionUp) {
    auto a = Ray({0, 0.5}, {1, 0});
    auto b = Segment({1, 0.75}, {1, 1});
    auto i = intersect(a, b);

    EXPECT_EQ(0, i.size());
}

TEST(CompassIntersections, RayLineSegmentNoIntersectionDown) {
    auto a = Ray({0, 0.5}, {1, 0});
    auto b = Segment({1, 0}, {1, 0.25});
    auto i = intersect(a, b);

    EXPECT_EQ(0, i.size());
}

// LINESEGMENT-LINESEGMENT

TEST(CompassIntersections, LineSegmentLineSegmentIntersection) {
    whiteboard << wb::clear;
    auto a = Segment({0, 0}, {1, 1});
    auto b = Segment({0, 1}, {1, 0});
    auto i = intersect(a, b);
    whiteboard << a << b;

    EXPECT_EQ(1, i.size());

    whiteboard << wb::color{255, 0, 0, 255} << i[0].position;

    EXPECT_VECTOR_ROUGHLY_EQUAL(vec2(0.5, 0.5), i[0].position);
    EXPECT_NEAR(0.5 * a.length(), i[0].alongA, PRECISION);
    EXPECT_NEAR(0.5 * b.length(), i[0].alongB, PRECISION);
}

TEST(CompassIntersections, LineSegmentLineSegmentNoIntersection) {
    auto a = Segment({0, 0}, {-1, -1});
    auto b = Segment({0, 1}, {1, 0});
    auto i = intersect(a, b);

    EXPECT_EQ(0, i.size());
}

TEST(CompassIntersections, LineSegmentLineSegmentSame) {
    auto a = Segment({0, 0}, {1, 1});
    auto b = a;
    auto i = intersect(a, b);

    EXPECT_EQ(0, i.size());
}

TEST(CompassIntersections, LineSegmentLineSegmentParallel) {
    auto a = Segment({0, 0}, {0, 1});
    auto b = Segment({1, 0}, {1, 1});
    auto i = intersect(a, b);

    EXPECT_EQ(0, i.size());
}

TEST(CompassIntersections, LineSegmentLineSegmentOverlap) {
    auto a = Segment({0, 0.5}, {0.75, 0.5});
    auto b = Segment({0.25, 0.5}, {1, 0.5});
    auto i = intersect(a, b);

    EXPECT_EQ(0, i.size());
}

TEST(CompassIntersections, LineSegmentLineSegmentContainedTouchingStartTip) {
    auto a = Segment({0, 0.5}, {1, 0.5});
    auto b = Segment({0, 0.5}, {0.5, 0.5});
    auto i = intersect(a, b);

    EXPECT_EQ(0, i.size());
}

TEST(CompassIntersections, LineSegmentLineSegmentContainedTouchingEndTip) {
    auto a = Segment({0, 0.5}, {1, 0.5});
    auto b = Segment({0.5, 0.5}, {1, 0.5});
    auto i = intersect(a, b);

    EXPECT_EQ(0, i.size());
}

TEST(CompassIntersections, LineSegmentLineSegmentContained) {
    auto a = Segment({0, 0.5}, {1, 0.5});
    auto b = Segment({0.25, 0.5}, {0.75, 0.5});
    auto i = intersect(a, b);

    EXPECT_EQ(0, i.size());
}

TEST(CompassIntersections, LineSegmentLineSegmentTipStartToEnd) {
    auto a = Segment({0, 0}, {1, 1});
    auto b = Segment({1, 1}, {1, 0});
    auto i = intersect(a, b);

    EXPECT_EQ(1, i.size());

    EXPECT_NEAR(1 * a.length(), i[0].alongA, PRECISION);
    EXPECT_NEAR(0 * b.length(), i[0].alongB, PRECISION);

    EXPECT_VECTOR_ROUGHLY_EQUAL(vec2(1, 1), i[0].position);
}

TEST(CompassIntersections, LineSegmentLineSegmentTipEndToEnd) {
    auto a = Segment({0, 0}, {0.5, 0.5});
    auto b = Segment({1, 0}, {0.5, 0.5});
    auto i = intersect(a, b);

    EXPECT_EQ(1, i.size());

    EXPECT_NEAR(1 * a.length(), i[0].alongA, PRECISION);
    EXPECT_NEAR(1 * b.length(), i[0].alongB, PRECISION);

    EXPECT_VECTOR_ROUGHLY_EQUAL(vec2(0.5, 0.5), i[0].position);
}

TEST(CompassIntersections, LineSegmentLineSegmentTipStartToStart) {
    auto a = Segment({0.5, 0.5}, {0, 0});
    auto b = Segment({0.5, 0.5}, {1, 0});
    auto i = intersect(a, b);

    EXPECT_EQ(1, i.size());

    EXPECT_NEAR(0 * a.length(), i[0].alongA, PRECISION);
    EXPECT_NEAR(0 * b.length(), i[0].alongB, PRECISION);

    EXPECT_VECTOR_ROUGHLY_EQUAL(vec2(0.5, 0.5), i[0].position);
}

//TEST(CompassIntersections, ArcSegmentArcSegmentIntersection1) {
//    auto a = Segment({0, 0}, {0, 1}, {1, 1});
//    auto b = Segment({1, 0}, {0, 1}, {0, 1});
//    auto i = intersect(a, b);
//
//    EXPECT_EQ(1, i.size());
//
//    EXPECT_VECTOR_ROUGHLY_EQUAL(vec2(0.5, 0.8660253882408142), i[0].position);
//}
//
//TEST(CompassIntersections, ArcSegmentArcSegmentIntersection2) {
//    auto a = Segment({0.25, 0}, {1, 0}, {0.25, 1});
//    auto b = Segment({0.75, 0}, {-1, 0}, {0.75, 1});
//    auto i = intersect(a, b);
//
//    EXPECT_EQ(2, i.size());
//
//    EXPECT_VECTOR_ROUGHLY_EQUAL(vec2(0.5, 1 - 0.0669872984290123), i[0].position);
//    EXPECT_VECTOR_ROUGHLY_EQUAL(vec2(0.5, 0.0669872984290123), i[1].position);
//}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}