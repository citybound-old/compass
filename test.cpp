//
// Created by Anselm Eickhoff on 12/02/16.
//

#include <gtest/gtest.h>
#define EIGEN_MATRIXBASE_PLUGIN "../../../../../eigen_2d_extensions.h"
#include "primitives.h"
#include "intersections.h"

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

TEST(Compass, SegmentLineSubdivie) {
    auto segment = Segment(vec2(0, 0), vec2(1, 1));
    auto pieces = segment.subdivide(vec2(0.25, 0.25));
    EXPECT_VECTOR_ROUGHLY_EQUAL(vec2(0.0, 0.0), pieces[0].start);
    EXPECT_VECTOR_ROUGHLY_EQUAL(vec2(0.25, 0.25), pieces[0].end);
    EXPECT_VECTOR_ROUGHLY_EQUAL(vec2(0.25, 0.25), pieces[1].start);
    EXPECT_VECTOR_ROUGHLY_EQUAL(vec2(1.0, 1.0), pieces[1].end);
}

// CIRCLE-CIRCLE

TEST(Compass, CircleCircle) {
    auto a = Circle(vec2(0.25, 0.5), 0.25);
    auto b = Circle(vec2(0.5, 0.5), 0.25);
    auto i = intersect(a, b);

    EXPECT_EQ(i.size(), 2);
    EXPECT_VECTOR_ROUGHLY_EQUAL(i[0].position, vec2(0.375, 1 - 0.28349363803863525));
    EXPECT_VECTOR_ROUGHLY_EQUAL(i[1].position, vec2(0.375, 0.28349363803863525));
    EXPECT_NEAR(i[0].u, 0.16666667017293113, PRECISION);
    EXPECT_NEAR(i[0].v, 0.3333333298270689, PRECISION);
    EXPECT_NEAR(i[1].u, 0.8333333298270689, PRECISION);
    EXPECT_NEAR(i[1].v, 0.6666666701729311, PRECISION);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}