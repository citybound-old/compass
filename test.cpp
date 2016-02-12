//
// Created by Anselm Eickhoff on 12/02/16.
//

#include <gtest/gtest.h>

TEST(CPlusPlus, CanCalculate) {
    EXPECT_EQ(1, 4 - 3);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}