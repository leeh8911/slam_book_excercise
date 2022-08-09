#include <iostream>

#include <gtest/gtest.h>

TEST(HelloTest, SayHello) {
    std::cout << "Hello world" <<std::endl;
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}