//
// Copyright 2024 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_TEST_UTIL_IO_TEXT_STREAM_TEST_H_
#define CODELIBRARY_TEST_UTIL_IO_TEXT_STREAM_TEST_H_

#include "codelibrary/base/timer.h"
#include "codelibrary/base/testing.h"
#include "codelibrary/util/io/text_stream.h"

namespace cl {
namespace test {

TEST(TextStreamTest, ReadDoublePerformance) {
    std::string test;
    int n = 10000;
    for (int i = 0; i < n; ++i) {
        test += "3.14159265358979323846 ";
    }

    Print("\nPerformace for reading {} PI from string:\n\n", n);
    Timer timer;
    timer.Start();
    std::istringstream iss(test);

    double sum1 = 0.0;
    double value;
    while (iss >> value) {
        sum1 += value;
    }

    timer.Stop();
    Print("std::istringstream {}\n", timer.elapsed_time());

    timer.Reset();
    timer.Start();
    double sum2 = 0.0;
    const char* str = test.c_str();
    char* end_ptr;
    while (*str != '\0') {
        double value = std::strtod(str, &end_ptr);
        if (str == end_ptr) {
            break;
        }
        sum2 += value;
        str = end_ptr;
    }
    timer.Stop();
    Print("std::strtod        {}\n", timer.elapsed_time());

    timer.Reset();
    timer.Start();
    double sum3 = 0.0;
    TextStream ts(test);
    while (ts >> value) {
        sum3 += value;
    }
    timer.Stop();
    Print("Our TextStream     {}\n\n", timer.elapsed_time());

    ASSERT_EQ(sum1, sum2);
    ASSERT_EQ(sum1, sum3);
}

} // namespace test
} // namespace cl

#endif // CODELIBRARY_TEST_UTIL_IO_TEXT_STREAM_TEST_H_
