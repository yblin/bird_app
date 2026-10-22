//
// Copyright 2020-2024 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_TEST_BASE_MESSAGE_TEST_H_
#define CODELIBRARY_TEST_BASE_MESSAGE_TEST_H_

#include "codelibrary/base/message.h"
#include "codelibrary/base/testing.h"

namespace cl {

class MessageTest : public Test {
public:
    struct Tmp {
        friend std::ostream& operator << (std::ostream& os, const Tmp&) {
            os << "test";
            return os;
        }
    };
};

TEST_F(MessageTest, TestPointer) {
    int* p = nullptr;
    Message msg;
    msg.Append(p);
    ASSERT_EQ(msg.text(), "(nullptr)");

    const int* p1 = nullptr;
    Message msg1(p1);
    ASSERT_EQ(msg1.text(), "(nullptr)");
}

TEST_F(MessageTest, TestObject) {
    Tmp t;
    Message msg(t);
    ASSERT_EQ(msg.text(), "test");
}

TEST_F(MessageTest, TestNumbers) {
    Message msg;
    msg << 0.1;
    ASSERT_EQ(msg.text(), "0.1");
}

TEST_F(MessageTest, TestArray) {
    Array<int> a(10);
    for (int i = 0; i < a.size(); ++i)
        a[i] = i;

    Message msg;
    msg.set_max_elements(0);
    msg << a;
    ASSERT_EQ(msg.text(), "[0, 1, 2, 3, 4, 5, 6, 7, 8, 9]");

    msg.clear();
    msg.set_max_elements(6);
    msg << a;
    ASSERT_EQ(msg.text(), "[0, 1, 2, ..., 7, 8, 9]");
}

} // namespace cl

#endif // CODELIBRARY_TEST_BASE_MESSAGE_TEST_H_
