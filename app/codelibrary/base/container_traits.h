//
// Copyright 2024 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_BASE_CONTAINER_TRAITS_H_
#define CODELIBRARY_BASE_CONTAINER_TRAITS_H_

#include <type_traits>

namespace cl {
namespace container_traits {

/**
 * Define a trait to check for the existence of the size() member function.
 */
template <typename T>
class has_size {
private:
    template <typename U>
    static constexpr auto check(int) -> decltype(std::declval<U>().size(),
                                                 std::true_type{});

    template <typename>
    static constexpr auto check(...) -> std::false_type;

public:
    static constexpr bool value = decltype(check<T>(0))::value;
};

} // namespace container_traits
} // namespace cl

#endif // CODELIBRARY_BASE_CONTAINER_TRAITS_H_
