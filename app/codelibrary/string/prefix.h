//
// Copyright 2024-2026 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_STRING_PREFIX_H_
#define CODELIBRARY_STRING_PREFIX_H_

#include <cctype>
#include <iterator>

namespace cl {

/**
 * Check if the string [first, last) has the prefix [prefix_first, prefix_last).
 */
template <typename Iterator1, typename Iterator2>
bool HasPrefix(Iterator1 first, Iterator1 last,
               Iterator2 prefix_first, Iterator2 prefix_last) {
    while (prefix_first != prefix_last) {
        if (first == last || *first != *prefix_first) {
            return false;
        }
        ++first;
        ++prefix_first;
    }
    return true;
}

/**
 * Check if the string has the specific prefix, supporting case-sensitive or
 * insensitive comparison.
 */
template <typename T>
bool HasPrefix(const T& str, const T& prefix, bool case_sensitive = true) {
    if (prefix.empty()) return true;
    if (prefix.size() > str.size()) return false;

    using Char = typename T::value_type;

    auto i = str.begin();
    for (auto j = prefix.begin(); j != prefix.end(); ++j, ++i) {
        auto c1 = *i;
        auto c2 = *j;
        if (!case_sensitive) {
            // Use unsigned char to avoid undefined behavior for chars with
            // negative values (e.g., 0x80-0xFF).
            using unsigned_char_type = typename std::make_unsigned<Char>::type;
            c1 = static_cast<Char>(
                std::tolower(static_cast<unsigned_char_type>(c1)));
            c2 = static_cast<Char>(
                std::tolower(static_cast<unsigned_char_type>(c2)));
        }

        if (c1 != c2) return false;
    }

    return true;
}

} // namespace cl

#endif // CODELIBRARY_STRING_PREFIX_H_
