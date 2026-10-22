//
// Copyright 2026 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_STRING_SUFFIX_H_
#define CODELIBRARY_STRING_SUFFIX_H_

#include <iterator>
#include <cctype>
#include <type_traits>

#include "codelibrary/base/log.h"

namespace cl {

/**
 * Check if the string [first, last) has the suffix [suffix_first, suffix_last).
 */
template <typename Iterator1, typename Iterator2>
bool HasSuffix(Iterator1 first, Iterator1 last,
               Iterator2 suffix_first, Iterator2 suffix_last) {
    auto str_len = std::distance(first, last);
    auto suffix_len = std::distance(suffix_first, suffix_last);

    if (suffix_len == 0) return true;
    if (suffix_len > str_len) return false;

    Iterator1 str_it = first;
    std::advance(str_it, str_len - suffix_len);

    while (suffix_first != suffix_last) {
        if (str_it == last || *str_it != *suffix_first) {
            return false;
        }
        ++str_it;
        ++suffix_first;
    }

    return true;
}

/**
 * Check if the string has the specific prefix, supporting case-sensitive or
 * insensitive comparison.
 */
template <typename T>
bool HasSuffix(const T& str, const T& suffix, bool case_sensitive = true) {
    if (suffix.empty()) return true;
    if (suffix.size() > str.size()) return false;

    auto str_it = str.begin();
    std::advance(str_it, str.size() - suffix.size());
    auto suffix_it = suffix.begin();

    using Char = typename T::value_type;

    while (suffix_it != suffix.end()) {
        Char c1 = *str_it;
        Char c2 = *suffix_it;

        // Case-insensitive comparison.
        if (!case_sensitive) {
            // Use unsigned char to avoid undefined behavior for chars with
            // negative values (e.g., 0x80-0xFF)
            using unsigned_char_type = typename std::make_unsigned<Char>::type;
            c1 = static_cast<Char>(
                        std::tolower(static_cast<unsigned_char_type>(c1)));
            c2 = static_cast<Char>(
                        std::tolower(static_cast<unsigned_char_type>(c2)));
        }

        if (c1 != c2) return false;

        ++str_it;
        ++suffix_it;
    }

    return true;
}

/**
 * Overload: Support char* suffix.
 */
inline bool HasSuffix(const std::string& str, const char* suffix,
                      bool case_sensitive = true) {
    CHECK(suffix);

    return HasSuffix(str, std::string(suffix), case_sensitive);
}
inline bool HasSuffix(const char* str, const char* suffix,
                      bool case_sensitive = true) {
    CHECK(str);
    CHECK(suffix);

    return HasSuffix(std::string(str), std::string(suffix), case_sensitive);
}
inline bool HasSuffix(const char* str, const std::string& suffix,
                      bool case_sensitive = true) {
    CHECK(str);

    return HasSuffix(std::string(str), std::string(suffix), case_sensitive);
}

} // namespace cl

#endif // CODELIBRARY_STRING_SUFFIX_H_
