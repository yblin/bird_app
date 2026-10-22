//
// Copyright 2022-2025 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_STRING_STRING_JOIN_H_
#define CODELIBRARY_STRING_STRING_JOIN_H_

#include "codelibrary/base/array.h"

namespace cl {

/**
 * StringJoin() is a counterpart to StringSplit(). It joins strings from a
 * 'list' by adding user defined separator.
 */
template <typename String>
String StringJoin(const Array<String>& list,
                  const typename String::value_type& c) {
    if (list.empty()) return String();

    typename String::size_type total = list.size() - 1;
    for (const auto& s : list) total += s.size();

    String result;
    result.reserve(total);
    for (int i = 0; i + 1 < list.size(); ++i) {
        result.append(list[i]);
        result += c;
    }
    result.append(list.back());
    return result;
}

} // namespace cl

#endif // CODELIBRARY_STRING_STRING_JOIN_H_
