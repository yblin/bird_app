//
// Copyright 2024-2025 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_BASE_MEMORY_H_
#define CODELIBRARY_BASE_MEMORY_H_

#include <limits>

#include "codelibrary/base/log.h"

namespace cl {

/**
 * Allocate 'size' bytes in aligned mode.
 */
inline void* AlignedAllocate(size_t size, int alignment) {
    CHECK(alignment > 0 && (alignment & (alignment - 1)) == 0);
    size_t offset = alignment - 1 + sizeof(void*);
    CHECK(size <= std::numeric_limits<size_t>::max() - offset);

    size_t n_bytes = size + offset;
    void* raw = std::malloc(n_bytes);
    CHECK(raw) << "Memory is not enough to allocate " << n_bytes
               << " bytes space.";

    void* aligned = reinterpret_cast<void*>((reinterpret_cast<size_t>(raw) +
                                             offset) & ~(alignment - 1));
    (reinterpret_cast<void**>(aligned))[-1] = raw;

    CHECK((reinterpret_cast<size_t>(aligned) & (alignment - 1)) == 0);
    return aligned;
}

/**
 * Aligned deallocate.
 */
inline void AlignedDeallocate(void* p) {
    if (p) {
        void* ptr = *(reinterpret_cast<void**>(p) - 1);
        std::free(ptr);
    }
}

} // namespace cl

#endif // CODELIBRARY_BASE_MEMORY_H_
