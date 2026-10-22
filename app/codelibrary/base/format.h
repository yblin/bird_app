//
// Copyright 2025-2026 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_BASE_FORMAT_H_
#define CODELIBRARY_BASE_FORMAT_H_

#include <iostream>

// =============================================================================
// Cross-compiler/OS Base Detection (Adjust MSVC handling: legacy -> force fmt)
// =============================================================================
#if defined(_MSC_VER)
// Microsoft Visual C++:
// - Version <1920 (VS2019): Force use fmt::format (no std::format support)
// - Version >=1920 (VS2019+): Support both std::format (C++20+) and fmt::format
#  if _MSC_VER < 1920
#    define CL_FORCE_LEGACY_MSVC_FMT 1
#  endif
#elif defined(__GNUC__) || defined(__GNUG__)
// GCC: Minimum recommended version 11.0+ (basic C++20 support)
// - Versions <11: Warn and force fmt (std::format unavailable)
#  if __GNUC__ < 11
#    define CL_FORCE_LEGACY_GCC_FMT 1
#  endif
#elif defined(__clang__)
// Clang: Minimum recommended version 12.0+ (basic C++20 support)
// - Versions <12: Warn and force fmt (std::format unavailable)
#  if __clang_major__ < 12
#    define CL_FORCE_LEGACY_CLANG_FMT 1
#  endif
#else
#  define CL_FORCE_UNKNOWN_COMPILER_FMT 1
#endif

// Unified C++ version macro (MSVC requires /Zc:__cplusplus for correct
// __cplusplus)
#if defined(_MSC_VER)
#  define CL_CPP_VERSION _MSVC_LANG
#else
#  define CL_CPP_VERSION __cplusplus
#endif

// Core logic: Determine if std::format is usable (override for legacy
// compilers).
#define CL_LEGACY_COMPILER (defined(CL_FORCE_LEGACY_MSVC_FMT)  ||   \
                            defined(CL_FORCE_LEGACY_GCC_FMT)   ||   \
                            defined(CL_FORCE_LEGACY_CLANG_FMT) ||   \
                            defined(CL_FORCE_UNKNOWN_COMPILER_FMT))

// VS2022 Special Case: Versions 17.0+ (_MSC_VER >= 1930) have working <format>
// but may lack __cpp_lib_format
#if defined(_MSC_VER) && _MSC_VER >= 1930 && CL_CPP_VERSION >= 202002L && \
    __has_include(<format>)
#  define CL_MSVC2022_COMPATIBLE
#endif

#if defined(CL_FORCE_USE_STD_FORMAT)
// User forced std::format: Reject if legacy compiler (no std::format support)
#  if CL_LEGACY_COMPILER
#    error "CL_FORCE_USE_STD_FORMAT is enabled but legacy compiler detected" \
           "(std::format not supported). Use fmt instead."
#  elif !(CL_MSVC2022_COMPATIBLE || \
        (CL_CPP_VERSION >= 202002L && __has_include(<format>) && \
         defined(__cpp_lib_format) && __cpp_lib_format >= 202106L))
#    error "CL_FORCE_USE_STD_FORMAT is enabled but <format> is not supported " \
           "(upgrade VS2022 to 17.0+ or enable C++20)"
#  endif
#  define CL_USE_STD_FORMAT 1
#  define CL_USE_FMT_FORMAT 0
#elif defined(CL_FORCE_USE_FMT_FORMAT)
// Force fmt: User request OR legacy compiler (no std::format option)
#  define CL_USE_STD_FORMAT 0
#  define CL_USE_FMT_FORMAT 1
#else
// Auto-detection:
// - For VS2022 17.0+: Use std::format if C++20+ and <format> exists (bypass
//   __cpp_lib_format check)
// - For other compilers: Use original 4-condition check
#  if defined(CL_MSVC2022_COMPATIBLE) || \
      (CL_CPP_VERSION >= 202002L && __has_include(<format>) && \
       defined(__cpp_lib_format) && __cpp_lib_format >= 202106L)
#    define CL_USE_STD_FORMAT 1
#    define CL_USE_FMT_FORMAT 0
#  else
#    define CL_USE_STD_FORMAT 0
#    define CL_USE_FMT_FORMAT 1
#  endif
#endif

#if CL_USE_STD_FORMAT
# include <ctime>
# include <format>
# include <string>
#else
// Use fmt::format: Compatible with system and third-party paths (no OS binding)
#  include <string>

// Third-party fmt: User must add third_party directory to compiler's additional
// include paths.
#  if !__has_include(<third_party/fmt/core.h>)
#    error "fmt headers not found: Configure system fmt or add third_party " \
           "to additional include directories"
#  endif
// Enable fmt header-only mode (avoids library linkage dependency, more
// cross-platform friendly)
#  if !defined(FMT_HEADER_ONLY)
#    define FMT_HEADER_ONLY
#  endif
#  include <third_party/fmt/chrono.h>
#  include <third_party/fmt/core.h>
#  include <third_party/fmt/format.h>

// Check fmt version (minimum support 7.0.0, compatible with major versions)
#  if !defined(FMT_VERSION) || FMT_VERSION < 70000
#    error "fmt version too old: cl::Format requires fmt >= 7.0.0"
#  endif
#endif

namespace cl {

/**
 * Core Interface: Narrow character formatting (compatible with std::format /
 * fmt::format).
 */
template <typename... Args>
[[nodiscard]] std::string Format(
#if CL_USE_STD_FORMAT
    std::format_string<Args...>
#else
    fmt::format_string<Args...>
#endif
        fmt_str, Args&&... args) {
#if CL_USE_STD_FORMAT
  return std::format(fmt_str, std::forward<Args>(args)...);
#else
  return fmt::format(fmt_str, std::forward<Args>(args)...);
#endif
}

/**
 * Equivalent to fmt::to_string: Convert single value to string with default
 * formatting (narrow char).
 */
template <typename T>
[[nodiscard]] std::string ToString(const T& value) {
#if CL_USE_STD_FORMAT
  return std::format("{}", value);
#else
  return fmt::to_string(value);
#endif
}

/**
 * Cross-backend print function.
 */
template <typename... Args>
inline void Print(const std::string& format_str, Args&&... args) {
#if CL_USE_STD_FORMAT
    printf("%s", cl::Format(format_str, std::forward<Args>(args)...).c_str());
#else
    fmt::print(format_str, std::forward<Args>(args)...);
#endif
}

/**
 * Thread-safe note: Both fmt::localtime and std::localtime are NOT thread-safe.
 */
inline std::tm LocalTime(std::time_t tt) {
#if CL_USE_STD_FORMAT
    // std::format uses std::tm (compatible with fmt's time format)
    return *std::localtime(&tt);
#else
    // fmt backend: use native fmt::localtime
    return fmt::localtime(tt);
#endif
}

/**
 * Runtime Support Check (Optional: Verify implementation at runtime).
 */
inline void PrintFormatSupportDetails() {
    std::cout << "====== cl::Format: Support Check Results ======" << std::endl;
    std::cout << "1. Compiler: "

#if defined(_MSC_VER)
              << "MSVC (v" << _MSC_VER << ")"
#elif defined(__GNUC__)
              << "GCC (" << __GNUC__ << "." << __GNUC_MINOR__ << ")"
#elif defined(__clang__)
              << "Clang (" << __clang_major__ << "." << __clang_minor__ << ")"
#else
              << "Unknown"
#endif
              << std::endl;

    std::cout << "2. C++ Version: "
              << ((CL_CPP_VERSION >= 202002L) ? "C++20+" : "C++17 or older")
              << " (CL_CPP_VERSION = " << CL_CPP_VERSION << ")" << std::endl;

#if __has_include(<format>)
    std::cout << "3. <format> Header Found: " << "YES" << std::endl;
#else
    std::cout << "3. <format> Header Found: " << "NO" << std::endl;
#endif

#if defined(CL_MSVC2022_COMPATIBLE)
    std::cout << "4. VS2022 Compatible: " << "YES" << std::endl;
#else
    std::cout << "4. VS2022 Compatible: " << "NO" << std::endl;
#endif

    std::cout << "5. __cpp_lib_format Defined: ";
#if defined(__cpp_lib_format)
    std::cout << "YES (value = " << __cpp_lib_format << ")";
#else
    std::cout << "NO";
#endif
    std::cout << std::endl;

#if defined(CL_FORCE_LEGACY_MSVC_FMT) || defined(CL_FORCE_LEGACY_GCC_FMT) || \
    defined(CL_FORCE_LEGACY_CLANG_FMT) || defined(CL_FORCE_UNKNOWN_COMPILER_FMT)
    std::cout << "6. Legacy Compiler: " << "YES" << std::endl;
#else
    std::cout << "6. Legacy Compiler: " << "NO" << std::endl;
#endif
    std::cout << "7. Using Implementation: "
              << (CL_USE_STD_FORMAT ? "std::format" : "fmt::format")
              << std::endl;
    std::cout << "===============================================" << std::endl;
}

}  // namespace cl

// =============================================================================
// Cleanup Temporary Macros (Avoid polluting global namespace)
#undef CL_CPP_VERSION
#undef CL_FORCE_LEGACY_MSVC_FMT
#undef CL_FORCE_LEGACY_GCC_FMT
#undef CL_FORCE_LEGACY_CLANG_FMT
#undef CL_FORCE_UNKNOWN_COMPILER_FMT

#endif  // CODELIBRARY_BASE_FORMAT_H_
