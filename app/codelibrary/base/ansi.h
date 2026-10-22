//
// Copyright 2023-2024 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_BASE_ANSI_H_
#define CODELIBRARY_BASE_ANSI_H_

#ifdef _MSC_VER
#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS
#endif
#endif

#ifdef _WIN32
#   ifndef NOMINMAX
#   define NOMINMAX
#   endif
#   ifndef ENABLE_VIRTUAL_TERMINAL_PROCESSING
#   define ENABLE_VIRTUAL_TERMINAL_PROCESSING 0x0004
#   endif
#   include <Windows.h>
#else
#   include <sys/ioctl.h>
#   include <unistd.h>
#endif

#include <cstdlib>

namespace cl {
namespace ansi {

/**
 * Support ANSI escape code?
 * ANSI escape sequences are a standard for in-band signaling to control cursor
 * location, color, font styling, and other options on terminals.
 */
inline bool SupportAnsiEscapeCode() {
    char* no_color_env = getenv("NO_COLOR");
    if (no_color_env && no_color_env[0] != '\0') {
        return false;
    }

#ifdef _WIN32
    // Set output mode to handle virtual terminal sequences
    HANDLE hOut = GetStdHandle(STD_OUTPUT_HANDLE);
    if (hOut == INVALID_HANDLE_VALUE) {
        return false;
    }
    DWORD dw_mode = 0;
    if (!GetConsoleMode(hOut, &dw_mode)) {
        return false;
    }
    dw_mode |= ENABLE_VIRTUAL_TERMINAL_PROCESSING;
    if (!SetConsoleMode(hOut, dw_mode)) {
        return false;
    }
#endif
    return true;
}

constexpr const char* RED        = "\x1B[31m";
constexpr const char* ORANGE_RED = "\x1B[38;5;203m";
constexpr const char* YELLOW     = "\x1B[33m";
constexpr const char* MAGENTA    = "\x1B[35m";
constexpr const char* CYAN       = "\x1B[36m";
constexpr const char* WHITE      = "\x1B[37m";
constexpr const char* RESET      = "\x1B[0m";

} // namespace ansi
} // namespace cl

#endif // CODELIBRARY_BASE_ANSI_H_
