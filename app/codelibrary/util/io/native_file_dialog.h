//
// Copyright 2022-2026 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_UTIL_IO_NATIVE_FILE_DIALOG_H_
#define CODELIBRARY_UTIL_IO_NATIVE_FILE_DIALOG_H_

#include <string>

#include "codelibrary/base/log.h"

#include "third_party/native_file_dialog/nfd.h"

namespace cl {
namespace io {

/**
 * Get single open file by native open dialog.
 *
 * Filters is splited by ';'.
 */
inline std::string OpenFileDialog(const std::string& default_path = "",
                                  const std::string& filters = "") {
    nfdchar_t* out_path = nullptr;
    nfdresult_t result = NFD_OpenDialog(filters.c_str(), default_path.c_str(),
                                        &out_path);
    if (result == NFD_OKAY) {
        std::string path(out_path);
        free(out_path);
        return path;
    }

    if (result == NFD_CANCEL) {
        return "";
    }

    LOG(ERROR) << NFD_GetError();
    return "";
}

/**
 * Save file by native open dialog.
 */
inline std::string SaveFileDialog(const std::string& default_path = "",
                                  const std::string& filters = "") {
    nfdchar_t* out_path = nullptr;
    nfdresult_t result = NFD_SaveDialog(filters.c_str(), default_path.c_str(),
                                        &out_path);
    if (result == NFD_OKAY) {
        std::string path(out_path);
        free(out_path);
        return path;
    }

    if (result == NFD_CANCEL) {
        return "";
    }

    LOG(ERROR) << NFD_GetError();
    return "";
}

} // namespace io
} // namespace cl

#endif // CODELIBRARY_UTIL_IO_NATIVE_FILE_DIALOG_H_
