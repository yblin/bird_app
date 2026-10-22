//
// Copyright 2022-2023 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_UTIL_IO_FILE_UTIL_H_
#define CODELIBRARY_UTIL_IO_FILE_UTIL_H_

#include <algorithm>
#include <fstream>
#include <string>

#include "codelibrary/base/log.h"

namespace cl {
namespace file_util {

/**
 * Get file suffix and return it in lowercase.
 */
inline std::string GetSuffix(const std::string& filename) {
    std::size_t index = filename.find_last_of('.');
    if (index == std::string::npos) return "";
    std::string suffix = filename.substr(index + 1);
    std::transform(suffix.begin(), suffix.end(), suffix.begin(), ::tolower);
    return suffix;
}

/**
 * Get file basename.
 */
inline std::string GetBaseName(const std::string& filename) {
    return filename.substr(filename.find_last_of("/\\") + 1);
}

/**
 * Stem + suffix = base name.
 */
inline std::string GetStemName(const std::string& filename) {
    std::string base_name = GetBaseName(filename);

    std::size_t index = base_name.find_last_of('.');
    if (index != std::string::npos) {
        return base_name.substr(0, index);
    } else {
        // No dot found, return the entire filename as the stem.
        return base_name;
    }
}

/**
 * Get the folder name of the file path.
 */
inline std::string GetFolderName(const std::string& path) {
    size_t pos = path.find_last_of("/\\");
    if (pos == std::string::npos) {
        return "./";
    }
    return path.substr(0, pos) + path[pos];;
}

/**
 * Get the bytes of the file.
 */
inline size_t GetFileBytes(const std::string& filename) {
    FILE* file = std::fopen(filename.c_str(), "rb");
    if (!file) return 0;
    std::fseek(file, 0, SEEK_END);
    size_t file_bytes = std::ftell(file);
    std::fclose(file);
    return file_bytes;
}

/**
 * Read the contents of a file into string.
 */
inline bool GetContent(const std::string& filepath, std::string* content) {
    CHECK(content);

    // Open the file in binary mode and position the file pointer at the end
    // This helps in quickly determining the file size.
    std::ifstream file(filepath, std::ios::in | std::ios::binary |
                                 std::ios::ate);

    if (!file.is_open()) {
        LOG(ERROR) << "Could not open file: " << filepath;
        return false;
    }

    std::streamsize size = file.tellg();

    // Obtain the file size
    file.seekg(0, std::ios::beg);

    content->resize(size, '\0');

    // Read the file content directly into the string buffer.
    file.read(&(*content)[0], size);

    return true;
}

/**
 * Save the content of a string into a file.
 */
inline bool SaveContent(const std::string& filepath,
                        const std::string& content) {
    std::ofstream file(filepath, std::ios::out | std::ios::binary);
    if (!file.is_open()) {
        LOG(ERROR) << "Could not open file for writing: " << filepath;
        return false;
    }
    // Write entire string content to file.
    file.write(content.data(), content.size());
    return true;
}

/**
 * Return true if the file is exist.
 */
inline bool IsFileExist(const std::string& filename) {
    std::ifstream f(filename.c_str());
    return f.good();
}

/**
 * Implement c++17 filesystem::copy_file.
 */
inline bool CopyFile(const std::string& source,
                     const std::string& destination) {
    CHECK(source != destination);

    std::ifstream src(source, std::ios::binary);
    if (!src.is_open()) {
        LOG(ERROR) << "Failed to open source file: " << source;
        return false;
    }

    std::ofstream dest(destination, std::ios::binary);
    if (!dest.is_open()) {
        LOG(ERROR) << "Failed to open destination file: " << destination;
        return false;
    }

    // Copy the content.
    dest << src.rdbuf();

    if (!dest) {
        LOG(ERROR) << "Error occurred during file copy.";
        return false;
    }

    return true;
}

} // namespace file_util
} // namespace cl

#endif // CODELIBRARY_UTIL_IO_FILE_UTIL_H_
