//
// Copyright 2016-2024 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_POINT_CLOUD_IO_XYZ_IO_H_
#define CODELIBRARY_POINT_CLOUD_IO_XYZ_IO_H_

#include <string>
#include <fstream>

#include "codelibrary/base/array.h"
#include "codelibrary/geometry/point_3d.h"
#include "codelibrary/string/string_split.h"
#include "codelibrary/util/io/line_reader.h"
#include "codelibrary/util/io/text_stream.h"
#include "codelibrary/util/color/rgb32_color.h"

#include "third_party/fmt/os.h"

namespace cl {
namespace point_cloud {

/**
 * XYZ (ASCII) point cloud file loader.
 */
class XYZLoader {
    enum VertexFormat {
        UNKNOWN,
        XYZ,
        XYZ_RGB,
        XYZ_RGB_NORMAL
    };

public:
    XYZLoader() = default;

    explicit XYZLoader(const std::string& filename) {
        Open(filename);
    }

    XYZLoader(const XYZLoader&) = delete;

    XYZLoader& operator=(const XYZLoader&) = delete;

    /**
     * Open file for loading.
     */
    bool Open(const std::string& filename) {
        if (!line_reader_.Open(filename)) return false;
        head_line_ = line_reader_.ReadLine();
        format_ = GetFormat(head_line_);
        return true;
    }

    /**
     * Check if a XYZ file is opened for loading.
     */
    bool is_open() const {
        return line_reader_.is_open();
    }

    /**
     * Close the open file.
     */
    void Close() {
        format_ = UNKNOWN;
        head_line_ = nullptr;
        line_reader_.Close();
    }

    /**
     * Load point cloud data from line reader.
     */
    template <typename T>
    bool Load(Array<Point3D<T>>* points,
              Array<RGB32Color>* colors = nullptr,
              Array<Vector3D<T>>* normals = nullptr) {
        static_assert(std::is_floating_point<T>::value, "");

        CHECK(points);
        if (!is_open()) {
            LOG(INFO) << "No open file for loading. Forget to call Open()?";
            return false;
        }

        points->clear();
        if (colors) colors->clear();
        if (normals) normals->clear();
        if (format_ == UNKNOWN) {
            LOG(ERROR) << "Unsupported XYZ format.";
            return false;
        }

        char* line = head_line_;
        do {
            if (!ReadPoint(line, points, colors, normals)) return false;
        } while ((line = line_reader_.ReadLine()));

        return true;
    }
    template <typename T>
    bool Load(Array<Point3D<T>>* points,
              Array<Vector3D<T>>* normals) const {
        return Load(points, nullptr, normals);
    }

    /**
     * Continuously load point cloud data. Load n points at each time.
     *
     * Return the number of points actually read.
     */
    template <typename T>
    int SuccessiveLoad(int n_points,
                       Array<Point3D<T>>* points,
                       Array<RGB32Color>* colors = nullptr,
                       Array<Vector3D<T>>* normals = nullptr)  {
        static_assert(std::is_floating_point<T>::value, "");

        CHECK(points);
        CHECK(n_points > 0);

        if (!is_open()) return 0;
        if (format_ == UNKNOWN) {
            LOG(ERROR) << "Unsupported XYZ format.";
            return 0;
        }

        int n = 0;
        if (line_reader_.n_line() == 1) {
            char* line = head_line_;
            do {
                if (!ReadPoint(line, points, colors, normals)) return n;
                if (++n == n_points) break;
            } while ((line = line_reader_.ReadLine()));
        } else {
            char* line = nullptr;
            while ((line = line_reader_.ReadLine())) {
                if (!ReadPoint(line, points, colors, normals)) return n;
                if (++n == n_points) break;
            }
        }

        return n;
    }
    template <typename T>
    int SuccessiveLoad(int n_points,
                       Array<Point3D<T>>* points,
                       Array<Vector3D<T>>* normals)  {
        return SuccessiveLoad(n_points, points, nullptr, normals);
    }

    /**
     * Return the first line of XYZ file.
     */
    const char* head_line() const {
        return head_line_;
    }

    /**
     * Return the number of read bytes.
     */
    size_t n_read_bytes() const {
        return line_reader_.n_read_bytes();
    }

    /**
     * Return the total number of bytes.
     */
    size_t n_total_bytes() const {
        return line_reader_.n_total_bytes();
    }

private:
    /**
     * Evaluate the vertex format of XYZ file.
     */
    static VertexFormat GetFormat(const char* line) {
        if (line == nullptr) return UNKNOWN;

        Array<std::string> items;
        StringSplit(line, ' ', &items);

        VertexFormat format = UNKNOWN;
        if (items.size() == 3) {
            format = XYZ;
        } else if (items.size() >= 6) {
            if (Check8Bit(items[3]) && Check8Bit(items[4]) &&
                Check8Bit(items[5])) {
                format = (items.size() >= 9) ? XYZ_RGB_NORMAL : XYZ_RGB;
            }
        }

        return format;
    }

    /**
     * Check if the integer is an valid 8-bits color.
     */
    static bool Check8Bit(const std::string& str) {
        int n = 0;
        for (size_t i = 0; i < str.size(); ++i) {
            if (!std::isdigit(str[i])) return false;
            n = n * 10 + str[i] - '0';
            if (n < 0 || n > 255) return false;
        }
        return true;
    }

    /**
     * Read point from 'line'.
     */
    template <typename T>
    bool ReadPoint(const char* line,
                   Array<Point3D<T>>* points,
                   Array<RGB32Color>* colors,
                   Array<Vector3D<T>>* normals) const {
        static_assert(std::is_floating_point<T>::value, "");

        T x, y, z;
        T nx, ny, nz;
        int r, g, b;

        TextStream stream(line);

        switch (format_) {
        case UNKNOWN:
            CHECK(false) << "Unknown format.";
            break;
        case XYZ:
            if (!stream.ReadFloat(&x) || !stream.ReadFloat(&y) ||
                !stream.ReadFloat(&z)) {
                LOG(ERROR) << ErrorMessage();
                return false;
            }
            points->emplace_back(x, y, z);
            break;

        case XYZ_RGB:
            if (!stream.ReadFloat(&x) || !stream.ReadFloat(&y) ||
                !stream.ReadFloat(&z)) {
                LOG(ERROR) << ErrorMessage();
                return false;
            }
            points->emplace_back(x, y, z);
            if (colors) {
                if (!stream.ReadInteger(&r) || !stream.ReadInteger(&g) ||
                    !stream.ReadInteger(&b)) {
                    LOG(ERROR) << ErrorMessage();
                    return false;
                }
                colors->emplace_back(r, g, b);
            }
            break;

        case XYZ_RGB_NORMAL:
            if (!stream.ReadFloat(&x) || !stream.ReadFloat(&y) ||
                !stream.ReadFloat(&z)) {
                LOG(ERROR) << ErrorMessage();
                return false;
            }
            points->emplace_back(x, y, z);
            if (normals) {
                if (!stream.ReadInteger(&r) || !stream.ReadInteger(&g) ||
                    !stream.ReadInteger(&b)) {
                    LOG(ERROR) << ErrorMessage();
                    return false;
                }
                if (!stream.ReadFloat(&nx) || !stream.ReadFloat(&ny) ||
                    !stream.ReadFloat(&nz)) {
                    LOG(ERROR) << ErrorMessage();
                    return false;
                }

                if (colors) colors->emplace_back(r, g, b);
                normals->emplace_back(nx, ny, nz);
            } else if (colors) {
                if (!stream.ReadInteger(&r) || !stream.ReadInteger(&g) ||
                    !stream.ReadInteger(&b)) {
                    LOG(ERROR) << ErrorMessage();
                    return false;
                }
                colors->emplace_back(r, g, b);
            }
            break;
        default:
            CHECK(false);
            break;
        }

        return true;
    }

    /**
     * Return common error message.
     */
    std::string ErrorMessage() const {
        return Format("Invalid XYZ format at line: {}", line_reader_.n_line());
    }

    char* head_line_ = nullptr;
    io::LineReader line_reader_;
    VertexFormat format_ = UNKNOWN;
};

/**
 * Write points into XYZ file.
 */
template <typename T>
bool WriteXYZPoints(const std::string& filename,
                    const Array<Point3D<T>>& points) {
    static_assert(std::is_floating_point<T>::value, "");

    std::ofstream out(filename, std::ios::trunc);
    for (const Point3D<T>& p : points) {
        out << Format("{} {} {}\n", p.x, p.y, p.z);
    }
    return true;
}

/**
 * Write color points into XYZ file.
 */
template <typename T>
bool WriteXYZPoints(const std::string& filename,
                    const Array<Point3D<T>>& points,
                    const Array<RGB32Color>& colors) {
    static_assert(std::is_floating_point<T>::value, "");

    CHECK(points.size() == colors.size());

    std::ofstream out(filename, std::ios::trunc);

    for (int i = 0; i < points.size(); ++i) {
        const Point3D<T>& p = points[i];
        const RGB32Color& c = colors[i];
        out << Format("{} {} {} {} {} {}\n", p.x, p.y, p.z,
                      c.red(), c.green(), c.blue());
    }

    return true;
}

/**
 * Write oriented points into XYZ file.
 */
template <typename T>
bool WriteXYZPoints(const std::string& filename,
                    const Array<Point3D<T>>& points,
                    const Array<Vector3D<T>>& normals) {
    static_assert(std::is_floating_point<T>::value, "");

    CHECK(points.size() == normals.size());

    std::ofstream out(filename, std::ios::trunc);

    for (int i = 0; i < points.size(); ++i) {
        const Point3D<T>& p = points[i];
        const Vector3D<T>& n = normals[i];
        out << Format("{} {} {} {} {} {}\n", p.x, p.y, p.z, n.x, n.y, n.z);
    }

    return true;
}

} // namespace point_cloud
} // namespace cl

#endif // CODELIBRARY_POINT_CLOUD_IO_XYZ_IO_H_
