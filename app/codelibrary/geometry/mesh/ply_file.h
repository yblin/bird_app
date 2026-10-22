//
// Copyright 2024-2026 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//
// The code is adapted from MiniPLY.
//

#ifndef CODELIBRARY_GEOMETRY_MESH_PLY_FILE_H_
#define CODELIBRARY_GEOMETRY_MESH_PLY_FILE_H_

#include <cstring>
#include <string>
#include <utility>

#include "codelibrary/base/array.h"
#include "codelibrary/base/format.h"
#include "codelibrary/geometry/point_3d.h"
#include "codelibrary/string/prefix.h"
#include "codelibrary/util/color/rgb32_color.h"
#include "codelibrary/util/io/binary_stream.h"
#include "codelibrary/util/io/line_reader.h"
#include "codelibrary/util/io/text_stream.h"

namespace cl {
namespace geometry {

/**
 * PLY file for mesh object IO.
 */
class PLYFile {
    static constexpr int PROPERTY_SIZE[]= { 1, 1, 2, 2, 4, 4, 4, 8 };

public:
    enum class PLYFormat {
        UNKNOWN,
        ASCII,
        BINARY_LITTLE_ENDIAN,
        BINARY_BIG_ENDIAN
    };

    enum class PropertyType {
        Int8,
        UInt8,
        Int16,
        UInt16,
        Int,
        UInt,
        Float,
        Double,

        // Special value used in Element::ListCountType to indicate a non-list
        // property.
        None,
    };

    /**
     * PLY property.
     */
    struct Property {
        // Name of property.
        std::string name;

        // Type of the data. Must be set to a value other than None.
        PropertyType type = PropertyType::None;

        // None indicates this is not a list type, otherwise it's the type for
        // the list count.
        PropertyType count_type = PropertyType::None;

        // Raw data for property.
        Array<uint8_t> raw_data;

        // row_count[i] is the number of items in row 'i'. It is only used in
        // list property.
        Array<int> row_count;

        /**
         * Extract data from property.
         */
        template <typename T>
        bool ExtractData(Array<T>* data) const {
            CHECK(data);

            data->clear();
            if (!row_count.empty()) {
                LOG(ERROR) << "Cannot extract fix-sized property from list "
                              "property.";
                return false;
            }

            if (raw_data.empty()) {
                LOG(INFO) << "The given property is empty.";
                return false;
            }

            int n_bytes = PROPERTY_SIZE[static_cast<int>(type)];
            if (raw_data.size() % n_bytes != 0) {
                LOG(ERROR) << "Invalid property data.";
                return false;
            }

            const uint8_t* ptr = raw_data.data();
            while (ptr < raw_data.end()) {
                T v = Convert<T>(ptr);
                data->push_back(v);
                ptr += n_bytes;
            }
            return true;
        }

        /**
         * Extract data from list property.
         */
        template <typename T>
        bool ExtractListData(Array<Array<T>>* data) const {
            CHECK(data);

            data->clear();
            if (row_count.empty() && !raw_data.empty()) {
                LOG(ERROR) << "Cannot extract list property from array "
                              "property.";
                return false;
            }

            if (raw_data.empty()) {
                LOG(INFO) << "The given property is empty.";
                return false;
            }

            int n_bytes = PROPERTY_SIZE[static_cast<int>(type)];
            if (raw_data.size() % n_bytes != 0) {
                LOG(ERROR) << "Invalid property data.";
                return false;
            }

            const uint8_t* ptr = raw_data.data();
            for (int count : row_count) {
                Array<T> t;
                for (int i = 0; i < count; ++i) {
                    T v = Convert<T>(ptr);
                    t.push_back(v);
                    ptr += n_bytes;
                    CHECK(ptr <= raw_data.end());
                }
                data->push_back(t);
            }
            return true;
        }

    private:
        template <class T>
        T Convert(const uint8_t* src) const {
            switch (type) {
            case PropertyType::Int8:
                return static_cast<T>(*reinterpret_cast<const int8_t*>(src));
            case PropertyType::UInt8:
                return static_cast<T>(*reinterpret_cast<const uint8_t*>(src));
            case PropertyType::Int16:
                return static_cast<T>(*reinterpret_cast<const int16_t*>(src));
            case PropertyType::UInt16:
                return static_cast<T>(*reinterpret_cast<const uint16_t*>(src));
            case PropertyType::Int:
                return static_cast<T>(*reinterpret_cast<const int*>(src));
            case PropertyType::UInt:
                return static_cast<T>(*reinterpret_cast<const uint32_t*>(src));
            case PropertyType::Float:
                return static_cast<T>(*reinterpret_cast<const float*>(src));
            case PropertyType::Double:
                return static_cast<T>(*reinterpret_cast<const double*>(src));
            case PropertyType::None:
                break;
            }

            return T();
        }
    };

    /**
     * PLY element.
     */
    struct Element {
        // Name of this element.
        std::string name;
        Array<Property> properties;

        // The number of items in this element (e.g. the number of vertices if
        // this is the vertex element).
        int count = 0;

        /**
         * Get property by name.
         */
        const Property* GetProperty(const std::string& name) const {
            for (const auto& prop : properties) {
                if (prop.name == name) return &prop;
            }
            return nullptr;
        }

        /**
         * Extract 3D points.
         */
        template <typename T>
        bool ExtractPoints(Array<Point3D<T>>* points) const {
            CHECK(points);

            points->clear();

            auto x_property = GetProperty("x");
            if (!x_property) {
                LOG(ERROR) << "Cannot find x property in element " << name
                           << ".";
                return false;
            }
            auto y_property = GetProperty("y");
            if (!y_property) {
                LOG(ERROR) << "Cannot find y property in element " << name
                           << ".";
                return false;
            }
            auto z_property = GetProperty("z");
            if (!z_property) {
                LOG(ERROR) << "Cannot find z property in element " << name
                           << ".";
                return false;
            }

            Array<T> x, y, z;
            if (!x_property->ExtractData(&x)) return false;
            if (!y_property->ExtractData(&y)) return false;
            if (!z_property->ExtractData(&z)) return false;
            CHECK(x.size() == y.size());
            CHECK(y.size() == z.size());

            points->reserve(x.size());
            for (int i = 0; i < x.size(); ++i) {
                points->emplace_back(x[i], y[i], z[i]);
            }
            return true;
        }

        /**
         * Extract 3D normal vectors.
         */
        template <typename T>
        bool ExtractNormals(Array<Vector3D<T>>* normals) const {
            CHECK(normals);

            normals->clear();

            auto x_property = GetProperty("nx");
            if (!x_property) {
                LOG(ERROR) << "Cannot find nx property in element " << name
                           << ".";
                return false;
            }
            auto y_property = GetProperty("ny");
            if (!y_property) {
                LOG(ERROR) << "Cannot find ny property in element " << name
                           << ".";
                return false;
            }
            auto z_property = GetProperty("nz");
            if (!z_property) {
                LOG(ERROR) << "Cannot find nz property in element " << name
                           << ".";
                return false;
            }

            Array<T> x, y, z;
            if (!x_property->ExtractData(&x)) return false;
            if (!y_property->ExtractData(&y)) return false;
            if (!z_property->ExtractData(&z)) return false;
            CHECK(x.size() == y.size());
            CHECK(y.size() == z.size());

            normals->reserve(x.size());
            for (int i = 0; i < x.size(); ++i) {
                normals->emplace_back(x[i], y[i], z[i]);
            }
            return true;
        }

        /**
         * Extract RGB colors.
         */
        bool ExtractColors(Array<RGB32Color>* colors) const {
            CHECK(colors);

            colors->clear();

            auto r_property = GetProperty("r");
            if (!r_property) r_property = GetProperty("red");
            if (!r_property) {
                LOG(ERROR) << "Cannot find r property in element " << name
                           << ".";
                return false;
            }
            auto g_property = GetProperty("g");
            if (!g_property) g_property = GetProperty("green");
            if (!g_property) {
                LOG(ERROR) << "Cannot find g property in element " << name
                           << ".";
                return false;
            }
            auto b_property = GetProperty("b");
            if (!b_property) b_property = GetProperty("blue");
            if (!b_property) {
                LOG(ERROR) << "Cannot find b property in element " << name
                           << ".";
                return false;
            }

            Array<int> r, g, b;
            if (!r_property->ExtractData(&r)) return false;
            if (!g_property->ExtractData(&g)) return false;
            if (!b_property->ExtractData(&b)) return false;
            CHECK(r.size() == g.size());
            CHECK(g.size() == b.size());

            colors->reserve(r.size());
            for (int i = 0; i < r.size(); ++i) {
                colors->emplace_back(r[i], g[i], b[i]);
            }
            return true;
        }
    };

    PLYFile() = default;

    explicit PLYFile(const std::string& filename) {
        Open(filename);
    }

    PLYFile(const PLYFile&) = delete;

    PLYFile& operator=(const PLYFile&) = delete;

    /**
     * Open file for loading.
     */
    bool Open(const std::string& filename) {
        this->Close();

        if (!line_reader_.Open(filename)) {
            return false;
        }

        const char* head = line_reader_.ReadLine();
        if (std::strcmp(head, "ply") != 0) {
            LOG(ERROR) << ErrorMessage("Cannot find keyword 'ply'.");
            return false;
        }

        head = ReadNextLine();
        TextStream stream(head);

        std::string keyword, format;
        stream >> keyword;
        if (keyword != "format") {
            LOG(ERROR) << ErrorMessage("Cannot find keyword 'format'.");
            return false;
        }

        stream >> format;
        if (format != "ascii" && format != "binary_little_endian" &&
            format !=  "binary_big_endian") {
            LOG(ERROR) << ErrorMessage("Unknown format: " + format + ".");
            return false;
        }
        if (format == "ascii") {
            format_ = PLYFormat::ASCII;
        } else if (format == "binary_little_endian") {
            format_ = PLYFormat::BINARY_LITTLE_ENDIAN;
        } else {
            format_ = PLYFormat::BINARY_BIG_ENDIAN;
        }

        std::string version;
        stream >> version;
        if (version.empty()) {
            LOG(ERROR) << ErrorMessage("Cannot find version.");
            return false;
        }

        head = ReadNextLine();
        while (true) {
            TextStream stream(head);
            std::string keyword;
            stream >> keyword;
            if (keyword != "element") {
                break;
            }

            std::string name;
            stream >> name;
            if (name.empty()) {
                LOG(ERROR) << ErrorMessage("Element name is empty.");
                return false;
            }

            int count = 0;
            stream >> count;
            if (count < 0) {
                LOG(ERROR) << ErrorMessage("Invliad count number of element: " +
                                           name + ".");
                return false;
            }

            Element element;
            element.name = name;
            element.count = count;

            do {
                head = ReadNextLine();
                if (!IsKeyword(head, "property")) break;
                ReadProperty(head, &element);
            } while (true);

            elements_.push_back(element);
        }

        if (!IsKeyword(head, "end_header")) {
            LOG(ERROR) << ErrorMessage("Expect 'end_header' here.");
        }

        return true;
    }

    /**
     * Return PLY header information.
     */
    std::string HeaderInfo() const {
        if (!is_open()) {
            return "No open file for loading. Forget to call Open()?";
        }

        static const Array<std::string> property_types = {
            "int8",
            "uint8",
            "int16",
            "uint16",
            "int32",
            "uint32",
            "float",
            "double",
            "none"
        };

        std::string msg = Format("{}\n", format_ == PLYFormat::BINARY_BIG_ENDIAN
                                 ? "binary_big_endian"
                                 : "binary_little_endian");

        for (const auto& element : elements_) {
            msg += Format("Element {} {}\n", element.name, element.count);
            for (const Property& prop : element.properties) {
                if (prop.count_type != PropertyType::None) {
                    msg += Format("Property list {} {} {}\n",
                                  property_types[static_cast<int>(
                                        prop.count_type)],
                                  property_types[static_cast<int>(prop.type)],
                                  prop.name);
                } else {
                    msg += Format("Property {} {}\n",
                                  property_types[static_cast<int>(prop.type)],
                                  prop.name);
                }
            }
        }

        return msg;
    }

    /**
     * Load the raw data of all elements.
     */
    void LoadElements() {
        while (LoadNextElement()) {}
    }

    /**
     * Load the raw data of the next element.
     *
     * Return false if an error occurred or no element can be loaded.
     */
    bool LoadNextElement() {
        if (loaded_element_ + 1 == elements_.size()) return false;
        ++loaded_element_;
        Element& element = elements_[loaded_element_];
        for (Property& prop : element.properties) {
            if (prop.count_type == PropertyType::None) {
                int size = PROPERTY_SIZE[static_cast<int>(prop.type)];
                if (element.count >= INT_MAX / size) {
                    LOG(ERROR) << "The size of property '" << prop.name
                               << "'is too large.";
                    return false;
                }

                prop.raw_data.reserve(size * element.count);
            }
        }

        if (format_ == PLYFormat::ASCII) {
            for (int i = 0; i < element.count; ++i) {
                const char* line = ReadNextLine();
                if (!line) return false;

                TextStream stream(line);
                for (Property& prop : element.properties) {
                    if (prop.count_type == PropertyType::None) {
                        if (!ReadPropertyData(&stream, &prop)) {
                            LOG(ERROR) << ErrorMessage("");
                            return false;
                        }
                    } else {
                        int count = 0;
                        stream >> count;
                        if (count < 0) {
                            LOG(ERROR) << ErrorMessage("Invalid count number.");
                            return false;
                        }
                        prop.row_count.push_back(count);
                        for (int i = 0; i < count; ++i) {
                            if (!ReadPropertyData(&stream, &prop)) {
                                LOG(ERROR) << ErrorMessage("");
                                return false;
                            }
                        }
                    }
                }
            }
        } else {
            uint8_t data[8];
            for (int i = 0; i < element.count; ++i) {
                for (Property& prop : element.properties) {
                    if (prop.count_type == PropertyType::None) {
                        int n_bytes =
                            PROPERTY_SIZE[static_cast<int>(prop.type)];
                        ReadBytes(n_bytes, data);
                        prop.raw_data.insert(data, data + n_bytes);
                    } else {
                        int n_bytes =
                            PROPERTY_SIZE[static_cast<int>(prop.count_type)];
                        ReadBytes(n_bytes, data);

                        int count;
                        BinaryStream stream(data, data + n_bytes);
                        stream >> count;
                        if (count < 0) {
                            LOG(ERROR) << "Invalid count of list property: "
                                       << prop.name;
                            return false;
                        }
                        prop.row_count.push_back(count);

                        n_bytes = PROPERTY_SIZE[static_cast<int>(prop.type)];
                        for (int i = 0; i < count; ++i) {
                            ReadBytes(n_bytes, data);
                            prop.raw_data.insert(data, data + n_bytes);
                        }
                    }
                }
            }
        }
        return true;
    }

    /**
     * Get element by name.
     */
    const Element* GetElement(const std::string& name) const {
        for (auto& element : elements_) {
            if (element.name == name) return &element;
        }

        return nullptr;
    }

    /**
     * Check if a PLY file is open for loading.
     */
    bool is_open() const {
        return format_ != PLYFormat::UNKNOWN;
    }

    /**
     * Close the open file.
     */
    void Close() {
        loaded_element_ = -1;
        format_ = PLYFormat::UNKNOWN;
        version_.clear();
        elements_.clear();
        line_reader_.Close();
    }

    PLYFormat format() const {
        return format_;
    }

    std::string version() const {
        return version_;
    }

private:
    /**
     * Return true if the line has a keyword.
     */
    static bool IsKeyword(const char* line, const char* keyword) {
        if (HasPrefix(line, line + std::strlen(line),
                      keyword, keyword + std::strlen(keyword))) {
            char c = line[std::strlen(keyword)];
            if (!std::isalnum(c) && c != '_') return true;
        }
        return false;
    }

    /**
     * Assign the type by name.
     */
    static bool SetType(const std::string& name, PropertyType* type) {
        CHECK(type);

        static const Array<std::pair<std::string, PropertyType>> types = {
            { "char",    PropertyType::Int8   },
            { "uchar",   PropertyType::UInt8  },
            { "short",   PropertyType::Int16  },
            { "ushort",  PropertyType::UInt16 },
            { "int",     PropertyType::Int    },
            { "uint",    PropertyType::UInt   },
            { "float",   PropertyType::Float  },
            { "float32", PropertyType::Float  },
            { "float64", PropertyType::Double },
            { "double",  PropertyType::Double },

            { "uint8",   PropertyType::UInt8  },
            { "uint16",  PropertyType::UInt16 },
            { "uint32",  PropertyType::UInt   },

            { "int8",    PropertyType::Int8   },
            { "int16",   PropertyType::Int16  },
            { "int32",   PropertyType::Int    }
        };

        for (int i = 0; i < types.size(); ++i) {
            if (types[i].first == name) {
                *type = types[i].second;
                return true;
            }
        }
        return false;
    }

    /**
     * Endian swap data.
     */
    static void EndianSwap(uint8_t* data, int n_bytes) {
        switch (n_bytes) {
        case 2:
            endian_util::Swap2(data);
            break;
        case 4:
            endian_util::Swap4(data);
            break;
        case 8:
            endian_util::Swap8(data);
            break;
        default: break;
        }
    }

    /**
     * Read next valid line.
     */
    const char* ReadNextLine() {
        const char* head;
        do {
            head = line_reader_.ReadLine();
        } while (IsKeyword(head, "comment") || IsKeyword(head, "obj_info"));
        return head;
    }

    /**
     * Return common error message.
     */
    std::string ErrorMessage(const std::string& message) {
        return Format("Invalid PLY format at line {}. {}",
                      line_reader_.n_line() - 1, message);
    }

    /**
     * Read n-bytes.
     */
    bool ReadBytes(int n_bytes, uint8_t* data) {
        CHECK(n_bytes > 0);

        auto begin = reinterpret_cast<const uint8_t*>(line_reader_.begin());
        auto end = reinterpret_cast<const uint8_t*>(line_reader_.end());
        if (begin + n_bytes >= end) {
            line_reader_.ReadNextBuffer();
            begin = reinterpret_cast<const uint8_t*>(line_reader_.begin());
            end = reinterpret_cast<const uint8_t*>(line_reader_.end());
            if (begin + n_bytes >= end) return false;
        }

        std::memcpy(data, begin, n_bytes);
        if (format_ == PLYFormat::BINARY_BIG_ENDIAN &&
            endian_util::GetEndian() == endian_util::Endian::LITTLE) {
            EndianSwap(data, n_bytes);
        }

        line_reader_.Seek(line_reader_.data_begin() + n_bytes);

        return true;
    }

    /**
     * Read a item for 'prop' from 'stream'.
     */
    bool ReadPropertyData(TextStream* stream, Property* prop) {
        CHECK(prop);

        switch (prop->type) {
        case PropertyType::UInt8:
        case PropertyType::Int8:
        {
            uint8_t t;
            if (!stream->ReadInteger(&t)) return false;

            prop->raw_data.push_back(t);
            break;
        }
        case PropertyType::UInt16:
        case PropertyType::Int16:
        {
            uint16_t t;
            if (!stream->ReadInteger(&t)) return false;

            uint8_t* ptr = reinterpret_cast<uint8_t*>(&t);
            prop->raw_data.push_back(ptr[0]);
            prop->raw_data.push_back(ptr[1]);
            break;
        }
        case PropertyType::Int:
        case PropertyType::UInt:
        {
            uint32_t t;
            if (!stream->ReadInteger(&t)) return false;

            uint8_t* ptr = reinterpret_cast<uint8_t*>(&t);
            prop->raw_data.push_back(ptr[0]);
            prop->raw_data.push_back(ptr[1]);
            prop->raw_data.push_back(ptr[2]);
            prop->raw_data.push_back(ptr[3]);
            break;
        }
        case PropertyType::Float:
        {
            float t;
            if (!stream->ReadFloat(&t)) return false;

            uint8_t* ptr = reinterpret_cast<uint8_t*>(&t);
            prop->raw_data.push_back(ptr[0]);
            prop->raw_data.push_back(ptr[1]);
            prop->raw_data.push_back(ptr[2]);
            prop->raw_data.push_back(ptr[3]);
            break;
        }
        case PropertyType::Double:
        {
            double t;
            if (!stream->ReadFloat(&t)) return false;

            uint8_t* ptr = reinterpret_cast<uint8_t*>(&t);
            prop->raw_data.push_back(ptr[0]);
            prop->raw_data.push_back(ptr[1]);
            prop->raw_data.push_back(ptr[2]);
            prop->raw_data.push_back(ptr[3]);
            prop->raw_data.push_back(ptr[4]);
            prop->raw_data.push_back(ptr[5]);
            prop->raw_data.push_back(ptr[6]);
            prop->raw_data.push_back(ptr[7]);
            break;
        }
        default:
            CHECK(false);
        }

        return true;
    }

    /**
     * Read property from header.
     */
    bool ReadProperty(const char* line, Element* element) {
        CHECK(element);

        TextStream stream(line);

        std::string keyword;
        stream >> keyword;
        if (keyword != "property") {
            LOG(ERROR) << ErrorMessage("Cannot find 'property'.");
            return false;
        }

        Property property;
        stream >> keyword;
        if (keyword == "list") {
            // This is a list property.
            std::string type_name;
            stream >> type_name;

            if (!SetType(type_name, &property.count_type)) {
                LOG(ERROR) << ErrorMessage("Unknown property type: " +
                                           type_name + ".");
                return false;
            }
            if (property.count_type == PropertyType::Float ||
                property.count_type == PropertyType::Double) {
                LOG(ERROR) << ErrorMessage("Expect an integer count type, but "
                                           "got: " + type_name + ".");
                return false;
            }

            stream >> keyword;
        }

        if (!SetType(keyword, &property.type)) {
            LOG(ERROR) << ErrorMessage("Unknown property type: " +
                                       keyword + ".");
            return false;
        }

        stream >> property.name;
        if (property.name.empty()) {
            LOG(ERROR) << ErrorMessage("Empty property name.");
            return false;
        }

        element->properties.push_back(property);
        return true;
    }

    // Current loaded element. Used by LoadNextElement().
    int loaded_element_ = -1;
    PLYFormat format_ = PLYFormat::UNKNOWN;
    std::string version_;
    io::LineReader line_reader_;
    Array<Element> elements_;
};

} // namespace geometry
} // namespace cl

#endif // CODELIBRARY_GEOMETRY_MESH_PLY_FILE_H_
