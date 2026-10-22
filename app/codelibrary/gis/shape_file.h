//
// Copyright 2024-2025 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GIS_SHAPE_FILE_H_
#define CODELIBRARY_GIS_SHAPE_FILE_H_

#include <algorithm>
#include <string>

#include "codelibrary/base/array.h"
#include "codelibrary/base/log.h"
#include "codelibrary/geometry/multi_polygon_2d.h"
#include "codelibrary/geometry/polyline.h"
#include "codelibrary/geometry/topology/even_odd_polygon_2d.h"
#include "codelibrary/util/io/file_util.h"
#include "codelibrary/util/io/text_stream.h"

#include "third_party/shapelib/shapefil.h"

namespace cl {
namespace gis {

/**
 * Field for shape file.
 */
struct ShapeField {
    DBFFieldType type;
    std::string name;
    int width;
    int precision;
};

/**
 * ESRI shape file based on shapelib.
 */
class ShapeFile {
public:
    ShapeFile() = default;

    explicit ShapeFile(const std::string& path) {
        Open(path);
    }

    virtual ~ShapeFile() {
        this->Close();
    }

    /**
     * Open shape file (.shp and .shx) for reading and writting fields.
     *
     * Note that both files have to be in the indicated directory, and must have
     * the expected extensions in lower case.
     */
    bool Open(const std::string& path) {
        filepath_ = path;
        handle_ = SHPOpen(filepath_.c_str(), "rb");
        if (!handle_) {
            LOG(ERROR) << "Open shape file \"" << path << "\" failed.";
            return false;
        }

        dbf_handle_ = DBFOpen(filepath_.c_str(), "rb+");       
        if (!dbf_handle_) {
            dbf_handle_ = DBFOpen(filepath_.c_str(), "rb");
            if (dbf_handle_) dbf_read_only_ = true;
        } else {
            dbf_read_only_ = false;
        }

        std::string prj_file = GetPrjFilePath();
        if (file_util::IsFileExist(prj_file)) {
            file_util::GetContent(prj_file, &projection_info_);
        }

        return true;
    }

    /**
     * Create a new shapefile (.shp) and index file (.shx) for writing, with an
     * optional coordinate system definition (.prj).
     *
     * Parameters:
     *  path            - The path of the shapefile. This can be the full path
     *                    to either the .shp or .shx file, or just the base
     *                    filename without extension.
     *  shape_type      - The type of shapes to be stored in the shapefile.
     *                    It must be one of: SHPT_POINT, SHPT_ARC, SHPT_POLYGON
     *                    or SHPT_MULTIPOINT.
     *  projection_info - (Optional) The spatial reference system description
     *                    in WKT format.
     *                    If provided, it will be written to a corresponding
     *                    .prj file.
     */
    bool Create(const std::string& path, int shape_type,
                const std::string& projection_info = "") {
        filepath_ = path;
        handle_ = SHPCreate(filepath_.c_str(), shape_type);

        if (!handle_) {
            LOG(ERROR) << "Cannot create file: " << path << " for writing.";
            return false;
        }

        if (!projection_info.empty()) {
            std::string prj_path = GetPrjFilePath();
            file_util::SaveContent(prj_path, projection_info);
        }

        return true;
    }

    /**
     * Create a new .shp, .shx and .dbf file of the desired type for writting,
     * with an optional coordinate system definition (.prj).
     *
     * Parameters:
     *  path            - The path of the shapefile. This can be the full path
     *                    to either the .shp or .shx file, or just the base
     *                    filename without extension.
     *  shape_type      - The type of shapes to be stored in the shapefile.
     *                    It must be one of: SHPT_POINT, SHPT_ARC, SHPT_POLYGON
     *                    or SHPT_MULTIPOINT.
     *  projection_info - (Optional) The spatial reference system description
     *                    in WKT format.
     *                    If provided, it will be written to a corresponding
     *                    .prj file.
     */
    bool CreateWithDBF(const std::string& path, int shape_type,
                       const std::string& projection_info = "") {
        filepath_ = path;
        handle_ = SHPCreate(filepath_.c_str(), shape_type);
        dbf_handle_ = DBFCreate(filepath_.c_str());
        if (handle_ == nullptr || dbf_handle_ == nullptr) {
            LOG(ERROR) << "Cannot create file: " << path << " for writing.";
            return false;
        }

        if (!projection_info.empty()) {
            std::string prj_path = GetPrjFilePath();
            file_util::SaveContent(prj_path, projection_info);
        }

        return true;
    }

    /**
     * Close shape file.
     */
    void Close() {
        if (handle_) {
            SHPClose(handle_);
            handle_ = nullptr;
        }
        if (dbf_handle_) {
            DBFClose(dbf_handle_);
            dbf_handle_ = nullptr;
        }

        projection_info_.clear();
    }

    /**
     * Read multi-polygons from the shapefile.
     */
    void Read(Array<RMultiPolygon2D>* polygons) {
        CHECK(polygons);

        polygons->clear();
        if (!handle_) return;

        int type = this->type();
        CHECK(type == SHPT_POLYGON  || type == SHPT_POLYGONM ||
              type == SHPT_POLYGONZ || type == SHPT_MULTIPATCH);

        const int n = this->n_entities();

        for (int i = 0; i < n; ++i) {
            SHPObject* shape = SHPReadObject(handle_, i);
            CHECK(shape) << "Unable to read shape " << i;

            int n_parts = shape->nParts;
            int n_vertices = shape->nVertices;

            if (n_parts == 0) {
                Array<RPoint2D> points;
                for (int j = 0; j < n_vertices; ++j) {
                    points.emplace_back(shape->padfX[j], shape->padfY[j]);
                }

                RPolygon2D poly(points);
                polygons->emplace_back(poly);
            } else {
                RMultiPolygon2D polygon;
                for (int i = 0; i < n_parts; ++i) {
                    int part_type = shape->panPartType[i];
                    CHECK(part_type == SHPP_OUTERRING ||
                          part_type == SHPP_INNERRING ||
                          part_type == SHPP_FIRSTRING ||
                          part_type == SHPP_RING);

                    int start = shape->panPartStart[i];
                    int end = i + 1 == n_parts ? n_vertices
                                               : shape->panPartStart[i + 1];
                    Array<RPoint2D> points;
                    for (int j = start; j < end; ++j) {
                        points.emplace_back(shape->padfX[j], shape->padfY[j]);
                    }

                    RPolygon2D poly(points);
                    if (poly.empty()) continue;

                    polygon.Insert(poly, poly.IsClockwise());
                }

                polygons->push_back(polygon);
            }

            SHPDestroyObject(shape);
        }
    }

    /**
     * Read polylines from the shapefile.
     */
    void Read(Array<RPolyline2D>* polylines) {
        CHECK(polylines);

        polylines->clear();
        if (!handle_) return;

        int type = this->type();
        CHECK(type == SHPT_ARC || type == SHPT_ARCM || type == SHPT_ARCZ);

        const int n = this->n_entities();

        for (int i = 0; i < n; ++i) {
            SHPObject* shape = SHPReadObject(handle_, i);
            CHECK(shape) << "Unable to read shape " << i;

            int n_parts = shape->nParts;
            int n_vertices = shape->nVertices;

            if (n_parts == 0) {
                Array<RPoint2D> points;
                for (int j = 0; j < n_vertices; ++j) {
                    points.emplace_back(shape->padfX[j], shape->padfY[j]);
                }

                RPolyline2D poly(points);
                polylines->emplace_back(poly);
            } else {
                CHECK(n_parts == 1);
                for (int i = 0; i < n_parts; ++i) {
                    int part_type = shape->panPartType[i];
                    CHECK(part_type == SHPP_OUTERRING ||
                          part_type == SHPP_INNERRING ||
                          part_type == SHPP_FIRSTRING ||
                          part_type == SHPP_RING);

                    int start = shape->panPartStart[i];
                    int end = i + 1 == n_parts ? n_vertices
                                               : shape->panPartStart[i + 1];
                    Array<RPoint2D> points;
                    for (int j = start; j < end; ++j) {
                        points.emplace_back(shape->padfX[j], shape->padfY[j]);
                    }

                    RPolyline2D polyline(points);
                    if (polyline.empty()) continue;

                    polylines->push_back(polyline);
                }
            }

            SHPDestroyObject(shape);
        }
    }

    /**
     * Read multi-polylines from the shapefile.
     */
    void Read(Array<Array<RPolyline2D>>* polylines) {
        CHECK(polylines);

        polylines->clear();
        if (!handle_) return;

        int type = this->type();
        CHECK(type == SHPT_ARC || type == SHPT_ARCM || type == SHPT_ARCZ);

        const int n = this->n_entities();
        for (int i = 0; i < n; ++i) {
            SHPObject* shape = SHPReadObject(handle_, i);
            CHECK(shape) << "Unable to read shape " << i;

            int n_parts = shape->nParts;
            int n_vertices = shape->nVertices;

            if (n_parts == 0) {
                Array<RPoint2D> points;
                for (int j = 0; j < n_vertices; ++j) {
                    points.emplace_back(shape->padfX[j], shape->padfY[j]);
                }

                RPolyline2D poly(points);
                Array<RPolyline2D> polys = {poly};
                polylines->emplace_back(polys);
            } else {
                Array<RPolyline2D> polys;
                for (int i = 0; i < n_parts; ++i) {
                    int part_type = shape->panPartType[i];
                    CHECK(part_type == SHPP_OUTERRING ||
                          part_type == SHPP_INNERRING ||
                          part_type == SHPP_FIRSTRING ||
                          part_type == SHPP_RING);

                    int start = shape->panPartStart[i];
                    int end = i + 1 == n_parts ? n_vertices
                                               : shape->panPartStart[i + 1];
                    Array<RPoint2D> points;
                    for (int j = start; j < end; ++j) {
                        points.emplace_back(shape->padfX[j], shape->padfY[j]);
                    }

                    RPolyline2D polyline(points);
                    if (polyline.empty()) continue;

                    polys.push_back(polyline);
                }
                polylines->push_back(polys);
            }

            SHPDestroyObject(shape);
        }
    }

    /**
     * Read 2D points from the shapefile.
     */
    void Read(Array<RPoint2D>* points) {
        CHECK(points);

        points->clear();
        if (!handle_) return;

        int type = this->type();
        CHECK(type == SHPT_POINT || type == SHPT_POINTM || type == SHPT_POINTZ);

        const int n = this->n_entities();
        for (int i = 0; i < n; ++i) {
            SHPObject* shape = SHPReadObject(handle_, i);
            CHECK(shape) << "Unable to read shape " << i;

            points->emplace_back(shape->padfX[0], shape->padfY[0]);

            SHPDestroyObject(shape);
        }
    }

    /**
     * Write 2D points to the shapefile.
     */
    void Write(const Array<RPoint2D>& points) {
        int n = points.size();

        Array<double> x(n), y(n);
        for (int i = 0; i < n; ++i) {
            x[i] = points[i].x;
            y[i] = points[i].y;
        }

        SHPObject* shape = SHPCreateObject(SHPT_POINT, -1, 0, nullptr, nullptr,
                                           n, x.data(), y.data(),
                                           nullptr, nullptr);
        SHPWriteObject(handle_, -1, shape);
        SHPDestroyObject(shape);
    }

    /**
     * Write 2D line to the shapefile.
     */
    void Write(const RSegment2D& line) {
        Array<double> x(2), y(2);
        x[0] = line.lower_point().x;
        x[1] = line.upper_point().x;
        y[0] = line.lower_point().y;
        y[1] = line.upper_point().y;

        SHPObject* shape = SHPCreateObject(SHPT_ARC, -1, 0, nullptr,
                                           nullptr, 2, x.data(), y.data(),
                                           nullptr, nullptr);

        SHPWriteObject(handle_, -1, shape);
        SHPDestroyObject(shape);
    }

    /**
     * Write 2D polylines to the shapefile.
     */
    void Write(const Array<RSegment2D>& lines) {
        for (const auto& line : lines) {
            Write(line);
        }
    }

    /**
     * Write 2D polyline to the shapefile.
     */
    void Write(const RPolyline2D& polyline) {
        int n = polyline.size();
        Array<double> x(n), y(n);
        for (int i = 0; i < n; ++i) {
            x[i] = polyline[i].x;
            y[i] = polyline[i].y;
        }

        SHPObject* shape = SHPCreateObject(SHPT_ARC, -1, 0, nullptr,
                                           nullptr, n, x.data(), y.data(),
                                           nullptr, nullptr);

        SHPWriteObject(handle_, -1, shape);
        SHPDestroyObject(shape);
    }

    /**
     * Write 2D polylines to the shapefile.
     */
    void Write(const Array<RPolyline2D>& polylines) {
        for (const auto& polyline : polylines) {
            Write(polyline);
        }
    }

    /**
     * Write 2D multi-polylines to the shapefile.
     */
    void Write(const Array<Array<RPolyline2D>>& polylines) {
        for (const auto& polyline : polylines) {
            Array<int> part_starts;
            Array<double> x, y;
            for (const auto& line : polyline) {
                part_starts.push_back(x.size());

                Array<double> tmp_x, tmp_y;
                for (auto p : line) {
                    tmp_x.push_back(p.x);
                    tmp_y.push_back(p.y);
                }
                x.insert(tmp_x);
                y.insert(tmp_y);
            }

            SHPObject* shape = nullptr;
            if (part_starts.size() == 1) {
                shape = SHPCreateObject(SHPT_ARC, -1, 0, nullptr, nullptr,
                                        x.size(), x.data(), y.data(), nullptr,
                                        nullptr);
            } else {
                shape = SHPCreateObject(SHPT_ARC, -1, part_starts.size(),
                                        part_starts.data(), nullptr, x.size(),
                                        x.data(), y.data(), nullptr, nullptr);
            }
            SHPWriteObject(handle_, -1, shape);
            SHPDestroyObject(shape);
        }
    }

    /**
     * Write 2D polygon to the shapefile.
     */
    void Write(const RPolygon2D& polygon) {
        int n = polygon.size();
        Array<double> x(n), y(n);
        if (!polygon.empty()) {
            for (int i = 0; i < n; ++i) {
                x[i] = polygon[i].x;
                y[i] = polygon[i].y;
            }
            if (polygon.vertices().front() != polygon.vertices().back()) {
                x.push_back(x[0]);
                y.push_back(y[0]);
                ++n;
            }
        }

        SHPObject* shape = SHPCreateObject(SHPT_POLYGON, -1, 0, nullptr,
                                           nullptr, n, x.data(), y.data(),
                                           nullptr, nullptr);
        SHPWriteObject(handle_, -1, shape);
        SHPDestroyObject(shape);
    }

    /**
     * Write 2D polygons to the shapefile.
     */
    void Write(const Array<RPolygon2D>& polygons) {
        for (const auto& polygon : polygons) {
            Write(polygon);
        }
    }

    /**
     * Write 2D polygon to the shapefile.
     */
    void Write(const RMultiPolygon2D& polygon) {
        Array<int> part_starts;
        Array<double> x, y;
        for (const auto& b : polygon.boundaries()) {
            part_starts.push_back(x.size());

            Array<double> tmp_x, tmp_y;
            for (auto p : b.polygon.vertices()) {
                tmp_x.push_back(p.x);
                tmp_y.push_back(p.y);
            }
            if (!b.polygon.empty()) {
                tmp_x.push_back(tmp_x.front());
                tmp_y.push_back(tmp_y.front());
            }

            if (b.polygon.IsCounterclockwise() == b.is_outer) {
                std::reverse(tmp_x.begin(), tmp_x.end());
                std::reverse(tmp_y.begin(), tmp_y.end());
            }

            x.insert(tmp_x);
            y.insert(tmp_y);
        }

        SHPObject* shape = SHPCreateObject(SHPT_POLYGON, -1, part_starts.size(),
                                           part_starts.data(),
                                           nullptr,
                                           x.size(),
                                           x.data(),
                                           y.data(),
                                           nullptr,
                                           nullptr);
        SHPWriteObject(handle_, -1, shape);
        SHPDestroyObject(shape);
    }

    /**
     * Write 2D polygons to the shapefile.
     */
    void Write(const Array<RMultiPolygon2D>& polygons) {
        for (const auto& polygon : polygons) {
            Write(polygon);
        }
    }

    /**
     * Write 2D point to the shapefile.
     */
    void Write(const RPoint2D& p) {
        SHPObject* shape = SHPCreateObject(SHPT_POINT, -1, 0, nullptr,
                                           nullptr, 1, &p.x, &p.y,
                                           nullptr, nullptr);
        SHPWriteObject(handle_, -1, shape);
        SHPDestroyObject(shape);
    }

    /**
     * Read all field names.
     */
    void GetFieldNames(Array<std::string>* field_names) const {
        CHECK(field_names);

        field_names->clear();
        if (!dbf_handle_) return;

        char title[XBASE_FLDNAME_LEN_READ + 1];
        for (int i = 0; i < dbf_handle_->nFields; ++i) {
            DBFGetFieldInfo(dbf_handle_, i, title, nullptr, nullptr);
            field_names->push_back(title);
        }
    }

    /**
     * Return the index of the field matching this name, or -1 on failure.
     * The comparison is case insensitive. However, lengths must match exactly.
     */
    int GetFieldID(const std::string& field) const {
        if (!dbf_handle_) return -1;

        return DBFGetFieldIndex(dbf_handle_, field.c_str());
    }

    /**
     * Read the value of one field and return it as a string. This function may
     * be used on any field type (including FTInteger and FTDouble) and will
     * return the string representation stored in the .dbf file.
     *
     * If the TRIM_DBF_WHITESPACE macro is defined in shapefil.h (it is by
     * default) then all leading and trailing space (ASCII 32) characters will
     * be stripped before the string is returned.
     */
    bool ReadField(const std::string& field_name, Array<std::string>* records) {
        CHECK(records);

        records->clear();

        int field_id = GetFieldID(field_name);
        if (field_id == -1) return false;

        int n_records = dbf_handle_->nRecords;
        for (int i = 0; i < n_records; ++i) {
            const char* str = DBFReadStringAttribute(dbf_handle_, i, field_id);
            if (str) {
                records->emplace_back(str);
            } else {
                records->emplace_back("");
            }
        }

        return true;
    }

    /**
     * Read the value of one field and return it as an integer. This can be used
     * even with FTString fields, though the returned value will be zero if not
     * interpretable as a number.
     */
    bool ReadField(const std::string& field_name, Array<int>* records) {
        CHECK(records);

        records->clear();

        int field_id = GetFieldID(field_name);
        if (field_id == -1) return false;

        int n_records = dbf_handle_->nRecords;
        for (int i = 0; i < n_records; ++i) {
            records->push_back(DBFReadIntegerAttribute(dbf_handle_, i,
                                                       field_id));
        }

        return true;
    }

    /**
     * Read the value of one field and return it as a double. This can be used
     * even with FTString fields, though the returned value will be zero if not
     * interpretable as a number.
     */
    bool ReadField(const std::string& field_name, Array<double>* records) {
        CHECK(records);

        records->clear();

        int field_id = GetFieldID(field_name);
        if (field_id == -1) return false;

        int n_records = dbf_handle_->nRecords;
        for (int i = 0; i < n_records; ++i) {
            records->push_back(DBFReadDoubleAttribute(dbf_handle_, i,
                                                      field_id));
        }

        return true;
    }

    /**
     * Add a string field with the max width of the string. It will create the
     * DBF file if it does not exist.
     *
     * Return value is the field number of the new field, or -1 if the addition
     * of the field failed.
     */
    int AddStringField(const std::string& field, int max_width) {
        CHECK(max_width >= 1);
        CHECK(field.size() <= XBASE_FLDNAME_LEN_READ)
                << "The field name is too long.";

        if (!OpenOrCreateDBFFile()) return false;
        return DBFAddField(dbf_handle_, field.c_str(), FTString, max_width, 0);
    }

    /**
     * Add a double field with the decimal numbers. It will create the DBF file
     * if it does not exist.
     *
     * Return value is the field number of the new field, or -1 if the addition
     * of the field failed.
     */
    int AddDoubleField(const std::string& field, int decimal = 6) {
        CHECK(decimal >= 0);
        CHECK(field.size() <= XBASE_FLDNAME_LEN_READ)
                << "The field name is too long.";

        if (!OpenOrCreateDBFFile()) return false;
        return DBFAddField(dbf_handle_, field.c_str(), FTDouble, 18, decimal);
    }

    /**
     * Add a int field. It will create the DBF file if it does not exist.
     *
     * Return value is the field number of the new field, or -1 if the addition
     * of the field failed.
     */
    int AddIntegerField(const std::string& field) {
        CHECK(field.size() <= XBASE_FLDNAME_LEN_READ)
                << "The field name is too long.";

        if (!OpenOrCreateDBFFile()) return false;
        return DBFAddField(dbf_handle_, field.c_str(), FTInteger, 9, 0);
    }

    /**
     * Delete a field from DBF file.
     */
    void DeleteField(const std::string& field) {
        int field_id = GetFieldID(field);
        if (field_id == -1) return;

        DBFDeleteField(dbf_handle_, field_id);
    }

    /**
     * Try to write field values into DBF file.
     */
    bool WriteField(const std::string& field_name,
                    const Array<std::string>& records) {
        if (!dbf_handle_) return false;

        int field_id = GetFieldID(field_name);
        if (field_id == -1) {
            int max_width = 1;
            for (const auto& record : records) {
                max_width = std::max(static_cast<int>(record.size()) + 1,
                                     max_width);
            }
            field_id = AddStringField(field_name, max_width);
            if (field_id == -1) return false;
        }

        DBFFieldType field_type = DBFGetFieldInfo(dbf_handle_, field_id,
                                                  nullptr,
                                                  nullptr,
                                                  nullptr);
        CHECK(field_type == FTString) << "The type of field \'"
                                      << field_name
                                      << "\' is not string.";

        for (int i = 0; i < records.size(); ++i) {
            DBFWriteStringAttribute(dbf_handle_, i, field_id,
                                    records[i].c_str());
        }

        return true;
    }

    /**
     * Try to write field values into DBF file.
     */
    bool WriteField(const std::string& field_name,
                    const Array<double>& records) {
        if (!dbf_handle_) return false;

        int field_id = GetFieldID(field_name);
        if (field_id == -1) {
            field_id = AddDoubleField(field_name);
            if (field_id == -1) return false;
        }

        DBFFieldType field_type = DBFGetFieldInfo(dbf_handle_, field_id,
                                                  nullptr,
                                                  nullptr,
                                                  nullptr);
        CHECK(field_type == FTDouble) << "The type of field \'"
                                      << field_name
                                      << "\' is not double.";

        for (int i = 0; i < records.size(); ++i) {
            DBFWriteDoubleAttribute(dbf_handle_, i, field_id, records[i]);
        }

        return true;
    }

    /**
     * Try to write field values into DBF file.
     */
    bool WriteField(const std::string& field_name, const Array<int>& records) {
        if (!dbf_handle_) return false;

        int field_id = GetFieldID(field_name);
        if (field_id == -1) {
            field_id = AddIntegerField(field_name);
            if (field_id == -1) return false;
        }

        DBFFieldType field_type = DBFGetFieldInfo(dbf_handle_, field_id,
                                                  nullptr,
                                                  nullptr,
                                                  nullptr);
        CHECK(field_type == FTInteger) << "The type of field \'"
                                       << field_name
                                       << "\' is not integer.";

        for (int i = 0; i < records.size(); ++i) {
            DBFWriteIntegerAttribute(dbf_handle_, i, field_id, records[i]);
        }

        return true;
    }

    /**
     * Get field shape field.
     */
    ShapeField GetShapeField(const std::string& name) const {
        CHECK(dbf_handle_);

        int field_id = this->GetFieldID(name);
        CHECK(field_id != -1);

        ShapeField field;
        field.name = name;

        field.type = DBFGetFieldInfo(dbf_handle_, field_id, nullptr,
                                     &field.width, &field.precision);
        return field;
    }

    /**
     * Retrun .prj file path.
     */
    std::string GetPrjFilePath() const {
        return file_util::GetFolderName(filepath_) +
               file_util::GetStemName(filepath_) + ".prj";
    }

    bool empty() const {
        return handle_ == nullptr;
    }

    /**
     * Return the number of entities in the shape file.
     */
    int n_entities() const {
        return handle_ ? handle_->nRecords : 0;
    }

    /**
     * Return the number of fields currently defined for the indicated
     * xBase(DBF) file.
     */
    int n_fields() const {
        return dbf_handle_ ? dbf_handle_->nFields : 0;
    }

    /**
     * Return the shape type.
     */
    int type() const {
        return handle_ ? handle_->nShapeType : 0;
    }

    /**
     * Return the file path of shape file.
     */
    const std::string& filepath() const {
        return filepath_;
    }

    /**
     * Stored projection file information.
     */
    const std::string& projection_info() const {
        return projection_info_;
    }

private:
    /**
     * Open or create a dbf file.
     */
    bool OpenOrCreateDBFFile() {
        CHECK(handle_) << "The shape file should be opened first.";

        if (dbf_handle_) {
            if (dbf_read_only_) {
                LOG(ERROR) << "The DBF file is read only. "
                           << "Please check the file status.";
                return false;
            }
            return true;
        }

        dbf_handle_ = DBFCreate(filepath_.c_str());
        dbf_read_only_ = false;
        if (!dbf_handle_) {
            LOG(ERROR) << "Create DBF file failed.";
            return false;
        }

        return true;
    }

    // Path of the shape file.
    std::string filepath_;

    SHPHandle handle_ = nullptr;

    // Current DBF file is read only or not.
    bool dbf_read_only_ = false;
    DBFHandle dbf_handle_ = nullptr;

    // Optional projection file information.
    std::string projection_info_;
};

} // namespace gis
} // namespace cl

#endif // CODELIBRARY_GIS_SHAPE_FILE_H_
