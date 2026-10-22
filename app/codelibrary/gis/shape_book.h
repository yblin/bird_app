//
// Copyright 2025-2026 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GIS_SHAPE_BOOK_H_
#define CODELIBRARY_GIS_SHAPE_BOOK_H_

#include <map>

#include "codelibrary/base/enumerate.h"
#include "codelibrary/base/pool.h"
#include "codelibrary/gis/shape_file.h"
#include "codelibrary/util/io/text_stream.h"

namespace cl {
namespace gis {

/**
 * Attribute for shape feature.
 */
struct ShapeAttribute {
    enum class Type {
        INT, DOUBLE, STRING
    };

    Type type;
    int int_data = 0;
    std::string string_data = "";
    double double_data = 0.0;

    ShapeAttribute() = default;

    ShapeAttribute(int v) {
        type = Type::INT;
        int_data = v;
    }

    ShapeAttribute(double v) {
        type = Type::DOUBLE;
        double_data = v;
    }

    ShapeAttribute(const std::string& v) {
        type = Type::STRING;
        string_data = v;
    }

    ShapeAttribute& operator=(int v) {
        type = Type::INT;
        int_data = v;
        return *this;
    }

    ShapeAttribute& operator=(double v) {
        type = Type::DOUBLE;
        double_data = v;
        return *this;
    }

    ShapeAttribute& operator=(const std::string v) {
        type = Type::STRING;
        string_data = v;
        return *this;
    }

    int get_int() const {
        switch(type) {
        case Type::INT:
            return int_data;
        case Type::DOUBLE:
            return static_cast<int>(double_data);
        case Type::STRING:
        {
            int t;
            TextStream stream(string_data);
            if (!stream.ReadInteger(&t))
                return 0;
            return t;
        }
        }
        return 0;
    }

    double get_double() const {
        switch(type) {
        case Type::INT:
            return int_data;
        case Type::DOUBLE:
            return double_data;
        case Type::STRING:
        {
            double t;
            TextStream stream(string_data);
            if (!stream.ReadFloat(&t))
                return 0.0;
            return t;
        }
        }
        return 0.0;
    }

    std::string get_string() const {
        switch(type) {
        case Type::INT:
            return ToString(int_data);
        case Type::DOUBLE:
            return ToString(double_data);
        case Type::STRING:
            return string_data;
        }
        return "";
    }
};

/**
 * Geometry and attribute entity in the shape file.
 */
struct BaseShapeFeature {
    std::unordered_map<std::string, ShapeAttribute> attributes;

    /**
     * Set value for the specific field.
     */
    template <typename T>
    void SetField(const std::string& field_name, const T& data) {
        attributes[field_name] = data;
    }
    void SetField(const std::string& field_name, const ShapeAttribute& data) {
        attributes[field_name] = data;
    }

    /**
     * Get field value.
     */
    int GetFieldAsInteger(const std::string& field_name) const {
        auto it = attributes.find(field_name);
        if (it == attributes.end()) {
            return 0;
        }

        return it->second.get_int();
    }

    /**
     * Get field value.
     */
    double GetFieldAsDouble(const std::string& field_name) const {
        auto it = attributes.find(field_name);
        if (it == attributes.end()) {
            return 0.0;
        }

        return it->second.get_double();
    }

    /**
     * Get field value.
     */
    std::string GetFieldAsString(const std::string& field_name) const {
        auto it = attributes.find(field_name);
        if (it == attributes.end()) {
            return "";
        }

        return it->second.get_string();
    }
};

/**
 * Shape feature for 2D geometry.
 */
struct ShapeFeature : public BaseShapeFeature {
    Array<Array<RPoint2D>> geometry;

    /**
     * Set geometry for feature.
     */
    void SetGeometry(const RPolygon2D& polygon) {
        geometry.clear();
        Array<RPoint2D> points = polygon.vertices();
        if (!polygon.empty()) {
            points.push_back(points.front());
        }
        if (!polygon.IsCounterclockwise()) {
            std::reverse(points.begin(), points.end());
        }
        geometry.push_back(points);
    }
    void SetGeometry(const RMultiPolygon2D& polygon) {
        geometry.clear();
        for (const auto& b : polygon.boundaries()) {
            Array<RPoint2D> points = b.polygon.vertices();
            if (!b.polygon.empty()) {
                points.push_back(points.front());
            }
            if (b.polygon.IsCounterclockwise() == b.is_outer) {
                std::reverse(points.begin(), points.end());
            }
            geometry.push_back(points);
        }
    }
    void SetGeometry(const Array<RPolyline2D>& multi_polyline) {
        geometry.clear();
        for (const auto& poly : multi_polyline) {
            geometry.push_back(poly);
        }
    }
    void SetGeometry(const RPolyline2D& polyline) {
        geometry.clear();
        geometry.push_back(polyline);
    }
    void SetGeometry(const RSegment2D& seg) {
        geometry.clear();
        Array<RPoint2D> ps(2);
        ps[0] = seg.lower_point();
        ps[1] = seg.upper_point();
        geometry.push_back(ps);
    }
    void SetGeometry(const RPoint2D& point) {
        geometry.clear();
        Array<RPoint2D> points = { point };
        geometry.push_back(points);
    }

    /**
     * Get polygon from the feature.
     */
    void GetPolygon(RMultiPolygon2D* polygon) const {
        CHECK(polygon);

        polygon->clear();
        bool first = true;
        for (const auto& points : geometry) {
            if (!points.empty()) {
                RPolygon2D poly(points);
                if (first) {
                    if (!poly.IsCounterclockwise()) {
                        poly.Reverse();
                    }
                    polygon->Insert(poly, true);
                    first = false;
                } else {
                    if (!poly.IsClockwise()) {
                        poly.Reverse();
                    }
                    polygon->Insert(poly, false);
                }
            }
        }
    }

    /**
     * Get multi-polyline from the feature.
     */
    void GetPolyline(Array<RPolyline2D>* polyline) const {
        CHECK(polyline);

        polyline->clear();
        for (const auto& points : geometry) {
            polyline->emplace_back(points);
        }
    }

    /**
     * Get polyline from the feature.
     */
    void GetPolyline(RPolyline2D* polyline) const {
        CHECK(polyline);
        CHECK(geometry.size() <= 1) << geometry.size();

        if (geometry.empty())
            polyline->clear();
        else
            *polyline = geometry.front();
    }

    /**
     * Return a single point.
     */
    RPoint2D GetPoint() const {
        CHECK(geometry.size() == 1);
        CHECK(geometry[0].size() == 1);

        return geometry[0][0];
    }
};

/**
 * Book shape file content into memeory.
 */
class ShapeBook {
public:
    enum GeometryType {
        POLYGON, POLYLINE, POINT
    };

    ShapeBook() = default;

    /**
     * Create a shape book from shape file.
     */
    ShapeBook(const std::string& path) {
        Open(path);
    }

    /**
     * Create an empty shape book.
     */
    ShapeBook(const std::string& projection_info, GeometryType geometry_type)
        : projection_info_(projection_info),
          geometry_type_(geometry_type) {}

    /**
     * Open a shape file.
     */
    bool Open(const std::string& path) {
        projection_info_.clear();
        features_.clear();
        feature_pool_.clear();

        if (!shape_file_.Open(path)) return false;

        projection_info_ = shape_file_.projection_info();

        int type = shape_file_.type();
        if (type == SHPT_POLYGON || type == SHPT_POLYGONM) {
            geometry_type_ = POLYGON;
        } else if (type == SHPT_ARC || type == SHPT_ARCM) {
            geometry_type_ = POLYLINE;
        } else if (type == SHPT_POINT || type == SHPT_POINTM) {
            geometry_type_ = POINT;
        } else {
            CHECK(false) << "Invalid geometry type: " << shape_file_.type()
                         << " for ShapeBook2D.";
        }
        return true;
    }

    /**
     * Close the shape file and clear the data.
     */
    void Close() {
        shape_file_.Close();
        projection_info_.clear();
        features_.clear();
        feature_pool_.clear();
        fields_.clear();
    }

    /**
     * Read all features with fields from the open shape file.
     */
    void ReadFeatures() {
        CHECK(!shape_file_.empty());

        features_.clear();
        feature_pool_.clear();

        Array<std::string> field_names;
        Array<Array<ShapeAttribute>> fields;
        this->ReadFields(&field_names, &fields);

        switch (geometry_type_) {
        case POLYGON:
        {
            Array<RMultiPolygon2D> polygons;
            shape_file_.Read(&polygons);
            for (auto&& [index, poly] : enumerate(polygons)) {
                auto f = this->AddFeature(poly);
                for (int i = 0; i < field_names.size(); ++i) {
                    f->SetField(field_names[i], fields[i][index]);
                }
            }
            break;
        }
        case POLYLINE:
        {
            Array<Array<RPolyline2D>> polylines;
            shape_file_.Read(&polylines);
            for (auto&& [index, poly] : enumerate(polylines)) {
                auto f = this->AddFeature(poly);
                for (int i = 0; i < field_names.size(); ++i) {
                    f->SetField(field_names[i], fields[i][index]);
                }
            }
            break;
        }
        case POINT:
        {
            Array<RPoint2D> points;
            shape_file_.Read(&points);
            for (auto&& [index, p] : enumerate(points)) {
                auto f = this->AddFeature(p);
                for (int i = 0; i < field_names.size(); ++i) {
                    f->SetField(field_names[i], fields[i][index]);
                }
            }
        }
        }
    }

    /**
     * Save geometries from memory into local file.
     */
    bool Save(const std::string& path) {
        ShapeFile shape_file;

        switch (geometry_type_) {
        case POLYGON:
        {
            if (!shape_file.CreateWithDBF(path, SHPT_POLYGON, projection_info_))
                return false;

            Array<RMultiPolygon2D> polygons;
            for (auto feature : features_) {
                RMultiPolygon2D polygon;
                feature->GetPolygon(&polygon);
                polygons.push_back(polygon);
            }

            shape_file.Write(polygons);
            break;
        }
        case POLYLINE:
            if (!shape_file.CreateWithDBF(path, SHPT_ARC, projection_info_))
                return false;
            for (auto feature : features_) {
                Array<RPolyline2D> multi_polyline;
                feature->GetPolyline(&multi_polyline);
                shape_file.Write(multi_polyline);
            }
            break;
        case POINT:
            if (!shape_file.CreateWithDBF(path, SHPT_POINT, projection_info_))
                return false;
            for (auto feature : features_) {
                RPoint2D p = feature->GetPoint();
                shape_file.Write(p);
            }
        }

        std::map<std::string, Array<ShapeAttribute>> fields;
        for (auto& field : fields_) {
            auto& field_data = fields[field.name];
            for (auto feature : features_) {
                auto it = feature->attributes.find(field.name);
                if (it == feature->attributes.end()) {
                    switch (field.type) {
                    case FTInteger:
                        field_data.emplace_back(0);
                        break;
                    case FTDouble:
                        field_data.emplace_back(0.0);
                        break;
                    case FTString:
                        field_data.emplace_back("");
                        break;
                    default:
                        CHECK(false) << "Unsupport type: " << field.type;
                    }
                } else {
                    field_data.push_back(it->second);
                }
            }

            int field_index = -1;

            switch (field.type) {
            case FTInteger:
            {
                field_index = shape_file.AddIntegerField(field.name);
                if (field_index == -1) continue;

                Array<int> data;
                for (auto& attribute : field_data) {
                    data.push_back(attribute.get_int());
                }
                shape_file.WriteField(field.name, data);
                break;
            }
            case FTDouble:
            {
                field_index = shape_file.AddDoubleField(field.name,
                                                        field.precision);
                if (field_index == -1) continue;

                Array<double> data;
                for (auto& attribute : field_data) {
                    data.push_back(attribute.get_double());
                }
                shape_file.WriteField(field.name, data);
                break;
            }
            case FTString:
            {
                field_index = shape_file.AddStringField(field.name,
                                                        field.width);
                if (field_index == -1) continue;
                Array<std::string> data;
                for (auto& attribute : field_data) {
                    data.push_back(attribute.get_string());
                }
                shape_file.WriteField(field.name, data);
                break;
            }
            default:
                CHECK(false) << "Unsupport type: " << field.type;
            }

            if (field_index == -1) {
                LOG(ERROR) << Format("Cannot add field ({}) into shape file "
                                     "\"{}\".", field.name, path);
            }
        }

        return true;
    }

    /**
     * Add a feature.
     */
    ShapeFeature* AddFeature(const RPolygon2D& polygon) {
        ShapeFeature* feature = feature_pool_.Allocate();
        feature->attributes.clear();
        feature->SetGeometry(polygon);
        features_.push_back(feature);
        return feature;
    }
    ShapeFeature* AddFeature(const RMultiPolygon2D& polygon) {
        ShapeFeature* feature = feature_pool_.Allocate();
        feature->attributes.clear();
        feature->SetGeometry(polygon);
        features_.push_back(feature);
        return feature;
    }
    ShapeFeature* AddFeature(const Array<RPolyline2D>& multi_polyline) {
        CHECK(geometry_type_ == POLYLINE);

        ShapeFeature* feature = feature_pool_.Allocate();
        feature->attributes.clear();
        feature->SetGeometry(multi_polyline);
        features_.push_back(feature);
        return feature;
    }
    ShapeFeature* AddFeature(const RPolyline2D& polyline) {
        CHECK(geometry_type_ == POLYLINE);

        ShapeFeature* feature = feature_pool_.Allocate();
        feature->attributes.clear();
        feature->SetGeometry(polyline);
        features_.push_back(feature);
        return feature;
    }
    ShapeFeature* AddFeature(const RSegment2D& segment) {
        CHECK(geometry_type_ == POLYLINE);

        ShapeFeature* feature = feature_pool_.Allocate();
        feature->attributes.clear();
        feature->SetGeometry(segment);
        features_.push_back(feature);
        return feature;
    }
    ShapeFeature* AddFeature(const RPoint2D& point) {
        CHECK(geometry_type_ == POINT);

        ShapeFeature* feature = feature_pool_.Allocate();
        feature->attributes.clear();
        feature->SetGeometry(point);
        features_.push_back(feature);
        return feature;
    }
    ShapeFeature* AddFeature(const ShapeFeature& feature) {
        ShapeFeature* f = feature_pool_.Allocate();
        *f = feature;
        features_.push_back(f);
        return f;
    }

    /**
     * Remove feature from the book.
     */
    void RemoveFeature(ShapeFeature* feature) {
        CHECK(feature);

        auto it = features_.find(feature);
        if (it != features_.end()) {
            features_.erase(it);
            feature_pool_.Deallocate(feature);
        }
    }

    /**
     * Add a string field for saving.
     */
    void AddField(const std::string& name, int max_width = 32) {
        AddStringField(name, max_width);
    }
    void AddStringField(const std::string &name, int max_width = 32) {
        ShapeField field;
        field.type = FTString;
        field.name = name;
        field.width = max_width;
        fields_.push_back(field);
    }

    /**
     * Add a integer field.
     */
    void AddIntegerField(const std::string &name) {
        ShapeField field;
        field.type = FTInteger;
        field.name = name;
        fields_.push_back(field);
    }

    /**
     * Add a double field.
     */
    void AddDoubleField(const std::string &name, int precision = 6) {
        ShapeField field;
        field.type = FTDouble;
        field.name = name;
        field.precision = precision;
        fields_.push_back(field);
    }

    const Array<ShapeFeature*>& features() const {
        return features_;
    }

    const std::string& projection_info() const {
        return projection_info_;
    }

    void set_projection_info(const std::string& projection) {
        projection_info_ = projection;
    }

    GeometryType geometry_type() const {
        return geometry_type_;
    }

private:
    /**
     * Read all fields from the open shape file.
     *
     * It resets the fields even if there is data in it.
     */
    void ReadFields(Array<std::string>* field_names,
                    Array<Array<ShapeAttribute>>* fields) {
        CHECK(field_names);
        CHECK(fields);
        CHECK(!shape_file_.empty());

        shape_file_.GetFieldNames(field_names);
        fields->clear();
        fields_.clear();
        for (auto field_name : *field_names) {
            ShapeField field = shape_file_.GetShapeField(field_name);
            fields_.push_back(field);

            Array<ShapeAttribute> attributes;
            Array<int> data1;
            Array<double> data2;
            Array<std::string> data3;
            switch (field.type) {
            case FTInteger:
                shape_file_.ReadField(field.name, &data1);
                for (auto d : data1) {
                    attributes.emplace_back(d);
                }
                break;
            case FTDouble:
                shape_file_.ReadField(field.name, &data2);
                for (auto d : data2) {
                    attributes.emplace_back(d);
                }
                break;
            case FTString:
                shape_file_.ReadField(field.name, &data3);
                for (auto d : data3) {
                    attributes.emplace_back(d);
                }

                break;
            default:
                CHECK(false) << "Type: " << field.type << " is not supported.";
            }
            CHECK(attributes.size() == shape_file_.n_entities());
            fields->push_back(attributes);
        }
    }

    ShapeFile shape_file_;

    // Projection information for .prj file.
    std::string projection_info_;

    // Geometry type of shape file.
    GeometryType geometry_type_;

    // Shapes and fields.
    Array<ShapeFeature*> features_;

    // Pool for features.
    Pool<ShapeFeature> feature_pool_;

    // Fields for saving.
    Array<ShapeField> fields_;
};

} // namespace gis
} // namespace cl

#endif // CODELIBRARY_GIS_SHAPE_BOOK_H_
