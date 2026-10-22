//
// Copyright 2024 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_UTIL_IO_TEXT_STREAM_H_
#define CODELIBRARY_UTIL_IO_TEXT_STREAM_H_

#include <string>

#include "codelibrary/base/log.h"

namespace cl {

/**
 * Fast stream for format inputing.
 */
class TextStream {
    static constexpr int EOF_FLAG  = 1;
    static constexpr int FAIL_FLAG = 2;

public:
    TextStream() = default;

    explicit TextStream(const std::string& str)
        : pos_(str.c_str()) {}

    explicit TextStream(const char* str)
        : pos_(str) {}

    /**
     * Reset the stream.
     */
    void Reset(const std::string& str) {
        pos_ = str.c_str();
    }

    /**
     * Reset the stream.
     */
    void Reset(const char* str) {
        pos_ = str;
    }

    /**
     * Read an integer from stream.
     *
     * For efficiency, we do not check for overflow.
     */
    template <typename Int>
    bool ReadInteger(Int* v) {
        static_assert(std::is_integral<Int>::value);

        CHECK(v);

        if (!Advance()) return false;

        bool negative = false;
        if (*pos_ == '-') {
            negative = true;
            ++pos_;
        } else if (*pos_ == '+') {
            ++pos_;
        }
        if (negative && !std::is_signed<Int>::value) {
            LOG(ERROR) << "The type of " << typeid(*v).name()
                       << " is unsigned.";
            return false;
        }

        int n_digits = 0;
        Int local_v = 0;
        while (std::isdigit(*pos_)) {
            local_v = local_v * 10 + static_cast<Int>(*pos_ - '0');
            ++n_digits;
            ++pos_;
        }
        *v = negative ? -local_v : local_v;

        if (n_digits == 0) {
            state_ |= FAIL_FLAG;
            LOG(ERROR) << "Not an integer number.";
            return false;
        }

        if (!*pos_ || *pos_ == delimiter_) {
            if (!*pos_) state_ |= EOF_FLAG;
            return true;
        } else {
            state_ |= FAIL_FLAG;
            return false;
        }
    }

    /**
     * Read a float from stream.
     */
    template <typename Float>
    bool ReadFloat(Float* v) {
        static_assert(std::is_floating_point<Float>::value);

        CHECK(v);

        if (!Advance()) return false;

        bool negative = false;
        if (*pos_ == '-') {
            negative = true;
            ++pos_;
        } else if (*pos_ == '+') {
            ++pos_;
        }

        static constexpr double double_digits[10] = {
            0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0
        };
        double local_v = 0.0;

        bool has_int_digits = std::isdigit(*pos_);
        if (has_int_digits) {
            do {
                local_v = local_v * 10.0 + double_digits[*pos_ - '0'];
                ++pos_;
            } while (std::isdigit(*pos_));
        } else if (*pos_ != '.') {
            state_ |= FAIL_FLAG;
            LOG(ERROR) << "Not a floating point number.";
            return false;
        }

        bool has_frac_digits = false;
        if (*pos_ == '.') {
            ++pos_;
            has_frac_digits = std::isdigit(*pos_);
            if (has_frac_digits) {
                double scale = 0.1;
                do {
                    local_v += scale * double_digits[*pos_ - '0'];
                    scale *= 0.1;
                    ++pos_;
                } while (std::isdigit(*pos_));
            } else if (!has_int_digits) {
                state_ |= FAIL_FLAG;
                LOG(ERROR) << "Floating point number has no digits before or "
                              "after the decimal point.";

                return false;
            }
        }

        bool has_exponent = *pos_ == 'e' || *pos_ == 'E';
        if (has_exponent) {
            ++pos_;
            bool negative_exponent = false;
            if (*pos_ == '-') {
                negative_exponent = true;
                ++pos_;
            } else if (*pos_ == '+') {
                ++pos_;
            }

            if (!std::isdigit(*pos_)) {
                state_ |= FAIL_FLAG;
                LOG(ERROR) << "Floating point exponent has no digits";
                return false;
            }

            double exponent = 0.0;
            do {
                exponent = exponent * 10.0 + double_digits[*pos_ - '0'];
                ++pos_;
            } while (std::isdigit(*pos_));

            if (negative_exponent) {
                exponent = -exponent;
            }
            local_v *= std::pow(10.0, exponent);
        }
        *v = negative ? -static_cast<Float>(local_v)
                      : static_cast<Float>(local_v);

        if (!*pos_ || *pos_ == delimiter_) {
          if (!*pos_) state_ |= EOF_FLAG;
            return true;
        } else {
            state_ |= FAIL_FLAG;
            return false;
        }
    }

    /**
     * Read a line.
     */
    bool ReadLine(std::string* line) {
        CHECK(line);

        line->clear();

        if (!Advance()) return false;

        while (*pos_ && *pos_ != '\n') {
            line->push_back(*pos_++);
        }

        if (!*pos_) state_ |= EOF_FLAG;
        return true;
    }

    void set_delimiter(char delimiter) {
        delimiter_ = delimiter;
    }

    /**
     * Check whether the stream has no errors.
     */
    explicit operator bool() const {
        return state_ == 0 || state_ == EOF_FLAG;
    }

    /**
     * Return true if the stream has reached end.
     */
    bool eof() const {
        return (state_ & EOF_FLAG) != 0;
    }

    /**
     * Return true if the most recent I/O operation on the stream completed
     * successfully.
     */
    bool good() const {
        return state_ == 0;
    }

    /**
     * Return true if an error has occurred on the stream.
     */
    bool fail() const {
        return state_ != 0 && state_ != EOF_FLAG;
    }

    friend TextStream& operator >>(TextStream& stream, std::string& str) {
        if (!stream.Advance()) return stream;

        str.clear();
        while (*stream.pos_ > ' ' && *stream.pos_ != stream.delimiter_) {
            str += *stream.pos_;
            ++stream.pos_;
        }
        if (!*stream.pos_) {
            stream.state_ |= EOF_FLAG;
        }

        return stream;
    }

    friend TextStream& operator >>(TextStream& stream, int8_t& v) {
        stream.ReadInteger(&v);
        return stream;
    }

    friend TextStream& operator >>(TextStream& stream, uint8_t& v) {
        stream.ReadInteger(&v);
        return stream;
    }

    friend TextStream& operator >>(TextStream& stream, int16_t& v) {
        stream.ReadInteger(&v);
        return stream;
    }

    friend TextStream& operator >>(TextStream& stream, uint16_t& v) {
        stream.ReadInteger(&v);
        return stream;
    }

    friend TextStream& operator >>(TextStream& stream, int32_t& v) {
        stream.ReadInteger(&v);
        return stream;
    }

    friend TextStream& operator >>(TextStream& stream, uint32_t& v) {
        stream.ReadInteger(&v);
        return stream;
    }

    friend TextStream& operator >>(TextStream& stream, int64_t& v) {
        stream.ReadInteger(&v);
        return stream;
    }

    friend TextStream& operator >>(TextStream& stream, uint64_t& v) {
        stream.ReadInteger(&v);
        return stream;
    }

    friend TextStream& operator >>(TextStream& stream, float& v) {
        stream.ReadFloat(&v);
        return stream;
    }

    friend TextStream& operator >>(TextStream& stream, double& v) {
        stream.ReadFloat(&v);
        return stream;
    }

private:
    /**
     * Advance the stream by skipping over whitespace or a specified delimiter.
     */
    bool Advance(bool with_trim = true) {
        if (!pos_) {
            state_ |= FAIL_FLAG;
            return false;
        }

        state_ &= ~FAIL_FLAG;

        if (!*pos_) {
            state_ |= EOF_FLAG;
            state_ |= FAIL_FLAG;
            return false;
        }

        if (!with_trim) return true;

        while (*pos_ && (*pos_ <= ' ' || *pos_ == delimiter_)) {
            ++pos_;
        }

        if (!*pos_) {
            state_ |= EOF_FLAG;
            state_ |= FAIL_FLAG;
        }

        return *pos_ != 0;
    }

    // The string should end with '\0'.
    char const* pos_ = nullptr;

    // Delimiter to split the elements.
    char delimiter_ = ' ';

    // State of stream.
    int state_ = 0;
};

} // namespace cl

#endif // CODELIBRARY_UTIL_IO_TEXT_STREAM_H_
