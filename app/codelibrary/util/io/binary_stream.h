//
// Copyright 2023-2024 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_UTIL_IO_BINARY_STREAM_H_
#define CODELIBRARY_UTIL_IO_BINARY_STREAM_H_

#include <cstring>

#include "codelibrary/base/array.h"
#include "codelibrary/base/log.h"
#include "codelibrary/util/io/endian_util.h"

namespace cl {

/**
 * Binary stream IO.
 */
class BinaryStream {
    using Endian = endian_util::Endian;

    static constexpr int EOF_FLAG  = 1;
    static constexpr int FAIL_FLAG = 2;

public:
    explicit BinaryStream(Endian endian = Endian::NATIVE)
        : endian_(endian) {}

    explicit BinaryStream(const uint8_t* begin, const uint8_t* end,
                          Endian endian = Endian::NATIVE)
        : pos_(begin), end_(end), endian_(endian) {}

    /**
     * Reset the stream.
     */
    void Reset(const uint8_t* begin, const uint8_t* end,
               Endian endian = Endian::NATIVE) {
        pos_ = begin;
        end_ = end;
        endian_ = endian;
    }

    /**
     * Read a n-bytes object.
     */
    template <typename T>
    bool Read(int n_bytes, T* v) {
        CHECK(n_bytes > 0);
        CHECK(v);

        if (!CanAdvance(n_bytes)) return false;

        if (endian_ != Endian::NATIVE) {
            Array<uint8_t> data(pos_, pos_ + n_bytes);
            std::reverse(data.begin(), data.end());
            std::memcpy(v, data.data(), n_bytes);
        } else {
            std::memcpy(v, pos_, n_bytes);
        }

        this->Advance(n_bytes);
        return true;
    }

    /**
     * Read a object with sizeof(object) bytes.
     */
    template <typename T>
    bool Read(T* v) {
        CHECK(v);

        size_t n = sizeof(T);
        CHECK(n > 0 && n < INT_MAX);
        int n_bytes = static_cast<int>(n);

        return Read(n_bytes, v);
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

    friend BinaryStream& operator >>(BinaryStream& stream, int8_t& v) {
       if (!stream.CanAdvance(1)) return stream;

        v = *stream.pos_;
        return stream.Advance(1);
    }

    friend BinaryStream& operator >>(BinaryStream& stream, uint8_t& v) {
        if (!stream.CanAdvance(1)) return stream;

        v = *stream.pos_;
        return stream.Advance(1);
    }

    friend BinaryStream& operator >>(BinaryStream& stream, int16_t& v) {
        if (!stream.CanAdvance(2)) return stream;

        std::memcpy(&v, stream.pos_, sizeof(v));
        if (stream.endian_ != Endian::NATIVE) {
            v = endian_util::Swap2(v);
        }

        return stream.Advance(2);
    }

    friend BinaryStream& operator >>(BinaryStream& stream, uint16_t& v) {
        if (!stream.CanAdvance(2)) return stream;

        std::memcpy(&v, stream.pos_, sizeof(v));
        if (stream.endian_ != Endian::NATIVE) {
            v = endian_util::Swap2(v);
        }

        return stream.Advance(2);
    }

    friend BinaryStream& operator >>(BinaryStream& stream, int32_t& v) {
        if (!stream.CanAdvance(4)) return stream;

        std::memcpy(&v, stream.pos_, sizeof(v));
        if (stream.endian_ != Endian::NATIVE) {
            v = endian_util::Swap4(v);
        }

        return stream.Advance(4);
    }

    friend BinaryStream& operator >>(BinaryStream& stream, uint32_t& v) {
        if (!stream.CanAdvance(4)) return stream;

        std::memcpy(&v, stream.pos_, sizeof(v));
        if (stream.endian_ != Endian::NATIVE) {
            v = endian_util::Swap4(v);
        }

        return stream.Advance(4);
    }

    friend BinaryStream& operator >>(BinaryStream& stream, int64_t& v) {
        if (!stream.CanAdvance(8)) return stream;

        std::memcpy(&v, stream.pos_, sizeof(v));
        if (stream.endian_ != Endian::NATIVE) {
            v = endian_util::Swap8(v);
        }

        return stream.Advance(8);
    }

    friend BinaryStream& operator >>(BinaryStream& stream, uint64_t& v) {
        if (!stream.CanAdvance(8)) return stream;

        std::memcpy(&v, stream.pos_, sizeof(v));
        if (stream.endian_ != Endian::NATIVE) {
            v = endian_util::Swap8(v);
        }

        return stream.Advance(8);
    }

    friend BinaryStream& operator >>(BinaryStream& stream, float& v) {
        if (!stream.CanAdvance(4)) return stream;

        if (stream.endian_ != Endian::NATIVE) {
            uint32_t temp;
            std::memcpy(&temp, stream.pos_, sizeof(temp));
            temp = endian_util::Swap4(temp);
            std::memcpy(&v, &temp, sizeof(v));
        } else {
            std::memcpy(&v, stream.pos_, sizeof(v));
        }

        return stream.Advance(4);
    }

    friend BinaryStream& operator >>(BinaryStream& stream, double& v) {
        if (!stream.CanAdvance(8)) return stream;

        if (stream.endian_ != Endian::NATIVE) {
            uint64_t temp;
            std::memcpy(&temp, stream.pos_, sizeof(temp));
            temp = endian_util::Swap8(temp);
            std::memcpy(&v, &temp, sizeof(v));
        } else {
            std::memcpy(&v, stream.pos_, sizeof(v));
        }

        return stream.Advance(8);
    }

private:
    /**
     * Check if the stream can advance by a specified number of bytes without
     * exceeding its bounds.
     */
    bool CanAdvance(int n_bytes) {
        if (pos_ + n_bytes > end_) {
            state_ |= FAIL_FLAG;
            return false;
        }

        state_ &= ~FAIL_FLAG;
        return true;
    }

    /**
     * Advance the stream forward N bytes without performing boundary checks.
     */
    BinaryStream& Advance(int n_bytes) {
        pos_ += n_bytes;

        if (pos_ == end_) {
            state_ |= EOF_FLAG;
        }
        return *this;
    }

    uint8_t const* pos_ = nullptr;
    uint8_t const* end_ = nullptr;
    Endian endian_;

    // State of stream.
    int state_ = 0;
};

} // namespace cl

#endif // CODELIBRARY_UTIL_IO_BINARY_STREAM_H_
