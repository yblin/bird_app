//
// Copyright 2015-2025 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_BASE_FLOAT_H_
#define CODELIBRARY_BASE_FLOAT_H_

#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <type_traits>

namespace cl {

template <typename T>
class Float {
    static_assert(std::is_floating_point<T>::value, "");

    Float() = delete;
};

template <>
class Float<float> {
    typedef union { float f; int i; } Float32;

public:
    static constexpr int N_MANTISSA_BITS = 23;
    static constexpr int EXPONENT_BIAS = 127;
    static constexpr int HIDDEN_BIT = 0x00800000;

    explicit Float(float x) {
        u_.f = x;
    }

    float value() const {
        return u_.f;
    }

    bool is_negative() const {
        return (u_.i & 0x80000000) != 0;
    }

    bool is_nonzero() const {
        return !(exponent() == 0 && mantissa() == 0);
    }

    /**
     * Get the (biased) exponent from the IEEE 754 representation.
     */
    int exponent() const {
        return (u_.i & 0x7F800000) >> N_MANTISSA_BITS;
    }

    int mantissa() const {
        return (u_.i & 0x007FFFFF);
    }

    bool is_inf() const {
        return exponent() == 0xFF && mantissa() == 0;
    }

    bool is_nan() const {
        return exponent() == 0xFF && mantissa() != 0;
    }

    /**
     * Return mantissa plus hidden bit.
     */
    int full_mantissa() const {
        int biased_exponent = exponent();
        int m = mantissa();
        if (biased_exponent == 0) return m;

        return m | HIDDEN_BIT;
    }

    /**
     * Return the unbiased (true) exponent value:
     *  For normalized numbers, it's (biased exponent - bias).
     *  For denormalized numbers, the exponent is 1 - bias.
     */
    int unbiased_exponent() const {
        int biased_exponent = exponent();
        if (biased_exponent == 0) {
            // Denormalized number (subnormal), exponent is 1 - bias.
            return 1 - EXPONENT_BIAS;
        } else {
            // Normalized number, subtract bias to get true exponent.
            return biased_exponent - EXPONENT_BIAS;
        }
    }

private:
    Float32 u_;
};

template <>
class Float<double> {
    typedef union { double f; int64_t i; } Float64;

public:
    static constexpr int N_MANTISSA_BITS = 52;
    static constexpr int EXPONENT_BIAS = 1023;
    static constexpr int64_t HIDDEN_BIT = 0x0010000000000000LL;

    explicit Float(double x) {
        u_.f = x;
    }

    double value() const {
        return u_.f;
    }

    bool is_negative() const {
        return (u_.i & 0x8000000000000000LL) != 0;
    }

    bool is_nonzero() const {
        return !(exponent() == 0 && mantissa() == 0);
    }

    /**
     * Get the (biased) exponent from the IEEE 754 representation.
     */
    int exponent() const {
        return (u_.i & 0x7FF0000000000000LL) >> N_MANTISSA_BITS;
    }

    int64_t mantissa() const {
        return (u_.i & 0x000FFFFFFFFFFFFFLL);
    }

    bool is_inf() const {
        return exponent() == 0x7FF && mantissa() == 0LL;
    }

    bool is_nan() const {
        return exponent() == 0x7FF && mantissa() != 0;
    }

    /**
     * Return mantissa plus hidden bit.
     */
    int64_t full_mantissa() const {
        int biased_exponent = exponent();
        int64_t m = mantissa();
        if (biased_exponent == 0) return m;

        return m | HIDDEN_BIT;
    }

    /**
     * Return the unbiased (true) exponent value:
     *  For normalized numbers, it's (biased exponent - bias).
     *  For denormalized numbers, the exponent is 1 - bias.
     */
    int unbiased_exponent() const {
        int biased_exponent = exponent();
        if (biased_exponent == 0) {
            // Denormalized number (subnormal), exponent is 1 - bias.
            return 1 - EXPONENT_BIAS;
        } else {
            // Normalized number, subtract bias to get true exponent.
            return biased_exponent - EXPONENT_BIAS;
        }
    }

private:
    Float64 u_;
};

/**
 * Convert float and double to string by Grisu2 algorithm.
 *
 * The following code is adapted by Milo Yip's dtoa_milo.h.
 *
 * Reference:
 *   Loitsch, Florian. "Printing floating-point numbers quickly and accurately
 *   with integers." ACM Sigplan Notices 45.6 (2010): 233-243.
 */
class FloatFormatter {
    struct FP {
        FP() {}

        FP(uint64_t f, int e) : f(f), e(e) {}

        explicit FP(float v) {
            Float<float> d(v);

            int biased_e = d.exponent();
            uint64_t significand = d.mantissa();

            // Note that it is not equal to 127 in d.unbiased_exponent().
            const int exponent_bias = 150;

            if (biased_e != 0) {
                f = significand | Float<float>::HIDDEN_BIT;
                e = biased_e - exponent_bias;
            }
            else {
                f = significand;
                e = 1 - exponent_bias;
            }
        }

        explicit FP(double v) {
            Float<double> d(v);

            int biased_e = d.exponent();
            uint64_t significand = d.mantissa();

            // Note that it is not equal to 1023 in d.unbiased_exponent().
            const int exponent_bias = 1075;

            if (biased_e != 0) {
                f = significand | Float<double>::HIDDEN_BIT;
                e = biased_e - exponent_bias;
            }
            else {
                f = significand;
                e = 1 - exponent_bias;
            }
        }

        FP operator-(const FP& rhs) const {
            return FP(f - rhs.f, e);
        }

        FP operator*(const FP& rhs) const {
            const uint64_t M32 = 0xFFFFFFFF;
            const uint64_t a = f >> 32;
            const uint64_t b = f & M32;
            const uint64_t c = rhs.f >> 32;
            const uint64_t d = rhs.f & M32;
            const uint64_t ac = a * c;
            const uint64_t bc = b * c;
            const uint64_t ad = a * d;
            const uint64_t bd = b * d;
            uint64_t tmp = (bd >> 32) + (ad & M32) + (bc & M32);
            tmp += 1U << 31;  /// mult_round
            return FP(ac + (ad >> 32) + (bc >> 32) + (tmp >> 32),
                      e + rhs.e + 64);
        }

        FP Normalize() const {
            FP res = *this;
            while (!(res.f & Float<double>::HIDDEN_BIT)) {
                res.f <<= 1;
                res.e--;
            }
            res.f <<= (64 - Float<double>::N_MANTISSA_BITS - 1);
            res.e = res.e - (64 - Float<double>::N_MANTISSA_BITS - 1);
            return res;
        }

        FP NormalizeBoundary() const {
            FP res = *this;
            while (!(res.f & (Float<double>::HIDDEN_BIT << 1))) {
                res.f <<= 1;
                res.e--;
            }
            res.f <<= (64 - Float<double>::N_MANTISSA_BITS - 2);
            res.e = res.e - (64 - Float<double>::N_MANTISSA_BITS - 2);
            return res;
        }

        void NormalizedBoundaries(FP* minus, FP* plus) const {
            FP pl = FP((f << 1) + 1, e - 1).NormalizeBoundary();
            FP mi = (f == Float<double>::HIDDEN_BIT) ? FP((f << 2) - 1, e - 2)
                                                     : FP((f << 1) - 1, e - 1);
            mi.f <<= mi.e - pl.e;
            mi.e = pl.e;
            *plus = pl;
            *minus = mi;
        }

        uint64_t f;
        int e;
    };

public:
    FloatFormatter() = default;

    /**
     * Write a float-point number into buffer. The emitted string will never
     * exceed 24 characters
     * Return the length of the written digits.
     */
    template <typename T>
    int Write(T value, char* buffer) {
        static_assert(
            std::is_same<T, float>::value || std::is_same<T, double>::value,
            "T must be float or double"
        );

        if (std::isnan(value)) {
            buffer[0] = 'n';
            buffer[1] = 'a';
            buffer[2] = 'n';
            buffer[3] = '\0';
            return 3;
        }

        int sign = 0;
        if (std::isinf(value)) {
            if (value < 0.0) {
                *buffer++ = '-';
                sign = 1;
            }
            buffer[0] = 'i';
            buffer[1] = 'n';
            buffer[2] = 'f';
            buffer[3] = '\0';
            return 3 + sign;
        }

        if (value == 0) {
            buffer[0] = '0';
            buffer[1] = '.';
            buffer[2] = '0';
            buffer[3] = '\0';
            return 3;
        }

        if (value < 0) {
            *buffer++ = '-';
            value = -value;
            sign = 1;
        }
        int length, k;
        Grisu2(value, buffer, &length, &k);
        Prettify(length, k, buffer);

        return sign + std::strlen(buffer);
    }

private:
    /**
     * Simple pure C++ implementation was faster than __builtin_clz version in
     * this situation.
     */
    static int CountDigit(uint32_t n) {
        if (n < 10) return 1;
        if (n < 100) return 2;
        if (n < 1000) return 3;
        if (n < 10000) return 4;
        if (n < 100000) return 5;
        if (n < 1000000) return 6;
        if (n < 10000000) return 7;
        if (n < 100000000) return 8;
        if (n < 1000000000) return 9;
        return 10;
    }

    /**
     * Lookup table to find a siutable K.
     */
    FP GetCachedPower(int e, int* k) {
        // 10^-348, 10^-340, ..., 10^340
        static const FP powers_ten[] = {
            { 18054884314459144840U, -1220 }, { 13451937075301367670U, -1193 },
            { 10022474136428063862U, -1166 }, { 14934650266808366570U, -1140 },
            { 11127181549972568877U, -1113 }, { 16580792590934885855U, -1087 },
            { 12353653155963782858U, -1060 }, { 18408377700990114895U, -1034 },
            { 13715310171984221708U, -1007 }, { 10218702384817765436U, -980 },
            { 15227053142812498563U, -954 },  { 11345038669416679861U, -927 },
            { 16905424996341287883U, -901 },  { 12595523146049147757U, -874 },
            { 9384396036005875287U,  -847 },  { 13983839803942852151U, -821 },
            { 10418772551374772303U, -794 },  { 15525180923007089351U, -768 },
            { 11567161174868858868U, -741 },  { 17236413322193710309U, -715 },
            { 12842128665889583758U, -688 },  { 9568131466127621947U,  -661 },
            { 14257626930069360058U, -635 },  { 10622759856335341974U, -608 },
            { 15829145694278690180U, -582 },  { 11793632577567316726U, -555 },
            { 17573882009934360870U, -529 },  { 13093562431584567480U, -502 },
            { 9755464219737475723U,  -475 },  { 14536774485912137811U, -449 },
            { 10830740992659433045U, -422 },  { 16139061738043178685U, -396 },
            { 12024538023802026127U, -369 },  { 17917957937422433684U, -343 },
            { 13349918974505688015U, -316 },  { 9946464728195732843U,  -289 },
            { 14821387422376473014U, -263 },  { 11042794154864902060U, -236 },
            { 16455045573212060422U, -210 },  { 12259964326927110867U, -183 },
            { 18268770466636286478U, -157 },  { 13611294676837538539U, -130 },
            { 10141204801825835212U, -103 },  { 15111572745182864684U, -77 },
            { 11258999068426240000U, -50 },   { 16777216000000000000U, -24 },
            { 12500000000000000000U,   3 },   { 9313225746154785156U,   30 },
            { 13877787807814456755U,  56 },   { 10339757656912845936U,  83 },
            { 15407439555097886824U, 109 },   { 11479437019748901445U, 136 },
            { 17105694144590052135U, 162 },   { 12744735289059618216U, 189 },
            { 9495567745759798747U,  216 },   { 14149498560666738074U, 242 },
            { 10542197943230523224U, 269 },   { 15709099088952724970U, 295 },
            { 11704190886730495818U, 322 },   { 17440603504673385349U, 348 },
            { 12994262207056124023U, 375 },   { 9681479787123295682U,  402 },
            { 14426529090290212157U, 428 },   { 10748601772107342003U, 455 },
            { 16016664761464807395U, 481 },   { 11933345169920330789U, 508 },
            { 17782069995880619868U, 534 },   { 13248674568444952270U, 561 },
            { 9871031767461413346U,  588 },   { 14708983551653345445U, 614 },
            { 10959046745042015199U, 641 },   { 16330252207878254650U, 667 },
            { 12166986024289022870U, 694 },   { 18130221999122236476U, 720 },
            { 13508068024458167312U, 747 },   { 10064294952495520794U, 774 },
            { 14996968138956309548U, 800 },   { 11173611982879273257U, 827 },
            { 16649979327439178909U, 853 },   { 12405201291620119593U, 880 },
            { 9242595204427927429U,  907 },   { 13772540099066387757U, 933 },
            { 10261342003245940623U, 960 },   { 15290591125556738113U, 986 },
            { 11392378155556871081U, 1013 },  { 16975966327722178521U, 1039 },
            { 12648080533535911531U, 1066 }
        };

        // dk must be positive, so can do ceiling in positive.
        double dk = (-61 - e) * 0.30102999566398114 + 347;
        int k1 = static_cast<int>(dk);
        if (dk - k1 > 0.0)
            ++k1;

        unsigned index = static_cast<unsigned>(k1 >> 3) + 1;
        // Decimal exponent no need lookup table.
        *k = -(-348 + static_cast<int>(index << 3));

        assert(index < sizeof(powers_ten) / sizeof(powers_ten[0]));
        return powers_ten[index];
    }

    void GrisuRound(int len, uint64_t delta, uint64_t rest,
                    uint64_t ten_kappa, uint64_t wp_w, char* buffer) {
        while (rest < wp_w && delta - rest >= ten_kappa &&
               (rest + ten_kappa < wp_w ||  /// closer
                wp_w - rest > rest + ten_kappa - wp_w)) {
            --buffer[len - 1];
            rest += ten_kappa;
        }
    }

    void DigitGen(const FP& w, const FP& mp, uint64_t delta, char* buffer,
                  int* len, int* k) {
        static const uint32_t pow10[] = {
            1, 10, 100, 1000, 10000, 100000, 1000000, 10000000,
            100000000, 1000000000
        };

        const FP one(uint64_t(1) << -mp.e, mp.e);
        const FP wp_w = mp - w;
        uint32_t p1 = static_cast<uint32_t>(mp.f >> -one.e);
        uint64_t p2 = mp.f & (one.f - 1);
        int kappa = CountDigit(p1);
        *len = 0;

        while (kappa > 0) {
            uint32_t d;
            switch (kappa) {
                case 10: d = p1 / 1000000000; p1 %= 1000000000; break;
                case  9: d = p1 /  100000000; p1 %=  100000000; break;
                case  8: d = p1 /   10000000; p1 %=   10000000; break;
                case  7: d = p1 /    1000000; p1 %=    1000000; break;
                case  6: d = p1 /     100000; p1 %=     100000; break;
                case  5: d = p1 /      10000; p1 %=      10000; break;
                case  4: d = p1 /       1000; p1 %=       1000; break;
                case  3: d = p1 /        100; p1 %=        100; break;
                case  2: d = p1 /         10; p1 %=         10; break;
                case  1: d = p1;              p1 =           0; break;
                default:
                    d = 0;
            }
            if (d || *len)
                buffer[(*len)++] = '0' + static_cast<char>(d);
            --kappa;
            uint64_t tmp = (static_cast<uint64_t>(p1) << -one.e) + p2;
            if (tmp <= delta) {
                *k += kappa;
                GrisuRound(*len, delta, tmp,
                           static_cast<uint64_t>(pow10[kappa]) << -one.e,
                           wp_w.f,
                           buffer);
                return;
            }
        }

        // kappa = 0
        for (;;) {
            p2 *= 10;
            delta *= 10;
            char d = static_cast<char>(p2 >> -one.e);
            if (d || *len)
                buffer[(*len)++] = '0' + d;
            p2 &= one.f - 1;
            --kappa;
            if (p2 < delta) {
                *k += kappa;
                GrisuRound(*len, delta, p2, one.f, wp_w.f * pow10[-kappa],
                           buffer);
                return;
            }
        }
    }

    /**
     * Grisu2 algorithm to write the double number into buffer.
     */
    template <typename T>
    void Grisu2(T value, char* buffer, int* length, int* k) {
        const FP v(value);
        FP w_m, w_p;
        v.NormalizedBoundaries(&w_m, &w_p);

        const FP c_mk = GetCachedPower(w_p.e, k);
        const FP w = v.Normalize() * c_mk;
        FP wp = w_p * c_mk;
        FP wm = w_m * c_mk;
        ++wm.f;
        --wp.f;

        DigitGen(w, wp, wp.f - wm.f, buffer, length, k);
    }

    /**
     * Write exponent k into buffer.
     */
    void WriteExponent(int k, char* buffer) {
        static constexpr char radix_100_table[200] = {
            '0', '0', '0', '1', '0', '2', '0', '3', '0', '4', //
            '0', '5', '0', '6', '0', '7', '0', '8', '0', '9', //
            '1', '0', '1', '1', '1', '2', '1', '3', '1', '4', //
            '1', '5', '1', '6', '1', '7', '1', '8', '1', '9', //
            '2', '0', '2', '1', '2', '2', '2', '3', '2', '4', //
            '2', '5', '2', '6', '2', '7', '2', '8', '2', '9', //
            '3', '0', '3', '1', '3', '2', '3', '3', '3', '4', //
            '3', '5', '3', '6', '3', '7', '3', '8', '3', '9', //
            '4', '0', '4', '1', '4', '2', '4', '3', '4', '4', //
            '4', '5', '4', '6', '4', '7', '4', '8', '4', '9', //
            '5', '0', '5', '1', '5', '2', '5', '3', '5', '4', //
            '5', '5', '5', '6', '5', '7', '5', '8', '5', '9', //
            '6', '0', '6', '1', '6', '2', '6', '3', '6', '4', //
            '6', '5', '6', '6', '6', '7', '6', '8', '6', '9', //
            '7', '0', '7', '1', '7', '2', '7', '3', '7', '4', //
            '7', '5', '7', '6', '7', '7', '7', '8', '7', '9', //
            '8', '0', '8', '1', '8', '2', '8', '3', '8', '4', //
            '8', '5', '8', '6', '8', '7', '8', '8', '8', '9', //
            '9', '0', '9', '1', '9', '2', '9', '3', '9', '4', //
            '9', '5', '9', '6', '9', '7', '9', '8', '9', '9'  //
        };

        if (k < 0) {
            *buffer++ = '-';
            k = -k;
        }

        if (k >= 100) {
            *buffer++ = '0' + static_cast<char>(k / 100);
            k %= 100;
            const char* d = radix_100_table + k * 2;
            *buffer++ = d[0];
            *buffer++ = d[1];
        } else if (k >= 10) {
            const char* d = radix_100_table + k * 2;
            *buffer++ = d[0];
            *buffer++ = d[1];
        } else
            *buffer++ = '0' + static_cast<char>(k);
        *buffer = '\0';
    }

    /**
     * Fine tune the output string buffer.
     */
    void Prettify(int length, int k, char* buffer) {
        const int kk = length + k;	// 10^(kk-1) <= v < 10^kk

        if (length <= kk && kk <= 21) {
            // 1234e7 -> 12340000000
            for (int i = length; i < kk; ++i)
                buffer[i] = '0';
            buffer[kk] = '\0';
        } else if (0 < kk && kk <= 21) {
            // 1234e-2 -> 12.34
            std::memmove(&buffer[kk + 1], &buffer[kk], length - kk);
            buffer[kk] = '.';
            buffer[length + 1] = '\0';
        } else if (-6 < kk && kk <= 0) {
            // 1234e-6 -> 0.001234
            const int offset = 2 - kk;
            std::memmove(&buffer[offset], &buffer[0], length);
            buffer[0] = '0';
            buffer[1] = '.';
            for (int i = 2; i < offset; i++)
                buffer[i] = '0';
            buffer[length + offset] = '\0';
        } else if (length == 1) {
            // 1e30
            buffer[1] = 'e';
            WriteExponent(kk - 1, &buffer[2]);
        } else {
            // 1234e30 -> 1.234e33
            std::memmove(&buffer[2], &buffer[1], length - 1);
            buffer[1] = '.';
            buffer[length + 1] = 'e';
            WriteExponent(kk - 1, &buffer[length + 2]);
        }
    }
};

} // namespace cl

#endif // CODELIBRARY_BASE_FLOAT_H_
