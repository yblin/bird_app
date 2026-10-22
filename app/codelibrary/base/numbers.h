//
// Copyright 2024 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//
// Implement c++20 <numbers>.
//

#ifndef CODELIBRARY_BASE_NUMBERS_H_
#define CODELIBRARY_BASE_NUMBERS_H_

namespace cl {
namespace numbers {

template <typename T>
constexpr T e_v = static_cast<T>(2.718281828459045);
template <typename T>
constexpr T log2e_v = static_cast<T>(1.4426950408889634);
template <typename T>
constexpr T log10e_v = static_cast<T>(0.4342944819032518);
template <typename T>
constexpr T pi_v = static_cast<T>(3.141592653589793);
template <typename T>
constexpr T inv_pi_v = static_cast<T>(0.3183098861837907);
template <typename T>
constexpr T inv_sqrtpi_v = static_cast<T>(0.5641895835477563);
template <typename T>
constexpr T ln2_v = static_cast<T>(0.6931471805599453);
template <typename T>
constexpr T ln10_v = static_cast<T>(2.302585092994046);
template <typename T>
constexpr T sqrt2_v = static_cast<T>(1.4142135623730951);
template <typename T>
constexpr T sqrt3_v = static_cast<T>(1.7320508075688772);
template <typename T>
constexpr T inv_sqrt3_v = static_cast<T>(0.5773502691896257);
template <typename T>
constexpr T egamma_v = static_cast<T>(0.5772156649015329);
template <typename T>
constexpr T phi_v = static_cast<T>(1.618033988749895);

template <>
inline constexpr double e_v<double> = 2.718281828459045;
template <>
inline constexpr double log2e_v<double> = 1.4426950408889634;
template <>
inline constexpr double log10e_v<double> = 0.4342944819032518;
template <>
inline constexpr double pi_v<double> = 3.141592653589793;
template <>
inline constexpr double inv_pi_v<double> = 0.3183098861837907;
template <>
inline constexpr double inv_sqrtpi_v<double> = 0.5641895835477563;
template <>
inline constexpr double ln2_v<double> = 0.6931471805599453;
template <>
inline constexpr double ln10_v<double> = 2.302585092994046;
template <>
inline constexpr double sqrt2_v<double> = 1.4142135623730951;
template <>
inline constexpr double sqrt3_v<double> = 1.7320508075688772;
template <>
inline constexpr double inv_sqrt3_v<double> = 0.5773502691896257;
template <>
inline constexpr double egamma_v<double> = 0.5772156649015329;
template <>
inline constexpr double phi_v<double> = 1.618033988749895;

template <>
inline constexpr float e_v<float> = static_cast<float>(e_v<double>);
template <>
inline constexpr float log2e_v<float> = static_cast<float>(log2e_v<double>);
template <>
inline constexpr float log10e_v<float> = static_cast<float>(log10e_v<double>);
template <>
inline constexpr float pi_v<float> = static_cast<float>(pi_v<double>);
template <>
inline constexpr float inv_pi_v<float> = static_cast<float>(inv_pi_v<double>);
template <>
inline constexpr float
inv_sqrtpi_v<float> = static_cast<float>(inv_sqrtpi_v<double>);
template <>
inline constexpr float ln2_v<float> = static_cast<float>(ln2_v<double>);
template <>
inline constexpr float ln10_v<float> = static_cast<float>(ln10_v<double>);
template <>
inline constexpr float sqrt2_v<float> = static_cast<float>(sqrt2_v<double>);
template <>
inline constexpr float sqrt3_v<float> = static_cast<float>(sqrt3_v<double>);
template <>
inline constexpr float
inv_sqrt3_v<float> = static_cast<float>(inv_sqrt3_v<double>);
template <>
inline constexpr float egamma_v<float> = static_cast<float>(egamma_v<double>);
template <>
inline constexpr float phi_v<float> = static_cast<float>(phi_v<double>);

template <>
inline constexpr long double e_v<long double> = e_v<double>;
template <>
inline constexpr long double log2e_v<long double> = log2e_v<double>;
template <>
inline constexpr long double log10e_v<long double> = log10e_v<double>;
template <>
inline constexpr long double pi_v<long double> = pi_v<double>;
template <>
inline constexpr long double inv_pi_v<long double> = inv_pi_v<double>;
template <>
inline constexpr long double inv_sqrtpi_v<long double> = inv_sqrtpi_v<double>;
template <>
inline constexpr long double ln2_v<long double> = ln2_v<double>;
template <>
inline constexpr long double ln10_v<long double> = ln10_v<double>;
template <>
inline constexpr long double sqrt2_v<long double> = sqrt2_v<double>;
template <>
inline constexpr long double sqrt3_v<long double> = sqrt3_v<double>;
template <>
inline constexpr long double inv_sqrt3_v<long double> = inv_sqrt3_v<double>;
template <>
inline constexpr long double egamma_v<long double> = egamma_v<double>;
template <>
inline constexpr long double phi_v<long double> = phi_v<double>;

inline constexpr double e          = e_v<double>;
inline constexpr double log2e      = log2e_v<double>;
inline constexpr double log10e     = log10e_v<double>;
inline constexpr double pi         = pi_v<double>;
inline constexpr double inv_pi     = inv_pi_v<double>;
inline constexpr double inv_sqrtpi = inv_sqrtpi_v<double>;
inline constexpr double ln2        = ln2_v<double>;
inline constexpr double ln10       = ln10_v<double>;
inline constexpr double sqrt2      = sqrt2_v<double>;
inline constexpr double sqrt3      = sqrt3_v<double>;
inline constexpr double inv_sqrt3  = inv_sqrt3_v<double>;
inline constexpr double egamma     = egamma_v<double>;
inline constexpr double phi        = phi_v<double>;

} // namespace numbers
} // namespace cl

#endif  // CODELIBRARY_BASE_NUMBERS_H_
