/**************************************************************************/
/*  fp_deterministic_math.h                                               */
/**************************************************************************/
/*                         This file is part of:                          */
/*                             GODOT ENGINE                               */
/*                        https://godotengine.org                         */
/**************************************************************************/
/* Copyright (c) 2014-present Godot Engine contributors (see AUTHORS.md). */
/* Copyright (c) 2007-2014 Juan Linietsky, Ariel Manzur.                  */
/*                                                                        */
/* Permission is hereby granted, free of charge, to any person obtaining  */
/* a copy of this software and associated documentation files (the        */
/* "Software"), to deal in the Software without restriction, including    */
/* without limitation the rights to use, copy, modify, merge, publish,    */
/* distribute, sublicense, and/or sell copies of the Software, and to     */
/* permit persons to whom the Software is furnished to do so, subject to  */
/* the following conditions:                                              */
/*                                                                        */
/* The above copyright notice and this permission notice shall be         */
/* included in all copies or substantial portions of the Software.        */
/*                                                                        */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,        */
/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF     */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. */
/* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY   */
/* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,   */
/* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE      */
/* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                 */
/**************************************************************************/

#pragma once

// Cross-platform deterministic math functions.
// These implementations use fixed polynomial approximations to ensure
// identical results across all platforms and compilers.
//
// Based on the Cephes math library approach, similar to Jolt Physics.

#include <cmath>
#include <cstdint>
#include <cstring>

namespace FPDeterministic {

// Constants
constexpr double PI_D = 3.14159265358979323846;
constexpr double PI_2_D = 1.57079632679489661923; // PI / 2
constexpr double PI_4_D = 0.78539816339744830962; // PI / 4

// ============================================================================
// DOUBLE-PRECISION TRIGONOMETRIC FUNCTIONS
// Polynomial approximations based on Cephes library
// ============================================================================

// Helper: Reduce angle to [-PI/4, PI/4] range
inline void sincos_reduce(double x, double &reduced, int &quadrant) {
	// Use extended precision for reduction
	constexpr double DP1 = 7.85398125648498535156e-1;
	constexpr double DP2 = 3.77489470793079817668e-8;
	constexpr double DP3 = 2.69515142907905952645e-15;
	constexpr double FOPI = 1.27323954473516268615; // 4/PI

	double y = std::floor(x * FOPI);
	quadrant = static_cast<int>(y);

	// Ensure quadrant is even for cosine symmetry
	if (quadrant & 1) {
		quadrant++;
		y += 1.0;
	}
	quadrant &= 7;

	// Extended precision modular arithmetic
	reduced = ((x - y * DP1) - y * DP2) - y * DP3;
}

// Polynomial coefficients for sin (Cephes)
inline double sin_poly(double x) {
	constexpr double S0 = -1.66666666666666324348e-1;
	constexpr double S1 = 8.33333333332248946124e-3;
	constexpr double S2 = -1.98412698298579493134e-4;
	constexpr double S3 = 2.75573137070700676789e-6;
	constexpr double S4 = -2.50507602534068634195e-8;
	constexpr double S5 = 1.58969099521155010221e-10;

	double x2 = x * x;
	return x + x * x2 * (S0 + x2 * (S1 + x2 * (S2 + x2 * (S3 + x2 * (S4 + x2 * S5)))));
}

// Polynomial coefficients for cos (Cephes)
inline double cos_poly(double x) {
	constexpr double C0 = 4.16666666666666019037e-2;
	constexpr double C1 = -1.38888888888741095749e-3;
	constexpr double C2 = 2.48015872894767294178e-5;
	constexpr double C3 = -2.75573143513906633035e-7;
	constexpr double C4 = 2.08757232129817482790e-9;
	constexpr double C5 = -1.13596475577881948265e-11;

	double x2 = x * x;
	return 1.0 - 0.5 * x2 + x2 * x2 * (C0 + x2 * (C1 + x2 * (C2 + x2 * (C3 + x2 * (C4 + x2 * C5)))));
}

inline double sin(double x) {
	// Handle special cases
	if (x == 0.0) {
		return x;
	}

	// Make argument positive
	double sign = 1.0;
	if (x < 0.0) {
		x = -x;
		sign = -1.0;
	}

	double reduced;
	int quadrant;
	sincos_reduce(x, reduced, quadrant);

	// Adjust sign based on quadrant
	if (quadrant > 3) {
		sign = -sign;
		quadrant -= 4;
	}

	double result;
	if (quadrant == 0 || quadrant == 2) {
		result = sin_poly(reduced);
	} else {
		result = cos_poly(reduced);
	}

	if (quadrant == 2 || quadrant == 3) {
		result = -result;
	}

	return sign * result;
}

inline double cos(double x) {
	// Make argument positive (cos is symmetric)
	if (x < 0.0) {
		x = -x;
	}

	double reduced;
	int quadrant;
	sincos_reduce(x, reduced, quadrant);

	double sign = 1.0;
	if (quadrant > 3) {
		quadrant -= 4;
	}

	double result;
	if (quadrant == 0) {
		result = cos_poly(reduced);
	} else if (quadrant == 1) {
		result = -sin_poly(reduced);
	} else if (quadrant == 2) {
		result = -cos_poly(reduced);
	} else {
		result = sin_poly(reduced);
	}

	return result;
}

inline double tan(double x) {
	double s = sin(x);
	double c = cos(x);
	// Handle division by zero
	if (c == 0.0) {
		return (s > 0.0) ? 1e308 : -1e308;
	}
	return s / c;
}

// Arc sine using polynomial approximation
inline double asin(double x) {
	// Clamp to valid range
	if (x <= -1.0) {
		return -PI_2_D;
	}
	if (x >= 1.0) {
		return PI_2_D;
	}

	double sign = 1.0;
	if (x < 0.0) {
		x = -x;
		sign = -1.0;
	}

	double result;
	if (x <= 0.5) {
		// Use direct polynomial for small x
		constexpr double P0 = -8.198089802484824371615e-1;
		constexpr double P1 = 1.956261983317594739197e1;
		constexpr double P2 = -1.011204800315498822197e2;
		constexpr double P3 = 1.950096193800715600000e2;
		constexpr double P4 = -1.168884943883949020000e2;
		constexpr double Q0 = -4.918853881490881290850e0;
		constexpr double Q1 = 4.052060926609787800000e1;
		constexpr double Q2 = -1.225175041340946800000e2;
		constexpr double Q3 = 1.622106448044847900000e2;
		constexpr double Q4 = -7.784318178609090000000e1;

		double x2 = x * x;
		double p = x2 * (P0 + x2 * (P1 + x2 * (P2 + x2 * (P3 + x2 * P4))));
		double q = 1.0 + x2 * (Q0 + x2 * (Q1 + x2 * (Q2 + x2 * (Q3 + x2 * Q4))));
		result = x + x * p / q;
	} else {
		// Use identity: asin(x) = PI/2 - 2*asin(sqrt((1-x)/2)) for x > 0.5
		double z = 0.5 * (1.0 - x);
		double s = std::sqrt(z);

		constexpr double P0 = -8.198089802484824371615e-1;
		constexpr double P1 = 1.956261983317594739197e1;
		constexpr double P2 = -1.011204800315498822197e2;
		constexpr double P3 = 1.950096193800715600000e2;
		constexpr double P4 = -1.168884943883949020000e2;
		constexpr double Q0 = -4.918853881490881290850e0;
		constexpr double Q1 = 4.052060926609787800000e1;
		constexpr double Q2 = -1.225175041340946800000e2;
		constexpr double Q3 = 1.622106448044847900000e2;
		constexpr double Q4 = -7.784318178609090000000e1;

		double p = z * (P0 + z * (P1 + z * (P2 + z * (P3 + z * P4))));
		double q = 1.0 + z * (Q0 + z * (Q1 + z * (Q2 + z * (Q3 + z * Q4))));
		result = PI_2_D - 2.0 * (s + s * p / q);
	}

	return sign * result;
}

inline double acos(double x) {
	// Clamp to valid range
	if (x <= -1.0) {
		return PI_D;
	}
	if (x >= 1.0) {
		return 0.0;
	}

	return PI_2_D - asin(x);
}

// Arc tangent using polynomial approximation
inline double atan(double x) {
	double sign = 1.0;
	if (x < 0.0) {
		x = -x;
		sign = -1.0;
	}

	double result;
	if (x <= 0.66) {
		// Use direct polynomial
		constexpr double P0 = -8.750608600031904122785e-1;
		constexpr double P1 = -1.615753718733365076637e1;
		constexpr double P2 = -7.500855792314704667340e1;
		constexpr double P3 = -1.228866684490136173410e2;
		constexpr double P4 = -6.485021904942025371773e1;
		constexpr double Q0 = 2.485846490142306297962e1;
		constexpr double Q1 = 1.650270098316988542046e2;
		constexpr double Q2 = 4.328810604912902668951e2;
		constexpr double Q3 = 4.853903996359136964868e2;
		constexpr double Q4 = 1.945506571482613964425e2;

		double x2 = x * x;
		double p = x2 * (P0 + x2 * (P1 + x2 * (P2 + x2 * (P3 + x2 * P4))));
		double q = 1.0 + x2 * (Q0 + x2 * (Q1 + x2 * (Q2 + x2 * (Q3 + x2 * Q4))));
		result = x + x * p / q;
	} else if (x <= 2.414213562373095) {
		// Use identity: atan(x) = PI/4 + atan((x-1)/(x+1)) for x in [tan(PI/8), tan(3*PI/8)]
		double y = (x - 1.0) / (x + 1.0);

		constexpr double P0 = -8.750608600031904122785e-1;
		constexpr double P1 = -1.615753718733365076637e1;
		constexpr double P2 = -7.500855792314704667340e1;
		constexpr double P3 = -1.228866684490136173410e2;
		constexpr double P4 = -6.485021904942025371773e1;
		constexpr double Q0 = 2.485846490142306297962e1;
		constexpr double Q1 = 1.650270098316988542046e2;
		constexpr double Q2 = 4.328810604912902668951e2;
		constexpr double Q3 = 4.853903996359136964868e2;
		constexpr double Q4 = 1.945506571482613964425e2;

		double y2 = y * y;
		double p = y2 * (P0 + y2 * (P1 + y2 * (P2 + y2 * (P3 + y2 * P4))));
		double q = 1.0 + y2 * (Q0 + y2 * (Q1 + y2 * (Q2 + y2 * (Q3 + y2 * Q4))));
		result = PI_4_D + y + y * p / q;
	} else {
		// Use identity: atan(x) = PI/2 - atan(1/x) for large x
		double y = 1.0 / x;

		constexpr double P0 = -8.750608600031904122785e-1;
		constexpr double P1 = -1.615753718733365076637e1;
		constexpr double P2 = -7.500855792314704667340e1;
		constexpr double P3 = -1.228866684490136173410e2;
		constexpr double P4 = -6.485021904942025371773e1;
		constexpr double Q0 = 2.485846490142306297962e1;
		constexpr double Q1 = 1.650270098316988542046e2;
		constexpr double Q2 = 4.328810604912902668951e2;
		constexpr double Q3 = 4.853903996359136964868e2;
		constexpr double Q4 = 1.945506571482613964425e2;

		double y2 = y * y;
		double p = y2 * (P0 + y2 * (P1 + y2 * (P2 + y2 * (P3 + y2 * P4))));
		double q = 1.0 + y2 * (Q0 + y2 * (Q1 + y2 * (Q2 + y2 * (Q3 + y2 * Q4))));
		result = PI_2_D - (y + y * p / q);
	}

	return sign * result;
}

inline double atan2(double y, double x) {
	// Handle special cases
	if (x == 0.0) {
		if (y > 0.0) {
			return PI_2_D;
		}
		if (y < 0.0) {
			return -PI_2_D;
		}
		return 0.0;
	}

	if (y == 0.0) {
		return (x > 0.0) ? 0.0 : PI_D;
	}

	double result = atan(y / x);

	if (x < 0.0) {
		if (y >= 0.0) {
			result += PI_D;
		} else {
			result -= PI_D;
		}
	}

	return result;
}

// ============================================================================
// EXPONENTIAL AND LOGARITHMIC FUNCTIONS
// Polynomial approximations for cross-platform determinism
// ============================================================================

inline double exp(double x) {
	// Handle edge cases
	if (x > 709.0) {
		return 1e308; // Overflow
	}
	if (x < -709.0) {
		return 0.0; // Underflow
	}
	if (x == 0.0) {
		return 1.0;
	}

	// Reduce to range [-0.5*ln(2), 0.5*ln(2)]
	constexpr double LOG2E = 1.4426950408889634073599;
	constexpr double LN2_HI = 6.93147180369123816490e-1;
	constexpr double LN2_LO = 1.90821492927058770002e-10;

	double k = std::floor(x * LOG2E + 0.5);
	double t = x - k * LN2_HI - k * LN2_LO;

	// Polynomial approximation for exp(t) - 1
	constexpr double P1 = 1.66666666666666019037e-1;
	constexpr double P2 = -2.77777777770155933842e-3;
	constexpr double P3 = 6.61375632143793436117e-5;
	constexpr double P4 = -1.65339022054652515390e-6;
	constexpr double P5 = 4.13813679705723846039e-8;

	double t2 = t * t;
	double c = t - t2 * (P1 + t2 * (P2 + t2 * (P3 + t2 * (P4 + t2 * P5))));
	double y = 1.0 - ((t * c) / (c - 2.0) - t);

	// Scale by 2^k using bit manipulation for determinism
	int64_t ki = static_cast<int64_t>(k);
	int64_t bits;
	std::memcpy(&bits, &y, sizeof(bits));
	bits += (ki << 52);
	std::memcpy(&y, &bits, sizeof(y));

	return y;
}

inline double log(double x) {
	// Handle edge cases
	if (x <= 0.0) {
		if (x == 0.0) {
			return -1e308; // -infinity
		}
		return 0.0 / 0.0; // NaN for negative
	}
	if (x == 1.0) {
		return 0.0;
	}

	// Extract exponent and mantissa
	int64_t bits;
	std::memcpy(&bits, &x, sizeof(bits));
	int e = static_cast<int>((bits >> 52) & 0x7FF) - 1023;
	bits = (bits & 0x000FFFFFFFFFFFFFLL) | 0x3FF0000000000000LL;
	double m;
	std::memcpy(&m, &bits, sizeof(m));

	// Adjust if mantissa > sqrt(2)
	if (m > 1.4142135623730950488) {
		m *= 0.5;
		e++;
	}

	// Use log(1+y) approximation where y = (m-1)/(m+1)
	double y = (m - 1.0) / (m + 1.0);
	double y2 = y * y;

	constexpr double L1 = 6.666666666666735130e-1;
	constexpr double L2 = 3.999999999940941908e-1;
	constexpr double L3 = 2.857142874366239149e-1;
	constexpr double L4 = 2.222219843214978396e-1;
	constexpr double L5 = 1.818357216161805012e-1;
	constexpr double L6 = 1.531383769920937332e-1;
	constexpr double L7 = 1.479819860511658591e-1;

	double r = y2 * (L1 + y2 * (L2 + y2 * (L3 + y2 * (L4 + y2 * (L5 + y2 * (L6 + y2 * L7))))));

	constexpr double LN2 = 0.69314718055994530942;
	return static_cast<double>(e) * LN2 + 2.0 * y + 2.0 * y * r;
}

inline double log2(double x) {
	constexpr double LOG2E = 1.4426950408889634073599;
	return log(x) * LOG2E;
}

inline double log1p(double x) {
	// For small x, log1p(x) ≈ x
	if (std::abs(x) < 1e-8) {
		return x;
	}
	return log(1.0 + x);
}

inline double pow(double base, double exponent) {
	// Handle special cases
	if (exponent == 0.0) {
		return 1.0;
	}
	if (base == 0.0) {
		return (exponent > 0.0) ? 0.0 : 1e308;
	}
	if (base == 1.0) {
		return 1.0;
	}
	if (exponent == 1.0) {
		return base;
	}
	if (exponent == 2.0) {
		return base * base;
	}

	// Check for integer exponent
	double int_part;
	if (std::modf(exponent, &int_part) == 0.0 && std::abs(int_part) < 64) {
		int n = static_cast<int>(int_part);
		if (n >= 0) {
			double result = 1.0;
			double b = base;
			while (n > 0) {
				if (n & 1) {
					result *= b;
				}
				b *= b;
				n >>= 1;
			}
			return result;
		} else {
			n = -n;
			double result = 1.0;
			double b = base;
			while (n > 0) {
				if (n & 1) {
					result *= b;
				}
				b *= b;
				n >>= 1;
			}
			return 1.0 / result;
		}
	}

	// General case: pow(x, y) = exp(y * log(x))
	if (base < 0.0) {
		// Negative base with non-integer exponent is undefined
		return 0.0 / 0.0; // NaN
	}

	return exp(exponent * log(base));
}

// ============================================================================
// HYPERBOLIC FUNCTIONS
// Defined in terms of exp for determinism
// ============================================================================

inline double sinh(double x) {
	if (std::abs(x) < 1e-5) {
		// Use Taylor series for small x: sinh(x) ≈ x + x³/6
		return x + (x * x * x) / 6.0;
	}
	double e = exp(x);
	return 0.5 * (e - 1.0 / e);
}

inline double cosh(double x) {
	double e = exp(x);
	return 0.5 * (e + 1.0 / e);
}

inline double tanh(double x) {
	if (x > 20.0) {
		return 1.0;
	}
	if (x < -20.0) {
		return -1.0;
	}
	double e2x = exp(2.0 * x);
	return (e2x - 1.0) / (e2x + 1.0);
}

inline double asinh(double x) {
	// asinh(x) = log(x + sqrt(x² + 1))
	double x2 = x * x;
	return log(x + std::sqrt(x2 + 1.0));
}

inline double acosh(double x) {
	if (x < 1.0) {
		return 0.0;
	}
	// acosh(x) = log(x + sqrt(x² - 1))
	return log(x + std::sqrt(x * x - 1.0));
}

inline double atanh(double x) {
	if (x <= -1.0) {
		return -1e308;
	}
	if (x >= 1.0) {
		return 1e308;
	}
	// atanh(x) = 0.5 * log((1 + x) / (1 - x))
	return 0.5 * log((1.0 + x) / (1.0 - x));
}

// ============================================================================
// FLOAT VERSIONS (using Jolt for trig, custom for exp/log)
// ============================================================================

inline float expf(float x) {
	return static_cast<float>(exp(static_cast<double>(x)));
}

inline float logf(float x) {
	return static_cast<float>(log(static_cast<double>(x)));
}

inline float log2f(float x) {
	return static_cast<float>(log2(static_cast<double>(x)));
}

inline float log1pf(float x) {
	return static_cast<float>(log1p(static_cast<double>(x)));
}

inline float powf(float base, float exponent) {
	return static_cast<float>(pow(static_cast<double>(base), static_cast<double>(exponent)));
}

inline float sinhf(float x) {
	return static_cast<float>(sinh(static_cast<double>(x)));
}

inline float coshf(float x) {
	return static_cast<float>(cosh(static_cast<double>(x)));
}

inline float tanhf(float x) {
	return static_cast<float>(tanh(static_cast<double>(x)));
}

inline float asinhf(float x) {
	return static_cast<float>(asinh(static_cast<double>(x)));
}

inline float acoshf(float x) {
	return static_cast<float>(acosh(static_cast<double>(x)));
}

inline float atanhf(float x) {
	return static_cast<float>(atanh(static_cast<double>(x)));
}

} // namespace FPDeterministic
