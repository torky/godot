/*
 * Deterministic math functions for cross-platform consistency.
 *
 * Copyright Naoki Shibata and contributors 2010 - 2025.
 * Distributed under the Boost Software License, Version 1.0.
 *    (See accompanying file LICENSE.txt or copy at
 *          http://www.boost.org/LICENSE_1_0.txt)
 *
 * Adapted for Godot Engine deterministic math requirements.
 *
 * Based on algorithms from SLEEF (SIMD Library for Evaluating Elementary Functions):
 *   https://github.com/shibatch/sleef
 *
 * Source files referenced:
 *   - src/libm/sleefsimddp.c  (double precision math functions)
 *   - src/libm/sleefsimdsp.c  (single precision math functions)
 *   - src/arch/helperpurec_scalar.h (pure C scalar implementations)
 *   - src/common/misc.h (constants and helper macros)
 */

#ifndef SLEEF_DETERMINISTIC_H
#define SLEEF_DETERMINISTIC_H

#include <stdint.h>
#include <string.h>
#include <float.h>
#include <math.h>

#ifdef _MSC_VER
#define sleef_sqrt(x) sqrt(x)
#define sleef_sqrtf(x) sqrtf(x)
#else
#define sleef_sqrt(x) __builtin_sqrt(x)
#define sleef_sqrtf(x) __builtin_sqrtf(x)
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Bit manipulation utilities for IEEE 754 doubles and floats
 * ============================================================================ */

typedef union {
	double value;
	struct {
#if defined(__BYTE_ORDER__) && __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
		uint32_t msw;
		uint32_t lsw;
#else
		uint32_t lsw;
		uint32_t msw;
#endif
	} parts;
	uint64_t bits;
} sleef_ieee_double;

typedef union {
	float value;
	uint32_t bits;
} sleef_ieee_float;

#define SLEEF_EXTRACT_WORDS(ix0, ix1, d) \
	do { \
		sleef_ieee_double ew_u; \
		ew_u.value = (d); \
		(ix0) = ew_u.parts.msw; \
		(ix1) = ew_u.parts.lsw; \
	} while (0)

#define SLEEF_GET_HIGH_WORD(i, d) \
	do { \
		sleef_ieee_double gh_u; \
		gh_u.value = (d); \
		(i) = gh_u.parts.msw; \
	} while (0)

#define SLEEF_GET_LOW_WORD(i, d) \
	do { \
		sleef_ieee_double gl_u; \
		gl_u.value = (d); \
		(i) = gl_u.parts.lsw; \
	} while (0)

#define SLEEF_INSERT_WORDS(d, ix0, ix1) \
	do { \
		sleef_ieee_double iw_u; \
		iw_u.parts.msw = (ix0); \
		iw_u.parts.lsw = (ix1); \
		(d) = iw_u.value; \
	} while (0)

#define SLEEF_SET_HIGH_WORD(d, v) \
	do { \
		sleef_ieee_double sh_u; \
		sh_u.value = (d); \
		sh_u.parts.msw = (v); \
		(d) = sh_u.value; \
	} while (0)

#define SLEEF_SET_LOW_WORD(d, v) \
	do { \
		sleef_ieee_double sl_u; \
		sl_u.value = (d); \
		sl_u.parts.lsw = (v); \
		(d) = sl_u.value; \
	} while (0)

#define SLEEF_GET_FLOAT_WORD(i, d) \
	do { \
		sleef_ieee_float gf_u; \
		gf_u.value = (d); \
		(i) = gf_u.bits; \
	} while (0)

#define SLEEF_SET_FLOAT_WORD(d, i) \
	do { \
		sleef_ieee_float sf_u; \
		sf_u.bits = (i); \
		(d) = sf_u.value; \
	} while (0)

/* ============================================================================
 * Constants
 * ============================================================================ */

static const double sleef_huge = 1.0e+300;
static const double sleef_tiny = 1.0e-300;
static const double sleef_twom1000 = 9.33263618503218878990e-302; /* 2**-1000 */
static const double sleef_two54 = 1.80143985094819840000e+16; /* 2^54 */
static const double sleef_ln2_hi = 6.93147180369123816490e-01; /* 0x3fe62e42, 0xfee00000 */
static const double sleef_ln2_lo = 1.90821492927058770002e-10; /* 0x3dea39ef, 0x35793c76 */
static const double sleef_invln2 = 1.44269504088896338700e+00; /* 0x3ff71547, 0x652b82fe */

/* ============================================================================
 * sleef_exp - Exponential function (double precision)
 *
 * Returns e^x. Based on fdlibm __ieee754_exp.
 * ============================================================================ */

static inline double sleef_exp(double x) {
	static const double halF[2] = { 0.5, -0.5 };
	static const double o_threshold = 7.09782712893383973096e+02;
	static const double u_threshold = -7.45133219101941108420e+02;
	static const double ln2HI[2] = { 6.93147180369123816490e-01, -6.93147180369123816490e-01 };
	static const double ln2LO[2] = { 1.90821492927058770002e-10, -1.90821492927058770002e-10 };
	static const double P1 = 1.66666666666666019037e-01;
	static const double P2 = -2.77777777770155933842e-03;
	static const double P3 = 6.61375632143793436117e-05;
	static const double P4 = -1.65339022054652515390e-06;
	static const double P5 = 4.13813679705723846039e-08;

	double y, hi = 0.0, lo = 0.0, c, t;
	int32_t k = 0, xsb;
	uint32_t hx;

	SLEEF_GET_HIGH_WORD(hx, x);
	xsb = (hx >> 31) & 1;
	hx &= 0x7fffffff;

	/* Filter out non-finite argument */
	if (hx >= 0x40862E42) {
		if (hx >= 0x7ff00000) {
			uint32_t lx;
			SLEEF_GET_LOW_WORD(lx, x);
			if (((hx & 0xfffff) | lx) != 0)
				return x + x; /* NaN */
			else
				return (xsb == 0) ? x : 0.0; /* exp(+-inf)={inf,0} */
		}
		if (x > o_threshold)
			return sleef_huge * sleef_huge; /* overflow */
		if (x < u_threshold)
			return sleef_twom1000 * sleef_twom1000; /* underflow */
	}

	/* Argument reduction */
	if (hx > 0x3fd62e42) {
		if (hx < 0x3FF0A2B2) {
			hi = x - ln2HI[xsb];
			lo = ln2LO[xsb];
			k = 1 - xsb - xsb;
		} else {
			k = (int32_t)(sleef_invln2 * x + halF[xsb]);
			t = k;
			hi = x - t * ln2HI[0];
			lo = t * ln2LO[0];
		}
		x = hi - lo;
	} else if (hx < 0x3e300000) {
		if (sleef_huge + x > 1.0)
			return 1.0 + x;
	} else {
		k = 0;
	}

	/* x is now in primary range */
	t = x * x;
	c = x - t * (P1 + t * (P2 + t * (P3 + t * (P4 + t * P5))));
	if (k == 0)
		return 1.0 - ((x * c) / (c - 2.0) - x);
	else
		y = 1.0 - ((lo - (x * c) / (2.0 - c)) - hi);

	if (k >= -1021) {
		uint32_t hy;
		SLEEF_GET_HIGH_WORD(hy, y);
		SLEEF_SET_HIGH_WORD(y, hy + (k << 20));
		return y;
	} else {
		uint32_t hy;
		SLEEF_GET_HIGH_WORD(hy, y);
		SLEEF_SET_HIGH_WORD(y, hy + ((k + 1000) << 20));
		return y * sleef_twom1000;
	}
}

/* ============================================================================
 * sleef_log - Natural logarithm (double precision)
 *
 * Returns ln(x). Based on fdlibm __ieee754_log.
 * ============================================================================ */

static inline double sleef_log(double x) {
	static const double Lg1 = 6.666666666666735130e-01;
	static const double Lg2 = 3.999999999940941908e-01;
	static const double Lg3 = 2.857142874366239149e-01;
	static const double Lg4 = 2.222219843214978396e-01;
	static const double Lg5 = 1.818357216161805012e-01;
	static const double Lg6 = 1.531383769920937332e-01;
	static const double Lg7 = 1.479819860511658591e-01;
	static const double zero = 0.0;

	double hfsq, f, s, z, R, w, t1, t2, dk;
	int32_t k, hx, i, j;
	uint32_t lx;

	SLEEF_EXTRACT_WORDS(hx, lx, x);

	k = 0;
	if (hx < 0x00100000) {
		if (((hx & 0x7fffffff) | lx) == 0)
			return -sleef_two54 / zero; /* log(+-0)=-inf */
		if (hx < 0)
			return (x - x) / zero; /* log(-#) = NaN */
		k -= 54;
		x *= sleef_two54;
		SLEEF_GET_HIGH_WORD(hx, x);
	}
	if (hx >= 0x7ff00000)
		return x + x;
	k += (hx >> 20) - 1023;
	hx &= 0x000fffff;
	i = (hx + 0x95f64) & 0x100000;
	SLEEF_SET_HIGH_WORD(x, hx | (i ^ 0x3ff00000));
	k += (i >> 20);
	f = x - 1.0;
	if ((0x000fffff & (2 + hx)) < 3) {
		if (f == zero) {
			if (k == 0)
				return zero;
			else {
				dk = (double)k;
				return dk * sleef_ln2_hi + dk * sleef_ln2_lo;
			}
		}
		R = f * f * (0.5 - 0.33333333333333333 * f);
		if (k == 0)
			return f - R;
		else {
			dk = (double)k;
			return dk * sleef_ln2_hi - ((R - dk * sleef_ln2_lo) - f);
		}
	}
	s = f / (2.0 + f);
	dk = (double)k;
	z = s * s;
	i = hx - 0x6147a;
	w = z * z;
	j = 0x6b851 - hx;
	t1 = w * (Lg2 + w * (Lg4 + w * Lg6));
	t2 = z * (Lg1 + w * (Lg3 + w * (Lg5 + w * Lg7)));
	i |= j;
	R = t2 + t1;
	if (i > 0) {
		hfsq = 0.5 * f * f;
		if (k == 0)
			return f - (hfsq - s * (hfsq + R));
		else
			return dk * sleef_ln2_hi - ((hfsq - (s * (hfsq + R) + dk * sleef_ln2_lo)) - f);
	} else {
		if (k == 0)
			return f - s * (f - R);
		else
			return dk * sleef_ln2_hi - ((s * (f - R) - dk * sleef_ln2_lo) - f);
	}
}

/* ============================================================================
 * sleef_expf - Exponential function (single precision)
 * ============================================================================ */

static inline float sleef_expf(float x) {
	static const float halF[2] = { 0.5f, -0.5f };
	static const float ln2HI[2] = { 6.9314575195e-01f, -6.9314575195e-01f };
	static const float ln2LO[2] = { 1.4286067653e-06f, -1.4286067653e-06f };
	static const float invln2 = 1.4426950216e+00f;
	static const float P1 = 1.6666667163e-01f;
	static const float P2 = -2.7777778450e-03f;
	static const float P3 = 6.6137559770e-05f;
	static const float P4 = -1.6533901999e-06f;
	static const float P5 = 4.1381369442e-08f;
	static const float hugef = 1.0e+30f;
	static const float twom100 = 7.8886090522e-31f;

	float y, hi = 0.0f, lo = 0.0f, c, t;
	int32_t k = 0, xsb;
	uint32_t hx;

	SLEEF_GET_FLOAT_WORD(hx, x);
	xsb = (hx >> 31) & 1;
	hx &= 0x7fffffff;

	if (hx >= 0x42b17218) {
		if (hx > 0x7f800000)
			return x + x; /* NaN */
		if (hx == 0x7f800000)
			return (xsb == 0) ? x : 0.0f;
		if (x > 88.7228394f)
			return hugef * hugef; /* overflow */
		if (x < -103.9720840f)
			return twom100 * twom100; /* underflow */
	}

	if (hx > 0x3eb17218) {
		if (hx < 0x3F851592) {
			hi = x - ln2HI[xsb];
			lo = ln2LO[xsb];
			k = 1 - xsb - xsb;
		} else {
			k = (int32_t)(invln2 * x + halF[xsb]);
			t = (float)k;
			hi = x - t * ln2HI[0];
			lo = t * ln2LO[0];
		}
		x = hi - lo;
	} else if (hx < 0x31800000) {
		if (hugef + x > 1.0f)
			return 1.0f + x;
	} else {
		k = 0;
	}

	t = x * x;
	c = x - t * (P1 + t * (P2 + t * (P3 + t * (P4 + t * P5))));
	if (k == 0)
		return 1.0f - ((x * c) / (c - 2.0f) - x);
	else
		y = 1.0f - ((lo - (x * c) / (2.0f - c)) - hi);

	if (k >= -125) {
		uint32_t hy;
		SLEEF_GET_FLOAT_WORD(hy, y);
		SLEEF_SET_FLOAT_WORD(y, hy + (k << 23));
		return y;
	} else {
		uint32_t hy;
		SLEEF_GET_FLOAT_WORD(hy, y);
		SLEEF_SET_FLOAT_WORD(y, hy + ((k + 100) << 23));
		return y * twom100;
	}
}

/* ============================================================================
 * sleef_logf - Natural logarithm (single precision)
 * ============================================================================ */

static inline float sleef_logf(float x) {
	static const float ln2_hi = 6.9313812256e-01f;
	static const float ln2_lo = 9.0580006145e-06f;
	static const float two25 = 3.355443200e+07f;
	static const float Lg1 = 6.6666668653e-01f;
	static const float Lg2 = 4.0000000596e-01f;
	static const float Lg3 = 2.8571429849e-01f;
	static const float Lg4 = 2.2222198546e-01f;
	static const float zero = 0.0f;

	float hfsq, f, s, z, R, w, t1, t2, dk;
	int32_t k, ix, i, j;

	SLEEF_GET_FLOAT_WORD(ix, x);

	k = 0;
	if (ix < 0x00800000) {
		if ((ix & 0x7fffffff) == 0)
			return -two25 / zero;
		if (ix < 0)
			return (x - x) / zero;
		k -= 25;
		x *= two25;
		SLEEF_GET_FLOAT_WORD(ix, x);
	}
	if (ix >= 0x7f800000)
		return x + x;
	k += (ix >> 23) - 127;
	ix &= 0x007fffff;
	i = (ix + (0x95f64 << 3)) & 0x800000;
	SLEEF_SET_FLOAT_WORD(x, ix | (i ^ 0x3f800000));
	k += (i >> 23);
	f = x - 1.0f;
	if ((0x007fffff & (15 + ix)) < 16) {
		if (f == zero) {
			if (k == 0)
				return zero;
			else {
				dk = (float)k;
				return dk * ln2_hi + dk * ln2_lo;
			}
		}
		R = f * f * (0.5f - 0.33333333333333333f * f);
		if (k == 0)
			return f - R;
		else {
			dk = (float)k;
			return dk * ln2_hi - ((R - dk * ln2_lo) - f);
		}
	}
	s = f / (2.0f + f);
	dk = (float)k;
	z = s * s;
	i = ix - (0x6147a << 3);
	w = z * z;
	j = (0x6b851 << 3) - ix;
	t1 = w * (Lg2 + w * Lg4);
	t2 = z * (Lg1 + w * Lg3);
	i |= j;
	R = t2 + t1;
	if (i > 0) {
		hfsq = 0.5f * f * f;
		if (k == 0)
			return f - (hfsq - s * (hfsq + R));
		else
			return dk * ln2_hi - ((hfsq - (s * (hfsq + R) + dk * ln2_lo)) - f);
	} else {
		if (k == 0)
			return f - s * (f - R);
		else
			return dk * ln2_hi - ((s * (f - R) - dk * ln2_lo) - f);
	}
}

/* ============================================================================
 * Hyperbolic functions - implemented using exp and log
 * ============================================================================ */

/* sinh(x) = (exp(x) - exp(-x)) / 2 */
static inline double sleef_sinh(double x) {
	double t, h;
	int32_t ix, jx;
	SLEEF_GET_HIGH_WORD(jx, x);
	ix = jx & 0x7fffffff;

	/* Handle special cases */
	if (ix >= 0x7ff00000)
		return x + x; /* sinh(NaN) = NaN, sinh(+-Inf) = +-Inf */

	h = 0.5;
	if (jx < 0)
		h = -h;

	/* |x| < 22: direct formula to avoid cancellation */
	if (ix < 0x40360000) { /* |x| < 22 */
		if (ix < 0x3e300000) /* |x| < 2^-28 */
			return x; /* sinh(tiny) = tiny with inexact */
		t = sleef_exp(x < 0 ? -x : x);
		if (ix < 0x3ff00000) /* |x| < 1 */
			return h * (2.0 * t - 2.0 / t);
		return h * (t - 1.0 / t);
	}

	/* |x| >= 22, use exp(x)/2 */
	if (ix < 0x40862E42) /* |x| < 709.78 */
		return h * sleef_exp(x < 0 ? -x : x);

	/* |x| >= 709.78, need to avoid overflow */
	t = sleef_exp(0.5 * (x < 0 ? -x : x));
	return h * t * t;
}

/* cosh(x) = (exp(x) + exp(-x)) / 2 */
static inline double sleef_cosh(double x) {
	double t;
	int32_t ix;
	SLEEF_GET_HIGH_WORD(ix, x);
	ix &= 0x7fffffff;

	/* Handle special cases */
	if (ix >= 0x7ff00000)
		return x * x; /* cosh(NaN) = NaN, cosh(+-Inf) = +Inf */

	/* |x| < 0.5*ln2 */
	if (ix < 0x3fd62e43) {
		t = sleef_exp(x < 0 ? -x : x);
		return 1.0 + (t - 1.0) * (t - 1.0) / (2.0 * t);
	}

	/* |x| < 22 */
	if (ix < 0x40360000) {
		t = sleef_exp(x < 0 ? -x : x);
		return 0.5 * t + 0.5 / t;
	}

	/* |x| < 709.78 */
	if (ix < 0x40862E42)
		return 0.5 * sleef_exp(x < 0 ? -x : x);

	/* |x| >= 709.78, avoid overflow */
	t = sleef_exp(0.5 * (x < 0 ? -x : x));
	return 0.5 * t * t;
}

/* tanh(x) = (exp(2x) - 1) / (exp(2x) + 1) */
static inline double sleef_tanh(double x) {
	double t, z;
	int32_t jx, ix;
	SLEEF_GET_HIGH_WORD(jx, x);
	ix = jx & 0x7fffffff;

	/* Handle special cases */
	if (ix >= 0x7ff00000) {
		if (jx >= 0)
			return 1.0 / x + 1.0; /* tanh(+Inf) = 1, tanh(NaN) = NaN */
		else
			return 1.0 / x - 1.0; /* tanh(-Inf) = -1 */
	}

	/* |x| < 22 */
	if (ix < 0x40360000) {
		if (ix < 0x3c800000) /* |x| < 2^-55 */
			return x;
		if (ix >= 0x3ff00000) { /* |x| >= 1 */
			t = sleef_exp(2.0 * (x < 0 ? -x : x));
			z = 1.0 - 2.0 / (t + 2.0);
		} else {
			t = sleef_exp(-2.0 * (x < 0 ? -x : x));
			z = -t / (t + 2.0);
		}
	} else { /* |x| >= 22 */
		z = 1.0;
	}
	return (jx >= 0) ? z : -z;
}

/* asinh(x) = sign(x) * log(|x| + sqrt(x*x + 1)) */
static inline double sleef_asinh(double x) {
	double t, w;
	int32_t hx, ix;
	SLEEF_GET_HIGH_WORD(hx, x);
	ix = hx & 0x7fffffff;

	if (ix >= 0x7ff00000)
		return x + x; /* asinh(NaN/Inf) = NaN/Inf */

	if (ix < 0x3e300000) { /* |x| < 2^-28 */
		if (sleef_huge + x > 1.0)
			return x;
	}

	if (ix > 0x41b00000) { /* |x| > 2^28 */
		w = sleef_log(x < 0 ? -x : x) + sleef_ln2_hi;
	} else if (ix > 0x40000000) { /* 2 < |x| < 2^28 */
		t = x < 0 ? -x : x;
		w = sleef_log(2.0 * t + 1.0 / (t + sleef_sqrt(t * t + 1.0)));
	} else { /* 2^-28 <= |x| <= 2 */
		t = x * x;
		w = sleef_log(1.0 + (x < 0 ? -x : x) + t / (1.0 + sleef_sqrt(1.0 + t)));
	}
	return (hx > 0) ? w : -w;
}

/* acosh(x) = log(x + sqrt(x*x - 1)) for x >= 1 */
static inline double sleef_acosh(double x) {
	double t;
	int32_t hx;
	uint32_t lx;
	SLEEF_EXTRACT_WORDS(hx, lx, x);

	if (hx < 0x3ff00000) { /* x < 1 */
		return (x - x) / (x - x); /* NaN */
	} else if (hx >= 0x41b00000) { /* x > 2^28 */
		if (hx >= 0x7ff00000)
			return x + x; /* acosh(Inf) = Inf, acosh(NaN) = NaN */
		return sleef_log(x) + sleef_ln2_hi;
	} else if (((hx - 0x3ff00000) | lx) == 0) {
		return 0.0; /* acosh(1) = 0 */
	} else if (hx > 0x40000000) { /* 2 < x < 2^28 */
		t = x * x;
		return sleef_log(2.0 * x - 1.0 / (x + sleef_sqrt(t - 1.0)));
	} else { /* 1 < x <= 2 */
		t = x - 1.0;
		return sleef_log(1.0 + t + sleef_sqrt(2.0 * t + t * t));
	}
}

/* atanh(x) = 0.5 * log((1 + x) / (1 - x)) for |x| < 1 */
static inline double sleef_atanh(double x) {
	double t;
	int32_t hx, ix;
	uint32_t lx;
	SLEEF_EXTRACT_WORDS(hx, lx, x);
	ix = hx & 0x7fffffff;

	if ((ix | ((lx | (uint32_t)(-lx)) >> 31)) > 0x3ff00000)
		return (x - x) / (x - x); /* |x| > 1: NaN */

	if (ix == 0x3ff00000)
		return x / 0.0; /* |x| == 1: +-Inf */

	if (ix < 0x3e300000 && (sleef_huge + x) > 0.0)
		return x; /* |x| < 2^-28 */

	SLEEF_SET_HIGH_WORD(x, ix);
	if (ix < 0x3fe00000) { /* |x| < 0.5 */
		t = x + x;
		t = 0.5 * sleef_log(1.0 + t + t * x / (1.0 - x));
	} else {
		t = 0.5 * sleef_log((1.0 + x) / (1.0 - x));
	}
	return (hx >= 0) ? t : -t;
}

/* Single precision hyperbolic functions */

static inline float sleef_sinhf(float x) {
	float t, h;
	int32_t ix, jx;
	SLEEF_GET_FLOAT_WORD(jx, x);
	ix = jx & 0x7fffffff;

	if (ix >= 0x7f800000)
		return x + x;

	h = 0.5f;
	if (jx < 0)
		h = -h;

	if (ix < 0x41b00000) {
		if (ix < 0x31800000)
			return x;
		t = sleef_expf(x < 0 ? -x : x);
		if (ix < 0x3f800000)
			return h * (2.0f * t - 2.0f / t);
		return h * (t - 1.0f / t);
	}

	if (ix < 0x42b17217)
		return h * sleef_expf(x < 0 ? -x : x);

	t = sleef_expf(0.5f * (x < 0 ? -x : x));
	return h * t * t;
}

static inline float sleef_coshf(float x) {
	float t;
	int32_t ix;
	SLEEF_GET_FLOAT_WORD(ix, x);
	ix &= 0x7fffffff;

	if (ix >= 0x7f800000)
		return x * x;

	if (ix < 0x3eb17218) {
		t = sleef_expf(x < 0 ? -x : x);
		return 1.0f + (t - 1.0f) * (t - 1.0f) / (2.0f * t);
	}

	if (ix < 0x41b00000) {
		t = sleef_expf(x < 0 ? -x : x);
		return 0.5f * t + 0.5f / t;
	}

	if (ix < 0x42b17217)
		return 0.5f * sleef_expf(x < 0 ? -x : x);

	t = sleef_expf(0.5f * (x < 0 ? -x : x));
	return 0.5f * t * t;
}

static inline float sleef_tanhf(float x) {
	float t, z;
	int32_t jx, ix;
	SLEEF_GET_FLOAT_WORD(jx, x);
	ix = jx & 0x7fffffff;

	if (ix >= 0x7f800000) {
		if (jx >= 0)
			return 1.0f / x + 1.0f;
		else
			return 1.0f / x - 1.0f;
	}

	if (ix < 0x41b00000) {
		if (ix < 0x24000000)
			return x;
		if (ix >= 0x3f800000) {
			t = sleef_expf(2.0f * (x < 0 ? -x : x));
			z = 1.0f - 2.0f / (t + 2.0f);
		} else {
			t = sleef_expf(-2.0f * (x < 0 ? -x : x));
			z = -t / (t + 2.0f);
		}
	} else {
		z = 1.0f;
	}
	return (jx >= 0) ? z : -z;
}

static inline float sleef_asinhf(float x) {
	float t, w;
	int32_t hx, ix;
	SLEEF_GET_FLOAT_WORD(hx, x);
	ix = hx & 0x7fffffff;

	if (ix >= 0x7f800000)
		return x + x;

	if (ix < 0x31800000) {
		if (1.0e30f + x > 1.0f)
			return x;
	}

	if (ix > 0x4d800000) {
		w = sleef_logf(x < 0 ? -x : x) + 0.693147180559945309417232121458176568f;
	} else if (ix > 0x40000000) {
		t = x < 0 ? -x : x;
		w = sleef_logf(2.0f * t + 1.0f / (t + sleef_sqrtf(t * t + 1.0f)));
	} else {
		t = x * x;
		w = sleef_logf(1.0f + (x < 0 ? -x : x) + t / (1.0f + sleef_sqrtf(1.0f + t)));
	}
	return (hx > 0) ? w : -w;
}

static inline float sleef_acoshf(float x) {
	float t;
	int32_t hx;
	SLEEF_GET_FLOAT_WORD(hx, x);

	if (hx < 0x3f800000) {
		return (x - x) / (x - x);
	} else if (hx >= 0x4d800000) {
		if (hx >= 0x7f800000)
			return x + x;
		return sleef_logf(x) + 0.693147180559945309417232121458176568f;
	} else if (hx == 0x3f800000) {
		return 0.0f;
	} else if (hx > 0x40000000) {
		t = x * x;
		return sleef_logf(2.0f * x - 1.0f / (x + sleef_sqrtf(t - 1.0f)));
	} else {
		t = x - 1.0f;
		return sleef_logf(1.0f + t + sleef_sqrtf(2.0f * t + t * t));
	}
}

static inline float sleef_atanhf(float x) {
	float t;
	int32_t hx, ix;
	SLEEF_GET_FLOAT_WORD(hx, x);
	ix = hx & 0x7fffffff;

	if (ix > 0x3f800000)
		return (x - x) / (x - x);

	if (ix == 0x3f800000)
		return x / 0.0f;

	if (ix < 0x31800000 && (1.0e30f + x) > 0.0f)
		return x;

	SLEEF_SET_FLOAT_WORD(x, ix);
	if (ix < 0x3f000000) {
		t = x + x;
		t = 0.5f * sleef_logf(1.0f + t + t * x / (1.0f - x));
	} else {
		t = 0.5f * sleef_logf((1.0f + x) / (1.0f - x));
	}
	return (hx >= 0) ? t : -t;
}

#ifdef __cplusplus
}
#endif

#endif /* SLEEF_DETERMINISTIC_H */
