# Cross-Platform Deterministic Physics via Fixed-Point Math

## Executive Summary

This document outlines a plan to make Godot's 3D physics system cross-platform deterministic using fixed-point math. The goal is to ensure identical physics simulation results across all platforms (Windows, Linux, macOS, consoles, mobile) regardless of CPU architecture, compiler, or floating-point hardware differences.

**Scope:**
- 3D physics only (ignoring 2D)
- Jolt Physics module integration
- C# bindings for deterministic math
- Rendering/view remains floating-point (non-deterministic is acceptable)

---

## Table of Contents

1. [Problem Statement](#1-problem-statement)
2. [Architecture Overview](#2-architecture-overview)
3. [Phase 1: Fixed-Point Math Library](#3-phase-1-fixed-point-math-library)
4. [Phase 2: Deterministic Physics Layer](#4-phase-2-deterministic-physics-layer)
5. [Phase 3: Jolt Physics Integration](#5-phase-3-jolt-physics-integration)
6. [Phase 4: C# Bindings](#6-phase-4-c-bindings)
7. [Phase 5: Conversion Layer](#7-phase-5-conversion-layer)
8. [Testing Strategy](#8-testing-strategy)
9. [Performance Considerations](#9-performance-considerations)
10. [Alternative Approaches](#10-alternative-approaches)
11. [File Structure](#11-file-structure)
12. [Implementation Timeline](#12-implementation-timeline)
13. [Open Questions](#13-open-questions)

---

## 1. Problem Statement

### Why Floating-Point is Non-Deterministic

Floating-point arithmetic can produce different results across:
- **CPU architectures**: x86 vs ARM vs RISC-V (different FPU implementations)
- **Compiler settings**: Optimization levels, fast-math flags
- **Platform differences**: 80-bit x87 vs 64-bit SSE vs 32-bit NEON
- **Instruction reordering**: `(a + b) + c` vs `a + (b + c)` can differ
- **Transcendental functions**: `sin()`, `cos()`, `sqrt()` implementations vary

### Impact on Networked Games

Non-determinism breaks:
- Lockstep multiplayer (all clients must compute identical state)
- Replay systems (recorded inputs must reproduce identical results)
- Anti-cheat verification (server can't verify client simulation)
- Save/load consistency (loaded game diverges from saved state)

### Fixed-Point Solution

Fixed-point math uses integers with an implicit decimal point, guaranteeing:
- Identical results on all platforms (integer math is deterministic)
- Reproducible simulations across runs
- Consistent behavior regardless of compiler/optimization

---

## 2. Architecture Overview

```
┌─────────────────────────────────────────────────────────────────────┐
│                           GAME LAYER (C#)                           │
├─────────────────────────────────────────────────────────────────────┤
│  User Scripts  │  Game Logic  │  AI  │  Networking (Lockstep)      │
└───────────┬────────────────────────────────────────┬────────────────┘
            │                                        │
            │ Fixed-Point API                        │ Float API (View)
            ▼                                        ▼
┌───────────────────────────────┐    ┌────────────────────────────────┐
│    DETERMINISTIC LAYER        │    │      RENDERING LAYER           │
│  (Fixed-Point Math - Q24.8    │───▶│   (Floating-Point)             │
│   or Q16.16)                  │    │                                │
├───────────────────────────────┤    ├────────────────────────────────┤
│ • FixedVector3                │    │ • Vector3 (unchanged)          │
│ • FixedQuaternion             │    │ • Transform3D (unchanged)      │
│ • FixedBasis                  │    │ • Camera3D, MeshInstance3D     │
│ • FixedTransform3D            │    │ • Shaders, Materials           │
│ • FixedAABB                   │    │ • Visual effects               │
└───────────┬───────────────────┘    └────────────────────────────────┘
            │
            │ Physics Interface
            ▼
┌─────────────────────────────────────────────────────────────────────┐
│                  DETERMINISTIC PHYSICS SERVER                        │
├─────────────────────────────────────────────────────────────────────┤
│  DeterministicPhysicsServer3D (new class)                           │
│  • Wraps or replaces JoltPhysicsServer3D                            │
│  • All internal math uses fixed-point                               │
│  • Deterministic collision detection                                │
│  • Deterministic constraint solving                                 │
└───────────┬─────────────────────────────────────────────────────────┘
            │
            │ Option A: Fixed-Point Jolt Fork
            │ Option B: Deterministic Float Wrapper
            ▼
┌─────────────────────────────────────────────────────────────────────┐
│                    JOLT PHYSICS ENGINE                               │
│  (Modified for determinism OR wrapped with conversion)              │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 3. Phase 1: Fixed-Point Math Library

### 3.1 Core Fixed-Point Type

**File:** `core/math/fixed_point.h`

```cpp
// Q16.16 format: 16 bits integer, 16 bits fraction
// Range: -32768.0 to 32767.99998 with precision of ~0.000015
// Alternative: Q24.8 for larger range, less precision

class Fixed64 {
private:
    int64_t raw_value;  // Internal representation

    static constexpr int FRACTION_BITS = 16;
    static constexpr int64_t ONE = 1LL << FRACTION_BITS;  // 65536
    static constexpr int64_t HALF = ONE >> 1;             // 32768

public:
    // Constructors
    Fixed64() : raw_value(0) {}
    explicit Fixed64(int32_t integer) : raw_value(int64_t(integer) << FRACTION_BITS) {}
    static Fixed64 from_raw(int64_t raw) { Fixed64 f; f.raw_value = raw; return f; }
    static Fixed64 from_float(float f) { return Fixed64(int64_t(f * ONE)); }  // Conversion only

    // Basic arithmetic (deterministic)
    Fixed64 operator+(Fixed64 other) const;
    Fixed64 operator-(Fixed64 other) const;
    Fixed64 operator*(Fixed64 other) const;  // Uses 128-bit intermediate
    Fixed64 operator/(Fixed64 other) const;  // Uses 128-bit intermediate

    // Comparison
    bool operator==(Fixed64 other) const { return raw_value == other.raw_value; }
    bool operator<(Fixed64 other) const { return raw_value < other.raw_value; }
    // ... other comparisons

    // Transcendental functions (lookup tables + interpolation)
    static Fixed64 sin(Fixed64 angle);   // Angle in radians
    static Fixed64 cos(Fixed64 angle);
    static Fixed64 sqrt(Fixed64 value);  // Newton-Raphson or lookup
    static Fixed64 atan2(Fixed64 y, Fixed64 x);

    // Conversion (for rendering only)
    float to_float() const { return float(raw_value) / float(ONE); }
    double to_double() const { return double(raw_value) / double(ONE); }
    int64_t get_raw() const { return raw_value; }
};
```

### 3.2 Fixed-Point Vector Types

**File:** `core/math/fixed_vector3.h`

```cpp
struct FixedVector3 {
    Fixed64 x, y, z;

    // Construction
    FixedVector3() = default;
    FixedVector3(Fixed64 x, Fixed64 y, Fixed64 z);
    static FixedVector3 from_vector3(const Vector3 &v);  // For initialization only

    // Vector operations (all deterministic)
    FixedVector3 operator+(const FixedVector3 &other) const;
    FixedVector3 operator-(const FixedVector3 &other) const;
    FixedVector3 operator*(Fixed64 scalar) const;
    FixedVector3 operator/(Fixed64 scalar) const;

    Fixed64 dot(const FixedVector3 &other) const;
    FixedVector3 cross(const FixedVector3 &other) const;

    Fixed64 length_squared() const;
    Fixed64 length() const;  // Uses fixed-point sqrt
    FixedVector3 normalized() const;

    // Conversion to rendering types
    Vector3 to_vector3() const;
};
```

### 3.3 Fixed-Point Rotation Types

**File:** `core/math/fixed_quaternion.h`

```cpp
struct FixedQuaternion {
    Fixed64 x, y, z, w;

    // Construction
    FixedQuaternion();  // Identity
    FixedQuaternion(Fixed64 x, Fixed64 y, Fixed64 z, Fixed64 w);
    static FixedQuaternion from_axis_angle(const FixedVector3 &axis, Fixed64 angle);
    static FixedQuaternion from_euler(const FixedVector3 &euler);

    // Operations
    FixedQuaternion operator*(const FixedQuaternion &other) const;
    FixedVector3 xform(const FixedVector3 &v) const;

    FixedQuaternion normalized() const;
    FixedQuaternion inverse() const;

    static FixedQuaternion slerp(const FixedQuaternion &a, const FixedQuaternion &b, Fixed64 t);

    // Conversion
    Quaternion to_quaternion() const;
};
```

**File:** `core/math/fixed_basis.h`

```cpp
struct FixedBasis {
    FixedVector3 rows[3];  // 3x3 rotation/scale matrix

    // Construction
    FixedBasis();  // Identity
    static FixedBasis from_quaternion(const FixedQuaternion &q);
    static FixedBasis from_euler(const FixedVector3 &euler);

    // Operations
    FixedBasis operator*(const FixedBasis &other) const;
    FixedVector3 xform(const FixedVector3 &v) const;
    FixedVector3 xform_inv(const FixedVector3 &v) const;

    FixedBasis transposed() const;
    FixedBasis inverse() const;
    FixedBasis orthonormalized() const;

    FixedQuaternion get_quaternion() const;

    // Conversion
    Basis to_basis() const;
};
```

### 3.4 Fixed-Point Transform

**File:** `core/math/fixed_transform_3d.h`

```cpp
struct FixedTransform3D {
    FixedBasis basis;
    FixedVector3 origin;

    // Construction
    FixedTransform3D();  // Identity
    FixedTransform3D(const FixedBasis &basis, const FixedVector3 &origin);

    // Operations
    FixedTransform3D operator*(const FixedTransform3D &other) const;
    FixedVector3 xform(const FixedVector3 &v) const;
    FixedVector3 xform_inv(const FixedVector3 &v) const;

    FixedTransform3D inverse() const;
    FixedTransform3D affine_inverse() const;

    // Interpolation
    static FixedTransform3D interpolate(const FixedTransform3D &a,
                                        const FixedTransform3D &b,
                                        Fixed64 t);

    // Conversion
    Transform3D to_transform_3d() const;
};
```

### 3.5 Transcendental Function Implementation

**File:** `core/math/fixed_math_funcs.cpp`

```cpp
// Lookup table approach for sin/cos
// Table size: 1024 entries for quarter circle (0 to PI/2)
// Memory: 8KB for sin table

static const int64_t SIN_TABLE[1024] = { /* precomputed values */ };

Fixed64 Fixed64::sin(Fixed64 angle) {
    // Normalize angle to [0, 2*PI)
    // Use symmetry to reduce to [0, PI/2]
    // Lookup + linear interpolation for sub-index precision
    // All operations are integer-based
}

Fixed64 Fixed64::sqrt(Fixed64 value) {
    // Newton-Raphson iteration (fixed number of iterations for determinism)
    // OR binary search approach
    // Both use only integer operations
}

Fixed64 Fixed64::atan2(Fixed64 y, Fixed64 x) {
    // CORDIC algorithm or lookup table
    // Deterministic implementation
}
```

### 3.6 Additional Types

**File:** `core/math/fixed_aabb.h`

```cpp
struct FixedAABB {
    FixedVector3 position;
    FixedVector3 size;

    bool intersects(const FixedAABB &other) const;
    bool contains_point(const FixedVector3 &point) const;
    FixedAABB merged_with(const FixedAABB &other) const;

    AABB to_aabb() const;
};
```

**File:** `core/math/fixed_plane.h`

```cpp
struct FixedPlane {
    FixedVector3 normal;
    Fixed64 d;

    Fixed64 distance_to(const FixedVector3 &point) const;
    bool is_point_over(const FixedVector3 &point) const;

    Plane to_plane() const;
};
```

---

## 4. Phase 2: Deterministic Physics Layer

### 4.1 Deterministic Physics Server Interface

**File:** `servers/deterministic_physics_server_3d.h`

```cpp
class DeterministicPhysicsServer3D : public PhysicsServer3D {
    // Extends PhysicsServer3D with fixed-point variants

public:
    // Fixed-point body state
    virtual void body_set_state_fixed(RID p_body, BodyState p_state,
                                       const FixedTransform3D &p_value) = 0;
    virtual FixedTransform3D body_get_state_fixed(RID p_body, BodyState p_state) const = 0;

    // Fixed-point velocities
    virtual void body_set_linear_velocity_fixed(RID p_body, const FixedVector3 &p_velocity) = 0;
    virtual FixedVector3 body_get_linear_velocity_fixed(RID p_body) const = 0;

    virtual void body_set_angular_velocity_fixed(RID p_body, const FixedVector3 &p_velocity) = 0;
    virtual FixedVector3 body_get_angular_velocity_fixed(RID p_body) const = 0;

    // Fixed-point forces
    virtual void body_apply_force_fixed(RID p_body, const FixedVector3 &p_force,
                                        const FixedVector3 &p_position) = 0;
    virtual void body_apply_impulse_fixed(RID p_body, const FixedVector3 &p_impulse,
                                          const FixedVector3 &p_position) = 0;

    // Deterministic step (fixed timestep required)
    virtual void step_deterministic(Fixed64 p_step) = 0;

    // State hashing for verification
    virtual uint64_t get_state_hash() const = 0;

    // State serialization for networking
    virtual PackedByteArray serialize_state() const = 0;
    virtual void deserialize_state(const PackedByteArray &p_data) = 0;
};
```

### 4.2 Deterministic Direct Body State

**File:** `servers/deterministic_physics_direct_body_state_3d.h`

```cpp
class DeterministicPhysicsDirectBodyState3D : public PhysicsDirectBodyState3D {
public:
    // Fixed-point getters
    virtual FixedTransform3D get_transform_fixed() const = 0;
    virtual FixedVector3 get_linear_velocity_fixed() const = 0;
    virtual FixedVector3 get_angular_velocity_fixed() const = 0;
    virtual FixedVector3 get_center_of_mass_fixed() const = 0;

    // Fixed-point setters
    virtual void set_transform_fixed(const FixedTransform3D &p_transform) = 0;
    virtual void set_linear_velocity_fixed(const FixedVector3 &p_velocity) = 0;
    virtual void set_angular_velocity_fixed(const FixedVector3 &p_velocity) = 0;

    // Fixed-point force application
    virtual void apply_force_fixed(const FixedVector3 &p_force,
                                   const FixedVector3 &p_position) = 0;
    virtual void apply_impulse_fixed(const FixedVector3 &p_impulse,
                                     const FixedVector3 &p_position) = 0;
};
```

### 4.3 Deterministic Collision Queries

**File:** `servers/deterministic_physics_direct_space_state_3d.h`

```cpp
class DeterministicPhysicsDirectSpaceState3D : public PhysicsDirectSpaceState3D {
public:
    // Fixed-point ray casting
    struct FixedRayResult {
        FixedVector3 position;
        FixedVector3 normal;
        Fixed64 distance;
        RID collider;
        ObjectID collider_id;
        int shape;
    };

    virtual bool intersect_ray_fixed(const FixedVector3 &p_from,
                                     const FixedVector3 &p_to,
                                     FixedRayResult &r_result,
                                     const HashSet<RID> &p_exclude = {},
                                     uint32_t p_collision_mask = UINT32_MAX) = 0;

    // Fixed-point shape casting
    virtual bool cast_motion_fixed(const FixedTransform3D &p_transform,
                                   const FixedVector3 &p_motion,
                                   RID p_shape,
                                   Fixed64 &r_closest_safe,
                                   Fixed64 &r_closest_unsafe) = 0;
};
```

---

## 5. Phase 3: Jolt Physics Integration

### 5.1 Challenge: Jolt Uses Floating-Point Internally

Jolt Physics is written entirely in floating-point. There are three integration approaches:

### Option A: Fixed-Point Wrapper (Recommended for Initial Implementation)

**Pros:**
- No Jolt modifications required
- Can use latest Jolt updates
- Simpler implementation

**Cons:**
- Conversion overhead at boundaries
- Potential precision loss during conversion
- Internal Jolt calculations remain float (not truly deterministic)

**Implementation:**

**File:** `modules/jolt_physics/jolt_deterministic_wrapper.h`

```cpp
class JoltDeterministicWrapper {
    // Convert fixed-point inputs to float for Jolt
    // Run Jolt simulation
    // Convert float outputs back to fixed-point
    // Key: Use IEEE 754 strict mode and disable optimizations

    static JPH::Vec3 to_jolt(const FixedVector3 &v) {
        return JPH::Vec3(v.x.to_float(), v.y.to_float(), v.z.to_float());
    }

    static FixedVector3 from_jolt(const JPH::Vec3 &v) {
        return FixedVector3(
            Fixed64::from_float(v.GetX()),
            Fixed64::from_float(v.GetY()),
            Fixed64::from_float(v.GetZ())
        );
    }
};
```

**Build Configuration for Deterministic Floats:**

**File:** `modules/jolt_physics/SCsub` (modifications)

```python
# Force strict IEEE 754 compliance
if env["platform"] == "windows":
    env_jolt.Append(CXXFLAGS=["/fp:strict"])
elif env["platform"] in ["linuxbsd", "macos"]:
    env_jolt.Append(CXXFLAGS=["-ffp-contract=off", "-fno-fast-math"])

# Disable SIMD for maximum portability (optional, significant perf hit)
# env_jolt.Append(CPPDEFINES=["JPH_USE_SSE4_1=0", "JPH_USE_SSE4_2=0", "JPH_USE_AVX=0", "JPH_USE_AVX2=0"])
```

### Option B: Soft-Float Jolt Fork

**Pros:**
- True determinism
- No conversion overhead

**Cons:**
- Major effort (thousands of files)
- Must maintain fork
- Significant performance impact (10-50x slower)

**Not recommended** unless Option A proves insufficient.

### Option C: Custom Deterministic Physics Engine

**Pros:**
- Full control
- Purpose-built for determinism

**Cons:**
- Massive development effort
- Years of work to match Jolt feature set

**Not recommended** for this project.

### 5.2 Recommended Implementation (Option A Enhanced)

**File:** `modules/jolt_physics/jolt_deterministic_physics_server_3d.h`

```cpp
class JoltDeterministicPhysicsServer3D : public DeterministicPhysicsServer3D {
    JoltPhysicsServer3D *jolt_server;  // Wrapped Jolt server

    // Fixed-point state storage (canonical state)
    HashMap<RID, FixedTransform3D> body_transforms_fixed;
    HashMap<RID, FixedVector3> body_linear_velocities_fixed;
    HashMap<RID, FixedVector3> body_angular_velocities_fixed;

public:
    void step_deterministic(Fixed64 p_step) override {
        // 1. Push fixed-point state to Jolt (convert to float)
        sync_state_to_jolt();

        // 2. Run Jolt step (deterministic float mode)
        jolt_server->step(p_step.to_float());

        // 3. Pull Jolt state back to fixed-point
        // CRITICAL: Round consistently to nearest fixed-point value
        sync_state_from_jolt();

        // 4. The fixed-point state is now the canonical truth
    }

private:
    void sync_state_to_jolt();
    void sync_state_from_jolt();

    // Deterministic rounding (always round toward negative infinity, or nearest)
    Fixed64 deterministic_round(float value) {
        // Use consistent rounding mode across all platforms
        return Fixed64::from_raw(int64_t(std::floor(value * Fixed64::ONE + 0.5)));
    }
};
```

### 5.3 Jolt Determinism Settings

Jolt has built-in determinism support that should be enabled:

**File:** `modules/jolt_physics/jolt_project_settings.cpp`

```cpp
// Add new project settings for determinism
GLOBAL_DEF("physics/jolt_3d/deterministic_mode", false);
GLOBAL_DEF("physics/jolt_3d/deterministic_simulation_velocity_steps", 10);
GLOBAL_DEF("physics/jolt_3d/deterministic_simulation_position_steps", 4);

// In JoltSpace3D initialization:
if (deterministic_mode) {
    // Disable multithreading (required for determinism)
    physics_system->SetPhysicsSettings(JPH::PhysicsSettings{
        .mNumVelocitySteps = 10,
        .mNumPositionSteps = 4,
        .mDeterministicSimulation = true  // If Jolt supports this flag
    });
}
```

---

## 6. Phase 4: C# Bindings

### 6.1 Fixed-Point Types for C#

**File:** `modules/mono/glue/GodotSharp/GodotSharp/Core/Fixed64.cs`

```csharp
using System;
using System.Runtime.InteropServices;

[StructLayout(LayoutKind.Sequential)]
public readonly struct Fixed64 : IEquatable<Fixed64>, IComparable<Fixed64>
{
    private readonly long _rawValue;

    public const int FractionBits = 16;
    public static readonly long One = 1L << FractionBits;

    // Constants
    public static readonly Fixed64 Zero = default;
    public static readonly Fixed64 OneValue = new Fixed64(One);
    public static readonly Fixed64 Pi = FromRaw(205887L);  // PI * One
    public static readonly Fixed64 TwoPi = FromRaw(411775L);

    private Fixed64(long rawValue) => _rawValue = rawValue;

    public static Fixed64 FromRaw(long raw) => new Fixed64(raw);
    public static Fixed64 FromInt(int value) => new Fixed64((long)value << FractionBits);

    // For initialization only - do not use in simulation!
    public static Fixed64 FromFloat(float value) => new Fixed64((long)(value * One));
    public float ToFloat() => (float)_rawValue / One;

    // Arithmetic operators
    public static Fixed64 operator +(Fixed64 a, Fixed64 b) => new Fixed64(a._rawValue + b._rawValue);
    public static Fixed64 operator -(Fixed64 a, Fixed64 b) => new Fixed64(a._rawValue - b._rawValue);
    public static Fixed64 operator -(Fixed64 a) => new Fixed64(-a._rawValue);

    public static Fixed64 operator *(Fixed64 a, Fixed64 b)
    {
        // Use 128-bit multiplication to avoid overflow
        long result = (long)(((Int128)a._rawValue * b._rawValue) >> FractionBits);
        return new Fixed64(result);
    }

    public static Fixed64 operator /(Fixed64 a, Fixed64 b)
    {
        // Use 128-bit division
        long result = (long)(((Int128)a._rawValue << FractionBits) / b._rawValue);
        return new Fixed64(result);
    }

    // Comparison
    public bool Equals(Fixed64 other) => _rawValue == other._rawValue;
    public int CompareTo(Fixed64 other) => _rawValue.CompareTo(other._rawValue);
    public static bool operator ==(Fixed64 a, Fixed64 b) => a._rawValue == b._rawValue;
    public static bool operator !=(Fixed64 a, Fixed64 b) => a._rawValue != b._rawValue;
    public static bool operator <(Fixed64 a, Fixed64 b) => a._rawValue < b._rawValue;
    public static bool operator >(Fixed64 a, Fixed64 b) => a._rawValue > b._rawValue;

    // Math functions (lookup tables or CORDIC)
    public static Fixed64 Sqrt(Fixed64 value) { /* implementation */ }
    public static Fixed64 Sin(Fixed64 angle) { /* lookup table */ }
    public static Fixed64 Cos(Fixed64 angle) { /* lookup table */ }
    public static Fixed64 Atan2(Fixed64 y, Fixed64 x) { /* CORDIC */ }

    public override int GetHashCode() => _rawValue.GetHashCode();
    public override bool Equals(object obj) => obj is Fixed64 f && Equals(f);
    public override string ToString() => ToFloat().ToString();
}
```

### 6.2 Fixed-Point Vector3 for C#

**File:** `modules/mono/glue/GodotSharp/GodotSharp/Core/FixedVector3.cs`

```csharp
using System;
using System.Runtime.InteropServices;

[StructLayout(LayoutKind.Sequential)]
public struct FixedVector3 : IEquatable<FixedVector3>
{
    public Fixed64 X;
    public Fixed64 Y;
    public Fixed64 Z;

    public FixedVector3(Fixed64 x, Fixed64 y, Fixed64 z)
    {
        X = x;
        Y = y;
        Z = z;
    }

    // Static constructors
    public static readonly FixedVector3 Zero = default;
    public static readonly FixedVector3 One = new(Fixed64.OneValue, Fixed64.OneValue, Fixed64.OneValue);
    public static readonly FixedVector3 Up = new(Fixed64.Zero, Fixed64.OneValue, Fixed64.Zero);
    public static readonly FixedVector3 Forward = new(Fixed64.Zero, Fixed64.Zero, -Fixed64.OneValue);

    // For initialization only
    public static FixedVector3 FromVector3(Vector3 v) => new(
        Fixed64.FromFloat(v.X),
        Fixed64.FromFloat(v.Y),
        Fixed64.FromFloat(v.Z)
    );

    public Vector3 ToVector3() => new(X.ToFloat(), Y.ToFloat(), Z.ToFloat());

    // Arithmetic
    public static FixedVector3 operator +(FixedVector3 a, FixedVector3 b) =>
        new(a.X + b.X, a.Y + b.Y, a.Z + b.Z);
    public static FixedVector3 operator -(FixedVector3 a, FixedVector3 b) =>
        new(a.X - b.X, a.Y - b.Y, a.Z - b.Z);
    public static FixedVector3 operator *(FixedVector3 v, Fixed64 s) =>
        new(v.X * s, v.Y * s, v.Z * s);
    public static FixedVector3 operator /(FixedVector3 v, Fixed64 s) =>
        new(v.X / s, v.Y / s, v.Z / s);

    // Vector operations
    public Fixed64 Dot(FixedVector3 other) => X * other.X + Y * other.Y + Z * other.Z;

    public FixedVector3 Cross(FixedVector3 other) => new(
        Y * other.Z - Z * other.Y,
        Z * other.X - X * other.Z,
        X * other.Y - Y * other.X
    );

    public Fixed64 LengthSquared() => X * X + Y * Y + Z * Z;
    public Fixed64 Length() => Fixed64.Sqrt(LengthSquared());

    public FixedVector3 Normalized()
    {
        Fixed64 len = Length();
        if (len == Fixed64.Zero) return Zero;
        return this / len;
    }

    public static FixedVector3 Lerp(FixedVector3 a, FixedVector3 b, Fixed64 t)
    {
        return a + (b - a) * t;
    }

    // Equality
    public bool Equals(FixedVector3 other) => X == other.X && Y == other.Y && Z == other.Z;
    public override bool Equals(object obj) => obj is FixedVector3 v && Equals(v);
    public override int GetHashCode() => HashCode.Combine(X, Y, Z);
    public static bool operator ==(FixedVector3 a, FixedVector3 b) => a.Equals(b);
    public static bool operator !=(FixedVector3 a, FixedVector3 b) => !a.Equals(b);
}
```

### 6.3 Additional C# Types

Similar implementations needed for:
- `FixedQuaternion.cs`
- `FixedBasis.cs`
- `FixedTransform3D.cs`
- `FixedAABB.cs`
- `FixedPlane.cs`
- `FixedMath.cs` (static math functions)

### 6.4 C# Physics Bindings

**File:** `modules/mono/glue/GodotSharp/GodotSharp/Core/DeterministicPhysicsServer3D.cs`

```csharp
public partial class DeterministicPhysicsServer3D : PhysicsServer3D
{
    // Singleton accessor
    public new static DeterministicPhysicsServer3D Singleton { get; }

    // Fixed-point body methods
    public void BodySetTransformFixed(Rid body, FixedTransform3D transform);
    public FixedTransform3D BodyGetTransformFixed(Rid body);

    public void BodySetLinearVelocityFixed(Rid body, FixedVector3 velocity);
    public FixedVector3 BodyGetLinearVelocityFixed(Rid body);

    public void BodyApplyForceFixed(Rid body, FixedVector3 force, FixedVector3 position);
    public void BodyApplyImpulseFixed(Rid body, FixedVector3 impulse, FixedVector3 position);

    // Deterministic simulation step
    public void StepDeterministic(Fixed64 delta);

    // State verification
    public ulong GetStateHash();
    public byte[] SerializeState();
    public void DeserializeState(byte[] data);
}
```

### 6.5 Interop Structs

**File:** `modules/mono/interop_types.h` (additions)

```cpp
// Fixed-point type sizes for C# interop
#define GODOT_FIXED64_SIZE 8
#define GODOT_FIXED_VECTOR3_SIZE 24
#define GODOT_FIXED_QUATERNION_SIZE 32
#define GODOT_FIXED_BASIS_SIZE 72
#define GODOT_FIXED_TRANSFORM3D_SIZE 96
```

---

## 7. Phase 5: Conversion Layer

### 7.1 Rendering Synchronization

The rendering system needs float transforms, but simulation uses fixed-point. A conversion layer bridges them:

**File:** `scene/3d/physics_body_3d.cpp` (modifications)

```cpp
void RigidBody3D::_notification(int p_what) {
    switch (p_what) {
        case NOTIFICATION_INTERNAL_PHYSICS_PROCESS: {
            // Get fixed-point state from physics
            FixedTransform3D fixed_transform =
                DeterministicPhysicsServer3D::get_singleton()
                    ->body_get_transform_fixed(get_rid());

            // Convert to float for rendering
            Transform3D render_transform = fixed_transform.to_transform_3d();

            // Optionally: interpolate for visual smoothness
            if (interpolation_enabled) {
                render_transform = prev_render_transform.interpolate_with(
                    render_transform,
                    Engine::get_singleton()->get_physics_interpolation_fraction()
                );
            }

            set_global_transform(render_transform);
            break;
        }
    }
}
```

### 7.2 Input Handling

User input from float world must be converted to fixed-point for simulation:

**File:** `modules/mono/glue/GodotSharp/GodotSharp/Core/InputHelpers.cs`

```csharp
public static class DeterministicInput
{
    // Convert mouse position to fixed-point world coordinates
    public static FixedVector3 GetWorldPositionFixed(Camera3D camera, Vector2 screenPos)
    {
        Vector3 floatPos = camera.ProjectPosition(screenPos, 1.0f);
        return FixedVector3.FromVector3(floatPos);
    }

    // Quantize input direction for network sync
    public static FixedVector3 QuantizeDirection(Vector3 direction)
    {
        // Ensure all clients compute identical direction vectors
        return FixedVector3.FromVector3(direction.Normalized()).Normalized();
    }
}
```

---

## 8. Testing Strategy

### 8.1 Determinism Verification Tests

**File:** `tests/core/math/test_fixed_point.h`

```cpp
TEST_CASE("[Fixed-Point] Cross-compilation determinism") {
    // These tests should produce identical results when:
    // - Compiled with different compilers (MSVC, GCC, Clang)
    // - Run on different platforms (Windows, Linux, macOS)
    // - Run on different architectures (x86, ARM)

    Fixed64 a = Fixed64::from_raw(123456789);
    Fixed64 b = Fixed64::from_raw(987654321);

    // Addition
    CHECK(a + b == Fixed64::from_raw(1111111110));

    // Multiplication
    CHECK(a * b == Fixed64::from_raw(/* precomputed */));

    // Trigonometry
    Fixed64 angle = Fixed64::pi() / Fixed64::from_int(4);  // 45 degrees
    CHECK(Fixed64::sin(angle) == Fixed64::from_raw(46341));  // ~0.707
    CHECK(Fixed64::cos(angle) == Fixed64::from_raw(46341));

    // Square root
    CHECK(Fixed64::sqrt(Fixed64::from_int(2)) == Fixed64::from_raw(92682));  // ~1.414
}

TEST_CASE("[Fixed-Point] Physics simulation determinism") {
    // Run identical simulation 1000 times
    // All runs must produce identical final state hash

    for (int run = 0; run < 1000; run++) {
        // Reset simulation
        // Run 1000 physics steps
        // Record state hash
        // Compare to expected hash
    }
}
```

### 8.2 Network Determinism Test

**File:** `tests/servers/test_deterministic_physics.h`

```cpp
TEST_CASE("[Deterministic Physics] Lockstep simulation") {
    // Create two independent physics simulations
    // Apply identical inputs
    // Run for N steps
    // Verify state hashes match

    DeterministicPhysicsServer3D server_a, server_b;

    // Setup identical scenes
    // ...

    // Simulate
    for (int step = 0; step < 10000; step++) {
        server_a.step_deterministic(Fixed64::from_raw(1092));  // 1/60 sec
        server_b.step_deterministic(Fixed64::from_raw(1092));

        CHECK(server_a.get_state_hash() == server_b.get_state_hash());
    }
}
```

### 8.3 C# Test Suite

```csharp
[TestFixture]
public class FixedPointTests
{
    [Test]
    public void TestArithmetic()
    {
        var a = Fixed64.FromRaw(123456789);
        var b = Fixed64.FromRaw(987654321);

        Assert.AreEqual(Fixed64.FromRaw(1111111110), a + b);
    }

    [Test]
    public void TestVectorOperations()
    {
        var v1 = new FixedVector3(Fixed64.OneValue, Fixed64.Zero, Fixed64.Zero);
        var v2 = new FixedVector3(Fixed64.Zero, Fixed64.OneValue, Fixed64.Zero);

        var cross = v1.Cross(v2);
        Assert.AreEqual(FixedVector3.Forward * -Fixed64.OneValue, cross);
    }
}
```

---

## 9. Performance Considerations

### 9.1 Expected Performance Impact

| Operation | Float Time | Fixed Time | Overhead |
|-----------|-----------|------------|----------|
| Addition | 1x | 1x | 0% |
| Multiplication | 1x | 1.5-2x | 50-100% |
| Division | 1x | 2-3x | 100-200% |
| Sin/Cos | 1x | 0.5-1x | -50-0% (LUT faster) |
| Sqrt | 1x | 2-4x | 100-300% |
| Overall Physics | 1x | 1.5-2x | 50-100% |

### 9.2 Optimization Strategies

1. **SIMD for Fixed-Point** (where applicable)
   - Pack multiple Fixed64 operations into vector instructions
   - Requires careful implementation to maintain determinism

2. **Lookup Table Optimization**
   - Precompute sin/cos tables with fine granularity
   - Trade memory for speed

3. **Lazy Conversion**
   - Only convert to float when needed for rendering
   - Cache converted values per frame

4. **Reduced Precision Option**
   - Offer Q24.8 mode for less precision but faster computation
   - Suitable for games that don't need 0.00001 precision

### 9.3 Memory Impact

| Type | Float Size | Fixed Size | Increase |
|------|-----------|------------|----------|
| Vector3 | 12 bytes | 24 bytes | 100% |
| Quaternion | 16 bytes | 32 bytes | 100% |
| Transform3D | 48 bytes | 96 bytes | 100% |
| Per Body State | ~200 bytes | ~400 bytes | 100% |

---

## 10. Alternative Approaches

### 10.1 IEEE 754 Strict Mode (Partial Solution)

Instead of fixed-point, enforce strict IEEE 754 compliance:

**Pros:**
- No code rewrite
- Near-native performance

**Cons:**
- Not truly deterministic across all platforms
- Transcendental functions still vary
- ARM FPUs behave differently than x86

**Verdict:** Can be combined with fixed-point for Jolt wrapper (Option A in Phase 3).

### 10.2 Soft-Float Library

Use a software floating-point library (e.g., Berkeley SoftFloat):

**Pros:**
- Drop-in replacement for float operations
- Deterministic

**Cons:**
- 10-50x slower than hardware float
- Complex integration

**Verdict:** Not recommended for performance-critical physics.

### 10.3 Existing Fixed-Point Libraries

Consider using established libraries:

- **libfixmath** (C): Mature, well-tested
- **fpm** (C++): Modern, header-only
- **FixedPointy** (C#): .NET compatible

**Verdict:** Could accelerate implementation. Recommend evaluating **fpm** for C++ side.

---

## 11. File Structure

```
godot/
├── core/
│   └── math/
│       ├── fixed_point.h           # Core Fixed64 type
│       ├── fixed_point.cpp
│       ├── fixed_vector3.h         # FixedVector3
│       ├── fixed_vector3.cpp
│       ├── fixed_quaternion.h      # FixedQuaternion
│       ├── fixed_quaternion.cpp
│       ├── fixed_basis.h           # FixedBasis
│       ├── fixed_basis.cpp
│       ├── fixed_transform_3d.h    # FixedTransform3D
│       ├── fixed_transform_3d.cpp
│       ├── fixed_aabb.h            # FixedAABB
│       ├── fixed_plane.h           # FixedPlane
│       └── fixed_math_funcs.h      # Trig lookup tables, etc.
│
├── servers/
│   ├── deterministic_physics_server_3d.h
│   ├── deterministic_physics_server_3d.cpp
│   ├── deterministic_physics_direct_body_state_3d.h
│   └── deterministic_physics_direct_space_state_3d.h
│
├── modules/
│   ├── jolt_physics/
│   │   ├── jolt_deterministic_physics_server_3d.h
│   │   ├── jolt_deterministic_physics_server_3d.cpp
│   │   ├── jolt_deterministic_wrapper.h
│   │   └── jolt_deterministic_wrapper.cpp
│   │
│   └── mono/
│       └── glue/
│           └── GodotSharp/
│               └── GodotSharp/
│                   └── Core/
│                       ├── Fixed64.cs
│                       ├── FixedVector3.cs
│                       ├── FixedQuaternion.cs
│                       ├── FixedBasis.cs
│                       ├── FixedTransform3D.cs
│                       ├── FixedMath.cs
│                       └── DeterministicPhysicsServer3D.cs
│
├── tests/
│   ├── core/
│   │   └── math/
│   │       └── test_fixed_point.h
│   └── servers/
│       └── test_deterministic_physics.h
│
└── specs/
    └── deterministic-fixed-point-physics.md  # This document
```

---

## 12. Implementation Timeline

### Phase 1: Fixed-Point Math Library (4-6 weeks)
- Week 1-2: Core Fixed64 type with basic arithmetic
- Week 3: FixedVector3, FixedQuaternion implementation
- Week 4: FixedBasis, FixedTransform3D implementation
- Week 5-6: Transcendental functions (sin, cos, sqrt, atan2)

### Phase 2: Deterministic Physics Interface (2-3 weeks)
- Week 7: DeterministicPhysicsServer3D interface design
- Week 8-9: Direct body state and space state interfaces

### Phase 3: Jolt Integration (4-6 weeks)
- Week 10-11: Jolt deterministic wrapper implementation
- Week 12-13: State synchronization and conversion
- Week 14-15: Testing and optimization

### Phase 4: C# Bindings (3-4 weeks)
- Week 16-17: C# fixed-point types
- Week 18-19: C# physics bindings and interop

### Phase 5: Integration and Testing (3-4 weeks)
- Week 20-21: Scene integration, conversion layer
- Week 22-23: Comprehensive testing, cross-platform verification

**Total Estimated Time: 16-23 weeks (4-6 months)**

---

## 13. Open Questions

1. **Fixed-Point Format Selection**
   - Q16.16 vs Q24.8 vs Q20.12?
   - Trade-off: range vs precision
   - Recommendation: Start with Q16.16, allow runtime configuration

2. **Jolt Integration Depth**
   - Option A (wrapper) vs Option B (fork)?
   - Recommendation: Start with Option A, evaluate if sufficient

3. **Multithreading Strategy**
   - Single-threaded physics for guaranteed determinism?
   - Or deterministic multithreading with synchronized ordering?
   - Recommendation: Start single-threaded, add deterministic MT later

4. **Existing Library Usage**
   - Build from scratch vs use fpm/libfixmath?
   - Recommendation: Evaluate fpm, may save weeks of development

5. **Backward Compatibility**
   - How to handle existing projects?
   - Opt-in deterministic mode vs default?
   - Recommendation: Opt-in via project setting

6. **GDScript Support**
   - Should fixed-point types be exposed to GDScript?
   - Recommendation: Focus on C# first, GDScript later if needed

7. **Editor Integration**
   - How to edit fixed-point values in inspector?
   - Convert to/from float for display?
   - Recommendation: Display as float, store as fixed

---

## Appendix A: Fixed-Point Precision Analysis

### Q16.16 Format
- **Range:** -32,768.0 to +32,767.99998
- **Precision:** ~0.000015 (1/65536)
- **Suitable for:** Most game physics, positions within ~30km
- **Overflow risk:** Position values > 32km need handling

### Q24.8 Format
- **Range:** -8,388,608.0 to +8,388,607.996
- **Precision:** ~0.004 (1/256)
- **Suitable for:** Large worlds, less precision-critical applications
- **Overflow risk:** Much larger range, coarser precision

### Q20.12 Format (Compromise)
- **Range:** -524,288.0 to +524,287.9998
- **Precision:** ~0.00024 (1/4096)
- **Suitable for:** Balance between range and precision
- **Overflow risk:** Moderate

---

## Appendix B: Reference Implementations

### Games Using Fixed-Point Physics
- **Age of Empires II: Definitive Edition** - Lockstep RTS
- **Factorio** - Deterministic simulation
- **Starcraft II** - Lockstep multiplayer
- **Many fighting games** - Frame-perfect determinism

### Open Source References
- **Box2D Lite** - Simple 2D physics (can study determinism patterns)
- **Rapier** (Rust) - Has deterministic mode
- **libfixmath** - Mature fixed-point library

---

*Document Version: 1.0*
*Last Updated: 2025-11-21*
*Author: Claude (AI Assistant)*
