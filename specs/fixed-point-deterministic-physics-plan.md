# Fixed-Point Deterministic Physics Plan for Godot Engine

## Executive Summary

This document outlines a comprehensive plan to implement fixed-point math for cross-platform deterministic physics in Godot Engine, specifically targeting 3D games using the Jolt physics engine.

---

## Table of Contents

1. [Problem Statement](#1-problem-statement)
2. [Current Architecture Analysis](#2-current-architecture-analysis)
3. [Technical Challenges](#3-technical-challenges)
4. [Proposed Solution Architecture](#4-proposed-solution-architecture)
5. [Implementation Phases](#5-implementation-phases)
6. [Detailed Change List](#6-detailed-change-list)
7. [Risk Assessment](#7-risk-assessment)
8. [Alternative Approaches](#8-alternative-approaches)
9. [Testing Strategy](#9-testing-strategy)
10. [Performance Considerations](#10-performance-considerations)

---

## 1. Problem Statement

### Goal
Achieve **cross-platform deterministic physics simulation** for 3D games, ensuring identical simulation results across:
- Different CPU architectures (x86, ARM, RISC-V)
- Different operating systems (Windows, Linux, macOS, consoles, mobile)
- Different compiler versions and optimization levels
- Different hardware (Intel, AMD, Apple Silicon, etc.)

### Why Floating-Point is Non-Deterministic

IEEE 754 floating-point arithmetic is non-deterministic across platforms due to:

1. **Extended Precision Registers**: x87 FPU uses 80-bit internal precision vs 64-bit double
2. **Fused Multiply-Add (FMA)**: Different rounding behavior when fusing operations
3. **SIMD Variations**: SSE, AVX, NEON have subtle implementation differences
4. **Compiler Optimizations**: Operation reordering, constant folding variations
5. **Math Library Implementations**: `sin()`, `cos()`, `sqrt()` vary by platform
6. **Denormal Handling**: Flush-to-zero behavior differs across hardware

### Use Cases Requiring Determinism

- **Lockstep Multiplayer**: All clients must compute identical simulation states
- **Replay Systems**: Recorded inputs must reproduce exact game states
- **Anti-Cheat**: Server verification of client-computed states
- **Competitive Gaming**: Ensuring fairness across different hardware

---

## 2. Current Architecture Analysis

### 2.1 Godot Core Math (`core/math/`)

| File | Types Defined | Current Precision |
|------|---------------|-------------------|
| `math_defs.h` | `real_t` typedef | `float` (default) or `double` |
| `vector3.h` | `Vector3` | `real_t` (x, y, z) |
| `vector2.h` | `Vector2` | `real_t` (x, y) |
| `quaternion.h` | `Quaternion` | `real_t` (x, y, z, w) |
| `basis.h` | `Basis` | 3x `Vector3` rows |
| `transform_3d.h` | `Transform3D` | `Basis` + `Vector3` origin |
| `aabb.h` | `AABB` | 2x `Vector3` |
| `plane.h` | `Plane` | `Vector3` + `real_t` d |

**Key Definition** (`core/math/math_defs.h:143-147`):
```cpp
#ifdef REAL_T_IS_DOUBLE
typedef double real_t;
#else
typedef float real_t;
#endif
```

### 2.2 Jolt Physics Integration (`modules/jolt_physics/`)

**Architecture Overview:**
```
Godot API (PhysicsServer3D)
         |
         v
JoltPhysicsServer3D
         |
    [Type Conversion Layer]  <-- modules/jolt_physics/misc/jolt_type_conversions.h
         |
         v
Jolt Physics Library (thirdparty/jolt_physics/)
```

**Type Conversion Boundary** (`jolt_type_conversions.h`):
```cpp
// Godot -> Jolt
JPH::Vec3 to_jolt(const Vector3 &p_vec) {
    return JPH::Vec3((float)p_vec.x, (float)p_vec.y, (float)p_vec.z);
}

// Jolt -> Godot
Vector3 to_godot(const JPH::Vec3 &p_vec) {
    return Vector3((real_t)p_vec.GetX(), (real_t)p_vec.GetY(), (real_t)p_vec.GetZ());
}
```

### 2.3 Jolt Internal Math

Jolt Physics uses:
- `JPH::Vec3`, `JPH::Vec4` - SIMD-accelerated 4-component vectors (float)
- `JPH::DVec3` - Double precision positions for large worlds
- `JPH::Quat` - Quaternion rotations (float)
- `JPH::Mat44` - 4x4 transformation matrices (float)
- All internal calculations use IEEE 754 floating-point

**Critical Issue**: Jolt's physics solver, collision detection, and constraint solving are all floating-point based. The library was not designed for fixed-point operation.

---

## 3. Technical Challenges

### 3.1 Jolt Physics Dependency

**Challenge Level: CRITICAL**

Jolt Physics is a sophisticated, high-performance physics engine with ~150,000+ lines of code. It uses:
- SIMD intrinsics (SSE, AVX, NEON) throughout
- IEEE 754 floating-point assumptions
- Platform-specific optimizations

**Options:**
1. **Fork and Modify Jolt** - Extremely invasive, ~6-12 months of work
2. **Replace Jolt** - Use or create a fixed-point physics engine
3. **Hybrid Approach** - Fixed-point at boundaries, accept internal non-determinism
4. **Alternative Determinism** - Soft-float library wrapper

### 3.2 Mathematical Operations

Fixed-point requires reimplementing:
- Trigonometric functions (`sin`, `cos`, `tan`, `atan2`)
- Square root and inverse square root
- Division with proper precision
- Quaternion operations (normalization, slerp)
- Matrix operations (inverse, determinant)

### 3.3 Range and Precision Trade-offs

**32-bit Fixed-Point (Q16.16):**
- Range: -32768.0 to +32767.99998
- Precision: ~0.000015 (15 microunits)
- Suitable for: Local-space physics, small worlds

**64-bit Fixed-Point (Q32.32):**
- Range: -2,147,483,648 to +2,147,483,647.999999999
- Precision: ~0.00000000023
- Suitable for: Large worlds, high precision

### 3.4 Performance Impact

Fixed-point operations are generally:
- **Faster** for basic arithmetic (add, subtract, multiply)
- **Slower** for division, square root, trigonometry
- **No SIMD benefit** without custom SIMD fixed-point implementations

---

## 4. Proposed Solution Architecture

### 4.1 Recommended Approach: Layered Deterministic Physics

Given the complexity of modifying Jolt, I recommend a **layered approach**:

```
┌─────────────────────────────────────────────────────────┐
│                    Game Logic Layer                      │
│         (Uses Fixed-Point for deterministic state)       │
└─────────────────────────────────────────────────────────┘
                           │
                           ▼
┌─────────────────────────────────────────────────────────┐
│              Deterministic Physics Layer                 │
│     (New fixed-point physics engine for gameplay)        │
│  - Position, velocity, collision response                │
│  - Deterministic across all platforms                    │
└─────────────────────────────────────────────────────────┘
                           │
                           ▼
┌─────────────────────────────────────────────────────────┐
│                  Jolt Physics Layer                      │
│     (Used for visual/non-deterministic physics)          │
│  - Ragdolls, debris, visual effects                      │
│  - Non-gameplay physics                                  │
└─────────────────────────────────────────────────────────┘
```

### 4.2 Core Components to Create

#### A. Fixed-Point Math Library (`core/math/fixed/`)

```
core/math/fixed/
├── fixed_point.h          # Core fixed-point type (FP32, FP64)
├── fixed_vector3.h        # Fixed-point Vector3
├── fixed_vector2.h        # Fixed-point Vector2
├── fixed_quaternion.h     # Fixed-point Quaternion
├── fixed_basis.h          # Fixed-point Basis (3x3 matrix)
├── fixed_transform3d.h    # Fixed-point Transform3D
├── fixed_aabb.h           # Fixed-point AABB
├── fixed_math_funcs.h     # Deterministic math functions
└── fixed_math_tables.h    # Precomputed lookup tables
```

#### B. Deterministic Physics Module (`modules/deterministic_physics/`)

```
modules/deterministic_physics/
├── det_physics_server_3d.h/cpp      # Physics server interface
├── det_body_3d.h/cpp                # Rigid body simulation
├── det_collision_shape_3d.h/cpp     # Collision shapes
├── det_collision_solver.h/cpp       # Collision detection/response
├── det_broadphase.h/cpp             # Spatial partitioning
├── det_constraint_solver.h/cpp      # Constraint solving
└── det_world_3d.h/cpp               # Physics world management
```

#### C. Conversion Layer

```cpp
// Convert between float and fixed-point at system boundaries
namespace DeterministicConversion {
    FixedVector3 to_fixed(const Vector3& v);
    Vector3 to_float(const FixedVector3& v);
    // ... etc
}
```

---

## 5. Implementation Phases

### Phase 1: Fixed-Point Math Foundation (4-6 weeks)

**Goal:** Create a complete fixed-point math library

**Tasks:**
1. Implement `FixedPoint<N>` template class with configurable precision
2. Implement `FixedVector3`, `FixedVector2` with all operations
3. Implement `FixedQuaternion` with slerp, multiplication, normalization
4. Implement `FixedBasis` and `FixedTransform3D`
5. Implement deterministic math functions:
   - Lookup table-based `sin`, `cos`, `tan`
   - Newton-Raphson `sqrt`, `rsqrt`
   - CORDIC-based `atan2`
6. Create comprehensive unit tests for cross-platform consistency

**Files to Create:**
- `core/math/fixed/fixed_point.h`
- `core/math/fixed/fixed_vector3.h`
- `core/math/fixed/fixed_vector2.h`
- `core/math/fixed/fixed_quaternion.h`
- `core/math/fixed/fixed_basis.h`
- `core/math/fixed/fixed_transform3d.h`
- `core/math/fixed/fixed_aabb.h`
- `core/math/fixed/fixed_math_funcs.h`
- `core/math/fixed/fixed_math_funcs.cpp`

### Phase 2: Deterministic Collision Detection (4-6 weeks)

**Goal:** Implement fixed-point collision detection

**Tasks:**
1. Implement fixed-point collision shapes:
   - Sphere, Box, Capsule, Cylinder
   - Convex polygon, Concave polygon (mesh)
   - Height map
2. Implement GJK/EPA in fixed-point for convex shapes
3. Implement SAT for primitive shapes
4. Implement fixed-point broadphase (BVH or Grid)
5. Create collision query interface

**Files to Create:**
- `modules/deterministic_physics/shapes/det_shape_3d.h/cpp`
- `modules/deterministic_physics/shapes/det_sphere_shape.h/cpp`
- `modules/deterministic_physics/shapes/det_box_shape.h/cpp`
- `modules/deterministic_physics/shapes/det_capsule_shape.h/cpp`
- `modules/deterministic_physics/collision/det_gjk.h/cpp`
- `modules/deterministic_physics/collision/det_sat.h/cpp`
- `modules/deterministic_physics/collision/det_broadphase.h/cpp`

### Phase 3: Deterministic Physics Solver (6-8 weeks)

**Goal:** Implement fixed-point rigid body dynamics

**Tasks:**
1. Implement rigid body representation:
   - Position, rotation (quaternion)
   - Linear/angular velocity
   - Mass, inertia tensor
2. Implement force/torque integration
3. Implement contact constraint solver
4. Implement joint constraints (optional):
   - Fixed, Hinge, Slider
5. Implement island-based sleeping

**Files to Create:**
- `modules/deterministic_physics/objects/det_body_3d.h/cpp`
- `modules/deterministic_physics/solver/det_contact_solver.h/cpp`
- `modules/deterministic_physics/solver/det_constraint_solver.h/cpp`
- `modules/deterministic_physics/solver/det_island_solver.h/cpp`
- `modules/deterministic_physics/det_world_3d.h/cpp`

### Phase 4: Godot Integration (3-4 weeks)

**Goal:** Integrate with Godot's physics server system

**Tasks:**
1. Implement `DeterministicPhysicsServer3D` class
2. Register as alternative physics backend
3. Create GDScript/C# bindings
4. Implement conversion layer for hybrid use with Jolt
5. Add project settings for deterministic physics

**Files to Create/Modify:**
- `modules/deterministic_physics/det_physics_server_3d.h/cpp`
- `modules/deterministic_physics/register_types.cpp`
- `modules/deterministic_physics/config.py`

### Phase 5: Testing and Validation (2-4 weeks)

**Goal:** Verify cross-platform determinism

**Tasks:**
1. Create determinism test suite
2. Run identical simulations on multiple platforms
3. Compare state checksums frame-by-frame
4. Performance benchmarking
5. Edge case testing (numerical limits, denormals)

---

## 6. Detailed Change List

### 6.1 New Files to Create

#### Core Math Library (13 files)
| File | Description | Lines (est.) |
|------|-------------|--------------|
| `core/math/fixed/fixed_point.h` | Core fixed-point type template | 400 |
| `core/math/fixed/fixed_point.cpp` | Fixed-point implementation | 200 |
| `core/math/fixed/fixed_vector3.h` | Fixed-point 3D vector | 350 |
| `core/math/fixed/fixed_vector2.h` | Fixed-point 2D vector | 250 |
| `core/math/fixed/fixed_quaternion.h` | Fixed-point quaternion | 400 |
| `core/math/fixed/fixed_basis.h` | Fixed-point 3x3 rotation matrix | 350 |
| `core/math/fixed/fixed_transform3d.h` | Fixed-point 3D transform | 200 |
| `core/math/fixed/fixed_aabb.h` | Fixed-point AABB | 150 |
| `core/math/fixed/fixed_plane.h` | Fixed-point plane | 100 |
| `core/math/fixed/fixed_math_funcs.h` | Deterministic math function declarations | 200 |
| `core/math/fixed/fixed_math_funcs.cpp` | Deterministic math implementations | 800 |
| `core/math/fixed/fixed_math_tables.h` | Precomputed sin/cos tables | 500 |
| `core/math/fixed/SCsub` | Build configuration | 20 |

#### Deterministic Physics Module (25+ files)
| File | Description | Lines (est.) |
|------|-------------|--------------|
| `modules/deterministic_physics/config.py` | Module configuration | 30 |
| `modules/deterministic_physics/register_types.cpp` | Module registration | 100 |
| `modules/deterministic_physics/det_physics_server_3d.h` | Server interface header | 300 |
| `modules/deterministic_physics/det_physics_server_3d.cpp` | Server implementation | 1500 |
| `modules/deterministic_physics/det_world_3d.h` | Physics world header | 150 |
| `modules/deterministic_physics/det_world_3d.cpp` | Physics world implementation | 600 |
| `modules/deterministic_physics/objects/det_body_3d.h` | Body header | 200 |
| `modules/deterministic_physics/objects/det_body_3d.cpp` | Body implementation | 500 |
| `modules/deterministic_physics/objects/det_area_3d.h/cpp` | Area trigger | 300 |
| `modules/deterministic_physics/shapes/det_shape_3d.h` | Base shape header | 100 |
| `modules/deterministic_physics/shapes/det_sphere_shape.h/cpp` | Sphere collision | 150 |
| `modules/deterministic_physics/shapes/det_box_shape.h/cpp` | Box collision | 200 |
| `modules/deterministic_physics/shapes/det_capsule_shape.h/cpp` | Capsule collision | 250 |
| `modules/deterministic_physics/collision/det_collision_solver.h/cpp` | Collision detection | 800 |
| `modules/deterministic_physics/collision/det_broadphase.h/cpp` | Spatial acceleration | 600 |
| `modules/deterministic_physics/solver/det_constraint_solver.h/cpp` | Constraint solver | 1000 |

**Total Estimated New Code: ~10,000-15,000 lines**

### 6.2 Files to Modify

| File | Modification |
|------|--------------|
| `core/math/SCsub` | Add `fixed/` subdirectory to build |
| `core/SCsub` | Include fixed-point math in core build |
| `servers/register_server_types.cpp` | Register deterministic physics server |
| `doc/classes/` | Add documentation for new types |

### 6.3 Build System Changes

| File | Change |
|------|--------|
| `SConstruct` | Add `deterministic_physics` module option |
| `modules/deterministic_physics/SCsub` | Module build script |
| `core/math/fixed/SCsub` | Fixed-point math build script |

---

## 7. Risk Assessment

### High Risk

| Risk | Impact | Mitigation |
|------|--------|------------|
| Performance degradation | May be 2-5x slower than Jolt | Optimize hot paths, use lookup tables, profile extensively |
| Fixed-point overflow | Simulation instability | Careful range analysis, saturation arithmetic |
| Numerical precision loss | Physics behaves incorrectly | Use 64-bit fixed-point where needed |

### Medium Risk

| Risk | Impact | Mitigation |
|------|--------|------------|
| API incompatibility | Existing code breaks | Provide conversion utilities |
| Missing edge cases | Rare bugs in production | Extensive fuzz testing |
| Integration complexity | Long development time | Phased approach, regular testing |

### Low Risk

| Risk | Impact | Mitigation |
|------|--------|------------|
| Documentation gaps | Developer confusion | Document during implementation |
| Platform-specific bugs | Inconsistent behavior | CI testing on multiple platforms |

---

## 8. Alternative Approaches

### 8.1 Soft-Float Library (Alternative A)

**Approach:** Use software floating-point emulation (e.g., Berkeley SoftFloat)

**Pros:**
- Guaranteed IEEE 754 compliance
- Could potentially wrap existing Jolt code
- Less code to write

**Cons:**
- 10-100x slower than hardware float
- Still subject to some platform variations
- Doesn't solve FMA/SIMD issues

### 8.2 Fork Jolt Physics (Alternative B)

**Approach:** Create a fixed-point fork of Jolt

**Pros:**
- Full Jolt feature set
- Single physics system

**Cons:**
- 6-12 months of work minimum
- Ongoing maintenance burden
- May break Jolt's SIMD optimizations

### 8.3 Use Existing Deterministic Engine (Alternative C)

**Approach:** Integrate an existing deterministic physics engine

**Candidates:**
- **Box2D** (2D only, has fixed-point forks)
- **Rapier** (Rust, would need bindings)
- **Custom engines** (various quality)

**Pros:**
- Faster time to market
- Proven implementations

**Cons:**
- May lack Godot integration
- Feature limitations
- Licensing concerns

### 8.4 Recommended: Hybrid Approach

Use the deterministic physics layer for **gameplay-critical** physics and Jolt for **visual** physics:

```
Deterministic Layer           Jolt Layer
- Player movement            - Ragdolls
- Projectiles               - Debris
- Game-critical collisions  - Particle physics
- Networked objects         - Visual effects
```

---

## 9. Testing Strategy

### 9.1 Unit Tests

```cpp
// Example: Cross-platform determinism test
TEST_CASE("FixedVector3 operations are deterministic") {
    FixedVector3 a(FP64(1.5), FP64(2.0), FP64(-3.5));
    FixedVector3 b(FP64(0.5), FP64(-1.0), FP64(2.0));

    FixedVector3 result = a.cross(b).normalized();

    // These exact values must match on ALL platforms
    CHECK(result.x.raw() == 0x00004A3D5C2E8F1A);
    CHECK(result.y.raw() == 0xFFFF8B2C4D1E0A3B);
    CHECK(result.z.raw() == 0x00001234567890AB);
}
```

### 9.2 Integration Tests

1. **Simulation Checksum Test:**
   - Run identical simulation for 1000 frames
   - Hash final state
   - Compare hash across Windows, Linux, macOS, ARM

2. **Replay Validation:**
   - Record input sequence
   - Play back on different platforms
   - Verify final state matches

### 9.3 Performance Benchmarks

| Benchmark | Target | Measurement |
|-----------|--------|-------------|
| 1000 rigid bodies | <16ms/frame | Frame time |
| Collision detection | <5ms/1000 pairs | Query time |
| Math operations | <2x float speed | Operations/second |

---

## 10. Performance Considerations

### 10.1 Optimization Strategies

1. **Lookup Tables:** Precompute sin/cos/tan for 65536 angles
2. **Newton-Raphson:** Fast approximate sqrt/rsqrt
3. **Batch Operations:** Process multiple bodies together
4. **SIMD Fixed-Point:** Custom SIMD for fixed-point (advanced)
5. **Island Optimization:** Only simulate awake islands

### 10.2 Expected Performance

| Operation | Float (ns) | Fixed-Point (ns) | Ratio |
|-----------|------------|------------------|-------|
| Add/Sub | 1 | 1 | 1.0x |
| Multiply | 1 | 2-3 | 2-3x |
| Divide | 5 | 10-20 | 2-4x |
| sqrt | 5 | 20-50 | 4-10x |
| sin/cos | 10 | 5-10 (LUT) | 0.5-1x |
| Normalize | 10 | 30-60 | 3-6x |

**Overall Physics Step:** Expect 2-4x slower than Jolt for equivalent simulation

---

## 11. Project Timeline Summary

| Phase | Duration | Deliverable |
|-------|----------|-------------|
| Phase 1: Math Library | 4-6 weeks | Complete fixed-point math |
| Phase 2: Collision | 4-6 weeks | Collision detection system |
| Phase 3: Physics Solver | 6-8 weeks | Rigid body dynamics |
| Phase 4: Integration | 3-4 weeks | Godot integration |
| Phase 5: Testing | 2-4 weeks | Validated determinism |
| **Total** | **19-28 weeks** | **Production-ready system** |

---

## 12. Decision Points Requiring Input

Before proceeding, the following decisions need to be made:

1. **Fixed-Point Precision:**
   - Q16.16 (32-bit) - Faster, limited range
   - Q32.32 (64-bit) - Slower, larger range
   - Configurable - Most flexible, more complex

2. **Scope of Determinism:**
   - Full physics replacement (all physics deterministic)
   - Hybrid approach (gameplay deterministic, visuals use Jolt)
   - Selective (only marked objects are deterministic)

3. **Feature Parity with Jolt:**
   - Full parity (joints, soft bodies, etc.) - More work
   - Core features only (rigid bodies, collision) - Faster delivery
   - Minimal (collision queries only) - Fastest

4. **API Design:**
   - Mirror existing PhysicsServer3D API
   - New deterministic-specific API
   - Hybrid with compatibility layer

---

## 13. Next Steps

Upon approval of this plan:

1. Create the `core/math/fixed/` directory structure
2. Implement `FixedPoint` base type with tests
3. Implement `FixedVector3` and validate cross-platform
4. Proceed through phases iteratively

---

*Document Version: 1.0*
*Created: 2025-01-21*
*Status: AWAITING APPROVAL*
