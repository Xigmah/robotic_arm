## The Core Idea: Transformation Matrices

In 3D, each joint doesn't just rotate in a plane - it can rotate around different axes (X, Y, or Z). To track where the end effector is, we need to think about **transformations**; how each joint moves and rotates the coordinate system.

### What is a Transformation Matrix?

A 4x4 transformation matrix encodes both **rotation** and **translation** (position shift):

```
    ┌                    ┐
    │ R11  R12  R13  Tx  │
T = │ R21  R22  R23  Ty  │
    │ R31  R32  R33  Tz  │
    │  0    0    0    1  │
    └                    ┘

Where:
- R (3x3 top-left) = rotation part
- T (right column) = translation (position) part
- Bottom row is always [0 0 0 1] (homogeneous coordinates)

Matrix4x4 stored as:
     col 0   col 1   col 2   col 3
   ┌─────────────────────────────────┐
r0 │ m[0][0] m[0][1] m[0][2] m[0][3] │
r1 │ m[1][0] m[1][1] m[1][2] m[1][3] │
r2 │ m[2][0] m[2][1] m[2][2] m[2][3] │
r3 │ m[3][0] m[3][1] m[3][2] m[3][3] │
   └─────────────────────────────────┘
```

### Why 4x4 Instead of 3x3?

**Homogeneous coordinates** let us combine rotation AND translation into a single matrix multiplication. Without it, you'd need separate operations:
- `new_point = R * old_point + T` (annoying)

With homogeneous coordinates:
- `new_point = T * old_point` (clean!)

## Basic Transformations

### 1. **Translation** (Moving in a straight line)

The diagonal is identity (no rotation), the right column adds position offset.

Move by distance `d` along X-axis:
```
         ┌              ┐
Trans_x =│ 1  0  0   d  │
         │ 0  1  0   0  │
         │ 0  0  1   0  │
         │ 0  0  0   1  │
         └              ┘
```
Move by distance `d` along Y-axis:
```
         ┌              ┐
Trans_y =│ 1  0  0   0  │
         │ 0  1  0   d  │
         │ 0  0  1   0  │
         │ 0  0  0   1  │
         └              ┘
```

Move by distance `d` along Z-axis:
```
         ┌              ┐
Trans_z =│ 1  0  0   0  │
         │ 0  1  0   0  │
         │ 0  0  1   d  │
         │ 0  0  0   1  │
         └              ┘
```

### 2. **Rotation** (Spinning around an axis)

The axis you're rotating around stays unchanged (1 on diagonal), the other two dimensions get the rotation applied.

**Rotation around Z-axis by angle θ:**
```
        ┌                        ┐
Rot_z = │ cos(θ) -sin(θ)  0   0  │
        │ sin(θ)  cos(θ)  0   0  │
        │   0       0     1   0  │
        │   0       0     0   1  │
        └                        ┘
```

**Rotation around Y-axis by angle θ:**
```
        ┌                        ┐
Rot_y = │  cos(θ)  0  sin(θ)  0  │
        │    0     1    0     0  │
        │ -sin(θ)  0  cos(θ)  0  │
        │    0     0    0     1  │
        └                        ┘
```

**Rotation around X-axis by angle θ:**
```
        ┌                        ┐
Rot_x = │  1    0       0     0  │
        │  0  cos(θ) -sin(θ)  0  │
        │  0  sin(θ)  cos(θ)  0  │
        │  0    0       0     1  │
        └                        ┘
```
