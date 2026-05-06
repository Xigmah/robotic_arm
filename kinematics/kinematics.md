# Forward Kinematics

## Denavit-Hartenberg (DH) Parameters

There's a **standard way** to define these transformations called DH parameters. It's a convention that describes any joint with 4 numbers:

For each joint i:
- **θ_i** (theta): Joint angle (this is what your servo controls)
- **d_i**: Offset along the previous Z-axis
- **a_i**: Link length (offset along the new X-axis)
- **α_i** (alpha): Twist angle (rotation around the new X-axis)

The transformation for joint i is:
```
T_i = Rot_z(θ_i) * Trans_z(d_i) * Trans_x(a_i) * Rot_x(α_i)
```

**For many arms, some of these are zero.** For example, a simple arm might have:
- θ = joint angle (variable)
- d = 0 (no offset along Z)
- a = link length (fixed)
- α = 0 (no twist)

So it simplifies to:
```
T_i = Rot_z(θ_i) * Trans_x(a_i)
```

## Forward Kinematics: Chain of Transformations

For a robotic arm, each joint applies a transformation. You **multiply these transformations together** to get the final end effector position.

### Example: Simple 3-DOF Arm

Let's say you have:
- **Joint 1:** Rotates around Z-axis (base rotation)
- **Joint 2:** Rotates around Y-axis (shoulder)
- **Joint 3:** Rotates around Y-axis (elbow)

Each joint also has a **link** attached to it (the arm segment).

```
                    Link 3 (L3)
                         │
                    ┌────┴────┐  ← Joint 3 (θ3, rotates around Y)
                    │         │
               Link 2 (L2)    │
                    │         │
               ┌────┴────┐    │
               │         │    │
          ┌────┴────┐    │    │
    Base  │ Joint 2 │    │    │  ← θ2 (rotates around Y)
     │    │  (θ2)   │    │    │
     │    └─────────┘    │    │
     │                   │    │
     │    Link 1 (L1)    │    │
     │         │         │    │
     └─────────┴─────────┴────┘
          Joint 1 (θ1)         ← θ1 (rotates around Z)
            Base
```

Each joint contributes TWO transformations:
1. **Rotation** by its joint angle
2. **Translation** by its link length

For Joint 1 (rotates around Z, then moves along X by L1):
```
T1 = Rot_z(θ1) * Trans_x(L1)
```

For Joint 2 (rotates around Y, then moves along X by L2):
```
T2 = Rot_y(θ2) * Trans_x(L2)
```

For Joint 3 (rotates around Y, then moves along X by L3):
```
T3 = Rot_y(θ3) * Trans_x(L3)
```

### Final Transformation:

The end effector position is:
```
T_final = T1 * T2 * T3
```

The position is in the **right column** of T_final:
```
         ┌               ┐
T_final =│ R11 R12 R13  X│  ← X is end effector x-coordinate
         │ R21 R22 R23  Y│  ← Y is end effector y-coordinate
         │ R31 R32 R33  Z│  ← Z is end effector z-coordinate
         │  0   0   0   1│
         └               ┘
```

So your `forwardKinematics()` function extracts:
```cpp
Vector3D endEffector;
endEffector.x = T_final[0][3];  // top-right element
endEffector.y = T_final[1][3];  // second row, right column
endEffector.z = T_final[2][3];  // third row, right column
```

### Concrete Example: Calculate by Hand

Let's do a simple 2-joint arm:
- Joint 1: rotates around Z, link length L1 = 100mm
- Joint 2: rotates around Y, link length L2 = 80mm
- Both joints at 0°

#### Step 1: Joint 1 (θ1 = 0°, rotate Z, translate X by 100)

```
T1 = Rot_z(0) * Trans_x(100)

     ┌              ┐   ┌              ┐
   = │ 1  0  0   0  │ * │ 1  0  0  100 │
     │ 0  1  0   0  │   │ 0  1  0   0  │
     │ 0  0  1   0  │   │ 0  0  1   0  │
     │ 0  0  0   1  │   │ 0  0  0   1  │
     └              ┘   └              ┘

     ┌              ┐
   = │ 1  0  0  100 │
     │ 0  1  0   0  │
     │ 0  0  1   0  │
     │ 0  0  0   1  │
     └              ┘
```

After joint 1, we're at position (100, 0, 0).

#### Step 2: Joint 2 (θ2 = 0°, rotate Y, translate X by 80)

```
T2 = Rot_y(0) * Trans_x(80)

     ┌              ┐   ┌              ┐
   = │ 1  0  0   0  │ * │ 1  0  0  80  │
     │ 0  1  0   0  │   │ 0  1  0   0  │
     │ 0  0  1   0  │   │ 0  0  1   0  │
     │ 0  0  0   1  │   │ 0  0  0   1  │
     └              ┘   └              ┘

     ┌              ┐
   = │ 1  0  0  80  │
     │ 0  1  0   0  │
     │ 0  0  1   0  │
     │ 0  0  0   1  │
     └              ┘
```

#### Step 3: Final transformation

```
T_final = T1 * T2

         ┌              ┐   ┌              ┐
       = │ 1  0  0  100 │ * │ 1  0  0  80  │
         │ 0  1  0   0  │   │ 0  1  0   0  │
         │ 0  0  1   0  │   │ 0  0  1   0  │
         │ 0  0  0   1  │   │ 0  0  0   1  │
         └              ┘   └              ┘

         ┌              ┐
       = │ 1  0  0  180 │  ← End effector at (180, 0, 0)
         │ 0  1  0   0  │
         │ 0  0  1   0  │
         │ 0  0  0   1  │
         └              ┘
```

**Result:** End effector is at **(180, 0, 0)** - which makes sense! Both joints straight, so it's L1 + L2 = 100 + 80 = 180mm along X.


# Inverse Kinematics

## The Geometric Approach

### **Step 1: Solve Base Angle (θ0)**

The base rotates around Z, so it only affects X-Y position:

```
θ0 = atan2(y, x)
```

This points the arm toward the target in the X-Y plane.

---

### **Step 2: Project to 2D Plane**

After base rotation, the problem becomes 2D (like a side view):
- **Horizontal distance from base:** `r = sqrt(x² + y²)`
- **Vertical distance:** `z`
- Now solve for θ1 and θ2 in this 2D plane

---

### **Step 3: Solve Shoulder and Elbow (θ1, θ2)**

This is a classic **2-link planar arm** IK problem.

Given:
- Target distance from base: `r` (horizontal)
- Target height: `z` (vertical)
- Link lengths: `L1`, `L2`, `L3`

**The geometry:**
```
      Target (r, z)
         ●
        /│
       / │
   L3 /  │ z
     /   │
    ●────┘
    │    r
 L2 │
    │
    ●
    │
 L1 │
    │
   Base
```

**Use law of cosines** to find the angles.

### **Law of Cosines:**

For a triangle with sides `a`, `b`, `c` and angle `C` opposite side `c`:

```
c² = a² + b² - 2ab·cos(C)

Solving for angle:
cos(C) = (a² + b² - c²) / (2ab)
C = acos((a² + b² - c²) / (2ab))
```

## Jacobian IK Approach

Each column of J corresponds to one joint. For each joint, you ask: **if this joint rotates by a tiny amount, what is the resulting velocity of the end effector in XYZ?**

The answer depends on two things:
- The **axis** the joint rotates around
- The **vector** from that joint's origin to the end effector

For a revolute joint, the end effector velocity contribution is the **cross product** of those two things:

```
J_column_i = axis_i × (end_effector_pos - joint_i_pos)
```

Your FK already gives you everything you need to compute this — joint positions and orientations at each step of the transform chain. So building J is essentially a second pass over your FK data.

Once you have J, the iterative solve is:
1. Compute current end effector position via FK
2. Compute error = target - current
3. Compute `Δθ = J⁺ * Δx`
4. Apply `Δθ` to joint angles
5. Repeat until error is small enough

`J⁺` is the pseudoinverse — the Buss paper covers several ways to approximate it, with damped least squares being the most practical for singularity handling.
