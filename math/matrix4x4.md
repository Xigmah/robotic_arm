## Understanding Matrix4x4 Structure

Before we code, let's visualize what we're building:
```
Matrix4x4 stored as:
     col 0   col 1   col 2   col 3
   ┌─────────────────────────────────┐
r0 │ m[0][0] m[0][1] m[0][2] m[0][3] │  ← Translation X
r1 │ m[1][0] m[1][1] m[1][2] m[1][3] │  ← Translation Y
r2 │ m[2][0] m[2][1] m[2][2] m[2][3] │  ← Translation Z
r3 │ m[3][0] m[3][1] m[3][2] m[3][3] │  ← Always [0 0 0 1]
   └─────────────────────────────────┘
      ↑
   Rotation part (3x3 top-left)
