# Runtime Execution Flow Diagrams

## Sequence Diagram 1: Joint Space Command

```
User Code          ArmController     KinematicChain      Joint(s)
    │                    │                   │              │
    │ create command     │                   │              │
    │ JointSpace(45°)    │                   │              │
    ├───────────────────►│                   │              │
    │                    │                   │              │
    │ execute(cmd)       │                   │              │
    ├───────────────────►│                   │              │
    │                    │                   │              │
    │                    │ cmd->execute()    │              │
    │                    │ (virtual call)    │              │
    │                    ├──────────┐        │              │
    │                    │          │        │              │
    │                    │◄─────────┘        │              │
    │                    │                   │              │
    │                    │ setJointAngles()  │              │
    │                    ├──────────────────►│              │
    │                    │                   │              │
    │                    │                   │ setAngle(45°)│
    │                    │                   ├─────────────►│
    │                    │                   │              │
    │                    │                   │ validate()   │
    │                    │                   │◄─────────────┤
    │                    │                   │              │
    │                    │                   │ OK           │
    │                    │                   │──────────────►│
    │                    │                   │              │
    │                    │  success          │              │
    │                    │◄──────────────────┤              │
    │                    │                   │              │
    │  done              │                   │              │
    │◄───────────────────┤                   │              │
    │                    │                   │              │
```

## Sequence Diagram 2: Cartesian Command (with IK)

```
User Code    ArmController  CartesianCommand  IKSolver  KinematicChain  Joint(s)
    │              │               │             │            │            │
    │ create cmd   │               │             │            │            │
    │ Cartesian    │               │             │            │            │
    │ (150,0,100)  │               │             │            │            │
    ├─────────────►│               │             │            │            │
    │              │               │             │            │            │
    │ execute(cmd) │               │             │            │            │
    ├─────────────►│               │             │            │            │
    │              │               │             │            │            │
    │              │ execute()     │             │            │            │
    │              ├──────────────►│             │            │            │
    │              │               │             │            │            │
    │              │               │ solve(xyz)  │            │            │
    │              │               ├────────────►│            │            │
    │              │               │             │            │            │
    │              │               │   IK math   │            │            │
    │              │               │   happens   │            │            │
    │              │               │             │            │            │
    │              │               │ angles[3]   │            │            │
    │              │               │◄────────────┤            │            │
    │              │               │             │            │            │
    │              │               │ applyAngles()            │            │
    │              │               ├──────────────────────────►│            │
    │              │               │                          │            │
    │              │               │                          │ set each   │
    │              │               │                          ├───────────►│
    │              │               │                          │            │
    │              │  success      │                          │            │
    │              │◄──────────────┤                          │            │
    │              │               │                          │            │
    │  done        │               │                          │            │
    │◄─────────────┤               │                          │            │
    │              │               │                          │            │
```

## Sequence Diagram 3: Forward Kinematics Query

```
User Code       ArmController    KinematicChain    Joint(s)    Transform
    │                 │                 │             │            │
    │ getEndEffector  │                 │             │            │
    │ Position()      │                 │             │            │
    ├────────────────►│                 │             │            │
    │                 │                 │             │            │
    │                 │ computeFK()     │             │            │
    │                 ├────────────────►│             │            │
    │                 │                 │             │            │
    │                 │                 │ getAngle()  │            │
    │                 │                 ├────────────►│            │
    │                 │                 │ angle1      │            │
    │                 │                 │◄────────────┤            │
    │                 │                 │             │            │
    │                 │                 │ getAngle()  │            │
    │                 │                 ├────────────►│            │
    │                 │                 │ angle2      │            │
    │                 │                 │◄────────────┤            │
    │                 │                 │             │            │
    │                 │                 │ buildTransform(angle1)   │
    │                 │                 ├─────────────────────────►│
    │                 │                 │             │  T1        │
    │                 │                 │◄─────────────────────────┤
    │                 │                 │             │            │
    │                 │                 │ buildTransform(angle2)   │
    │                 │                 ├─────────────────────────►│
    │                 │                 │             │  T2        │
    │                 │                 │◄─────────────────────────┤
    │                 │                 │             │            │
    │                 │                 │ T1 * T2 * T3...         │
    │                 │                 ├──────────┐  │            │
    │                 │                 │          │  │            │
    │                 │                 │◄─────────┘  │            │
    │                 │                 │             │            │
    │                 │  position(x,y,z)│             │            │
    │                 │◄────────────────┤             │            │
    │                 │                 │             │            │
    │  (x, y, z)      │                 │             │            │
    │◄────────────────┤                 │             │            │
    │                 │                 │             │            │
```

## State Machine View: Joint Validation

```
                    ┌──────────────┐
         ┌─────────►│   IDLE       │
         │          └──────┬───────┘
         │                 │
         │                 │ setAngle() called
         │                 ▼
         │          ┌──────────────┐
         │          │  VALIDATING  │
         │          └──────┬───────┘
         │                 │
         │          ┌──────┴──────┐
         │          │             │
         │      angle < min   angle > max
         │          │             │
         │          ▼             ▼
         │   ┌────────────┐ ┌────────────┐
         │   │   REJECT   │ │   REJECT   │
         │   └─────┬──────┘ └─────┬──────┘
         │         │              │
         │         └──────┬───────┘
         │                │
         │                │ throw exception
         │                ▼
         │         ┌────────────┐
         └─────────┤   ERROR    │
                   └────────────┘
         
         If valid (min ≤ angle ≤ max):
         
         ┌──────────────┐
         │  VALIDATING  │
         └──────┬───────┘
                │
                │ angle is valid
                ▼
         ┌──────────────┐
         │   UPDATING   │
         └──────┬───────┘
                │
                │ store new angle
                ▼
         ┌──────────────┐
         │   IDLE       │
         └──────────────┘
```

## Memory Ownership Flow

```
Stack (User Code):
┌─────────────────────────────────────┐
│  main() {                           │
│    ArmController controller;        │◄──── Lives on stack
│    CartesianCommand cmd(x,y,z);     │◄──── Lives on stack
│    controller.execute(&cmd);        │
│  }                                  │
└─────────────────────────────────────┘
         │
         │ controller contains:
         ▼
Heap (Owned by ArmController):
┌─────────────────────────────────────┐
│  unique_ptr<KinematicChain>         │◄──── Owned by controller
│    └─ vector<unique_ptr<Joint>>     │◄──── Owned by chain
│          ├─ Joint #1                │
│          ├─ Joint #2                │
│          └─ Joint #3                │
│                                     │
│  unique_ptr<IKSolver>               │◄──── Owned by controller
│    └─ AnalyticIK or JacobianIK     │
└─────────────────────────────────────┘

When controller goes out of scope:
  → unique_ptrs automatically delete KinematicChain and IKSolver
  → KinematicChain's vector automatically deletes all Joints
  → No manual delete needed! (RAII)
```

## Execution Timeline: Path Command

```
Time  ──────────────────────────────────────────────────►

t=0ms   Start PathCommand
        │
        ├─ Parse waypoints: [A, B, C, D]
        │
t=10ms  ├─ Interpolate A→B (compute intermediate points)
        │  └─ Points: A, A', A'', A''', B
        │
t=20ms  ├─ Execute sub-command: move to A'
        │  └─ IK solve → Joint angles
        │  └─ Apply to chain
        │
t=30ms  ├─ Execute sub-command: move to A''
        │  └─ IK solve → Joint angles
        │  └─ Apply to chain
        │
t=40ms  ├─ Execute sub-command: move to A'''
        │
...     │
        │
t=100ms ├─ Reached B
        │
        ├─ Interpolate B→C
        │
...     │
        │
t=300ms └─ Reached D (final waypoint)
        
        PathCommand complete
```

## Plugin Architecture (Future Extension)

```
Current State:
┌──────────────────┐
│  ArmController   │
│                  │
│  hardcoded IK    │◄─ Currently uses specific IK solver
└──────────────────┘

Future State (easily extensible):
┌──────────────────┐
│  ArmController   │
│                  │
│  IK interface    │◄─ Points to abstract interface
└────────┬─────────┘
         │
         │ Can swap at runtime
         │
    ┌────┴─────┬──────────┬───────────┐
    │          │          │           │
    ▼          ▼          ▼           ▼
┌────────┐┌────────┐┌─────────┐┌──────────┐
│Analytic││Jacobian││ CCD IK  ││ Learning │
│  IK    ││  IK    ││(future) ││Based IK  │
│        ││        ││         ││(future)  │
└────────┘└────────┘└─────────┘└──────────┘

All you need: implement InverseKinematics interface
```

## Error Propagation Flow

```
Joint::setAngle(90°)
    │
    │ Check: 90° > maxLimit (85°)
    │
    ├─ throw JointLimitException("Joint 2 exceeded max limit")
    │
    ▼
CartesianCommand::execute()
    │
    │ catch (JointLimitException& e)
    │
    ├─ log error
    │
    ├─ return COMMAND_FAILED
    │
    ▼
ArmController::execute()
    │
    │ check return code
    │
    ├─ if FAILED: rollback to previous state
    │
    └─ notify user: "Target unreachable"
```

## Testing Flow

```
test_kinematics.cpp:

TEST: ForwardKinematics_3DOF_AllZeros
│
├─ Create KinematicChain with 3 joints
├─ Set all joint angles to 0°
├─ Call computeForwardKinematics()
│
└─ Assert: end effector at (L1+L2+L3, 0, 0)
   Expected: (180mm, 0, 0)
   Actual:   (180mm, 0, 0)
   PASS ✓

TEST: InverseKinematics_ReachableTarget
│
├─ Create arm with known geometry
├─ Target: (100, 50, 75)
├─ Call solver->solve(target)
│
├─ Get joint angles: [θ1, θ2, θ3]
├─ Apply to arm
├─ Compute forward kinematics
│
└─ Assert: FK result ≈ original target
   Error: 0.01mm
   PASS ✓
```
