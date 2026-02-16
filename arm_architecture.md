# Robotic Arm C++ Architecture

## High-Level Class Diagram

```
┌─────────────────────────────────────────────────────────────┐
│                     ArmController                            │
│  - owns KinematicChain                                      │
│  - owns IK solver                                           │
│  - executes commands                                        │
└──────────────┬──────────────────────────────────────────────┘
               │
               │ owns
               ▼
┌─────────────────────────────────────────────────────────────┐
│                   KinematicChain                             │
│  - vector<unique_ptr<Joint>>                                │
│  - ArmConfiguration                                         │
│  - computeForwardKinematics()                               │
└──────────────┬──────────────────────────────────────────────┘
               │
               │ owns multiple
               ▼
        ┌──────────────┐
        │    Joint     │ ◄────────────────────┐
        │  (abstract)  │                      │
        │              │                      │
        │ + getAngle() │                      │
        │ + setAngle() │                      │ inherits
        │ + getLimits()│                      │
        └──────────────┘                      │
               △                              │
               │                              │
               │ inherits                     │
       ┌───────┴────────┐             ┌──────────────┐
       │                │             │ PrismaticJoint│
┌──────────────┐ ┌──────────────┐    │   (future)    │
│RevoluteJoint │ │CustomJoint   │    └───────────────┘
│              │ │  (optional)  │
│ - angle      │ │              │
│ - limits     │ └──────────────┘
│ - offset     │
└──────────────┘
```

## Inverse Kinematics Hierarchy

```
┌─────────────────────────────────────────────────────────────┐
│              InverseKinematics (abstract)                    │
│                                                              │
│  + solve(target_position) -> joint_angles                   │
│  + isReachable(position) -> bool                            │
└──────────────┬───────────────────────────────────────────────┘
               │
               │ inherits
       ┌───────┴────────────────┐
       │                        │
┌──────────────┐         ┌──────────────┐
│ AnalyticIK   │         │ JacobianIK   │
│              │         │              │
│ - 3DOF solns │         │ - iterative  │
│ - trig based │         │ - general    │
└──────────────┘         └──────────────┘
```

## Command Pattern Structure

```
                    ┌──────────────────┐
                    │   ArmController  │
                    │                  │
                    │  execute(cmd)    │
                    └────────┬─────────┘
                             │
                             │ receives
                             ▼
                    ┌──────────────────┐
                    │   ArmCommand     │
                    │   (abstract)     │
                    │                  │
                    │  + execute()     │
                    │  + validate()    │
                    └────────┬─────────┘
                             │
                             │ inherits
         ┌───────────────────┼───────────────────┐
         │                   │                   │
         ▼                   ▼                   ▼
┌─────────────────┐ ┌─────────────────┐ ┌─────────────────┐
│JointSpaceCommand│ │CartesianCommand │ │  PathCommand    │
│                 │ │                 │ │                 │
│ - joint_angles  │ │ - xyz_position  │ │ - waypoints[]   │
│                 │ │ - uses IK       │ │ - interpolation │
└─────────────────┘ └─────────────────┘ └─────────────────┘
```

## Data Flow Architecture

```
┌─────────────┐
│    User     │
│   Code      │
└──────┬──────┘
       │
       │ creates command
       ▼
┌─────────────────┐
│  ArmCommand     │
└──────┬──────────┘
       │
       │ passed to
       ▼
┌─────────────────┐         ┌──────────────────┐
│ ArmController   │────────▶│ InverseKinematics│
│                 │  uses   │                  │
└──────┬──────────┘         └──────────────────┘
       │
       │ updates
       ▼
┌─────────────────┐         ┌──────────────────┐
│ KinematicChain  │────────▶│ Joint (multiple) │
│                 │  owns   │                  │
└──────┬──────────┘         └──────────────────┘
       │
       │ computes
       ▼
┌─────────────────┐
│ End Effector    │
│   Position      │
│   (output)      │
└─────────────────┘
```

## Composition View: What Owns What

```
ArmController
├── KinematicChain
│   ├── ArmConfiguration (link lengths, limits, DH params)
│   ├── Joint #1 (unique_ptr<Joint>)
│   ├── Joint #2 (unique_ptr<Joint>)
│   └── Joint #3 (unique_ptr<Joint>)
│
└── InverseKinematics (unique_ptr<IKSolver>)
    └── (specific solver implementation)
```

## Key Responsibilities (SRP - Single Responsibility Principle)

```
┌─────────────────────────────────────────────────────────────┐
│ Joint                                                        │
│ ➜ Knows: its own state (angle, limits)                     │
│ ➜ Does: validates ranges, stores position                   │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│ KinematicChain                                              │
│ ➜ Knows: arm geometry, all joint states                    │
│ ➜ Does: forward kinematics, manages joint collection        │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│ InverseKinematics                                           │
│ ➜ Knows: how to map XYZ → joint angles                     │
│ ➜ Does: solves IK, checks reachability                      │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│ ArmCommand                                                  │
│ ➜ Knows: what motion is requested                          │
│ ➜ Does: encapsulates command data, validates itself         │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│ ArmController                                               │
│ ➜ Knows: nothing about math, delegates everything          │
│ ➜ Does: orchestrates components, executes commands          │
└─────────────────────────────────────────────────────────────┘
```

## Interface vs Implementation Split

```
Public Interface Layer (what users see):
┌─────────────────────────────────────────┐
│         ArmController API               │
│                                         │
│  - execute(command)                     │
│  - getCurrentPosition()                 │
│  - getJointAngles()                     │
└─────────────────────────────────────────┘

Implementation Layer (hidden complexity):
┌─────────────────────────────────────────┐
│  Kinematics, Joint management, IK       │
│  (users don't need to know this exists) │
└─────────────────────────────────────────┘
```

## Polymorphism in Action

```
Scenario: "Move to position (150, 0, 100)"

┌─────────────────────────────────────────────────────────┐
│  User creates: CartesianCommand(150, 0, 100)           │
└──────────────────┬──────────────────────────────────────┘
                   │
                   │ passed as ArmCommand* (base pointer)
                   ▼
┌─────────────────────────────────────────────────────────┐
│  ArmController::execute(ArmCommand* cmd)                │
│                                                         │
│  cmd->execute()  ◄─── virtual dispatch                 │
└──────────────────┬──────────────────────────────────────┘
                   │
                   │ polymorphic call
                   ▼
┌─────────────────────────────────────────────────────────┐
│  CartesianCommand::execute()                            │
│  - calls IK solver                                      │
│  - gets joint angles                                    │
│  - updates KinematicChain                               │
└─────────────────────────────────────────────────────────┘

No if/else statements needed! 
Each command knows how to execute itself.
```

## Extension Points (Where you'll add features later)

```
Easy to Add Later:
│
├─ New Joint Types
│  └─ Just inherit from Joint base class
│
├─ New IK Solvers  
│  └─ Just inherit from InverseKinematics
│
├─ New Command Types
│  └─ Just inherit from ArmCommand
│
└─ New Arm Configurations
   └─ Just create new ArmConfiguration instance
   
No need to modify existing code! (Open/Closed Principle)
```
