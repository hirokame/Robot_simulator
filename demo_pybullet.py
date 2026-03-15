# ============================================================
#  PyBullet Quick Demo
#  Install:  pip install pybullet
#  Run:      python demo_pybullet.py
# ============================================================
#
#  PyBullet is a Python binding for the Bullet physics engine.
#  It is lightweight, easy to install, and great for:
#    - Reinforcement learning experiments
#    - Robot arm / mobile robot simulation
#    - Collision detection & rigid-body dynamics
#
# ============================================================

import pybullet as p
import pybullet_data
import time
import math

# ----------------------------------------------------------
# 1. CONNECT TO THE PHYSICS SERVER
#    p.GUI   → opens a 3-D viewer window (interactive)
#    p.DIRECT → headless (no window, faster, good for training)
# ----------------------------------------------------------
client = p.connect(p.GUI)          # change to p.DIRECT if you don't want a window
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # lets us load built-in URDFs

print("=" * 55)
print("  PyBullet Demo  —  Koji's Robotics Playground")
print("=" * 55)

# ----------------------------------------------------------
# 2. SET UP THE WORLD
#    Gravity = -9.8 m/s² downward (like Earth)
# ----------------------------------------------------------
p.setGravity(0, 0, -9.8)

# Load a flat ground plane
plane_id = p.loadURDF("plane.urdf")
print(f"\n[World]  Ground plane loaded  (id={plane_id})")

# ----------------------------------------------------------
# 3. LOAD A ROBOT — the classic KUKA robot arm
#    URDF = Unified Robot Description Format
#    It describes the robot's links, joints, mass, inertia
# ----------------------------------------------------------
robot_start_pos = [0, 0, 0]
robot_start_ori = p.getQuaternionFromEuler([0, 0, 0])  # no rotation

kuka_id = p.loadURDF("kuka_iiwa/model.urdf",
                      basePosition=robot_start_pos,
                      baseOrientation=robot_start_ori,
                      useFixedBase=True)   # bolt the base to the ground

num_joints = p.getNumJoints(kuka_id)
print(f"[Robot]  KUKA iiwa loaded     (id={kuka_id}, joints={num_joints})")

# ----------------------------------------------------------
# 4. INSPECT THE JOINTS
# ----------------------------------------------------------
print("\n[Joints]")
for j in range(num_joints):
    info = p.getJointInfo(kuka_id, j)
    joint_name  = info[1].decode("utf-8")
    joint_type  = ["REVOLUTE","PRISMATIC","SPHERICAL","PLANAR","FIXED"][info[2]]
    print(f"  Joint {j:2d}  {joint_name:<30s}  type={joint_type}")

# ----------------------------------------------------------
# 5. MOVE THE ARM — set target angles for each joint
#    POSITION_CONTROL: PD controller drives joint to target
# ----------------------------------------------------------
target_angles = [
    math.radians(30),   # joint 0
    math.radians(45),   # joint 1
    math.radians(0),    # joint 2
    math.radians(-60),  # joint 3
    math.radians(0),    # joint 4
    math.radians(45),   # joint 5
    math.radians(0),    # joint 6
]

for j, angle in enumerate(target_angles):
    p.setJointMotorControl2(
        bodyUniqueId=kuka_id,
        jointIndex=j,
        controlMode=p.POSITION_CONTROL,
        targetPosition=angle,
        force=500          # maximum motor force (N·m)
    )

print("\n[Control]  Commanding arm to target pose...")

# ----------------------------------------------------------
# 6. STEP THE SIMULATION — run for 300 physics steps
#    Each step = 1/240 second by default (240 Hz)
#    Total simulated time ≈ 300 / 240 = 1.25 seconds
# ----------------------------------------------------------
print("[Sim]     Stepping 300 physics steps (≈1.25 s simulated)...")

for step in range(300):
    p.stepSimulation()
    time.sleep(1. / 240.)   # slow down so the viewer is watchable
                             # remove this line for maximum speed

# ----------------------------------------------------------
# 7. READ JOINT STATES
#    Returns: (position, velocity, reaction_forces, motor_torque)
# ----------------------------------------------------------
print("\n[Results]  Final joint states:")
print(f"  {'Joint':<8} {'Position (rad)':>16} {'Position (°)':>14} {'Velocity (rad/s)':>18}")
print("  " + "-" * 60)
for j in range(num_joints):
    pos, vel, _, torque = p.getJointState(kuka_id, j)
    print(f"  Joint {j:<3}  {pos:>16.4f}   {math.degrees(pos):>12.2f}°   {vel:>16.4f}")

# ----------------------------------------------------------
# 8. READ THE END-EFFECTOR (tip of the arm) POSITION
# ----------------------------------------------------------
end_effector_idx = num_joints - 1
link_state = p.getLinkState(kuka_id, end_effector_idx)
world_pos = link_state[4]   # [4] = world position of the link frame
print(f"\n[End-effector]  World position: x={world_pos[0]:.4f}  y={world_pos[1]:.4f}  z={world_pos[2]:.4f}")

# ----------------------------------------------------------
# 9. SPAWN A BALL AND LET IT FALL
# ----------------------------------------------------------
ball_visual   = p.createVisualShape(p.GEOM_SPHERE, radius=0.05, rgbaColor=[1, 0.2, 0.2, 1])
ball_collision = p.createCollisionShape(p.GEOM_SPHERE, radius=0.05)
ball_id = p.createMultiBody(
    baseMass=0.5,
    baseCollisionShapeIndex=ball_collision,
    baseVisualShapeIndex=ball_visual,
    basePosition=[0.5, 0.0, 1.0]   # start 1 m above the ground
)
print(f"\n[Ball]  Spawned at (0.5, 0.0, 1.0), mass=0.5 kg")

print("[Sim]   Dropping ball — stepping 240 more steps (1 second)...")
for step in range(240):
    p.stepSimulation()
    time.sleep(1. / 240.)

ball_pos, ball_ori = p.getBasePositionAndOrientation(ball_id)
print(f"[Ball]  Resting position: x={ball_pos[0]:.4f}  y={ball_pos[1]:.4f}  z={ball_pos[2]:.4f}")

# ----------------------------------------------------------
# 10. CLEANUP
# ----------------------------------------------------------
print("\n[Done]  Disconnecting from physics server.")
p.disconnect()
print("=" * 55)
print("  Demo complete! Try changing target_angles or")
print("  spawning more objects to explore further.")
print("=" * 55)
