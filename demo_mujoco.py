# ============================================================
#  MuJoCo Quick Demo
#  Install:  pip install mujoco
#  Run:      python demo_mujoco.py
# ============================================================
#
#  MuJoCo (Multi-Joint dynamics with Contact) was acquired
#  by DeepMind in 2021 and made free. It is the #1 simulator
#  for robotics research and reinforcement learning because:
#    - Extremely accurate contact / friction physics
#    - Very fast (C core, Python bindings via `mujoco`)
#    - Used in DeepMind Control Suite, OpenAI Gym, etc.
#
# ============================================================

import mujoco
import mujoco.viewer   # interactive 3-D viewer (optional)
import numpy as np
import time

print("=" * 55)
print("  MuJoCo Demo  —  Koji's Robotics Playground")
print("=" * 55)
print(f"\n  MuJoCo version: {mujoco.__version__}")

# ----------------------------------------------------------
# 1. DEFINE A ROBOT IN MJCF (MuJoCo XML format)
#
#  We build a simple 2-link robot arm (like a human elbow):
#    - Link 1 ("upper_arm"): attached to world, hinge joint
#    - Link 2 ("forearm"):   attached to link1, another hinge
#
#  MJCF key concepts:
#    <worldbody>     → root of the kinematic tree
#    <body>          → a rigid link
#    <joint>         → connects parent body to child body
#    <geom>          → visual + collision shape
#    <actuator>      → motors that drive the joints
#    <sensor>        → sensors that measure things
# ----------------------------------------------------------
XML = """
<mujoco model="two_link_arm">

  <!-- Physics options: timestep=2ms, gravity downward -->
  <option timestep="0.002" gravity="0 0 -9.81"/>

  <!-- Default geometry settings -->
  <default>
    <geom rgba="0.4 0.6 0.9 1"/>
  </default>

  <worldbody>

    <!-- Ground plane -->
    <geom name="floor" type="plane" size="2 2 0.1" rgba="0.8 0.8 0.8 1"/>

    <!-- BASE (fixed to world) -->
    <body name="base" pos="0 0 0.05">
      <geom name="base_geom" type="cylinder" size="0.06 0.05" rgba="0.3 0.3 0.3 1"/>

      <!-- UPPER ARM — rotates around Z axis (horizontal swing) -->
      <body name="upper_arm" pos="0 0 0.05">
        <joint name="shoulder" type="hinge" axis="0 0 1"
               range="-180 180" damping="0.5"/>
        <geom name="upper_geom" type="capsule"
              fromto="0 0 0  0.4 0 0" size="0.025" rgba="0.2 0.7 0.4 1"/>

        <!-- FOREARM — rotates around Z axis (elbow bend) -->
        <body name="forearm" pos="0.4 0 0">
          <joint name="elbow" type="hinge" axis="0 0 1"
                 range="-170 170" damping="0.3"/>
          <geom name="fore_geom" type="capsule"
                fromto="0 0 0  0.3 0 0" size="0.02" rgba="0.9 0.5 0.2 1"/>

          <!-- END EFFECTOR (tip of the arm) -->
          <site name="tip" pos="0.3 0 0" size="0.015" rgba="1 0 0 1"/>
        </body>
      </body>
    </body>

  </worldbody>

  <!-- ACTUATORS: position servos on each joint -->
  <actuator>
    <position name="shoulder_motor" joint="shoulder" kp="100" ctrlrange="-3.14 3.14"/>
    <position name="elbow_motor"    joint="elbow"    kp="80"  ctrlrange="-3.0  3.0"/>
  </actuator>

  <!-- SENSORS: measure joint angles and end-effector position -->
  <sensor>
    <jointpos  name="shoulder_angle" joint="shoulder"/>
    <jointpos  name="elbow_angle"    joint="elbow"/>
    <jointvel  name="shoulder_vel"   joint="shoulder"/>
    <jointvel  name="elbow_vel"      joint="elbow"/>
    <framepos  name="tip_pos"        objtype="site" objname="tip"/>
  </sensor>

</mujoco>
"""

# ----------------------------------------------------------
# 2. LOAD THE MODEL
# ----------------------------------------------------------
model = mujoco.MjModel.from_xml_string(XML)
data  = mujoco.MjData(model)      # data holds the simulation STATE

print(f"\n[Model]   Loaded: '{model.name}'")
print(f"          Bodies  : {model.nbody}")
print(f"          Joints  : {model.njnt}")
print(f"          Actuators: {model.nu}")
print(f"          Sensors : {model.nsensor}")

# ----------------------------------------------------------
# 3. PRINT JOINT & ACTUATOR NAMES
# ----------------------------------------------------------
print("\n[Joints]")
for i in range(model.njnt):
    print(f"  {i}: {model.joint(i).name}")

print("\n[Actuators]")
for i in range(model.nu):
    print(f"  {i}: {model.actuator(i).name}")

# ----------------------------------------------------------
# 4. HELPER: read all sensor data by name
# ----------------------------------------------------------
def read_sensor(name):
    sid   = model.sensor(name).id
    start = model.sensor_adr[sid]
    dim   = model.sensor_dim[sid]
    return data.sensordata[start : start + dim].copy()

# ----------------------------------------------------------
# 5. DEFINE A SIMPLE MOTION SEQUENCE
#    Each entry: (shoulder_rad, elbow_rad, hold_seconds)
# ----------------------------------------------------------
motion_sequence = [
    ( 0.0,   0.0,  0.5),   # home position
    ( 1.0,  -1.5,  1.0),   # reach right and bend elbow
    (-1.0,  -1.0,  1.0),   # swing left
    ( 0.5,   0.8,  1.0),   # another pose
    ( 0.0,   0.0,  0.5),   # return home
]

print("\n[Sim]  Running motion sequence...\n")
print(f"  {'Time(s)':>8}  {'Shoulder(°)':>12}  {'Elbow(°)':>10}  {'Tip X':>8}  {'Tip Y':>8}  {'Tip Z':>8}")
print("  " + "-" * 65)

sim_time   = 0.0
step_count = 0

for (shoulder_target, elbow_target, hold_sec) in motion_sequence:

    # Set actuator commands (position targets in radians)
    data.ctrl[0] = shoulder_target
    data.ctrl[1] = elbow_target

    steps = int(hold_sec / model.opt.timestep)

    for _ in range(steps):
        mujoco.mj_step(model, data)    # advance physics by one timestep
        sim_time   += model.opt.timestep
        step_count += 1

    # Read sensors after settling
    s_angle = read_sensor("shoulder_angle")[0]
    e_angle = read_sensor("elbow_angle")[0]
    tip_pos = read_sensor("tip_pos")

    print(f"  {sim_time:>8.3f}  "
          f"{np.degrees(s_angle):>11.2f}°  "
          f"{np.degrees(e_angle):>9.2f}°  "
          f"{tip_pos[0]:>8.4f}  "
          f"{tip_pos[1]:>8.4f}  "
          f"{tip_pos[2]:>8.4f}")

print(f"\n[Sim]  Done. Total steps: {step_count}  |  Simulated time: {sim_time:.3f}s")

# ----------------------------------------------------------
# 6. FORWARD KINEMATICS CHECK
#    MuJoCo's mj_kinematics() updates positions without
#    stepping the physics. Useful for FK-only queries.
# ----------------------------------------------------------
print("\n[FK Check]  Setting joint angles directly and computing tip position...")
data.qpos[0] = np.radians(45)   # shoulder = 45°
data.qpos[1] = np.radians(-90)  # elbow    = -90°
mujoco.mj_kinematics(model, data)   # update positions without dynamics

tip_id  = model.site("tip").id
tip_xyz = data.site_xpos[tip_id]
print(f"  Shoulder=45°, Elbow=-90° → Tip = ({tip_xyz[0]:.4f}, {tip_xyz[1]:.4f}, {tip_xyz[2]:.4f})")

# ----------------------------------------------------------
# 7. OPEN THE INTERACTIVE VIEWER (optional — comment out
#    if you are running headless / on a server)
# ----------------------------------------------------------
# Uncomment the block below to open the 3-D viewer:
#
# print("\n[Viewer]  Opening interactive viewer. Close window to exit.")
# with mujoco.viewer.launch_passive(model, data) as viewer:
#     for _ in range(5000):
#         mujoco.mj_step(model, data)
#         viewer.sync()
#         time.sleep(model.opt.timestep)

print("\n" + "=" * 55)
print("  MuJoCo demo complete!")
print("  Next steps:")
print("    • Uncomment the viewer block to see the 3-D robot")
print("    • Try dm_control for RL environments on top of MuJoCo")
print("    • pip install dm-control gymnasium[mujoco]")
print("=" * 55)
