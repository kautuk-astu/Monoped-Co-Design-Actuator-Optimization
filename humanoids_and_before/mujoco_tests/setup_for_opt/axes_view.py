import mujoco
import mujoco.viewer as mujoco_viewer
import numpy as np
import time

# Define the XML model for the cuboidal base with the sliding joint and RK4 solver
xml_model = """
<mujoco model="cuboidal_base_sliding_z">
    <compiler angle="degree" coordinate="local"/>
    <option gravity="0 0 -9.81" timestep="0.01">
    </option>

    <default>
        <joint armature="0.01" damping="1" stiffness="0"/>
        <geom size="0.1 0.1 0.1" friction="1 0.5 0.5"/>
    </default>

    <worldbody>
        <!-- Ground Plane -->
        <geom name="x_axis" type="capsule" size="0.01 0.3" pos="0.15 0 0" euler="0 0 1.57" rgba="1 0 0 1" />
        <geom name="y_axis" type="capsule" size="0.01 0.3" pos="0 0.15 0" euler="0 1.57 0" rgba="0 1 0 1" />
        <geom name="z_axis" type="capsule" size="0.01 0.3" pos="0 0 0" euler="0 0 0" rgba="0 0 1 1" />

    </worldbody>
</mujoco>
"""

# Load the model from the XML string
m = mujoco.MjModel.from_xml_string(xml_model)
#sim = mujoco_py.MjSim(m)

d = mujoco.MjData(m)
#joint_id = m.joint_name2id("slide_z")
# joint_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "slide_z")
# print("joint_id:", joint_id)
# force_index = m.jnt_dofadr[joint_id]


 
with mujoco.viewer.launch_passive(m, d) as viewer:
  # Close the viewer automatically after 30 wall-seconds.
  start = time.time()
  while viewer.is_running() and time.time() - start < 30:
    step_start = time.time()
    #d.qfrc_applied[force_index] = 100
 
    # mj_step can be replaced with code that also evaluates
    # a policy and applies a control signal before stepping the physics.
    mujoco.mj_step(m, d)
 
    # Example modification of a viewer option: toggle contact points every two seconds.
    with viewer.lock():
      viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(d.time % 2)
 
    # Pick up changes to the physics state, apply perturbations, update options from GUI.
    viewer.sync()
 
    # Rudimentary time keeping, will drift relative to wall clock.
    time_until_next_step = m.opt.timestep - (time.time() - step_start)
    if time_until_next_step > 0:
      time.sleep(time_until_next_step)
