import mujoco as mj
import numpy as np

# Load the model
import time
 
import mujoco
import mujoco.viewer
 
#m = mujoco.MjModel.from_xml_path('test_pb/hopper.xml')
xml = """
<mujoco>
  <worldbody>
    <geom name="red_box" type="box" size=".2 .2 .2" rgba="1 0 0 1"/>
    <geom name="green_sphere" pos=".2 .2 .2" size=".1" rgba="0 1 0 1"/>
  </worldbody>
</mujoco>
"""
#m = mujoco.MjModel.from_xml_string(xml)
#command to load the model from xml file 
#m = mujoco.MjModel.from_xml_path('test_pb/hopper_gemini.xml')
#m = mujoco.MjModel.from_xml_path('test_pb/hopper_mujoco.xml')
m = mujoco.MjModel.from_xml_path('test_pb/monoped_hopper_inspired.xml')
d = mujoco.MjData(m)
 
with mujoco.viewer.launch_passive(m, d) as viewer:
  # Close the viewer automatically after 30 wall-seconds.
  start = time.time()
  while viewer.is_running() and time.time() - start < 30:
    step_start = time.time()
 
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
