import mujoco
import mujoco.viewer as mujoco_viewer
import numpy as np
import time

# Define the XML model for the cuboidal base with the sliding joint and RK4 solver
xml_model = """
<mujoco model="cuboidal_base_sliding_z">
    <compiler angle="radian" coordinate="local"/>
    <option gravity="0 0 0" timestep="0.01">
    </option>

    <equality>
        <weld body1="slot" />
        <!-- <weld body2="base" /> -->
    </equality>


    <asset>
        <material name="collision" rgba="0.1 0.5 0.9 0" />           
        <material name="obstacle" rgba="0.2 0.6 0.2 1" />   

        
        <mesh name="slot" file="/home/stochlab/Aastha_pybullet/test_pb/meshes/meshes/slot_link.STL" scale="1 1 1" />
        <mesh name="base" file="/home/stochlab/Aastha_pybullet/test_pb/meshes/meshes/base_Link.STL" scale="1 1 1" />
        <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="512" />
        <texture name="texplane" type="2d" builtin="checker" rgb1=".2 .3 .4" rgb2=".1 0.15 0.2" width="512" height="512" mark="cross" markrgb=".8 .8 .8" />
        <texture name="texgeom" type="cube" builtin="flat" mark="cross" width="127" height="1278" rgb1="0.8 0.6 0.4" rgb2="0.8 0.6 0.4" markrgb="1 1 1" random="0.01" />
        <material name="matplane" reflectance="0.3" texture="texplane" texrepeat="1 1" texuniform="true" />
        <material name="matgeom" texture="texgeom" texuniform="true" rgba="0.8 0.6 .4 1" />
    </asset>

    <actuator>
        <position name="slide_z" kp="0" joint="slide_z" />
        <motor name="FL_hip_joint" joint="FL_hip_joint" ctrllimited="true" ctrlrange="-6 6" gear="1" forcelimited = "true" forcerange="-6 6"/>
    </actuator>




    <default>
        <joint limited="true" damping="1" />
        <geom contype="0" conaffinity="0" condim="1" solref="0.005 1" />
        <!-- <motor forcelimited="true" forcerange="-6 6" ctrllimited="true" ctrlrange="-6 6" gear="1" />         -->
        <equality solref="0.005 1" />
        <default class="collision">
            <geom material="collision" contype="1" user="2" />
            <default class="collision-left">
                <geom contype="2" conaffinity="4" />
            </default>
            <default class="collision-right">
                <geom contype="4" conaffinity="2" />
            </default>
        </default>
        <default class="obstacle">
            <geom material="obstacle" condim="3" contype="8" conaffinity="15" user="1" />
        </default>
    </default>

    <worldbody>
        <geom name="floor" pos="0 0 0" size="0 0 .25" type="plane" material="matplane" condim="3" conaffinity="15" friction="1 1 1" />
	    <light directional="false" diffuse=".2 .2 .2" specular="0 0 0" pos="0 0 5" dir="0 0 -1" castshadow="false" />      
        
        <light mode="targetbodycom" target="slot" directional="false" diffuse=".8 .8 .8" specular="0.3 0.3 0.3" pos="0 0 4.0" dir="0 0 -1" />
        
        <body name="slot" pos="0 0 0.34" euler="0 0 0">
		<freejoint name="root" />
            <inertial pos="1.4485E-05 0 0.16127" quat="0 0 0 1" mass="3.00" diaginertia="0.044062 0.044343 0.00032708" />
            <geom type="mesh" contype="0" conaffinity="0" group="1" rgba="1 0.423529 0.0392157 1" mesh="slot" quat="0 0 0 1" />
            <geom class="collision" size="0.105 0.195 0.035" pos="0 -0.005 0" type="box" rgba="1 0.423529 0.0392157 0" group="2" />

            <body name="base" pos="0 -0.108 0.054" euler="0 0 3.14">
                <inertial pos="-1.9883E-05 -0.056007 -0.027565" quat="0 0 0 1" mass="0.800" diaginertia="0.00035155 0.0016709 0.0013474" />
                <joint name="slide_z" pos="0 0 0" type="slide" axis="0 0 1" limited="true" range="-0.35 0.05" />
                <geom type="mesh" contype="0" conaffinity="0" group="1" rgba="0.0 0.0 0.0 1" mesh="base" />
                <geom class="collision" size="0.0235 0.082 0.022" pos="0.025 -0.072 0" quat="0 0 0 1" type="box" rgba="0.2 0 0 0" group="2" />
                
                <body name="FL_hip" pos="0 -0.1 -0.19" euler="0 0 0">
                    <inertial pos="-2.498E-14 -0.070952 -0.079802" quat="0 0 0 1" mass="0.080" diaginertia="7.7478E-05 7.7478E-05 4.3389E-06" />
                    <joint name="FL_hip_joint" pos="0 0 0" type ="hinge" axis="0 1 0" limited="true" range="0 1.57" />
                    <geom type="capsule" size = "0.01 0.1" contype="0" conaffinity="0" group="1" rgba="1.0 0.27 0.0 1" />
                    <geom class="collision" size="0.0235 0.082 0.172" pos="0.025 -0.072 0" quat="0 0 0 0.5" type="box" rgba="0.2 0 0 0" group="2" />
                    
                    
                </body>
                

            </body>
        </body>
    </worldbody>
</mujoco>
"""

# Load the model from the XML string
#m = mujoco.MjModel.from_xml_string(xml_model)
m = mujoco.MjModel.from_xml_path('test_pb/monoped_hopper_inspired.xml')

#load the model from the xml file
#m = mujoco.MjModel.from_xml_path('test_pb/single_leg.xml')
#sim = mujoco_py.MjSim(m)

d = mujoco.MjData(m)

#joint_id = m.joint_name2id("slide_z")
joint_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "rootz")
force_index = m.jnt_dofadr[joint_id]
joint_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "thigh_joint")
knee_force_index = m.jnt_dofadr[joint_id]
total_forces = d.qfrc_passive + d.qfrc_actuator + d.qfrc_applied

# joint_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "FL_hip_joint")
# force_index = m.jnt_dofadr[joint_id]
# knee_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "FL_thigh_joint")
# knee_force_index = m.jnt_dofadr[knee_id]
# slide_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "slide_z")
# slide_force_index = m.jnt_dofadr[slide_id]
# print("joint_id:", joint_id)
# force_index = m.jnt_dofadr[joint_id]


def get_ground_contact_forces(model, data):
  """
  Extracts ground contact forces from MuJoCo simulation data.

  Args:
    model: MuJoCo model object.
    data: MuJoCo data object.

  Returns:
    A list of contact forces, where each element is a tuple 
    containing the contact force vector and the corresponding 
    body index.
  """

  ground_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "slot_geom")  # Assuming "floor" is the ground geom
  ground_contact_forces = []

  for i in range(data.ncon):
    con = data.contact[i]
    if con.geom1 == ground_id or con.geom2 == ground_id:
      # Calculate the contact force vector
      force = np.zeros(6, dtype=np.float64)
      mujoco.mj_contactForce(model, data, i, force) 

      # Extract linear force (first 3 elements)
      contact_force = force[:3] 

      # Get the body index 
      body_id = con.geom1 if con.geom1 != ground_id else con.geom2

      ground_contact_forces.append((contact_force, body_id))

  return ground_contact_forces

# Example usage:



 
with mujoco.viewer.launch_passive(m, d) as viewer:
  # Close the viewer automatically after 30 wall-seconds.
  start = time.time()
  while viewer.is_running() and time.time() - start < 30:
    step_start = time.time()
    print("slider force:", d.qfrc_applied[force_index])

# Define the desired torque
    continuous_torque = 107 # Example: Apply a continuous torque of 0.5 Nm

# Simulation loop # Simulate for 100 steps
    # Apply torque continuously within the loop
    d.qfrc_applied[force_index] = continuous_torque 
    #d.qfrc_applied[knee_force_index] = continuous_torque
    #d.qfrc_applied[slide_id] = continuous_torque
    #d.qfrc_applied[slide_force_index] = 0.5
    #d.qfrc_applied[force_index] = 100
 
    # mj_step can be replaced with code that also evaluates
    # a policy and applies a control signal before stepping the physics.
    mujoco.mj_step(m, d)

    ground_forces = get_ground_contact_forces(m, d)

# Print the contact forces (optional)
    for force, body_id in ground_forces:
        print(f"Contact force on body {body_id}: {force}")
 
    # Example modification of a viewer option: toggle contact points every two seconds.
    with viewer.lock():
      viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(d.time % 2)
 
    # Pick up changes to the physics state, apply perturbations, update options from GUI.
    viewer.sync()
 
    # Rudimentary time keeping, will drift relative to wall clock.
    time_until_next_step = m.opt.timestep - (time.time() - step_start)
    if time_until_next_step > 0:
      time.sleep(time_until_next_step)