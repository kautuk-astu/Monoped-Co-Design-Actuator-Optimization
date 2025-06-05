import numpy as np
import mujoco as mj
import vmc_mujoco as vmc
import time
import mujoco.viewer as mj_viewer
import len_rad_id as lri
import vmc_hopper_roc as vmc_hr
import vmc_roc_damper as vmc_rd
import run_ctrl_para as rcp

xml_path = 'hopper_roc.xml'
m = mj.MjModel.from_xml_path('hopper_roc.xml')
d = mj.MjData(m)

lin_kp_rise = 42.39
lin_kd_rise = 0.46
lin_kp_fall = 2.4
lin_kd_fall = 7.37
rot_kp_rise = 1.23
rot_kp_fall = 0.82
#30, 0, 1, 0, 10, 1 gives 0.16 m max height 
action = np.array([lin_kp_rise, lin_kd_rise, lin_kp_fall, lin_kd_fall, rot_kp_rise, rot_kp_fall])
base_height_store, height_reward, power_store = rcp.run(xml_path, m, d, action)
base_max = np.max(base_height_store)
print("base_max:", base_max)
height_reward_max = np.min(height_reward)
print("height_reward:", height_reward_max) 

#CODE WORKS FOR SPECIFICALLY CHOSEN VALUES OF ACTION
