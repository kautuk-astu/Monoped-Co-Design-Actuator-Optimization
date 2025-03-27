import numpy as np
import csv
import mujoco as mj
import vmc_mujoco as vmc
import time
import run_ctrl_para as rcp

# File to store results
csv_filename = "ifelse_1202_2020_5.csv"
with open("ifelse_1202_2020_5.csv", "w") as file:
    pass  # Opens in write mode and immediately closes, clearing the content


# Define parameter ranges using linspace
lin_kp_rise_range = np.linspace(5, 100, 5)  # 5 evenly spaced values from 10 to 50
lin_kp_fall_range = np.linspace(0, 5, 5)
lin_kd_rise_range = np.linspace(0, 5, 5)
lin_kd_fall_range = np.linspace(5, 30, 5)
rot_kp_rise_range = np.linspace(5, 30, 5)
rot_kp_fall_range = np.linspace(0, 5, 5)


best_reward = 100
best_params = None

xml_path = 'hopper_roc.xml'
m = mj.MjModel.from_xml_path('hopper_roc.xml')
d = mj.MjData(m)

# Open CSV file to save results
with open(csv_filename, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["lin_kp_rise", "lin_kd_rise", "lin_kp_fall","lin_kd_fall", "rot_kp_rise", "rot_kp_fall"])  # CSV Headers
    for kp_rise in lin_kp_rise_range:
        for kp_fall in lin_kp_fall_range:
            for kd_rise in lin_kd_rise_range:
                for kd_fall in lin_kd_fall_range:
                    for kp_rise_rot in rot_kp_rise_range:
                        for kp_fall_rot in rot_kp_fall_range:
                            action = np.array([kp_rise, kd_rise, kp_fall, kd_fall, kp_rise_rot, kp_fall_rot])
                            base_height_store, base_vel_store, height_reward = rcp.run(xml_path, m, d, action)
                            base_max = np.max(base_height_store)
                            height_reward_max = np.max(height_reward)
                            writer.writerow([kp_rise, kp_fall, kd_rise, kd_fall, kp_rise_rot, kp_fall_rot, height_reward_max, base_max])
                            print(f"kp_rise: {kp_rise}, kp_fall: {kp_fall}, kd_rise: {kd_rise}, kd_fall: {kd_fall}, kp_rise_rot: {kp_rise_rot}, kp_fall_rot: {kp_fall_rot}, reward: {height_reward_max}")
                            if height_reward_max > best_reward:
                                best_reward = height_reward_max
                                best_params = action       
           

print(f"Best reward: {best_reward}, Best parameters: {best_params}")
print(f"Results saved to {csv_filename}")
