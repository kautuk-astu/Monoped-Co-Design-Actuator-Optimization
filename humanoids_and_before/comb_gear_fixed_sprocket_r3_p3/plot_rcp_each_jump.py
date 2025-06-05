import numpy as np
import mujoco as mj
import time
import len_rad_id as lri
import vmc_roc_action as vmc_ra
import ik as ik
import random
import csv
import matplotlib.pyplot as plt

np.random.seed(0)
random.seed(0)

def run(xml_path, action, mass, ik_value):
    m = mj.MjModel.from_xml_path(xml_path)
    d = mj.MjData(m)
    mj.mj_setTotalmass(m, mass)

    mj.mj_resetData(m, d)
    m.opt.iterations = 500  
    m.opt.tolerance = 1e-10

    total_energy = 0
    hip_energy = 0
    knee_energy = 0
    jump_energy = 0
    hip_energy_jump = 0
    knee_energy_jump = 0
    jump_results = []

    current_jump_max = float('-inf')
    max_height_after_control = float('-inf')
    jump_count = 0
    in_air = False
    hip_actuator_id = m.actuator('torque1').id    
    knee_actuator_id = m.actuator('torque2').id

    controller = vmc_ra.Controller(xml_path, m, d)
    lp = lri.len_rad_id(xml_path, m, d)
    l1 = lp.thigh_total_length()
    l2 = lp.calf_total_length()
    theta1, theta2 = ik.inverse_kinematics(ik_value, 0, l1, l2)

    torque1_store = []
    torque2_store = []
    torque1_from_start = []
    torque2_from_start = []

    # Data logs for plotting
    time_log = []
    base_height_log = []
    base_velocity_log = []
    joint1_pos_log = []
    joint2_pos_log = []
    joint1_vel_log = []
    joint2_vel_log = []

    d.qpos[1] = theta1
    d.qpos[2] = theta2
    d.qvel[:] = 0

    start = time.time()
    while True:
        step_start = time.time()
        t_elapsed = time.time() - start

        

        if t_elapsed < 3:
            hip_error = theta1 - d.qpos[1]
            knee_error = theta2 - d.qpos[2]
            hip_torque = 100 * hip_error - 10 * d.qvel[1]
            knee_torque = 100 * knee_error - 10 * d.qvel[2]
            
        else:
            contact_force = controller.get_ground_contact_forces()
            forces = np.array([0, 0, 0])  
            if len(contact_force) > 0:
                forces = contact_force[0][0]

            if np.all(forces == 0):  # in air
                hip_error = theta1 - d.qpos[1]
                knee_error = theta2 - d.qpos[2]
                hip_torque = 100 * hip_error - 10 * d.qvel[1]
                knee_torque = 100 * knee_error - 10 * d.qvel[2]

                if not in_air:
                    in_air = True
                    current_jump_max = d.qpos[0]
                    jump_energy = 0
                    hip_energy_jump = 0 
                    knee_energy_jump = 0
            else:  # on ground
                if in_air:
                    in_air = False
                    jump_count += 1
                    jump_results.append((current_jump_max, jump_energy, hip_energy_jump, knee_energy_jump))
                    max_height_after_control = max(max_height_after_control, current_jump_max)

                    if jump_count == 2:
                        break

                controller.lin_sd_var(action)
                controller.rot_sd_var(action)
                controller.linear_spring_force(action)
                controller.linear_damper_force(action)
                controller.torsional_spring_force(action)
                controller.total_linear_force(action)
                controller.fz_by_ee(action)
                controller.fx_by_ee(action)
                controller.force_applied_ground(action)
                joint_torque = controller.joint_torque(action)
                hip_torque = np.clip(joint_torque[0], -30.25, 30.25)
                knee_torque = np.clip(joint_torque[1], -64.49828, 64.49828)

        d.ctrl[hip_actuator_id] = hip_torque
        d.ctrl[knee_actuator_id] = knee_torque
        torque1_from_start.append(hip_torque)
        torque2_from_start.append(knee_torque)
        # torque1_store.append(hip_torque)
        # torque2_store.append(knee_torque)

        if t_elapsed >= 3:
            torque1_store.append(hip_torque)
            torque2_store.append(knee_torque)
            hip_power = hip_torque * d.qvel[1]
            knee_power = knee_torque * d.qvel[2]

            if hip_power > 0:
                e_hip = hip_power * m.opt.timestep
                hip_energy += e_hip
                total_energy += e_hip
                hip_energy_jump += e_hip
                jump_energy += e_hip

            if knee_power > 0:
                e_knee = knee_power * m.opt.timestep
                knee_energy += e_knee
                total_energy += e_knee
                knee_energy_jump += e_knee
                jump_energy += e_knee
            time_log.append(t_elapsed)
            base_height_log.append(d.qpos[0])
            base_velocity_log.append(d.qvel[0])
            joint1_pos_log.append(d.qpos[1])
            joint2_pos_log.append(d.qpos[2])
            joint1_vel_log.append(d.qvel[1])
            joint2_vel_log.append(d.qvel[2])

        mj.mj_step(m, d)

        if t_elapsed >= 3 and in_air:
            current_jump_max = max(current_jump_max, d.qpos[0])

        time_until_next_step = m.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)

    if in_air:
        jump_results.append((current_jump_max, jump_energy, hip_energy_jump, knee_energy_jump))
        max_height_after_control = max(max_height_after_control, current_jump_max)

    return (
        max_height_after_control, jump_count, total_energy, hip_energy, knee_energy, jump_results,
        time_log, base_height_log, base_velocity_log,
        joint1_pos_log, joint2_pos_log, joint1_vel_log, joint2_vel_log,
        torque1_store, torque2_store, torque1_from_start, torque2_from_start
    )

# Run simulation
#xml_path = 'your_path_here.xml'
xml_path = '/home/stochlab/repo/Aastha_Coopt_Monoped/Monoped-optimization/joint_cma/comb_gear_fixed_sprocket_r3_p3/modified_robot_437_346_55_86.xml'
action = np.array([62.3, 2.6, 12.3])
result = run(xml_path, action, 4.492614814814815, 0.6)

(max_height_after_control, jump_count, total_energy, hip_energy, knee_energy, jump_results,
 time_log, base_height_log, base_velocity_log,
 joint1_pos_log, joint2_pos_log, joint1_vel_log, joint2_vel_log,
 torque1_store, torque2_store, torque1_from_start, torque2_from_start) = result

print(f"Max Height After Control: {max_height_after_control:.3f}")
print(f"Jump Count: {jump_count}")
for i, (h, e_total, e_hip, e_knee) in enumerate(jump_results):
    print(f"Jump {i+1}: Height = {h:.3f}, Total Energy = {e_total:.2f}, Hip = {e_hip:.2f}, Knee = {e_knee:.2f}")

# Save CSV log
with open("jump_logs.csv", "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["Time", "Base Height", "Base Velocity", "Hip Pos", "Knee Pos", "Hip Vel", "Knee Vel", "Hip Torque", "Knee Torque"])
    for i in range(len(time_log)):
        writer.writerow([
            time_log[i], base_height_log[i], base_velocity_log[i],
            joint1_pos_log[i], joint2_pos_log[i],
            joint1_vel_log[i], joint2_vel_log[i],
            torque1_store[i], torque2_store[i]
        ])
with open("torque_from_start.csv", "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["Time", "Hip Torque", "Knee Torque"])
    for i in range(len(torque1_from_start)):
        writer.writerow([i, torque1_from_start[i], torque2_from_start[i]])

import numpy as np
#Find indices where height > -0.4 (indicating potential take-off)
threshold_height = -0.53
land_height = -0.5

jump_indices = []
in_jump = False

for i in range(1, len(base_height_log)):
    # Takeoff detection (relaxed velocity constraint)
    if (not in_jump and
        base_height_log[i-1] < threshold_height and 
        base_height_log[i] >= threshold_height):
        start_idx = i
        in_jump = True

    # Landing detection
    elif in_jump and base_height_log[i] <= land_height:
        end_idx = i
        jump_indices.append((start_idx, end_idx))
        in_jump = False

# Plot second jump if available
if len(jump_indices) >= 2:
    jump_start_idx, jump_end_idx = jump_indices[1]  # second jump

    time_crop = time_log[jump_start_idx:jump_end_idx + 1]
    height_crop = base_height_log[jump_start_idx:jump_end_idx + 1]

    plt.figure()
    plt.plot(time_crop, height_crop)
    plt.xlabel("Time (s)")
    plt.ylabel("Base Height (m)")
    plt.title("Base Height During Second Jump (Start to Landing)")
    plt.grid(True)
    plt.show()
else:
    print("Second jump not found. Detected jumps:", len(jump_indices))

# 5.088474273681641,-0.3954138231311631,2.9255009745035703,-0.4697329048719946,1.4076015569362843,4.094627786603956,-12.909734556794776,-54.84506046225475,158.39305807829604
# 5.090680122375488,-0.3898387156524918,2.6550388398862905,-0.4626647942986086,1.3911341484766784,2.9888464865243325,-3.732110565757253,-44.49405851879712,68.26355901388138
# 9.39473819732666,-0.39740094989982544,3.2799200072153405,-0.5145393625137749,1.4242546011010115,4.657248380390446,-14.053939172427981,-55.990620635941625,168.16979981815535
# 9.39694857597351,-0.39113196541494943,2.994829582619335,-0.5063900224658032,1.4056409123630669,3.5078041694435766,-4.735338851125915,-45.31111253127009,76.84516547892916
# threshold_height = -0.49
# land_height = -0.5

# # Track multiple jumps
# jump_indices = []
# in_jump = False

# # Find all jump start and end indices
# for i in range(1, len(base_height_log)):
#     # Detect takeoff
#     if (not in_jump and
#         base_height_log[i-1] < threshold_height and 
#         base_height_log[i] >= threshold_height and 
#         base_velocity_log[i] > 0.5):
#         start_idx = i
#         in_jump = True

#     # Detect landing
#     elif in_jump and base_height_log[i] <= land_height:
#         end_idx = i
#         jump_indices.append((start_idx, end_idx))
#         in_jump = False

# # Check if we have at least two jumps
# if len(jump_indices) >= 2:
#     jump_start_idx, jump_end_idx = jump_indices[1]  # second jump

#     time_crop = time_log[jump_start_idx:jump_end_idx + 1]
#     height_crop = base_height_log[jump_start_idx:jump_end_idx + 1]

#     # Plot second jump
#     plt.figure()
#     plt.plot(time_crop, height_crop)
#     plt.xlabel("Time (s)")
#     plt.ylabel("Base Height (m)")
#     plt.title("Base Height During Second Jump (Start to Landing)")
#     plt.grid(True)
#     plt.show()
# else:
#     print("Second jump not found.")


# Parameters
# height_threshold = -0.5
# velocity_threshold = 0.5
# timestep = 0.002  # Adjust this based on your simulation timestep
# stable_window = int(0.1 / timestep)  # approx 0.1s window

# start_idx = None
# peak_idx = None
# end_idx = None
# found_first_below = False

# # Step 1: Find start of second jump
# for i in range(len(base_height_log)):
#     if base_height_log[i] < height_threshold:
#         found_first_below = True
#         start_idx = i
#         break

# # Step 2: Find local maximum height after start
# if start_idx is not None:
#     peak_height = base_height_log[start_idx]
#     for i in range(start_idx + 1, len(base_height_log)):
#         if base_height_log[i] > peak_height:
#             peak_height = base_height_log[i]
#             peak_idx = i
#         else:
#             # We assume peak is reached once height stops increasing
#             break

# # Step 3: Detect end of jump (stabilization)
# if peak_idx is not None:
#     for i in range(peak_idx + 1, len(base_height_log) - stable_window):
#         stable = True
#         for j in range(stable_window):
#             if abs(base_velocity_log[i + j]) > velocity_threshold:
#                 stable = False
#                 break
#         if stable:
#             end_idx = i + stable_window
#             break

# # Plot if everything is found
# if start_idx is not None and end_idx is not None:
#     t_jump = time_log[start_idx:end_idx]
#     qpos1_jump = joint1_pos_log[start_idx:end_idx]
#     qpos2_jump = joint2_pos_log[start_idx:end_idx]
#     qvel1_jump = joint1_vel_log[start_idx:end_idx]
#     qvel2_jump = joint2_vel_log[start_idx:end_idx]
#     base_pos_jump = base_height_log[start_idx:end_idx]
#     base_vel_jump = base_velocity_log[start_idx:end_idx]
#     torque1_jump = torque1_store[start_idx:end_idx]
#     torque2_jump = torque2_store[start_idx:end_idx]

#     # Save to CSV
#     import pandas as pd
#     df_jump = pd.DataFrame({
#         "time": t_jump,
#         "qpos1": qpos1_jump,
#         "qpos2": qpos2_jump,
#         "qvel1": qvel1_jump,
#         "qvel2": qvel2_jump,
#         "torque1": torque1_jump,
#         "torque2": torque2_jump,
#         "base_pos": base_pos_jump,
#         "base_vel": base_vel_jump,
#     })
#     df_jump.to_csv("second_jump_log.csv", index=False)

#     # Plotting
#     import matplotlib.pyplot as plt

#     plt.figure(figsize=(10, 6))
#     plt.plot(t_jump, base_pos_jump)
#     plt.xlabel("Time (s)")
#     plt.ylabel("Base Height (m)")
#     plt.title("Base Height During Second Jump")
#     plt.grid()
#     plt.show()

#     # Additional plots as needed...

# else:
#     print("Could not identify the full second jump range.")



# start_idx = None
# end_idx = None
# found_first_below = False

# for i in range(len(base_height_log)):
#     if base_height_log[i] < -0.5:
#         if not found_first_below:
#             found_first_below = True
#             start_idx = i
#     elif found_first_below and base_height_log[i] >= -0.5:
#         end_idx = i
#         break

# if start_idx is not None and end_idx is not None:
#     # Extract sliced logs for second jump
#     t_jump = time_log[start_idx:end_idx]
#     qpos1_jump = joint1_pos_log[start_idx:end_idx]
#     qpos2_jump = joint2_pos_log[start_idx:end_idx]
#     qvel1_jump = joint1_vel_log[start_idx:end_idx]
#     qvel2_jump = joint2_vel_log[start_idx:end_idx]
#     base_pos_jump = base_height_log[start_idx:end_idx]
#     base_vel_jump = base_velocity_log[start_idx:end_idx]
#     torque1_jump = torque1_store[start_idx:end_idx]
#     torque2_jump = torque2_store[start_idx:end_idx]

#     # Plotting
#     import matplotlib.pyplot as plt

#     plt.figure(figsize=(10, 6))
#     plt.plot(t_jump, qpos1_jump, label="Hip Angle")
#     plt.plot(t_jump, qpos2_jump, label="Knee Angle")
#     plt.xlabel("Time (s)")
#     plt.ylabel("Joint Angles (rad)")
#     plt.title("Joint Angles During Second Jump")
#     plt.legend()
#     plt.grid()
#     plt.show()

#     plt.figure(figsize=(10, 6))
#     plt.plot(t_jump, qvel1_jump, label="Hip Velocity")
#     plt.plot(t_jump, qvel2_jump, label="Knee Velocity")
#     plt.xlabel("Time (s)")
#     plt.ylabel("Joint Velocities (rad/s)")
#     plt.title("Joint Velocities During Second Jump")
#     plt.legend()
#     plt.grid()
#     plt.show()

#     plt.figure(figsize=(10, 6))
#     plt.plot(t_jump, torque1_jump, label="Hip Torque")
#     plt.plot(t_jump, torque2_jump, label="Knee Torque")
#     plt.xlabel("Time (s)")
#     plt.ylabel("Torque (Nm)")
#     plt.title("Joint Torques During Second Jump")
#     plt.legend()
#     plt.grid()
#     plt.show()

#     plt.figure(figsize=(10, 6))
#     plt.plot(t_jump, base_pos_jump)
#     plt.xlabel("Time (s)")
#     plt.ylabel("Base Height (m)")
#     plt.title("Base Height During Second Jump")
#     plt.grid()
#     plt.show()

#     plt.figure(figsize=(10, 6))
#     plt.plot(t_jump, base_vel_jump)
#     plt.xlabel("Time (s)")
#     plt.ylabel("Base Velocity (m/s)")
#     plt.title("Base Velocity During Second Jump")
#     plt.grid()
#     plt.show()

# else:
#     print("Second jump start/end not clearly found.")
# Plotting
# plt.figure(figsize=(14, 12))

# plt.subplot(3, 2, 1)
# plt.plot(time_log, base_height_log)
# plt.title("Base Height")
# plt.xlabel("Time (s)")
# plt.ylabel("Height (m)")
# plt.grid()

# plt.subplot(3, 2, 2)
# plt.plot(time_log, base_velocity_log)
# plt.title("Base Velocity")
# plt.xlabel("Time (s)")
# plt.ylabel("Velocity (m/s)")
# plt.grid()

# plt.subplot(3, 2, 3)
# plt.plot(time_log, joint1_pos_log, label="Hip")
# plt.plot(time_log, joint2_pos_log, label="Knee")
# plt.title("Joint Positions")
# plt.xlabel("Time (s)")
# plt.ylabel("Radians")
# plt.legend()
# plt.grid()

# plt.subplot(3, 2, 4)
# plt.plot(time_log, joint1_vel_log, label="Hip")
# plt.plot(time_log, joint2_vel_log, label="Knee")
# plt.title("Joint Velocities")
# plt.xlabel("Time (s)")
# plt.ylabel("Rad/s")
# plt.legend()
# plt.grid()

# plt.subplot(3, 2, 5)
# plt.plot(time_log, torque1_store, label="Hip")
# plt.plot(time_log, torque2_store, label="Knee")
# plt.title("Joint Torques")
# plt.xlabel("Time (s)")
# plt.ylabel("Nm")
# plt.legend()
# plt.grid()

# plt.tight_layout()
# plt.savefig("jump_plot.png")
# plt.show()