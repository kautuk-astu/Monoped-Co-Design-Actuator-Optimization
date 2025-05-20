import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

# --- Read and preprocess your data (as you've already done) ---
filename = '/home/stochlab/repo/Aastha_Coopt_Monoped/Monoped-optimization/corrected_joint_cma/store_variables_opt_jump.csv'
time_offset = 0
x0_shift = 0.5

# Font sizes
axis_label_fontsize = 15
tick_fontsize = 10
title_fontsize = 14
legend_fontsize = 10

df = pd.read_csv(filename)
df = df.iloc[2:261].copy()
df['time_store'] -= df['time_store'].iloc[0]
df['base_store'] += x0_shift



# Step size for uniform time (e.g., 0.01s)
dt = 0.01
uniform_time = np.arange(df['time_store'].iloc[0], df['time_store'].iloc[-1], dt)

# Interpolating all required columns
df_interp = pd.DataFrame({'time_store': uniform_time})
for col in ['base_store', 'v0_store', 'theta1_store', 'theta2_store', 'omega1_store', 'omega2_store', 'torque1_store', 'torque2_store']:
    df_interp[col] = np.interp(uniform_time, df['time_store'], df[col])

# --- Setup figure and axes ---
fig = plt.figure(figsize=(14, 9))
gs = fig.add_gridspec(2, 2, height_ratios=[2, 1])
ax1 = fig.add_subplot(gs[0, 0])
ax2 = fig.add_subplot(gs[0, 1])
ax3 = fig.add_subplot(gs[1, :])

# Plot static data
ax1.plot(df['time_store'], df['base_store'], color='purple', label='$x_b$')
ax1.set_ylabel('[m]', fontsize=axis_label_fontsize)
ax1.set_title('Base Height and Velocity', fontsize=title_fontsize)
ax1.set_xlabel('time [s]', fontsize=axis_label_fontsize)
ax1.grid(True)

ax1_twin = ax1.twinx()
ax1_twin.spines["right"].set_position(("axes", 1))
ax1_twin.plot(df['time_store'], df['v0_store'], '--', color='purple', label='$\dot{x}_b$')
ax1_twin.set_ylabel('[m/s]', fontsize=axis_label_fontsize)

# Add legends manually for both axes
lines_1, labels_1 = ax1.get_legend_handles_labels()
lines_1b, labels_1b = ax1_twin.get_legend_handles_labels()
ax1.legend(lines_1 + lines_1b, labels_1 + labels_1b)

# === Actuated state with twin y-axis ===
ax2.plot(df['time_store'], df['theta1_store'], color='crimson', label='$x_h$')
ax2.plot(df['time_store'], df['theta2_store'], color='dodgerblue', label='$x_k$')
ax2.set_ylabel('[rad]', fontsize=axis_label_fontsize)
ax2.set_title('Hip and Knee Angles and Velocities', fontsize=title_fontsize)
ax2.set_xlabel('time [s]', fontsize=axis_label_fontsize)
ax2.grid(True)

ax2_twin = ax2.twinx()
ax2_twin.spines["right"].set_position(("axes", 1.0))


ax2_twin.plot(df['time_store'], df['omega1_store'], '--', color='crimson', label='$\dot{x}_h$')
ax2_twin.plot(df['time_store'], df['omega2_store'], '--', color='dodgerblue', label='$\dot{x}_k$')
ax2_twin.set_ylabel('[rad/s]', fontsize=axis_label_fontsize)

lines_2, labels_2 = ax2.get_legend_handles_labels()
lines_2b, labels_2b = ax2_twin.get_legend_handles_labels()
ax2.legend(lines_2 + lines_2b, labels_2 + labels_2b)

# === Control torques ===
ax3.plot(df['time_store'], df['torque1_store'], color='darkorange', label='$u_h$')
ax3.plot(df['time_store'], df['torque2_store'], color='seagreen', label='$u_k$')
ax3.set_title('Control torques', fontsize=title_fontsize)
ax3.set_xlabel('time [s]',fontsize=axis_label_fontsize)
ax3.set_ylabel('[Nm]', fontsize=axis_label_fontsize)
ax3.grid(True)
ax3.legend(loc='best', fontsize=legend_fontsize)


# Add vertical time line and time text to all axes
vline1 = ax1.axvline(x=0, color='black', linestyle='--')
vline2 = ax2.axvline(x=0, color='black', linestyle='--')
vline3 = ax3.axvline(x=0, color='black', linestyle='--')

time_text = fig.text(0.45, 0.92, '', fontsize=12)

# --- Animation update function ---
def update(frame):
    #current_time = df['time_store'].iloc[frame]
    current_time = df_interp['time_store'].iloc[frame]
    vline1.set_xdata([current_time])
    vline2.set_xdata([current_time])
    vline3.set_xdata([current_time])
    time_text.set_text(f'Time: {current_time:.2f} s')
    return vline1, vline2, vline3, time_text


# --- Run the animation ---
# ani = animation.FuncAnimation(
#     fig, update,
#     frames=len(df),
#     interval=40,  # ~25 FPS
#     blit=False  # ensure compatibility
# )
ani = animation.FuncAnimation(
    fig, update,
    frames=len(df_interp),
    interval=20,  # ~25 FPS
    blit=False  # ensure compatibility
)
ani.save('/home/stochlab/repo/Aastha_Coopt_Monoped/Monoped-optimization/corrected_joint_cma/caseC_animation.mp4', writer='ffmpeg', fps=60, dpi=200)
fig.subplots_adjust(wspace=0.8)

fig.tight_layout(pad=2.0)

plt.tight_layout()
plt.show()

