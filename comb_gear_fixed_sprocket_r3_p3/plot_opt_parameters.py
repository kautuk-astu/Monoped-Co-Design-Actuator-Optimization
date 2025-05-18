import pandas as pd
import matplotlib.pyplot as plt

# === Parameters ===
filename = 'jump_logs.csv'  # Replace with your actual CSV file
time_offset = 5.972448348999023
x0_shift = 1

# === Read CSV ===
df = pd.read_csv(filename)

# === Select rows ===
# Example: use rows 300 to 600 (modify this range as needed)
df = df.iloc[290:610].copy()

# === Adjust time and x0 ===
df['Time'] = df['Time'] - time_offset
df['Time'] = df['Time'] - df['Time'].iloc[0]
df['Base Height'] = df['Base Height'] + x0_shift

# === Plotting ===
# fig, axs = plt.subplots(2, 2, figsize=(10, 8), gridspec_kw={'height_ratios': [2, 1]})
# (ax1, ax2), (ax3, _) = axs

# # Top-left: x0 and x0_dot
# ax1.plot(df['Time'], df['Base Height'], color='purple', label='$x_0$')
# ax1.plot(df['Time'], df['Base Velocity'], '--', color='purple', label='$\dot{x}_0$')
# ax1.set_title("Underactuated state")
# ax1.set_xlabel("time [s]")
# ax1.set_ylabel("[m] / [m/s]")
# ax1.grid(True)
# ax1.legend(loc='best')

# # Top-right: x1, x2 and their derivatives
# ax2.plot(df['Time'], df['Hip Pos'], color='crimson', label='$x_1$')
# ax2.plot(df['Time'], df['Hip Vel'], '--', color='crimson', label='$\dot{x}_1$')
# ax2.plot(df['Time'], df['Knee Pos'], color='dodgerblue', label='$x_2$')
# ax2.plot(df['Time'], df['Knee Vel'], '--', color='dodgerblue', label='$\dot{x}_2$')
# ax2.set_title("Actuated state")
# ax2.set_xlabel("time [s]")
# ax2.set_ylabel("[rad] / [rad/s]")
# ax2.grid(True)
# ax2.legend(loc='best')

# # Bottom: control torques
# ax3.plot(df['Time'], df['Hip Torque'], color='darkorange', label='$u_0$')
# ax3.plot(df['Time'], df['Knee Torque'], color='seagreen', label='$u_1$')
# ax3.set_title("Control torques")
# ax3.set_xlabel("time [s]")
# ax3.set_ylabel("[Nm]")
# ax3.grid(True)
# ax3.legend(loc='best')

# # === Adjust layout ===
# fig.tight_layout()
# plt.show()

#fig, axs = plt.subplots(2, 2, figsize=(12, 6), gridspec_kw={'height_ratios': [2, 1]})
fig = plt.figure(figsize=(7, 9))
gs = fig.add_gridspec(2, 2, height_ratios=[2, 1])
#(ax1, ax2), (ax3, ax4) = axs  # ax4 unused
ax1 = fig.add_subplot(gs[0, 0])  # top-left
ax2 = fig.add_subplot(gs[0, 1])  # top-right
ax3 = fig.add_subplot(gs[1, :])

# === Underactuated state with twin y-axis ===
ax1.plot(df['Time'], df['Base Height'], color='purple', label='$x_0$')
ax1.set_ylabel('[m]')
ax1.set_title('Underactuated state')
ax1.set_xlabel('time [s]')
ax1.grid(True)

ax1_twin = ax1.twinx()
ax1_twin.plot(df['Time'], df['Base Velocity'], '--', color='purple', label='$\dot{x}_0$')
ax1_twin.set_ylabel('[m/s]')

# Add legends manually for both axes
lines_1, labels_1 = ax1.get_legend_handles_labels()
lines_1b, labels_1b = ax1_twin.get_legend_handles_labels()
ax1.legend(lines_1 + lines_1b, labels_1 + labels_1b)

# === Actuated state with twin y-axis ===
ax2.plot(df['Time'], df['Hip Pos'], color='crimson', label='$x_1$')
ax2.plot(df['Time'], df['Knee Pos'], color='dodgerblue', label='$x_2$')
ax2.set_ylabel('[rad]')
ax2.set_title('Actuated state')
ax2.set_xlabel('time [s]')
ax2.grid(True)

ax2_twin = ax2.twinx()
ax2_twin.plot(df['Time'], df['Hip Vel'], '--', color='crimson', label='$\dot{x}_1$')
ax2_twin.plot(df['Time'], df['Knee Vel'], '--', color='dodgerblue', label='$\dot{x}_2$')
ax2_twin.set_ylabel('[rad/s]')

lines_2, labels_2 = ax2.get_legend_handles_labels()
lines_2b, labels_2b = ax2_twin.get_legend_handles_labels()
ax2.legend(lines_2 + lines_2b, labels_2 + labels_2b)

# === Control torques ===
ax3.plot(df['Time'], df['Hip Torque'], color='darkorange', label='$u_0$')
ax3.plot(df['Time'], df['Knee Torque'], color='seagreen', label='$u_1$')
ax3.set_title('Control torques')
ax3.set_xlabel('time [s]')
ax3.set_ylabel('[Nm]')
ax3.grid(True)
ax3.legend(loc='best')

# === Final layout ===
fig.tight_layout()
plt.savefig('jump_logs_plot.png', dpi=300)  # Save the figure
plt.show()

# 9.197008848190308,-0.3853235349440201,2.7421321450463414,-0.5003204881324271,1.4043031882828818,2.4702906358937655,3.5429505235604015,-35.542930629109584,-5.803955859915504
# 9.19906497001648,-0.379863345649245,2.7178056998774984,-0.49550008860044426,1.410680336346285,2.3485108164073414,2.8482598641946906,-34.807172387443636,0.5052359274012836
# 9.201121091842651,-0.3744715625684904,2.674174185064253,-0.49100436444225154,1.4164797914605907,2.147808205125492,2.948461517546109,-33.24971869044441,-1.076726117543469
# 9.20332384109497,-0.36916132089471426,2.636143893457118,-0.4868887139952279,1.4222810251400044,1.9678971533569745,2.854132863188734,-31.862173217461603,-0.7135629419110963
