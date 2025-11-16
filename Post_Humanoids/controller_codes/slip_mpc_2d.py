import numpy as np
from scipy.optimize import minimize
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.patches as patches

class SLIPHopper2DMPC:
    def __init__(self):
        # Physics parameters
        self.m = 5.0           # mass (kg)
        self.g = 9.81          # gravity (m/s²)
        self.k = 300.0        # spring stiffness (N/m)
        self.c = 50.0          # damping coefficient (Ns/m)
        self.l0 = 0.7          # nominal spring length (m)
        self.I = 0.2           # moment of inertia (kg·m²)
        
        # MPC parameters
        self.dt = 0.01         # time step (s) - slightly larger for 2D stability
        self.horizon = 15      # prediction horizon steps
        self.control_horizon = 15
        
        # 2D Control constraints
        self.u_length_min = 0.6   # min spring rest length (m)
        self.u_length_max = 1.9 # max spring rest length (m)
        self.u_angle_min = -0.04    # min leg angle (rad, from vertical)
        self.u_angle_max = 0.04     # max leg angle (rad, from vertical)
        
        # Target (2D position)
        self.target_x = 2.0        # target horizontal position (m)
        self.target_z = 1.2        # target vertical position (m)
        
        # Storage for visualization
        self.spring_forces = []
        self.foot_positions_x = []
        self.foot_positions_z = []
    
    def slip_dynamics_2d(self, state, u):
        """2D SLIP dynamics with full state"""
        x, xdot, z, zdot, theta, thetadot, l, ldot = state
        u_length, u_angle = u
        
        # Calculate foot position
        foot_x = x + l * np.sin(theta)
        foot_z = z - l * np.cos(theta)
        self.foot_positions_x.append(foot_x)
        self.foot_positions_z.append(foot_z)
        
        # Smooth ground contact detection
        ground_penetration = foot_z  # Positive when foot below ground (z=0)
        alpha = 1.0 / (1.0 + np.exp(30.0 * ground_penetration))
        
        # Spring and damping forces
        f_spring = self.k * (u_length - l)
        f_damping = self.c * ldot
        
        # Total force along leg direction
        f_total = alpha * (f_spring - f_damping)
        self.spring_forces.append(f_total)
        
        # Force components in world coordinates
        fx = f_total * np.sin(theta)
        fz = f_total * np.cos(theta)
        
        # Torque about hip (if foot is fixed during stance)
        tau = 0.0
        if alpha > 0.8:  # Strong contact
            # Torque tries to bring leg to desired angle
            tau = 50.0 * (u_angle - theta) - 5.0 * thetadot
        
        # Equations of motion
        xddot = fx / self.m
        zddot = fz / self.m - self.g
        thetaddot = tau / self.I
        
        # Spring compression dynamics
        if alpha > 0.1:  # Some contact
            # Relative acceleration along leg direction
            leg_accel = (xddot * np.sin(theta) + (zddot + self.g) * np.cos(theta) + 
                        l * thetadot**2)
            lddot = -leg_accel
        else:
            lddot = 0.0  # Free flight - no spring dynamics
        
        return np.array([xdot, xddot, zdot, zddot, 
                        thetadot, thetaddot, ldot, lddot])
    
    def simulate_slip_2d(self, state, u_sequence):
        """Simulate 2D SLIP model"""
        states = [state]
        self.spring_forces = []
        self.foot_positions_x = []
        self.foot_positions_z = []
        
        for i in range(min(len(u_sequence)//2, self.horizon)):
            u_length = u_sequence[2*i]
            u_angle = u_sequence[2*i + 1]
            
            state_dot = self.slip_dynamics_2d(states[-1], [u_length, u_angle])
            new_state = states[-1] + state_dot * self.dt
            
            # Physical constraints
            new_state[6] = max(0.3, new_state[6])  # Spring length > 0.3m
            new_state[2] = max(0.1, new_state[2])  # Body height > 0.1m
            
            states.append(new_state)
        
        return np.array(states)
    
    def cost_function_2d(self, u_sequence, state):
        """2D cost function - penalize both x and z errors"""
        states = self.simulate_slip_2d(state, u_sequence)
        
        if len(states) < 2:
            return 1000.0  # Large penalty for failed simulations
        
        # Extract positions
        x_positions = states[:, 0]
        z_positions = states[:, 2]
        thetas = states[:, 4]
        
        # Calculate costs
        peak_height = np.max(z_positions)
        final_x = x_positions[-1]
        avg_theta = np.mean(np.abs(thetas))
        
        height_error = np.square(peak_height - self.target_z)
        position_error = np.square(final_x - self.target_x)
        posture_error = avg_theta  # Penalize leaning too much
        
        # Control effort
        u_lengths = u_sequence[::2]
        u_angles = u_sequence[1::2]
        
        length_effort = np.sum(np.square(u_lengths - self.l0))
        angle_effort = np.sum(np.square(u_angles))
        
        total_cost = (10.0 * height_error +
                      8.0 * position_error +
                      2.0 * posture_error +
                      0.1 * length_effort +
                      0.05 * angle_effort)
        
        return total_cost
    
    def solve_mpc_2d(self, current_state):
        """2D MPC optimization"""
        # Initial guess: nominal length, zero angle
        u0 = np.zeros(2 * self.control_horizon)
        u0[::2] = self.l0
        u0[1::2] = 0.0
        
        # Bounds for both control types
        bounds = []
        for i in range(self.control_horizon):
            bounds.append((self.u_length_min, self.u_length_max))
            bounds.append((self.u_angle_min, self.u_angle_max))
        
        result = minimize(self.cost_function_2d, u0, args=(current_state,),
                         method='SLSQP', bounds=bounds,
                         options={'maxiter': 50, 'disp': False, 'ftol': 1e-3})
        
        if not result.success:
            # Fallback to nominal control
            return np.array([self.l0, 0.0] * self.control_horizon)
        
        return result.x
    
    def control_loop_2d(self, total_time=4.0):
        """Main 2D control loop"""
        # Initial state: [x, ẋ, z, ż, θ, θ̇, l, Ḻ]
        state = np.array([0.0, 0.0, self.l0, 0.0, 0.0, 0.0, self.l0, 0.0])
        
        # Storage for results
        time_history = []
        state_history = [state]
        control_history = []
        phase_history = []
        force_history = []
        foot_x_history = []
        foot_z_history = []
        
        print("Starting 2D MPC simulation...")
        
        for t in np.arange(0, total_time, self.dt):
            # Solve MPC problem
            u_sequence = self.solve_mpc_2d(state)
            u_length = u_sequence[0]
            u_angle = u_sequence[1]
            
            # Simulate dynamics
            state_dot = self.slip_dynamics_2d(state, [u_length, u_angle])
            state = state + state_dot * self.dt
            
            # Store results
            time_history.append(t)
            state_history.append(state)
            control_history.append([u_length, u_angle])
            force_history.append(self.spring_forces[-1] if self.spring_forces else 0)
            foot_x_history.append(self.foot_positions_x[-1] if self.foot_positions_x else 0)
            foot_z_history.append(self.foot_positions_z[-1] if self.foot_positions_z else 0)
            
            # Detect phase (simple contact detection)
            foot_z = state[2] - state[6] * np.cos(state[4])
            phase = 1 if foot_z <= 0.05 else 0  # Small threshold
            phase_history.append(phase)
            
            # Print progress
            if len(time_history) % 40 == 0:
                print(f"Time: {t:.2f}s, X: {state[0]:.2f}m, Z: {state[2]:.2f}m")
        
        return (np.array(time_history), 
                np.array(state_history), 
                np.array(control_history),
                np.array(phase_history),
                np.array(force_history),
                np.array(foot_x_history),
                np.array(foot_z_history))

def visualize_2d_results(time, states, controls, phases, forces, foot_x, foot_z, target_x, target_z):
    """Create comprehensive 2D visualization"""
    fig = plt.figure(figsize=(16, 12))
    
    # Main 2D trajectory plot
    ax1 = plt.subplot2grid((3, 3), (0, 0), colspan=2, rowspan=2)
    # Positions over time
    ax2 = plt.subplot2grid((3, 3), (0, 2))
    # Controls
    ax3 = plt.subplot2grid((3, 3), (1, 2))
    # Phase and forces
    ax4 = plt.subplot2grid((3, 3), (2, 0), colspan=3)
    
    # Plot 1: 2D Trajectory
    ax1.plot(states[:-1, 0], states[:-1, 2], 'b-', label='Body Trajectory', linewidth=2, alpha=0.7)
    ax1.plot(foot_x, foot_z, 'r--', label='Foot Position', linewidth=1, alpha=0.7)
    
    # Mark target
    ax1.plot(target_x, target_z, 'ro', markersize=10, label='Target', markeredgecolor='black')
    
    # Draw ground
    ax1.axhline(y=0, color='green', linestyle='-', linewidth=3, label='Ground')
    
    # Draw robot at few positions
    for i in range(0, len(time), 30):
        if i < len(states) - 1:
            x, z, theta, l = states[i, 0], states[i, 2], states[i, 4], states[i, 6]
            foot_x_pos = x + l * np.sin(theta)
            foot_z_pos = z - l * np.cos(theta)
            
            # Draw body
            ax1.plot(x, z, 'o', color='blue', markersize=8)
            
            # Draw leg
            ax1.plot([x, foot_x_pos], [z, foot_z_pos], 'k-', linewidth=2)
    
    ax1.set_xlabel('Horizontal Position (m)')
    ax1.set_ylabel('Vertical Position (m)')
    ax1.set_title('2D Hopper Trajectory')
    ax1.legend()
    ax1.grid(True)
    ax1.axis('equal')
    
    # Plot 2: Positions over time
    ax2.plot(time, states[:-1, 0], 'b-', label='X Position', linewidth=2)
    ax2.plot(time, states[:-1, 2], 'r-', label='Z Position', linewidth=2)
    ax2.axhline(y=target_z, color='r', linestyle='--', label='Target Height')
    ax2.axhline(y=target_x, color='b', linestyle='--', label='Target Distance')
    ax2.set_ylabel('Position (m)')
    ax2.set_title('Position vs Time')
    ax2.legend()
    ax2.grid(True)
    
    # Plot 3: Controls
    ax3.plot(time, controls[:, 0], 'g-', label='Spring Length', linewidth=2)
    ax3_twin = ax3.twinx()
    ax3_twin.plot(time, np.degrees(controls[:, 1]), 'orange', label='Leg Angle (°)', linewidth=2)
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Length (m)')
    ax3_twin.set_ylabel('Angle (°)')
    ax3.set_title('Control Inputs')
    ax3.legend(loc='upper left')
    ax3_twin.legend(loc='upper right')
    ax3.grid(True)
    
    # Plot 4: Phase and Forces
    # Shade stance phases
    for i in range(len(phases)-1):
        if phases[i] == 1:
            ax4.axvspan(time[i], time[i+1], alpha=0.3, color='gray')
    
    ax4.plot(time, forces, 'r-', label='Spring Force', linewidth=2)
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Force (N)')
    ax4.set_title('Spring Force (Gray = Stance Phase)')
    ax4.legend()
    ax4.grid(True)
    
    plt.tight_layout()
    return fig

def create_2d_animation(time, states, foot_x, foot_z):
    """Create animated visualization of 2D hopping"""
    fig, ax = plt.subplots(figsize=(10, 8))
    
    # Set up plot
    ax.set_xlim(-0.5, 3.0)
    ax.set_ylim(-0.2, 2.0)
    ax.set_xlabel('Horizontal Position (m)')
    ax.set_ylabel('Vertical Position (m)')
    ax.set_title('2D Hopper Animation')
    ax.axhline(y=0, color='green', linewidth=3, label='Ground')
    ax.grid(True)
    
    # Create objects for animation
    body, = ax.plot([], [], 'bo', markersize=12)
    leg, = ax.plot([], [], 'k-', linewidth=3)
    foot, = ax.plot([], [], 'ro', markersize=8)
    trajectory, = ax.plot([], [], 'b--', alpha=0.5)
    
    # Target marker
    target = ax.plot(2.0, 1.2, 'rx', markersize=15, markeredgewidth=2)[0]
    
    # Initialize
    x_traj, z_traj = [], []
    
    def animate(i):
        if i < len(states) - 1:
            state = states[i]
            x, z, theta, l = state[0], state[2], state[4], state[6]
            foot_x_pos = foot_x[i] if i < len(foot_x) else x + l * np.sin(theta)
            foot_z_pos = foot_z[i] if i < len(foot_z) else z - l * np.cos(theta)
            
            # Update positions
            body.set_data([x], [z])
            leg.set_data([x, foot_x_pos], [z, foot_z_pos])
            foot.set_data([foot_x_pos], [foot_z_pos])
            
            # Update trajectory
            x_traj.append(x)
            z_traj.append(z)
            trajectory.set_data(x_traj, z_traj)
            
            # Time annotation
            ax.set_title(f'2D Hopper Animation - Time: {time[i]:.2f}s')
        
        return body, leg, foot, trajectory
    
    ani = FuncAnimation(fig, animate, frames=len(time), interval=50, blit=True)
    return ani

def analyze_2d_performance(time, states, target_x, target_z):
    """Analyze 2D jumping performance"""
    print("\n" + "="*60)
    print("2D PERFORMANCE ANALYSIS")
    print("="*60)
    
    x_positions = states[:-1, 0]
    z_positions = states[:-1, 2]
    
    # Calculate metrics
    peak_height = np.max(z_positions)
    final_x = x_positions[-1]
    total_distance = np.max(x_positions) - np.min(x_positions)
    
    height_error = abs(peak_height - target_z)
    position_error = abs(final_x - target_x)
    
    print(f"Peak height achieved: {peak_height:.3f}m (Target: {target_z:.3f}m)")
    print(f"Final horizontal position: {final_x:.3f}m (Target: {target_x:.3f}m)")
    print(f"Total distance traveled: {total_distance:.3f}m")
    print(f"Height error: {height_error:.3f}m")
    print(f"Position error: {position_error:.3f}m")
    
    # Detect jumps
    z_velocities = states[:-1, 3]
    jump_indices = np.where((z_velocities[:-1] > 0) & (z_velocities[1:] < 0))[0]
    print(f"Number of jumps detected: {len(jump_indices)}")
    
    return peak_height, final_x, total_distance

# Main execution
if __name__ == "__main__":
    print("2D SLIP-based MPC for Hopper Robot")
    print("="*50)
    
    # Create and run the 2D controller
    hopper_2d = SLIPHopper2DMPC()
    time, states, controls, phases, forces, foot_x, foot_z = hopper_2d.control_loop_2d(total_time=4.0)
    
    # Analyze results
    peak_height, final_x, total_distance = analyze_2d_performance(
        time, states, hopper_2d.target_x, hopper_2d.target_z
    )
    
    # Visualize results
    fig = visualize_2d_results(time, states, controls, phases, forces, 
                              foot_x, foot_z, hopper_2d.target_x, hopper_2d.target_z)
    
    # Create animation (optional - can be slow)
    # ani = create_2d_animation(time, states, foot_x, foot_z)
    
    plt.tight_layout()
    plt.show()
    
    # Save animation if desired
    # ani.save('2d_hopper_animation.mp4', writer='ffmpeg', fps=30)
    
    print("\n2D simulation completed! Check the plots for results.")