import numpy as np
from scipy.optimize import minimize
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

class SLIPHopperMPC:
    def __init__(self):
        # Physics parameters
        self.m = 4.0        # mass (kg)
        self.g = 9.81       # gravity (m/s²)
        self.k = 8000.0     # spring stiffness (N/m)
        self.c = 50.0       # damping coefficient (Ns/m)
        self.l0 = 0.7       # nominal spring length (m)
        
        # MPC parameters
        self.dt = 0.001  # time step (s)
        self.horizon = 20   # prediction horizon steps
        self.control_horizon = 20  # control horizon steps
        
        # Control constraints
        self.u_min = 0.7   # min spring rest length (m)
        self.u_max = 0.9   # max spring rest length (m)
        self.f_max = 230.0 # max force (N)
        
        # Target height for jumping
        self.target_height = 1.5 # target jump height (m)
        
        # Storage for visualization
        self.spring_forces = []
    
    def slip_dynamics_smooth(self, state, u):
        """Smooth approximation of hybrid SLIP dynamics"""
        z, zdot, l, ldot = state
        u = np.clip(u, self.u_min, self.u_max)
        
        # Smooth contact transition (sigmoid function)
        contact_penetration = z - l
        alpha = 1.0 / (1.0 + np.exp(20.0 * contact_penetration))  # Steep transition
        
        # Spring force with smooth activation
        f_spring = alpha * (self.k * (u - l) - self.c * ldot)
        f_spring = np.clip(f_spring, -self.f_max, self.f_max)
        
        # Store for visualization
        self.spring_forces.append(f_spring)
        
        # Equations of motion
        zddot = (f_spring / self.m) - self.g
        lddot = -alpha * zddot  # Spring compresses only during contact
        
        return np.array([zdot, zddot, ldot, lddot])
    
    def detect_phase_transitions(self, state_trajectory):
        """Detect phase transitions in a simulated trajectory"""
        phases = []
        for state in state_trajectory:
            z, _, l, _ = state
            phases.append(1 if (z - l <= 0) else 0)  # 1 = stance, 0 = flight
        return np.array(phases)
    
    def simulate_slip(self, state, u_sequence):
        """Simulate SLIP model with smooth dynamics"""
        states = [state]
        self.spring_forces = []  # Reset force storage
        
        for i in range(min(len(u_sequence), self.horizon)):
            u = u_sequence[i] if i < len(u_sequence) else u_sequence[-1]
            
            # Use smooth dynamics
            state_dot = self.slip_dynamics_smooth(states[-1], u)
            new_state = states[-1] + state_dot * self.dt
            
            # Physical constraints
            new_state[2] = max(0.7, new_state[2])  # Spring length > 0.1m
            new_state[0] = max(new_state[2], new_state[0])  # z >= l
            
            states.append(new_state)
        
        return np.array(states)
    
    def cost_function(self, u_sequence, state):
        """MPC cost function with phase-aware terms"""
        states = self.simulate_slip(state, u_sequence)
        z_positions = states[:, 0]
        peak_height = np.max(z_positions)
        
        # Detect phases for specialized costing
        phases = self.detect_phase_transitions(states)
        stance_count = np.sum(phases == 1)
        
        # Phase-specific costs
        takeoff_velocity = 0
        if stance_count > 0:
            # Estimate takeoff velocity from stance phase
            stance_indices = np.where(phases == 1)[0] #np.where returns a tuple, [0] extracts the first array
            if len(stance_indices) > 0:
                takeoff_idx = stance_indices[-1]
                takeoff_velocity = states[takeoff_idx, 1]  # zdot at end of stance
        
        # Cost components
        height_error = np.square(peak_height - self.target_height)
        control_effort = np.sum(np.square(u_sequence - self.l0))
        velocity_penalty = np.square(takeoff_velocity - np.sqrt(2 * self.g * self.target_height))
        
        # Penalize excessive control changes for smoothness
        control_smoothness = np.sum(np.square(np.diff(u_sequence)))
        
        total_cost = (10000.0 * height_error + 
                      80 * control_effort + 
                      5.0 * velocity_penalty +
                      0.01 * control_smoothness)
        
        return total_cost
    
    def solve_mpc(self, current_state):
        """Solve the MPC optimization problem"""
        # Initial guess for control sequence (nominal spring length)
        u0 = np.ones(self.control_horizon) * self.l0
        
        # Define bounds for control inputs
        bounds = [(self.u_min, self.u_max)] * self.control_horizon
        
        # Solve optimization problem
        result = minimize(self.cost_function, u0, args=(current_state,),
                         method='L-BFGS-B', bounds=bounds,
                         options={'maxiter': 100, 'disp': False, 'ftol': 1e-4})
        
        if not result.success:
            print(f"MPC optimization warning: {result.message}")
            # Fallback: return nominal control
            return np.ones(self.control_horizon) * self.l0
        
        return result.x
    
    def control_loop(self, total_time=10.0):
        """Main control loop simulation"""
        # Initial state: [z, zdot, l, ldot]
        state = np.array([self.l0, 0.0, self.l0, 0.0])
        # jump_count = 0
        # reset_after_jump = True
        
        # Storage for results
        time_history = []
        state_history = [state]
        control_history = []
        phase_history = []
        force_history = []
        
        # Run simulation
        for t in np.arange(0, total_time, self.dt):
            # Solve MPC problem
            # if reset_after_jump and jump_count > 0:
            #     state = np.array([self.l0, 0.0, self.l0, 0.0])
            u_sequence = self.solve_mpc(state)
            
            # Apply first control input
            u = u_sequence[0]
            
            # Simulate system with applied control
            state_dot = self.slip_dynamics_smooth(state, u)
            state = state + state_dot * self.dt
            
            # Ensure physical constraints
            state[2] = max(0.7, state[2])  # Spring length > 0.7m
            state[0] = max(state[2], state[0])  # z >= l (can't go below ground)
            
            # Detect current phase
            current_phase = 1 if (state[0] - state[2] <= 0) else 0
            # was_in_flight = (state[0] - state[2] > 0)
            # now_landing = (state[0] - state[2] <= 0)

            # if was_in_flight and now_landing:
            #     jump_count += 1
            #     print(f"Jump {jump_count} completed at time {t:.2f}s")
            #     if reset_after_jump:
            #     # Don't use the landed state, reset for next jump
            #         continue
            # # Reset to nominal state but keep current height
            # state = new_state
            
            # Store results
            time_history.append(t)
            state_history.append(state)
            control_history.append(u)
            phase_history.append(current_phase)
            force_history.append(self.spring_forces[-1] if self.spring_forces else 0)
            
            # Print progress
            if len(time_history) % 50 == 0:
                print(f"Time: {t:.2f}s, Height: {state[0]:.2f}m, Phase: {'Stance' if current_phase else 'Flight'}")
        
        return (np.array(time_history), 
                np.array(state_history), 
                np.array(control_history),
                np.array(phase_history),
                np.array(force_history))

def visualize_results(time, states, controls, phases, forces, target_height):
    """Create comprehensive visualization of the results"""
    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 10))
    
    # Plot 1: Positions
    ax1.plot(time, states[:-1, 0], 'b-', label='Body Height', linewidth=2)
    ax1.plot(time, states[:-1, 2], 'g-', label='Spring Length', linewidth=2)
    ax1.axhline(y=target_height, color='r', linestyle='--', label='Target Height')
    
    # Shade stance phases
    stance_starts = np.where(np.diff(phases) == 1)[0]
    stance_ends = np.where(np.diff(phases) == -1)[0]
    
    for start, end in zip(stance_starts, stance_ends):
        if start < len(time) and end < len(time):
            ax1.axvspan(time[start], time[end], alpha=0.2, color='gray', label='Stance' if start == stance_starts[0] else "")
    
    ax1.set_ylabel('Position (m)')
    ax1.set_title('Body and Spring Position')
    ax1.legend()
    ax1.grid(True)
    
    # Plot 2: Velocity and Force
    ax2.plot(time, states[:-1, 1], 'purple', label='Body Velocity', linewidth=2)
    ax2_twin = ax2.twinx()
    ax2_twin.plot(time, forces, 'orange', label='Spring Force', linewidth=2)
    ax2.set_ylabel('Velocity (m/s)')
    ax2_twin.set_ylabel('Force (N)')
    ax2.set_title('Velocity and Spring Force')
    ax2.legend(loc='upper left')
    ax2_twin.legend(loc='upper right')
    ax2.grid(True)
    
    # Plot 3: Control Input
    ax3.plot(time, controls, 'r-', label='Spring Rest Length (Control)', linewidth=2)
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Control (m)')
    ax3.set_title('MPC Control Input')
    ax3.legend()
    ax3.grid(True)
    
    # Plot 4: Phase Diagram
    # Create a simple animation-like representation
    for i in range(0, len(time), 20):  # Sample every 20 steps
        if i < len(time):
            t = time[i]
            z = states[i, 0]
            l = states[i, 2]
            phase = phases[i]
            
            color = 'red' if phase == 1 else 'blue'
            ax4.plot(t, z, 'o', color=color, markersize=8, alpha=0.7)
    
    ax4.axhline(y=target_height, color='r', linestyle='--', label='Target Height')
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Body Height (m)')
    ax4.set_title('Phase Diagram (Red=Stance, Blue=Flight)')
    ax4.legend()
    ax4.grid(True)
    
    plt.tight_layout()
    return fig

def analyze_performance(time, states, phases, target_height):
    """Analyze and print performance metrics"""
    print("\n" + "="*50)
    print("PERFORMANCE ANALYSIS")
    print("="*50)
    
    # Calculate jump statistics
    max_height = np.max(states[:-1, 0])
    max_height_time = time[np.argmax(states[:-1, 0])]
    
    # Count jumps (stance -> flight transitions)
    phase_changes = np.diff(phases)
    jump_count = np.sum(phase_changes == -1)  # Stance to flight transitions
    
    print(f"Maximum height achieved: {max_height:.3f}m (at {max_height_time:.2f}s)")
    print(f"Target height: {target_height:.3f}m")
    print(f"Height error: {abs(max_height - target_height):.3f}m")
    print(f"Number of jumps detected: {jump_count}")
    
    # Analyze stance phases
    stance_indices = np.where(phases == 1)[0]
    if len(stance_indices) > 0:
        stance_durations = []
        current_duration = 0
        
        for i in range(1, len(phases)):
            if phases[i] == 1:
                current_duration += time[i] - time[i-1]
            elif phases[i-1] == 1:
                stance_durations.append(current_duration)
                current_duration = 0
        
        if current_duration > 0:
            stance_durations.append(current_duration)
        
        print(f"Average stance duration: {np.mean(stance_durations):.3f}s")
        print(f"Number of stance phases: {len(stance_durations)}")
    
    return max_height, jump_count

# Main execution
if __name__ == "__main__":
    print("SLIP-based MPC for Vertical Hopper")
    print("="*40)
    
    # Create and run the controller
    hopper = SLIPHopperMPC()
    time, states, controls, phases, forces = hopper.control_loop(total_time=1.0)
    
    # Analyze results
    max_height, jump_count = analyze_performance(time, states, phases, hopper.target_height)
    
    # Visualize results
    fig = visualize_results(time, states, controls, phases, forces, hopper.target_height)
    plt.show()
    
    print("\nSimulation completed successfully!")