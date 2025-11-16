import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize

class VerticalHopperEnhanced:
    def __init__(self):
        # OPTIMIZED Physics parameters for higher jumps
        self.m = 3.0           # Reduced mass (kg) - lighter robot jumps higher
        self.g = 9.81          # gravity (m/s²)
        self.k = 5000.0        # Increased spring stiffness (N/m) - more energy storage
        self.c = 30.0          # Reduced damping - less energy loss
        self.l0 = 0.7          # nominal spring length (m)
        
        # EXPANDED Control range for more energy storage
        self.u_min = 0.55      # Increased compression (was 0.65)
        self.u_max = 0.85      # Increased extension (was 0.75)
        
        # Simulation parameters
        self.dt = 0.005        # Smaller time step for better accuracy
        self.T = 400           # More time steps (2 seconds total)
        
        # Target height - now achievable
        self.target_height = 1.2
        
        # Physical limits
        self.l_min = 0.3
        self.l_max = 1.0
        self.f_max = 3000.0    # Increased force limit
        
    def dynamics(self, x, u):
        """Improved dynamics with better energy handling"""
        z, zdot, l, ldot = x
        u_length = np.clip(u[0], self.u_min, self.u_max)
        
        # Enhanced contact detection
        ground_penetration = z - l
        alpha = 1.0 / (1.0 + np.exp(25.0 * ground_penetration))
        
        # Improved spring model with energy conservation
        f_spring = alpha * self.k * (u_length - l)
        f_damping = alpha * self.c * ldot
        
        # Only apply damping during strong contact to conserve energy
        if alpha < 0.7:  # Flight or light contact
            f_damping *= 0.1  # Greatly reduce damping in flight
        
        f_total = f_spring - f_damping
        f_total = np.clip(f_total, 0, self.f_max)
        
        # Equations of motion
        zddot = f_total / self.m - self.g
        lddot = -alpha * zddot
        
        return np.array([zdot, zddot, ldot, lddot])
    
    def calculate_energy(self, x, u):
        """Calculate total mechanical energy"""
        z, zdot, l, ldot = x
        u_length = u[0]
        
        # Kinetic energy
        E_kinetic = 0.5 * self.m * zdot**2
        
        # Potential energy (gravity)
        E_potential = self.m * self.g * z
        
        # Spring potential energy
        E_spring = 0.5 * self.k * (u_length - l)**2
        
        return E_kinetic + E_potential + E_spring
    
    def simulate_trajectory(self, us):
        """Enhanced simulation with energy monitoring"""
        x = np.array([self.l0, 0.0, self.l0, 0.0])
        xs = [x.copy()]
        energies = []
        
        for i, u in enumerate(us):
            # RK4 integration
            k1 = self.dynamics(x, u)
            k2 = self.dynamics(x + 0.5 * self.dt * k1, u)
            k3 = self.dynamics(x + 0.5 * self.dt * k2, u)
            k4 = self.dynamics(x + self.dt * k3, u)
            
            xdot = (k1 + 2*k2 + 2*k3 + k4) / 6
            x = x + self.dt * xdot
            
            # Apply constraints
            x[2] = np.clip(x[2], self.l_min, self.l_max)
            x[0] = max(x[2] + 0.01, x[0])  # Small clearance
            
            xs.append(x.copy())
            energies.append(self.calculate_energy(x, u))
        
        return np.array(xs), np.array(energies)
    
    def create_optimized_jump_control(self):
        """Create control sequence optimized for maximum jump height"""
        print("Creating optimized jump sequence...")
        
        def jump_cost(u_flat):
            u_sequence = u_flat.reshape(-1, 1)
            xs, energies = self.simulate_trajectory(u_sequence)
            
            # Primary goal: maximize peak height
            peak_height = np.max(xs[:, 0])
            height_error = (self.target_height - peak_height) ** 2
            
            # Secondary: minimize energy usage
            avg_energy = np.mean(energies)
            energy_penalty = 0.001 * avg_energy
            
            # Encourage proper jumping rhythm
            u_diff = np.diff(u_sequence.flatten())
            smoothness_penalty = 0.01 * np.sum(u_diff**2)
            
            return height_error + energy_penalty + smoothness_penalty
        
        # Initial guess: optimized jumping pattern
        u0 = np.ones(self.T) * self.l0
        
        # Create a more aggressive jumping pattern
        for i in range(self.T):
            # Multiple jump cycles with proper timing
            cycle_time = (i * self.dt) % 1.0  # 1-second cycles
            
            if cycle_time < 0.2:
                # Strong compression
                u0[i] = self.u_min
            elif cycle_time < 0.25:
                # Quick extension for explosive jump
                u0[i] = self.u_max
            elif cycle_time < 0.4:
                # Prepare for next jump
                u0[i] = self.l0 - 0.05
            else:
                # Recovery
                u0[i] = self.l0
        
        # Optimize using more robust method
        bounds = [(self.u_min, self.u_max)] * self.T
        
        result = minimize(jump_cost, u0, method='L-BFGS-B', bounds=bounds,
                         options={'maxiter': 50, 'disp': True})
        
        if result.success:
            print("Optimization successful!")
            return result.x.reshape(-1, 1)
        else:
            print("Optimization failed, using initial guess")
            return u0.reshape(-1, 1)
    
    def create_energy_optimized_control(self):
        """Control sequence focused on energy management"""
        print("Creating energy-optimized control...")
        us = []
        
        for t in range(self.T):
            time_in_cycle = (t * self.dt) % 1.2  # Longer cycle for higher jumps
            
            if time_in_cycle < 0.3:  # Longer compression phase
                # Deep compression for more energy storage
                compression = 0.8  # 80% of maximum compression
                u = self.u_min + (1 - compression) * (self.l0 - self.u_min)
            elif time_in_cycle < 0.35:  # Very quick extension
                # Full extension for maximum thrust
                u = self.u_max
            elif time_in_cycle < 0.6:  # Flight phase preparation
                # Slight compression for landing preparation
                u = self.l0 - 0.03
            else:  # Landing and recovery
                u = self.l0
            
            us.append(np.array([u]))
        
        return np.array(us)
    
    def analyze_energy_management(self, xs, us, energies):
        """Detailed energy analysis"""
        print("\n" + "="*60)
        print("ENERGY ANALYSIS")
        print("="*60)
        
        peak_height = np.max(xs[:, 0])
        max_energy = np.max(energies)
        min_energy = np.min(energies)
        energy_variation = max_energy - min_energy
        
        print(f"Peak height: {peak_height:.3f}m")
        print(f"Maximum energy: {max_energy:.2f}J")
        print(f"Minimum energy: {min_energy:.2f}J") 
        print(f"Energy variation: {energy_variation:.2f}J")
        
        # Calculate energy efficiency
        takeoff_energy = energies[0]
        peak_energy = max_energy
        if peak_energy > takeoff_energy:
            efficiency = (peak_height - self.l0) / (peak_energy - takeoff_energy)
            print(f"Energy efficiency: {efficiency:.3f}m/J")
        
        return peak_height, max_energy
    
    def visualize_enhanced_results(self, xs, us, energies, title="Enhanced Hopper Results"):
        """Enhanced visualization with energy plots"""
        time_xs = np.arange(0, len(xs)) * self.dt
        time_us = np.arange(0, len(us)) * self.dt
        time_energy = np.arange(0, len(energies)) * self.dt
        
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(16, 12))
        
        # Plot 1: Positions with energy background
        ax1.plot(time_xs, xs[:, 0], 'b-', label='Body Height', linewidth=3)
        ax1.plot(time_xs, xs[:, 2], 'g-', label='Spring Length', linewidth=2)
        ax1.axhline(y=self.target_height, color='r', linestyle='--', 
                   label='Target Height', linewidth=2)
        ax1.fill_between(time_energy, 0, 2, where=energies > np.mean(energies),
                        alpha=0.2, color='yellow', label='High Energy')
        ax1.set_ylabel('Position (m)')
        ax1.set_ylim(0, 1.5)
        ax1.legend()
        ax1.grid(True)
        
        # Plot 2: Energy trajectory
        ax2.plot(time_energy, energies, 'purple', linewidth=2)
        ax2.set_ylabel('Total Energy (J)')
        ax2.set_title('Energy Management')
        ax2.grid(True)
        
        # Plot 3: Control input with energy overlay
        ax3.plot(time_us, us.flatten(), 'r-', label='Control', linewidth=2)
        ax3_twin = ax3.twinx()
        ax3_twin.plot(time_energy, energies, 'purple', alpha=0.5, label='Energy')
        ax3.set_ylabel('Control (m)')
        ax3_twin.set_ylabel('Energy (J)')
        ax3.legend(loc='upper left')
        ax3_twin.legend(loc='upper right')
        ax3.grid(True)
        
        # Plot 4: Height vs Energy (phase-like plot)
        ax4.scatter(xs[:len(energies), 0], energies, c=time_energy, cmap='viridis')
        ax4.set_xlabel('Height (m)')
        ax4.set_ylabel('Energy (J)')
        ax4.set_title('Height vs Energy')
        ax4.grid(True)
        
        plt.suptitle(title, fontsize=16)
        plt.tight_layout()
        return fig

def main():
    """Main execution with enhanced parameters"""
    print("Enhanced Vertical Hopper - Targeting 1.2m Jumps")
    print("="*60)
    
    hopper = VerticalHopperEnhanced()
    
    # Try energy-optimized control first
    try:
        print("1. Energy-optimized control sequence...")
        us_energy = hopper.create_energy_optimized_control()
        xs_energy, energies_energy = hopper.simulate_trajectory(us_energy)
        
        peak_height_energy, max_energy = hopper.analyze_energy_management(
            xs_energy, us_energy, energies_energy)
        
        fig_energy = hopper.visualize_enhanced_results(
            xs_energy, us_energy, energies_energy, "Energy-Optimized Control")
        
    except Exception as e:
        print(f"Energy-optimized control failed: {e}")
        us_energy = None
    
    # Try optimized jump control
    try:
        print("\n2. Optimized jump control...")
        us_jump = hopper.create_optimized_jump_control()
        xs_jump, energies_jump = hopper.simulate_trajectory(us_jump)
        
        peak_height_jump, _ = hopper.analyze_energy_management(
            xs_jump, us_jump, energies_jump)
        
        fig_jump = hopper.visualize_enhanced_results(
            xs_jump, us_jump, energies_jump, "Jump-Optimized Control")
        
    except Exception as e:
        print(f"Jump-optimized control failed: {e}")
        us_jump = None
    
    # Results comparison
    print("\n" + "="*50)
    print("FINAL RESULTS COMPARISON")
    print("="*50)
    
    if us_energy is not None:
        print(f"Energy-optimized: {peak_height_energy:.3f}m")
    
    if us_jump is not None:
        print(f"Jump-optimized: {peak_height_jump:.3f}m")
    
    if us_energy is not None and us_jump is not None:
        if peak_height_jump > peak_height_energy:
            print("✓ Jump-optimized control achieved higher jumps!")
        else:
            print("✓ Energy-optimized control was more effective!")
    
    plt.show()
    print("\nEnhanced simulation completed!")

if __name__ == "__main__":
    main()