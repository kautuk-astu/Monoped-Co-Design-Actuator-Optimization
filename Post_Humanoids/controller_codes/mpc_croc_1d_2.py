import numpy as np
import matplotlib.pyplot as plt
from cyipopt import minimize_ipopt
from scipy.optimize import minimize
import warnings

class TunedIPOPTHopper:
    def __init__(self):
        # TUNED parameters for better jumping performance
        self.m = 2.8           # Slightly reduced mass
        self.g = 9.81
        self.k = 5500.0        # Increased stiffness
        self.c = 25.0          # Reduced damping
        self.l0 = 0.7
        self.u_min = 0.50      # Wider range for more energy storage
        self.u_max = 0.90
        self.dt = 0.005
        self.T = 200
        self.target_height = 1.2
        
        # Better IPOPT options
        self.ipopt_options = {
            'max_iter': 500,           # More iterations
            'tol': 1e-8,               # Tighter tolerance
            'constr_viol_tol': 1e-8,
            'acceptable_tol': 1e-6,
            'print_level': 5,
            'mu_strategy': 'adaptive',
            'hessian_approximation': 'limited-memory',
            'limited_memory_max_history': 50,
            'alpha_for_y': 'min',
            'recalc_y': 'yes',
        }
        
    def slip_dynamics(self, x, u):
        """Improved dynamics with better energy conservation"""
        z, zdot, l, ldot = x
        u_physical = np.clip(u[0], self.u_min, self.u_max)
        
        # Sharper contact transition
        penetration = z - l
        alpha = 1.0 / (1.0 + np.exp(30.0 * penetration))
        
        # Enhanced spring model
        f_spring = alpha * self.k * (u_physical - l)
        
        # Smart damping: minimal during flight, normal during contact
        if alpha > 0.8:  # Strong contact
            f_damping = alpha * self.c * ldot
        else:  # Flight or weak contact
            f_damping = alpha * self.c * ldot * 0.1
            
        f_total = np.clip(f_spring - f_damping, 0, 4000.0)
        
        zddot = f_total / self.m - self.g
        lddot = -alpha * zddot
        
        return np.array([zdot, zddot, ldot, lddot])
    
    def simulate_trajectory(self, u_sequence):
        """More accurate simulation"""
        x = np.array([self.l0, 0.0, self.l0, 0.0])
        xs = [x.copy()]
        energies = []
        
        for i, u in enumerate(u_sequence):
            # RK4 integration
            k1 = self.slip_dynamics(x, u)
            k2 = self.slip_dynamics(x + 0.5 * self.dt * k1, u)
            k3 = self.slip_dynamics(x + 0.5 * self.dt * k2, u)
            k4 = self.slip_dynamics(x + self.dt * k3, u)
            
            xdot = (k1 + 2*k2 + 2*k3 + k4) / 6
            x = x + self.dt * xdot
            
            # Apply constraints more carefully
            x[2] = np.clip(x[2], 0.3, 1.0)
            x[0] = max(x[2] + 0.02, x[0])  # Small clearance
            
            xs.append(x.copy())
            
            # Energy monitoring
            kinetic = 0.5 * self.m * x[1]**2
            potential = self.m * self.g * x[0]
            spring_energy = 0.5 * self.k * (u[0] - x[2])**2
            energies.append(kinetic + potential + spring_energy)
        
        return np.array(xs), np.array(energies)
    
    def enhanced_cost_function(self, u_flat):
        """Much better cost function for IPOPT"""
        u_sequence = u_flat.reshape(-1, 1)
        xs, energies = self.simulate_trajectory(u_sequence)
        
        heights = xs[:, 0]
        velocities = xs[:, 1]
        
        # Primary objective with better weighting
        peak_height = np.max(heights)
        
        # Asymmetric cost: heavier penalty for being below target
        if peak_height < self.target_height:
            height_error = 20.0 * (self.target_height - peak_height) ** 2
        else:
            height_error = 2.0 * (self.target_height - peak_height) ** 2  # Reward exceeding
        
        # Control effort with adaptive weighting
        control_effort = 0.002 * np.sum((u_sequence - self.l0) ** 2)
        
        # Smoothness penalty
        control_smoothness = 0.0005 * np.sum(np.diff(u_sequence.flatten()) ** 2)
        
        # Energy efficiency bonus
        if len(energies) > 0:
            energy_ratio = np.max(energies) / (self.m * self.g * self.target_height)
            energy_penalty = 0.1 * max(0, energy_ratio - 2.0)  # Penalize excessive energy
        else:
            energy_penalty = 0.0
            
        # Jump quality: encourage proper jumping rhythm
        takeoffs = np.where((velocities[:-1] <= 0) & (velocities[1:] > 0))[0]
        if len(takeoffs) < 2:  # Want multiple jumps
            rhythm_penalty = 10.0 * (2 - len(takeoffs))
        else:
            rhythm_penalty = 0.0
        
        total_cost = height_error + control_effort + control_smoothness + energy_penalty + rhythm_penalty
        
        return total_cost
    
    def cost_gradient(self, u_flat):
        """More accurate gradient computation"""
        grad = np.zeros_like(u_flat)
        eps = 1e-7  # Smaller epsilon for better accuracy
        
        base_cost = self.enhanced_cost_function(u_flat)
        
        for i in range(len(u_flat)):
            u_perturbed = u_flat.copy()
            u_perturbed[i] += eps
            cost_plus = self.enhanced_cost_function(u_perturbed)
            grad[i] = (cost_plus - base_cost) / eps
        
        return grad
    
    def create_optimized_initial_guess(self):
        """Much better initial guess based on successful patterns"""
        u0 = np.ones(self.T) * 0.5
        
        for i in range(self.T):
            time = i * self.dt
            # Multi-frequency pattern for better exploration
            cycle1 = time % 0.7  # Primary jump cycle
            cycle2 = time % 0.3  # Secondary oscillation
            
            if cycle1 < 0.15:
                u0[i] = 0.0  # Strong compression
            elif cycle1 < 0.20:
                u0[i] = 1.0  # Explosive extension
            elif cycle1 < 0.35:
                u0[i] = 0.1  # Recovery
            elif cycle1 < 0.40:
                u0[i] = 0.8  # Secondary push
            else:
                u0[i] = 0.5  # Neutral
                
            # Add small oscillations for better gradient information
            u0[i] += 0.05 * np.sin(2 * np.pi * cycle2 / 0.3)
            u0[i] = np.clip(u0[i], 0.0, 1.0)
                
        return u0
    
    def optimize_with_tuned_ipopt(self):
        """Better IPOPT optimization"""
        print("="*60)
        print("TUNED IPOPT OPTIMIZATION")
        print("="*60)
        
        u0 = self.create_optimized_initial_guess()
        bounds = [(0.0, 1.0)] * self.T
        
        print(f"Initial guess cost: {self.enhanced_cost_function(u0):.6f}")
        
        try:
            print("Running tuned IPOPT optimization...")
            result = minimize_ipopt(
                self.enhanced_cost_function,
                u0,
                bounds=bounds,
                options=self.ipopt_options,
                jac=self.cost_gradient
            )
            
            print(f"Final cost: {result.fun:.6f}")
            print(f"IPOPT message: {result.message}")
            
            u_physical = self.u_min + result.x.reshape(-1, 1) * (self.u_max - self.u_min)
            return u_physical, result, "Tuned-IPOPT"
                
        except Exception as e:
            print(f"IPOPT error: {e}")
            return self.robust_fallback(u0, bounds)
    
    def robust_fallback(self, u0, bounds):
        """Multiple fallback strategies"""
        strategies = [
            ('L-BFGS-B', {'maxiter': 200, 'ftol': 1e-10}),
            ('SLSQP', {'maxiter': 200, 'ftol': 1e-10}),
            ('trust-constr', {'maxiter': 100, 'verbose': 1})
        ]
        
        best_result = None
        best_cost = float('inf')
        
        for method_name, options in strategies:
            try:
                print(f"Trying {method_name}...")
                result = minimize(
                    self.enhanced_cost_function,
                    u0,
                    method=method_name,
                    bounds=bounds,
                    options=options
                )
                
                if result.fun < best_cost:
                    best_cost = result.fun
                    best_result = result
                    best_method = method_name
                    print(f"✓ {method_name} found better solution: {result.fun:.6f}")
                    
            except Exception as e:
                print(f"✗ {method_name} failed: {e}")
                continue
        
        if best_result is not None:
            u_physical = self.u_min + best_result.x.reshape(-1, 1) * (self.u_max - self.u_min)
            return u_physical, best_result, f"Fallback-{best_method}"
        else:
            # Last resort: use initial guess
            u_physical = self.u_min + u0.reshape(-1, 1) * (self.u_max - self.u_min)
            return u_physical, None, "Initial-Guess-Only"
    
    def analyze_performance_detailed(self, xs, us, energies):
        """Detailed performance analysis"""
        heights = xs[:, 0]
        velocities = xs[:, 1]
        
        peak_height = np.max(heights)
        avg_height = np.mean(heights)
        max_velocity = np.max(np.abs(velocities))
        min_height = np.min(heights)
        
        print("\n" + "="*60)
        print("DETAILED PERFORMANCE ANALYSIS")
        print("="*60)
        print(f"Peak height: {peak_height:.3f} m")
        print(f"Target height: {self.target_height:.3f} m")
        print(f"Achievement: {peak_height/self.target_height*100:.1f}%")
        print(f"Average height: {avg_height:.3f} m")
        print(f"Min height: {min_height:.3f} m")
        print(f"Max velocity: {max_velocity:.3f} m/s")
        
        if len(energies) > 0:
            print(f"Max energy: {np.max(energies):.1f} J")
            print(f"Energy efficiency: {peak_height/np.max(energies)*1000:.3f} m/kJ")
        
        # Detailed jump analysis
        takeoffs = np.where((velocities[:-1] <= 0) & (velocities[1:] > 0))[0]
        landings = np.where((velocities[:-1] > 0) & (velocities[1:] <= 0))[0]
        num_jumps = min(len(takeoffs), len(landings))
        
        print(f"Number of jumps: {num_jumps}")
        
        if num_jumps > 0:
            for i in range(num_jumps):
                if i < len(takeoffs) and i < len(landings):
                    start, end = takeoffs[i], landings[i]
                    if end > start:
                        jump_height = np.max(heights[start:end])
                        jump_duration = (end - start) * self.dt
                        print(f"  Jump {i+1}: {jump_height:.3f} m, {jump_duration:.3f} s")
        
        return peak_height, num_jumps

def main():
    """Main execution"""
    print("TUNED IPOPT VERTICAL HOPPER")
    print("="*60)
    print("Using enhanced physics and better IPOPT configuration")
    print("="*60)
    
    warnings.filterwarnings('ignore')
    
    hopper = TunedIPOPTHopper()
    
    # Optimize with tuned IPOPT
    us_physical, result, method = hopper.optimize_with_tuned_ipopt()
    
    # Simulate trajectory
    xs, energies = hopper.simulate_trajectory(us_physical)
    
    # Analyze performance
    peak_height, num_jumps = hopper.analyze_performance_detailed(xs, us_physical, energies)
    
    # Performance assessment
    print("\n" + "="*50)
    if peak_height >= 1.15:
        print("🎉 EXCELLENT! >95% target achievement!")
    elif peak_height >= 1.08:
        print("✅ VERY GOOD! >90% target achievement!")
    elif peak_height >= 0.96:
        print("⚠️  GOOD! >80% target achievement")
    else:
        print("🔧 NEEDS IMPROVEMENT")
        print("The optimization may need more iterations or better tuning.")
    
    print(f"Method: {method}")
    if result is not None:
        if hasattr(result, 'nfev'):
            print(f"Function evaluations: {result.nfev}")
        if hasattr(result, 'nit'):
            print(f"Iterations: {result.nit}")
        if hasattr(result, 'message'):
            print(f"Message: {result.message}")
    print("="*50)
    
    # Plot results
    time_xs = np.arange(len(xs)) * hopper.dt
    time_us = np.arange(len(us_physical)) * hopper.dt
    
    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 10))
    
    ax1.plot(time_xs, xs[:, 0], 'b-', linewidth=2)
    ax1.axhline(hopper.target_height, color='r', linestyle='--', linewidth=2)
    ax1.set_ylabel('Height (m)')
    ax1.set_title(f'Peak: {peak_height:.3f}m ({peak_height/hopper.target_height*100:.1f}%)')
    ax1.grid(True)
    ax1.set_ylim(0, 1.5)
    
    ax2.plot(time_xs, xs[:, 1], 'purple', linewidth=2)
    ax2.set_ylabel('Velocity (m/s)')
    ax2.grid(True)
    
    ax3.plot(time_us, us_physical, 'r-', linewidth=2)
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Control (m)')
    ax3.grid(True)
    
    if len(energies) > 0:
        time_energy = np.arange(len(energies)) * hopper.dt
        ax4.plot(time_energy, energies, 'orange', linewidth=2)
        ax4.set_xlabel('Time (s)')
        ax4.set_ylabel('Energy (J)')
        ax4.grid(True)
    
    plt.suptitle(f'Tuned IPOPT Optimization - {method}', fontsize=16)
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()