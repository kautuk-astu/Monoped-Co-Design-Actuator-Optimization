import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize, Bounds
import time

def monoped_jump_optimization():
    """Monoped Jump Optimization - Simplified version with trust-constr"""
    
    # Parameters
    params = {
        'm': 0.66,           # mass (kg)
        'l_thigh': 0.12,     # thigh length (m)
        'l_shank': 0.12,     # shank length (m) 
        'g': 9.81,           # gravity (m/s²)
        'N': 10,             # Number of points
    }
    
    # Fixed stance time for now
    params['Tst'] = 0.25
    
    # Parameters for dynamics
    params['I_hip'] = 0.01   # Hip moment of inertia (kg·m²)
    params['I_knee'] = 0.005 # Knee moment of inertia (kg·m²)
    params['tau_hip_max'] = 15.0  # Max hip torque (N·m)
    params['tau_knee_max'] = 10.0 # Max knee torque (N·m)
    
    # SIMPLIFIED: Only optimize joint angles and GRF
    params['num_states_per_step'] = 2  # ONLY q_hip, q_knee
    params['num_initial_states'] = 2   # z_base_0, zd_base_0
    
    # SIMPLIFIED: Reduced constraint counts
    params['num_dynamics_eq'] = 4 * (params['N'] - 1)  # Only joint dynamics
    params['num_kinematic_eq'] = 1 * params['N']       # Only foot height
    params['num_torque_dynamics_eq'] = 0               # REMOVED for now
    params['num_takeoff_eq'] = 1                       # Single take-off condition
    params['total_eq'] = (params['num_dynamics_eq'] + params['num_kinematic_eq'] + 
                         params['num_takeoff_eq'])
    
    params['num_state_ineq'] = 4 * params['N']         # 2 states * 2 bounds each
    params['num_grf_ineq'] = params['N']               # GRF >= 0
    params['num_torque_ineq'] = 0                      # REMOVED for now
    params['num_final_ineq'] = 1                       # Final velocity >= 0
    params['total_ineq'] = (params['num_state_ineq'] + params['num_grf_ineq'] + 
                           params['num_final_ineq'])
    
    # Reasonable bounds
    params['q_lb'] = np.array([-0.8, 0.3])    # More conservative bounds
    params['q_ub'] = np.array([0.3, 1.8])
    
    # State bounds - only for the 2 states we're optimizing directly
    params['state_lb'] = np.array([params['q_lb'][0], params['q_lb'][1]])
    params['state_ub'] = np.array([params['q_ub'][0], params['q_ub'][1]])
    
    # Initial state bounds
    params['initial_z_lb'] = np.array([0.20, -0.5])
    params['initial_z_ub'] = np.array([0.30, 0.0])
    
    params['Fz_ub'] = 4 * params['m'] * params['g']   # Reduced max GRF
    
    # Target take-off velocity
    params['min_takeoff_velocity'] = 1.5
    
    # Get bounds and initial guess
    lb, ub = get_bounds(params)
    x0 = initialize_feasible_guess(params, lb, ub)
    
    # Test constraints at initial point
    print('Testing constraints at initial point...')
    c, ceq = constraints(x0, params)
    print(f'Number of equality constraints: {len(ceq)} (expected: {params["total_eq"]})')
    print(f'Number of inequality constraints: {len(c)} (expected: {params["total_ineq"]})')
    print(f'Max equality constraint violation: {np.max(np.abs(ceq)):.3e}')
    print(f'Max inequality constraint violation: {np.max(c):.3e}')
    
    # If initial guess has large violations, fix them
    if np.max(np.abs(ceq)) > 1.0 or np.max(c) > 1.0:
        print("Improving initial guess feasibility...")
        x0 = improve_initial_guess(x0, params, lb, ub)
        c, ceq = constraints(x0, params)
        print(f'After improvement - Max eq violation: {np.max(np.abs(ceq)):.3e}, Max ineq violation: {np.max(c):.3e}')
    
    print('\n=== Starting optimization with trust-constr ===')
    
    # Define constraints for trust-constr
    nonlinear_constraints = [
        {'type': 'eq', 'fun': lambda x: constraints(x, params)[1]},
        {'type': 'ineq', 'fun': lambda x: -constraints(x, params)[0]}
    ]
    
    bounds = Bounds(lb, ub)
    
    # trust-constr specific options
    options = {
        'maxiter': 500,
        'verbose': 2,
        'xtol': 1e-4,
        'gtol': 1e-4,
        'barrier_tol': 1e-6,
    }
    
    start_time = time.time()
    
    try:
        # Use trust-constr method
        result = minimize(lambda x: objective(x, params), x0,
                         method='trust-constr', 
                         bounds=bounds,
                         constraints=nonlinear_constraints,
                         options=options)
        
    except Exception as e:
        print(f"trust-constr failed: {e}")
        print("Falling back to improved initial guess...")
        result = type('obj', (object,), {'x': x0, 'success': False, 'message': 'Fallback to initial guess'})()
    
    solve_time = time.time() - start_time
    
    # Results
    x_opt = result.x if hasattr(result, 'x') else x0
    h_max, results = extract_results(x_opt, params)
    c_final, ceq_final = constraints(x_opt, params)
    
    print('\n=== OPTIMIZATION RESULTS ===')
    print(f'Success: {getattr(result, "success", False)}')
    print(f'Message: {getattr(result, "message", "Custom fallback")}')
    print(f'Solve time: {solve_time:.2f} seconds')
    print(f'Stance time: {params["Tst"]:.3f} s')
    print(f'Max jump height: {h_max:.3f} m')
    print(f'Take-off velocity: {results["v_to"]:.3f} m/s')
    print(f'Take-off height: {results["h_to"]:.3f} m')
    print(f'Initial height: {results["states"][0, 4]:.3f} m')
    print(f'Initial velocity: {results["states"][0, 5]:.3f} m/s')
    print(f'Final equality constraint violation: {np.max(np.abs(ceq_final)):.3e}')
    print(f'Final inequality constraint violation: {np.max(c_final):.3e}')
    
    if results['v_to'] >= params['min_takeoff_velocity']:
        print(f'SUCCESS: Excellent jump with take-off velocity > {params["min_takeoff_velocity"]:.1f} m/s!')
    elif results['v_to'] > 0:
        print(f'Partial success: Positive take-off velocity ({results["v_to"]:.3f} m/s)')
    else:
        print(f'Failed: Negative take-off velocity ({results["v_to"]:.3f} m/s)')
    
    # Plot results
    plot_results(results, params)
    
    return result, params, results

def get_bounds(params):
    """Bounds for simplified optimization variables"""
    N = params['N']
    
    # Bounds for joint angles only
    state_lb = params['state_lb']
    state_ub = params['state_ub']
    
    lb_states = np.tile(state_lb, N)
    ub_states = np.tile(state_ub, N)
    
    # Bounds for initial z states
    lb_initial = params['initial_z_lb']
    ub_initial = params['initial_z_ub']
    
    # Bounds for GRF
    Fz_lb = np.zeros(N)
    Fz_ub = params['Fz_ub'] * np.ones(N)
    
    # Combine all bounds
    lb = np.concatenate([lb_states, lb_initial, Fz_lb])
    ub = np.concatenate([ub_states, ub_initial, Fz_ub])
    
    return lb, ub

def initialize_feasible_guess(params, lb, ub):
    """Create a physically feasible initial guess"""
    N = params['N']
    m = params['m']
    g = params['g']
    
    # Initialize joint angles only
    states_joints = np.zeros((N, 2))  # Only q_hip, q_knee
    
    # Reasonable initial conditions
    initial_z = np.array([0.25, -0.2])  # Start at reasonable height with small downward velocity
    
    Fz_guess = np.zeros(N)
    
    # Create a simple crouch-and-jump motion that satisfies constraints
    for k in range(N):
        t = k / (N - 1) if N > 1 else 0
        
        if t < 0.7:  # Compression phase
            progress = t / 0.7
            # Crouch down
            q_hip = -0.4 * (1 - 0.3 * progress)  # Hip angle
            q_knee = 1.0 + 0.3 * progress        # Knee angle (more bent)
            Fz_guess[k] = m * g * (1 + 1.0 * progress)
            
        else:  # Extension phase
            progress = (t - 0.7) / 0.3
            # Push off
            q_hip = -0.28 + 0.2 * progress       # Hip extends
            q_knee = 1.3 - 0.5 * progress        # Knee extends
            Fz_guess[k] = m * g * (2.0 - 1.5 * progress)
        
        states_joints[k, :] = [q_hip, q_knee]
    
    # Ensure bounds
    for i in range(2):
        states_joints[:, i] = np.maximum(lb[i], np.minimum(ub[i], states_joints[:, i]))
    
    initial_z = np.maximum(params['initial_z_lb'], np.minimum(params['initial_z_ub'], initial_z))
    Fz_guess = np.maximum(0, np.minimum(params['Fz_ub'], Fz_guess))
    
    x0 = np.concatenate([states_joints.flatten(), initial_z, Fz_guess])
    
    return x0

def improve_initial_guess(x0, params, lb, ub):
    """Improve initial guess to satisfy constraints better"""
    print("Running constraint satisfaction for initial guess...")
    
    # Objective: minimize constraint violations
    def constraint_violation(x):
        c, ceq = constraints(x, params)
        return np.sum(np.maximum(0, c)**2) + np.sum(ceq**2)
    
    # Use trust-constr to improve feasibility
    constraints_list = [
        {'type': 'eq', 'fun': lambda x: constraints(x, params)[1]},
        {'type': 'ineq', 'fun': lambda x: -constraints(x, params)[0]}
    ]
    
    result = minimize(constraint_violation, x0, 
                     method='trust-constr',
                     bounds=Bounds(lb, ub),
                     constraints=constraints_list,
                     options={'maxiter': 100, 'verbose': 0})
    
    if result.success and constraint_violation(result.x) < constraint_violation(x0):
        print("Initial guess improved successfully")
        return result.x
    else:
        print("Could not improve initial guess, using original")
        return x0

def objective(x, params):
    """Objective function focused on jump height"""
    h_max, results = extract_results(x, params)
    
    # Main objective - maximize height
    J = 100 * np.exp(-h_max)
    
    # Strong penalty for negative take-off velocity
    if results['v_to'] < 0:
        J += 50 * (-results['v_to'])
    
    # Penalty for low take-off velocity
    if results['v_to'] < params['min_takeoff_velocity']:
        J += 10 * (params['min_takeoff_velocity'] - results['v_to'])**2
    
    # Small regularization
    J += 0.001 * np.sum(results['Fz']**2) * params['Tst'] / params['N']
    
    return J

def constraints(x, params):
    """Simplified constraints"""
    N = params['N']
    dt = params['Tst'] / (N - 1) if N > 1 else params['Tst']
    m = params['m']
    g = params['g']
    l_thigh = params['l_thigh']
    l_shank = params['l_shank']
    
    # Preallocate constraints
    ceq = np.zeros(params['total_eq'])
    c = np.zeros(params['total_ineq'])
    
    # Parse variables
    states_joints, initial_z, Fz = parse_decision_variables(x, params)
    
    # Reconstruct full states
    states_full = reconstruct_full_states(states_joints, initial_z, Fz, params)
    
    eq_count = 0
    ineq_count = 0
    
    # SIMPLIFIED Dynamics Constraints
    for k in range(N - 1):
        current = states_full[k, :]
        next_state = states_full[k + 1, :]
        
        # Simple position continuity
        ceq[eq_count:eq_count+4] = [
            (states_joints[k+1, 0] - states_joints[k, 0]) / 0.1,  # Hip angle continuity
            (states_joints[k+1, 1] - states_joints[k, 1]) / 0.1,  # Knee angle continuity
            (next_state[4] - (current[4] + current[5] * dt)) / 0.1,  # Base height
            (next_state[5] - (current[5] + calculate_base_acceleration(current, Fz[k], params) * dt)) / 0.1  # Base velocity
        ]
        eq_count += 4
    
    # Kinematic Constraints - foot on ground during stance
    for k in range(N):
        q_hip = states_full[k, 0]
        q_knee = states_full[k, 1]
        z_base = states_full[k, 4]
        
        foot_height = calculate_foot_height(q_hip, q_knee, z_base, l_thigh, l_shank)
        
        # Foot should be on ground when GRF is significant
        if Fz[k] > 0.1 * m * g:
            ceq[eq_count] = foot_height / 0.01  # Foot on ground
        else:
            ceq[eq_count] = 0
        eq_count += 1
    
    # Take-off Condition - foot should leave ground at the end
    foot_height_final = calculate_foot_height(states_full[N-1, 0], states_full[N-1, 1], 
                                            states_full[N-1, 4], l_thigh, l_shank)
    ceq[eq_count] = max(0, foot_height_final - 0.01) / 0.01
    eq_count += 1
    
    # Inequality Constraints
    for k in range(N):
        # State bounds
        c[ineq_count:ineq_count+4] = [
            (states_joints[k, 0] - params['q_ub'][0]) / 0.1,
            (params['q_lb'][0] - states_joints[k, 0]) / 0.1,
            (states_joints[k, 1] - params['q_ub'][1]) / 0.1,
            (params['q_lb'][1] - states_joints[k, 1]) / 0.1
        ]
        ineq_count += 4
        
        # GRF bounds
        c[ineq_count] = -Fz[k] / 10.0
        ineq_count += 1
    
    # Final velocity - must be upward for jump
    c[ineq_count] = (-states_full[N-1, 5]) / 1.0
    ineq_count += 1
    
    return c, ceq

def parse_decision_variables(x, params):
    """Parse simplified decision variables"""
    N = params['N']
    
    # Joint angles only (q_hip, q_knee) for all time steps
    states_joints = x[:2*N].reshape(N, 2)
    
    # Initial z states (z_base_0, zd_base_0)
    initial_z = x[2*N:2*N+2]
    
    # GRF for all time steps
    Fz = x[2*N+2:2*N+2+N]
    
    return states_joints, initial_z, Fz

def reconstruct_full_states(states_joints, initial_z, Fz, params):
    """Reconstruct full states from simplified variables"""
    N = params['N']
    dt = params['Tst'] / (N - 1) if N > 1 else params['Tst']
    
    states_full = np.zeros((N, 6))
    
    # Set initial state
    states_full[0, 0] = states_joints[0, 0]  # q_hip
    states_full[0, 1] = states_joints[0, 1]  # q_knee
    states_full[0, 4] = initial_z[0]         # z_base
    states_full[0, 5] = initial_z[1]         # zd_base
    
    # Compute velocities from finite differences
    for k in range(1, N):
        states_full[k, 0] = states_joints[k, 0]  # q_hip
        states_full[k, 1] = states_joints[k, 1]  # q_knee
        states_full[k, 2] = (states_joints[k, 0] - states_joints[k-1, 0]) / dt  # qd_hip
        states_full[k, 3] = (states_joints[k, 1] - states_joints[k-1, 1]) / dt  # qd_knee
    
    # Compute base states from dynamics
    for k in range(1, N):
        current = states_full[k-1, :]
        zdd = calculate_base_acceleration(current, Fz[k-1], params)
        states_full[k, 4] = current[4] + current[5] * dt  # z_base
        states_full[k, 5] = current[5] + zdd * dt         # zd_base
    
    return states_full

def calculate_base_acceleration(state, Fz, params):
    """Calculate base acceleration from GRF"""
    foot_height = calculate_foot_height(state[0], state[1], state[4], 
                                      params['l_thigh'], params['l_shank'])
    
    # If foot is on ground and we have GRF, use stance dynamics
    if foot_height <= 0.005 and Fz > 0.1 * params['m'] * params['g']:
        return (Fz - params['m'] * params['g']) / params['m']
    else:
        return -params['g']  # Free fall

def calculate_foot_height(q_hip, q_knee, z_base, l_thigh, l_shank):
    return z_base - l_thigh * np.cos(q_hip) - l_shank * np.cos(q_hip + q_knee)

def extract_results(x, params):
    """Extract results from simplified variables"""
    states_joints, initial_z, Fz = parse_decision_variables(x, params)
    states_full = reconstruct_full_states(states_joints, initial_z, Fz, params)
    
    results = {}
    results['Tst'] = params['Tst']
    results['dt'] = params['Tst'] / params['N']
    results['h_to'] = states_full[-1, 4]
    results['v_to'] = states_full[-1, 5]
    h_max = results['h_to'] + (results['v_to']**2) / (2 * params['g'])
    
    results['states'] = states_full
    results['Fz'] = Fz
    results['time'] = np.linspace(0, params['Tst'], params['N'])
    
    results['foot_height'] = np.zeros(params['N'])
    for k in range(params['N']):
        results['foot_height'][k] = calculate_foot_height(
            states_full[k, 0], states_full[k, 1], states_full[k, 4],
            params['l_thigh'], params['l_shank'])
    
    return h_max, results

def plot_results(results, params):
    """Plot results"""
    fig, axes = plt.subplots(2, 3, figsize=(15, 10))
    
    # Base Height
    axes[0,0].plot(results['time'], results['states'][:, 4], 'b-', linewidth=2)
    axes[0,0].set_xlabel('Time (s)')
    axes[0,0].set_ylabel('Base Height (m)')
    axes[0,0].set_title('Base Height')
    axes[0,0].grid(True)
    
    # Base Velocity
    axes[0,1].plot(results['time'], results['states'][:, 5], 'r-', linewidth=2)
    axes[0,1].set_xlabel('Time (s)')
    axes[0,1].set_ylabel('Base Velocity (m/s)')
    axes[0,1].set_title('Base Velocity')
    axes[0,1].grid(True)
    
    # Joint Angles
    axes[0,2].plot(results['time'], np.rad2deg(results['states'][:, 0]), 'b-', linewidth=2, label='Hip')
    axes[0,2].plot(results['time'], np.rad2deg(results['states'][:, 1]), 'r-', linewidth=2, label='Knee')
    axes[0,2].set_xlabel('Time (s)')
    axes[0,2].set_ylabel('Joint Angle (deg)')
    axes[0,2].set_title('Joint Angles')
    axes[0,2].legend()
    axes[0,2].grid(True)
    
    # Ground Reaction Force
    axes[1,0].plot(results['time'], results['Fz'], 'k-', linewidth=2)
    axes[1,0].set_xlabel('Time (s)')
    axes[1,0].set_ylabel('GRF (N)')
    axes[1,0].set_title('Ground Reaction Force')
    axes[1,0].grid(True)
    
    # Foot Height
    axes[1,1].plot(results['time'], results['foot_height'], 'g-', linewidth=2, label='Foot')
    axes[1,1].plot(results['time'], np.zeros_like(results['time']), 'r--', label='Ground')
    axes[1,1].set_xlabel('Time (s)')
    axes[1,1].set_ylabel('Foot Height (m)')
    axes[1,1].set_title('Foot Height')
    axes[1,1].legend()
    axes[1,1].grid(True)
    
    # Results summary
    axes[1,2].axis('off')
    max_height = results['h_to'] + (results['v_to']**2) / (2 * params['g'])
    text_str = f'Max Height: {max_height:.3f} m\nTake-off Vel: {results["v_to"]:.3f} m/s\nTake-off Height: {results["h_to"]:.3f} m\nStance Time: {results["Tst"]:.3f} s'
    axes[1,2].text(0.1, 0.5, text_str, fontsize=12, verticalalignment='center')
    
    plt.suptitle(f'Jump Optimization Results - trust-constr method')
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    result, params, results = monoped_jump_optimization()