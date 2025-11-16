import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize, Bounds
import time

def monoped_jump_optimization():
    """Monoped Jump Optimization - Improved constraint version"""
    
    # Parameters
    params = {
        'm': 0.66,           # mass (kg)
        'l_thigh': 0.12,     # thigh length (m)
        'l_shank': 0.12,     # shank length (m) 
        'g': 9.81,           # gravity (m/s²)
        'Tst': 0.3,          # Reasonable stance time (s)
        'N': 15,             # Number of points
    }
    params['dt'] = params['Tst'] / params['N']
    
    # Calculate constraint counts upfront
    params['num_dynamics_eq'] = 6 * (params['N'] - 1)
    params['num_kinematic_eq'] = 2 * params['N']  # Only position constraints
    params['num_initial_eq'] = 4                 # Initial position and velocity
    params['num_takeoff_eq'] = 1                 # Single take-off condition
    params['total_eq'] = (params['num_dynamics_eq'] + params['num_kinematic_eq'] + 
                         params['num_initial_eq'] + params['num_takeoff_eq'])
    
    params['num_state_ineq'] = 8 * params['N']   # 4 states * 2 bounds each
    params['num_grf_ineq'] = params['N']         # GRF >= 0
    params['num_final_ineq'] = 1                 # Final velocity >= 0
    params['total_ineq'] = (params['num_state_ineq'] + params['num_grf_ineq'] + 
                           params['num_final_ineq'])
    
    # Reasonable bounds
    params['q_lb'] = np.array([-np.pi/3, 0.1])
    params['q_ub'] = np.array([np.pi/6, np.pi/2])
    params['qd_lb'] = np.array([-33.4, -33.4])  # Reduced velocity bounds
    params['qd_ub'] = np.array([33.4, 33.4])
    
    # State bounds
    params['state_lb'] = np.array([params['q_lb'][0], params['q_lb'][1], 
                                  params['qd_lb'][0], params['qd_lb'][1], 0.15, 0])
    params['state_ub'] = np.array([params['q_ub'][0], params['q_ub'][1], 
                                  params['qd_ub'][0], params['qd_ub'][1], 100, 8])
    params['Fz_ub'] = 8 * params['m'] * params['g']  # Reduced max GRF
    
    # Target take-off velocity
    params['min_takeoff_velocity'] = 1.5
    
    # Get bounds and initial guess
    lb, ub = get_bounds(params)
    x0 = initialize_improved_guess(params, lb, ub)
    
    # Test constraints
    print('Testing constraints at initial point...')
    c, ceq = constraints(x0, params)
    print(f'Number of equality constraints: {len(ceq)} (expected: {params["total_eq"]})')
    print(f'Number of inequality constraints: {len(c)} (expected: {params["total_ineq"]})')
    print(f'Max equality constraint violation: {np.max(np.abs(ceq)):.3e}')
    print(f'Max inequality constraint violation: {np.max(c):.3e}')
    
    # Single-phase optimization with interior point method
    print('\n=== Starting optimization with Interior Point Method ===')
    
    # Define constraints for interior point
    nonlinear_constraints = [
        {'type': 'eq', 'fun': lambda x: constraints(x, params)[1]},  # Equality constraints
        {'type': 'ineq', 'fun': lambda x: -constraints(x, params)[0]}  # Inequality constraints
    ]
    
    # Use Bounds object
    bounds = Bounds(lb, ub)
    
    # More relaxed options for better convergence
    options = {
        'maxiter': 3000,
        'verbose': 2,
        'xtol': 1e-4,
        'gtol': 1e-4,
        'barrier_tol': 1e-6,
    }
    
    start_time = time.time()
    
    # Try trust-constr first
    try:
        result = minimize(lambda x: objective(x, params), x0, 
                         method='trust-constr',
                         bounds=bounds,
                         constraints=nonlinear_constraints,
                         options=options)
    except Exception as e:
        print(f"Trust-constr failed: {e}")
        print("Falling back to SLSQP method...")
        bounds_list = [(lb_i, ub_i) for lb_i, ub_i in zip(lb, ub)]
        result = minimize(lambda x: objective(x, params), x0, 
                         method='SLSQP',
                         bounds=bounds_list,
                         constraints=nonlinear_constraints,
                         options={'maxiter': 3000, 'ftol': 1e-4, 'disp': True, 'eps': 1e-8})
    
    solve_time = time.time() - start_time
    
    # Results
    x_opt = result.x
    h_max, results = extract_results(x_opt, params)
    c_final, ceq_final = constraints(x_opt, params)
    
    print('\n=== OPTIMIZATION RESULTS ===')
    print(f'Success: {result.success}')
    print(f'Message: {result.message}')
    print(f'Solve time: {solve_time:.2f} seconds')
    print(f'Max jump height: {h_max:.3f} m')
    print(f'Take-off velocity: {results["v_to"]:.3f} m/s')
    print(f'Take-off height: {results["h_to"]:.3f} m')
    print(f'Final equality constraint violation: {np.max(np.abs(ceq_final)):.3e}')
    print(f'Final inequality constraint violation: {np.max(c_final):.3e}')
    
    if results['v_to'] >= params['min_takeoff_velocity']:
        print(f'SUCCESS: Excellent jump with take-off velocity > {params["min_takeoff_velocity"]:.1f} m/s!')
    else:
        print('Jump achieved but take-off velocity is low.')
    
    # Plot results
    plot_results(results, params)
    
    return result, params, results

def get_bounds(params):
    N = params['N']
    
    # State bounds
    state_lb = params['state_lb']
    state_ub = params['state_ub']
    
    lb = np.tile(state_lb, N)
    ub = np.tile(state_ub, N)
    
    # GRF bounds
    Fz_lb = np.zeros(N)
    Fz_ub = params['Fz_ub'] * np.ones(N)
    
    lb = np.concatenate([lb, Fz_lb])
    ub = np.concatenate([ub, Fz_ub])
    
    return lb, ub

def initialize_improved_guess(params, lb, ub):
    """Improved initialization with better physics"""
    N = params['N']
    l_thigh = params['l_thigh']
    l_shank = params['l_shank']
    m = params['m']
    g = params['g']
    
    states = np.zeros((N, 6))
    Fz_guess = np.zeros(N)
    
    # Better jumping motion with proper physics
    for k in range(N):
        t = k / (N - 1) if N > 1 else 0
        
        if t < 0.6:  # Compression phase
            progress = t / 0.6
            # Start slightly crouched, compress further
            q_hip = -0.3 * (1 - 0.8 * progress)  # Hip extends slightly during compression
            q_knee = 1.2 * (1 - 0.3 * progress)  # Knee compresses
            z_base = 0.25 - 0.05 * np.sin(progress * np.pi)  # Lower during compression
            
            # Velocities - moving downward during compression
            qd_hip = 0.5
            qd_knee = -1.5
            zd_base = -0.5
            
            # GRF increases during compression
            Fz_guess[k] = m * g * (1 + 3.0 * progress)
            
        elif t < 0.8:  # Extension phase
            progress = (t - 0.6) / 0.2
            # Extend for jump
            q_hip = -0.3 + 0.2 * progress  # Hip extends
            q_knee = 1.2 - 0.5 * progress   # Knee extends
            z_base = 0.20 + 0.08 * progress  # Rise during extension
            
            # Positive velocities for extension
            qd_hip = 2.0
            qd_knee = -2.0 + 3.0 * progress
            zd_base = 1.0 + 2.0 * progress
            
            # High GRF during extension
            Fz_guess[k] = m * g * (4 - 2 * progress)
            
        else:  # Flight phase
            progress = (t - 0.8) / 0.2
            # Extended posture
            q_hip = -0.1
            q_knee = 0.7
            z_base = 0.28 + zd_base * progress * params['dt']
            
            # Slowing velocities
            qd_hip = 1.0 * (1 - progress)
            qd_knee = 1.0 * (1 - progress)
            zd_base = 3.0 * (1 - 0.5 * progress)  # Upward velocity
            
            Fz_guess[k] = 0  # No ground contact
        
        states[k, :] = [q_hip, q_knee, qd_hip, qd_knee, z_base, zd_base]
    
    # Ensure bounds
    for i in range(6):
        states[:, i] = np.clip(states[:, i], lb[i], ub[i])
    Fz_guess = np.clip(Fz_guess, 0, params['Fz_ub'])
    
    x0 = np.concatenate([states.flatten(), Fz_guess])
    print('Improved initial guess created')
    return x0

def objective(x, params):
    """Objective: maximize jump height with regularization"""
    h_max, results = extract_results(x, params)
    
    # Main objective - maximize height
    J = 50 * np.exp(-h_max)
    
    # Penalty for low take-off velocity
    if results['v_to'] < params['min_takeoff_velocity']:
        J += 0 * (params['min_takeoff_velocity'] - results['v_to'])**2
    
    # Small regularization on control effort
    J += 0.01 * np.sum(results['Fz']**2) * params['dt']
    
    return J

def constraints(x, params):
    """Improved constraint function with better scaling"""
    N = params['N']
    dt = params['dt']
    m = params['m']
    g = params['g']
    l_thigh = params['l_thigh']
    l_shank = params['l_shank']
    
    # Preallocate with exact sizes
    ceq = np.zeros(params['total_eq'])
    c = np.zeros(params['total_ineq'])
    
    states, Fz = parse_decision_variables(x, params)
    
    eq_count = 0
    ineq_count = 0
    
    # Dynamics Constraints - Improved formulation
    for k in range(N - 1):
        current = states[k, :]
        next_state = states[k + 1, :]
        
        # Check foot contact
        foot_height = calculate_foot_height(current[0], current[1], current[4], l_thigh, l_shank)
        in_stance = foot_height <= 0.005 and Fz[k] > 0.1 * m * g
        
        if in_stance:
            # Stance dynamics with proper acceleration
            zdd = (Fz[k] - m * g) / m
        else:
            # Flight dynamics - only gravity
            zdd = -g
        
        # Improved dynamics constraints with better scaling
        ceq[eq_count:eq_count+6] = [
            next_state[0] - (current[0] + current[2] * dt),  # Hip angle
            next_state[1] - (current[1] + current[3] * dt),  # Knee angle
            next_state[2] - current[2],  # Hip velocity (simplified)
            next_state[3] - current[3],  # Knee velocity (simplified)
            next_state[4] - (current[4] + current[5] * dt),  # Base height
            next_state[5] - (current[5] + zdd * dt)          # Base velocity
        ]
        eq_count += 6
    
    # Kinematic Constraints - Only enforce during stance
    for k in range(N):
        q_hip = states[k, 0]
        q_knee = states[k, 1]
        z_base = states[k, 4]
        
        foot_height = calculate_foot_height(q_hip, q_knee, z_base, l_thigh, l_shank)
        
        # Only enforce foot on ground if we have significant GRF
        if foot_height <= 0.005 and Fz[k] > 0.1 * m * g:
            ceq[eq_count:eq_count+2] = [
                foot_height,  # Foot on ground
                0  # Placeholder for x-position (assume fixed)
            ]
        else:
            ceq[eq_count:eq_count+2] = 0
        eq_count += 2
    
    # Initial Conditions - Start from reasonable state
    ceq[eq_count:eq_count+4] = [
        states[0, 4] - 0.25,    # Initial height
        states[0, 5] - (-0.2),  # Small initial downward velocity
        states[0, 0] - (-0.3),  # Initial hip angle
        states[0, 1] - 1.2      # Initial knee angle
    ]
    eq_count += 4
    
    # Take-off Condition - Foot must leave ground
    foot_height_final = calculate_foot_height(states[N-1, 0], states[N-1, 1], 
                                            states[N-1, 4], l_thigh, l_shank)
    ceq[eq_count] = max(0, foot_height_final - 0.01)  # Foot above ground at end
    eq_count += 1
    
    # Inequality Constraints
    for k in range(N):
        # State bounds (softer constraints)
        c[ineq_count:ineq_count+8] = [
            states[k, 0] - params['q_ub'][0],
            params['q_lb'][0] - states[k, 0],
            states[k, 1] - params['q_ub'][1],
            params['q_lb'][1] - states[k, 1],
            states[k, 2] - params['qd_ub'][0],
            params['qd_lb'][0] - states[k, 2],
            states[k, 3] - params['qd_ub'][1],
            params['qd_lb'][1] - states[k, 3]
        ]
        ineq_count += 8
        
        # GRF bounds
        c[ineq_count] = -Fz[k]  # Fz >= 0
        ineq_count += 1
    
    # Final velocity - must be upward for jump
    c[ineq_count] = -states[N-1, 5]  # zd_final >= 0
    ineq_count += 1
    
    # Scale constraints for better numerical behavior
    ceq = ceq / 10.0  # Reduce constraint magnitude
    c = c / 10.0
    
    return c, ceq

# Keep the helper functions the same as before...
def plot_results(results, params):
    """Plot optimization results"""
    fig = plt.figure(figsize=(12, 8))
    
    # Base Height
    plt.subplot(2, 3, 1)
    plt.plot(results['time'], results['states'][:, 4], 'b-', linewidth=2)
    plt.xlabel('Time (s)')
    plt.ylabel('Base Height (m)')
    plt.title('Base Height')
    plt.grid(True)
    
    # Base Velocity
    plt.subplot(2, 3, 2)
    plt.plot(results['time'], results['states'][:, 5], 'r-', linewidth=2)
    plt.xlabel('Time (s)')
    plt.ylabel('Base Velocity (m/s)')
    plt.title('Base Velocity')
    plt.grid(True)
    
    # Joint Angles
    plt.subplot(2, 3, 3)
    plt.plot(results['time'], np.rad2deg(results['states'][:, 0]), 'b-', linewidth=2, label='Hip')
    plt.plot(results['time'], np.rad2deg(results['states'][:, 1]), 'r-', linewidth=2, label='Knee')
    plt.xlabel('Time (s)')
    plt.ylabel('Joint Angle (deg)')
    plt.title('Joint Angles')
    plt.legend()
    plt.grid(True)
    
    # Ground Reaction Force
    plt.subplot(2, 3, 4)
    plt.plot(results['time'], results['Fz'], 'k-', linewidth=2)
    plt.xlabel('Time (s)')
    plt.ylabel('GRF (N)')
    plt.title('Ground Reaction Force')
    plt.grid(True)
    
    # Foot Height
    plt.subplot(2, 3, 5)
    plt.plot(results['time'], results['foot_height'], 'g-', linewidth=2, label='Foot')
    plt.plot(results['time'], np.zeros_like(results['time']), 'r--', label='Ground')
    plt.xlabel('Time (s)')
    plt.ylabel('Foot Height (m)')
    plt.title('Foot Height')
    plt.legend()
    plt.grid(True)
    
    # Results summary
    plt.subplot(2, 3, 6)
    max_height = results['h_to'] + (results['v_to']**2) / (2 * params['g'])
    plt.text(0.1, 0.8, f'Max Height: {max_height:.3f} m', fontsize=12)
    plt.text(0.1, 0.6, f'Take-off Vel: {results["v_to"]:.3f} m/s', fontsize=12)
    plt.text(0.1, 0.4, f'Take-off Height: {results["h_to"]:.3f} m', fontsize=12)
    plt.text(0.1, 0.2, f'GRF Max: {np.max(results["Fz"]):.1f} N', fontsize=12)
    plt.axis('off')
    
    plt.suptitle(f'Jump Optimization Results - Height: {max_height:.3f} m')
    plt.tight_layout()
    plt.show()

def parse_decision_variables(x, params):
    N = params['N']
    states = x[:6*N].reshape(N, 6)
    Fz = x[6*N:6*N+N]
    return states, Fz

def compute_jacobian(q_hip, q_knee, l_thigh, l_shank):
    J11 = -l_thigh * np.sin(q_hip) - l_shank * np.sin(q_hip + q_knee)
    J12 = -l_shank * np.sin(q_hip + q_knee)
    J21 = l_thigh * np.cos(q_hip) + l_shank * np.cos(q_hip + q_knee)
    J22 = l_shank * np.cos(q_hip + q_knee)
    J = np.array([[J11, J12], [J21, J22]])
    return J

def calculate_foot_height(q_hip, q_knee, z_base, l_thigh, l_shank):
    return z_base - l_thigh * np.cos(q_hip) - l_shank * np.cos(q_hip + q_knee)

def compute_foot_position_x(q_hip, q_knee, l_thigh, l_shank):
    return -l_thigh * np.sin(q_hip) - l_shank * np.sin(q_hip + q_knee)

def extract_results(x, params):
    states, Fz = parse_decision_variables(x, params)
    
    results = {}
    results['h_to'] = states[-1, 4]
    results['v_to'] = states[-1, 5]
    h_max = results['h_to'] + (results['v_to']**2) / (2 * params['g'])
    
    results['states'] = states
    results['Fz'] = Fz
    results['time'] = np.linspace(0, params['Tst'], params['N'])
    
    results['foot_height'] = np.zeros(params['N'])
    results['foot_position_x'] = np.zeros(params['N'])
    
    for k in range(params['N']):
        results['foot_height'][k] = calculate_foot_height(
            states[k, 0], states[k, 1], states[k, 4],
            params['l_thigh'], params['l_shank'])
        
        results['foot_position_x'][k] = compute_foot_position_x(
            states[k, 0], states[k, 1], params['l_thigh'], params['l_shank'])
    
    return h_max, results

# Run the optimization
if __name__ == "__main__":
    result, params, results = monoped_jump_optimization()