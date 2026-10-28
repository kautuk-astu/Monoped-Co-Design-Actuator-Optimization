"""Six-variable screening optimisation for the two-link MuJoCo monoped.

WARNING: For each gearbox topology, actuator mass and efficiency are held at
one COMPAct design point as the ratios change. This is a screening model,
NOT a validated ratio-dependent COMPAct optimisation. Motor constants are
unverified placeholders. Run COMPAct again for promising new ratios.
"""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
import cma
from jump_engine import evaluate, CSV_COLUMNS

ROOT = Path(__file__).resolve().parent
NAMES = ['hip_ratio', 'knee_ratio', 'hip_Kp', 'hip_Kd', 'knee_Kp', 'knee_Kd']
LOW = np.array([4., 4., 20., 0.1, 20., 0.1])
HIGH = np.array([25., 25., 900., 30., 900., 30.])

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--generations', type=int, default=25)
    p.add_argument('--population', type=int, default=12)
    p.add_argument('--duration', type=float, default=1.4)
    p.add_argument('--seed', type=int, default=42)
    p.add_argument('--candidate', default='all', help='all or a candidate name, e.g. D_WPG_DSPG')
    p.add_argument('--output', default='results_joint')
    args = p.parse_args()
    if args.generations < 1 or args.population < 4: p.error('Need generations>=1 and population>=4')
    motor = json.loads((ROOT/'motor_config.json').read_text())
    candidates = sorted((ROOT/'compact_candidates').glob('?_*.json'))
    if args.candidate != 'all':
        candidates = [path for path in candidates if path.stem == args.candidate]
    if not candidates: p.error('No matching compact_candidates JSON file')
    out = ROOT / args.output
    out.mkdir(parents=True, exist_ok=True)
    summary = []
    speed_max = float(motor['max_speed_rad_s'])
    current_max = float(motor['max_current_A'])
    eps = 1e-6

    for idx, path in enumerate(candidates):
        item = json.loads(path.read_text())
        hip, knee = item['actuators']['hip'], item['actuators']['knee']
        base_ratios = [float(hip['gear_ratio']), float(knee['gear_ratio'])]
        cfg = dict(motor, gearbox_efficiencies=[hip['efficiency'], knee['efficiency']],
                   actuator_masses_kg=[hip['actuator_mass_kg'], knee['actuator_mass_kg']])

        def decode(y):
            return LOW + np.clip(np.asarray(y), 0., 1.) * (HIGH - LOW)

        def feasible(metrics):
            return (metrics.get('valid_jump', False)
                    and metrics.get('overspeed_time_s', float('inf')) <= eps
                    and metrics.get('peak_motor_speed_rad_s', float('inf')) <= speed_max + eps
                    and metrics.get('peak_current_A', float('inf')) <= current_max + eps
                    and metrics.get('joint_limit_duration_s', float('inf')) <= eps)

        def ranking(x, metrics):
            # Feasible designs always beat infeasible ones. Infeasible candidates
            # are ranked by violation severity and then height to guide search.
            height = float(metrics.get('com_rise_m', 0.))
            energy = float(metrics.get('energy_proxy_J', 1e6))
            over = max(0., float(metrics.get('peak_motor_speed_rad_s', 1e6)) - speed_max)
            joint = float(metrics.get('joint_limit_duration_s', 1.))
            if feasible(metrics):
                return 10. + height - 0.00005 * energy
            return (height - 0.00005 * energy - 0.3
                    - 0.03 * over / speed_max
                    - 0.5 * float(metrics.get('overspeed_time_s', 0.))
                    - joint)

        # Start close to the fixed-ratio design and the prior best PD gains if available.
        initial = np.array(base_ratios + [220., 18., 700., 15.], dtype=float)
        previous = ROOT/'results_compact'/f'{path.stem}_best.json'
        if previous.exists():
            old = json.loads(previous.read_text())
            gains = old.get('pd_gains', {})
            initial = np.array(base_ratios + [gains.get('hip_Kp', 220.),
                gains.get('hip_Kd', 18.), gains.get('knee_Kp', 700.),
                gains.get('knee_Kd', 15.)], dtype=float)
        center = np.clip((initial-LOW)/(HIGH-LOW), 0.02, 0.98)
        es = cma.CMAEvolutionStrategy(center, 0.17, {
            'bounds': [0, 1], 'seed': args.seed + idx, 'popsize': args.population,
            'verbose': -9})
        best_any = None
        best_valid = None
        history = []

        def consider(x, metrics):
            nonlocal best_any, best_valid
            rank = ranking(x, metrics)
            candidate = (rank, x.copy(), metrics)
            if best_any is None or rank > best_any[0]: best_any = candidate
            if feasible(metrics) and (best_valid is None or rank > best_valid[0]):
                best_valid = candidate
            return rank

        # Explicitly evaluate the original design, even if CMA-ES misses it.
        _, m, _ = evaluate(initial, cfg, duration=args.duration)
        consider(initial, m)
        print(f'\nEvaluating {path.stem} (six-variable screening)', flush=True)
        for generation in range(1, args.generations + 1):
            ys = es.ask()
            losses = []
            for y in ys:
                x = decode(y)
                _, metrics, _ = evaluate(x, cfg, duration=args.duration)
                losses.append(-consider(x, metrics))
            es.tell(ys, losses)
            history.append([generation,
                '' if best_valid is None else best_valid[2]['com_rise_m'],
                best_any[0], int(best_valid is not None)])
            valid_height = 'none' if best_valid is None else f"{best_valid[2]['com_rise_m']:.4f} m"
            print(f'  generation {generation}/{args.generations}: best strictly feasible jump={valid_height}', flush=True)

        chosen = best_valid if best_valid is not None else best_any
        _, metrics, rows = evaluate(chosen[1], cfg, duration=args.duration, record=True)
        result = {
            'candidate': path.stem, 'strictly_feasible': feasible(metrics),
            'parameters': dict(zip(NAMES, map(float, chosen[1]))),
            'metrics': metrics, 'screening_score': chosen[0],
            'assumptions': {
                'ratio_dependent_COMPAct_model': False,
                'mass_efficiency': 'Held constant at the original COMPAct design point while varying ratios',
                'original_ratios': base_ratios,
                'actuator_masses_kg': cfg['actuator_masses_kg'],
                'gearbox_efficiencies': cfg['gearbox_efficiencies'],
                'motor_constants_verified': False,
                'gear_inertia_model': 'No ratio-dependent gearbox inertia',
                'strict_speed_constraint': speed_max,
                'strict_current_constraint': current_max,
            },
        }
        (out/f'{path.stem}_best.json').write_text(json.dumps(result, indent=2))
        with (out/f'{path.stem}_trajectory.csv').open('w', newline='') as f:
            w=csv.writer(f);w.writerow(CSV_COLUMNS);w.writerows(rows)
        with (out/f'{path.stem}_convergence.csv').open('w', newline='') as f:
            w=csv.writer(f);w.writerow(['generation','best_feasible_com_rise_m','best_screening_score','has_feasible']);w.writerows(history)
        summary.append({'candidate':path.stem,'strictly_feasible':feasible(metrics),
            **dict(zip(NAMES, map(float,chosen[1]))),
            'com_rise_m':metrics.get('com_rise_m',0.),
            'peak_motor_speed_rad_s':metrics.get('peak_motor_speed_rad_s'),
            'overspeed_time_s':metrics.get('overspeed_time_s'),
            'peak_current_A':metrics.get('peak_current_A'),
            'energy_proxy_J':metrics.get('energy_proxy_J'),
            'screening_score':chosen[0]})
    with (out/'comparison.csv').open('w', newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(summary[0]));w.writeheader();w.writerows(summary)
    print('\nRESULTS (screening only; COMPAct mass/efficiency not recomputed per ratio)')
    for row in sorted(summary,key=lambda r:(r['strictly_feasible'],r['screening_score']),reverse=True):
        print(f"{row['candidate']:15s} feasible={row['strictly_feasible']} "
              f"ratios={row['hip_ratio']:.2f}/{row['knee_ratio']:.2f} "
              f"rise={row['com_rise_m']:.4f}m peak_speed={row['peak_motor_speed_rad_s']:.1f}rad/s")
    print('Saved:', out/'comparison.csv')

if __name__ == '__main__': main()
