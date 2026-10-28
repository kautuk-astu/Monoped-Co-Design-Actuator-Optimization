"""Run a single jump with corrected flight segmentation and post-step contact forces."""
import argparse,csv,json
from pathlib import Path
from jump_engine import evaluate,CSV_COLUMNS
ROOT=Path(__file__).resolve().parent

def main():
    p=argparse.ArgumentParser()
    p.add_argument('--duration',type=float,default=1.4)
    p.add_argument('--hold',type=float,default=.10)
    p.add_argument('--push',type=float,default=.16)
    p.add_argument('--min-flight',type=float,default=.08)
    p.add_argument('--hip-ratio',type=float,default=10.)
    p.add_argument('--knee-ratio',type=float,default=10.)
    p.add_argument('--hip-kp',type=float,default=350.)
    p.add_argument('--hip-kd',type=float,default=12.)
    p.add_argument('--knee-kp',type=float,default=350.)
    p.add_argument('--knee-kd',type=float,default=12.)
    a=p.parse_args()
    if not (4<=a.hip_ratio<=25 and 4<=a.knee_ratio<=25):p.error('Ratios must be 4..25')
    cfg=json.loads((ROOT/'motor_config.json').read_text())
    x=[a.hip_ratio,a.knee_ratio,a.hip_kp,a.hip_kd,a.knee_kp,a.knee_kd]
    score,metrics,rows=evaluate(x,cfg,a.duration,a.hold,a.push,a.min_flight,record=True)
    out=ROOT/'results';out.mkdir(exist_ok=True)
    (out/'first_jump.json').write_text(json.dumps(metrics,indent=2))
    with (out/'first_jump.csv').open('w',newline='') as f:
        w=csv.writer(f);w.writerow(CSV_COLUMNS);w.writerows(rows)
    print(json.dumps(metrics,indent=2))
    print('Saved results/first_jump.csv and results/first_jump.json')
if __name__=='__main__':main()
