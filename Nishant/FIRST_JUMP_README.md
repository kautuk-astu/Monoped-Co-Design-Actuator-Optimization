# First powered jump diagnostic

Run in the extracted `two_link_codesign` directory:

```bash
source .venv/bin/activate
python first_jump.py
```

The script saves `results/first_jump.json` and `results/first_jump.csv`.
It starts with the confirmed crouch, holds briefly, and commands leg extension using PD control.
The foot-floor contact, normal force, positive-velocity takeoff, apex and landing are recorded.
**No successful jump is guaranteed.** The motor parameters are illustrative placeholders, not verified MAD MC12 data. The transmission is idealised and the energy is a proxy.
The original `optimize.py` is included unchanged; do not use its old objective for scientific conclusions until it is updated with the corrected takeoff logic and initial state.
