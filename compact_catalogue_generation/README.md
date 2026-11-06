# COMPAct MAD_M6C12_KV170 catalogue generation

This directory generates isolated catalogues for SSPG, CPG, WPG, and DSPG at
0.1 requested-ratio spacing with 18 process workers. It does not write to the
COMPAct source repository or its existing `results` directory.

Run from this directory or from anywhere else with:

```bash
bash compact_catalogue_generation/run_catalogues.sh
```

By default, the launcher expects COMPAct to be a sibling of this repository.
For another location, set `COMPACT_REPO`:

```bash
COMPACT_REPO=/absolute/path/to/COMPAct bash compact_catalogue_generation/run_catalogues.sh
```

Each run is written under `outputs/run_<UTC timestamp>/`. It contains four
topology CSV files, one combined CSV, and `metadata.json`. Catalogue rows contain
the gearbox type, requested ratio, actual ratio, mass, efficiency, feasibility
status, and failure reason. Failed and infeasible evaluations remain in the CSV
files. CAD-equation writes are disabled in the private runtime copies. Temporary
COMPAct output and worker copies are deleted when the run ends; no CAD files are
generated or delivered.
