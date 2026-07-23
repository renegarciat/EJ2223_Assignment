# EJ2223 — IPM Traction Motor Redesign

Coursework project (EJ2223) redesigning a traction motor as an **Interior
Permanent Magnet (IPM)** machine, starting from an existing motor
(`DD-14-10-POW`). Goal: same envelope/output as the reference motor, but
with the saliency inverted ($L_q > L_d$) for a wider field-weakening
range. The work spans analytical sizing (MATLAB), FEM validation
(COMSOL), and a written design report (LaTeX).

## Repository layout

| Path | Contents |
|---|---|
| `MATLAB_Code/` | Analytical sizing classes, COMSOL scripting, tests, plotting/diagnostic scripts. See [doc/MATLAB_API_REFERENCE.md](doc/MATLAB_API_REFERENCE.md). |
| `IPM_Design_Report/` | LaTeX source, figures, bibliography, and compiled PDF for the design report. |
| `COMSOL_models/` | Saved `.mph` FEM models. |
| `references/` | Course slides, the reference paper, and other source PDFs the sizing equations are based on. |
| `doc/` | Developer documentation: API reference, architecture diagram, known-issue logs, design-decision notes. See [doc/README.md](doc/README.md). |

## Design pipeline

<img src="doc/ipm_motor_pipeline_architecture.svg" alt="IPM motor design pipeline" width="480" />

1. **`MotorSpec`** — validated user inputs (torque, speeds, DC-link voltage, pole/slot count, airgap, magnet grade).
2. **`EssonsSizer`** and **`IPMRotorSizer`** — turn the spec into a stator/rotor geometry and key electrical parameters (inductances, PM flux linkage), following the analytical method of Di Gerlando & Ricca, *"Design Modeling and Sizing Equations of V-shape IPM Motors,"* ICEM 2022. `IPMRotorSizer` also runs a rotor-bridge centrifugal-stress check once the design converges.
3. **`MotorGeometry`** — collects the sizing results into COMSOL-ready dimensions.
4. **`ComsolInterface`** — builds/updates a 2D FEM sector model in COMSOL (geometry, materials, physics, mesh) and can run a study to extract torque.
5. **Design report** (`IPM_Design_Report/`) — documents the requirements, the analytical design, and validation against the FEM results.

Full class-by-class detail (constructors, inputs, results) is in
[doc/MATLAB_API_REFERENCE.md](doc/MATLAB_API_REFERENCE.md).

## Quick start

Run the full analytical sizing pipeline for the project's current design point:

```matlab
run('MATLAB_Code/main.m')
```

Run the test suite (validates the sizing equations against the reference paper's worked example):

```matlab
addpath('MATLAB_Code/tests')
run_all_tests
```

## Open questions / known limitations

- Whether the reference motor (`DD-14-10-POW`) is itself IPM or SPM is unresolved: rotor images suggest IPM, but it's been described as SPM with slight anisotropy accounting for $L_q \neq L_d$.
- `main.m`'s current design point (`poles=10, AlphaM=0.83, Whr_fraction=0.1`) fails rotor-geometry feasibility — see [doc/bug_001.md](doc/bug_001.md).
- `EssonsSizer`'s stator bore estimate doesn't match the reference paper's own worked example (~6% off) — see [doc/bug_002.md](doc/bug_002.md).
- The no-load saturation/leakage model (`IPMRotorSizer.computeSaturationModel_`) is a calibrated approximation, currently ~10–20% off the paper's reference values; everything downstream (stack length, torque) inherits that gap. See the diagnostics in `MATLAB_Code/tests/test_IPMRotorSizer.m`.

More background and sourcing for specific design decisions live in `doc/` — see [doc/README.md](doc/README.md) for the index.
