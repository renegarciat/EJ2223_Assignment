# Documentation index

| Doc | What it's for |
|---|---|
| [MATLAB_API_REFERENCE.md](MATLAB_API_REFERENCE.md) | Class-by-class reference for `MATLAB_Code/`: constructors, inputs, result properties, methods. Start here to know what a class does before reading its source. |
| [ipm_motor_pipeline_architecture.svg](ipm_motor_pipeline_architecture.svg) | Diagram of the sizing → geometry → COMSOL pipeline (embedded in the API reference and root README). |
| [rotor_bridge_stress_notes.md](rotor_bridge_stress_notes.md) | Working notes for the rotor-bridge structural check: paper sourcing, material-property provenance (datasheets), and an implementation log. |
| [bug_001.md](bug_001.md) | Rotor geometry can come out infeasible (`h_ob <= 0`) for some `AlphaM`/`Whr_fraction`/pole combinations — needs a clearer up-front validation check. |
| [bug_002.md](bug_002.md) | `EssonsSizer`'s stator bore estimate doesn't match the reference paper's worked example (~6% off); `IPMRotorSizer` has a `StatorBore_mm` override to sidestep it for equation validation. Open. |
| [course.txt](course.txt) | Raw transcript of an external reference course ("Design of V-Type IPM Motors," Ali Jamali Fard) kept for lookup/citation — not project-authored. |

For the project-level overview (what this repo is, how the pieces fit together, quick start), see the [root README](../README.md).
