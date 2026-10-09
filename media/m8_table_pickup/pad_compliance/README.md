This separate quasistatic probe measures the left pad contact model. It uses the original archived table-acquisition scene and bounded native robot/jaw actuators, with no workpiece welds, object actuators or applied object wrenches. The M8 thread law, friction, contact margin, actuator caps and geometry remain unchanged.

Each case holds the saved end-acquisition configuration for 0.5 s. The report measures native normal force and force-weighted signed contact indentation over the final 0.1 s. The original finite actuators provide the load; these are separate probe integrations, not additional accepted segments of the actual demo.

| Normal law | Mean force per pad | Mean indentation | Local secant stiffness |
|---|---:|---:|---:|
| Original `solref=".0008 1"` | 16.000 N | 3.17 nm | 5.04 GN/m |
| Direct `solref="-31250 -2500"` | 15.997 N | 0.158 µm | 101 MN/m |

The direct law preserves the original normal damping coefficient, while reducing its normalized normal stiffness by 50×. Explicit `solreffriction=".0008 1"`, impedance `.9999 .9999 .0001`, friction coefficient 0.8 and zero margin preserve the original tangent law. The measured stiffness reduction is 49.8×. Both settled measurement windows have bilateral loading and zero solver warnings. The full physical lift/roll pilot still fails its [every-step bilateral-load diagnostic](../failures/roll_3s_direct_normal_50us_allsteps/validation.json).

These coefficients describe **numerical contact regularization**, not calibrated rubber. Direct coefficients have units of s⁻² and s⁻¹. Effective N/m depends on the native constraint inertia, impedance, contact point distribution and multiplicity. The reported force/indentation ratios are local secant observations of this pose. The probe does not measure stiffness across other poses, derivative stiffness, dynamic material behavior or hardware properties.

For an illustrative uniform-compression comparison only, a 9 × 5.5 mm face and 3 mm thickness with an assumed 10 MPa Young modulus would compress about 97 µm under 16 N, corresponding to 165 kN/m. The direct contact pilot is roughly 613× stiffer than that reference. Neither the assumed modulus nor the uniform-compression approximation is calibrated. The K≈51 extrapolation in [compliance_interpretation.json](compliance_interpretation.json) is **not a recommended operating setting**: retaining the original damping would produce an approximately 49 s slow normal mode. The later material-target pilot was rejected after actual jaw-backing contacts; it is not part of these two measured cases.

[Native force/depth measurements](native_pad_compliance_probe.json) include the final native contact rows and settled summary statistics. The complete probe history is not archived. [The exact original probe source](probe_source.py) and [manifest](manifest.json) bind the input trace, original scene XML/assets, native engine/plugin, unchanged Cartesian controller and frozen helper snapshots. Configuration helper snapshots were taken when packaging; later configuration fields do not regenerate the original scene.

To reproduce from the repository root with the verified native engine:

```bash
scripts/run_m8.sh media/m8_table_pickup/pad_compliance/reproduce_probe.py
```

The wrapper checks source/helper/runtime hashes and the byte-identical [published original trace](../failures/roll_3s_50us/trace.npz). It changes only root, input and output paths in the preserved probe source. Results are written to `outputs/m8_table_pickup/pad_compliance_reproduction/`. If repository helpers have changed, use the accompanying frozen helper snapshots before reproducing.
