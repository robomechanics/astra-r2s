# Recorded free-nut contact diagnostics

`validation.json` is the latest aggregation of eight actual probes, with every
original strict gate retained. It remains **failed** on contact-search travel
convergence: 40→80 seeds changes travel 2.477%, and the additional 80→160 probe
changes it 2.627%. All measured lead fits are within 2%; absolute travel under
an ideal applied torque is still sensitive to contact search.

The reverse probe was physically rerun at +100 µNm: +50 µNm against gravity
turned only 0.0867 revolutions, below the original >0.1-turn direction gate.
The stronger run turns 0.5343 revolutions with a 1.25354 mm/rev fit. This changes
the applied test load, not the thread model or acceptance thresholds.
`historical/` preserves the earlier failed aggregate and reverse trace.

Other cases retain their original plugin hash. Current/reference equivalence
is documented in [geometry optimization](../../docs/m8_sdf_performance.md),
including a fresh complete nominal trace matching byte for byte. Per-case
configuration, engine hashes, forces, warnings and NPZ traces remain available;
no old run is described as a fresh run. `followup.json` contains the additional
reverse and160-seed measurements. These are pre-engaged free-body world-wrench
unit probes, not physical hand or thread-starting tests. Reported SDF depth is
not an independently measured overlap bound; soft-contact elastic energy is
excluded from rigid-body energy diagnostics.
