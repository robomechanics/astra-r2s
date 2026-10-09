# Fresh table-supported controller software proof

The complete suite passed **672 tests** in **88.97 seconds** (89.685 seconds including the proof wrapper). All **74 source/test files** and the mapped native MuJoCo library remained byte-identical before and after testing. The two existing Gym warnings concern unbounded observation-space limits. This proof qualifies software behavior, not thread capture, native contact convergence, hardware fidelity or a completed trajectory.

The new supported controller uses actual local crest return, a continuous 150 ms robot-yaw brake and fresh native quiet/load gates. Its right-arm inertia approximation has two transverse acceleration rows and three rotation rows, with no axial acceleration/position/lead row. Total PD plus feedforward retains the existing Cartesian and native motor limits. Feedforward is disabled for pickup, transport and open-hand movement. Existing carried-demo sources, scenes, thread geometry, core/plugin sources and legacy test bodies are preserved.

`software_proof.json` binds every exact archived source file, the command, runtime, wrapper and observed test count. `source_files/` contains those exact 74 files. `pytest_all.log` is the original complete output. The whole-suite count is separate from historical 402/281 proofs and the cold diagnostic's 67+6/31/69/50 reports; those results are not added to 672.

Verify this package from its directory:

```sh
sha256sum -c SHA256SUMS
```

The native runtime is the custom GCC MuJoCo 3.15.0 build with the recorded SDF search settings, invoked through `scripts/run_m8.sh`. Read `docs/m8_supported_agent_handoff.md` for installation, source pin, exact fresh command, force timing, archive restoration and physical acceptance limits. A supported Gym/policy wrapper is not supplied.

The included launcher accepts `--repository-root`, `--proof`, an absent `--output` and an exact `--producer`. `--prepare-only` checks source/runtime/HEAD identities without initializing or integrating a model. Without that option, it starts one fresh continuous table/rest-spawn trajectory and records source/runtime identities before and after native closure. A native failure remains a failure. The proof directory must contain the original three proof files; rebuilding on a different host requires generating a new local proof/runtime binding rather than relabeling its library as these recorded bytes.
