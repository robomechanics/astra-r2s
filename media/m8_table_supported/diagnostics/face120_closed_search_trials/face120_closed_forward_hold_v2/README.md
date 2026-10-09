# face120_closed_forward_hold_v2

Separate **cold local trial**, native duration 2.53060 s.
Original guards held: False. Original abort:
`{"elapsed_s": 2.5306, "failed_checks": ["bilateral_left_pad_load"]}`. Original termination:
`None`. **Zero formed overlap or loaded interior
contact; no opening/reset/full trajectory qualification.**

![Actual cold entry state](render/entry_detail.png)

[Normal 1x MP4](render/demo.mp4) · [Normal 1x GIF](render/demo.gif) ·
[Actual endpoint state](render/endpoint_detail.png)

Media uses exact saved qpos/qvel and two fixed real cameras, with `mj_forward`
only. Caption forces are original native solve records at logged time minus dt;
saved poses are postintegration. Physical occlusion remains. No body hiding,
interpolation, geometry change, free-body posing or physics integration occurs.

Original reports, raw all-step ledger, state archive, declared cold
initialization, stdout/stderr log, harness/helper sources and audits are
retained. No correction applied to original counter; independently verified origin.

Use the [package instructions](../README.md) for exact cold dependencies,
canonical model/grip parent, actual state parent, commands and proof scopes.
