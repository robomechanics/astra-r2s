"""Publish one exact still and a byte-exact native progress prefix; no fake validation."""
from pathlib import Path
import hashlib,json,shutil
ROOT=Path('/workspace/astra-r2s')
source=ROOT/'outputs/m8_table_supported/progress_bolt_over_bore'
render=ROOT/'outputs/m8_table_supported/progress_bolt_over_bore_render'
target=ROOT/'media/m8_table_supported/progress_bolt_over_bore'
if target.exists():raise FileExistsError(target)
sha=lambda v:hashlib.sha256(v).hexdigest()
files={}
for name in ('trace.npz','prefix_identity.json','scene.xml','supported_scene.zip','controller_source.py',
 'scene_source.py','engagement_observer_source.py','software_tests.json','software_tests.log',
 'manifest.json','verify_sources.py','run_publication_identity.json'):
 label='software_manifest.json' if name=='manifest.json' else name
 files[label]=(source/name).read_bytes()
files['recorded_app_renderer_source.py']=(source/'renderer_source.py').read_bytes()
for subtree in ('recorded_sources','frozen_audit_sources'):
 for path in sorted((source/subtree).rglob('*')):
  if path.is_file():files[str(path.relative_to(source))]=path.read_bytes()
for path in sorted(render.rglob('*')):
 if path.is_file():files[str(path.relative_to(render))]=path.read_bytes()
identity=json.loads(files['prefix_identity.json'])
manifest=json.loads(files['render_manifest.json'])
assert identity['trajectory_sha256']==sha(files['trace.npz'])==manifest['trajectory_sha256']
assert identity['screenshot_sample_index']==manifest['screenshot_sample_index']
assert identity['selected_qpos_sha256']==manifest['screenshot_qpos_sha256']
assert identity['selected_qvel_sha256']==manifest['screenshot_qvel_sha256']
assert identity['original_selected_sample']==manifest['original_saved_sample']
assert manifest['video_frames']==0 and identity['screenshot_phase']=='align_over_hole'
assert not manifest['original_saved_sample']['thread_engaged']
assert manifest['original_saved_sample']['thread_contacts']==0
for name,raw in files.items():
 if name.endswith('.json'):json.loads(raw,parse_constant=lambda token: (_ for _ in ()).throw(ValueError(token)))
files['prefix_snapshot_source.py']=(ROOT/'outputs/m8_table_supported/create_supported_prefix.py').read_bytes()
files['prefix_package_source.py']=Path(__file__).read_bytes()
files['README.md']=('''# Fresh native bolt pickup and alignment progress\n\nThis exact saved state belongs to the fresh, continuing `full_v2` trial. At\n4.82 s native physics, the right fingers have picked up the separate bolt,\ntransported its complete shaft clear of the original three-pin rest, and\naligned it above the M8 female bore. The left fingers stabilize the free block\non the plain solid table. The saved state still has no thread contact or\nformed-thread capture. No qualified turn or completed trajectory is claimed.\n\n`demo.png` replays one actual postintegration qpos/qvel row using native\n`mj_forward` only, with no integration, interpolation or manual free-body edits.\nIts force captions come from the original solved sample immediately before\nintegration, at saved state time minus timestep. Table/left force ledgers remain\nopen in the continuing parent trial; this screenshot does not certify whole-task\nload retention. The archived 213-test producer proof is separate from physics\nqualification and applies to the exact source revision recorded here.\n\n- `trace.npz`: byte-exact, closed-readable parent phase-prefix snapshot.\n- `prefix_identity.json`: source/model/controller IDs, endpoint, selected native state\n  hashes and original solved force sample. Its original `partial` field describes\n  requested phase selection; this copied prefix is incomplete regardless.\n- `render_manifest.json`, `renderer_source.py` and `renderer_sources/`: exact replay\n  sources, selected state and image hashes, and native runtime identity.\n- `scene.xml`, `supported_scene.zip`, `scene_source.py`, `controller_source.py` and\n  `recorded_sources/`: exact recorded native model/source dependency snapshots.\n- `software_tests.json`, `software_tests.log`, `software_manifest.json` and\n  `verify_sources.py`: original historical 213-test producer proof, unchanged.\n- `frozen_audit_sources/`: auditor sources frozen for the parent trial; no complete\n  independent physical audit is claimed or fabricated in this progress package.\n- `SHA256SUMS`: all primary and supporting files.\n\nThe original native parent trajectory continues without restoring or splicing this\ncheckpoint. After it closes, this prefix can be compared against its final rows;\nits prefix-file SHA remains distinct from the eventual whole final trace SHA.\nThe first carried-block demonstration and earlier supported pilot/failure archives\nremain unchanged. Replaying requires the matched native CPU engine described in\n`docs/m8_setup.md`; stock MuJoCo is insufficient for this M8 evidence.\n''').encode()
target.mkdir(parents=True)
for name,raw in files.items():
 path=target/name;path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes(raw)
(target/'SHA256SUMS').write_text(''.join(f'{sha(raw)}  {name}\n' for name,raw in sorted(files.items())))
for name,raw in files.items():assert (target/name).read_bytes()==raw
print(json.dumps({'folder':str(target.relative_to(ROOT)),'files':len(files),'bytes':sum(map(len,files.values())),
 'trace_sha256':identity['trajectory_sha256'],'screenshot_time_s':identity['screenshot_time_s'],
 'screenshot_phase':identity['screenshot_phase'],'thread_contacts':0,'capture':False,
 'sample_original_table_upward_force_N':manifest['original_saved_sample']['table_support']['table_upward_force_N'],
 'sample_original_right_pad_normal_force_N':manifest['original_saved_sample']['contact']['pad_normal_force_N']},indent=2))
