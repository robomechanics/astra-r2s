from pathlib import Path
import sys,json,hashlib,numpy as np
kind=sys.argv[1];run=Path(sys.argv[2]);prefix=Path(sys.argv[3]);metadata=Path(sys.argv[4]) if len(sys.argv)>4 else None
root=Path('media/m8_table_pickup');final=run/'insertion_trace.npz'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def load(p):
 with np.load(p,allow_pickle=False) as z:
  arr={k:z[k].copy() for k in ('time','qpos','qvel','controller')};rows=json.loads(str(z['info_json']));report=json.loads(str(z['metadata_json']))
 return arr,rows,report
f,fr,report=load(final);q,qr,pr=load(prefix);n=len(q['time'])
assert n<=len(f['time'])
for key in q:assert np.array_equal(q[key],f[key][:n]),key
assert qr==fr[:n]
identity_keys=('scene_config','control_config','runtime','controller_sha256','scene_source_sha256','engagement_observer_source_sha256','model_fingerprint','model_xml_sha256')
assert all(pr[k]==report[k] for k in identity_keys)
target='media/m8_table_pickup/full/trace.npz' if run.name=='full_faster_entry' else 'media/m8_table_pickup/failures/conservative_second_turn_grasp_abort/trace.npz'
r={'kind':'exact_immutable_progress_prefix_to_final_trace_link','prefix_path':str(prefix),'render_time_prefix_sha256':sha(prefix),'final_trace_path':target,'audited_source_run_trace_path':str(final),'final_trace_sha256':sha(final),'prefix_and_final_whole_hashes_are_distinct':sha(prefix)!=sha(final),'all_prefix_state_arrays_and_native_samples_match_final_prefix':True,'fixed_metadata_identities_match':True,'prefix_rows':n,'final_rows':len(fr),'prefix_end_time_s':float(q['time'][-1]),'prefix_end_phase':qr[-1]['phase'],'original_source_outcome':{k:report[k] for k in ('passed','partial','aborted')},'scope':'Progress prefix identity stays separate from final whole-trace identity. Final outcome is preserved; matching earlier rows does not convert a progress capture candidate into overall acceptance.'}
if metadata:
 m=json.loads(metadata.read_text());time=m['recorded_physics_time_s'];indices=np.flatnonzero(f['time']==time);assert len(indices)==1;i=int(indices[0]);qp=np.asarray(m['qpos'],dtype='<f8');qv=np.asarray(m['qvel'],dtype='<f8')
 assert np.array_equal(qp,f['qpos'][i]) and np.array_equal(qv,f['qvel'][i]) and m['sample']==fr[i]
 assert hashlib.sha256(qp.tobytes()).hexdigest()==m['qpos_float64_le_bytes_sha256'];assert hashlib.sha256(qv.tobytes()).hexdigest()==m['qvel_float64_le_bytes_sha256']
 if 'time_float64_le_bytes_sha256' in m:assert hashlib.sha256(np.asarray(time,dtype='<f8').tobytes()).hexdigest()==m['time_float64_le_bytes_sha256']
 if 'recorded_time_float64_le_bytes_sha256' in m:assert hashlib.sha256(np.asarray(time,dtype='<f8').tobytes()).hexdigest()==m['recorded_time_float64_le_bytes_sha256']
 assert m['controller_sha256']==report['controller_sha256'];assert m['controller_whole_source_sha256']==sha(run/'controller_source.py');assert m['scene_source_sha256']==report['scene_source_sha256'];assert m['runtime']==report['runtime']
 assert m['render_time_prefix_sha256']==sha(prefix)
 image=Path(m['image'])
 if not image.exists():image=metadata.parent/image.name
 assert sha(image)==m['image_sha256']
 r.update({'progress_metadata_path':str(metadata),'progress_metadata_sha256':sha(metadata),'published_image_path':str(image),'published_image_sha256':sha(image),'exact_image_state_qpos_qvel_native_sample_match':True,'final_sample_index':i,'image_physics_time_s':float(time),'image_phase':fr[i]['phase'],'qpos_f64_le_sha256':m['qpos_float64_le_bytes_sha256'],'qvel_f64_le_sha256':m['qvel_float64_le_bytes_sha256'],'force_timing_note':'Original force/geometry sample inputs are evaluated before integration; saved qpos/qvel are after integration. This crossmatch does not reconstruct forces or integrate physics.'})
output=root/(kind+'_trace_link.json');output.write_text(json.dumps(r,indent=2,allow_nan=False)+'\n')
print(json.dumps({'output':str(output),'prefix_rows':n,'exact_match':True,'final_trace_sha256':r['final_trace_sha256']},indent=2))
