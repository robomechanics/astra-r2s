from pathlib import Path
import hashlib,json,numpy as np
import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot as plt
p=Path('outputs/m8_table_supported/diagnostics/stop_transfer_B200_v2');a=np.load(p/'original_native_force_ledger.npz');d=json.loads((p/'declaration.json').read_text());t=a['elapsed_s'];mg=d['bolt_weight_N'];last=t>=.98
fig,ax=plt.subplots(3,1,figsize=(9,6),constrained_layout=True)
ax[0].plot(t,a['thread_gravity_opposing_force_N'],color='#2563eb',lw=.65,label='Native thread upward reaction')
ax[0].plot(t,a['hand_gravity_opposing_force_N'],color='#ea580c',lw=.65,label='Native right-pad upward reaction')
ax[0].axhline(mg,color='black',ls='--',lw=1,label='Actual bolt weight')
ax[0].set_ylabel('Signed force (N)');ax[0].legend(ncol=3,fontsize=8);ax[0].set_xlabel('Diagnostic elapsed time (s)')
ax[1].plot((t[last]-.98)*1000,a['thread_gravity_opposing_force_N'][last],color='#2563eb',lw=1)
ax[1].plot((t[last]-.98)*1000,a['right_pad_gravity_opposing_force_N'][last],color='#ea580c',lw=1)
ax[1].axhline(mg,color='black',ls='--',lw=1);ax[1].set_ylabel('Signed force (N)');ax[1].set_xlabel('Final20ms (ms)')
ax[2].bar(['Mean signed\nthread support','Mean positive\nright-pad support'],[np.mean(a['thread_gravity_opposing_force_N'][-2000:])/mg*100,np.mean(np.maximum(a['right_pad_gravity_opposing_force_N'][-2000:],0))/mg*100],color=['#2563eb','#ea580c']);ax[2].axhline(90,color='#2563eb',ls='--',lw=1,label='Thread≥90%');ax[2].axhline(10,color='#ea580c',ls='--',lw=1,label='Positive hand≤10%');ax[2].set_ylabel('Bolt weight (%)');ax[2].legend(loc='upper right',fontsize=8);ax[2].set_ylim(0,115)
fig.suptitle('Cold checkpoint diagnostic: larger damping leaves native load chatter\nNo actual interior flank contact or qualified capture',fontsize=12)
fig.savefig(p/'native_load_chatter.png',dpi=180)
(p/'native_load_chatter_binding.json').write_text(json.dumps({'scope':'Scientific plot from original all-step cold-checkpoint force ledger; not a full native trajectory or qualified capture','ledger_sha256':hashlib.sha256((p/'original_native_force_ledger.npz').read_bytes()).hexdigest(),'plot_sha256':hashlib.sha256((p/'native_load_chatter.png').read_bytes()).hexdigest(),'last_mean_window_s':.1,'native_timestep_s':.00005},indent=2)+'\n')
print(p/'native_load_chatter.png')
