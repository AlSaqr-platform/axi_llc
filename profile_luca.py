import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np
import math

def time_h(beta,rw):
    return 3 + rw*2 + beta + 1

def time_m(beta,rw,f_ratio):
    ax_beta = beta + 1
    if(rw):
        cdc_time = 1 + 3*f_ratio + 3 + 2*f_ratio 
    else:
        cdc_time = 2*(2 + 3*f_ratio)
    llc_len = 8
    single_trx = 4 + cdc_time + 19*f_ratio + llc_len*f_ratio*2
    return 6 + math.ceil(ax_beta / llc_len) * single_trx 

sns.set(font_scale=2)
df_final = pd.DataFrame()
data = pd.read_csv("traces_rw.dat", delimiter=',', sep='\n')
data['TOT'] = data['ACC'] + data['CHAN'] - 2
data['t_0'] = data['t_val'] - data['t_val'].min()
data['H/M'] = "-"
data['Exp'] = "-"
idx_r = 0
idx_w = 0

betas= np.arange(0,256)
exp_t_m_r = np.zeros(256)
exp_t_m_w = np.zeros(256)
exp_t_h_r = np.zeros(256)
exp_t_h_w = np.zeros(256)

for i, val in enumerate(betas):
    exp_t_m_r[i] = time_m(val,1,2)
    exp_t_m_w[i] = time_m(val,0,2)
    exp_t_h_r[i] = time_h(val,1)
    exp_t_h_w[i] = time_h(val,0)

betas_sample = np.arange(7,256,8)
print(betas_sample)
diff_r_h = []
diff_w_h = []
diff_r_m = []
diff_w_m = []

for i, row in data.iterrows():
    if(data.at[i,'UTIL'].astype(float)>0.1):
        data.at[i,'H/M'] = 'HIT'
        if(data.at[i,'W/R'] == 'R'):
            data.at[i,'Exp'] = time_h(data.at[i,'LEN'],1)
            data.at[i,'Diff'] = abs( data.at[i,'Exp'] - data.at[i,'TOT'] ) *100 / data.at[i,'Exp']
            diff_r_h.append(data.at[i,'Diff'])
        else:
            data.at[i,'Exp'] = time_h(data.at[i,'LEN'],0)
            data.at[i,'Diff'] = abs( data.at[i,'Exp'] - data.at[i,'TOT'] ) *100 / data.at[i,'Exp']
            diff_w_h.append(data.at[i,'Diff'])
    else:
        data.at[i,'H/M'] = 'MISS'
        if(data.at[i,'W/R'] == 'R'):
            data.at[i,'Exp'] = time_m(data.at[i,'LEN'],1,2)
            data.at[i,'Diff'] = abs( data.at[i,'Exp'] - data.at[i,'TOT'] ) *100 / data.at[i,'Exp']
            diff_r_m.append(data.at[i,'Diff'])
        else:
            data.at[i,'Exp'] = time_m(data.at[i,'LEN'],0,2)
            data.at[i,'Diff'] = abs( data.at[i,'Exp'] - data.at[i,'TOT'] ) *100 / data.at[i,'Exp']
            diff_w_h.append(data.at[i,'Diff'])
    
figure, ax = plt.subplots(1,2)
print(data.loc[data['H/M']=='MISS'])
#ax2 = ax[0].twinx()
sns.lineplot(ax=ax[0],data=data.loc[data['H/M']=='MISS'],x='LEN',style="W/R",y='TOT',markersize=8,markers=['o','X'])
sns.lineplot(ax=ax[1],data=data.loc[data['H/M']=='HIT'], x='LEN',style="W/R",y='TOT',markersize=8,markers=['o','X'])
#sns.lineplot(ax=ax2,data=data.loc[data['H/M']=='MISS'],x='LEN',style="W/R",y='Diff',markersize=5,markers=['x','+'])
#sns.lineplot(ax=ax[1],data=data.loc[data['H/M']=='HIT'], x='Diff',style="W/R",y='TOT',markersize=10,markers=True)
ax[0].plot(betas,exp_t_m_r,'--',label='Bound R')
ax[0].plot(betas,exp_t_m_w,label='Bound W')
ax[0].set_xlabel('Burst length',fontsize=24)
#ax2.set_ylabel('Cycles of difference (%)',fontsize=24)
ax[0].set_ylabel('Number of cycles',fontsize=24)
ax[0].set_title('Execution time in isolation - MISS',fontsize=24)
ax[0].legend()
#ax.set_xticks(np.arange(0,16))
ax[1].plot(betas,exp_t_h_r,'--',label='Bound R')
ax[1].plot(betas,exp_t_h_w,label='Bound W')
ax[1].legend()
ax[1].set_xlabel('Burst length',fontsize=24)
ax[1].set_ylabel('Number of cycles',fontsize=24)
ax[1].set_title('Execution time in isolation - HIT',fontsize=24)

#plt.ylim(0, 1600)

plt.subplots_adjust( top=0.5,
                     bottom=0.11,
                     left=0.11,
                     right=0.9,
                     hspace=0.2,
                     wspace=0.3)

plt.show()

figure.savefig('measurements.pdf', bbox_inches='tight', format='pdf')

