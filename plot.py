import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import csv

def read_csv(p):
    with open(p) as f: return list(csv.DictReader(f))

def plot_grid(ax,bldgs,demand,towers,title,gw,gh,radii):
    ax.set_xlim(-0.5,gw-0.5);ax.set_ylim(gh-0.5,-0.5);ax.set_aspect('equal')
    ax.set_title(title,fontsize=11,fontweight='bold',pad=8)
    for b in bldgs:
        ax.add_patch(plt.Rectangle((int(b['x'])-0.5,int(b['y'])-0.5),1,1,fc='#444',ec='#333',lw=0.5))
    if demand:
        mx=max(float(d['demand'])for d in demand)
        for d in demand:
            x,y,dm=int(d['x']),int(d['y']),float(d['demand'])
            ax.add_patch(plt.Rectangle((x-0.5,y-0.5),1,1,fc=(0.3,0.6,1,0.15+0.5*dm/mx),ec='none'))
            if gw<=15:ax.text(x,y,str(int(dm)),ha='center',va='center',fontsize=6,color='#666')
    cols={'Macro':'#e74c3c','Micro':'#3498db','Pico':'#2ecc71'}
    sz={'Macro':180 if gw<20 else 60,'Micro':100 if gw<20 else 35,'Pico':50 if gw<20 else 20}
    mk={'Macro':'^','Micro':'s','Pico':'o'}
    for t in towers:
        x,y,n=float(t['x']),float(t['y']),t['name']
        ax.scatter(x,y,c=cols.get(n,'#999'),s=sz.get(n,30),marker=mk.get(n,'o'),
                   edgecolors='white',linewidths=1 if gw<20 else 0.7,zorder=5)
        ax.add_patch(plt.Circle((x,y),radii.get(n,2),fill=False,ec=cols.get(n,'#999'),lw=0.8,ls='--',alpha=0.3))
    handles=[]
    for n in ['Macro','Micro','Pico']:
        if any(t['name']==n for t in towers):
            handles.append(plt.scatter([],[],c=cols[n],s=sz[n],marker=mk[n],edgecolors='white',lw=1,label=n))
    handles+=[mpatches.Patch(fc='#444',ec='#333',label='Building'),mpatches.Patch(fc=(0.3,0.6,1,0.4),label='Demand')]
    ax.legend(handles=handles,loc='upper right',fontsize=7,framealpha=0.9)

# ---- Plot 1: Small grid CSP vs SA ----
sb,sd=read_csv('small_buildings.csv'),read_csv('small_demand.csv')
sc,ss=read_csv('small_csp_towers.csv'),read_csv('small_sa_towers.csv')
ri={'Macro':5.1,'Micro':2.9,'Pico':1.8}
cost_map={'Macro':5000,'Micro':2500,'Pico':1000}

fig,(a1,a2)=plt.subplots(1,2,figsize=(14,6))
fig.suptitle('9×9 Grid — CSP vs SA (Both reach 100% QoS)',fontsize=14,fontweight='bold')
cc=sum(cost_map[t['name']]for t in sc);cs=sum(cost_map[t['name']]for t in ss)
plot_grid(a1,sb,sd,sc,f'CSP (MRV) — {len(sc)} tower, ${cc:,}',9,9,ri)
plot_grid(a2,sb,sd,ss,f'SA (Cluster+SA) — {len(ss)} towers, ${cs:,}',9,9,ri)
plt.tight_layout();plt.savefig('plot_small_grid.png',dpi=150,bbox_inches='tight');plt.close()
print('Saved: plot_small_grid.png')

# ---- Plot 2: Big grid ----
bb,bd,bt=read_csv('big_buildings.csv'),read_csv('big_demand.csv'),read_csv('big_sa_towers.csv')
ri2={'Macro':81,'Micro':54,'Pico':37}
cost_map2={'Macro':60000,'Micro':30000,'Pico':12000}
tc=sum(cost_map2[t['name']]for t in bt)

fig,ax=plt.subplots(1,1,figsize=(14,10))
plot_grid(ax,bb,bd,bt,f'300×250 Grid — Window+SA ({len(bt)} towers, ${tc:,}, QoS=97.2%)\nCSP infeasible (10,321 variables)',300,250,ri2)
plt.tight_layout();plt.savefig('plot_big_grid.png',dpi=150,bbox_inches='tight');plt.close()
print('Saved: plot_big_grid.png')

# ---- Plot 3: Comparison bars ----
fig,axes=plt.subplots(1,3,figsize=(15,5))
fig.suptitle('Algorithm Comparison',fontsize=14,fontweight='bold')

# Small grid comparison
algos=['Plain BT','MRV','MRV+LCV','Window+SA','Greedy+SA','Cluster+SA']
nodes=[1434,797,1003,0,0,0]
times=[0.005,0.003,0.005,0.009,0.013,0.012]
colors=['#e74c3c','#e67e22','#f1c40f','#9b59b6','#3498db','#2ecc71']

ax=axes[0];ax.bar(range(len(algos)),[max(n,0.5)for n in nodes],color=colors)
ax.set_yscale('log');ax.set_ylabel('Nodes Explored (log)');ax.set_title('9×9 Grid: Search Effort')
ax.set_xticks(range(len(algos)));ax.set_xticklabels(algos,rotation=45,ha='right',fontsize=7)
for i,n in enumerate(nodes):ax.text(i,max(n,1)*1.3,str(n),ha='center',fontsize=7)

# Big grid scores
algos2=['A: Window+SA','B: Greedy+SA','C: Cluster+SA']
scores2=[97.16,91.03,56.53]
colors2=['#9b59b6','#3498db','#2ecc71']

ax=axes[1];bars=ax.bar(range(3),scores2,color=colors2)
ax.set_ylabel('QoS Score (%)');ax.set_title('300×250 Grid: QoS Score')
ax.set_xticks(range(3));ax.set_xticklabels(algos2,rotation=45,ha='right',fontsize=8)
ax.set_ylim(0,105);ax.axhline(100,color='red',ls='--',alpha=0.3)
for i,v in enumerate(scores2):ax.text(i,v+1.5,f'{v:.1f}%',ha='center',fontsize=9,fontweight='bold')

# Big grid towers
towers2=[14,23,33]
ax=axes[2];ax.bar(range(3),towers2,color=colors2)
ax.set_ylabel('Towers Placed');ax.set_title('300×250 Grid: Tower Count')
ax.set_xticks(range(3));ax.set_xticklabels(algos2,rotation=45,ha='right',fontsize=8)
for i,v in enumerate(towers2):ax.text(i,v+0.5,str(v),ha='center',fontsize=9)

plt.tight_layout();plt.savefig('plot_comparison.png',dpi=150,bbox_inches='tight');plt.close()
print('Saved: plot_comparison.png')
