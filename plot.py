import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import csv
import os
import glob

# ── Helpers ──

def read_csv(p):
    with open(p) as f:
        return list(csv.DictReader(f))

def read_grid_info(p):
    """Read rows,cols from the grid info CSV exported by C++."""
    rows = read_csv(p)
    return int(rows[0]['rows']), int(rows[0]['cols'])

def find_tower_files(prefix):
    """Find all tower CSV files matching a prefix, return dict of name -> data."""
    files = sorted(glob.glob(f'{prefix}_*_towers.csv'))
    result = {}
    for f in files:
        # e.g. "small_D_MRV_towers.csv" -> "D_MRV"
        base = os.path.basename(f)
        name = base.replace(f'{prefix}_', '').replace('_towers.csv', '')
        data = read_csv(f)
        result[name] = data
    return result

def plot_grid(ax, bldgs, demand, towers, title, gw, gh, radii, effective_radii=None):
    """Draw one grid panel with buildings, demand heatmap, and tower markers."""
    ax.set_xlim(-0.5, gw - 0.5)
    ax.set_ylim(gh - 0.5, -0.5)
    ax.set_aspect('equal')
    ax.set_title(title, fontsize=10, fontweight='bold', pad=6)

    # Grid lines
    for i in range(gw + 1):
        ax.axvline(i - 0.5, color='#ddd', lw=0.3, zorder=0)
    for j in range(gh + 1):
        ax.axhline(j - 0.5, color='#ddd', lw=0.3, zorder=0)

    # Buildings
    for b in bldgs:
        x, y = int(b['x']), int(b['y'])
        ax.add_patch(plt.Rectangle((x - 0.5, y - 0.5), 1, 1,
                                    fc='#444', ec='#333', lw=0.5, zorder=2))

    # Demand heatmap
    if demand:
        mx = max(float(d['demand']) for d in demand)
        for d in demand:
            x, y, dm = int(d['x']), int(d['y']), float(d['demand'])
            alpha = 0.15 + 0.5 * dm / mx
            ax.add_patch(plt.Rectangle((x - 0.5, y - 0.5), 1, 1,
                                        fc=(0.3, 0.6, 1, alpha), ec='none', zorder=1))
            # Only show demand numbers on small grids
            if gw <= 20:
                ax.text(x, y, str(int(dm)), ha='center', va='center',
                        fontsize=max(4, 7 - gw // 5), color='#555', zorder=3)

    # Tower markers + range circles
    cols = {'Macro': '#e74c3c', 'Micro': '#3498db', 'Pico': '#2ecc71'}
    small = gw <= 20
    sz = {'Macro': 160 if small else 60, 'Micro': 90 if small else 35, 'Pico': 45 if small else 20}
    mk = {'Macro': '^', 'Micro': 's', 'Pico': 'o'}

    for t in towers:
        x, y, n = float(t['x']), float(t['y']), t['name']
        ax.scatter(x, y, c=cols.get(n, '#999'), s=sz.get(n, 30),
                   marker=mk.get(n, 'o'), edgecolors='white',
                   linewidths=1 if small else 0.7, zorder=5)

        # Given radius (dashed)
        r = radii.get(n, 2)
        ax.add_patch(plt.Circle((x, y), r, fill=False, ec=cols.get(n, '#999'),
                                 lw=0.8, ls='--', alpha=0.3, zorder=4))

        # Effective radius (solid, thinner) — computed from physics
        if effective_radii:
            er = effective_radii.get(n, r)
            ax.add_patch(plt.Circle((x, y), er, fill=False, ec=cols.get(n, '#999'),
                                     lw=1.2, ls='-', alpha=0.45, zorder=4))

    # Legend
    handles = []
    for n in ['Macro', 'Micro', 'Pico']:
        if any(t['name'] == n for t in towers):
            handles.append(plt.scatter([], [], c=cols[n], s=sz[n], marker=mk[n],
                                        edgecolors='white', lw=1, label=n))
    handles += [
        mpatches.Patch(fc='#444', ec='#333', label='Building'),
        mpatches.Patch(fc=(0.3, 0.6, 1, 0.4), label='Demand'),
    ]
    # Circle legend entries
    handles.append(plt.Line2D([], [], color='gray', ls='--', lw=0.8, alpha=0.5, label='Given radius'))
    if effective_radii:
        handles.append(plt.Line2D([], [], color='gray', ls='-', lw=1.2, alpha=0.6, label='Effective radius'))
    ax.legend(handles=handles, loc='upper right', fontsize=6, framealpha=0.9)

    # Clean up axes
    ax.set_xticks(range(0, gw, max(1, gw // 10)))
    ax.set_yticks(range(0, gh, max(1, gh // 10)))
    ax.tick_params(labelsize=6)


# ── Cost helpers ──
cost_small = {'Macro': 5000, 'Micro': 2500, 'Pico': 1000}
cost_big   = {'Macro': 60000, 'Micro': 30000, 'Pico': 12000}

def total_cost(towers, cost_map):
    return sum(cost_map[t['name']] for t in towers)

def compute_effective_radii(powers, noise_floor=1e-4, path_loss_exp=3.5):
    """Compute maxr = (power / (noise_floor * 1.01))^(1/path_loss_exp) for each tower type."""
    thr = noise_floor * 1.01
    return {name: (p / thr) ** (1.0 / path_loss_exp) for name, p in powers.items()}


# ════════════════════════════════════════════════════════════
#  PLOT 1: Small Grid — ALL algorithms
# ════════════════════════════════════════════════════════════
print("--- Plot 1: Small Grid (all algorithms) ---")

R, C = read_grid_info('small_grid_info.csv')
sb = read_csv('small_buildings.csv')
sd = read_csv('small_demand.csv')
ri = {'Macro': 5.1, 'Micro': 2.9, 'Pico': 1.8}

# Effective radii from the actual physics: maxr = (power / (noise_floor * 1.01))^(1/3.5)
powers_small = {'Macro': 0.15, 'Micro': 0.02, 'Pico': 0.004}
eri = compute_effective_radii(powers_small)
print(f"  Effective radii: Macro={eri['Macro']:.1f}, Micro={eri['Micro']:.1f}, Pico={eri['Pico']:.1f}")

# Find all tower files
tower_files = find_tower_files('small')
print(f"  Grid: {R}x{C}, found {len(tower_files)} algorithm results: {list(tower_files.keys())}")

# Read results table for scores/nodes
results_table = {}
if os.path.exists('small_results.csv'):
    for row in read_csv('small_results.csv'):
        results_table[row['algorithm']] = row

# Layout: up to 6 panels (2 rows x 3 cols)
n_algs = len(tower_files)
ncols = min(n_algs, 3)
nrows = (n_algs + ncols - 1) // ncols
fig, axes = plt.subplots(nrows, ncols, figsize=(6 * ncols, 5.5 * nrows))
if n_algs == 1:
    axes = [[axes]]
elif nrows == 1:
    axes = [axes]

fig.suptitle(f'{R}×{C} Grid — All Algorithms (CSP + Heuristic)',
             fontsize=14, fontweight='bold', y=1.02)

for idx, (name, towers) in enumerate(tower_files.items()):
    row, col = idx // ncols, idx % ncols
    ax = axes[row][col]
    
    cost = total_cost(towers, cost_small)
    info = results_table.get(name, {})
    score = info.get('score', '?')
    nodes = info.get('nodes', '')
    
    label = f'{name} — {len(towers)} towers, ${cost:,}'
    if score != '?':
        label += f', QoS={score}%'
    if nodes and int(float(nodes)) > 0:
        label += f', {int(float(nodes)):,} nodes'
    
    plot_grid(ax, sb, sd, towers, label, C, R, ri, effective_radii=eri)

# Hide unused axes
for idx in range(n_algs, nrows * ncols):
    row, col = idx // ncols, idx % ncols
    axes[row][col].set_visible(False)

plt.tight_layout()
plt.savefig('plot_small_grid.png', dpi=150, bbox_inches='tight')
plt.close()
print("  Saved: plot_small_grid.png")


# ════════════════════════════════════════════════════════════
#  PLOT 2: Big Grid — ALL SA algorithms
# ════════════════════════════════════════════════════════════
print("\n--- Plot 2: Big Grid (all SA algorithms) ---")

R2, C2 = read_grid_info('big_grid_info.csv')
bb = read_csv('big_buildings.csv')
bd = read_csv('big_demand.csv')
ri2 = {'Macro': 81, 'Micro': 54, 'Pico': 37}

# Effective radii for big grid towers
powers_big = {'Macro': 600, 'Micro': 150, 'Pico': 25}
eri2 = compute_effective_radii(powers_big)
print(f"  Effective radii: Macro={eri2['Macro']:.1f}, Micro={eri2['Micro']:.1f}, Pico={eri2['Pico']:.1f}")

tower_files_big = find_tower_files('big')
print(f"  Grid: {R2}x{C2}, found {len(tower_files_big)} algorithm results: {list(tower_files_big.keys())}")

big_results = {}
if os.path.exists('big_results.csv'):
    for row in read_csv('big_results.csv'):
        big_results[row['algorithm']] = row

n_big = len(tower_files_big)
fig, axes_big = plt.subplots(1, n_big, figsize=(7 * n_big, 8))
if n_big == 1:
    axes_big = [axes_big]

fig.suptitle(f'{C2}×{R2} Grid — Heuristic Algorithms at Scale\n(CSP infeasible)',
             fontsize=14, fontweight='bold')

for idx, (name, towers) in enumerate(tower_files_big.items()):
    ax = axes_big[idx]
    cost = total_cost(towers, cost_big)
    info = big_results.get(name, {})
    score = info.get('score', '?')
    
    label = f'{name} — {len(towers)} towers, ${cost:,}'
    if score != '?':
        label += f', QoS={score}%'
    
    plot_grid(ax, bb, bd, towers, label, C2, R2, ri2, effective_radii=eri2)

plt.tight_layout()
plt.savefig('plot_big_grid.png', dpi=150, bbox_inches='tight')
plt.close()
print("  Saved: plot_big_grid.png")


# ════════════════════════════════════════════════════════════
#  PLOT 3: Comparison bar charts
# ════════════════════════════════════════════════════════════
print("\n--- Plot 3: Comparison Charts ---")

fig, axes = plt.subplots(1, 3, figsize=(16, 5.5))
fig.suptitle('Algorithm Comparison', fontsize=14, fontweight='bold')

# ── Panel 1: CSP node counts (small grid) ──
ax = axes[0]
if os.path.exists('small_results.csv'):
    rows = read_csv('small_results.csv')
    names = [r['algorithm'] for r in rows]
    nodes = [int(float(r['nodes'])) for r in rows]
    colors = ['#e74c3c', '#e67e22', '#f1c40f', '#9b59b6', '#3498db', '#2ecc71']
    colors = colors[:len(names)]
    
    # Use log scale; replace 0 with 0.5 for display
    display_nodes = [max(n, 0.5) for n in nodes]
    ax.bar(range(len(names)), display_nodes, color=colors)
    ax.set_yscale('log')
    ax.set_ylabel('Nodes Explored (log scale)')
    ax.set_title(f'{R}×{C} Grid: Search Effort')
    ax.set_xticks(range(len(names)))
    ax.set_xticklabels(names, rotation=45, ha='right', fontsize=7)
    for i, n in enumerate(nodes):
        ax.text(i, max(n, 1) * 1.5, f'{n:,}', ha='center', fontsize=7)

# ── Panel 2: Big grid QoS scores ──
ax = axes[1]
if os.path.exists('big_results.csv'):
    rows = read_csv('big_results.csv')
    names = [r['algorithm'] for r in rows]
    scores = [float(r['score']) for r in rows]
    colors2 = ['#9b59b6', '#3498db', '#2ecc71'][:len(names)]
    
    ax.bar(range(len(names)), scores, color=colors2)
    ax.set_ylabel('QoS Score (%)')
    ax.set_title(f'{C2}×{R2} Grid: QoS Score')
    ax.set_xticks(range(len(names)))
    ax.set_xticklabels(names, rotation=45, ha='right', fontsize=8)
    ax.set_ylim(0, 105)
    ax.axhline(100, color='red', ls='--', alpha=0.3)
    for i, v in enumerate(scores):
        ax.text(i, v + 1.5, f'{v:.1f}%', ha='center', fontsize=9, fontweight='bold')

# ── Panel 3: Big grid tower counts ──
ax = axes[2]
if os.path.exists('big_results.csv'):
    rows = read_csv('big_results.csv')
    names = [r['algorithm'] for r in rows]
    towers = [int(r['towers']) for r in rows]
    
    ax.bar(range(len(names)), towers, color=colors2)
    ax.set_ylabel('Towers Placed')
    ax.set_title(f'{C2}×{R2} Grid: Tower Count')
    ax.set_xticks(range(len(names)))
    ax.set_xticklabels(names, rotation=45, ha='right', fontsize=8)
    for i, v in enumerate(towers):
        ax.text(i, v + 0.5, str(v), ha='center', fontsize=9)

plt.tight_layout()
plt.savefig('plot_comparison.png', dpi=150, bbox_inches='tight')
plt.close()
print("  Saved: plot_comparison.png")

print("\nDone! All plots generated.")