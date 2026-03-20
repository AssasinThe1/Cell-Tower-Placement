import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import matplotlib.patches as mpatches
from matplotlib.lines import Line2D
import glob, os, csv

# ── Styling ──────────────────────────────────────────────────────────────────
BG       = "#0d0d1a"
PANEL_BG = "#12122a"

TOWER_STYLES = {
    "Macro": dict(marker="^", s=120, color="#FFD700", edgecolors="#FF8C00", linewidths=1.2, zorder=5),
    "Micro": dict(marker="o", s=60,  color="#00CFFF", edgecolors="#0080AA", linewidths=1.0, zorder=5),
    "Pico":  dict(marker="s", s=35,  color="#FF69B4", edgecolors="#CC1060", linewidths=0.8, zorder=5),
}

def load_grid(path):
    raw  = np.loadtxt(path, delimiter=",")
    mask = (raw == -999.0)
    return np.where(mask, np.nan, raw.astype(float))

def load_towers(path):
    """Returns dict: type_name -> list of (x, y)"""
    if not os.path.exists(path):
        return {}
    towers = {}
    with open(path) as f:
        for row in csv.DictReader(f):
            t = row["type"]
            towers.setdefault(t, []).append((float(row["x"]), float(row["y"])))
    return towers

def load_scores(path="scores.csv"):
    if not os.path.exists(path):
        return {}
    with open(path) as f:
        return {r["algorithm"]: {
            "score":      float(r["score"]),
            "towers":     int(r["towers"]),
            "total_cost": float(r["total_cost"]),
        } for r in csv.DictReader(f)}

def style_ax(ax, title, fontsize=9):
    ax.set_facecolor(PANEL_BG)
    ax.set_title(title, fontsize=fontsize, fontweight="bold", color="white", pad=4)
    ax.set_xlabel("Grid X", fontsize=7, color="#aaaacc")
    ax.set_ylabel("Grid Y", fontsize=7, color="#aaaacc")
    ax.tick_params(colors="#888899", labelsize=6)
    for sp in ax.spines.values():
        sp.set_color("#333355")

def plot_map(ax, data, title, label, vmin=None, vmax=None, cmap="turbo"):
    cm = plt.colormaps[cmap].copy()
    cm.set_bad("#080810")
    vmin = vmin if vmin is not None else float(np.nanpercentile(data, 5))
    vmax = vmax if vmax is not None else float(np.nanmax(data))
    im = ax.imshow(data, cmap=cm, origin="lower", vmin=vmin, vmax=vmax,
                   interpolation="nearest", aspect="auto")
    style_ax(ax, title)
    cb = plt.colorbar(im, ax=ax, fraction=0.035, pad=0.03)
    cb.set_label(label, fontsize=7, color="#aaaacc")
    cb.ax.tick_params(colors="#888899", labelsize=6)
    return im

def overlay_towers(ax, towers: dict):
    """Scatter each tower type with its own marker style."""
    for ttype, coords in towers.items():
        if not coords:
            continue
        xs = [c[0] for c in coords]
        ys = [c[1] for c in coords]
        style = TOWER_STYLES.get(ttype, dict(marker="*", s=80, color="white",
                                              edgecolors="grey", linewidths=0.8, zorder=5))
        ax.scatter(xs, ys, **style, label=ttype)

def tower_legend(fig, all_types):
    handles = []
    for t in all_types:
        st = TOWER_STYLES.get(t, {})
        handles.append(Line2D([0], [0],
            marker=st.get("marker","o"),
            color="none",
            markerfacecolor=st.get("color","white"),
            markeredgecolor=st.get("edgecolors","grey"),
            markeredgewidth=st.get("linewidths",1),
            markersize=8, label=t))
    fig.legend(handles=handles, loc="lower center",
               ncol=len(handles), fontsize=8,
               facecolor=PANEL_BG, edgecolor="#333355",
               labelcolor="white", framealpha=0.85,
               bbox_to_anchor=(0.5, -0.02))

# ── Figure 2+: Signal maps, 2 algorithms per figure ─────────────────────────
def show_signal_maps(algo_names, signal_grids, all_towers, scores):
    # Shared dB scale for fair visual comparison
    all_vals    = np.concatenate([g[~np.isnan(g)].ravel() for g in signal_grids])
    shared_vmin = float(np.percentile(all_vals, 5))
    shared_vmax = float(np.nanmax(all_vals))

    chunks = [algo_names[i:i+2] for i in range(0, len(algo_names), 2)]

    for chunk_idx, chunk in enumerate(chunks):
        fig, axes = plt.subplots(1, len(chunk),
                                  figsize=(8 * len(chunk), 6),
                                  facecolor=BG)
        if len(chunk) == 1:
            axes = [axes]

        fig.suptitle(f"Signal Strength — Algorithms "
                     f"{', '.join(chunk)}",
                     fontsize=12, fontweight="bold", color="white")

        for ax, name in zip(axes, chunk):
            idx = algo_names.index(name)
            s   = scores.get(name, {})
            subtitle = (f"\nQoS {s['score']:.1f}%  ·  {s['towers']} towers  "
                        f"·  ${s['total_cost']/1000:.0f}k") if s else ""
            plot_map(ax, signal_grids[idx], name + subtitle,
                     "Signal Strength (dB)", shared_vmin, shared_vmax)
            overlay_towers(ax, all_towers.get(name, {}))

        all_types = list(TOWER_STYLES.keys())
        tower_legend(fig, all_types)
        plt.tight_layout()
        fname = f"signal_comparison_{chunk_idx+1}.png"
        plt.savefig(fname, dpi=150, bbox_inches="tight", facecolor=BG)
        print(f"Saved: {fname}")
        plt.show()
        plt.close("all")

# Replace show_overview — adds per-type tower count table under each subplot
def show_overview(demand_data, algo_names, all_towers, scores):
    n    = len(algo_names)
    cols = min(n, 3)
    rows = 1 + (n + cols - 1) // cols

    fig = plt.figure(figsize=(6.5 * cols, 5.0 * rows), facecolor=BG)
    fig.suptitle("Demand Map & Tower Placements per Algorithm",
                 fontsize=13, fontweight="bold", color="white", y=1.01)
    gs = gridspec.GridSpec(rows, cols, figure=fig, hspace=0.65, wspace=0.35)

    ax_d = fig.add_subplot(gs[0, :])
    plot_map(ax_d, demand_data, "Network Demand (Mbps)", "Demand (Mbps)", cmap="plasma")

    for idx, name in enumerate(algo_names):
        ax = fig.add_subplot(gs[1 + idx // cols, idx % cols])
        s  = scores.get(name, {})

        # Count per type
        tw       = all_towers.get(name, {})
        type_str = "  ".join(
            f"{t[0]}:{len(tw[t])}" for t in ["Macro","Micro","Pico"] if t in tw
        )
        subtitle = ""
        if s:
            subtitle = (f"\nQoS {s['score']:.1f}%  ·  {s['towers']} towers  "
                        f"·  ${s['total_cost']/1000:.0f}k"
                        f"\n{type_str}")

        plot_map(ax, demand_data, name + subtitle, "Demand (Mbps)", cmap="plasma")
        overlay_towers(ax, tw)

    tower_legend(fig, list(TOWER_STYLES.keys()))
    plt.tight_layout()
    plt.savefig("overview.png", dpi=150, bbox_inches="tight", facecolor=BG)
    print("Saved: overview.png")
    plt.show()
    plt.close("all")


# Replace show_comparison_chart — adds stacked bar for tower type breakdown
def show_comparison_chart(scores, all_towers):
    if not scores:
        return

    names  = list(scores.keys())
    qos    = [scores[n]["score"]      for n in names]
    costs  = [scores[n]["total_cost"] for n in names]

    type_names = ["Macro", "Micro", "Pico"]
    type_colors = {"Macro": "#FFD700", "Micro": "#00CFFF", "Pico": "#FF69B4"}

    fig, axes = plt.subplots(1, 3, figsize=(15, 5), facecolor=BG)
    fig.suptitle("Algorithm Comparison", fontsize=13,
                 fontweight="bold", color="white")

    palette  = ["#4e79a7", "#f28e2b", "#59a14f", "#e15759", "#76b7b2"]
    best_q   = max(range(len(qos)), key=lambda i: qos[i])
    x        = np.arange(len(names))

    # Panel 1: QoS
    ax = axes[0]
    ax.set_facecolor(PANEL_BG)
    bars = ax.bar(x, qos, color=palette[:len(names)],
                  edgecolor="#333355", linewidth=0.8)
    bars[best_q].set_edgecolor("gold")
    bars[best_q].set_linewidth(2.5)
    for bar, v in zip(bars, qos):
        ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() * 1.01,
                f"{v:.1f}%", ha="center", va="bottom", fontsize=8, color="white")
    ax.set_title("Demand-Weighted QoS (%)", fontsize=9, fontweight="bold", color="white")
    ax.set_ylabel("QoS (%)", fontsize=8, color="#aaaacc")

    # Panel 2: Stacked tower type breakdown
    ax = axes[1]
    ax.set_facecolor(PANEL_BG)
    bottoms = np.zeros(len(names))
    for t in type_names:
        counts = [len(all_towers.get(n, {}).get(t, [])) for n in names]
        bars_t = ax.bar(x, counts, bottom=bottoms,
                        color=type_colors[t], edgecolor="#333355",
                        linewidth=0.6, label=t)
        for bar, c, b in zip(bars_t, counts, bottoms):
            if c > 0:
                ax.text(bar.get_x() + bar.get_width()/2,
                        b + c / 2,
                        str(c), ha="center", va="center",
                        fontsize=8, fontweight="bold", color="#0d0d1a")
        bottoms += counts
    # Total count on top
    for i, name in enumerate(names):
        total = sum(len(all_towers.get(name, {}).get(t, [])) for t in type_names)
        ax.text(i, bottoms[i] + 0.15, str(total),
                ha="center", va="bottom", fontsize=8, color="white")

    ax.set_title("Tower Type Breakdown", fontsize=9, fontweight="bold", color="white")
    ax.set_ylabel("Count", fontsize=8, color="#aaaacc")
    ax.legend(loc="upper right", fontsize=7, facecolor=PANEL_BG,
              edgecolor="#333355", labelcolor="white", framealpha=0.85)

    # Panel 3: Cost
    ax = axes[2]
    ax.set_facecolor(PANEL_BG)
    bars = ax.bar(x, costs, color=palette[:len(names)],
                  edgecolor="#333355", linewidth=0.8)
    bars[best_q].set_edgecolor("gold")
    bars[best_q].set_linewidth(2.5)
    for bar, v in zip(bars, costs):
        ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() * 1.01,
                f"${v/1000:.0f}k", ha="center", va="bottom",
                fontsize=8, color="white")
    ax.set_title("Total Cost ($)", fontsize=9, fontweight="bold", color="white")
    ax.set_ylabel("Cost ($)", fontsize=8, color="#aaaacc")

    for ax in axes:
        ax.set_xticks(x)
        ax.set_xticklabels(names, rotation=15, ha="right",
                           fontsize=8, color="white")
        ax.tick_params(colors="#888899", labelsize=8)
        ax.spines[["top","right"]].set_visible(False)
        for sp in ax.spines.values():
            sp.set_color("#333355")

    plt.tight_layout()
    plt.savefig("comparison_chart.png", dpi=150, bbox_inches="tight", facecolor=BG)
    print("Saved: comparison_chart.png")
    plt.show()
    plt.close("all")

# ── Entry point ───────────────────────────────────────────────────────────────
def visualize_all(signal_pattern="*_signal_map.csv",
                  demand_file="demand_map.csv",
                  scores_file="scores.csv"):

    signal_files = sorted(glob.glob(signal_pattern))
    if not signal_files:
        print(f"No files matching '{signal_pattern}'")
        return

    algo_names   = [os.path.basename(f).replace("_signal_map.csv", "")
                    for f in signal_files]
    signal_grids = [load_grid(f) for f in signal_files]
    demand_data  = load_grid(demand_file) if os.path.exists(demand_file) else None
    scores       = load_scores(scores_file)

    # Load tower positions for each algorithm
    all_towers = {
        name: load_towers(f"{name}_towers.csv")
        for name in algo_names
    }

    if demand_data is not None:
        show_overview(demand_data, algo_names, all_towers, scores)

    show_signal_maps(algo_names, signal_grids, all_towers, scores)
    show_comparison_chart(scores, all_towers)

if __name__ == "__main__":
    visualize_all()