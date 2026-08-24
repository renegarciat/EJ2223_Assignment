#!/usr/bin/env python3
"""
rotor_pole_geometry.py

To-scale cross-section drawing of a single V-shape IPM rotor pole, for the
EJ2223 IPM design report's "Stage 1: Rotor Geometry" subsection
(IPM_Design_Report.tex, eqs. ps_ext, ib_len, d12, dps, dir, wm), following
Di Gerlando & Ricca's V-shape sizing method (ICEM 2022).

Every derived length below is computed with the *same* formulas as the
report (transcribed 1:1 from the .tex) at this project's own converged
Stage-1 design point, so the figure is numerically self-consistent with the
report text -- not a separate, hand-tuned illustration. It is, however, a
SIMPLIFIED schematic, not a full manufacturing CAD reconstruction:
  - The pole-shoe / bridge corner fillets (rounded in a real lamination)
    are drawn as straight/arc joins.
  - The magnet pocket is drawn as exactly the magnet's own rectangle (no
    separate pocket clearance, since the report does not define one).
  - "Rib" (w_hr, h_ry) is drawn as: h_ry = the solid steel layer directly
    under the magnet pocket, and w_hr = a corner label near the outer
    bridge/interpolar boundary -- the report does not derive the rib's
    exact 2D shape beyond these two scalar lengths.
Every LABELED dimension (D_r, D_ir, g, t_m, w_m, alpha_v, w_ib, w_ob,
b_ps, d_ps, h_ib, h_ry, w_hr) is exact to the computed value.

Run:      python3 rotor_pole_geometry.py
Requires: numpy, matplotlib
Output:   ../IPM_Design_Report/figures/rotor_pole_geometry.png
"""
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Arc
from matplotlib.lines import Line2D

# ----------------------------------------------------------------------
# 1. Design point -- this project's converged Stage-1 inputs (mm / deg)
#    (IPM_Design_Report.tex, Sec. "Stage 1: Rotor Geometry")
# ----------------------------------------------------------------------
P = 10          # pole count
N_s = 12        # slot count
D_r = 37.450416  # rotor outer diameter [mm]  (Formula-2 D^3L bore D_is - 2g)
D_is = 39.450416 # stator bore [mm]  (Formula 2, J_s,rms=12A/mm^2 -- see report's Stage 1 discussion)
g = 1.0         # airgap [mm]

alpha_m = 0.80       # magnet embrace ratio [-]
k_rib = 0.1          # rib width as a fraction of tau_s [-]
t_m = 3.0            # magnet thickness [mm]
alpha_v_deg = 78.0   # magnet V-tilt angle, from the d-axis [deg]
w_ib = 2.5           # inner bridge width [mm]
w_ob = 0.5           # outer bridge width [mm]

alpha_v = np.radians(alpha_v_deg)

# ----------------------------------------------------------------------
# 2. Stage-1 derived quantities (eqs. ps_ext, ib_len, d12, dps, dir, wm)
# ----------------------------------------------------------------------
tau_r = np.pi * D_r / P
tau_s = np.pi * D_is / N_s
w_hr = k_rib * tau_s
h_ry = 1.5 * w_hr

b_ps = alpha_m * tau_r
h_ib = t_m * np.sin(alpha_v)

R_r = D_r / 2
phi_shoe = alpha_m * np.pi / P                       # pole-shoe half-angle [rad]
d12 = (R_r - w_ob) * np.sin(phi_shoe)

d_ps = ((d12 - w_ib / 2) / np.tan(alpha_v)
        + ((R_r - w_ob) - d12 / np.tan(phi_shoe))
        + w_ob)

D_ir = D_r - 2 * (d_ps + h_ib + h_ry)
w_m = (d12 - w_ib / 2) / np.sin(alpha_v)

theta_p = 2 * np.pi / P                              # full pole pitch [rad]

print("Stage-1 rotor geometry -- computed from IPM_Design_Report.tex eqs.:")
for name, val in [("tau_r [mm]", tau_r), ("tau_s [mm]", tau_s),
                   ("w_hr [mm]", w_hr), ("h_ry [mm]", h_ry),
                   ("b_ps [mm]", b_ps), ("h_ib [mm]", h_ib),
                   ("d12  [mm]", d12), ("d_ps [mm]", d_ps),
                   ("D_ir [mm]", D_ir), ("w_m  [mm]", w_m)]:
    print(f"  {name:12s} = {val:8.4f}")

# ----------------------------------------------------------------------
# 3. Geometry construction
#    Local frame: origin = rotor center, +Y = this pole's d-axis,
#    phi measured from the d-axis (phi=0 -> straight up).
# ----------------------------------------------------------------------
def pt(r, phi):
    return np.array([r * np.sin(phi), r * np.cos(phi)])

def arc_pts(r, phi0, phi1, n=60):
    return np.array([pt(r, p) for p in np.linspace(phi0, phi1, n)])

R_is = D_is / 2
R_ir = D_ir / 2
half_tile = theta_p / 2                              # one full pole pitch, split evenly

# --- magnet bar (right-hand half of the V; left is the mirror image) --
long_dir = np.array([np.sin(alpha_v), np.cos(alpha_v)])     # bar's long axis
normal_dir = np.array([-np.cos(alpha_v), np.sin(alpha_v)])  # toward the pole shoe

r_mid = R_r - d_ps - h_ib / 2                # radial midpoint of the magnet band
x_mid = d12 - (w_m / 2) * np.sin(alpha_v)    # centers the outer tip at x = d12
M = np.array([x_mid, r_mid])
A = M - (w_m / 2) * long_dir                 # inner tip (centerline, near vertex)
B = M + (w_m / 2) * long_dir                 # outer tip (centerline, near shoe)

magnet_R = np.array([A + (t_m / 2) * normal_dir, B + (t_m / 2) * normal_dir,
                      B - (t_m / 2) * normal_dir, A - (t_m / 2) * normal_dir])
magnet_L = magnet_R * np.array([-1, 1])      # mirror across the d-axis

# --- pole shoe boundary (top at R_r, bottom at R_r - d_ps) -------------
shoe_top = arc_pts(R_r, -phi_shoe, phi_shoe, 40)
shoe_bottom = arc_pts(R_r - d_ps, phi_shoe, -phi_shoe, 40)
shoe_poly = np.vstack([shoe_top, shoe_bottom])

# --- rotor-core steel outline for one tile (one pole pitch) -----------
core_outer = arc_pts(R_r, -half_tile, half_tile, 80)
core_inner = arc_pts(R_ir, half_tile, -half_tile, 80)      # reversed, closes the loop
core_poly = np.vstack([core_outer, core_inner])

# ----------------------------------------------------------------------
# 4. Drawing
# ----------------------------------------------------------------------
STEEL = "#c9cdd3"
MAGNET = "#e2622c"
AIRGAP = "#dceefc"
EDGE = "#2b2f36"
DIM = "#1f5fa8"

fig, ax = plt.subplots(figsize=(11.0, 5.2))

# rotor core (steel), one full pole-pitch tile
ax.add_patch(Polygon(core_poly, closed=True, facecolor=STEEL, edgecolor=EDGE,
                      linewidth=1.1, zorder=1))

# jagged "break" line along D_ir: core continues inward to the shaft, not shown
n_zig, amp = 26, 0.35
zz_phi = np.linspace(-half_tile, half_tile, n_zig)
zz_r = R_ir + amp * (np.array([1, -1] * (n_zig // 2 + 1))[:n_zig])
zz_pts = np.array([pt(r, p) for r, p in zip(zz_r, zz_phi)])
ax.plot(zz_pts[:, 0], zz_pts[:, 1], color=EDGE, linewidth=1.1, zorder=2)
ax.text(0, R_ir - 1.2, "(core continues to the shaft)", ha="center", va="top",
        fontsize=8, style="italic", color="#555555")

# stator bore arc + shaded airgap band
is_arc = arc_pts(R_is, -half_tile, half_tile, 80)
airgap_band = np.vstack([arc_pts(R_r, -half_tile, half_tile, 80),
                          arc_pts(R_is, half_tile, -half_tile, 80)])
ax.add_patch(Polygon(airgap_band, closed=True, facecolor=AIRGAP, edgecolor="none", zorder=0.5))
ax.plot(is_arc[:, 0], is_arc[:, 1], color="#5a9bd8", linewidth=0.9, linestyle="--", zorder=2)

# pole shoe outline (already steel-colored via core fill; draw its boundary)
ax.plot(shoe_poly[:, 0], shoe_poly[:, 1], color=EDGE, linewidth=1.0, zorder=3)

# magnets
for poly in (magnet_R, magnet_L):
    ax.add_patch(Polygon(poly, closed=True, facecolor=MAGNET, edgecolor=EDGE,
                          linewidth=1.1, zorder=4))

# tile boundary lines (radial edges at +-half_tile, for context)
for s in (-1, 1):
    p0, p1 = pt(R_ir, s * half_tile), pt(R_r, s * half_tile)
    ax.plot([p0[0], p1[0]], [p0[1], p1[1]], color="#9aa0aa", linewidth=0.7,
             linestyle=":", zorder=1.5)

# d-axis reference line
ax.plot([0, 0], [R_ir * 0.98, R_is + 1.5], color="#9aa0aa", linewidth=0.7,
         linestyle="-.", zorder=1.5)
ax.text(0.15, R_is + 1.3, "d-axis", fontsize=7.5, color="#666666", style="italic")

# ----------------------------------------------------------------------
# 5. Dimensioning helpers
# ----------------------------------------------------------------------
def vdim(x_dim, y0, y1, label, x_feature=None, side=1, fontsize=8.5, stub=0.9):
    """Vertical dimension line at x=x_dim between y0 and y1. If x_feature is
    given, short stub extension lines connect the feature edge to the
    dimension line (stub length only, not a full line back to the feature --
    keeps the drawing uncluttered)."""
    if x_feature is not None:
        for y in (y0, y1):
            ax.plot([x_feature, x_dim + side * 0], [y, y], color=DIM, linewidth=0.6,
                     alpha=0.5, zorder=5)
    ax.annotate("", xy=(x_dim, y1), xytext=(x_dim, y0),
                arrowprops=dict(arrowstyle="<->", color=DIM, linewidth=1.1), zorder=6)
    ax.text(x_dim + side * 0.35, (y0 + y1) / 2, label, color=DIM, fontsize=fontsize,
            ha="left" if side > 0 else "right", va="center", rotation=90,
            bbox=dict(boxstyle="round,pad=0.12", fc="white", ec="none", alpha=0.85), zorder=6)

def hdim(y_dim, x0, x1, label, y_from=None, fontsize=8.5):
    """Horizontal dimension line at y=y_dim between x0 and x1."""
    if y_from is not None:
        ax.plot([x0, x0], [y_from, y_dim], color=DIM, linewidth=0.6, alpha=0.6, zorder=5)
        ax.plot([x1, x1], [y_from, y_dim], color=DIM, linewidth=0.6, alpha=0.6, zorder=5)
    ax.annotate("", xy=(x1, y_dim), xytext=(x0, y_dim),
                arrowprops=dict(arrowstyle="<->", color=DIM, linewidth=1.1), zorder=6)
    ax.text((x0 + x1) / 2, y_dim + 0.32, label, color=DIM, fontsize=fontsize,
            ha="center", va="bottom",
            bbox=dict(boxstyle="round,pad=0.12", fc="white", ec="none", alpha=0.85), zorder=6)

def leader(target, text_pos, label, fontsize=8):
    ax.annotate(label, xy=target, xytext=text_pos, fontsize=fontsize, color="#333333",
                ha="left", va="center",
                arrowprops=dict(arrowstyle="-", color="#333333", linewidth=0.8,
                                 shrinkA=0, shrinkB=2), zorder=7)

# reference tile widths, used to keep every callout close to the drawing
x_tau = R_r * np.sin(half_tile)     # tile half-width at the rotor OD
x_is = R_is * np.sin(half_tile)     # tile half-width at the stator bore

# --- D_r, D_ir: compact leader-line callouts (this is a broken/cropped
#     view -- the rotor center is off-screen, so these are not drawn as
#     radius arrows back to an unshown origin) --------------------------
leader((-x_tau, R_r), (-(x_is + 4.5), R_r + 0.4), r"$D_r=%.2f$ mm" % D_r, fontsize=8)
leader((-x_tau * 0.95, R_ir), (-(x_is + 4.5), R_ir - 0.3), r"$D_{ir}=%.2f$ mm" % D_ir, fontsize=8)

# --- g: small local dimension, right beside the tile --------------------
x_g = x_tau + 0.8
vdim(x_g, R_r, R_is, r"$g$", x_feature=x_tau, side=1, fontsize=7.5, stub=0.4)

# --- right stack: d_ps, h_ib, h_ry (radial layer stack, on the d-axis) -
x_right = x_is + 2.2
vdim(x_right, R_r - d_ps, R_r, r"$d_{ps}$", x_feature=x_tau, side=1)
vdim(x_right, R_r - d_ps - h_ib, R_r - d_ps, r"$h_{ib}$", x_feature=x_tau * 0.55, side=1)
vdim(x_right, R_ir, R_r - d_ps - h_ib, r"$h_{ry}$", x_feature=x_tau * 0.25, side=1)

# --- top: tau_r (full pole pitch) and b_ps (shoe width) ----------------
y_tau = R_r * np.cos(half_tile) + 3.2
hdim(y_tau, -x_tau, x_tau, r"$\tau_r$ (pole pitch)", y_from=R_r * np.cos(half_tile))
y_bps = R_r + 1.1
hdim(y_bps, -b_ps / 2, b_ps / 2, r"$b_{ps}$", y_from=R_r)

# --- w_hr: corner label near the outer bridge / interpolar boundary ---
p_whr = pt(R_r, half_tile - 0.25 * (half_tile - phi_shoe))
leader(p_whr, (x_tau * 0.55, R_r - 0.35), r"$w_{hr}=%.2f$ mm" % w_hr, fontsize=8)

# --- t_m: thickness across the right magnet bar ------------------------
t_mid = (A + B) / 2
t_p1 = t_mid + (t_m / 2) * normal_dir
t_p2 = t_mid - (t_m / 2) * normal_dir
ax.annotate("", xy=t_p1, xytext=t_p2,
            arrowprops=dict(arrowstyle="<->", color="#7a2b0a", linewidth=1.0), zorder=8)
ax.text(t_mid[0] + 0.6, t_mid[1] + 0.05, r"$t_m$", color="#7a2b0a", fontsize=8.5,
        bbox=dict(boxstyle="round,pad=0.1", fc="white", ec="none", alpha=0.85), zorder=8)

# --- w_m: length along the right magnet bar's centerline ---------------
off = 1.55 * normal_dir
ax.annotate("", xy=A - off, xytext=B - off,
            arrowprops=dict(arrowstyle="<->", color="#7a2b0a", linewidth=1.0), zorder=8)
w_m_mid = (A + B) / 2 - off
ax.text(w_m_mid[0], w_m_mid[1] - 0.55, r"$w_m$", color="#7a2b0a", fontsize=8.5,
        ha="center", bbox=dict(boxstyle="round,pad=0.1", fc="white", ec="none", alpha=0.85), zorder=8)

# --- alpha_v: angle between the bar's long axis and the d-axis ---------
vertex_ref = np.array([0.0, R_ir + h_ry])
arc_radius = 2.6
ang0_std = 90.0  # d-axis, standard (0deg = +x) convention
ang1_std = np.degrees(np.arctan2(long_dir[1], long_dir[0]))
ax.add_patch(Arc(vertex_ref, 2 * arc_radius, 2 * arc_radius, angle=0,
                  theta1=ang1_std, theta2=ang0_std, color="#7a2b0a", linewidth=1.0, zorder=8))
ax.plot([vertex_ref[0], vertex_ref[0]], [vertex_ref[1], vertex_ref[1] + arc_radius + 0.6],
        color="#9aa0aa", linewidth=0.6, linestyle="-.", zorder=3)
mid_ang = np.radians((ang0_std + ang1_std) / 2)
lp = vertex_ref + (arc_radius + 1.0) * np.array([np.cos(mid_ang), np.sin(mid_ang)])
ax.text(lp[0], lp[1], r"$\alpha_v$", color="#7a2b0a", fontsize=9, ha="center", va="center",
        bbox=dict(boxstyle="round,pad=0.1", fc="white", ec="none", alpha=0.85), zorder=8)

# --- w_ib: inner bridge, between the two magnets' inner tips -----------
A_L = A * np.array([-1, 1])
hdim(A[1] - 1.0, A_L[0], A[0], r"$w_{ib}$", y_from=A[1], fontsize=8)

# --- w_ob: outer bridge, leader to the sliver above the magnet tip -----
tip_out = B + (t_m / 2) * normal_dir
p_top = pt(R_r, np.arctan2(tip_out[0], tip_out[1]))
leader((tip_out + p_top) / 2, (tip_out[0] + 2.4, tip_out[1] + 1.0),
       r"$w_{ob}$", fontsize=8)

# ----------------------------------------------------------------------
# 6. Cosmetics
# ----------------------------------------------------------------------
ax.set_aspect("equal")
ax.axis("off")
x_lo = -(x_is + 13.5)
x_hi = x_right + 3.0
y_lo = R_ir - 1.9
y_hi = y_tau + 1.6
ax.set_xlim(x_lo, x_hi)
ax.set_ylim(y_lo, y_hi)

legend_handles = [
    Line2D([0], [0], marker="s", color="none", markerfacecolor=STEEL,
           markeredgecolor=EDGE, markersize=10, label="Rotor steel"),
    Line2D([0], [0], marker="s", color="none", markerfacecolor=MAGNET,
           markeredgecolor=EDGE, markersize=10, label="Permanent magnet"),
    Line2D([0], [0], marker="s", color="none", markerfacecolor=AIRGAP,
           markeredgecolor="none", markersize=10, label="Airgap"),
]
ax.legend(handles=legend_handles, loc="lower left", frameon=False, fontsize=8,
          bbox_to_anchor=(0.0, 0.0), bbox_transform=ax.transAxes,
          handletextpad=0.5, borderaxespad=0.3)

ax.set_title("One rotor pole -- V-cavity cross-section, to scale (all lengths in mm)\n"
             r"$P=%d$, $N_s=%d$, $D_r=%.2f$, $D_{is}=%.2f$, $g=%.1f$, $\alpha_m=%.2f$, "
             r"$t_m=%.1f$, $\alpha_v=%d^\circ$, $w_{ib}=%.1f$, $w_{ob}=%.1f$"
             % (P, N_s, D_r, D_is, g, alpha_m, t_m, alpha_v_deg, w_ib, w_ob),
             fontsize=9.5, pad=10)

fig.tight_layout()

out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..",
                        "IPM_Design_Report", "figures")
os.makedirs(out_dir, exist_ok=True)
out_path = os.path.join(out_dir, "rotor_pole_geometry.png")
fig.savefig(out_path, dpi=300, bbox_inches="tight", pad_inches=0.15)
print(f"\nSaved {out_path}")
