"""Figuras del README, a partir de los datos grabados el 4-oct-2026."""
import csv, glob, math, sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from PIL import Image

RAIZ = Path(__file__).resolve().parent.parent
EKF = RAIZ / "LOCALIZACION_EKF"
sys.path.insert(0, str(EKF))
from evaluar import leer_tum, asociar, alinear_inicio  # noqa: E402

OUT = Path(__file__).parent
AZUL, NARANJA, AQUA = "#2a78d6", "#eb6834", "#1baf7a"     # paleta validada (3 slots)
TINTA, TINTA2, REJILLA = "#0b0b0b", "#52514e", "#e4e3df"
plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 13, "axes.edgecolor": TINTA2,
                     "axes.labelcolor": TINTA2, "xtick.color": TINTA2, "ytick.color": TINTA2,
                     "axes.spines.top": False, "axes.spines.right": False, "figure.dpi": 150})

# ---------- mapa: recorridos y base de carga ----------
pgm = Image.open(RAIZ / "NAV2/mapas/almacen.pgm"); H = pgm.size[1]
def mapa(ax, x0, x1, y0, y1):
    m = np.array(pgm, dtype=float)
    ext = [-14.876, -14.876 + pgm.size[0] * 0.05, -25.057, -25.057 + H * 0.05]
    ax.imshow(m, cmap="gray", extent=ext, vmin=0, vmax=255, interpolation="nearest")
    ax.set_xlim(x0, x1); ax.set_ylim(y0, y1); ax.set_aspect("equal")
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")

nav2 = [(-3.046, -0.058), (-3.117, -3.063), (2.944, -3.171), (2.946, -0.049)]
ekf_r = [(-3.0, 1.0), (-3.0, -3.1), (3.0, -3.1), (3.0, 1.0)]
fig, ax = plt.subplots(figsize=(7.5, 6))
mapa(ax, -6, 6, -5, 3)
c = np.array(nav2 + nav2[:1]); ax.plot(c[:, 0], c[:, 1], color=NARANJA, lw=2, label="recorrido Nav2 (4 puntos × 3 vueltas)")
ax.plot(c[:-1, 0], c[:-1, 1], "o", color=NARANJA, ms=8, mec="white", mew=2)
ax.plot([0.157], [0.0], "s", color=TINTA, ms=11, mec="white", mew=2)
ax.annotate("base de carga\n(invisible para el LiDAR)", (0.157, 0.0), (0.6, 1.3), color=TINTA,
            fontsize=12, arrowprops=dict(arrowstyle="-", color=TINTA2))
ax.annotate("tramo 4 → 1", (-1.5, -0.05), (-4.2, 1.6), color=TINTA2, fontsize=12,
            arrowprops=dict(arrowstyle="-", color=TINTA2))
ax.legend(loc="lower center", bbox_to_anchor=(0.5, 1.0), frameon=False, fontsize=12)
fig.savefig(OUT / "mapa_base.png", bbox_inches="tight"); plt.close(fig)

fig, ax = plt.subplots(figsize=(6, 8))
mapa(ax, -14.9, 15.2, -25.1, 25.1)
ax.plot([0.157], [0.0], "s", color=TINTA, ms=7)
ax.set_title("Mapa del almacén con slam_toolbox\n601 × 1003 celdas de 5 cm", color=TINTA, fontsize=13)
fig.savefig(OUT / "mapa_completo.png", bbox_inches="tight"); plt.close(fig)

# ---------- trayectorias, condicion 'ambos' ----------
d = sorted(EKF.glob("resultados/2026*_ambos"))[0]
tr, pr, yr = leer_tum(d / "verdad.tum")
fig, ax = plt.subplots(figsize=(7, 6.3))
ax.plot(pr[:, 0], pr[:, 1], color=TINTA, lw=3.2, label="verdad del simulador")
for nombre, col, et in [("odom_degradada", NARANJA, "odometría degradada"), ("ekf", AZUL, "EKF: odometría + IMU"),
                        ("slam", AQUA, "AMCL (mapa)")]:
    t, p, y = leer_tum(d / f"{nombre}.tum"); ie, ir = asociar(t, tr)
    q = alinear_inicio(p[ie], y[ie], pr[ir], yr[ir])
    ax.plot(q[:, 0], q[:, 1], color=col, lw=2 if nombre != "odom_degradada" else 1.6, label=et)
ax.set_aspect("equal"); ax.grid(color=REJILLA, lw=0.8); ax.set_axisbelow(True)
ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
ax.legend(loc="lower center", bbox_to_anchor=(0.5, 1.0), ncol=2, frameon=False, fontsize=12)
fig.savefig(OUT / "trayectorias_ambos.png", bbox_inches="tight"); plt.close(fig)

# ---------- ATE por condicion ----------
filas = []
for f in EKF.glob("resultados/2026100*_*/metricas.csv"):
    cond = f.parent.name.split("_", 2)[-1]
    if cond == "prueba_humo": continue
    filas += [dict(r, cond=cond) for r in csv.DictReader(open(f))]
conds = ["nada", "giro", "lineal", "ambos", "ruido", "imu_sesgo"]
etq = ["sin\ndegradar", "error\nde giro", "error\nlineal 5 %", "giro +\nlineal", "giro + lineal\n+ ruido", "giro + lineal\n+ sesgo IMU"]
med = lambda c, e: np.mean([float(r["ate_inicio_m"]) for r in filas if r["cond"] == c and r["estimador"] == e])
odo = [med(c, "odom_degradada") for c in conds]; ekf = [med(c, "ekf") for c in conds]
amcl = [med(c, "slam") for c in conds]
x = np.arange(len(conds)); w = 0.36
fig, ax = plt.subplots(figsize=(10, 5))
b1 = ax.bar(x - w / 2 - 0.01, odo, w, color=NARANJA, label="odometría degradada")
b2 = ax.bar(x + w / 2 + 0.01, ekf, w, color=AZUL, label="EKF: odometría + IMU")
franja = ax.axhspan(min(amcl), max(amcl), color=AQUA, alpha=0.35, lw=0,
                    label=f"AMCL con mapa: {min(amcl):.2f}–{max(amcl):.2f} m en todas".replace(".", ","))
for b, v in list(zip(b1, odo)) + list(zip(b2, ekf)):
    ax.text(b.get_x() + b.get_width() / 2, v + 0.06, f"{v:.2f}".replace(".", ","), ha="center", color=TINTA, fontsize=11)
ax.set_xticks(x, etq, fontsize=11.5); ax.set_ylabel("ATE (m), media de 2 repeticiones")
ax.yaxis.grid(color=REJILLA, lw=0.8); ax.set_axisbelow(True); ax.set_ylim(0, 4.9)
ax.legend(loc="lower center", bbox_to_anchor=(0.5, 1.0), ncol=3, frameon=False, fontsize=12)
fig.savefig(OUT / "ate_condiciones.png", bbox_inches="tight"); plt.close(fig)

# ---------- error de orientacion frente al tiempo: el sesgo de la IMU ----------
fig, ax = plt.subplots(figsize=(9, 4.8))
for cond, col, et in [("imu_sesgo", AZUL, "EKF con sesgo de IMU 0,002 rad/s"), ("ambos", AQUA, "EKF sin sesgo de IMU")]:
    for k, dd in enumerate(sorted(EKF.glob(f"resultados/2026100*_{cond}"))):
        tr, pr, yr = leer_tum(dd / "verdad.tum"); t, p, y = leer_tum(dd / "ekf.tum"); ie, ir = asociar(t, tr)
        yu, yru = np.unwrap(y[ie]), np.unwrap(yr[ir])
        err = np.degrees((yu - yu[0]) - (yru - yru[0])); tt = t[ie] - t[ie][0]
        ax.plot(tt, err, color=col, lw=2, label=et if k == 0 else None)
T = np.linspace(0, 280, 50)
ax.plot(T, np.degrees(0.002 * T), "--", color=TINTA2, lw=1.6, label="predicción: sesgo × tiempo")
ax.set_xlabel("tiempo (s)"); ax.set_ylabel("error de orientación (°)")
ax.yaxis.grid(color=REJILLA, lw=0.8); ax.set_axisbelow(True)
ax.legend(loc="upper left", frameon=False, fontsize=12)
fig.savefig(OUT / "sesgo_imu.png", bbox_inches="tight"); plt.close(fig)
print("figuras en", OUT, "| ATE odo", np.round(odo, 2), "ekf", np.round(ekf, 2))
