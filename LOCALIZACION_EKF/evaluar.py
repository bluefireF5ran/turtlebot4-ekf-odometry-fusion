#!/usr/bin/env python3
"""Evalua trayectorias estimadas contra la verdad del simulador: ATE y RPE.

Es la misma metodologia que el trabajo de SfM de Vision por Computador (ATE
tras alineacion de Umeyama, RPE entre poses), aplicada a odometria.

    python3 evaluar.py resultados/<carpeta_de_una_ejecucion>
    python3 evaluar.py resultados/*/            # varias ejecuciones -> tabla resumen

Cada carpeta tiene ficheros TUM (t x y z qx qy qz qw), uno por estimador, y
verdad.tum. Escribe en la carpeta: metricas.csv y trayectorias.png.

No necesita ROS: solo numpy y matplotlib. Se puede probar en Windows con
    python evaluar.py --prueba
"""
import argparse
import csv
import math
import sys
from pathlib import Path

import numpy as np

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")


def leer_tum(ruta):
    d = np.loadtxt(ruta, comments="#", ndmin=2)
    t, x, y = d[:, 0], d[:, 1], d[:, 2]
    qz, qw = d[:, 6], d[:, 7]
    yaw = 2 * np.arctan2(qz, qw)
    return t, np.stack([x, y], 1), yaw


def asociar(t_est, t_ref, tolerancia=0.05):
    """Para cada instante estimado, el indice de la verdad mas cercana en tiempo."""
    idx = np.clip(np.searchsorted(t_ref, t_est), 1, len(t_ref) - 1)
    previo = idx - 1
    idx = np.where(np.abs(t_ref[previo] - t_est) < np.abs(t_ref[idx] - t_est), previo, idx)
    ok = np.abs(t_ref[idx] - t_est) <= tolerancia
    return np.nonzero(ok)[0], idx[ok]


def umeyama_2d(a, b):
    """Rotacion R y traslacion t (sin escala: la odometria es metrica) que
    minimizan |R a + t - b|^2."""
    ma, mb = a.mean(0), b.mean(0)
    H = (a - ma).T @ (b - mb)
    U, _, Vt = np.linalg.svd(H)
    D = np.diag([1, np.sign(np.linalg.det(Vt.T @ U.T))])
    R = Vt.T @ D @ U.T
    return R, mb - R @ ma


def alinear_inicio(p, yaw, p_ref, yaw_ref):
    """Alinea con la pose inicial de la verdad. Para odometria es lo honesto:
    la deriva se acumula desde el arranque y Umeyama la reparte y la esconde."""
    d = yaw_ref[0] - yaw[0]
    R = np.array([[math.cos(d), -math.sin(d)], [math.sin(d), math.cos(d)]])
    return (p - p[0]) @ R.T + p_ref[0]


def rpe(p, yaw, p_ref, yaw_ref, paso_m=1.0):
    """Error relativo por cada `paso_m` metros recorridos (segun la verdad):
    error de traslacion en % de la distancia y error de giro en grados por metro."""
    dist = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(p_ref, axis=0), axis=1))])
    errores_t, errores_r = [], []
    j = 0
    for i in range(len(p_ref)):
        while j < len(p_ref) and dist[j] - dist[i] < paso_m:
            j += 1
        if j >= len(p_ref):
            break
        def relativo(pp, yy):
            c, s = math.cos(-yy[i]), math.sin(-yy[i])
            dx, dy = pp[j] - pp[i]
            return np.array([c * dx - s * dy, s * dx + c * dy]), yy[j] - yy[i]
        dt_e, dy_e = relativo(p, yaw)
        dt_r, dy_r = relativo(p_ref, yaw_ref)
        errores_t.append(np.linalg.norm(dt_e - dt_r) / paso_m * 100)
        errores_r.append(abs(math.degrees(math.atan2(math.sin(dy_e - dy_r), math.cos(dy_e - dy_r)))) / paso_m)
    if not errores_t:
        return float("nan"), float("nan")
    return float(np.mean(errores_t)), float(np.mean(errores_r))


def evaluar_carpeta(carpeta, figura=True):
    carpeta = Path(carpeta)
    t_ref, p_ref, yaw_ref = leer_tum(carpeta / "verdad.tum")
    recorrido = float(np.linalg.norm(np.diff(p_ref, axis=0), axis=1).sum())
    filas, curvas = [], {}
    for f in sorted(carpeta.glob("*.tum")):
        if f.stem == "verdad":
            continue
        t, p, yaw = leer_tum(f)
        ie, ir = asociar(t, t_ref)
        if len(ie) < 10:
            print(f"  {f.stem}: solo {len(ie)} poses asociadas en tiempo, se ignora")
            continue
        p, yaw, pr, yr = p[ie], yaw[ie], p_ref[ir], yaw_ref[ir]
        p_ini = alinear_inicio(p, yaw, pr, yr)
        ate_ini = float(np.sqrt(np.mean(np.sum((p_ini - pr) ** 2, 1))))
        final = float(np.linalg.norm(p_ini[-1] - pr[-1]))
        R, tr = umeyama_2d(p, pr)
        ate_um = float(np.sqrt(np.mean(np.sum((p @ R.T + tr - pr) ** 2, 1))))
        rpe_t, rpe_r = rpe(p, yaw, pr, yr)
        # Escalas: separan el error de distancia (radio de rueda) del de giro
        # (distancia entre ruedas), que el ATE mezcla. Giro neto con signo (el
        # recorrido gira siempre hacia el mismo lado) y distancia muestreada a
        # ~1 Hz, para que el temblor de AMCL no las infle.
        yu, yru = np.unwrap(yaw), np.unwrap(yr)
        giro_ref = yru[-1] - yru[0]
        cada = max(1, int(round(1.0 / max(np.median(np.diff(t[ie])), 1e-3))))
        dist = lambda q: float(np.linalg.norm(np.diff(q[::cada], axis=0), axis=1).sum())
        filas.append(dict(ejecucion=carpeta.name, estimador=f.stem, recorrido_m=round(recorrido, 2),
                          ate_inicio_m=round(ate_ini, 4), ate_umeyama_m=round(ate_um, 4),
                          error_final_m=round(final, 4), error_final_pct=round(100 * final / recorrido, 2),
                          rpe_tras_pct=round(rpe_t, 2), rpe_giro_grad_m=round(rpe_r, 3),
                          escala_dist=round(dist(p) / dist(pr), 4),
                          escala_giro=round((yu[-1] - yu[0]) / giro_ref, 4) if abs(giro_ref) > 1 else float("nan"),
                          error_yaw_final_grad=round(math.degrees((yu[-1] - yu[0]) - giro_ref), 2)))
        curvas[f.stem] = p_ini
    if not filas:
        return []
    with open(carpeta / "metricas.csv", "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(filas[0]))
        w.writeheader()
        w.writerows(filas)
    if figura:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(7, 7))
        ax.plot(p_ref[:, 0], p_ref[:, 1], "k", lw=2.5, label="verdad (simulador)")
        for nombre, c in curvas.items():
            ax.plot(c[:, 0], c[:, 1], lw=1.5, label=nombre)
        ax.set_aspect("equal")
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        ax.grid(alpha=0.3)
        ax.legend()
        ax.set_title(f"{carpeta.name}: {recorrido:.1f} m recorridos")
        fig.savefig(carpeta / "trayectorias.png", dpi=120, bbox_inches="tight")
        plt.close(fig)
    return filas


def prueba_sintetica(carpeta):
    """Genera una vuelta cuadrada con odometria degradada y comprueba que las
    metricas ordenan bien: verdad < ekf < degradada."""
    carpeta = Path(carpeta)
    carpeta.mkdir(parents=True, exist_ok=True)
    dt, v = 0.05, 0.3
    t, x, y, yaw = 0.0, 0.0, 0.0, 0.0
    rng = np.random.default_rng(0)
    est = {"verdad": [], "odom_degradada": [], "ekf": []}
    pose = {k: [0.0, 0.0, 0.0] for k in est}
    for vuelta in range(3):
        for lado in range(4):
            n_giro = int((math.pi / 2) / 0.5 / dt)
            for vx, w, n in [(v, 0.0, int(2.0 / v / dt)), (0.0, (math.pi / 2) / (n_giro * dt), n_giro)]:
                for _ in range(n):
                    t += dt
                    for k, p in pose.items():
                        vk, wk = vx, w
                        if k == "odom_degradada":
                            vk, wk = vx * 1.03, w * 1.04 + 0.01
                        if k == "ekf":
                            vk, wk = vx * 1.03, w + rng.normal(0, 0.002)
                        p[2] += wk * dt
                        p[0] += vk * math.cos(p[2]) * dt
                        p[1] += vk * math.sin(p[2]) * dt
                        est[k].append((t, p[0], p[1], 0, 0, 0, math.sin(p[2] / 2), math.cos(p[2] / 2)))
    for k, filas in est.items():
        np.savetxt(carpeta / f"{k}.tum", np.array(filas), fmt="%.6f")
    filas = evaluar_carpeta(carpeta)
    ate = {f["estimador"]: f["ate_inicio_m"] for f in filas}
    assert ate["ekf"] < ate["odom_degradada"], ate
    print("Prueba sintetica OK:", ate)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("carpetas", nargs="*")
    ap.add_argument("--prueba", action="store_true")
    args = ap.parse_args()
    if args.prueba:
        prueba_sintetica(Path(__file__).parent / "resultados" / "prueba_sintetica")
        return
    todas = []
    for c in args.carpetas:
        todas += evaluar_carpeta(c)
    if not todas:
        print("Nada que evaluar.")
        return
    print(f"\n{'ejecucion':28s} {'estimador':18s} {'ATE ini (m)':>11s} {'ATE Umey (m)':>12s} "
          f"{'final %':>8s} {'RPE %':>6s} {'giro °/m':>8s} {'esc dist':>8s} {'esc giro':>8s} {'yaw fin °':>9s}")
    for f in todas:
        print(f"{f['ejecucion']:28s} {f['estimador']:18s} {f['ate_inicio_m']:11.3f} {f['ate_umeyama_m']:12.3f} "
              f"{f['error_final_pct']:8.2f} {f['rpe_tras_pct']:6.2f} {f['rpe_giro_grad_m']:8.3f} "
              f"{f['escala_dist']:8.3f} {f['escala_giro']:8.3f} {f['error_yaw_final_grad']:9.1f}")
    # Carpetas 'AAAAMMDD_HHMMSS_<condicion>': se agrupan las repeticiones.
    condicion = lambda f: f["ejecucion"].split("_", 2)[-1]
    print("\nPor condicion y estimador (media ± desviacion entre repeticiones):")
    print(f"  {'condicion':12s} {'estimador':16s} {'n':>2s} {'ATE ini (m)':>15s} {'final %':>13s} "
          f"{'esc dist':>7s} {'esc giro':>8s}")
    for cond in sorted({condicion(f) for f in todas}):
        for est in sorted({f["estimador"] for f in todas if condicion(f) == cond}):
            sub = [f for f in todas if condicion(f) == cond and f["estimador"] == est]
            m = lambda k: (np.mean([f[k] for f in sub]), np.std([f[k] for f in sub]))
            (a, sa), (fp, sf) = m("ate_inicio_m"), m("error_final_pct")
            print(f"  {cond:12s} {est:16s} {len(sub):2d} {a:8.3f} ± {sa:5.3f} {fp:6.2f} ± {sf:4.2f} "
                  f"{m('escala_dist')[0]:7.3f} {m('escala_giro')[0]:8.3f}")


if __name__ == "__main__":
    main()
