#!/usr/bin/env python3
"""Plots and a summary for the shot campaign (campanha_chute.py).

    python3 campanha_graficos.py [folder]      # default: ./campanha

Reads <folder>/tentativas.csv and trajetorias.jsonl, writes PNGs to
<folder>/graficos and a text summary to <folder>/resumo.md. Runs on the host:
needs only numpy and matplotlib.
"""
import csv
import json
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt   # noqa: E402
import numpy as np                # noqa: E402

GOL_X, GOL_MEIA = 4500.0, 500.0
CORES = {"gol": "#2a9d8f", "fora": "#e76f51", "curto": "#e9c46a",
         "sem_chute": "#8d99ae", "gol_contra": "#6a040f"}


def carregar(pasta):
    with open(os.path.join(pasta, "tentativas.csv")) as arquivo:
        linhas = [l for l in csv.DictReader(arquivo) if l["resultado"] != "ERRO"]
    return linhas


def col(linhas, nome):
    """A column as floats, NaN where the attempt has no value for it."""
    return np.array([float(l[nome]) if l.get(nome) not in (None, "") else np.nan for l in linhas])


def wilson(k, n, z=1.96):
    """Centre and half width of the 95% interval for a proportion."""
    if n == 0:
        return np.nan, np.nan, np.nan
    p = k / n
    centro = (p + z * z / (2 * n)) / (1 + z * z / n)
    meia = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / (1 + z * z / n)
    return p, max(0.0, centro - meia), min(1.0, centro + meia)


def taxa_por_faixa(ax, x, sucesso, bordas, rotulo_x, titulo, minimo=5):
    """Success rate per bin of x with its 95% interval; bins under `minimo` are skipped."""
    centros, taxas, baixo, alto, n_faixa = [], [], [], [], []
    for a, b in zip(bordas, bordas[1:]):
        dentro = (x >= a) & (x < b)
        n = int(dentro.sum())
        if n < minimo:
            continue
        p, lo, hi = wilson(int(sucesso[dentro].sum()), n)
        # At 0% or 100% rounding can leave the bound a hair past p.
        centros.append((a + b) / 2); taxas.append(p)
        baixo.append(max(0.0, p - lo)); alto.append(max(0.0, hi - p))
        n_faixa.append(n)
    ax.errorbar(centros, np.array(taxas) * 100, yerr=[np.array(baixo) * 100, np.array(alto) * 100],
                fmt="o-", color="#264653", capsize=3)
    for c, t, n in zip(centros, taxas, n_faixa):
        ax.annotate(f"n={n}", (c, t * 100), textcoords="offset points", xytext=(0, 8),
                    ha="center", fontsize=7, color="#555")
    ax.set_ylim(-5, 108); ax.set_xlabel(rotulo_x); ax.set_ylabel("%"); ax.set_title(titulo)
    ax.grid(alpha=0.3)


def campo(ax):
    ax.add_patch(plt.Rectangle((-4500, -3000), 9000, 6000, fill=False, color="#555"))
    ax.plot([0, 0], [-3000, 3000], color="#bbb", lw=0.8)
    for lado in (-1, 1):
        ax.add_patch(plt.Rectangle((lado * 4500 - (0 if lado < 0 else 1000), -1000), 1000, 2000,
                                   fill=False, color="#555"))
    ax.plot([GOL_X, GOL_X], [-GOL_MEIA, GOL_MEIA], color="#2a9d8f", lw=4)
    ax.set_xlim(-4700, 4900); ax.set_ylim(-3200, 3200); ax.set_aspect("equal")
    ax.set_xlabel("x (mm), attacking to the right"); ax.set_ylabel("y (mm)")


def salvar(fig, pasta, nome):
    fig.tight_layout()
    fig.savefig(os.path.join(pasta, nome), dpi=130)
    plt.close(fig)


def main():
    pasta = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(os.path.abspath(__file__)), "campanha")
    linhas = carregar(pasta)
    if not linhas:
        print("no attempts in", pasta)
        return 1
    saida = os.path.join(pasta, "graficos")
    os.makedirs(saida, exist_ok=True)

    n = len(linhas)
    res = np.array([l["resultado"] for l in linhas])
    gol = res == "gol"
    chutou = col(linhas, "chutou") == 1
    bx, by = col(linhas, "bola_x"), col(linhas, "bola_y")
    mira = col(linhas, "mira_y")
    erro_y, erro_dir, erro_corpo = col(linhas, "erro_y"), col(linhas, "erro_direcao"), col(linhas, "erro_corpo")
    d_chute, d_bola_gol = col(linhas, "dist_chute_gol"), col(linhas, "dist_bola_gol")
    t_chute, contorno, d_robo = col(linhas, "t_chute"), col(linhas, "angulo_contorno"), col(linhas, "dist_robo_bola")
    ang_gol = np.abs(col(linhas, "angulo_bola_gol"))
    y_linha, arrasto, recuo = col(linhas, "y_na_linha"), col(linhas, "arrasto"), col(linhas, "recuo")
    vel = col(linhas, "vel_saida")

    # 1. Outcomes.
    fig, ax = plt.subplots(figsize=(6, 3.6))
    nomes = [k for k in CORES if (res == k).any()]
    ax.bar(nomes, [(res == k).sum() for k in nomes], color=[CORES[k] for k in nomes])
    for i, k in enumerate(nomes):
        ax.text(i, (res == k).sum(), f"{(res == k).mean() * 100:.0f}%", ha="center", va="bottom")
    ax.set_title(f"Outcome of {n} attempts"); ax.set_ylabel("attempts")
    salvar(fig, saida, "01_resultados.png")

    # 2. Accuracy by distance.
    fig, eixos = plt.subplots(1, 2, figsize=(11, 3.8))
    taxa_por_faixa(eixos[0], d_chute[chutou], gol[chutou], np.arange(0, 7001, 750),
                   "distance from the kick to the aim point (mm)", "Goals per shot, by shot distance")
    taxa_por_faixa(eixos[1], d_bola_gol, gol, np.arange(0, 7001, 750),
                   "distance from the ball's start to the goal (mm)", "Goals per attempt, by starting distance")
    salvar(fig, saida, "02_precisao_por_distancia.png")

    # 3. Accuracy by ball position.
    fig, eixos = plt.subplots(1, 2, figsize=(13, 4.6))
    campo(eixos[0])
    passo = 750.0
    xs, ys = np.arange(-4500, 4501, passo), np.arange(-3000, 3001, passo)
    grade = np.full((len(ys) - 1, len(xs) - 1), np.nan)
    for i in range(len(xs) - 1):
        for j in range(len(ys) - 1):
            dentro = (bx >= xs[i]) & (bx < xs[i + 1]) & (by >= ys[j]) & (by < ys[j + 1])
            if dentro.sum() >= 3:
                grade[j, i] = gol[dentro].mean() * 100
    malha = eixos[0].pcolormesh(xs, ys, grade, cmap="RdYlGn", vmin=0, vmax=100, alpha=0.85)
    fig.colorbar(malha, ax=eixos[0], label="goals per attempt (%)")
    eixos[0].set_title("Where the ball started: goal rate (cells with 3+ attempts)")
    campo(eixos[1])
    for k in nomes:
        eixos[1].scatter(bx[res == k], by[res == k], s=14, color=CORES[k], label=k)
    eixos[1].legend(fontsize=7, loc="lower left"); eixos[1].set_title("Ball start, by outcome")
    salvar(fig, saida, "03_precisao_por_posicao.png")

    # 4. Accuracy by angle to the goal.
    fig, eixos = plt.subplots(1, 2, figsize=(11, 3.8))
    taxa_por_faixa(eixos[0], ang_gol, gol, np.arange(0, 91, 10),
                   "angle of the ball off the goal's axis (deg)", "Goals per attempt, by angle to the goal")
    ok = chutou & ~np.isnan(erro_y)
    eixos[1].scatter(ang_gol[ok], np.abs(erro_y[ok]), s=12, color="#264653", alpha=0.6)
    eixos[1].axhline(GOL_MEIA, color="#e76f51", ls="--", lw=0.8)
    eixos[1].set_xlabel("angle of the ball off the goal's axis (deg)"); eixos[1].set_ylabel("|miss on the goal line| (mm)")
    eixos[1].set_title("Miss distance by angle"); eixos[1].set_yscale("symlog", linthresh=100); eixos[1].grid(alpha=0.3)
    salvar(fig, saida, "04_precisao_por_angulo.png")

    # 5. Accuracy by aim point.
    miras = sorted(set(mira[~np.isnan(mira)]))
    fig, eixos = plt.subplots(1, 3, figsize=(15, 3.9))
    taxas = [wilson(int(gol[chutou & (mira == m)].sum()), int((chutou & (mira == m)).sum())) for m in miras]
    eixos[0].bar([str(int(m)) for m in miras], [t[0] * 100 for t in taxas], color="#2a9d8f",
                 yerr=[[max(0.0, t[0] - t[1]) * 100 for t in taxas], [max(0.0, t[2] - t[0]) * 100 for t in taxas]], capsize=3)
    eixos[0].set_xlabel("aim point inside the goal, y (mm)"); eixos[0].set_ylabel("goals per shot (%)")
    eixos[0].set_title("Goal rate by aim point"); eixos[0].set_ylim(0, 108)
    dados = [erro_y[ok & (mira == m)] for m in miras]
    if all(len(d) for d in dados):
        eixos[1].boxplot(dados, tick_labels=[str(int(m)) for m in miras], showfliers=False)
    eixos[1].axhline(0, color="#999", lw=0.8)
    eixos[1].set_xlabel("aim point, y (mm)"); eixos[1].set_ylabel("landing y - aim y (mm)")
    eixos[1].set_title("Signed miss by aim point (outliers hidden)")
    for m in miras:
        sel = ok & (mira == m)
        eixos[2].scatter(np.clip(y_linha[sel], -1500, 1500), np.full(sel.sum(), m) + np.random.default_rng(0).uniform(-40, 40, sel.sum()),
                         s=10, alpha=0.6)
        eixos[2].plot([m], [m], marker="x", color="k")
    eixos[2].axvline(-GOL_MEIA, color="#e76f51"); eixos[2].axvline(GOL_MEIA, color="#e76f51")
    eixos[2].set_xlabel("where the shot met the goal line, y (mm, clipped at 1500)"); eixos[2].set_ylabel("aim y (mm)")
    eixos[2].set_title("Landing point per aim (x = aim, red = posts)")
    salvar(fig, saida, "05_precisao_por_mira.png")

    # 6. Where the error comes from.
    fig, eixos = plt.subplots(1, 3, figsize=(15, 3.9))
    ed = erro_dir[chutou & ~np.isnan(erro_dir)]
    eixos[0].hist(np.clip(ed, -30, 30), bins=41, color="#264653")
    eixos[0].set_xlabel("shot direction error (deg, clipped at 30)"); eixos[0].set_title("Direction the ball left vs the aim line")
    sel = chutou & ~np.isnan(erro_dir) & ~np.isnan(erro_corpo)
    eixos[1].scatter(erro_corpo[sel], erro_dir[sel], s=12, c=[CORES.get(r, "#000") for r in res[sel]])
    lim = 30
    eixos[1].plot([-lim, lim], [-lim, lim], color="#999", lw=0.8)
    eixos[1].set_xlim(-lim, lim); eixos[1].set_ylim(-lim, lim); eixos[1].grid(alpha=0.3)
    eixos[1].set_xlabel("body error at the kick (deg)"); eixos[1].set_ylabel("shot direction error (deg)")
    eixos[1].set_title("Is the miss the body's aim? (on the diagonal = yes)")
    eixos[2].scatter(d_chute[sel], np.abs(erro_dir[sel]), s=12, color="#264653", alpha=0.6)
    d = np.linspace(500, 7000, 50)
    eixos[2].plot(d, np.degrees(np.arctan(GOL_MEIA / d)), color="#e76f51", ls="--", label="half goal seen from there")
    eixos[2].set_ylim(0, 30); eixos[2].legend(fontsize=7); eixos[2].grid(alpha=0.3)
    eixos[2].set_xlabel("shot distance (mm)"); eixos[2].set_ylabel("|direction error| (deg)")
    eixos[2].set_title("Direction error against the tolerance the goal allows")
    salvar(fig, saida, "06_origem_do_erro.png")

    # 7. The approach: time, kicks, dribbling.
    fig, eixos = plt.subplots(1, 3, figsize=(15, 3.9))
    pontos = eixos[0].scatter(contorno[chutou], t_chute[chutou], c=d_robo[chutou], s=14, cmap="viridis")
    fig.colorbar(pontos, ax=eixos[0], label="robot to ball at the start (mm)")
    eixos[0].set_xlabel("how far round the ball it had to go (deg)"); eixos[0].set_ylabel("time to kick (s)")
    eixos[0].set_title("Time to kick"); eixos[0].grid(alpha=0.3)
    taxa_por_faixa(eixos[1], contorno, chutou, np.arange(0, 181, 30),
                   "how far round the ball it had to go (deg)", "Attempts that ended in a kick")
    eixos[2].hist(arrasto[chutou], bins=30, color="#264653", alpha=0.8, label="ball moved before the kick")
    eixos[2].hist(recuo[~np.isnan(recuo)], bins=30, color="#e76f51", alpha=0.6, label="ball pushed toward our goal")
    eixos[2].set_xlabel("mm"); eixos[2].set_yscale("log"); eixos[2].legend(fontsize=7)
    eixos[2].set_title("Ball handling before the kick")
    salvar(fig, saida, "07_aproximacao.png")

    # 8. Shot traces.
    caminho = os.path.join(pasta, "trajetorias.jsonl")
    if os.path.exists(caminho):
        fig, ax = plt.subplots(figsize=(9, 6)); campo(ax)
        with open(caminho) as arquivo:
            for texto in arquivo:
                t = json.loads(texto)
                if t["bola"]:
                    b = np.array(t["bola"])
                    ax.plot(b[:, 0], b[:, 1], color=CORES.get(t["resultado"], "#000"), lw=0.6, alpha=0.5)
        ax.set_title("Ball path of every attempt, by outcome")
        salvar(fig, saida, "08_trajetorias.png")

    # Summary.
    def med(v):
        v = v[~np.isnan(v)]
        return (np.median(v), np.percentile(v, 90)) if len(v) else (float("nan"), float("nan"))
    nk = int(chutou.sum())
    p, lo, hi = wilson(int(gol.sum()), n)
    pk, lok, hik = wilson(int(gol[chutou].sum()), nk)
    texto = [f"# Shot campaign: {n} attempts", "",
             f"- Goals per attempt: {p * 100:.1f}% (95% interval {lo * 100:.0f}-{hi * 100:.0f}%)",
             f"- Attempts that kicked: {nk} ({nk / n * 100:.1f}%)",
             f"- Goals per shot: {pk * 100:.1f}% (95% interval {lok * 100:.0f}-{hik * 100:.0f}%)",
             "- Outcomes: " + ", ".join(f"{k} {(res == k).sum()}" for k in nomes),
             "- |miss on the goal line|: median %.0f mm, p90 %.0f mm" % med(np.abs(erro_y[chutou])),
             "- |shot direction error|: median %.1f deg, p90 %.1f deg" % med(np.abs(erro_dir[chutou])),
             "- |body error at the kick|: median %.1f deg, p90 %.1f deg" % med(np.abs(erro_corpo[chutou])),
             "- Time to kick: median %.1f s, p90 %.1f s" % med(t_chute[chutou]),
             "- Ball speed leaving the kick: median %.0f mm/s" % med(vel[chutou])[0],
             "- Ball pushed toward our goal by more than 100 mm: %d attempts" % int((recuo > 100).sum()), "",
             "## By aim point", "", "| aim y | shots | goals per shot | median signed miss (mm) |", "|---|---|---|---|"]
    for m in miras:
        sel = chutou & (mira == m)
        pm, _, _ = wilson(int(gol[sel].sum()), int(sel.sum()))
        texto.append(f"| {m:+.0f} | {int(sel.sum())} | {pm * 100:.0f}% | {np.nanmedian(erro_y[sel]) if sel.any() else float('nan'):+.0f} |")
    with open(os.path.join(pasta, "resumo.md"), "w") as arquivo:
        arquivo.write("\n".join(texto) + "\n")
    print("\n".join(texto))
    print("\nplots in", saida)
    return 0


if __name__ == "__main__":
    sys.exit(main())
