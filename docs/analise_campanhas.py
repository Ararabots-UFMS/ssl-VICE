#!/usr/bin/env python3
"""Comparison figures and numbers across the shot campaigns, for analise-campanhas.md.

    python3 analise_campanhas.py        # writes docs/analise-campanhas/*.png

Each experiment folder is one campanha_chute.py run (tentativas.csv). Pairs of runs
on the same seed share ball and robot placements attempt by attempt.
"""
import collections
import csv
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt   # noqa: E402
import numpy as np                # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from campanha_graficos import CORES, fracao_coberta, wilson  # noqa: E402

AQUI = os.path.dirname(os.path.abspath(__file__))
SAIDA = os.path.join(AQUI, "analise-campanhas")

# folder, label, seed
EXPERIMENTOS = [
    ("campanha_antes", "Baseline, before refactor", "1"),
    ("campanha", "After refactor", "1"),
    ("campo_centro", "Push through centre", "1"),
    ("goleiro_linha", "Keeper on the line", "1"),
    ("goleiro_area", "Keeper in the area", "1"),
    ("linha_so_chegada", "Keeper on the line, A: line-up", "1"),
    ("linha_chegada_vao", "Keeper on the line, B: + gap gate", "1"),
    ("linha_tudo", "Keeper on the line, C: + predicted turn", "1"),
    ("linha_na_linha", "Keeper on the line, D: + push on the line", "1"),
]
PERTO_DO_GOL = [
    ("area_sem_recorte", "Before (planner area fix only)"),
    ("area_com_recorte", "+ push target clipped at the area"),
    ("area_na_linha", "+ push target on the line (stopped at 50)"),
    ("area_centro", "+ push through the ball centre"),
]
ORDEM = ["gol", "defendido", "fora", "curto", "sem_chute", "gol_contra"]


def carregar(pasta, seed=None):
    caminho = os.path.join(AQUI, pasta, "tentativas.csv")
    if not os.path.exists(caminho):
        return {}
    with open(caminho) as f:
        return {int(l["tentativa"]): l for l in csv.DictReader(f)
                if l["resultado"] != "ERRO" and (seed is None or l["seed"] == seed)}


def num(l, campo):
    v = l.get(campo, "")
    return float(v) if v not in ("", None) else math.nan


def salvar(fig, nome):
    fig.tight_layout()
    fig.savefig(os.path.join(SAIDA, nome), dpi=130)
    plt.close(fig)


def resumo(nome, d, ks=None):
    ks = sorted(d) if ks is None else ks
    c = collections.Counter(d[k]["resultado"] for k in ks)
    t = [num(d[k], "t_chute") for k in ks if d[k]["t_chute"]]
    arr = [num(d[k], "arrasto") for k in ks if d[k]["arrasto"]]
    p, lo, hi = wilson(c["gol"], len(ks))
    return dict(nome=nome, n=len(ks), c=c, gol=p, lo=lo, hi=hi,
                t50=np.median(t) if t else math.nan,
                t90=np.percentile(t, 90) if t else math.nan,
                empurrou=sum(a > 300 for a in arr))


def linha_md(r):
    c = r["c"]
    return (f"| {r['nome']} | {r['n']} | **{r['gol'] * 100:.1f}%** ({r['lo'] * 100:.0f}–{r['hi'] * 100:.0f}) | "
            f"{c['defendido']} | {c['fora']} | {c['curto']} | {c['sem_chute']} | {c['gol_contra']} | "
            f"{r['t50']:.1f} / {r['t90']:.1f} | {r['empurrou']} |")


def mira_da_estrategia():
    """The aim the strategy's goal rater picks from where the ball was kicked.

    The rater (skills/avaliacao_chute.melhor_alvo) is pure geometry, so with a static
    keeper it can be rerun here exactly. Needs src/strategy and src/utils importable.
    """
    raiz = os.path.dirname(AQUI)
    sys.path[:0] = [os.path.join(raiz, "src", "strategy"), os.path.join(raiz, "src", "utils")]
    from types import SimpleNamespace
    from unittest.mock import MagicMock
    for m in ("movement_interfaces", "movement_interfaces.msg"):
        sys.modules.setdefault(m, MagicMock())
    try:
        from strategy.skills import avaliacao_chute
    except ImportError:
        return []
    gol = SimpleNamespace(x=4500.0, y=0.0)
    saida = []
    for p in ("goleiro_linha", "goleiro_area"):
        for l in carregar(p).values():
            if l["chutou"] != "1" or not l["y_na_linha"] or not l["goleiro_x"]:
                continue
            bola = SimpleNamespace(position_x=num(l, "bola_chute_x"), position_y=num(l, "bola_chute_y"))
            goleiro = {0: SimpleNamespace(position_x=num(l, "goleiro_x"), position_y=num(l, "goleiro_y"))}
            _, y, nota = avaliacao_chute.melhor_alvo(bola, gol, goleiro)
            if y is not None:
                saida.append(dict(pasta=p, erro=num(l, "y_na_linha") - y, nota=nota,
                                  resultado=l["resultado"]))
    return saida


def main():
    os.makedirs(SAIDA, exist_ok=True)
    print("| experiment | attempts | goals (95% CI) | saved | wide | short | no kick | own goal | "
          "time to kick median / p90 (s) | ball pushed >300 mm first |")
    print("|---|---|---|---|---|---|---|---|---|---|")

    # 1. Outcomes of every experiment.
    tabela = []
    for pasta, nome, seed in EXPERIMENTOS:
        d = carregar(pasta, seed)
        if d:
            tabela.append(resumo(nome, d))
    for pasta, nome in PERTO_DO_GOL:
        d = carregar(pasta)
        if d:
            tabela.append(resumo("Near goal: " + nome, d))
    for r in tabela:
        print(linha_md(r))

    fig, ax = plt.subplots(figsize=(10, 0.55 * len(tabela) + 1.2))
    for i, r in enumerate(tabela):
        esq = 0.0
        for k in ORDEM:
            v = r["c"][k] / r["n"] * 100
            ax.barh(i, v, left=esq, color=CORES[k], label=k if i == 0 else None)
            esq += v
    ax.set_yticks(range(len(tabela)), [f"{r['nome']} (n={r['n']})" for r in tabela], fontsize=8)
    ax.invert_yaxis(); ax.set_xlabel("% of attempts"); ax.set_xlim(0, 100)
    ax.legend(ncol=6, fontsize=8, loc="lower center", bbox_to_anchor=(0.5, 1.0))
    ax.set_title("Outcome of every experiment", pad=24)
    salvar(fig, "01_resultados.png")

    # 2. Paired comparisons: same placements, different code or conditions.
    print("\n## Paired")
    base = carregar("campanha_antes", "1")
    pares = [("After refactor", carregar("campanha", "1")),
             ("Push through centre", carregar("campo_centro", "1")),
             ("Keeper on the line", carregar("goleiro_linha", "1")),
             ("Keeper in the area", carregar("goleiro_area", "1"))]
    for nome, d in pares:
        ks = sorted(set(base) & set(d))
        if not ks:
            continue
        a, b = resumo("baseline", base, ks), resumo(nome, d, ks)
        mudou = collections.Counter((base[k]["resultado"], d[k]["resultado"]) for k in ks
                                    if base[k]["resultado"] != d[k]["resultado"])
        print(f"- {nome}: same {len(ks)} placements, baseline {a['c']['gol']} goals vs {b['c']['gol']}; "
              f"changed {dict(mudou)}")
    a = carregar("area_sem_recorte")
    for pasta, nome in PERTO_DO_GOL[1:]:
        d = carregar(pasta)
        ks = sorted(set(a) & set(d))
        print(f"- near goal {nome}: same {len(ks)} placements, before {resumo('', a, ks)['c']['gol']} "
              f"goals vs {resumo('', d, ks)['c']['gol']}")

    # 3. Near their goal: the four variants with intervals.
    fig, ax = plt.subplots(figsize=(8, 3.4))
    ref = carregar("area_sem_recorte")
    for i, (pasta, nome) in enumerate(PERTO_DO_GOL):
        d = carregar(pasta)
        ks = sorted(set(ref) & set(d))
        r = resumo(nome, d, ks)
        ax.errorbar(r["gol"] * 100, i, xerr=[[(r["gol"] - r["lo"]) * 100], [(r["hi"] - r["gol"]) * 100]],
                    fmt="o", color="#264653", capsize=4)
        ax.annotate(f"{r['c']['gol']}/{r['n']}, {r['c']['sem_chute']} no kick", (r["gol"] * 100, i),
                    textcoords="offset points", xytext=(8, 6), fontsize=8)
    ax.set_yticks(range(len(PERTO_DO_GOL)), [n for _, n in PERTO_DO_GOL], fontsize=8)
    ax.invert_yaxis(); ax.set_xlim(70, 102); ax.set_xlabel("goals per attempt (%), 95% interval")
    ax.set_title("Ball near their area (x 2500–3400, |y| ≤ 1500), same placements")
    ax.grid(alpha=0.3)
    salvar(fig, "02_perto_do_gol.png")

    # 4. First touch: where the ball sat on the robot's front.
    toques = [l for p in ("goleiro_linha", "goleiro_area") for l in carregar(p).values()
              if l.get("toque_yy", "") != ""]
    if toques:
        xx = np.array([num(l, "toque_xx") for l in toques])
        yy = np.array([num(l, "toque_yy") for l in toques])
        chute = np.array([l["toque_foi_chute"] == "1" for l in toques])
        armado = np.array([l["toque_armado"] == "1" for l in toques])
        res = np.array([l["resultado"] for l in toques])
        fig, eixos = plt.subplots(1, 3, figsize=(15, 4.2))
        ax = eixos[0]
        ax.add_patch(plt.Rectangle((-40, 0), 80, 31.5, color="#2a9d8f", alpha=0.2, label="grSim fires here"))
        ax.scatter(yy[chute], xx[chute], s=10, color="#2a9d8f", label="first touch was the kick")
        ax.scatter(yy[~chute], xx[~chute], s=14, color="#e76f51", label="first touch was a push")
        ax.set_xlim(-200, 200); ax.set_ylim(-150, 120); ax.axvline(0, color="#999", lw=0.6)
        ax.set_xlabel("ball sideways on the robot's front, yy (mm)")
        ax.set_ylabel("ball ahead of the kicker plate, xx (mm)")
        ax.legend(fontsize=7, loc="lower left"); ax.set_title("Where the ball was at the first touch")
        eixos[1].hist([np.abs(yy[chute]), np.abs(yy[~chute])], bins=np.arange(0, 200, 10), stacked=True,
                      color=["#2a9d8f", "#e76f51"], label=["kick", "push"])
        eixos[1].axvline(40, color="#264653", ls="--", lw=0.9, label="grSim limit (40 mm)")
        eixos[1].set_xlabel("|yy| at the first touch (mm)"); eixos[1].set_ylabel("attempts")
        eixos[1].legend(fontsize=7); eixos[1].set_title("Sideways offset of the ball at the first touch")
        nomes = [k for k in ORDEM if (res == k).any()]
        larg = 0.38
        for j, (sel, rot, cor) in enumerate(((chute, "first touch = kick", "#2a9d8f"),
                                             (~chute, "first touch = push", "#e76f51"))):
            tot = max(1, sel.sum())
            eixos[2].bar(np.arange(len(nomes)) + (j - 0.5) * larg,
                         [((res == k) & sel).sum() / tot * 100 for k in nomes], larg, color=cor, label=rot)
        eixos[2].set_xticks(range(len(nomes)), nomes); eixos[2].set_ylabel("% of those attempts")
        eixos[2].legend(fontsize=7); eixos[2].set_title("How the attempt ended")
        salvar(fig, "03_primeiro_toque.png")
        print(f"\n## First touch ({len(toques)} attempts, keeper runs)")
        print(f"- kick: {chute.sum()}  push: {(~chute).sum()} ({(~chute).mean() * 100:.0f}%), "
              f"armed at the push: {(armado & ~chute).sum()}/{(~chute).sum()}")
        print(f"- |yy| kick median {np.median(np.abs(yy[chute])):.0f} p90 {np.percentile(np.abs(yy[chute]), 90):.0f}; "
              f"push median {np.median(np.abs(yy[~chute])):.0f} p90 {np.percentile(np.abs(yy[~chute]), 90):.0f}")
        print(f"- push outcomes {dict(collections.Counter(res[~chute]))}; "
              f"kick outcomes {dict(collections.Counter(res[chute]))}")
        print(f"- no-kick attempts whose first touch was a push: "
              f"{((res == 'sem_chute') & ~chute).sum()}/{(res == 'sem_chute').sum()}")
        t_c = [num(l, "t_chute") for l, c in zip(toques, chute) if c and l["t_chute"]]
        t_p = [num(l, "t_chute") for l, c in zip(toques, chute) if not c and l["t_chute"]]
        if t_p:
            print(f"- time to kick: first touch kick median {np.median(t_c):.1f} s, "
                  f"after a push median {np.median(t_p):.1f} s")

    # 5. Against the keeper.
    linhas = {p: carregar(p) for p in ("goleiro_linha", "goleiro_area")}
    if any(linhas.values()):
        fig, eixos = plt.subplots(1, 3, figsize=(15, 4.2))
        for p, cor, rot in (("goleiro_linha", "#264653", "keeper on the line"),
                            ("goleiro_area", "#e76f51", "keeper in the area")):
            d = list(linhas[p].values())
            if not d:
                continue
            bx = np.array([num(l, "bola_x") for l in d]); by = np.array([num(l, "bola_y") for l in d])
            kx = np.array([num(l, "goleiro_x") for l in d]); ky = np.array([num(l, "goleiro_y") for l in d])
            gol = np.array([l["resultado"] == "gol" for l in d])
            defen = np.array([l["resultado"] == "defendido" for l in d])
            cob = np.array([fracao_coberta(*v) for v in zip(bx, by, kx, ky)])
            bordas = np.linspace(0, 0.8, 9)
            c, t, e = [], [], []
            for a, b in zip(bordas, bordas[1:]):
                m = (cob >= a) & (cob < b)
                if m.sum() >= 5:
                    pp, lo, hi = wilson(int(gol[m].sum()), int(m.sum()))
                    c.append((a + b) / 2); t.append(pp * 100); e.append([max(0.0, pp - lo) * 100, max(0.0, hi - pp) * 100])
            if c:
                eixos[0].errorbar(c, t, yerr=np.array(e).T, fmt="o-", capsize=3, color=cor, label=rot)
            # Distance of the keeper from its goal line.
            dist = 4500 - kx
            bordas = np.arange(0, 1001, 200)
            c, t = [], []
            for a, b in zip(bordas, bordas[1:]):
                m = (dist >= a) & (dist < b)
                if m.sum() >= 5:
                    c.append((a + b) / 2); t.append(defen[m].mean() * 100)
            if c:
                eixos[1].plot(c, t, "o-", color=cor, label=rot)
            # Is the shot placed away from the keeper?
            yl = np.array([num(l, "y_na_linha") for l in d])
            ok = ~np.isnan(yl) & (np.abs(yl) < 1500)
            eixos[2].scatter(ky[ok], yl[ok], s=8, color=cor, alpha=0.6, label=rot)
        eixos[0].set_xlabel("share of the goal mouth the keeper hides, seen from the ball")
        eixos[0].set_ylabel("goals per attempt (%)"); eixos[0].set_ylim(0, 105)
        eixos[0].set_title("Goals vs how much goal the keeper covers"); eixos[0].legend(fontsize=7)
        eixos[0].grid(alpha=0.3)
        eixos[1].set_xlabel("keeper distance from its goal line (mm)"); eixos[1].set_ylabel("saved (%)")
        eixos[1].set_title("Saves vs how far the keeper came out"); eixos[1].legend(fontsize=7)
        eixos[1].grid(alpha=0.3)
        eixos[2].axhline(500, color="#e76f51", lw=0.8); eixos[2].axhline(-500, color="#e76f51", lw=0.8)
        eixos[2].plot([-1000, 1000], [-1000, 1000], ":", color="#999")
        eixos[2].set_xlabel("keeper y (mm)"); eixos[2].set_ylabel("where the shot met the goal line (mm)")
        eixos[2].set_title("Shots go to the side away from the keeper"); eixos[2].legend(fontsize=7)
        salvar(fig, "04_goleiro.png")
        for p, d in linhas.items():
            if not d:
                continue
            res = collections.Counter(l["resultado"] for l in d.values())
            chutes = sum(l["chutou"] == "1" for l in d.values())
            print(f"\n## {p}: {len(d)} attempts {dict(res)}; saved {res['defendido']}/{chutes} shots")
            for l in d.values():
                if l["resultado"] == "defendido":
                    print(f"  saved #{l['tentativa']}: ball ({l['bola_x']},{l['bola_y']}) keeper "
                          f"({l['goleiro_x']},{l['goleiro_y']}) shot y {l['y_na_linha']} "
                          f"dir err {l['erro_direcao']} body err {l['erro_corpo']}")

    # 6. Against the strategy's own aim: rebuilt offline from the ball and the keeper.
    mira = mira_da_estrategia()
    if mira:
        fig, eixos = plt.subplots(1, 2, figsize=(11, 3.9))
        for p, cor, rot in (("goleiro_linha", "#264653", "keeper on the line"),
                            ("goleiro_area", "#e76f51", "keeper in the area")):
            pts = [m for m in mira if m["pasta"] == p]
            if not pts:
                continue
            erro = np.array([m["erro"] for m in pts]); nota = np.array([m["nota"] for m in pts])
            salvo = np.array([m["resultado"] == "defendido" for m in pts])
            eixos[0].hist(np.clip(np.abs(erro), 0, 1000), bins=np.arange(0, 1001, 50), alpha=0.6,
                          color=cor, label=rot)
            eixos[1].scatter(nota[~salvo], np.clip(np.abs(erro[~salvo]), 0, 1500), s=10, color=cor,
                             alpha=0.5, label=rot)
            eixos[1].scatter(nota[salvo], np.clip(np.abs(erro[salvo]), 0, 1500), s=60, marker="x",
                             color="#3a86ff", label="saved" if p == "goleiro_linha" else None)
            print(f"- {p}: {len(pts)} shots, |landing - aim| median {np.median(np.abs(erro)):.0f} mm, "
                  f"p90 {np.percentile(np.abs(erro), 90):.0f} mm; saved {int(salvo.sum())}, gap score of "
                  f"saved shots {np.round(nota[salvo], 2).tolist()}")
        eixos[0].set_xlabel("|where the shot met the goal line - the strategy's aim| (mm, clipped at 1000)")
        eixos[0].set_ylabel("shots"); eixos[0].legend(fontsize=7)
        eixos[0].set_title("How far from its own aim the shot landed")
        eixos[1].set_xlabel("gap score from the goal rater (1 = the gap is 0.3 rad or wider)")
        eixos[1].set_ylabel("|landing - aim| (mm)"); eixos[1].legend(fontsize=7); eixos[1].grid(alpha=0.3)
        eixos[1].set_title("Saved shots: narrow gaps and bad kicks")
        salvar(fig, "06_mira.png")

    # 7. Time to kick, every experiment.
    fig, ax = plt.subplots(figsize=(10, 3.8))
    dados, rot = [], []
    for pasta, nome, seed in EXPERIMENTOS:
        d = carregar(pasta, seed)
        t = [num(l, "t_chute") for l in d.values() if l["t_chute"]]
        if t:
            dados.append(t); rot.append(f"{nome}\n(n={len(t)})")
    ax.boxplot(dados, tick_labels=rot, showfliers=True, flierprops=dict(markersize=3))
    ax.set_ylabel("time to kick (s)"); ax.set_title("Time from the start to the kick"); ax.grid(alpha=0.3)
    plt.setp(ax.get_xticklabels(), fontsize=8)
    salvar(fig, "05_tempo.png")
    print("\nfigures in", SAIDA)
    return 0


if __name__ == "__main__":
    sys.exit(main())
