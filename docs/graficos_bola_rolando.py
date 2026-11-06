#!/usr/bin/env python3
"""Graficos da campanha de BOLA ROLANDO - um por pergunta desta investigacao.

    python3 graficos_bola_rolando.py campanha_final
    python3 graficos_bola_rolando.py --evolucao SAIDA rotulo=pasta ...

POR QUE NAO REUSA campanha_graficos.py: aquele responde as perguntas da bola
PARADA (precisao por distancia, por angulo, por mira). Aqui as perguntas sao
outras, e nenhuma delas aparece la:

    a bola ainda estava ANDANDO quando ele pegou?
    quantas investidas ele gasta ate ter a bola?
    ele chega antes de o atrito matar a bola?
    onde a bola encosta na placa do chutador?

Le <pasta>/antes/tentativas.csv e <pasta>/depois/tentativas.csv quando as duas
existem (par antes/depois), ou <pasta>/tentativas.csv sozinho.
"""
import csv
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt       # noqa: E402

DESACEL = 1350.0          # mm/s2, atrito da bola medido no grSim (28/10/2026)
PARADA = 150.0            # abaixo disto a bola esta parada para efeito de teste
PLACA = 40.0              # grSim so dispara com a bola a 40 mm do centro da placa
A, D = "#e76f51", "#2a9d8f"      # antes (vermelho), depois (verde)
FUNDO, TINTA, FRACO = "#11151c", "#cdd6e4", "#8d99ae"


def estilo(ax, titulo, x, y):
    ax.set_facecolor(FUNDO)
    ax.set_title(titulo, color=TINTA, fontsize=11, pad=9)
    ax.set_xlabel(x, color=FRACO, fontsize=9)
    ax.set_ylabel(y, color=FRACO, fontsize=9)
    ax.tick_params(colors=FRACO, labelsize=8)
    for lado in ax.spines.values():
        lado.set_color("#2a3140")
    ax.grid(True, color="#1d2430", lw=0.7)
    ax.set_axisbelow(True)


def salvar(fig, saida, nome):
    fig.patch.set_facecolor(FUNDO)
    fig.tight_layout()
    fig.savefig(os.path.join(saida, nome), dpi=130, facecolor=FUNDO)
    plt.close(fig)
    print("   ", nome)


def num(linha, campo):
    v = linha.get(campo, "")
    return float(v) if v not in ("", "None", None) else None


def ler(caminho):
    with open(caminho) as arquivo:
        return [l for l in csv.DictReader(arquivo) if l["resultado"] != "ERRO"]


def col(linhas, campo):
    return [v for v in (num(l, campo) for l in linhas) if v is not None]


# ----------------------------------------------------------------- 1
def g_velocidade_no_toque(grupos, saida):
    """A pergunta central: a bola ainda andava quando ele a pegou?"""
    fig, ax = plt.subplots(figsize=(7.2, 3.8))
    faixas = [0, 150, 400, 800, 1500, 2500, 4000]
    rotulos = ["parada\n<150", "150-400", "400-800", "800-1500", "1,5k-2,5k", ">2,5k"]
    larg = 0.8 / max(len(grupos), 1)
    for k, (nome, linhas) in enumerate(grupos.items()):
        v = col(linhas, "vel_bola_toque")
        cont = [sum(1 for x in v if faixas[i] <= x < faixas[i + 1])
                for i in range(len(faixas) - 1)]
        ax.bar([i + k * larg for i in range(len(cont))], cont, larg * 0.92,
               label=f"{nome} (n={len(v)})", color=(A if k == 0 else D))
    ax.set_xticks([i + larg * (len(grupos) - 1) / 2 for i in range(len(rotulos))])
    ax.set_xticklabels(rotulos, fontsize=8)
    estilo(ax, "A bola ainda andava quando ele a pegou?",
           "velocidade da bola no instante do contato (mm/s)", "tentativas")
    ax.legend(facecolor=FUNDO, edgecolor="#2a3140", labelcolor=TINTA, fontsize=8)
    salvar(fig, saida, "01_bola_andando_no_contato.png")


# ----------------------------------------------------------------- 2
def g_investidas(grupos, saida):
    """Quantas vezes ele chega na bola antes de finalmente leva-la."""
    fig, ax = plt.subplots(figsize=(7.2, 3.8))
    larg = 0.8 / max(len(grupos), 1)
    cats = ["1", "2", "3", "4+"]
    for k, (nome, linhas) in enumerate(grupos.items()):
        v = [int(x) for x in col(linhas, "investidas")]
        cont = [v.count(1), v.count(2), v.count(3), sum(1 for x in v if x >= 4)]
        ax.bar([i + k * larg for i in range(4)], cont, larg * 0.92,
               label=f"{nome} (mediana {sorted(v)[len(v) // 2] if v else 0})",
               color=(A if k == 0 else D))
    ax.set_xticks([i + larg * (len(grupos) - 1) / 2 for i in range(4)])
    ax.set_xticklabels(cats)
    estilo(ax, "Investidas ate ter a bola  (1 = pegou de primeira)",
           "numero de aproximacoes", "tentativas")
    ax.legend(facecolor=FUNDO, edgecolor="#2a3140", labelcolor=TINTA, fontsize=8)
    salvar(fig, saida, "02_investidas.png")


# ----------------------------------------------------------------- 3
def g_corrida_contra_atrito(grupos, saida):
    """Ele chega antes de o atrito matar a bola? Diagonal = empate."""
    fig, ax = plt.subplots(figsize=(5.6, 5.2))
    teto = 1.0
    for k, (nome, linhas) in enumerate(grupos.items()):
        xs, ys = [], []
        for l in linhas:
            tt = num(l, "t_toque")
            v = math.hypot(num(l, "bola_vx") or 0.0, num(l, "bola_vy") or 0.0)
            if tt is None or v <= 0:
                continue
            xs.append(v / DESACEL)       # quando a bola para
            ys.append(tt)                # quando ele pegou
        if xs:
            teto = max(teto, max(xs + ys))
        ax.scatter(xs, ys, s=34, color=(A if k == 0 else D), alpha=0.85,
                   edgecolors="none", label=nome)
    lim = teto * 1.1
    ax.plot([0, lim], [0, lim], color=FRACO, lw=1, ls="--")
    ax.fill_between([0, lim], [0, lim], [lim, lim], color="#e76f51", alpha=0.07)
    ax.text(lim * 0.52, lim * 0.9, "pegou DEPOIS de a bola parar",
            color="#e76f51", fontsize=8.5)
    ax.text(lim * 0.46, lim * 0.12, "pegou com a bola\nainda andando",
            color=D, fontsize=8.5)
    ax.set_xlim(0, lim); ax.set_ylim(0, lim)
    estilo(ax, "Corrida contra o atrito",
           "quando a bola para sozinha (s)", "quando ele pegou a bola (s)")
    ax.legend(facecolor=FUNDO, edgecolor="#2a3140", labelcolor=TINTA, fontsize=8)
    salvar(fig, saida, "03_corrida_contra_atrito.png")


# ----------------------------------------------------------------- 4
def g_primeiro_toque(grupos, saida):
    """Onde a bola encosta no robo. Fora da janela, o grSim nao dispara."""
    fig, ax = plt.subplots(figsize=(6.4, 4.4))
    ax.axhspan(-PLACA, PLACA, color=D, alpha=0.12)
    ax.axhline(PLACA, color=D, lw=1, ls="--")
    ax.axhline(-PLACA, color=D, lw=1, ls="--")
    ax.text(2, PLACA + 7, "janela do chutador do grSim (+-40 mm)",
            color=D, fontsize=8.5)
    for k, (nome, linhas) in enumerate(grupos.items()):
        xs = [num(l, "toque_xx") for l in linhas if num(l, "toque_xx") is not None]
        ys = [num(l, "toque_yy") for l in linhas if num(l, "toque_xx") is not None]
        dentro = sum(1 for y in ys if abs(y) <= PLACA)
        ax.scatter(xs, ys, s=38, color=(A if k == 0 else D), alpha=0.85,
                   edgecolors="none", label=f"{nome}: {dentro}/{len(ys)} na janela")
    estilo(ax, "Onde a bola encostou no robo, no primeiro toque",
           "a frente da placa (mm)", "para o lado do centro do chutador (mm)")
    ax.legend(facecolor=FUNDO, edgecolor="#2a3140", labelcolor=TINTA, fontsize=8)
    salvar(fig, saida, "04_primeiro_toque.png")


# ----------------------------------------------------------------- 5
def g_pareado(grupos, saida):
    """Cada colocacao, antes -> depois. So faz sentido com o par."""
    if len(grupos) < 2:
        return
    (na, la), (nd, ld) = list(grupos.items())[:2]
    ia = {l["tentativa"]: l for l in la}
    idd = {l["tentativa"]: l for l in ld}
    ks = sorted(set(ia) & set(idd), key=int)
    if not ks:
        return
    ordem = ["nao_alcancou", "sem_chute", "gol_empurrado", "fora", "curto",
             "gol_contra", "gol"]
    pos = {r: i for i, r in enumerate(ordem)}
    fig, ax = plt.subplots(figsize=(7.4, max(3.2, 0.32 * len(ks) + 1.4)))
    melhor = pior = igual = 0
    for linha, k in enumerate(ks):
        ya, yd = pos.get(ia[k]["resultado"], 0), pos.get(idd[k]["resultado"], 0)
        cor = D if yd > ya else (A if yd < ya else FRACO)
        melhor += yd > ya; pior += yd < ya; igual += yd == ya
        ax.annotate("", xy=(yd, linha), xytext=(ya, linha),
                    arrowprops=dict(arrowstyle="-|>", color=cor, lw=1.6,
                                    shrinkA=3, shrinkB=3, alpha=0.9))
        ax.scatter([ya], [linha], s=26, color=FRACO, zorder=3)
        ax.scatter([yd], [linha], s=34, color=cor, zorder=3)
    ax.set_yticks(range(len(ks)))
    ax.set_yticklabels([f"#{k}" for k in ks], fontsize=7)
    ax.set_xticks(range(len(ordem)))
    ax.set_xticklabels(ordem, rotation=25, ha="right", fontsize=8)
    estilo(ax, f"Mesma colocacao, {na} -> {nd}   "
               f"(melhora {melhor}, piora {pior}, empata {igual})",
           "resultado  (pior a esquerda)", "")
    salvar(fig, saida, "05_pareado.png")


# ----------------------------------------------------------------- 6
def g_tempo_ate_posse(grupos, saida):
    """Quanto ele demora para TER a bola, contra a distancia de onde largou."""
    fig, ax = plt.subplots(figsize=(7.0, 4.2))
    dmax = 100.0
    for k, (nome, linhas) in enumerate(grupos.items()):
        xs = [num(l, "dist_robo_bola") for l in linhas if num(l, "t_toque") is not None]
        ys = [num(l, "t_toque") for l in linhas if num(l, "t_toque") is not None]
        if xs:
            dmax = max(dmax, max(xs))
        vel = [x / y for x, y in zip(xs, ys) if y > 0]
        vmed = sorted(vel)[len(vel) // 2] if vel else float("nan")
        ax.scatter(xs, ys, s=38, color=(A if k == 0 else D), alpha=0.85,
                   edgecolors="none", label=f"{nome}: {vmed:.0f} mm/s efetivos")
    d = [0, dmax * 1.05]
    for v, rot, cor in ((539.0, "539 mm/s (ate ENCOSTAR)", FRACO),
                        (161.0, "161 mm/s (ate TER, medido 28/10)", "#e9c46a")):
        ax.plot(d, [x / v for x in d], ls="--", lw=1, color=cor, label=rot)
    estilo(ax, "Tempo ate TER a bola, por distancia de largada",
           "distancia robo-bola na largada (mm)", "t_toque (s)")
    ax.legend(facecolor=FUNDO, edgecolor="#2a3140", labelcolor=TINTA, fontsize=8)
    salvar(fig, saida, "06_tempo_ate_posse.png")


# ----------------------------------------------------------------- 7
def g_placar(grupos, saida):
    """O placar, com gol de chute separado de gol empurrado."""
    ordem = ["gol", "gol_empurrado", "curto", "fora", "sem_chute",
             "nao_alcancou", "gol_contra"]
    cores = {"gol": D, "gol_empurrado": "#7b6d8d", "curto": "#e9c46a",
             "fora": A, "sem_chute": FRACO, "nao_alcancou": "#4a4e69",
             "gol_contra": "#6a040f"}
    fig, ax = plt.subplots(figsize=(7.4, 1.0 + 0.9 * len(grupos)))
    for k, (nome, linhas) in enumerate(grupos.items()):
        esq = 0
        for r in ordem:
            n = sum(1 for l in linhas if l["resultado"] == r)
            if not n:
                continue
            ax.barh(k, n, left=esq, color=cores[r], edgecolor=FUNDO, height=0.62)
            ax.text(esq + n / 2, k, f"{r}\n{n}", ha="center", va="center",
                    fontsize=7.5, color="#0d1117" if r == "gol" else TINTA)
            esq += n
    ax.set_yticks(range(len(grupos)))
    ax.set_yticklabels(list(grupos), fontsize=9, color=TINTA)
    estilo(ax, "Placar  (gol de CHUTE separado de gol empurrado)",
           "tentativas", "")
    salvar(fig, saida, "00_placar.png")


def wilson(k, n):
    """Intervalo de Wilson a 95%. Com n=20 a barra de erro e O NUMERO."""
    if n == 0:
        return 0.0, 0.0, 0.0
    z, p = 1.96, k / n
    d = 1.0 + z * z / n
    centro = (p + z * z / (2.0 * n)) / d
    meia = z * math.sqrt(p * (1.0 - p) / n + z * z / (4.0 * n * n)) / d
    return p, max(0.0, centro - meia), min(1.0, centro + meia)


def _placar(linhas):
    """Os numeros que importam num lote, como fracoes de tentativa."""
    n = len(linhas)
    r = [l.get("resultado", "") for l in linhas]
    return dict(
        n=n,
        gol=sum(1 for x in r if x == "gol"),
        chutou=sum(1 for l in linhas if l.get("chutou") == "1"),
        sem_chute=sum(1 for x in r if x == "sem_chute"),
        fora=sum(1 for x in r if x == "fora"),
        perdido=sum(1 for x in r if x in ("nao_alcancou", "gol_contra")),
        armado=sum(1 for l in linhas if l.get("toque_armado") == "1"),
    )


def g_evolucao(etapas, saida):
    """A EVOLUCAO: taxa de gol por etapa, com a barra de erro desenhada.

    POR QUE A BARRA E O GRAFICO, e nao um enfeite: 20 de 20 e 38 de 40 nao sao
    comparaveis a olho. Com n=20 o intervalo de 95% de 90% vai de 70 a 97 - um
    lote de 20 nao distingue 80% de 95%. Sem a barra, esta figura so contaria
    uma historia bonita.
    """
    fig, (ax, ax2) = plt.subplots(2, 1, figsize=(max(7.0, 1.6 * len(etapas)), 7.4),
                                  gridspec_kw=dict(height_ratios=[1.25, 1.0]))
    xs = range(len(etapas))
    ps, los, his, ns = [], [], [], []
    for _, linhas in etapas:
        e = _placar(linhas)
        p, lo, hi = wilson(e["gol"], e["n"])
        ps.append(p * 100); los.append((p - lo) * 100); his.append((hi - p) * 100)
        ns.append(e["n"])
    ax.errorbar(list(xs), ps, yerr=[los, his], fmt="o-", color=D, ecolor=FRACO,
                elinewidth=1.4, capsize=5, markersize=7, lw=1.8)
    for x, p, n in zip(xs, ps, ns):
        ax.annotate(f"{p:.0f}%\nn={n}", (x, p), textcoords="offset points",
                    xytext=(0, 13), ha="center", fontsize=8, color=TINTA)
    ax.set_xticks(list(xs))
    ax.set_xticklabels([r for r, _ in etapas], fontsize=8, color=TINTA,
                       rotation=18, ha="right")
    ax.set_ylim(0, 108)
    estilo(ax, "Gol por tentativa, por etapa  (barra = 95% de Wilson)",
           "", "% das tentativas")

    # O QUE SOBROU: as falhas empilhadas, em numero absoluto.
    chaves = [("sem_chute", "#e9c46a"), ("fora", "#e76f51"), ("perdido", "#9d4edd")]
    base = [0.0] * len(etapas)
    for nome, cor in chaves:
        vals = [_placar(l)[nome] for _, l in etapas]
        ax2.bar(list(xs), vals, bottom=base, color=cor, edgecolor=FUNDO,
                width=0.6, label=nome)
        base = [b + v for b, v in zip(base, vals)]
    for x, t in zip(xs, base):
        if t:
            ax2.text(x, t + 0.15, f"{t:.0f}", ha="center", fontsize=8, color=TINTA)
    ax2.set_xticks(list(xs))
    ax2.set_xticklabels([r for r, _ in etapas], fontsize=8, color=TINTA,
                        rotation=18, ha="right")
    estilo(ax2, "O que sobrou de falha  (absoluto; 'perdido' = nao alcancou + gol contra)",
           "", "tentativas")
    ax2.legend(facecolor=FUNDO, edgecolor="#2a3140", labelcolor=TINTA, fontsize=8)
    salvar(fig, saida, "10_evolucao.png")


def g_funil(grupos, saida):
    """O FUNIL do estado atual: alcancou -> encostou -> armou -> chutou -> gol.

    Cada degrau e uma coisa diferente de consertar, e e por isso que o placar
    sozinho nao serve: 'sem_chute' pode ser nao chegar na bola, chegar e nao
    alinhar, ou alinhar e nao disparar. O degrau que cai e o defeito.
    """
    fig, ax = plt.subplots(figsize=(8.2, 4.4))
    passos = [("alcancou", lambda l: l.get("alcancou") == "1"),
              ("encostou", lambda l: l.get("encostou") == "1"),
              ("armado no toque", lambda l: l.get("toque_armado") == "1"),
              ("chutou", lambda l: l.get("chutou") == "1"),
              ("gol", lambda l: l.get("resultado") == "gol")]
    larg = 0.8 / max(1, len(grupos))
    for k, (nome, linhas) in enumerate(grupos.items()):
        n = len(linhas) or 1
        vals = [100.0 * sum(1 for l in linhas if f(l)) / n for _, f in passos]
        xs = [i + k * larg - 0.4 + larg / 2 for i in range(len(passos))]
        cor = D if (len(grupos) == 1 or nome == "depois") else A
        ax.bar(xs, vals, width=larg * 0.9, color=cor, edgecolor=FUNDO,
               label=f"{nome} (n={len(linhas)})")
        for x, v in zip(xs, vals):
            ax.text(x, v + 1.2, f"{v:.0f}", ha="center", fontsize=8, color=TINTA)
    ax.set_xticks(range(len(passos)))
    ax.set_xticklabels([p for p, _ in passos], fontsize=9, color=TINTA)
    ax.set_ylim(0, 112)
    estilo(ax, "Funil: onde a tentativa morre", "", "% das tentativas")
    ax.legend(facecolor=FUNDO, edgecolor="#2a3140", labelcolor=TINTA, fontsize=8)
    salvar(fig, saida, "11_funil.png")


def g_por_seed(grupos, saida):
    """Taxa de gol POR SEED, no lote de varias seeds.

    POR QUE ESTE GRAFICO EXISTE: uma seed nao e amostra do campo, e um conjunto
    fixo de colocacoes. 20 de 20 na seed 11 e 38 de 40 na seed 12 (03/11/2026)
    sao as duas verdade e nenhuma e a taxa. Se as barras aqui nao se sobrepoem,
    o que o lote mede e a seed, nao o atacante.
    """
    linhas = [l for g in grupos.values() for l in g]
    seeds = sorted({l.get("seed", "") for l in linhas})
    if len(seeds) < 2:
        return
    fig, ax = plt.subplots(figsize=(max(6.0, 1.3 * len(seeds)), 4.2))
    ps, los, his = [], [], []
    for sd in seeds:
        sub = [l for l in linhas if l.get("seed") == sd]
        p, lo, hi = wilson(sum(1 for l in sub if l.get("resultado") == "gol"), len(sub))
        ps.append(p * 100); los.append((p - lo) * 100); his.append((hi - p) * 100)
    todos = [l for l in linhas]
    pt, lot, hit = wilson(sum(1 for l in todos if l.get("resultado") == "gol"), len(todos))
    ax.axhspan(lot * 100, hit * 100, color=D, alpha=0.16)
    ax.axhline(pt * 100, color=D, lw=1.6,
               label=f"lote todo {pt*100:.1f}%  (n={len(todos)})")
    ax.errorbar(range(len(seeds)), ps, yerr=[los, his], fmt="o", color=TINTA,
                ecolor=FRACO, elinewidth=1.3, capsize=5, markersize=6)
    ax.set_xticks(range(len(seeds)))
    ax.set_xticklabels([f"seed {x}" for x in seeds], fontsize=8, color=TINTA)
    ax.set_ylim(0, 108)
    estilo(ax, "Gol por tentativa, por seed  (a faixa e o lote inteiro)",
           "", "% das tentativas")
    ax.legend(facecolor=FUNDO, edgecolor="#2a3140", labelcolor=TINTA, fontsize=8)
    salvar(fig, saida, "12_por_seed.png")


def g_precisao(grupos, saida):
    """A MARGEM QUE SOBRA: erro na linha do gol contra a folga ate a trave.

    Nao basta o erro ser pequeno: ele tem de ser menor que o que a mira pedida
    deixa de gol. Mirando no meio sobram 500 mm; mirando em +350 sobram 150.
    MEDIDO (t15 do lote de 40 da seed 12): erro de 180 mm com mira +350 - erro
    pequeno, bola fora por 30 mm.
    """
    fig, ax = plt.subplots(figsize=(7.6, 4.6))
    for nome, linhas in grupos.items():
        xs, ys, cs = [], [], []
        for l in linhas:
            m, e = num(l, "mira_y"), num(l, "erro_y")
            if m is None or e is None:
                continue
            xs.append(500.0 - abs(m)); ys.append(abs(e))
            cs.append(D if l.get("resultado") == "gol" else A)
        if not xs:
            continue
        ax.scatter(xs, ys, c=cs, s=30, edgecolor=FUNDO, lw=0.5, zorder=3)
    lim = 560
    ax.plot([0, lim], [0, lim], color=FRACO, lw=1.3, ls="--", zorder=2)
    ax.fill_between([0, lim], [0, lim], [lim, lim], color=A, alpha=0.10, zorder=1)
    ax.text(lim * 0.52, lim * 0.82, "erro MAIOR que a folga\n-> bola fora",
            color=A, fontsize=9, ha="center")
    ax.set_xlim(0, lim); ax.set_ylim(0, lim)
    estilo(ax, "Erro na linha do gol vs folga ate a trave  (verde = gol)",
           "folga que a mira deixa (mm)", "|erro na linha| (mm)")
    salvar(fig, saida, "13_precisao_vs_folga.png")


def main():
    if len(sys.argv) > 1 and sys.argv[1] == "--evolucao":
        if len(sys.argv) < 4:
            print("uso: --evolucao PASTA_SAIDA rotulo=pasta rotulo=pasta ...")
            return 1
        destino, etapas = sys.argv[2], []
        for arg in sys.argv[3:]:
            rotulo, _, pasta = arg.partition("=")
            c = os.path.join(pasta or rotulo, "tentativas.csv")
            if not os.path.exists(c):
                print("sem tentativas.csv em", pasta or rotulo)
                return 1
            etapas.append((rotulo, ler(c)))
        saida = os.path.join(destino, "graficos")
        os.makedirs(saida, exist_ok=True)
        print("gravando em", saida)
        g_evolucao(etapas, saida)
        return 0

    pasta = sys.argv[1] if len(sys.argv) > 1 else "campanha_final"
    grupos = {}
    # Formato do par INTERCALADO (--par CHAVE): um CSV so, com a coluna 'cond'.
    unico = os.path.join(pasta, "tentativas.csv")
    if os.path.exists(unico):
        linhas = ler(unico)
        conds = [c for c in ("antes", "depois")
                 if any(l.get("cond") == c for l in linhas)]
        if conds:
            for c in conds:
                grupos[c] = [l for l in linhas if l.get("cond") == c]
        else:
            grupos["campanha"] = linhas
    # Formato antigo: uma subpasta por condicao.
    for nome in ("antes", "depois"):
        c = os.path.join(pasta, nome, "tentativas.csv")
        if not grupos and os.path.exists(c):
            grupos[nome] = ler(c)
    if not grupos:
        print("sem tentativas.csv em", pasta)
        return 1
    saida = os.path.join(pasta, "graficos")
    os.makedirs(saida, exist_ok=True)
    print("gravando em", saida)
    g_placar(grupos, saida)
    g_velocidade_no_toque(grupos, saida)
    g_investidas(grupos, saida)
    g_corrida_contra_atrito(grupos, saida)
    g_primeiro_toque(grupos, saida)
    g_pareado(grupos, saida)
    g_tempo_ate_posse(grupos, saida)
    g_funil(grupos, saida)
    g_por_seed(grupos, saida)
    g_precisao(grupos, saida)
    return 0


if __name__ == "__main__":
    sys.exit(main())
