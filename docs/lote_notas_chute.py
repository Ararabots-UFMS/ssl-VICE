#!/usr/bin/env python3
"""Executa repeticoes dos cenarios de nota de chute e gera CSV/PNG."""

import csv
import datetime as dt
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
VALIDAR = ROOT / "docs" / "ararabots.sh"
SCENARIOS = ("nota_chute_passe", "nota_chute_gol", "nota_chute_gol_dinamico")


def read_replay(path):
    text = Path(path).read_text(encoding="utf-8")
    i = text.index("const D = ") + len("const D = ")
    j = text.index(";\n", i)
    return json.loads(text[i:j])


def replay_shot_metrics(data, diagnostic_text):
    """Cruza evento do chute, trajetória da bola e posição real do goleiro."""
    empty = {
        "t_chute_s": None, "alvo_chute_y": None, "goleiro_y_no_chute": None,
        "goleiro_y_previsto": None, "t_chegada_previsto_s": None,
        "bola_y_linha_gol": None, "goleiro_y_linha_gol": None,
        "dist_goleiro_min_mm": None, "resultado_chute": "sem_disparo",
    }
    shots = [e for e in data.get("eventos", []) if e.get("flat_kick") or e.get("chip_kick")]
    if not shots:
        return empty
    t_kick = min(float(e["t"]) for e in shots)
    result = dict(empty)
    result["t_chute_s"] = round(t_kick, 3)
    aim = re.search(r"\[JG\] CHUTE_ARMADO .*?tipo=gol .*?alvo=[-+\d.]+,([-+\d.]+)", diagnostic_text)
    if aim:
        result["alvo_chute_y"] = float(aim.group(1))

    frames = data.get("quadros", [])
    before = [f for f in frames if f["t"] <= t_kick]
    after = [f for f in frames if f["t"] >= t_kick]
    if not before or not after:
        result["resultado_chute"] = "sem_trajetoria"
        return result
    f0 = before[-1]
    def keeper_y(frame):
        return next((float(r[2]) for r in frame.get("y", []) if r[0] == 0), None)
    gk_y0 = keeper_y(f0)
    result["goleiro_y_no_chute"] = round(gk_y0) if gk_y0 is not None else None

    # Predição linear simples com o movimento observado nos 250 ms anteriores.
    hist = []
    for f in before:
        if t_kick - 0.25 <= f["t"] <= t_kick:
            y = keeper_y(f)
            if y is not None:
                hist.append((float(f["t"]), y))
    vy = 0.0
    if len(hist) >= 2 and gk_y0 is not None:
        mt = sum(t for t, _ in hist) / len(hist)
        my = sum(y for _, y in hist) / len(hist)
        den = sum((t - mt) ** 2 for t, _ in hist)
        if den:
            vy = sum((t - mt) * (y - my) for t, y in hist) / den
    x0, y0 = map(float, f0["b"][:2])
    later = next((f for f in after if f["t"] > t_kick and f["b"][0] > x0 + 10), None)
    if later and gk_y0 is not None:
        vx = (float(later["b"][0]) - x0) / (float(later["t"]) - t_kick)
        if vx > 0:
            t_arrival = max(0.0, (4500.0 - x0) / vx)
            result["t_chegada_previsto_s"] = round(t_arrival, 3)
            result["goleiro_y_previsto"] = round(max(-440.0, min(440.0, gk_y0 + vy * t_arrival)))

    # Proximidade mínima entre a bola e o goleiro depois do disparo.
    min_dist = None
    for f in after:
        gy = keeper_y(f)
        if gy is None:
            continue
        yr = next((r for r in f.get("y", []) if r[0] == 0), None)
        if yr is not None:
            dist = ((float(f["b"][0]) - float(yr[1])) ** 2
                    + (float(f["b"][1]) - float(yr[2])) ** 2) ** 0.5
            min_dist = dist if min_dist is None else min(min_dist, dist)
    if min_dist is not None:
        result["dist_goleiro_min_mm"] = round(min_dist)

    sent = 1.0 if float(data.get("sentido_ataque") or 1.0) >= 0 else -1.0
    crossing = None
    for a, b in zip(after, after[1:]):
        xa, xb = float(a["b"][0]) * sent, float(b["b"][0]) * sent
        if xa < 4500.0 <= xb and xb != xa:
            q = (4500.0 - xa) / (xb - xa)
            crossing = (a, b, q)
            break
    if crossing:
        a, b, q = crossing
        y_cross = float(a["b"][1]) + q * (float(b["b"][1]) - float(a["b"][1]))
        result["bola_y_linha_gol"] = round(y_cross)
        gy0, gy1 = keeper_y(a), keeper_y(b)
        if gy0 is not None and gy1 is not None:
            result["goleiro_y_linha_gol"] = round(gy0 + q * (gy1 - gy0))

    if data.get("gol") == "nosso":
        classification = "gol"
    elif min_dist is not None and min_dist <= 115:
        classification = "defesa_goleiro_provavel"
    elif result["bola_y_linha_gol"] is not None and abs(result["bola_y_linha_gol"]) > 500:
        classification = "fora_da_boca"
    elif crossing:
        classification = "cruzou_sem_gol"
    else:
        classification = "parou_antes_da_linha"
    result["resultado_chute"] = classification
    return result


def extract_diagnostics(text):
    notes = re.findall(r"\[JG\] NOTA_CHUTE .*?nota=([-+\d.]+).*?limiar_direto=([-+\d.]+).*?limiar_baixa=([-+\d.]+)", text)
    targets = re.findall(r"\[JG\] alvo_chute=(\w+)", text)
    pass_targets = re.findall(r"\[JG\] PASSE_ALVO_FIXO rid=(\d+).*?avanco=([-+\d.]+)", text)
    invalid = re.findall(r"\[JG\] PASSE_INVALIDADO .*?causa=([\w_]+)", text)
    note = float(notes[0][0]) if notes else None
    threshold = float(notes[0][1]) if notes else None
    target = targets[0] if targets else None
    low_threshold = float(notes[0][2]) if notes else None
    if target is None and note is not None:
        # A primeira seleção acontece em _escolher_alvo_chute: nota acima do
        # limiar direto escolhe gol; abaixo dele tenta primeiro um passe viável.
        if note > threshold:
            target = "gol"
        elif pass_targets:
            target = "passe"
        elif low_threshold is not None and note > low_threshold:
            target = "gol"
        else:
            target = "bloqueado"
    return {
        "nota_primeira": note,
        "limiar_direto": threshold,
        "alvo_primeiro": target,
        "receptor_passe": int(pass_targets[0][0]) if pass_targets else None,
        "avanco_passe_mm": float(pass_targets[0][1]) if pass_targets else None,
        "passe_recebido": "PASSE_RECEBIDO" in text,
        "passe_invalidado": bool(invalid),
        "causa_invalidacao": invalid[0] if invalid else None,
        "qtd_notas": len(notes),
    }


def extract_note_timeline(text, scenario, repetition):
    """Extrai a nota a cada ciclo, com tempo relativo à primeira avaliação."""
    pattern = re.compile(
        r"\[JG\] NOTA_CHUTE t_mono=([-+\d.]+) bola=([-+\d.]+),([-+\d.]+) "
        r"nota=([-+\d.]+).*?amostras=(\d+) inimigos=(\d+)"
    )
    raw = [(float(t), float(bx), float(by), float(note), int(samples), int(enemies))
           for t, bx, by, note, samples, enemies in pattern.findall(text)]
    if not raw:
        return []
    t0 = raw[0][0]
    return [{"cenario": scenario, "repeticao": repetition,
             "t_s": round(t - t0, 4), "nota": note,
             "bola_x_mm": bx, "bola_y_mm": by,
             "amostras": samples, "inimigos": enemies}
            for t, bx, by, note, samples, enemies in raw]


def write_note_timeline(out, scenarios):
    rows = []
    for scenario in scenarios:
        for path in sorted((out / "diagnosticos").glob(f"{scenario}_*.log")):
            match = re.search(r"_(\d+)\.log$", path.name)
            if not match:
                continue
            rows.extend(extract_note_timeline(
                path.read_text(encoding="utf-8", errors="replace"),
                scenario, int(match.group(1))))
    path = out / "notas_tempo.csv"
    fields = ["cenario", "repeticao", "t_s", "nota", "bola_x_mm",
              "bola_y_mm", "amostras", "inimigos"]
    with path.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    return rows


def plot_note_timeline(note_rows, out, scenario):
    """Plota mediana temporal e intervalo interquartil entre as execuções."""
    selected = [r for r in note_rows if r["cenario"] == scenario]
    if not selected:
        return
    os.environ.setdefault("MPLCONFIGDIR", str(out / ".mplconfig"))
    import matplotlib.pyplot as plt

    bins = {}
    per_run = {}
    for row in selected:
        bucket = round(float(row["t_s"]) / 0.1) * 0.1
        per_run.setdefault((row["repeticao"], bucket), []).append(float(row["nota"]))
    for (rep, bucket), values in per_run.items():
        bins.setdefault(bucket, []).append((rep, sum(values) / len(values)))
    xs = sorted(bins)
    medians, lower, upper = [], [], []
    for x in xs:
        vals = sorted(v for _, v in bins[x])
        n = len(vals)
        medians.append(vals[n // 2] if n % 2 else (vals[n // 2 - 1] + vals[n // 2]) / 2)
        lower.append(vals[int((n - 1) * 0.25)])
        upper.append(vals[int((n - 1) * 0.75)])
    fig, ax = plt.subplots(figsize=(10, 5.8))
    ax.plot(xs, medians, color="#6554c0", label="Mediana entre rodadas")
    ax.fill_between(xs, lower, upper, color="#6554c0", alpha=.2,
                    label="Faixa interquartil (25–75%)")
    ax.axhline(.3, color="#bd3b3b", linestyle="--",
               label="Limiar de chute direto (0,3)")
    ax.set_ylim(0, 1.05)
    ax.set_xlabel("Segundos desde a primeira avaliação da nota")
    ax.set_ylabel("Nota do melhor alvo de gol")
    ax.set_title("Evolução temporal da nota — goleiro dinâmico")
    ax.grid(alpha=.25)
    ax.legend()
    fig.tight_layout()
    fig.savefig(out / "evolucao_notas.png", dpi=170)
    plt.close(fig)


def parse_output(text):
    m = re.search(r"REPLAY: (.+?\.html)", text)
    return Path(m.group(1).strip()) if m else None


def wilson(success, total):
    if not total:
        return (0.0, 0.0)
    z = 1.96
    p = success / total
    den = 1 + z * z / total
    center = (p + z * z / (2 * total)) / den
    half = z * ((p * (1 - p) / total + z * z / (4 * total * total)) ** 0.5) / den
    return max(0, center - half), min(1, center + half)


def plot(rows, out, scenarios):
    os.environ.setdefault("MPLCONFIGDIR", str(out / ".mplconfig"))
    import matplotlib.pyplot as plt

    scenarios = list(scenarios)
    names = [s.replace("nota_chute_", "") for s in scenarios]
    valid = {s: [r for r in rows if r["cenario"] == s and r["status"] == "OK"] for s in scenarios}
    # Passe é sucesso quando o receptor confirma; cenários de gol quando chuta.
    successes = {
        s: sum((r["alvo_primeiro"] == "passe" and r["passe_recebido"])
               if s == "nota_chute_passe"
               else (r["alvo_primeiro"] == "gol" and r["disparou"])
               for r in valid[s])
        for s in scenarios
    }
    fig, ax = plt.subplots(figsize=(10, 5.8))
    rates = []
    errors = [[], []]
    totals = []
    for s in scenarios:
        n, k = len(valid[s]), successes[s]
        totals.append(n)
        rates.append(k / n if n else 0)
        lo, hi = wilson(k, n)
        errors[0].append((k / n - lo) if n else 0)
        errors[1].append((hi - k / n) if n else 0)
    palette = {"nota_chute_passe": "#e89a38", "nota_chute_gol": "#3489c8", "nota_chute_gol_dinamico": "#6554c0"}
    bars = ax.bar(names, rates, color=[palette.get(s, "#3489c8") for s in scenarios], yerr=errors, capsize=5)
    ax.set_ylim(0, 1.12)
    ax.set_ylabel("Taxa de execução da ação esperada")
    ax.set_title("Nota de chute: ação esperada por cenário (IC 95% Wilson)")
    ax.grid(axis="y", alpha=.25)
    for bar, k, n in zip(bars, successes.values(), totals):
        ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + .045, f"{k}/{n}", ha="center", va="bottom")
    ax.text(.01, -.19, "Passe: alvo passe e receptor confirmou. Gol: alvo gol e houve disparo. ICs descrevem a incerteza da taxa.", transform=ax.transAxes, fontsize=8)
    fig.tight_layout()
    fig.savefig(out / "taxa_acao_esperada.png", dpi=170)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(10, 5.8))
    datasets = [[r["nota_primeira"] for r in valid[s] if r["nota_primeira"] is not None] for s in scenarios]
    ax.boxplot(datasets, tick_labels=names, showmeans=True)
    ax.axhline(.3, color="#bd3b3b", linestyle="--", label="limiar de chute direto (0,3)")
    ax.set_ylabel("Nota do melhor alvo de gol na primeira avaliação")
    ax.set_title("Distribuição da nota de chute por cenário")
    ax.grid(axis="y", alpha=.25)
    ax.legend()
    fig.tight_layout()
    fig.savefig(out / "distribuicao_notas.png", dpi=170)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(10, 5.8))
    fields = (("disparou", "Disparos"), ("gol_a_favor", "Gols a favor"), ("passe_recebido", "Passes recebidos"))
    x = range(len(scenarios))
    width = .24
    for j, (field, label) in enumerate(fields):
        counts = []
        for s in scenarios:
            rs = valid[s]
            counts.append(sum((r[field] is True) if field != "gol_a_favor" else (r[field] in ("nosso", "GOL A FAVOR")) for r in rs) / len(rs) if rs else 0)
        ax.bar([a + (j-1)*width for a in x], counts, width, label=label)
    ax.set_xticks(list(x), names)
    ax.set_ylim(0, 1.05)
    ax.set_ylabel("Taxa entre execuções válidas")
    ax.set_title("Resultados observados (separados da decisão inicial)")
    ax.grid(axis="y", alpha=.25)
    ax.legend()
    fig.tight_layout()
    fig.savefig(out / "resultados_observados.png", dpi=170)
    plt.close(fig)

    outcomes = ["gol", "defesa_goleiro_provavel", "fora_da_boca",
                "cruzou_sem_gol", "parou_antes_da_linha", "sem_disparo"]
    labels = ["Gol", "Defesa provável", "Fora da boca", "Cruzou sem gol",
              "Parou antes", "Sem disparo"]
    colors = ["#2ca02c", "#d95f02", "#9467bd", "#8c564b", "#7f7f7f", "#1f77b4"]
    fig, ax = plt.subplots(figsize=(11, 6))
    bottom = [0.0] * len(scenarios)
    for outcome, label, color in zip(outcomes, labels, colors):
        values = []
        for s in scenarios:
            rs = valid[s]
            n = len(rs)
            values.append(sum(r.get("resultado_chute") == outcome for r in rs) / n if n else 0)
        ax.bar(names, values, bottom=bottom, label=label, color=color)
        bottom = [a + b for a, b in zip(bottom, values)]
    ax.set_ylim(0, 1)
    ax.set_ylabel("Proporção das execuções válidas")
    ax.set_title("O que aconteceu depois do primeiro chute")
    ax.grid(axis="y", alpha=.25)
    ax.legend(loc="upper left", bbox_to_anchor=(1.01, 1.0))
    fig.tight_layout()
    fig.savefig(out / "classificacao_chutes.png", dpi=170, bbox_inches="tight")
    plt.close(fig)


def main():
    reps = int(sys.argv[1]) if len(sys.argv) > 1 else 10
    if reps < 1:
        raise SystemExit("repetições deve ser >= 1")
    args = sys.argv[2:]
    resume = "--resume" in args
    if resume:
        args.remove("--resume")
        if not args:
            raise SystemExit("uso: lote_notas_chute.py N [cenários...] --resume <pasta-do-lote>")
        resume_dir = args.pop()
        out = Path(resume_dir).expanduser().resolve()
    else:
        out = None
    scenarios = args or list(SCENARIOS)
    unknown = [s for s in scenarios if s not in SCENARIOS]
    if unknown:
        raise SystemExit(f"cenário(s) desconhecido(s): {', '.join(unknown)}\nOpções: {', '.join(SCENARIOS)}")
    if resume:
        if not (out / "metricas.csv").is_file():
            raise SystemExit(f"não encontrei metricas.csv em {out}")
    else:
        stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        out = ROOT / "docs" / "resultados_nota_chute" / stamp
        out.mkdir(parents=True)
    (out / "diagnosticos").mkdir(exist_ok=True)
    (out / "replays").mkdir(exist_ok=True)
    rows = []
    csv_path = out / "metricas.csv"
    fields = ["cenario", "repeticao", "status", "motivo", "replay", "nota_primeira", "limiar_direto", "alvo_primeiro", "receptor_passe", "avanco_passe_mm", "passe_recebido", "passe_invalidado", "causa_invalidacao", "qtd_notas", "disparou", "gol_a_favor", "duracao_s"]
    fields += ["t_chute_s", "alvo_chute_y", "goleiro_y_no_chute", "goleiro_y_previsto", "t_chegada_previsto_s", "bola_y_linha_gol", "goleiro_y_linha_gol", "dist_goleiro_min_mm", "resultado_chute"]
    completed = set()
    if resume:
        with csv_path.open(encoding="utf-8", newline="") as old:
            for saved in csv.DictReader(old):
                if saved.get("status") != "OK":
                    continue
                saved["repeticao"] = int(saved["repeticao"])
                diag_path = out / "diagnosticos" / f"{saved['cenario']}_{saved['repeticao']:02d}.log"
                if diag_path.is_file():
                    diagnostic_text = diag_path.read_text(encoding="utf-8")
                    saved.update(extract_diagnostics(diagnostic_text))
                    replay_path = Path(saved.get("replay") or "")
                    if replay_path.is_file():
                        saved.update(replay_shot_metrics(read_replay(replay_path), diagnostic_text))
                for key in ("nota_primeira", "limiar_direto", "avanco_passe_mm"):
                    value = saved.get(key)
                    saved[key] = float(value) if value not in (None, "") else None
                for key in ("receptor_passe", "qtd_notas", "duracao_s"):
                    value = saved.get(key)
                    saved[key] = int(float(value)) if value not in (None, "") else None
                for key in ("passe_recebido", "passe_invalidado", "disparou"):
                    value = saved.get(key, "")
                    saved[key] = value if isinstance(value, bool) else str(value).lower() == "true"
                rows.append(saved)
                completed.add((saved["cenario"], saved["repeticao"]))
        # A planilha retoma só execuções válidas; tentativas bloqueadas ficam
        # registradas nos logs, sem duplicar número de repetição no CSV final.
        with csv_path.open("w", newline="", encoding="utf-8") as clean:
            writer = csv.DictWriter(clean, fieldnames=fields)
            writer.writeheader()
            writer.writerows({key: row.get(key) for key in fields} for row in rows)
    print(f"Lote: {out}\nCenários: {', '.join(scenarios)}\n{reps} execuções por cenário ({reps * len(scenarios)} no total).", flush=True)
    with csv_path.open("a" if resume else "w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=fields)
        if not resume:
            writer.writeheader()
            fh.flush()
        for scenario in scenarios:
            for rep in range(1, reps + 1):
                if (scenario, rep) in completed:
                    continue
                done_selected = sum(r["cenario"] in scenarios and r["repeticao"] <= reps for r in rows)
                print(f"[{done_selected+1}/{reps*len(scenarios)}] {scenario} rodada {rep}/{reps}", flush=True)
                zombies = subprocess.run(["docker", "exec", "vice", "sh", "-lc", "ps -eo stat | grep -c '^Z' || true"], text=True, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
                try:
                    zombie_count = int(zombies.stdout.strip() or "0")
                except ValueError:
                    zombie_count = 0
                if zombie_count >= 150:
                    print(f"    Reiniciando ambiente antes da rodada ({zombie_count} processos zumbi).", flush=True)
                    subprocess.run(["bash", str(VALIDAR), "parar"], cwd=ROOT, check=False)
                    prep_env = os.environ.copy()
                    prep_env.update({"DIAG_JOGO": "1", "ARARABOTS_ROBOS": "3"})
                    prep = subprocess.run(["bash", str(VALIDAR), "preparar", "--headless"], cwd=ROOT, env=prep_env, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
                    print(prep.stdout, end="", flush=True)
                    if prep.returncode:
                        raise SystemExit("não consegui reiniciar o ambiente; lote salvo para retomar")
                subprocess.run(["docker", "exec", "vice", "sh", "-lc", ": > /tmp/diag_jogo.log"], check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                env = os.environ.copy()
                env.update({"DIAG_JOGO": "1", "ARARABOTS_ROBOS": "3"})
                started = time.time()
                (out / "logs").mkdir(exist_ok=True)
                log_path = out / "logs" / f"{scenario}_{rep:02d}.txt"
                chunks = []
                with log_path.open("w", encoding="utf-8") as log:
                    proc = subprocess.Popen(["bash", str(VALIDAR), "validar", "1", scenario], cwd=ROOT, env=env, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, bufsize=1)
                    try:
                        for line in proc.stdout:
                            print("    " + line, end="", flush=True)
                            log.write(line)
                            chunks.append(line)
                        return_code = proc.wait(timeout=240)
                    except KeyboardInterrupt:
                        proc.terminate()
                        proc.wait(timeout=10)
                        raise
                output = "".join(chunks)
                replay = parse_output(output)
                diag = subprocess.run(["docker", "exec", "vice", "cat", "/tmp/diag_jogo.log"], text=True, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
                diag_file = out / "diagnosticos" / f"{scenario}_{rep:02d}.log"
                diag_file.write_text(diag.stdout, encoding="utf-8")
                row = {f: None for f in fields}
                row.update({"cenario": scenario, "repeticao": rep, "status": "ERRO", "motivo": f"validador_exit_{return_code}", "passe_recebido": False, "passe_invalidado": False, "qtd_notas": 0})
                if replay and replay.is_file():
                    replay_copy = out / "replays" / f"{scenario}_{rep:02d}.html"
                    shutil.copy2(replay, replay_copy)
                    try:
                        data = read_replay(replay_copy)
                        d = extract_diagnostics(diag.stdout)
                        shot_metrics = replay_shot_metrics(data, diag.stdout)
                        goal = data.get("gol")
                        row.update(d)
                        row.update(shot_metrics)
                        row.update({"status": "OK", "motivo": None, "replay": str(replay_copy), "disparou": bool(data.get("disparou")), "gol_a_favor": goal})
                    except Exception as exc:
                        row.update({"replay": str(replay_copy), "motivo": f"erro_leitura:{exc}"})
                else:
                    reason = re.search(r"(?:XX|BLOQUEADO|erro|GRSIM-CAIU)[^\n]*", output, re.I)
                    row["motivo"] = reason.group(0)[:300] if reason else row["motivo"]
                row["duracao_s"] = round(time.time() - started, 1)
                rows.append(row)
                writer.writerow({f: row.get(f) for f in fields})
                fh.flush()
                print(f"    status={row['status']} alvo={row['alvo_primeiro']} disparou={row['disparou']} passe_recebido={row['passe_recebido']} gol={row['gol_a_favor']} ({row['duracao_s']}s)", flush=True)
                # Se o ambiente parou, as rodadas restantes não são tentadas às cegas.
                if row["status"] != "OK":
                    print("Execução sem replay válido; encerrando o lote para não contar falha de infraestrutura como falha da estratégia.", flush=True)
                    break
            else:
                continue
            break
    if rows and any(r["status"] == "OK" for r in rows):
        plot(rows, out, scenarios)
    note_rows = write_note_timeline(out, scenarios)
    if "nota_chute_gol_dinamico" in scenarios:
        plot_note_timeline(note_rows, out, "nota_chute_gol_dinamico")
    summary = {}
    for s in scenarios:
        rs = [r for r in rows if r["cenario"] == s and r["status"] == "OK"]
        n = len(rs)
        goals = sum(r.get("gol_a_favor") == "nosso" for r in rs)
        shots = sum(r.get("disparou") is True for r in rs)
        saves = sum(r.get("resultado_chute") == "defesa_goleiro_provavel" for r in rs)
        lo, hi = wilson(goals, n)
        summary[s] = {
            "tentadas": sum(r["cenario"] == s for r in rows),
            "validas": n,
            "disparos": shots,
            "gols": goals,
            "taxa_gols": goals / n if n else None,
            "ic95_wilson_gols": [lo, hi] if n else None,
            "defesas_provaveis": saves,
            "sem_disparo": n - shots,
            "avaliacoes_de_nota": sum(len(extract_note_timeline(
                (out / "diagnosticos" / f"{s}_{int(r['repeticao']):02d}.log").read_text(encoding="utf-8", errors="replace"),
                s, int(r["repeticao"]))) for r in rs),
        }
    (out / "resumo.json").write_text(json.dumps({"repeticoes_planejadas": reps, "cenarios": summary}, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"Dados e gráficos: {out}")
    print(json.dumps(summary, ensure_ascii=False))


if __name__ == "__main__":
    main()
