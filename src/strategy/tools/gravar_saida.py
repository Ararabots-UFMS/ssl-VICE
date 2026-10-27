"""Record every command the strategy sends, to prove a refactor changes nothing.

Runs the tree over seeded random worlds for several ticks each, with a toy physics
so the stateful parts (role hysteresis, aim lock, carrier phases) evolve, under
several experiment-switch combinations. Same code in, same file out.

    git worktree add /tmp/antes origin/dev          # the code before your change
    cd src/strategy
    PYTHONPATH=/tmp/antes/src/strategy:/tmp/antes/src/utils \
        python3 tools/gravar_saida.py --saida /tmp/antes.jsonl
    PYTHONPATH=.:../utils python3 tools/gravar_saida.py --saida /tmp/depois.jsonl
    cmp /tmp/antes.jsonl /tmp/depois.jsonl          # silent = identical behaviour

A behaviour change is expected to make them differ; this tool is for refactors.
"""

import argparse
import json
import math
import os
import random
import sys
import time
from types import SimpleNamespace
from unittest.mock import MagicMock

_m = MagicMock()
sys.modules["movement_interfaces"] = _m
sys.modules["movement_interfaces.msg"] = _m.msg
# Simulated clock: freekick timers must not depend on wall time.
_RELOGIO = [1000.0]
time.time = lambda: _RELOGIO[0]
time.monotonic = lambda: _RELOGIO[0]

VARIANTES = [
    {},
    {"ARARABOTS_SEM_PLANEJADOR": "1"},
    {"ARARABOTS_SEM_MIRA_FIRME": "1", "ARARABOTS_SEM_PRESSAO_BOLA": "1",
     "ARARABOTS_SEM_PROTECAO": "1", "ARARABOTS_SEM_PARAR_E_MIRAR": "1"},
    {"ARARABOTS_PAPEIS_FIXOS": "1", "ARARABOTS_SEM_ORBITA": "1",
     "ARARABOTS_SEM_EMPURRAO": "1", "ARARABOTS_SEM_CHEGADA_ALINHADA": "1"},
    {"ARARABOTS_COM_DESTINO_LONGO": "1", "ARARABOTS_SO_POSICIONAR": "1",
     "ARARABOTS_COM_CHEGADA_EM_MOVIMENTO": "1", "ARARABOTS_SEM_DISPARO_ALINHADO": "1",
     "ARARABOTS_SEM_ORIENTACAO_LADO": "1"},
]


def robo(rid, x, y, th, vx=0.0, vy=0.0):
    return SimpleNamespace(id=rid, robot_id=rid, position_x=x, position_y=y,
                           orientation=th, velocity_x=vx, velocity_y=vy)


def mundo(rng):
    n_al = rng.choice([1, 2, 3, 4, 4, 4, 5, 6])
    ids = list(range(n_al)) if rng.random() < 0.85 else list(range(1, n_al + 1))
    al = {i: robo(i, rng.uniform(-4300, 4300), rng.uniform(-2900, 2900),
                  rng.uniform(-math.pi, math.pi)) for i in ids}
    n_in = rng.choice([0, 0, 1, 2, 3, 4, 6])
    en = {i: robo(i, rng.uniform(-4300, 4300), rng.uniform(-2900, 2900),
                  rng.uniform(-math.pi, math.pi)) for i in range(n_in)}
    bx, by = rng.uniform(-4400, 4400), rng.uniform(-2950, 2950)
    # a third of the worlds put the ball right next to someone, where most logic is
    if rng.random() < 0.35 and al:
        r = rng.choice(list(al.values()))
        a = rng.uniform(-math.pi, math.pi)
        d = rng.uniform(90, 400)
        bx, by = r.position_x + d * math.cos(a), r.position_y + d * math.sin(a)
    v = rng.choice([0, 0, 0, 300, 1500, 4000])
    a = rng.uniform(-math.pi, math.pi)
    bola = SimpleNamespace(position_x=bx, position_y=by,
                           velocity_x=v * math.cos(a), velocity_y=v * math.sin(a))
    return al, en, bola, rng.random() < 0.5


def avancar(al, en, bola, cmds, rng):
    """Toy physics: robots step toward their targets, ball rolls and gets kicked."""
    por_id = {c["robot_id"]: c for c in cmds}
    for rid, r in al.items():
        c = por_id.get(rid)
        if c is None or c["target_x"] is None:
            continue
        dx, dy = c["target_x"] - r.position_x, c["target_y"] - r.position_y
        d = math.hypot(dx, dy)
        passo = min(d, 90.0)
        if d > 1e-9:
            r.velocity_x, r.velocity_y = dx / d * passo * 10, dy / d * passo * 10
            r.position_x += dx / d * passo + rng.uniform(-2, 2)
            r.position_y += dy / d * passo + rng.uniform(-2, 2)
        if c["angle"] is not None:
            e = math.atan2(math.sin(c["angle"] - r.orientation),
                           math.cos(c["angle"] - r.orientation))
            r.orientation += max(-0.4, min(0.4, e))
        db = math.hypot(bola.position_x - r.position_x, bola.position_y - r.position_y)
        if db < 115:
            if c["kick"] > 0:
                k = c["kick"] * 1000.0
                bola.velocity_x = k * math.cos(r.orientation)
                bola.velocity_y = k * math.sin(r.orientation)
            else:
                bola.velocity_x, bola.velocity_y = r.velocity_x, r.velocity_y
    for e in en.values():
        dx, dy = bola.position_x - e.position_x, bola.position_y - e.position_y
        d = math.hypot(dx, dy) or 1.0
        e.position_x += dx / d * 40 + rng.uniform(-20, 20)
        e.position_y += dy / d * 40 + rng.uniform(-20, 20)
    bola.position_x = max(-4500, min(4500, bola.position_x + bola.velocity_x * 0.1))
    bola.position_y = max(-3000, min(3000, bola.position_y + bola.velocity_y * 0.1))
    bola.velocity_x *= 0.8
    bola.velocity_y *= 0.8


def r6(v):
    if v is None:
        return None
    if isinstance(v, bool):
        return v
    if isinstance(v, (int, float)):
        return round(float(v), 6)
    if isinstance(v, (list, tuple)):
        return [r6(x) for x in v]
    return str(v)


CAMPOS = ("robot_id", "target_x", "target_y", "vel_x", "vel_y", "angle", "kick",
          "field_border", "penalty_area", "center_area", "ball", "enemy_ids", "ally_ids")


def serial(cmds):
    out = []
    for c in cmds or []:
        out.append({k: r6(getattr(c, k, None)) for k in CAMPOS})
    return out


# ---------------------------------------------------------------- adapters
def jogo():
    from strategy.context import GameConfig, TickContext
    from strategy.plays.running import NormalStart
    from strategy.context import TreeDeps

    play = NormalStart("NormalStart", TreeDeps.null())

    def tick(al, en, bola, lado):
        ctx = TickContext(ally_robots=dict(al), enemy_robots=dict(en), balls=(bola,),
                          referee_command="FORCE_START",
                          config=GameConfig(on_positive_half=lado))
        _, acao = play.run(ctx)
        return acao
    return tick


def bola_parada():
    """Set-piece tactics, called the way their plays call them."""
    from strategy.tatics.kickoff import OurKickoff, TheirKickoff
    from strategy.tatics.stop import goAwayFromBall
    from strategy.tatics.halt import HaltAction
    from strategy.tatics.freekick import OurFreekick, TheirFreekick, EstadoFreekick, MedidasCampo

    est = {"fk": EstadoFreekick()}

    def tick(al, en, bola, lado, qual):
        if qual == "kick_nosso":
            return OurKickoff(ally_robots=al, on_positive_half=lado).execute()
        if qual == "kick_deles":
            return TheirKickoff(ally_robots=al, on_positive_half=lado).execute()
        if qual == "stop":
            return goAwayFromBall(al, bola, lado).execute()
        if qual == "halt":
            return HaltAction(al).execute()
        cls = OurFreekick if qual == "fk_nosso" else TheirFreekick
        _, cmds = cls(ally_robots=al, ball=bola, on_positive_half=lado,
                      enemy_robots=en, estado=est["fk"], medidas=MedidasCampo()).execute()
        return cmds
    return tick, est


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--saida", required=True)
    ap.add_argument("--mundos", type=int, default=300)
    ap.add_argument("--ticks", type=int, default=50)
    args = ap.parse_args()

    devnull = open(os.devnull, "w")
    with open(args.saida, "w") as f:
        for iv, var in enumerate(VARIANTES):
            for k in [k for k in os.environ if k.startswith("ARARABOTS_")]:
                del os.environ[k]
            os.environ.update(var)
            for w in range(args.mundos):
                rng = random.Random(iv * 100000 + w)
                al, en, bola, lado = mundo(rng)
                tick = jogo()
                for t in range(args.ticks):
                    _RELOGIO[0] += 0.1
                    out = sys.stdout
                    sys.stdout = devnull
                    try:
                        cmds = tick(al, en, bola, lado)
                        res = serial(cmds)
                    except Exception as e:  # a crash is part of the behaviour
                        res = "ERRO %s: %s" % (type(e).__name__, e)
                        cmds = []
                    finally:
                        sys.stdout = out
                    f.write(json.dumps({"v": iv, "w": w, "t": t, "jogo": res}) + "\n")
                    if isinstance(res, str):
                        break
                    avancar(al, en, bola, res, rng)
            # set pieces: fewer worlds, each kind
            for qual in ("kick_nosso", "kick_deles", "stop", "halt", "fk_nosso", "fk_deles"):
                for w in range(args.mundos // 6):
                    rng = random.Random(9000000 + iv * 100000 + w)
                    al, en, bola, lado = mundo(rng)
                    tick, _ = bola_parada()
                    for t in range(args.ticks // 2):
                        _RELOGIO[0] += 0.1
                        out = sys.stdout
                        sys.stdout = devnull
                        try:
                            res = serial(tick(al, en, bola, lado, qual))
                        except Exception as e:
                            res = "ERRO %s: %s" % (type(e).__name__, e)
                        finally:
                            sys.stdout = out
                        f.write(json.dumps({"v": iv, "q": qual, "w": w, "t": t,
                                            "cmd": res}) + "\n")
                        if isinstance(res, str):
                            break
                        avancar(al, en, bola, res, rng)
    print("ok", args.saida)


if __name__ == "__main__":
    main()
