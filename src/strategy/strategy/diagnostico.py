"""Diagnostic output, switched on with DIAG_JOGO=1.

Every probe in the strategy goes through here so the switch is read in one place
and a probe never changes what the robots do. Tags in use: [BT] tree, [LG] kickoff
timing, [JG] open play, [TR] role swaps, [AB] movement A/B test, [FK] free kick.
"""

import os

ARQUIVO_JOGO = "/tmp/diag_jogo.log"
_arquivo_iniciado = False


def ativo() -> bool:
    return bool(os.environ.get("DIAG_JOGO"))


def log(linha: str) -> None:
    if ativo():
        print(linha, flush=True)


def log_e_arquivo(linha: str) -> None:
    """Print and append to ARQUIVO_JOGO, which is truncated on the first write."""
    global _arquivo_iniciado
    if not ativo():
        return
    print(linha, flush=True)
    try:
        with open(ARQUIVO_JOGO, "a" if _arquivo_iniciado else "w", encoding="utf-8") as f:
            f.write(linha + "\n")
        _arquivo_iniciado = True
    except OSError:
        pass
