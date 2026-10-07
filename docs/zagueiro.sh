#!/bin/bash
# Menu dedicado aos cenarios de zagueiro/cobertura do time azul.
# Reutiliza a montagem, a gravacao e os replays de ararabots.sh.

ZAGUEIRO_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$ZAGUEIRO_SCRIPT_DIR/ararabots.sh"

# Goleiro + tres de linha: portador, apoio e cobertura precisam coexistir.
export ARARABOTS_ROBOS=4
export CENARIO_ALVO=zv_corredor

listar_cenarios() {
    python3 "$PY" listar | awk -F '|' '
        $1 ~ /^zv_/ || $1 == "zagueiro_vs_dois" ||
        $1 == "defesa_3v3" || $1 == "cobertura_chute_longe"
    '
}

uso() {
    cat <<'USO'
Testes de zagueiro/cobertura — nosso time e o azul.

  ./docs/zagueiro.sh                 menu e montagem do ambiente
  ./docs/zagueiro.sh --headless      simulador sem janela
  ./docs/zagueiro.sh --janela        simulador com janela
  ./docs/zagueiro.sh --so-menu       usa o ambiente que ja esta rodando
  ./docs/zagueiro.sh listar          lista os cenarios de zagueiro
  ./docs/zagueiro.sh preparar        monta o ambiente
  ./docs/zagueiro.sh limpar          limpa os nodes ROS
  ./docs/zagueiro.sh parar           encerra o ambiente

No menu: numero = um teste; t = todos os testes de zagueiro;
d = repetir um teste; s = mudar a duracao; q = sair.
Os cenarios incluem companheiros e adversarios para avaliar a cobertura.
USO
}

case "${1:-menu}" in
    listar) listar_cenarios ;;
    preparar) shift; cmd_preparar "$@" ;;
    limpar) cmd_limpar ;;
    parar) cmd_parar ;;
    -h|--help) uso ;;
    menu) shift 2>/dev/null || true; cmd_menu "$@" ;;
    --headless|--janela|--window|--so-menu) cmd_menu "$@" ;;
    *) echo "Opcao invalida: $1" >&2; uso >&2; exit 2 ;;
esac
