#!/bin/bash
# Inicializa o ambiente com a cobertura do time azul forcada.

ZAGUEIRO_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$ZAGUEIRO_SCRIPT_DIR/ararabots.sh"

export ARARABOTS_FORCAR_COBERTURA=1
export ARARABOTS_SO_NOSSOS=1
export ARARABOTS_ROBOS=2

selecionar_categoria() {
  case "$1" in
    cobertura)
      ZAGUEIRO_CATEGORIA=zc_
      export CENARIO_ALVO=zc_centro ARARABOTS_SO_NOSSOS=1 ARARABOTS_ROBOS=2 ;;
    goleiro)
      ZAGUEIRO_CATEGORIA=zg_
      export CENARIO_ALVO=zg_centro ARARABOTS_SO_NOSSOS=1 ARARABOTS_ROBOS=2 ;;
    atacante)
      ZAGUEIRO_CATEGORIA=atacante
      # zv_disputa e zv_deles usam quatro vagas por time no grSim.
      export CENARIO_ALVO=zv_deles ARARABOTS_SO_NOSSOS= ARARABOTS_INIMIGO_PARADO= ARARABOTS_ROBOS=4 ;;
    *) return 2 ;;
  esac
}

listar_cenarios() {
  local nome titulo
  while IFS='|' read -r nome titulo; do
    case "$ZAGUEIRO_CATEGORIA:$nome" in
      zc_:zc_*|zg_:zg_*|atacante:b1_*|atacante:zv_disputa|atacante:zv_deles)
        printf '%s|%s\n' "$nome" "$titulo" ;;
    esac
  done < <(python3 "$PY" listar)
  return 0
}

menu_categorias() {
  local escolha
  while true; do
    printf '\n1) Cobertura e bola\n2) Zagueiro, goleiro e bola\n3) Zagueiro x atacante inimigo\nq) Sair\n'
    read -rp 'Categoria: ' escolha || return 0
    case "$escolha" in
      1) selecionar_categoria cobertura; cmd_menu "$@" ;;
      2) selecionar_categoria goleiro; cmd_menu "$@" ;;
      3) selecionar_categoria atacante; cmd_menu "$@" ;;
      q|Q) return 0 ;;
      *) echo 'Categoria invalida' ;;
    esac
  done
}

uso() {
  cat <<'USO'
Ambiente de zagueiro/cobertura — nosso time e o azul.

  ./docs/zagueiro.sh                 escolhe uma categoria de testes
  ./docs/zagueiro.sh cobertura       so zagueiro e bola
  ./docs/zagueiro.sh goleiro         zagueiro, goleiro e bola
  ./docs/zagueiro.sh atacante        zagueiro contra um atacante amarelo
  ./docs/zagueiro.sh cobertura --headless  simulador sem janela
  ./docs/zagueiro.sh goleiro --headless     simulador sem janela
  ./docs/zagueiro.sh atacante --headless    simulador sem janela
  ./docs/zagueiro.sh listar          lista os testes por categoria
  ./docs/zagueiro.sh preparar        monta o ambiente
  ./docs/zagueiro.sh preparar --headless  simulador sem janela
  ./docs/zagueiro.sh preparar --janela    simulador com janela
  ./docs/zagueiro.sh limpar          limpa os nodes ROS
  ./docs/zagueiro.sh parar           encerra o ambiente

Em cobertura e goleiro, apenas os robos azuis estao em campo. Em atacante,
um robo amarelo joga contra o zagueiro azul.
USO
}

case "${1:-menu}" in
menu)
  shift 2>/dev/null || true
  menu_categorias "$@"
  ;;
cobertura | goleiro | atacante)
  categoria="$1"
  shift
  selecionar_categoria "$categoria"
  cmd_menu "$@"
  ;;
listar)
  for categoria in cobertura goleiro atacante; do
    selecionar_categoria "$categoria"
    printf '\n%s:\n' "$categoria"
    listar_cenarios
  done
  ;;
preparar)
  shift
  cmd_preparar "$@"
  ;;
limpar) cmd_limpar ;;
parar) cmd_parar ;;
-h | --help) uso ;;
--headless | --janela | --window | --so-menu) menu_categorias "$@" ;;
*)
  echo "Opcao invalida: $1" >&2
  uso >&2
  exit 2
  ;;
esac
