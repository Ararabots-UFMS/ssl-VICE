"""Chaves de experimento: ligar e desligar UMA modificacao por vez.

POR QUE ISTO EXISTE
-------------------
Quatro mudancas de comportamento entraram no mesmo dia (03/10/2026). Medir as
quatro juntas produz um numero que nao se sabe atribuir - a regra do projeto e
UMA mudanca por lote. Para medir o antes e o depois de cada uma sem trocar de
branch no meio do lote, cada uma tem uma chave propria.

O DEFAULT E O COMPORTAMENTO NOVO. A chave DESLIGA a modificacao, devolvendo o
codigo antigo. Assim o lote "desligada" mede a linha de base e o lote "ligada"
mede a mudanca, com o MESMO binario.

COMO LIGAR A CHAVE (os dois jeitos funcionam):

    variavel de ambiente:   ARARABOTS_SEM_ORBITA=1 ./ararabots.sh validar 3 ...
    arquivo em /tmp:        touch /tmp/ararabots_sem_orbita

A variavel e o caminho normal - ela esta plumbada no 'ros_d' do ararabots.sh,
que e quem sobe o strategyNode. O arquivo existe porque bandeira lida pela
ESTRATEGIA nao chega la quando alguem roda o node na mao; e e o padrao que o
projeto ja usava (/tmp/ararabots_papeis_fixos).

ATENCAO, ja custou lote: bandeira em /tmp NAO sobrevive a remontagem do
container, e o 'validar' remonta quando a contagem de robos do cenario difere da
atual. Com variavel de ambiente isso nao acontece.
"""

import os

# As quatro modificacoes de 03/10/2026 que mudam comportamento.
#
#   ORIENTACAO_LADO  o corpo nao vira as costas para a bola (chute.py)
#   ORBITA           bola atras de nos -> contorna e pega por tras (aproximacao.py)
#   PROTECAO         saida sob pressao por varredura, fugindo de quem prensa
#   PRESSAO_BOLA     pressao medida na BOLA, nao no corpo do robo
#
# As duas de 07/10/2026, que sairam da LEITURA DOS REPLAYS do lote de 03/10 -
# as duas respondem "por que a bola nao andava em nenhuma condicao":
#
#   MIRA_FIRME       a mira escolhida vale por 2 s mesmo que o ciclo seguinte
#                    diga 'bloqueado'. A trava existia e excluia justamente a
#                    transicao que acontecia: medido em 'orientacao_terco',
#                    132 ciclos 'passe' alternando com 148 'bloqueado', e como
#                    TODA direcao do ciclo sai da mira, a linha de tiro girava
#                    e o portador orbitava a bola a 314-652 mm sem nunca
#                    encostar nela (alinhamento mediano t = 0,11).
#   EMPURRAO         no contato, o alvo fica ALEM da bola quando a chegada esta
#                    alinhada. Era 53 mm AQUEM dela em 100% dos quadros de
#                    contato medidos (sonda guiada por replay), e o casco
#                    impede chegar a menos de 111 mm: o erro residual virava
#                    ~0,13 m/s e a bola nao saia do lugar.
CHAVES = ("ORIENTACAO_LADO", "ORBITA", "PROTECAO", "PRESSAO_BOLA",
          "MIRA_FIRME", "EMPURRAO")


def desligado(nome):
    """A modificacao 'nome' esta DESLIGADA (isto e, vale o codigo antigo)?"""
    if nome not in CHAVES:                      # erro de digitacao nao passa calado
        raise ValueError("chave de experimento desconhecida: %r" % (nome,))
    if os.environ.get("ARARABOTS_SEM_" + nome):
        return True
    return os.path.exists("/tmp/ararabots_sem_" + nome.lower())


def estado():
    """Dicionario {chave: ligada?}, para o diagnostico e para o replay."""
    return {c: (not desligado(c)) for c in CHAVES}
