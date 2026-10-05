"""Posicionar-se sem a bola: oferta, marcacao, bloqueio, espaco.

CAMADA DE SKILLS - nivel 2.

POR QUE ESTE MODULO EXISTE
--------------------------
"Ficar entre a bola e o nosso gol" estava escrito duas vezes com constantes
diferentes (BLOQUEIO_DIST no apoio, COBERTURA_FRACAO na cobertura), e
"oferecer-se para receber" outras duas (alvo_do_papel no jogo corrido,
_posicao_de_apoio na bola parada). Sao quatro variantes de duas ideias.

E AVANCO_MINIMO_PASSE existia DUAS vezes com o mesmo valor e o mesmo motivo:
tatics/goalkeeper.py e tatics/running.py.
"""

from math import hypot

from strategy.skills.geometria import no_campo, versor

# --- oferta ofensiva ------------------------------------------------------
# 1800/1600, e nao 1200/900: com 1200 a frente e 900 de lado o apoio ficava
# DENTRO do corredor por onde o portador precisa empurrar, e o chute caiu de
# 5525 para 1536 mm/s. Medido duas vezes, revertido duas vezes.
APOIO_AVANCO = 1800.0
APOIO_ABERTURA = 1600.0
# Recuo do apoio quando a via esta obstruida: atras da bola, onde a linha
# costuma estar limpa porque a marcacao vem de frente.
APOIO_RECUO = 900.0
# O receptor precisa estar ao menos isto A FRENTE da bola, projetado no eixo do
# ataque. Menos que isso e passe lateral, ou passe para tras. A licao vem da
# cobranca de falta (_companheiro_para_passe).
AVANCO_MINIMO_PASSE = 600.0
# Quanto a frente da bola o apoio ocupa espaco com a bola solta.
PORTADOR_ESPACO = 1500.0

# --- marcacao e bloqueio -------------------------------------------------
# A que distancia da bola o apoio se planta para tapar a linha de chute quando o
# adversario tem a posse. Medido: a bola saindo do pe deles direto para o nosso
# fundo, 5461 e 5377 mm em linha reta, com UM unico toque na partida inteira.
BLOQUEIO_DIST = 600.0
# A que distancia do adversario marcado o apoio se planta, do lado do nosso gol.
# 400 mm: fora do raio de posse (250) para nao virar dividida, e perto o
# bastante para tirar o angulo de quem for receber.
MARCACAO_DIST = 400.0
# Fracao do caminho bola->nosso gol em que a cobertura se planta.
COBERTURA_FRACAO = 0.45


def a_frente_da_bola(robo, bx, by, gol_ataque):
    """Projecao do robo no eixo bola->gol de ataque. Negativo = atras da bola."""
    ux, uy, _ = versor(bx, by, gol_ataque.x, gol_ataque.y)
    return (robo.position_x - bx) * ux + (robo.position_y - by) * uy


def pode_receber(robo, bx, by, gol_ataque, avanco_min=AVANCO_MINIMO_PASSE):
    """Este companheiro esta adiantado o bastante para ser alvo de passe?"""
    return a_frente_da_bola(robo, bx, by, gol_ataque) >= avanco_min


def oferta_de_passe(bx, by, uax, uay, pax, pay, lado,
                    avanco=APOIO_AVANCO, abertura=APOIO_ABERTURA):
    """A frente da bola e ABERTO, fora do corredor do portador."""
    return no_campo(bx + uax * avanco + pax * abertura * lado,
                    by + uay * avanco + pay * abertura * lado)


def recuo_de_apoio(bx, by, uax, uay, pax, pay, lado,
                   recuo=APOIO_RECUO, abertura=APOIO_ABERTURA):
    """Via obstruida: atras da bola, meia abertura."""
    return no_campo(bx - uax * recuo + pax * abertura * 0.5 * lado,
                    by - uay * recuo + pay * abertura * 0.5 * lado)


def ocupar_espaco(bx, by, uax, uay, avanco=PORTADOR_ESPACO, fracao_y=0.3):
    """Adiante da bola, no meio: alguem tem de estar a frente na recuperacao.

    Medido: ZERO quadros com robo nosso alem de x=3500 em todas as execucoes
    desta fase, porque todo mundo descia.
    """
    return no_campo(bx + uax * avanco, by + uay * avanco * fracao_y)


def bloquear_linha(bx, by, nosso_gol, distancia=BLOQUEIO_DIST):
    """Entre a bola e a NOSSA meta, a 'distancia' da bola."""
    ux, uy, _ = versor(bx, by, nosso_gol.x, nosso_gol.y)
    return no_campo(bx + ux * distancia, by + uy * distancia)


def marcar(ameaca, nosso_gol, distancia=MARCACAO_DIST):
    """Entre o adversario marcado e a NOSSA meta, encostado nele."""
    ux, uy, _ = versor(ameaca.position_x, ameaca.position_y,
                       nosso_gol.x, nosso_gol.y)
    return no_campo(ameaca.position_x + ux * distancia,
                    ameaca.position_y + uy * distancia)


def ameaca_mais_perigosa(inimigos, nosso_gol):
    """O adversario mais proximo da nossa meta, na nossa metade. None se nao ha.

    E quem recebe o passe seguinte. Simplificacao conhecida: nao pondera quem
    tem linha de recepcao livre - e o item B6 do plano.
    """
    if not inimigos:
        return None
    sinal = 1.0 if nosso_gol.x > 0 else -1.0
    candidatos = [e for e in inimigos.values() if e.position_x * sinal > 0]
    if not candidatos:
        return None
    return min(candidatos, key=lambda e: hypot(e.position_x - nosso_gol.x,
                                               e.position_y - nosso_gol.y))


def cobertura_na_linha(bx, by, nosso_gol, fracao=COBERTURA_FRACAO,
                       recuo_extra=0.0, margem=200.0):
    """SOBRE a reta bola->nosso gol, nao ao lado dela.

    O deslocamento para nao empilhar era PERPENDICULAR a essa linha, ou seja
    tirava a cobertura de cima dela de proposito. Medido nos replays: o
    adversario chuta do meio-campo, a bola percorre 4122-5437 mm em linha reta
    ate a nossa linha de fundo, atravessa o time inteiro, e o unico que toca
    nela e o GOLEIRO. Quem sobra se espalha AO LONGO da reta, mais perto do gol:
    dois corpos no mesmo corredor cobrem o rebote, dois ao lado nao cobrem nada.
    """
    ux, uy, n = versor(bx, by, nosso_gol.x, nosso_gol.y)
    alvo = min(n * fracao + recuo_extra, n - margem)
    return no_campo(bx + ux * alvo, by + uy * alvo)
