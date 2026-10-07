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


# --- saida sob pressao -----------------------------------------------------
# Alcance considerado ao pontuar uma direcao de saida. 1800 mm: o bastante para
# a bola sair da prensa e ainda parar em campo.
ALCANCE_SAIDA = 1800.0
# Quanto a direcao da propria meta e proibida, em radianos. Um chute forte para
# o nosso campo entrega a bola com velocidade onde eles chutam.
CONE_PROIBIDO = 0.9
# Peso do vies de ataque, em mm por radiano de desvio: a direcao mais aberta
# ganha, mas entre duas parecidas vence a que aponta mais para o gol deles.
PESO_ATAQUE = 260.0


def saida_sob_pressao(bx, by, inimigos, gol_ataque, nosso_gol, ameaca=None,
                      alcance=ALCANCE_SAIDA, passos=24):
    """Para onde mandar a bola quando o portador esta prensado.

    O QUE ISTO SUBSTITUI, e por que
    -------------------------------
    O alivio era FIXO: "joga na lateral", sempre no mesmo y limite, com 800 mm
    de avanco. Tirava a bola da prensa, mas sem olhar se a lateral estava livre
    - e, principalmente, sem relacao com ONDE o adversario esta. O pedido do
    Felipe e o oposto: o corpo entre o adversario e a bola, em vez de so mirar
    a lateral.

    Aqui a direcao e escolhida por VARREDURA ANGULAR: pontua cada direcao pela
    folga ate o adversario mais proximo da linha, com um vies para o lado do
    ataque, e proibe o cone da propria meta. E o mesmo principio que a saida de
    bola do terco defensivo ja usava, e o mesmo que o item B1 do plano quer para
    o chute a gol.

    O "CORPO ENTRE O ADVERSARIO E A BOLA" EXIGE RESTRINGIR A VARREDURA.
    ------------------------------------------------------------------
    Isto eu achei MEDINDO, depois de errar o raciocinio. Como o portador mira
    ATRAVES da bola (ver alvo_do_papel), ele se posiciona do lado OPOSTO a
    direcao de saida. Entao o corpo so fica entre o adversario e a bola se a
    direcao de saida apontar para LONGE do adversario.
    
    Sem a restricao, a varredura escolhia a direcao mais aberta - que muitas
    vezes nao e "fugir da pressao" - e o resultado media assim (sonda offline,
    8 largadas x 3 cenas de prensa, escudo = angulo entre bola->robo e
    bola->adversario; 0 e o casco no meio):
    
        prensa frontal         escudo mediano  74 graus
        prensa lateral         escudo mediano 157 graus
        prensa no nosso terco  escudo mediano 163 graus
    
    Ou seja: na metade dos casos o robo ia para o lado OPOSTO ao adversario -
    protegia nada. Com 'ameaca' informada, as direcoes que empurram a bola na
    direcao dela sao descartadas, e o corpo passa a ficar no meio por geometria.
    """
    from math import atan2, cos, sin
    from strategy.skills.geometria import folga_lateral, no_campo, norm_ang

    ang_ataque = atan2(gol_ataque.y - by, gol_ataque.x - bx)
    ang_meta = atan2(nosso_gol.y - by, nosso_gol.x - bx)
    ang_ameaca = None if ameaca is None else atan2(ameaca.position_y - by,
                                                  ameaca.position_x - bx)
    melhor, melhor_nota = None, None
    for k in range(passos):
        a = -3.14159265 + 2 * 3.14159265 * k / passos
        if abs(norm_ang(a - ang_meta)) < CONE_PROIBIDO:
            continue                      # nunca na direcao da propria meta
        if ang_ameaca is not None and abs(norm_ang(a - ang_ameaca)) < 1.57:
            continue                      # nunca para o lado de quem prensa
        ux, uy = cos(a), sin(a)
        folga = min(folga_lateral(bx, by, ux, uy, alcance, inimigos), 1500.0)
        nota = folga - PESO_ATAQUE * abs(norm_ang(a - ang_ataque))
        if melhor_nota is None or nota > melhor_nota:
            melhor_nota = nota
            melhor = no_campo(bx + ux * alcance, by + uy * alcance)
    return melhor


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
