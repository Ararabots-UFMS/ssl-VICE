"""Geometria e predicados de campo. Sem estado, sem ROS, sem bola.

CAMADA DE SKILLS - nivel 1 (o mais baixo).

POR QUE ESTE MODULO EXISTE
--------------------------
As mesmas contas estavam escritas em tres lugares: 'linha_livre' em
tatics/running.py, '_linha_livre' como metodo em tatics/freekick.py (com outra
folga por default), e uma varredura propria em tatics/goalkeeper.py. As tres
respondiam a mesma pergunta - "tem alguem no caminho?" - e divergiram: a do
goleiro era chamada com um VERSOR no lugar das coordenadas de destino, entao
conferia um segmento de 1 mm na origem do campo e nao conferia nada.

Nada aqui depende de rclpy: da para testar com dicionarios de robos de mentira.
"""

from math import cos, hypot, pi, sin

# Folga exigida entre o segmento e o adversario mais proximo, em mm.
#
# 180 mm: o robo tem 180 de diametro, logo isto equivale a "cabe um robo". E o
# criterio BINARIO que o item B1 do plano substitui por varredura angular - ver
# documentacao/strategy-analysis/implementation-plan.md.
FOLGA_LINHA = 180.0

# Limites uteis do campo para alvos (Division B e 9000 x 6000; aqui fica a
# margem de 200 mm que impede pedir alvo fora da linha).
LIMITE_X_ALVO = 4300.0
LIMITE_Y_ALVO = 2800.0


def norm_ang(a):
    """Angulo em (-pi, pi]."""
    while a > pi:
        a -= 2.0 * pi
    while a <= -pi:
        a += 2.0 * pi
    return a


def dist(ax, ay, bx, by):
    return hypot(ax - bx, ay - by)


def versor(ax, ay, bx, by):
    """Versor a->b e a norma. A norma nunca volta zero (vira 1.0)."""
    dx, dy = bx - ax, by - ay
    n = hypot(dx, dy) or 1.0
    return dx / n, dy / n, n


def perpendicular(ux, uy):
    return -uy, ux


def no_campo(x, y, limite_x=LIMITE_X_ALVO, limite_y=LIMITE_Y_ALVO):
    """Satura um alvo dentro do campo."""
    return (max(-limite_x, min(limite_x, x)),
            max(-limite_y, min(limite_y, y)))


def linha_livre(ox, oy, dx_, dy_, inimigos, folga=FOLGA_LINHA):
    """Nenhum adversario a menos de 'folga' do segmento origem -> destino.

    ATENCAO ao contrato, porque ja foi violado: os quatro primeiros argumentos
    sao COORDENADAS (origem e destino), nao um ponto e um versor.
    """
    vx, vy = dx_ - ox, dy_ - oy
    comp2 = vx * vx + vy * vy
    if comp2 <= 1.0:
        return True
    for r in (inimigos or {}).values():
        wx, wy = r.position_x - ox, r.position_y - oy
        t = max(0.0, min(1.0, (wx * vx + wy * vy) / comp2))
        if hypot(wx - t * vx, wy - t * vy) < folga:
            return False
    return True


def livre_do_lado(x, y, inimigos, folga=320.0):
    """Nenhum adversario a menos de 'folga' deste ponto."""
    return not any(hypot(e.position_x - x, e.position_y - y) < folga
                   for e in (inimigos or {}).values())


def projecao_no_eixo(px, py, ox, oy, ux, uy):
    """Projecao COM SINAL de (o->p) no eixo (ux, uy)."""
    return (px - ox) * ux + (py - oy) * uy


# Velocidade media que um robo realmente alcanca num lance, para estimar quanto
# tempo ele leva para chegar a um ponto. MEDIDO no cenario 'cobertura_chute_longe'
# (4 robos, velocidade media maxima por janela de 0,05-3 s): 755, 982 e 1127 mm/s.
# A versao anterior assumia 2000 mm/s (o teto do solver, que o robo nao atinge
# por causa da aceleracao) e a skill escolhia pontos inalcancaveis a tempo.
VELOCIDADE_ROBO_ESTIMADA = 1000.0  # mm/s


def tempo_de_chegada(ox, oy, px, py, velocidade=VELOCIDADE_ROBO_ESTIMADA):
    """Tempo estimado (s) para ir de (o) a (p) em linha reta, a 'velocidade'."""
    if velocidade <= 0:
        return float("inf")
    return dist(ox, oy, px, py) / velocidade


def folga_lateral(ox, oy, ux, uy, alcance, inimigos):
    """Menor desvio lateral de um adversario em relacao ao raio o + t*u.

    So conta quem esta ENTRE a origem e 'alcance' - adversario atras de nos ou
    alem do alvo nao atrapalha o tiro. Devolve 9999.0 quando nao ha ninguem no
    caminho.
    """
    folga = 9999.0
    for e in (inimigos or {}).values():
        px = e.position_x - ox
        py = e.position_y - oy
        proj = px * ux + py * uy
        if proj <= 0 or proj > alcance:
            continue
        folga = min(folga, abs(-px * uy + py * ux))
    return folga


def angulo_do_corpo(robo):
    """Versor do eixo do corpo do robo."""
    return cos(float(robo.orientation)), sin(float(robo.orientation))
