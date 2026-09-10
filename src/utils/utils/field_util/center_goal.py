from utils.math_util import Vector2D


class CenterGoal:
    """Centro de cada gol, em milimetros.

    4500, e nao 2250. 2250 e a meia-largura de um campo SSL-EL (4500 x 3000);
    este projeto roda em Division B, 9000 x 6000 - confirmado pelas regras
    oficiais (sslrules.pdf, secao 2.1.1) e pelo proprio /game_state, que reporta
    campo=9000mm.

    O QUE O VALOR ERRADO CAUSAVA, e nao e sutil: o goleiro se posicionava
    2250 mm A FRENTE da propria meta - ou seja, abandonava o gol e parava perto
    do meio-campo - e os atacantes miravam um ponto vazio no meio do campo
    adversario. Em jogo aberto o time inteiro converge para o centro.

    O defeito estava duplicado em quatro taticas (freekick, kickoff, stop,
    running); a refatoracao que extraiu esta classe centralizou o valor mas
    trouxe o numero antigo junto. Agora ha um lugar so para corrigir, que era
    justamente o objetivo dela.
    """

    GOAL_POSITIVE = Vector2D(4500.0, 0.0)
    GOAL_NEGATIVE = Vector2D(-4500.0, 0.0)
