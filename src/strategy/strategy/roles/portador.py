"""PORTADOR: has the ball or goes for it.

Phases, decided every tick by geometry (skills/aproximacao.fase_de_aproximacao):

  CONTORNAR    go to a fixed point behind the ball; the ball is an obstacle, so the
               planner does the circling.
  MIRAR        hold that point and turn until the body is on the line of fire.
  EMPURRAR     go through the ball towards the kick target; this is where it kicks.
  DESENCOSTAR  inside the ball's keep-out: step radially out first.

A phase only holds for the tick that decided it. Branches that return early
(ball already kicked, legacy loose ball) leave it empty.
"""

from dataclasses import dataclass
import math
from math import atan2, cos, hypot, sin
from typing import Optional

from strategy import campo, diagnostico
from strategy.analysis.jogo import AnaliseJogo
from strategy.analysis.mundo import SITUACAO_SOLTA
from strategy.roles.atribuicao import PAPEL_PORTADOR
from strategy.roles.base import Papel, versor_ataque
from strategy.skills import aproximacao, experimento, geometria
from strategy.skills import bola as skill_bola

CONTORNAR = "contornar"
MIRAR = "mirar"
EMPURRAR = "empurrar"
DESENCOSTAR = "desencostar"

# Body error still worth stopping to aim for. 0.25 rad = 14 deg, inside the
# kick gate's 20 so it leaves the aim phase already within it.
TOL_CORPO_MIRA = 0.25
# Only worth stopping to aim close to the ball; far away it turns while driving.
RAIO_MIRA_PARADO = 600.0
# Limites da BOLA em jogo (meio campo 4500 x 3000), com folga para o raio dela.
LIMITE_BOLA_X, LIMITE_BOLA_Y = 4400.0, 2900.0
# Dentro disto a bola e NOSSA: quem manda e a maquina de fases, nao a
# interceptacao. ALCANCE_CONTATO do chute e 260; aqui um pouco mais, porque a
# bola escapa e volta entre quadros.
RAIO_BOLA_NOSSA = 320.0
# Folga entre onde o empurrao pode deixar a bola e a borda da caixa da area.
FOLGA_BOLA_AREA = 80.0
# Avanco do empurrao quando ja nao cabe o avanco cheio (chave
# AVANCO_MINIMO_NA_AREA). Nao e "um empurrao curto": e o minimo para o alvo de
# movimento ficar ALEM da bola e o canal do chute continuar aberto - ver o
# comentario em '_avanco_antes_da_area'. 150 mm passa do raio do robo (90) mais
# o da bola (21), entao o alvo cai do outro lado dela.
AVANCO_MINIMO_NA_AREA = 150.0
# 2 s of aiming without turning, then push anyway: standing still loses the ball.
CICLOS_MAX_MIRANDO = 20
# Error drop per tick that still counts as turning (0.1 rad/s).
GIRO_MIN_MIRA = 0.01
# Arrival speed at the waiting point, towards the ball: turns the point into a
# state so the planner comes from behind (-121/-142 deg -> -6/-8 measured).
VEL_CHEGADA_CONTORNO = 400.0
# Arrival speed when pushing, only with the COM_CHEGADA_EM_MOVIMENTO experiment.
VEL_CHEGADA_EMPURRAO = 600.0
# Start the push only this close to the line of fire: grSim fires with the ball within
# 40 mm of the kicker centre, and entering at 90 made 16% of first touches pushes.
LATERAL_ENTRADA = 40.0
# Leave the push beyond this: the old entry limit, the hull's radius.
LATERAL_SAIDA = 90.0
# ... and only from near the waiting point, where the last stretch is straight.
RAIO_ENTRADA = aproximacao.APROXIMACAO_ATRAS + 150.0
# Not settled after 1.5 s that close: accept the old corridor rather than stall.
CICLOS_MAX_ALINHANDO = 15


@dataclass
class MemoriaPortador:
    fase: Optional[str] = None
    ciclos_mirando: int = 0
    erro_mira: Optional[float] = None
    ciclos_alinhando: int = 0


class Portador(Papel):
    nome = PAPEL_PORTADOR

    def __init__(self):
        self.memoria = MemoriaPortador()

    def fase(self) -> Optional[str]:
        if experimento.desligado("PLANEJADOR"):
            return None
        return self.memoria.fase

    def alvo(self, a: AnaliseJogo, rid: int, ordem: int):
        m = self.memoria
        fase_anterior, m.fase = m.fase, None
        ball, robo = a.bola, a.mundo.aliados[rid]
        bx, by = ball.position_x, ball.position_y
        rx, ry = robo.position_x, robo.position_y
        ux, uy, _, _ = versor_ataque(a)
        solta = a.mundo.situacao == SITUACAO_SOLTA

        # Legacy loose ball (planner switched off): run to where it will stop.
        if solta and experimento.desligado("PLANEJADOR"):
            fx, fy = skill_bola.onde_a_bola_vai(ball, 1.0)
            direcao = a.alvo_chute if a.alvo_chute is not None else (
                fx + ux * 1000.0, fy + uy * 1000.0)
            return aproximacao.ponto_de_aproximacao(
                rx, ry, fx, fy, direcao, aproximacao.AVANCO_SOLTA,
                contornar=not experimento.desligado("ORBITA"),
                empurrar=not experimento.desligado("EMPURRAO"))

        # Ball already kicked: intercept beyond it. Catching a ball in flight brakes
        # it (robot.cpp:168); the 1.6 m follow-through is what produces the kick.
        # SO VAI PARA A INTERCEPTACAO PURA SE A BOLA ESTIVER MESMO VOANDO.
        #
        # DEFEITO MEDIDO EM 28/10/2026. Este ramo sai ANTES da maquina de
        # fases, e deixa 'fase' em None - o proprio cabecalho do modulo diz
        # "(ball already kicked, legacy loose ball) leave it empty". Mas e a
        # FASE que libera o gatilho: chute.orientacao_do_corpo so devolve
        # "pode armar" em 'empurrar' ou 'mirar'.
        #
        # Com a bola rolando, 'bola_ja_saiu' (250 mm/s no Kalman) fica VERDADE
        # a execucao inteira: medimos fase=None em 24 de 24 amostras. O
        # portador roda 15 s num ramo que nunca arma, e so marca quando a bola
        # ja parou e ele cai na maquina de fases por acaso:
        #
        #     chutador armado no toque:  20 de 21 viram gol (95%)
        #     nao armado:                12 de 30 (40%), 15 sem_chute
        #
        # 250 mm/s foi escolhido para "a bola JA FOI CHUTADA - ninguem
        # persegue, intercepta", pensando em bola de chute (4-6 m/s). Uma bola
        # de passe a 1-3 m/s morre em 1-2 s e e para COLETAR, nao para
        # interceptar. VEL_PERSEGUIR (600) e o limiar que o proprio bola.py ja
        # documentava para "REALMENTE viajando".
        rapida = (skill_bola.bola_ja_saiu(ball)
                  if experimento.desligado("INTERCEPTAR_SO_RAPIDA")
                  else skill_bola.bola_viajando(ball))

        # BOLA QUE NOS MESMOS ESTAMOS EMPURRANDO NAO E BOLA VIAJANDO.
        #
        # DEFEITO MEDIDO em 03/11/2026, nas DUAS unicas falhas do lote de 20
        # headless (docs/campanha_corrigida). O ramo da interceptacao sai
        # ANTES da maquina de fases e deixa fase=None, e so a fase libera o
        # gatilho - entao ele nunca arma. Com a bola em contato isso se
        # realimenta: o robo empurra, a bola anda, a bola andando parece bola
        # viajando, ele volta a interceptar, empurra mais. Nada quebra o laco.
        #
        #   t0  contato a 1,92 s em (-784, 1058); o portador empurrou a bola
        #       3,2 m ATE A NOSSA PROPRIA META, (-4229, 189), e ficou 18 s
        #       parado a 950 mm dela. 3 investidas, nenhum chute.
        #   t4  contato a 0,65 s; manteve ~120 mm por ~10 s empurrando a bola
        #       de y=1847 a y=3193 - PARA FORA do campo (meia-largura 3000).
        #
        # Em contato quem decide e a maquina de fases: ela contorna, mira e
        # chuta. Interceptar so faz sentido com a bola LONGE e vindo.
        if rapida and not experimento.desligado("NAO_INTERCEPTAR_EM_CONTATO"):
            if hypot(bx - rx, by - ry) < RAIO_BOLA_NOSSA:
                rapida = False

        if rapida:
            avanco = aproximacao.AVANCO_SOLTA if solta else aproximacao.AVANCO_PORTADOR
            return self._fora_das_areas(
                ball, aproximacao.ponto_de_interceptacao(rx, ry, ball, avanco))

        direcao = a.alvo_chute if a.alvo_chute is not None else (
            bx + ux * 1000.0, by + uy * 1000.0)
        avanco = aproximacao.AVANCO_SOLTA if solta else aproximacao.AVANCO_PORTADOR

        if not experimento.desligado("PLANEJADOR"):
            fase = aproximacao.fase_de_aproximacao(rx, ry, bx, by, direcao, fase_anterior)
            if experimento.ligado("SO_POSICIONAR") and fase == EMPURRAR:
                fase = CONTORNAR
            fase = self._so_na_linha(robo, ball, direcao, fase, fase_anterior)
            fase = self._talvez_mirar(robo, ball, direcao, fase)
            m.fase = fase
            if fase == MIRAR:
                return aproximacao.ponto_de_espera(bx, by, direcao)
            if fase == DESENCOSTAR:
                return aproximacao.ponto_de_desencoste(rx, ry, bx, by)
            if fase == CONTORNAR:
                return aproximacao.ponto_de_espera(bx, by, direcao)

        # EMPURRAR. The long-destination variant lost the ball (measured 09/10).
        if experimento.ligado("COM_DESTINO_LONGO") and not experimento.desligado("PLANEJADOR"):
            return self._fora_das_areas(ball, aproximacao.destino_de_empurrao(bx, by, direcao))
        ponto = aproximacao.ponto_de_aproximacao(
            rx, ry, bx, by, direcao,
            self._avanco_antes_da_area(ball, direcao, avanco),
            contornar=not experimento.desligado("ORBITA"),
            empurrar=not experimento.desligado("EMPURRAO"))
        return self._fora_das_areas(ball, ponto)

    @staticmethod
    def _avanco_antes_da_area(ball, direcao, avanco):
        """Encurta o empurrao para que A BOLA nao saia de jogo.

        DEFEITO MEDIDO em 03/11/2026, nas 4 tentativas sem gol da campanha
        pareada de 20 pares (docs/campanha_gatilho). DUAS delas, e e o mesmo
        defeito nas duas:

            t9  a bola para em (2879, 427) depois de rolar livre - lugar
                legal, 1,6 m do gol, tiro livre. O portador empurra para o
                gol e a bola atravessa x=3380, a borda da caixa da area
                DELES. Bola final (3684, 52), DENTRO. O portador nao entra;
                fica 11 s parado a 320 mm dela. sem_chute, 14,9 s.
            t6  a bola para em (3091, -1383), fora da caixa pelo y. O
                empurrao para o gol entra na caixa 410 mm adiante, em
                (3380, -1100). Bola final (3445, -317), dentro. Mesmo fim.

        POR QUE '_fora_das_areas' NAO RESOLVIA: ela recorta o ALVO DO ROBO, e
        o robo para na borda - certo, e e justamente isso que garante que a
        bola siga sozinha para dentro e fique fora de alcance para sempre.
        Nada olhava para onde a BOLA ia parar.

        POR QUE ENCURTAR, e nao proibir a fase: trocar EMPURRAR por MIRAR
        manda o robo para ponto_de_espera, 300 mm atras da bola, e
        ALCANCE_CONTATO e 260 - ele nunca toca a bola e nunca arma. O
        empurrao E o passo que leva ao contato; o defeito e so o alvo 500 mm
        ATRAVES da bola. Com o avanco cortado ele ainda fecha os ~130 mm do
        contato e o gatilho dispara, mas nao tem para onde empurrar a bola
        depois.
        """
        if experimento.desligado("AVANCO_ANTES_DA_AREA"):
            return avanco
        bx, by = ball.position_x, ball.position_y
        ang = atan2(direcao[1] - by, direcao[0] - bx)
        raio = 9000.0
        fim = (bx + cos(ang) * raio, by + sin(ang) * raio)
        corte = campo.recortar_antes_das_areas(bx, by, fim[0], fim[1])
        # A BORDA DO CAMPO CONTA IGUAL. Bola fora e jogada morta, exatamente
        # como bola na area. MEDIDO em 03/11/2026, t4 do lote de 20 headless:
        # o portador manteve contato por ~10 s empurrando a bola de y=1847
        # para y=3193 - meia-largura do campo e 3000 - e a tentativa acabou
        # em 2,7 s de bola viva, sem chute. A versao anterior desta funcao
        # so olhava as caixas de area e deixou isso passar.
        borda = geometria.no_campo(corte[0], corte[1],
                                   LIMITE_BOLA_X, LIMITE_BOLA_Y)
        livre = hypot(borda[0] - bx, borda[1] - by)
        if livre >= avanco + FOLGA_BOLA_AREA:
            return avanco
        # PERTO DA CAIXA O AVANCO CAI A UM MINIMO, NAO A ZERO.
        #
        # Zero quebrava o invariante do chute, que esta escrito em
        # skills/chute.py (docstring de 'chutar_em'): o control so publica o
        # TeamCommand - que leva o kick - ENQUANTO o robo esta indo a algum
        # lugar. Robo que CHEGA ao alvo cala o canal, e o chute fica preso no
        # cache, armado, sem nunca ser enviado. Com avanco zero o alvo de
        # movimento e a PROPRIA BOLA, que e alcancavel: ele chega e o tiro
        # nunca sai.
        #
        # MEDIDO em 06/11/2026, pareado de 10 na faixa x 2500-3400
        # (docs/campanha_area10): nas duas falhas de 'sem_chute' o robo ficou
        # 19,6 s com toque_armado=1 e ZERO disparos, e a bola foi escoltada
        # para dentro da caixa nos dois bracos. E o mesmo caso que o docstring
        # do 'chutar_em' descreve ('arma=True' 34 a 110 vezes por lote, zero
        # disparos, robo a 92-96 mm da bola).
        #
        # AVANCO_MINIMO mantem o alvo ALEM da bola, entao o canal fica aberto e
        # o tiro sai; e e pequeno o bastante para a bola nao ser empurrada para
        # dentro da area antes disso. O chassi nao atravessa a bola, entao o
        # alvo segue inalcancavel - que e tudo que o invariante pede.
        if experimento.desligado("AVANCO_MINIMO_NA_AREA"):
            return 0.0
        return min(avanco, AVANCO_MINIMO_NA_AREA)

    @staticmethod
    def _fora_das_areas(ball, ponto):
        """A target in a penalty area, cut back along the line from the ball.

        The planner would move it to the area's nearest edge, off the line of fire:
        the robot then met the ball off-centre and pushed it into the area.
        """
        if experimento.desligado("RECORTE_AREA"):
            return ponto
        return campo.recortar_antes_das_areas(ball.position_x, ball.position_y,
                                              ponto[0], ponto[1])

    def _so_na_linha(self, robo, ball, direcao, fase, fase_anterior):
        """Push only from on the line of fire; until then keep going to the waiting point."""
        m = self.memoria
        if fase != EMPURRAR or experimento.desligado("CHEGADA_NA_LINHA"):
            m.ciclos_alinhando = 0
            return fase
        bx, by = ball.position_x, ball.position_y
        a = atan2(direcao[1] - by, direcao[0] - bx)
        dx, dy = robo.position_x - bx, robo.position_y - by
        lateral = abs(dx * sin(a) - dy * cos(a))
        if fase_anterior == EMPURRAR:
            return fase if lateral <= LATERAL_SAIDA else CONTORNAR
        if hypot(dx, dy) > RAIO_ENTRADA:
            m.ciclos_alinhando = 0
            return CONTORNAR
        if lateral <= LATERAL_ENTRADA:
            m.ciclos_alinhando = 0
            return EMPURRAR
        m.ciclos_alinhando += 1
        if m.ciclos_alinhando >= CICLOS_MAX_ALINHANDO:
            m.ciclos_alinhando = 0
            return EMPURRAR
        return CONTORNAR

    def _talvez_mirar(self, robo, ball, direcao, fase):
        """Stop and turn before pushing: it reached the ball before finishing the turn.

        The cap counts only ticks without turning, so a slow 180 deg turn is not cut.
        """
        m = self.memoria
        if fase != EMPURRAR or experimento.desligado("PARAR_E_MIRAR"):
            m.ciclos_mirando, m.erro_mira = 0, None
            return fase
        bx, by = ball.position_x, ball.position_y
        erro = abs(geometria.norm_ang(
            robo.orientation - atan2(direcao[1] - by, direcao[0] - bx)))
        d = hypot(robo.position_x - bx, robo.position_y - by)
        n = m.ciclos_mirando
        if m.erro_mira is not None and erro < m.erro_mira - GIRO_MIN_MIRA:
            n = 0
        if erro > TOL_CORPO_MIRA and d < RAIO_MIRA_PARADO and n < CICLOS_MAX_MIRANDO:
            m.ciclos_mirando, m.erro_mira = n + 1, erro
            return MIRAR
        m.ciclos_mirando, m.erro_mira = 0, None
        return fase

    def velocidade_chegada(self, a: AnaliseJogo, rid: int, x: float, y: float):
        vx = vy = 0.0
        if a.alvo_chute is None:
            return vx, vy
        bx, by = a.bola.position_x, a.bola.position_y
        if not experimento.desligado("CHEGADA_ALINHADA") and self.memoria.fase == CONTORNAR:
            cx, cy = bx - x, by - y
            n = hypot(cx, cy) or 1.0
            vx, vy = cx / n * VEL_CHEGADA_CONTORNO, cy / n * VEL_CHEGADA_CONTORNO
        if experimento.ligado("COM_CHEGADA_EM_MOVIMENTO") and self.memoria.fase == EMPURRAR:
            dx, dy = a.alvo_chute[0] - bx, a.alvo_chute[1] - by
            n = hypot(dx, dy) or 1.0
            vx, vy = dx / n * VEL_CHEGADA_EMPURRAO, dy / n * VEL_CHEGADA_EMPURRAO
        return vx, vy

    def evita_bola(self, a: AnaliseJogo, rid: int) -> bool:
        # The ball is an obstacle while circling and aiming, and only stops being
        # one to push through it. Not while stepping out: it is already inside it.
        if experimento.desligado("PLANEJADOR"):
            return False
        return self.memoria.fase in (CONTORNAR, MIRAR)

    def diagnosticar(self, a: AnaliseJogo, rid: int, cmd) -> None:
        robo = a.mundo.aliados[rid]
        if a.alvo_chute is not None:
            # Where the robot, its velocity and its target sit across the line of fire.
            bx, by = a.bola.position_x, a.bola.position_y
            ang = atan2(a.alvo_chute[1] - by, a.alvo_chute[0] - bx)
            nx, ny = -sin(ang), cos(ang)
            diagnostico.log(
                "[JG] LINHA r%d fase=%s lat=%.0f vlat=%.0f alvo_lat=%.0f ao_longo=%.0f "
                "corpo=%.1f" %
                (rid, self.memoria.fase,
                 (robo.position_x - bx) * nx + (robo.position_y - by) * ny,
                 (robo.velocity_x or 0.0) * nx + (robo.velocity_y or 0.0) * ny,
                 (cmd.target_x - bx) * nx + (cmd.target_y - by) * ny,
                 (robo.position_x - bx) * cos(ang) + (robo.position_y - by) * sin(ang),
                 math.degrees(geometria.norm_ang(robo.orientation - ang))))
        receptor = a.alvo.receptor[2] if a.alvo.receptor is not None else None
        diagnostico.log_e_arquivo(
            "[JG] MOV r%d fase=%s tipo=%s receptor=%s d_bola=%.0f "
            "alvo=%.0f,%.0f evita_bola=%s kick=%.1f" %
            (rid, self.memoria.fase, a.tipo_alvo, receptor,
             hypot(robo.position_x - a.bola.position_x, robo.position_y - a.bola.position_y),
             cmd.target_x, cmd.target_y, cmd.ball, cmd.kick))
