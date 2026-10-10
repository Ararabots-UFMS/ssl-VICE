#!/usr/bin/env python3
"""ararabots.py - toda a parte Python dos testes, num arquivo so.

Roda DENTRO do container 'vice' (menos 'resumo', que le os JSON no host).
Normalmente voce nao chama este arquivo direto: quem chama e o ararabots.sh.

    python3 ararabots.py listar                 nomes e titulos dos cenarios
    python3 ararabots.py posicionar <cenario>   monta o cenario no grSim
    python3 ararabots.py rodar <cenario> [s]    dispara a falta e grava
    python3 ararabots.py cadeia                 diagnostico dos topicos ROS
    python3 ararabots.py fps                    taxa real de quadros do grSim
    python3 ararabots.py pronto <elo> [s]       espera um elo ficar pronto
    python3 ararabots.py esperar [s]            espera a estrategia comandar
    python3 ararabots.py resumo [rotulo]        resume a dispersao das execucoes
    python3 ararabots.py atrito [v1 v2 ...]     mede o alcance da bola
    python3 ararabots.py decisao                confere a decisao da jogada

POR QUE 'posicionar' e 'rodar' sao separados: entre os dois o shell REINICIA os
nodes ROS. O filtro de Kalman leva dezenas de segundos para aceitar um teleporte
de 5 m (e faz bem: bola nenhuma pula assim), entao subimos tudo depois de a bola
ja estar no lugar - cada node nasce vendo o mundo na posicao do cenario.
"""
import base64
import glob
import json
import math
import os
import socket
import struct
import subprocess
import sys
import time

try:
    import rclpy
    from rclpy.node import Node
    from system_interfaces.msg import (
        VisionMessage, GameState, TeamCommand, RefereeMessage,
    )
except ImportError:
    # Fora do container so funcionam as ferramentas que nao usam ROS ('resumo').
    rclpy = None



GRSIM_HOST = "127.0.0.1"
GRSIM_PORT = 20011          # porta de comandos/replacement do grSim
GC_HOST = "127.0.0.1"
GC_PORT = 8081
SAIDA_DIR = "/tmp/cenarios_freekick"

# Segundos de HALT apos o teleporte, para o driver se reancorar.
ESPERA_HALT = 8.0

# Segundos de comando ao adversario ANTES do comando do arbitro, para o goleiro
# entrar na jogada ja em movimento em vez de partir do repouso.
PRE_VARREDURA = 5.0

# Gol: bola cruzando a linha com |y| dentro da largura do gol (Division B: 1 m).
GOL_X = 4500.0
GOL_MEIA_LARGURA = 500.0
# Tolerancia entre o relogio de captura da visao e o nosso, em segundos. Acima
# disto o pacote vem de OUTRO remetente no mesmo grupo multicast - ver a guarda
# em Gravador._visao_crua, e o estrago que ela evita.
TOL_RELOGIO = 2.0

# ===============================================================
# CONFIGURAÇÕES E FLUXOS DO ÁRBITRO
# ===============================================================
FLUXOS_ARBITRAGEM = {
    "freekick": ["STOP", "DIRECT_FREE_KICK_YELLOW"],
    "kickoff":  ["STOP", "PREPARE_KICKOFF_YELLOW", "NORMAL_START"]
}

# ==========================================================================
#  Cenarios
# ==========================================================================
# Coordenadas em MILIMETROS (mesma unidade da visao). A conversao para metros,
# que e o que o grSim espera no replacement, acontece em posicionar().
#
# Somos AZUIS e defendemos o gol de x negativo (on_positive_half=False),
# entao atacamos o lado positivo.
#
# O robo 0 e sempre o goleiro na logica do time.

# FUNCOES fixas em todos os cenarios (o codigo so trata o robo 0 como goleiro;
# as demais funcoes sao intencao do teste, para o relatorio ficar interpretavel):
#
#   NOSSOS (azul)          ADVERSARIOS (amarelo)
#   0 goleiro (nunca cobra)  0 goleiro
#   1 cobrador               1 barreira (>= 500 mm da bola, regra 5.3.3)
#   2 apoio / linha de passe 2 marcador
#   3 cobertura / recuo
# OS CENARIOS SAO AGRUPADOS POR TIPO.
#
# Cada entrada tem "tipo", e a interface (painel da GUI) pede primeiro o tipo e
# depois o cenario. Sem isso a lista tinha 33 nomes numa coluna so, misturando
# cobranca de falta com teste de orientacao.
#
#   bola_parada  cobranca de falta: a parte mais madura (5 gols em 6)
#   kickoff      inicio de partida, a favor e contra
#   jogo         jogo corrido
#   orientacao   o corpo nao vira as costas para a bola      (03/10/2026)
#   orbita       bola atras de nos: contorna e pega por tras (03/10/2026)
#   pressao      pressao medida na bola, nao no corpo        (03/10/2026)
#   protecao     saida sob pressao fugindo de quem prensa    (03/10/2026)
#   robustez     contagem de robos: casos limite que ja quebraram o no
#
# REMOVIDOS EM 03/10/2026, e por que:
#   limiar               testava o 'limiar de chute' (kick_threshold), que virou codigo morto: a decisao de chutar hoje e de alvo_do_chute + linha_livre
#   meio                 mesmo motivo: o criterio dele era 'o chute deve ficar DESATIVADO aquem do limiar', e esse limiar nao existe mais
#   regressao_cobranca   era copia exata de 'um_so_cobrador' (mesma bola, mesmos robos). A regressao da bola parada usa o original
#
# AO ACRESCENTAR UM CENARIO: ponha o "tipo" e descreva no documento
# docs/cenarios.md - que existe para isto e tem de ser atualizado junto.
CENARIOS = {
    "ataque": {        "tipo": "bola_parada",
        "titulo": "Nosso freekick no ataque",
        "descricao": (
            "Bola no terco de ataque, alem do limiar. Esperado: o cobrador (1) "
            "posiciona atras da bola, empurra rumo ao gol adversario e ATIVA o "
            "chute; o goleiro (0) permanece na meta; apoio (2) da linha de passe "
            "e cobertura (3) segura o contra-ataque."
        ),
        "bola": (2500.0, 0.0),
        "azuis": [(0, -4300, 0, 0), (1, 1900, 150, 0),
                  (2, 1700, 1400, 0)],
        "amarelos": [(0, 4300, 0, 180), (1, 3050, 0, 180), (2, 2900, 1100, 180)],
        "comando": ("DIRECT", "BLUE"),
    },
    "passe": {        "tipo": "bola_parada",
        "titulo": "Falta longe do gol: tem de sair PASSE, nao chute",
        "descricao": (
            "Bola no meio-campo, aquem do limiar de chute, com um companheiro "
            "ADIANTADO e com a linha livre de adversarios. O esperado e o "
            "cobrador tocar para ele com forca de passe (~2,5 m/s), e nao "
            "empurrar a bola sozinho ate a jogada expirar. Complementa o "
            "cenario 'meio', em que os companheiros estao ATRAS da bola e o "
            "certo e justamente NAO passar."
        ),
        "bola": (0.0, 0.0),
        "azuis": [(0, -4300, 0, 0), (1, -400, 0, 0), (2, 1600, 400, 0)],
        "amarelos": [(0, 4300, 0, 180)],
        "comando": ("DIRECT", "BLUE"),
    },
    "defesa": {        "tipo": "bola_parada",
        "titulo": "Nosso freekick no campo de defesa",
        "descricao": (
            "Bola perto do nosso gol. O cobrador deve afastar a bola do perigo "
            "com o chute DESATIVADO; o goleiro nao pode deixar a meta."
        ),
        "bola": (-2800.0, -600.0),
        "azuis": [(0, -4300, 0, 0), (1, -3200, -750, 0),
                  (2, -1800, 900, 0)],
        "amarelos": [(0, 4300, 0, 180), (1, -2250, -800, 180), (2, -1400, 200, 180)],
        "comando": ("DIRECT", "BLUE"),
    },
    "lateral": {        "tipo": "bola_parada",
        "titulo": "Bola junto a linha lateral",
        "descricao": (
            "Bola quase na lateral (y=2700 de 3000). Caso de borda: o ponto de "
            "aproximacao de _go_to_goal pode cair fora do campo."
        ),
        "bola": (2000.0, 2700.0),
        "azuis": [(0, -4300, 0, 0), (1, 1400, 2500, 0),
                  (2, 1600, 1100, 0)],
        "amarelos": [(0, 4300, 0, 180), (1, 2500, 2450, 180), (2, 2900, 1500, 180)],
        "comando": ("DIRECT", "BLUE"),
    },
    "canto": {        "tipo": "bola_parada",
        "titulo": "Bola no canto do campo de ataque",
        "descricao": (
            "Bola no canto ofensivo. A linha bola->gol fica bem inclinada e o "
            "ponto de aproximacao vai para perto da quina."
        ),
        "bola": (4100.0, 2700.0),
        "azuis": [(0, -4300, 0, 0), (1, 3550, 2500, 0),
                  (2, 3200, 1000, 0)],
        "amarelos": [(0, 4300, 0, 180), (1, 3950, 2050, 180), (2, 3900, 900, 180)],
        "comando": ("DIRECT", "BLUE"),
    },
    "deles_meio": {        "tipo": "bola_parada",
        "titulo": "Freekick DELES no meio-campo",
        "descricao": (
            "Cobranca amarela no centro, com cobrador a 600 mm da bola e um "
            "companheiro avancado. Esperado: nossos robos recuam para a linha "
            "entre a bola e o NOSSO gol formando bloqueio, e o goleiro fica na "
            "meta (classe TheirFreekick)."
        ),
        "bola": (0.0, 0.0),
        "azuis": [(0, -4300, 0, 0), (1, -900, 0, 180),
                  (2, -1300, 900, 180)],
        "amarelos": [(0, 4300, 0, 180), (1, 600, 0, 180), (2, 1400, 900, 180)],
        "comando": ("DIRECT", "YELLOW"),
    },
    "deles_perto_gol": {        "tipo": "bola_parada",
        "titulo": "Freekick DELES perto do nosso gol",
        "descricao": (
            "Perigo maximo: cobranca rente a nossa area. Esperado: bloqueio "
            "concentrado entre a bola e o gol, e o goleiro na linha."
        ),
        "bola": (-3200.0, 500.0),
        "azuis": [(0, -4300, 0, 0), (1, -3750, 450, 180),
                  (2, -3600, -300, 180)],
        "amarelos": [(0, 4300, 0, 180), (1, -2600, 500, 180), (2, -2400, -400, 180)],
        "comando": ("DIRECT", "YELLOW"),
    },

    # ---------------------------------------------------------------------
    #  Contagem de robos: casos de robustez.
    #
    #  A logica trata o robo 0 como goleiro (if 0 in self.ally_robots) e itera
    #  sobre o resto. Variar quem esta em campo exercita caminhos que os
    #  cenarios cheios nunca tocam - inclusive o de lista vazia.
    # ---------------------------------------------------------------------
    "dois_goleiro_e_cobrador": {        "tipo": "robustez",
        "titulo": "Dois em campo: goleiro + cobrador",
        "descricao": (
            "So o robo 0 (goleiro) e o robo 1 (cobrador). Esperado: goleiro na "
            "meta e o cobrador executando a falta sozinho, com chute ativo por "
            "estar alem do limiar. Se o goleiro sair da meta aqui, o problema e "
            "do alvo dele, nao de disputa com companheiros."
        ),
        "bola": (2500.0, 0.0),
        "azuis": [(0, -4300, 0, 0), (1, 1900, 150, 0)],
        "amarelos": [(0, 4300, 0, 180), (1, 3050, 0, 180)],
        "comando": ("DIRECT", "BLUE"),
    },
    "dois_sem_goleiro": {        "tipo": "robustez",
        "titulo": "Dois em campo, SEM goleiro",
        "descricao": (
            "Robos 1 e 2, sem o robo 0. Testa o caminho em que "
            "'if 0 in self.ally_robots' e falso: nenhum comando de goleiro deve "
            "ser gerado, e os dois de linha devem tratar a bola. Revela se os "
            "dois recebem o MESMO alvo e disputam a mesma posicao."
        ),
        "bola": (2500.0, 0.0),
        "azuis": [(1, 1900, 150, 0), (2, 1700, 1400, 0)],
        "amarelos": [(0, 4300, 0, 180), (1, 3050, 0, 180)],
        "comando": ("DIRECT", "BLUE"),
    },
    "um_so_cobrador": {        "tipo": "bola_parada",
        "titulo": "Um em campo: apenas o cobrador",
        "descricao": (
            "Somente o robo 1. Sem goleiro e sem apoio: e o caso minimo em que a "
            "cobranca ainda deve acontecer. Se falhar aqui, o problema esta na "
            "propria execucao da falta, nao no time."
        ),
        "bola": (2500.0, 0.0),
        "azuis": [(1, 1900, 150, 0)],
        "amarelos": [(0, 4300, 0, 180)],
        "comando": ("DIRECT", "BLUE"),
    },
    # ------------------------------------------------------------------
    #  BURACO DE COBERTURA que ficou visivel na rev. 19.
    #
    #  Os 14 cenarios anteriores colocam o cobrador SEMPRE ATRAS da bola, entao
    #  a fase de 'contornar' nunca era exercitada a partir da pior geometria. Ela
    #  e a unica que depende da bola ser OBSTACULO - e o caminho novo perdeu o
    #  avoid_ball (o MovementManager nunca o define e o MovementCommand nao tem
    #  campo por robo).
    #
    #  Sem este cenario, "o resultado esta positivo" nao diz nada sobre essa
    #  perda: nao ha teste que a toque.
    # ------------------------------------------------------------------
    "cobrador_na_frente": {        "tipo": "bola_parada",
        "titulo": "Cobrador do LADO ERRADO: exige contornar a bola",
        "descricao": (
            "O cobrador nasce ENTRE a bola e o gol adversario, que e a pior "
            "geometria possivel: para chutar ele precisa dar a volta sem passar "
            "por cima da bola. E o unico cenario que exercita a fase de "
            "contorno desde o inicio, e portanto o unico que mede a perda do "
            "avoid_ball na movimentacao nova. Esperado: ele contorna por fora e "
            "chuta; se empurrar a bola para o nosso campo durante a volta, a "
            "perda do obstaculo e grave."
        ),
        "bola": (2500.0, 0.0),
        "azuis": [(1, 3100.0, 0.0, 3.14)],
        "amarelos": [(0, 4300, 0, 180)],
        "comando": ("DIRECT", "BLUE"),
    },

    # ------------------------------------------------------------------
    #  JOGO CORRIDO - o unico cenario que NAO e uma bola parada.
    #
    #  Todos os outros terminam num DIRECT/KICKOFF e medem uma cobranca. Este
    #  usa FORCE_START: a arvore cai em NormalStart (plays/running.py) e o time
    #  joga. Serve para VER COMPORTAMENTO, nao para medir cobranca - nao ha
    #  criterio de sucesso aqui, so observacao.
    #
    #  Os dois times completos, em formacao de saida, com a bola no centro.
    # ------------------------------------------------------------------
    "jogo": {        "tipo": "jogo",
        "titulo": "Jogo corrido: os dois times, bola ao centro",
        "descricao": (
            "FORCE_START com os dois times em campo. A arvore cai em "
            "NormalStart e o time joga livremente. Serve para observar "
            "comportamento - posicionamento, disputa, o goleiro na meta - e "
            "nao para medir uma cobranca. Sem criterio de gol: o que interessa "
            "e o replay."
        ),
        "bola": (0.0, 0.0),
        # QUATRO por time (goleiro + TRES de linha).
        #
        # Eram tres, o que deixa so DOIS de linha - e com dois nao ha papeis:
        # um e o eleito que vai a bola, o outro faz tudo o mais. Medimos que
        # dedicar esse segundo ao apoio ofensivo derruba o chute de 5525 para
        # 1536 mm/s, porque o eleito fica sozinho.
        #
        # O grSim so cria os robos que o "Robots Count" do ~/.grsim.xml permite,
        # e o preparar o forcava em 3. Agora e ajustavel:
        #     ARARABOTS_ROBOS=4 ./ararabots.sh ...
        # Este cenario PRECISA de 4; os de bola parada cabem em 3 e continuam
        # rodando com o padrao (cada robo a mais custa FPS e CPU do controle).
        "azuis": [(0, -4300.0, 0.0, 0.0),
                  (1, -1200.0, 0.0, 0.0),
                  (2, -2200.0, 1200.0, 0.0),
                  (3, -2200.0, -1200.0, 0.0)],
        "amarelos": [(0, 4300, 0, 180),
                     (1, 1200, 0, 180),
                     (2, 2200, -1200, 180),
                     (3, 2200, 1200, 180)],
        "comando": ("FORCE_START", "BLUE"),
    },

    "um_so_goleiro": {        "tipo": "robustez",
        "titulo": "Um em campo: apenas o goleiro",
        "descricao": (
            "Somente o robo 0. Nao ha quem cobre a falta. Esperado: um unico "
            "comando, mandando o goleiro para a meta, e nenhuma tentativa de "
            "chute. Serve para confirmar que o goleiro NAO assume a cobranca."
        ),
        "bola": (2500.0, 0.0),
        "azuis": [(0, -4300, 0, 0)],
        "amarelos": [(0, 4300, 0, 180)],
        "comando": ("DIRECT", "BLUE"),
    },
    "campo_vazio": {        "tipo": "robustez",
        "titulo": "Ninguem em campo do nosso time",
        "descricao": (
            "Nenhum robo aliado. Caso limite: com ally_robots vazio a acao "
            "devolve TaskStatus.RUNNING e nada e publicado. Serve para verificar "
            "que o sistema nao quebra nem gera comando para robo inexistente."
        ),
        "bola": (2500.0, 0.0),
        "azuis": [],
        "amarelos": [(0, 4300, 0, 180), (1, 3050, 0, 180)],
        "comando": ("DIRECT", "BLUE"),
    },
    # ======================================================================
    #  CENARIOS DAS MODIFICACOES DE 03/10/2026  (tres por modificacao)
    # ======================================================================
    #
    # COMO USAR: cada trio mede UMA modificacao. Rode o lote com a chave LIGADA
    # (= modificacao desligada, linha de base) e depois sem ela:
    #
    #   ARARABOTS_SEM_ORBITA=1 ./ararabots.sh validar 3 orbita_frontal   (antes)
    #                          ./ararabots.sh validar 3 orbita_frontal   (depois)
    #
    # As demais modificacoes ficam no estado NOVO nas duas rodadas - e o que faz
    # o delta medir so a que esta sendo testada. Ver skills/experimento.py.
    #
    # ISOLAMENTO: 'orientacao_*' poe o robo na faixa de 600 a 700 mm da bola, em
    # que a regra de orientacao vale e a orbita (que exige d < RAIO_ENCAIXE=600)
    # NAO dispara. Nos cenarios de orbita o robo entra a menos de 600 mm, e a
    # orientacao muda junto - as duas tratam a mesma condicao ("lado errado"), e
    # separa-las por geometria nao da; o que separa e a METRICA de cada lote.

    # --- 1. ORIENTACAO: o corpo nao vira as costas para a bola ---------------
    "orientacao_meio": {        "tipo": "orientacao",
        "titulo": "Lado errado a 650 mm, bola no meio: o corpo olha a bola?",
        "descricao": (
            "O robo 1 esta ENTRE a bola e o gol adversario, a 650 mm dela - "
            "dentro do raio de orientacao (700) e fora do raio da orbita (600). "
            "Com a chave LIGADA ele recebe ordem de apontar para o gol, de "
            "costas para a bola. Sem a chave, olha para a bola e contorna."
        ),
        "bola": (0.0, 0.0),
        "azuis": [(0, -4300, 0, 0), (1, 650, 0, 0),
                  (2, 2000, 1500, 0), (3, -1500, -500, 0)],
        "amarelos": [(0, 4400, 1500, 180), (1, 2500, -800, 180)],
        "comando": ("FORCE_START", "BLUE"),
    },
    "orientacao_lateral": {        "tipo": "orientacao",
        "titulo": "Lado errado a 650 mm, bola na lateral",
        "descricao": (
            "Mesma geometria com a bola fora do eixo: o alvo do chute muda de "
            "direcao e a ordem antiga gira o corpo junto, sem olhar onde o robo "
            "esta."
        ),
        "bola": (800.0, 1900.0),
        "azuis": [(0, -4300, 0, 0), (1, 1430, 1950, 0),
                  (2, 2200, 300, 0), (3, -1000, 900, 0)],
        "amarelos": [(0, 4400, 0, 180), (1, 3000, 1500, 180)],
        "comando": ("FORCE_START", "BLUE"),
    },
    "orientacao_terco": {        "tipo": "orientacao",
        "titulo": "Lado errado a 650 mm, bola no nosso terco",
        "descricao": (
            "No nosso terco a saida de bola escolhe a direcao, entao a ordem "
            "antiga pode apontar o corpo para a lateral com a bola atras. E o "
            "caso mais perigoso: de costas para a bola na frente da nossa area."
        ),
        "bola": (-2900.0, 400.0),
        "azuis": [(0, -4300, 0, 0), (1, -2260, 450, 0),
                  (2, -800, 1200, 0), (3, -1800, -900, 0)],
        "amarelos": [(0, 4400, 0, 180), (1, -1900, 900, 180)],
        "comando": ("FORCE_START", "BLUE"),
    },

    # --- 2. ORBITA: bola atras de nos -> contorna e pega por tras -----------
    "orbita_frontal": {        "tipo": "orbita",
        "titulo": "Bola ATRAS do robo, na linha de tiro: contorna?",
        "descricao": (
            "O robo 1 esta a 260 mm da bola, exatamente entre ela e o gol "
            "adversario - o pior caso medido (175 graus de erro). Com a chave "
            "LIGADA ele empurra a bola para o nosso campo; sem ela, orbita e "
            "pega por tras."
        ),
        "bola": (1500.0, 0.0),
        "azuis": [(0, -4300, 0, 0), (1, 1760, 0, 0),
                  (2, 2800, 1400, 0), (3, 0, -900, 0)],
        "amarelos": [(0, 4400, 1500, 180)],
        "comando": ("FORCE_START", "BLUE"),
    },
    "orbita_diagonal": {        "tipo": "orbita",
        "titulo": "Bola atras na diagonal, 400 mm",
        "descricao": (
            "Lado errado por 135 graus, nao 180: o contorno tem de escolher o "
            "sentido mais curto do arco. Com a chave LIGADA o alvo perto puxa "
            "para a linha de tiro atravessando a bola."
        ),
        "bola": (500.0, -600.0),
        "azuis": [(0, -4300, 0, 0), (1, 783, -317, 0),
                  (2, 2400, 800, 0), (3, -1200, -1200, 0)],
        "amarelos": [(0, 4400, 1200, 180), (1, 2600, -1600, 180)],
        "comando": ("FORCE_START", "BLUE"),
    },
    "orbita_colado": {        "tipo": "orbita",
        "titulo": "Bola atras e COLADO (150 mm): da a volta sem empurrar?",
        "descricao": (
            "A 150 mm o robo ja esta praticamente em contato. O risco e ele "
            "empurrar a bola para tras enquanto decide. O arco tem raio 260, "
            "entao o primeiro alvo o afasta da bola antes de contornar."
        ),
        "bola": (2200.0, 300.0),
        "azuis": [(0, -4300, 0, 0), (1, 2350, 300, 0),
                  (2, 1000, -800, 0), (3, -500, 1000, 0)],
        "amarelos": [(0, 4400, -1200, 180)],
        "comando": ("FORCE_START", "BLUE"),
    },

    # --- 3. PRESSAO MEDIDA NA BOLA ------------------------------------------
    #
    # Em todos os tres: os adversarios estao a MENOS de 400 mm da BOLA e a MAIS
    # de 400 mm do nosso portador. Com a chave LIGADA (conta so o corpo do robo)
    # o alivio NAO dispara e 'alvo_chute' fica bloqueado; sem a chave, dispara.
    "pressao_na_bola": {        "tipo": "pressao",
        "titulo": "Dois prensando a BOLA, longe do nosso corpo",
        "descricao": (
            "Portador atras da bola (a posicao certa para empurrar); os dois "
            "amarelos a ~310 e ~350 mm da BOLA e a ~460 mm do robo. Com a chave "
            "LIGADA, zero adversarios 'perto' e o portador posiciona para sempre."
        ),
        "bola": (1200.0, 0.0),
        "azuis": [(0, -4300, 0, 0), (1, 1000, 0, 0),
                  (2, 2600, 1500, 0), (3, -900, -900, 0)],
        "amarelos": [(0, 4400, 0, 180), (1, 1500, 60, 180), (2, 1450, -260, 180)],
        "comando": ("FORCE_START", "BLUE"),
    },
    "pressao_na_bola_lateral": {        "tipo": "pressao",
        "titulo": "Prensa na bola junto a lateral",
        "descricao": (
            "Mesma assimetria com a bola perto da linha: a saida tem menos "
            "opcoes, o que torna o disparo do alivio mais decisivo."
        ),
        "bola": (600.0, 2300.0),
        "azuis": [(0, -4300, 0, 0), (1, 380, 2300, 0),
                  (2, 2400, 900, 0), (3, -800, 1200, 0)],
        "amarelos": [(0, 4400, 0, 180), (1, 900, 2380, 180), (2, 850, 2050, 180)],
        "comando": ("FORCE_START", "BLUE"),
    },
    "pressao_na_bola_terco": {        "tipo": "pressao",
        "titulo": "Prensa na bola no nosso terco",
        "descricao": (
            "Aqui a SAIDA DE BOLA tem precedencia sobre o alivio, entao o "
            "esperado e que a chave mude pouco - serve de controle negativo do "
            "trio. Se mudar muito, a precedencia esta errada."
        ),
        "bola": (-2600.0, 300.0),
        "azuis": [(0, -4300, 0, 0), (1, -2820, 300, 0),
                  (2, 0, 1200, 0), (3, -1200, -800, 0)],
        "amarelos": [(0, 4400, 0, 180), (1, -2350, 420, 180), (2, -2300, 80, 180)],
        "comando": ("FORCE_START", "BLUE"),
    },

    # --- 4. PROTECAO DE POSSE (varredura fugindo de quem prensa) ------------
    #
    # Nos tres o alivio DISPARA nas duas rodadas (a pressao e sentida tambem no
    # corpo), entao o que a chave muda e so a DIRECAO escolhida: lateral fixa
    # contra varredura que foge da ameaca.
    "protecao_frontal": {        "tipo": "protecao",
        "titulo": "Prensa colada: o corpo fica entre o adversario e a bola?",
        "descricao": (
            "Adversario a 240 mm do portador E da bola: o alivio dispara nas "
            "duas rodadas. Com a chave LIGADA a bola vai para a lateral fixa; "
            "sem ela, para o lado oposto a quem prensa, com o casco no meio."
        ),
        "bola": (1000.0, 0.0),
        "azuis": [(0, -4300, 0, 0), (1, 880, 0, 0),
                  (2, 2600, 1400, 0), (3, -600, -1000, 0)],
        "amarelos": [(0, 4400, 0, 180), (1, 1150, 180, 180), (2, 1200, -200, 180)],
        "comando": ("FORCE_START", "BLUE"),
    },
    "protecao_dois_lados": {        "tipo": "protecao",
        "titulo": "Prensado por dois lados: sobra direcao boa?",
        "descricao": (
            "Dois adversarios em flancos opostos. A varredura tem de achar a "
            "unica direcao com folga; a lateral fixa pode jogar em cima de um "
            "deles."
        ),
        "bola": (300.0, 800.0),
        "azuis": [(0, -4300, 0, 0), (1, 120, 800, 0),
                  (2, 2200, -400, 0), (3, -1000, 1600, 0)],
        "amarelos": [(0, 4400, 0, 180), (1, 480, 1060, 180), (2, 520, 540, 180)],
        "comando": ("FORCE_START", "BLUE"),
    },
    "protecao_contra_tres": {        "tipo": "protecao",
        "titulo": "Tres em cima: a direcao de maior folga ainda existe?",
        "descricao": (
            "O caso medido em jogo ('eles chegam com tres e nos com um'). "
            "Nenhuma direcao esta limpa - a varredura escolhe a MENOS pior, que "
            "e o principio da saida de bola. Mede se isso vale a pena."
        ),
        "bola": (1800.0, -400.0),
        "azuis": [(0, -4300, 0, 0), (1, 1650, -400, 0),
                  (2, 2600, 1200, 0), (3, 200, -1400, 0)],
        "amarelos": [(0, 4400, 0, 180), (1, 2050, -200, 180),
                     (2, 2000, -700, 180), (3, 1750, -50, 180)],
        "comando": ("FORCE_START", "BLUE"),
    },

    # ======================================================================
    #  O PORTADOR SOZINHO: so ele e a bola em campo  (07/10/2026)
    # ======================================================================
    #
    # PEDIDO DO FELIPE: "teste so o portador, sozinho, com so ele e a bola em
    # campo, com a bola em diversas posicoes diferentes, para testar a acao dele
    # e principalmente se ele faz o contorno".
    #
    # POR QUE ISTO MEDE MELHOR QUE OS CENARIOS DE TIME. Nenhum amarelo em campo
    # (os robos que o cenario nao lista ficam atras da linha de fundo e
    # DESLIGADOS, fora da visao) e nenhum companheiro. Consequencias, todas
    # desejadas aqui:
    #
    #   - 'linha_livre' ate o gol esta sempre limpa, entao a mira e sempre
    #     'gol'. A mira nao pisca, e foi o piscar dela que escondeu as quatro
    #     modificacoes de 03/10 (ver mira_firme);
    #   - sem apoio nao ha passe, entao a direcao de saida e sempre bola->gol;
    #   - sem adversario nao ha pressao, alivio nem disputa.
    #
    # Sobra UMA coisa sendo medida: o portador chegando na bola. E como a
    # direcao de saida e fixa, o angulo da largada e a unica variavel.
    #
    # O QUE VARIA: o angulo entre "de onde eu venho" e "para onde a bola tem de
    # ir". O robo fica em (0,0) e a bola se move em volta dele.
    #
    #     0 graus     bola a frente, na linha do gol      nao precisa contornar
    #     90 graus    bola ao lado (os dois lados)        contorna um quarto
    #     135 graus   bola na diagonal de tras            contorna, lado curto
    #     180 graus   bola atras, na linha de tiro        contorna meia volta
    #
    # E a DISTANCIA, porque o contorno so existe dentro de RAIO_ENCAIXE (600 mm):
    # a 150 e 400 mm o ramo da orbita dispara; a 1200 mm, nao - ali quem resolve
    # e o recuo da aproximacao, e o cenario existe para mostrar a diferenca.
    #
    # METRICA DO TRIO: 'recuo' - o quanto a bola chegou a andar PARA O NOSSO
    # CAMPO em algum momento da execucao. E o defeito que o contorno existe para
    # evitar, e e o unico numero que distingue "contornou" de "empurrou para
    # tras e depois corrigiu". 'avanco' sozinho nao distingue.
    "portador_alinhado": {        "tipo": "portador",
        "titulo": "Bola a frente, alinhada com o gol: o caso facil",
        "descricao": (
            "Zero graus: o robo ja esta atras da bola, na linha do gol, e nao "
            "ha nada a contornar. E a referencia do trio - se a bola nao andar "
            "AQUI, o problema nao e o contorno. Esperado: empurra e chuta."
        ),
        "bola": (700.0, 0.0),
        "azuis": [(1, 0, 0, 0)],
        "amarelos": [],
        "comando": ("FORCE_START", "BLUE"),
    },
    "portador_bola_atras_400": {        "tipo": "portador",
        "titulo": "Bola ATRAS a 400 mm: contorna meia volta?",
        "descricao": (
            "180 graus, dentro do raio do contorno (600). O robo esta entre a "
            "bola e o gol: ir reto nela empurra para o NOSSO campo, porque sem "
            "dribbler a bola sai na direcao robo->bola. Esperado: orbita ate o "
            "lado certo e so depois empurra, com recuo perto de zero."
        ),
        "bola": (-400.0, 0.0),
        "azuis": [(1, 0, 0, 0)],
        "amarelos": [],
        "comando": ("FORCE_START", "BLUE"),
    },
    "portador_bola_atras_150": {        "tipo": "portador",
        "titulo": "Bola atras e COLADA (150 mm): da a volta sem empurrar?",
        "descricao": (
            "180 graus praticamente em contato - o casco tem 90 mm e a bola 21, "
            "entao 111 mm ja e toque. O risco e empurrar enquanto decide. O arco "
            "do contorno tem raio 260, logo o primeiro alvo AFASTA o robo da "
            "bola antes de girar. Pior caso do trio."
        ),
        "bola": (-150.0, 0.0),
        "azuis": [(1, 0, 0, 0)],
        "amarelos": [],
        "comando": ("FORCE_START", "BLUE"),
    },
    "portador_bola_lado_esq": {        "tipo": "portador",
        "titulo": "Bola 90 graus a esquerda: um quarto de volta",
        "descricao": (
            "A bola esta ao lado, nao atras: o erro e de 90 graus, no limite em "
            "que o ramo do contorno liga (ele exige mais de 90). Mede a "
            "fronteira - e com ela o risco de ficar oscilando em cima do "
            "limite, que e um defeito que esta aproximacao ja teve."
        ),
        "bola": (0.0, 400.0),
        "azuis": [(1, 0, 0, 0)],
        "amarelos": [],
        "comando": ("FORCE_START", "BLUE"),
    },
    "portador_bola_lado_dir": {        "tipo": "portador",
        "titulo": "Bola 90 graus a direita: o arco escolhe o lado curto?",
        "descricao": (
            "Espelho do anterior. Os dois juntos mostram se o contorno escolhe o "
            "sentido MAIS CURTO do arco ou se tem um lado preferido - um erro de "
            "sinal aqui so aparece comparando os dois."
        ),
        "bola": (0.0, -400.0),
        "azuis": [(1, 0, 0, 0)],
        "amarelos": [],
        "comando": ("FORCE_START", "BLUE"),
    },
    "portador_bola_diagonal": {        "tipo": "portador",
        "titulo": "Bola 135 graus atras: contorno parcial",
        "descricao": (
            "Lado errado por 135 graus, nao 180: o contorno tem de girar menos "
            "de meia volta e entregar o robo alinhado. E o caso mais comum em "
            "jogo - 180 exato quase nunca acontece."
        ),
        "bola": (-300.0, 300.0),
        "azuis": [(1, 0, 0, 0)],
        "amarelos": [],
        "comando": ("FORCE_START", "BLUE"),
    },
    "portador_bola_atras_longe": {        "tipo": "portador",
        "titulo": "Bola atras a 1200 mm: FORA do raio do contorno",
        "descricao": (
            "180 graus, mas a 1200 mm - o dobro do RAIO_ENCAIXE (600), onde o "
            "ramo da orbita NAO dispara. Quem tem de resolver aqui e o recuo "
            "continuo da aproximacao, mirando um ponto atras da bola. Se este "
            "passar e os de 400 e 150 falharem, o defeito esta no contorno; se "
            "este falhar tambem, esta na aproximacao de longe."
        ),
        "bola": (-1200.0, 0.0),
        "azuis": [(1, 0, 0, 0)],
        "amarelos": [],
        "comando": ("FORCE_START", "BLUE"),
    },
    # --- CONTORNO LONGO: a bola a metros de distancia, do lado errado -------
    #
    # PEDIDO DO FELIPE (09/10/2026): "quero cenarios onde ele esteja longe para
    # vermos o contorno longo".
    #
    # POR QUE ELES MEDEM OUTRA COISA. Nos oito de cima o robo nasce a 150-1200
    # mm da bola: a decisao de contornar e quase imediata e o que se ve e a
    # chegada. Aqui ele nasce a 3-7 m, entao o caminho inteiro e o teste - e e
    # onde se ve se o desvio e UM arco so, planejado, ou uma sequencia de
    # correcoes. Com o contorno a cargo do planejador (avoid_ball), a volta e um
    # caminho continuo; com o arco feito a mao, o alvo pulava 0,9 rad por ciclo
    # e o robo perseguia uma cenoura - a distancia isso aparece como zigue-zague.
    "portador_longe_atras": {        "tipo": "portador",
        "titulo": "Bola a 4 m, atras do robo: o contorno longo",
        "descricao": (
            "180 graus a 4 m. O robo esta entre a bola e o gol, com o campo "
            "todo para resolver: ele tem de chegar ATRAS dela, e o caminho "
            "inteiro e a medida. Se ele for reto, encosta pelo lado errado e "
            "empurra a bola para o nosso campo."
        ),
        "bola": (-1500.0, 0.0),
        "azuis": [(1, 2500, 0, 0)],
        "amarelos": [],
        "comando": ("FORCE_START", "BLUE"),
    },
    "portador_longe_diagonal": {        "tipo": "portador",
        "titulo": "Bola a 4,5 m na diagonal: contorno longo com angulo",
        "descricao": (
            "A bola esta longe E fora do eixo, entao a linha de tiro nao e a "
            "linha de chegada. O ponto de espera fica atras da bola na direcao "
            "do gol, e o caminho tem de terminar alinhado com ela - nao apenas "
            "perto."
        ),
        "bola": (-1200.0, 1600.0),
        "azuis": [(1, 2600, -1200, 0)],
        "amarelos": [],
        "comando": ("FORCE_START", "BLUE"),
    },
    "portador_longe_lado": {        "tipo": "portador",
        "titulo": "Bola a 3,5 m, 90 graus: contorna ou corta?",
        "descricao": (
            "A 90 graus e longe, a tentacao e cortar reto e chegar pela "
            "lateral. Com a bola como obstaculo do planejador, o caminho passa "
            "por tras; sem ela, passa por cima - e a bola sai de raspao."
        ),
        "bola": (0.0, 0.0),
        "azuis": [(1, 0, -3500, 0)],
        "amarelos": [],
        "comando": ("FORCE_START", "BLUE"),
    },
    "portador_atravessa_campo": {        "tipo": "portador",
        "titulo": "Bola no terco de ataque, robo no nosso: 7 m de corrida",
        "descricao": (
            "A corrida mais longa que o campo permite, com a bola do lado certo "
            "no fim. Mede a coisa mais basica e que nunca foi medida sozinha: o "
            "robo atravessa o campo e chega em condicao de chutar, ou chega "
            "freando e de lado?"
        ),
        "bola": (3000.0, 400.0),
        "azuis": [(1, -3600, -600, 0)],
        "amarelos": [],
        "comando": ("FORCE_START", "BLUE"),
    },

    "portador_bola_no_terco": {        "tipo": "portador",
        "titulo": "Bola atras, no nosso terco: contorna ou tira na lateral?",
        "descricao": (
            "Mesma geometria de 180 graus, mas com a bola no nosso terco "
            "defensivo, onde a saida de bola escolhe a direcao em vez do gol. "
            "Aqui o contorno e a saida de bola se encontram: o alvo nao e mais "
            "'o gol deles', e o lado certo de chegada muda com ele."
        ),
        "bola": (-3100.0, 300.0),
        "azuis": [(1, -2600, 300, 0)],
        "amarelos": [],
        "comando": ("FORCE_START", "BLUE"),
    },

    # ======================================================================
    #  CENARIOS DAS MODIFICACOES DE 07/10/2026  (tres por modificacao)
    # ======================================================================
    #
    # As duas nasceram da LEITURA DOS REPLAYS do lote de 03/10, nao de hipotese:
    # os quatro trios anteriores nao mostraram ganho de RESULTADO porque a bola
    # nao saia do lugar em NENHUMA das condicoes, e a sonda guiada por replay
    # mostrou por que. Ver documentacao/estrategia/RESULTADOS-07-10.md.

    # --- 6. MIRA FIRME: a mira vale 2 s mesmo contra 'bloqueado' ------------
    #
    # O QUE OS TRES PRECISAM PRODUZIR: o tipo de mira ALTERNANDO. Com a chave
    # LIGADA (trava antiga, que so valia entre dois alvos validos) a alternancia
    # passa livre e a linha de tiro gira; sem a chave, a mira escolhida segura.
    # Medido em 'orientacao_terco': 65 trocas de mira em 245 ciclos -> 8.
    #
    # Como se produz a alternancia com adversario PARADO: o que se move e o
    # NOSSO apoio. O amarelo fica a pouco mais de FOLGA_LINHA (180 mm) da reta
    # bola->apoio, entao a linha abre e fecha conforme o apoio anda.
    "mira_gol_fechado": {        "tipo": "mira",
        "titulo": "Gol fechado e passe piscando: a mira para de girar?",
        "descricao": (
            "O gol esta tapado por um amarelo na linha, entao 'gol' nunca "
            "vale. O apoio (robo 2) esta a frente e um amarelo fica rente a "
            "reta bola->apoio: o tipo alterna passe <-> bloqueado a cada poucos "
            "ciclos. Como TODA direcao do ciclo sai da mira, o portador orbita "
            "a bola sem encostar nela. Metrica: trocas de mira e distancia "
            "minima do portador a bola."
        ),
        "bola": (0.0, 0.0),
        "azuis": [(0, -4300, 0, 0), (1, -700, 0, 0),
                  (2, 1700, 900, 0), (3, -1600, -800, 0)],
        "amarelos": [(0, 4400, 0, 180), (1, 2200, 0, 180), (2, 950, 700, 180)],
        "comando": ("FORCE_START", "BLUE"),
    },
    "mira_apoio_que_entra": {        "tipo": "mira",
        "titulo": "Apoio na fronteira do avanco minimo: a mira aguenta?",
        "descricao": (
            "O apoio comeca a 620 mm a frente da bola, rente ao limite de "
            "AVANCO_MINIMO_PASSE (600). Qualquer recuo dele invalida o passe e "
            "o tipo cai para 'bloqueado'; qualquer avanco o revalida. E a "
            "fronteira que mais pisca, e a trava antiga nao a cobria."
        ),
        "bola": (-400.0, 600.0),
        "azuis": [(0, -4300, 0, 0), (1, -1100, 600, 0),
                  (2, 220, 1100, 0), (3, -2000, -400, 0)],
        "amarelos": [(0, 4400, 0, 180), (1, 1800, 400, 180)],
        "comando": ("FORCE_START", "BLUE"),
    },
    "mira_dois_apoios": {        "tipo": "mira",
        "titulo": "Dois candidatos a receber: o alvo do passe fica quieto?",
        "descricao": (
            "Dois aliados a frente, os dois elegiveis. O papel de apoio tem "
            "histerese, mas o ALVO do passe seguia o apoio da vez - e o "
            "congelamento era descartado a cada ciclo em que a linha fechava. "
            "Com o gol tapado, o portador tem de escolher um e manter."
        ),
        "bola": (600.0, -200.0),
        "azuis": [(0, -4300, 0, 0), (1, -100, -200, 0),
                  (2, 2100, 700, 0), (3, 2000, -1100, 0)],
        "amarelos": [(0, 4400, 0, 180), (1, 2600, -200, 180),
                     (2, 1400, 300, 180)],
        "comando": ("FORCE_START", "BLUE"),
    },

    # --- 7. EMPURRAO: no contato, o alvo fica ALEM da bola ------------------
    #
    # O QUE OS TRES PRECISAM PRODUZIR: o portador CHEGANDO ALINHADO na bola com
    # espaco a frente. Com a chave LIGADA o alvo fica 53 mm AQUEM da bola, que
    # e inalcancavel (casco 90 + bola 21 = 111 mm): sobra erro de ~58 mm, ou
    # 0,13 m/s de comando, e a bola nao sai do lugar. Sem a chave o alvo fica
    # 180 mm ALEM e o comando vira ~0,67 m/s.
    #
    # Metrica do trio: AVANCO DA BOLA no eixo de ataque (nao o modulo do
    # deslocamento - foi isso que escondeu o resultado do lote de 03/10) e
    # velocidade maxima da bola.
    "empurrao_reto": {        "tipo": "empurrao",
        "titulo": "Atras da bola, alinhado, gol livre: a bola anda?",
        "descricao": (
            "A geometria mais simples que existe: portador 400 mm atras da "
            "bola, na linha do gol, campo livre a frente. Se a bola nao andar "
            "aqui, nao anda em lugar nenhum."
        ),
        "bola": (0.0, 0.0),
        "azuis": [(0, -4300, 0, 0), (1, -400, 0, 0),
                  (2, 1800, 1200, 0), (3, -1500, -900, 0)],
        "amarelos": [(0, 4400, 1200, 180)],
        "comando": ("FORCE_START", "BLUE"),
    },
    "empurrao_terco": {        "tipo": "empurrao",
        "titulo": "Saida do nosso terco: a bola sai da area?",
        "descricao": (
            "Bola no nosso terco com o gol deles fechado - a direcao vem do "
            "afastamento, nao do gol. Aqui o empurrao e o que tira a bola de "
            "casa: medimos quantos milimetros ela avanca no eixo de ataque."
        ),
        "bola": (-2900.0, 300.0),
        "azuis": [(0, -4300, 0, 0), (1, -3350, 300, 0),
                  (2, -1200, 1100, 0), (3, -2600, -1000, 0)],
        "amarelos": [(0, 4400, 0, 180), (1, 500, 300, 180), (2, 1600, 0, 180)],
        "comando": ("FORCE_START", "BLUE"),
    },
    "empurrao_apos_contorno": {        "tipo": "empurrao",
        "titulo": "Contorna e depois empurra: o arranque existe?",
        "descricao": (
            "Portador do lado errado a 300 mm: ele tem de orbitar e so depois "
            "empurrar. Mede o que o lote de 03/10 nao conseguiu separar - a "
            "orbita entregava o robo no lado certo e ali ele ficava, porque o "
            "alvo de contato nao mandava atravessar a bola."
        ),
        "bola": (1200.0, 0.0),
        "azuis": [(0, -4300, 0, 0), (1, 1500, 0, 0),
                  (2, 2600, 1300, 0), (3, 0, -1000, 0)],
        "amarelos": [(0, 4400, 1400, 180)],
        "comando": ("FORCE_START", "BLUE"),
    },

    # --- 5. chutar_em: REGRESSAO, nao comportamento novo --------------------
    #
    # A consolidacao dos tres canais num 'chutar_em' nao muda decisao - ela
    # junta orientacao, armamento e forca numa chamada. Nao existe "antes e
    # depois" de comportamento para medir: o que importa e que o que JA
    # funcionava continue funcionando. Por isso o trio reaproveita as
    # geometrias com resultado conhecido, e o critério e NAO PIORAR.
    "regressao_chute_livre": {        "tipo": "jogo",
        "titulo": "Chute com o gol livre (resultado conhecido: 3 de 3)",
        "descricao": (
            "Campo limpo, goleiro adversario fora da reta. Em campo limpo o "
            "chute saiu em 3 de 3 com 5555-5944 mm/s. O lote tem de repetir."
        ),
        "bola": (0.0, 0.0),
        "azuis": [(0, -4300, 0, 0), (1, -600, 0, 0)],
        "amarelos": [(0, 4400, 1500, 180)],
        "comando": ("FORCE_START", "BLUE"),
    },
    "regressao_goleiro_central": {        "tipo": "jogo",
        "titulo": "Decisão de chute com goleiro no centro da meta",
        "descricao": (
            "Goleiro amarelo centralizado na meta bloqueia o chute reto. "
            "O portador tem um apoio adiantado: observar no replay se a "
            "estratégia escolhe um canto livre ou prefere passar. O cenário "
            "não exige gol, pois o goleiro pode defender um chute correto."
        ),
        "bola": (0.0, 0.0),
        "azuis": [(0, -4300, 0, 0), (1, -600, 0, 0),
                  (2, 1800, 900, 0), (3, -1600, -900, 0)],
        "amarelos": [(0, 4400, 0, 180)],
        "comando": ("FORCE_START", "BLUE"),
    },
    "regressao_goleiro_central_sem_apoio": {        "tipo": "jogo",
        "titulo": "Chute com goleiro central, sem apoio",
        "descricao": (
            "Goleiro amarelo centralizado na meta, sem apoio nem cobertura "
            "azul em campo. Isola a decisão do portador: observar no replay "
            "se mira um canto livre ou mantém o chute bloqueado. Não exige gol."
        ),
        "bola": (0.0, 0.0),
        "azuis": [(0, -4300, 0, 0), (1, -600, 0, 0)],
        "amarelos": [(0, 4400, 0, 180)],
        "comando": ("FORCE_START", "BLUE"),
    },
    "regressao_passe": {        "tipo": "jogo",
        "titulo": "Passe com o gol fechado (resultado conhecido: passe sai)",
        "descricao": (
            "Gol bloqueado, apoio adiantado e livre: o esperado e passe com "
            "forca de passe (~2,5 m/s), nao chute de 6."
        ),
        "bola": (500.0, -800.0),
        # TRES ROBOS DE LINHA, e nao dois.
        #
        # Com dois, a distribuicao de papeis da PORTADOR ao mais proximo e
        # COBERTURA ao outro (a prioridade e portador -> cobertura -> apoio,
        # porque deixar a linha do gol descoberta custa mais). Sem APOIO nao ha
        # receptor, e 'alvo_do_chute' nunca escolhe passe: o pre-voo mostrou
        # 'tipo=bloqueado' num cenario chamado 'regressao_passe'.
        #
        # Isto e consequencia conhecida da regra de papeis, nao defeito deste
        # cenario - mas vale a anotacao: EM JOGO COM DOIS ROBOS DE LINHA O TIME
        # NAO PASSA.
        "azuis": [(0, -4300, 0, 0), (1, 100, -900, 0), (2, 2400, 800, 0),
                  (3, -600, 600, 0)],
        # OS AMARELOS TAPAM O GOL, NAO A LINHA DE PASSE.
        #
        # A primeira versao os punha em (1600,-500) e (2000,-200), em cima da
        # reta bola->apoio: o pre-voo mostrou 'tipo=bloqueado' em vez de
        # 'passe', isto e, o cenario nao testava o que diz o nome. Agora eles
        # ficam no corredor do gol e a linha de passe fica limpa.
        "amarelos": [(0, 4300, 0, 180), (1, 2200, -900, 180), (2, 3000, -600, 180)],
        "comando": ("FORCE_START", "BLUE"),
    },

    # NOVOS CENÁRIOS DE KICKOFF
    "kickoff_favor": {        "tipo": "kickoff",
        "titulo": "Nosso Kickoff",
        "descricao": "Kickoff a favor no centro do campo",
        "bola": (0.0, 0.0),
        "comando": ("KICKOFF", "BLUE"),
        "azuis": [
            (0, -2200.0, 0.0, 0.0),     # Robo 0: Goleiro na nossa meta
            (1, -300.0, 0.0, 0.0),      # Robo 1: Cobrador atras da bola
            (2, -1000.0, 600.0, 0.0),   # Robo 2: Apoio
        ],
        "amarelos": [
            (0, 2200.0, 0.0, 3.14),     # Robo 0: Goleiro deles
            (1, 1000.0, -500.0, 3.14),  # Robo 1: Defesa amarela
            (2, 1000.0, 500.0, 3.14),   # Robo 2: Defesa amarela
        ],
    },
    "kickoff_contra": {        "tipo": "kickoff",
        "titulo": "Kickoff Adversario",
        "descricao": "Kickoff deles no centro do campo",
        "bola": (0.0, 0.0),
        "comando": ("KICKOFF", "YELLOW"),
        "azuis": [
            (0, -2200.0, 0.0, 0.0),     # Robo 0: Goleiro na nossa meta
            (1, -750.0, -400.0, 0.0),   # Robo 1: Defesa azul (fora do raio central)
            (2, -750.0, 400.0, 0.0),    # Robo 2: Defesa azul
        ],
        "amarelos": [
            (0, 2200.0, 0.0, 3.14),     # Robo 0: Goleiro deles
            (1, 300.0, 0.0, 3.14),      # Robo 1: Cobrador amarelo
            (2, 1000.0, 600.0, 3.14),   # Robo 2: Apoio amarelo
        ],
    },
}


# ==========================================================================
#  grSim - teleporte de bola e robos
# ==========================================================================
def _carregar_protobuf():
    """Importa o grSim_Packet_pb2 que ja vem gerado no ssl-VICE.

    DOIS CAMINHOS, e isto nao e paranoia: com 'colcon build --symlink-install' o
    pacote vira um egg-link e o diretorio dentro de install/ NAO existe. O
    caminho de install so vale para um build copiado. Depois de uma
    reconstrucao limpa (rm -rf build install log), a versao antiga quebrava com
    ModuleNotFoundError no meio do 'posicionar' - e o sintoma no lote e apenas
    "cenario nao montou", sem dizer por que.
    """
    candidatos = [
        "/root/ssl-VICE/install/grsim_messenger/lib/python3.10/site-packages/"
        "grsim_messenger/protobuf",
        "/root/ssl-VICE/src/grsim_messenger/grsim_messenger/protobuf",
    ]
    for caminho in candidatos:
        if os.path.isdir(caminho) and caminho not in sys.path:
            sys.path.insert(0, caminho)
    import grSim_Packet_pb2

    return grSim_Packet_pb2


ROBOS_POR_TIME = 11   # "Robots Count" do grSim

FONTE_ROS = (
    "source /opt/ros/humble/setup.bash && "
    "source /root/ssl-VICE/install/local_setup.bash"
)


# NOTA sobre reiniciar o visionNode
#
# Isso e feito pelo ararabots.sh (cmd_cenario), de FORA deste processo, entre
# o "posicionar" e o "gravar". Nao da para fazer aqui dentro: se este script
# subir o visionNode como processo filho, o DDS nao consegue descobri-lo a partir
# do proprio pai - a assinatura fica muda (0 mensagens), embora qualquer outro
# processo leia o mesmo topico normalmente.
#
# Por que reiniciar: o filtro de Kalman absorve um teleporte de 5 m bem devagar
# (e faz bem - bola nenhuma pula assim na vida real). Sem o node novo, ele
# seguiria publicando a posicao ANTIGA por dezenas de segundos e a estrategia
# decidiria em cima dela. Com o rastreador zerado, o objeto nasce ja na posicao
# certa. Matar o visionNode nao derruba o resto: o sim_one.py nao tem on_exit.


_SOCK_GRSIM = None


def _sock_grsim():
    """Um socket UDP so, reaproveitado. Abrir e fechar a 10 Hz nao e de graca."""
    global _SOCK_GRSIM
    if _SOCK_GRSIM is None:
        _SOCK_GRSIM = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    return _SOCK_GRSIM


def _enviar_agora(pacote):
    """Envia UMA vez, sem dormir. Para comandos que se repetem sozinhos.

    POR QUE ISTO EXISTE - defeito do INSTRUMENTO, nao da estrategia
    ---------------------------------------------------------------
    O envio antigo era sempre 3 datagramas com sleep(0,05) entre eles: 150 ms
    de bloqueio por chamada. Isso e correto para um TELEPORTE (acontece uma vez
    e nao pode se perder), mas o laco de 'rodar' chama comandar_amarelos a
    10 Hz - e durante esses 150 ms o processo NAO faz spin no rclpy.

    Consequencia medida no proprio laco: cada volta custava ~0,25 s (0,10 de
    spin + 0,15 dormindo), entao ficavamos ~60% do tempo surdos. As amostras de
    visao que chegavam nesse intervalo se acumulavam na fila do DDS e eram
    carimbadas com o time.time() do momento em que finalmente foram tratadas.

    Isso envenena exatamente a medicao que queremos fazer: a velocidade da bola
    logo apos o disparo depende de dt entre quadros consecutivos, e o dt estava
    sendo inventado pelo atraso do nosso proprio laco.

    Um comando de velocidade perdido nao custa nada - vem outro em 100 ms.
    """
    _sock_grsim().sendto(pacote.SerializeToString(), (GRSIM_HOST, GRSIM_PORT))


def _enviar(pacote):
    """Envia repetido, para o que NAO pode se perder (teleporte, replacement).

    UDP nao garante entrega e o grSim aplica o ultimo que chegar. Aqui o
    bloqueio de 150 ms nao atrapalha: so acontece entre etapas, nunca durante
    a gravacao.
    """
    for _ in range(3):
        _sock_grsim().sendto(pacote.SerializeToString(), (GRSIM_HOST, GRSIM_PORT))
        time.sleep(0.05)


def _cmd_amarelo(c, rid, ori, vx_campo, vy_campo, girar=0.0, chute=0.0):
    """Adiciona ao pacote um comando de velocidade para UM robo amarelo.

    CONVERSAO DE REFERENCIAL - o erro que isto corrige
    -------------------------------------------------
    grSim espera veltangent/velnormal no referencial DO ROBO (eixo do corpo),
    nao no do campo. O controlador anterior mandava a direcao de campo direto, e
    como os amarelos sao posicionados com angulo 180 graus, todo comando saia
    invertido: o adversario andava para tras do que deveria.
    """
    ca, sa = math.cos(ori), math.sin(ori)
    rc = c.robot_commands.add()
    rc.id = int(rid)
    rc.wheelsspeed = False
    rc.veltangent = float(vx_campo * ca + vy_campo * sa)
    rc.velnormal = float(-vx_campo * sa + vy_campo * ca)
    rc.velangular = float(girar)
    rc.kickspeedx = float(chute)
    rc.kickspeedz = 0.0
    rc.spinner = False


# VELOCIDADE DO ADVERSARIO, limitada para igualar a nossa.
#
# Medido no mesmo jogo, janela de 10 quadros: os amarelos cravam 1300 mm/s (e o
# 'vel' daqui) e os nossos nao passam de 700 mm/s no p99, com p90 entre 260 e
# 480. O planejador nosso tem teto de 2000 mm/s e nao chega perto; o campo
# 'aggressiveness' existe na mensagem, e copiado no movement_manager e nunca
# lido pelo planejador, ou seja nao ha como pedir mais.
#
# Com o dobro da nossa velocidade o adversario ganha TODA corrida ate a bola, e
# qualquer teste de tatica passa a medir quem chega primeiro em vez de quem
# decide melhor. Igualar a velocidade e o que torna a comparacao honesta
# enquanto a equipe olha a falta de velocidade na cadeia de movimento.
#
#     ARARABOTS_VEL_INIMIGO=1.3 ./ararabots.sh ...   (volta ao antigo)
VEL_INIMIGO = float(os.environ.get("ARARABOTS_VEL_INIMIGO", "0.65"))


def _ir_para(rx, ry, ax, ay, vel=None, freio=400.0):
    if vel is None:
        vel = VEL_INIMIGO
    """Vetor de velocidade de (rx,ry) ate (ax,ay), freando na chegada.

    O freio proporcional existe pelo mesmo motivo da tatica: sem
    desaceleracao o robo passa do alvo e volta, oscilando para sempre.
    """
    dx, dy = ax - rx, ay - ry
    d = math.hypot(dx, dy)
    # Zona morta MAIOR para o goleiro nao ficar tremendo.
    #
    # Com 20 mm ele microcorrigia sem parar em cima do alvo - o Felipe viu a
    # velocidade oscilando entre 0,000 e 0,001, um tremor que nao e movimento
    # nenhum e ainda polui o replay. 60 mm e menor que o raio do robo: parar
    # dentro disso e estar no lugar.
    if d < 60.0:
        return 0.0, 0.0
    v = vel * min(1.0, d / freio)
    return dx / d * v, dy / d * v


# Varredura poste a poste do goleiro adversario, em mm no eixo y.
#
# 440 mm deixa so 60 mm ate a trave (o robo tem 90 de raio, entao ele encosta na
# trave com o corpo, que e o que um goleiro faz mesmo). 600 mm/s da tempo de
# chegar na ponta antes de inverter - com 900 mm/s o alvo fugia e ele cobria so
# ~620 mm dos 1000 da meta.
#
# Triangular e nao senoidal de proposito: velocidade constante faz ele passar o
# mesmo tempo em cada y, entao o resultado do teste e interpretavel.
# O ALVO PASSA DO POSTE DE PROPOSITO - senao ele nao chega la.
#
# MEDIDO (um_so_cobrador, modo padrao): com o alvo em +-440 mm o goleiro
# alcancou +-380. A diferenca e o atraso do seguidor somado a zona morta de
# 60 mm do _ir_para: contra um alvo que anda a 600 mm/s ele nunca fecha os
# ultimos milimetros, entao o que se pede NAO e o que se cobre.
#
# 520 mm passa 20 mm do poste (meia-largura 500). Com o atraso medido isso
# coloca o CENTRO do goleiro por volta de +-460, e como ele tem 90 mm de raio o
# corpo cobre o poste inteiro - que e o que um goleiro faz ao encostar na trave.
# Pedir menos que o poste era deixar os cantos abertos por construcao.
#
# CONFIRMADO em 6 execucoes: ele varreu -473..+466 (antes +-380). Cobre a meta
# de 1000 mm inteira, e nas 4 execucoes sem gol a bola parou com o goleiro a
# menos de 55 mm dela em y - foram DEFESAS, nao erros de pontaria.
VARREDURA_ALCANCE = GOL_MEIA_LARGURA + 20.0
VARREDURA_VEL = 600.0

# Acima disto a bola foi chutada/empurrada e o goleiro para de varrer para
# interceptar. Empurrao fica em ~1500 mm/s e chute em 5000+; 400 mm/s so filtra
# o ruido da visao (~3 mm por quadro).
VEL_BOLA_EM_JOGO = 400.0


def _varredura_y():
    a = VARREDURA_ALCANCE
    periodo = 4.0 * a / VARREDURA_VEL          # ida e volta completa
    f = (time.time() % periodo) / periodo
    return (-a + 4.0 * a * f) if f < 0.5 else (3.0 * a - 4.0 * a * f)


def comandar_amarelos(bola, amarelos, azuis=None, modo="nossa_falta", bola_vel=None):
    """Adversario com PAPEIS, em vez de um robo solto atacando a bola."""
    if not amarelos:
        return
    # ADVERSARIO PARADO: isola a nossa estrategia do comportamento deles.
    #
    # Com o amarelo jogando nao da para saber se o nosso atacante e passivo ou
    # se apenas perde a disputa. Parado, sobra so o nosso lado: se mesmo assim
    # ele nao chega na bola, o defeito e nosso.
    #
    #     ARARABOTS_INIMIGO_PARADO=1 ./ararabots.sh validar 3 jogo
    if os.environ.get("ARARABOTS_INIMIGO_PARADO"):
        return
    azuis = azuis or {}
    pb = _carregar_protobuf()
    bx, by = bola
    pacote = pb.grSim_Packet()
    c = pacote.commands
    c.timestamp = 0.0
    c.isteamyellow = True

    # a meta DELES fica em +x (atacamos +x), a nossa em -x
    meta_deles_x, nossa_meta_x = GOL_X, -GOL_X
    ids = sorted(amarelos)

    # ------------------------------------------------------------- goleiro
    if 0 in amarelos:
        gx, gy, gori = amarelos[0]
        alvo_x = meta_deles_x - 120.0

        bvx, bvy = bola_vel if bola_vel else (0.0, 0.0)
        alvo_y = by
        if bvx > 300.0:
            t = max(0.0, (alvo_x - bx) / bvx)
            if t < 3.0:
                alvo_y = by + bvy * t
        alvo_y = max(-GOL_MEIA_LARGURA + 60.0, min(GOL_MEIA_LARGURA - 60.0, alvo_y))

        # O GOLEIRO VARRE ENQUANTO A BOLA ESTA PARADA - nos DOIS modos.
        #
        # BUG QUE ISTO CORRIGE (relatado pelo Felipe): no modo padrao ("segue a
        # bola") o alvo do goleiro e o y da bola. Numa cobranca de falta a bola
        # fica parada por toda a fase de aproximacao, entao o alvo nao muda e o
        # goleiro fica IMOVEL ate o chute. O que se via no replay era ele
        # "so comecar a se mexer depois do chute" - e literalmente o que o
        # codigo mandava.
        #
        # Consequencia para a medicao: o canto estava SEMPRE aberto no instante
        # do disparo, e o goleiro partia do repouso (o grSim zera a velocidade
        # sem comando novo), entao ele nem tinha inercia para chegar. O numero
        # de gol contra esse goleiro e otimista por construcao - o mesmo tipo de
        # vies que o §9-0001 descreve para o goleiro imovel.
        #
        # Agora: bola parada -> varre poste a poste; bola em jogo -> intercepta,
        # que e a logica de reflexo que ja existia acima.
        #
        # A letra 'k' (GOLEIRO_PATRULHA) continua valendo e agora significa
        # "varre SEMPRE, mesmo com a bola em movimento" - util para medir chute
        # no canto sem que o goleiro corrija a rota atras da bola.
        vel_bola = math.hypot(bvx, bvy)
        if os.environ.get("GOLEIRO_PATRULHA") or vel_bola < VEL_BOLA_EM_JOGO:
            alvo_y = _varredura_y()

        # Mais velocidade e freio MAIS CURTO para o goleiro.
        #
        # freio=120 fazia ele desacelerar a 12 cm do alvo - com o alvo em
        # movimento, isso e uma frenagem permanente. Com 60 mm ele so alivia
        # ao encostar no ponto, e cobre a meta inteira.
        # Velocidade PROPORCIONAL ao alvo, e freio longo o bastante para nao
        # ultrapassar.
        #
        # Com vel=3,0 e freio=60 o goleiro perseguia um alvo que anda a
        # 600 mm/s indo a 3 m/s: passava direto toda vez e oscilava de -937 a
        # +709, ou seja 1849 mm num gol de 1000. Cobria "um lado" porque estava
        # sempre fora da meta, nao dentro dela.
        #
        # 1,2 m/s tem folga sobre os 600 mm/s do alvo, e frear a partir de
        # 250 mm faz ele chegar sem passar.
        vx, vy = _ir_para(gx, gy, alvo_x, alvo_y, freio=250.0)
        _cmd_amarelo(c, 0, gori, vx, vy)

    linha = [r for r in ids if r != 0]

    # SO O GOLEIRO EM CAMPO: envia agora e sai.
    #
    # BUG QUE ISTO CORRIGE: todos os ramos de modo abaixo sao guardados por
    # 'and linha', e o _enviar_agora mora DENTRO deles. Quando o unico
    # adversario e o goleiro - que e exatamente o cenario 12,
    # 'um_so_cobrador' - a lista fica vazia, nenhum ramo casa, e o pacote que
    # acabamos de montar para o goleiro e simplesmente descartado.
    #
    # O goleiro nunca recebeu comando nenhum nesses cenarios. Media-se a
    # cobranca contra um obstaculo imovel achando que era um goleiro, e tanto
    # "segue a bola" quanto a patrulha eram inuteis pelo mesmo motivo.
    #
    # MEDIDO: com a patrulha ligada, a amplitude do goleiro em y foi de 11 mm em
    # 1005 amostras - ou seja, zero movimento.
    if not linha:
        _enviar_agora(pacote)
        return

    # ------------------------------------------------------------- nosso KICKOFF (falta azul)
    if modo == "nosso_kickoff" and linha:
        # Robôs amarelos defendem: mantêm-se no campo deles (x > 750 mm) fora do círculo central
        for ordem, rid in enumerate(linha):
            rx, ry, rori = amarelos[rid]
            lado = 1.0 if ordem % 2 == 0 else -1.0
            ax = 850.0 + (ordem // 2) * 350.0
            ay = lado * (600.0 + (ordem // 2) * 450.0)
            vx, vy = _ir_para(rx, ry, ax, ay, vel=VEL_INIMIGO)
            _cmd_amarelo(c, rid, rori, vx, vy)
        _enviar_agora(pacote)
        return

    # ------------------------------------------------------------- KICKOFF deles (falta amarela)
    if modo == "kickoff_deles" and linha:
        cobrador = min(linha, key=lambda r: math.hypot(amarelos[r][0] - bx, amarelos[r][1] - by))
        for ordem, rid in enumerate(linha):
            rx, ry, rori = amarelos[rid]
            if rid == cobrador:
                # Cobrador se posiciona em +x (atrás da bola) apontando para o nosso gol (-x)
                dist_bola = math.hypot(rx - bx, ry - by)
                atras_x, atras_y = bx + 220.0, by
                perto = dist_bola < 140.0
                alvo = (bx, by) if math.hypot(rx - atras_x, ry - atras_y) < 150.0 else (atras_x, atras_y)
                vx, vy = _ir_para(rx, ry, alvo[0], alvo[1], vel=min(0.9, VEL_INIMIGO), freio=250.0)
                _cmd_amarelo(c, rid, rori, vx, vy, chute=5.0 if perto else 0.0)
            else:
                # Demais robôs aguardam o passe na metade do seu campo
                lado = 1.0 if ordem % 2 == 0 else -1.0
                ax = 1100.0
                ay = lado * (750.0 + ordem * 300.0)
                vx, vy = _ir_para(rx, ry, ax, ay, vel=1.0)
                _cmd_amarelo(c, rid, rori, vx, vy)
        _enviar_agora(pacote)
        return

    # ------------------------------------------------------------- falta DELES
    if modo == "falta_deles" and linha:
        cobrador = min(linha, key=lambda r: math.hypot(amarelos[r][0] - bx,
                                                       amarelos[r][1] - by))
        for ordem, rid in enumerate(linha):
            rx, ry, rori = amarelos[rid]
            if rid == cobrador:
                dx, dy = nossa_meta_x - bx, 0.0 - by
                n = math.hypot(dx, dy) or 1.0
                ux, uy = dx / n, dy / n
                atras_x, atras_y = bx - ux * 260.0, by - uy * 260.0
                dist_bola = math.hypot(rx - bx, ry - by)
                proj = (rx - bx) * ux + (ry - by) * uy
                if proj > -80.0:
                    ang = math.atan2(ry - by, rx - bx) + 0.7
                    raio = max(600.0, dist_bola)
                    alvo = (bx + raio * math.cos(ang), by + raio * math.sin(ang))
                    vx, vy = _ir_para(rx, ry, alvo[0], alvo[1], vel=1.1)
                    _cmd_amarelo(c, rid, rori, vx, vy)
                else:
                    perto = dist_bola < 140.0
                    alvo = (bx, by) if math.hypot(rx - atras_x, ry - atras_y) < 150.0 \
                        else (atras_x, atras_y)
                    vx, vy = _ir_para(rx, ry, alvo[0], alvo[1], vel=min(0.9, VEL_INIMIGO), freio=250.0)
                    _cmd_amarelo(c, rid, rori, vx, vy, chute=5.0 if perto else 0.0)
            else:
                lado = 1.0 if ordem % 2 == 0 else -1.0
                ax = bx - 900.0
                ay = max(-2400.0, min(2400.0, by + lado * 900.0))
                vx, vy = _ir_para(rx, ry, ax, ay, vel=1.0)
                _cmd_amarelo(c, rid, rori, vx, vy)
        _enviar_agora(pacote)
        return

    # ------------------------------------------------------------- nossa falta
    if modo == "nossa_falta" and linha:
        dx, dy = meta_deles_x - bx, 0.0 - by
        n = math.hypot(dx, dy) or 1.0
        ux, uy = dx / n, dy / n
        px, py = -uy, ux
        for ordem, rid in enumerate(linha):
            rx, ry, rori = amarelos[rid]
            lado = (ordem // 2 + 1) * (1.0 if ordem % 2 == 0 else -1.0)
            ax = bx + ux * 700.0 + px * lado * 180.0
            ay = by + uy * 700.0 + py * lado * 180.0
            if math.hypot(ax - bx, ay - by) < 550.0:
                ax, ay = bx + ux * 600.0, by + uy * 600.0
            vx, vy = _ir_para(rx, ry, ax, ay, vel=VEL_INIMIGO)
            _cmd_amarelo(c, rid, rori, vx, vy)
        _enviar_agora(pacote)
        return

    # ------------------------------------------------------------- jogo normal
    # ADVERSARIO = A NOSSA ESTRATEGIA ANTIGA. Azul joga a nova, amarelo a velha.
    #
    # POR QUE MUDOU: antes os amarelos MARCAVAM os nossos robos e nunca iam a
    # bola - ficavam a 350 mm de cada azul, na direcao do gol. Sem ninguem
    # disputando, a bola parava e o jogo travava; media-se a estrategia nova
    # contra um adversario que nao joga, e o replay nao mostrava jogo nenhum.
    #
    # O perfil aqui e o que a NOSSA tatica fazia antes desta fase, e ele e util
    # justamente por ser diferente: TODOS vao a bola, sem papeis e sem eleicao,
    # e empurram na direcao do gol adversario (o nosso). E o comportamento que
    # produzia 36% de amontoado quando era nosso - agora ele e o sparring.
    #
    # Dois perfis distintos em campo geram disputa de verdade, que e o que
    # faltava para o jogo andar.
    nossa_meta = -meta_deles_x
    for rid in linha:
        rx, ry, rori = amarelos[rid]
        # direcao bola -> gol que eles atacam (o nosso)
        dgx, dgy = nossa_meta - bx, 0.0 - by
        n = math.hypot(dgx, dgy) or 1.0
        ux, uy = dgx / n, dgy / n
        # atras da bola, e depois atravessa - a mesma ideia do empurrao, sem
        # nenhuma das travas que a versao nova ganhou
        d_bola = math.hypot(rx - bx, ry - by)
        proj = (rx - bx) * ux + (ry - by) * uy
        if proj < -80.0:
            ax, ay = bx + ux * 300.0, by + uy * 300.0      # atravessa
        else:
            ax, ay = bx - ux * 220.0, by - uy * 220.0      # posiciona atras
        vx, vy = _ir_para(rx, ry, ax, ay, vel=VEL_INIMIGO)
        # chuta quando encosta, apontando para a nossa meta
        chute = 5.5 if d_bola < 140.0 else 0.0
        _cmd_amarelo(c, rid, math.atan2(uy, ux), vx, vy, chute=chute)
    _enviar_agora(pacote)


def mover_bola(x, y):
    """Move so a bola, sem tocar nos robos."""
    pb = _carregar_protobuf()
    pacote = pb.grSim_Packet()
    pacote.replacement.ball.x = x / 1000.0
    pacote.replacement.ball.y = y / 1000.0
    pacote.replacement.ball.vx = 0.0
    pacote.replacement.ball.vy = 0.0
    _enviar(pacote)


def posicionar(cenario):
    """Monta o cenario no grSim: bola, robos usados e o resto DESLIGADO.

    grSim espera METROS e angulo em GRAUS.

    Todo robo que o cenario nao lista entra com turnon=False e estacionado fora
    do campo. Isso importa: sslworld.cpp:1102 pula os robos desligados ao montar
    o pacote de visao, entao eles somem do jogo - nao aparecem no /visionTopic,
    nao viram aliados nem obstaculos, e nao poluem o relatorio. Sem isso os 22
    robos ficam em campo e fica impossivel enxergar o que a jogada fez.
    """
    pb = _carregar_protobuf()
    pacote = pb.grSim_Packet()
    rep = pacote.replacement

    bx, by = cenario["bola"]
    rep.ball.x = bx / 1000.0
    rep.ball.y = by / 1000.0
    rep.ball.vx = 0.0
    rep.ball.vy = 0.0

    # CAMPO LIMPO: so os nossos.
    #
    # Isola a nossa estrategia por completo. O ARARABOTS_INIMIGO_PARADO apenas
    # deixa de COMANDAR os amarelos - eles continuam em campo como obstaculos e
    # como inimigos no game_state, e a tatica muda de situacao por causa deles.
    # Aqui eles nem entram: turnon=False, fora da visao (sslworld.cpp:1102), o
    # que deixa o teste medindo exclusivamente o nosso lado.
    #
    #     ARARABOTS_SO_NOSSOS=1 ./ararabots.sh validar 3 jogo
    so_nossos = bool(os.environ.get("ARARABOTS_SO_NOSSOS"))

    for time_amarelo, chave in ((False, "azuis"), (True, "amarelos")):
        if so_nossos and time_amarelo:
            usados = {}
        else:
            usados = {rid: (x, y, d) for rid, x, y, d in cenario.get(chave, [])}
        for rid in range(ROBOS_POR_TIME):
            r = rep.robots.add()
            r.id = rid
            r.yellowteam = time_amarelo
            if rid in usados:
                x, y, direcao = usados[rid]
                r.x = x / 1000.0
                r.y = y / 1000.0
                r.dir = float(direcao)
                r.turnon = True
            else:
                # Estacionado atras da linha de fundo e desligado.
                r.x = 6.0 if time_amarelo else -6.0
                r.y = -3.6 + rid * 0.35
                r.dir = 0.0
                r.turnon = False

    _enviar(pacote)


# ==========================================================================
#  game-controller - cliente WebSocket minimo (sem dependencias externas)
# ==========================================================================
def enviar_comando_arbitro(tipo, time_cor="UNKNOWN"):
    sock = socket.create_connection((GC_HOST, GC_PORT), timeout=5)
    try:
        chave = base64.b64encode(os.urandom(16)).decode()
        req = (
            f"GET /api/control HTTP/1.1\r\n"
            f"Host: {GC_HOST}:{GC_PORT}\r\n"
            "Upgrade: websocket\r\nConnection: Upgrade\r\n"
            f"Sec-WebSocket-Key: {chave}\r\n"
            "Sec-WebSocket-Version: 13\r\n\r\n"
        )
        sock.sendall(req.encode())
        resp = b""
        while b"\r\n\r\n" not in resp:
            pedaco = sock.recv(4096)
            if not pedaco:
                raise RuntimeError("conexao fechada no handshake")
            resp += pedaco

        msg = json.dumps(
            {
                "change": {
                    "origin": "cenarios-freekick",
                    "newCommandChange": {
                        "command": {"type": tipo, "forTeam": time_cor}
                    },
                }
            }
        ).encode()

        cabecalho = bytearray([0x81])
        n = len(msg)
        if n < 126:
            cabecalho.append(0x80 | n)
        else:
            cabecalho.append(0x80 | 126)
            cabecalho += struct.pack(">H", n)
        mascara = os.urandom(4)
        cabecalho += mascara
        sock.sendall(
            bytes(cabecalho) + bytes(b ^ mascara[i % 4] for i, b in enumerate(msg))
        )
        sock.settimeout(1.0)
        try:
            sock.recv(4096)
        except socket.timeout:
            pass
    finally:
        sock.close()


# ==========================================================================
#  Gravacao via ROS 2
# ==========================================================================
PORTA_STATUS_AZUL = 30011   # "Blue Team status send port" do ~/.grsim.xml


def _ler_varint(buf, i):
    """Le um varint do protobuf. Devolve (valor, proximo indice)."""
    val = 0
    desloc = 0
    while i < len(buf):
        b = buf[i]
        i += 1
        val |= (b & 0x7F) << desloc
        if not (b & 0x80):
            return val, i
        desloc += 7
    return val, i


def _decodificar_status(buf):
    """Decodifica um Robots_Status do grSim, sem depender do protoc.

    POR QUE DECODIFICAR NA MAO
    --------------------------
    O grSim_Robotstatus.proto existe no fonte do grSim, mas o ssl-VICE so
    compila Commands, Packet e Replacement - nao ha modulo pronto para importar
    dentro do container. A mensagem tem quatro campos escalares, entao o parser
    manual cabe em vinte linhas e evita adicionar uma dependencia de build ao
    ambiente de testes de todo mundo.

    Robots_Status { repeated Robot_Status robots_status = 1 }
    Robot_Status  { int32 robot_id=1; bool infrared=2; bool flat_kick=3;
                    bool chip_kick=4 }

    Devolve: [{"id":n, "infrared":bool, "flat_kick":bool, "chip_kick":bool}, ...]
    """
    saida = []
    i = 0
    while i < len(buf):
        chave, i = _ler_varint(buf, i)
        campo, tipo = chave >> 3, chave & 0x07
        if campo != 1 or tipo != 2:
            break
        tam, i = _ler_varint(buf, i)
        sub, i = buf[i:i + tam], i + tam
        r = {"id": -1, "infrared": False, "flat_kick": False, "chip_kick": False}
        j = 0
        while j < len(sub):
            k, j = _ler_varint(sub, j)
            c, t = k >> 3, k & 0x07
            if t != 0:
                break
            v, j = _ler_varint(sub, j)
            if c == 1:
                r["id"] = v
            elif c == 2:
                r["infrared"] = bool(v)
            elif c == 3:
                r["flat_kick"] = bool(v)
            elif c == 4:
                r["chip_kick"] = bool(v)
        saida.append(r)
    return saida


IP_VISAO_GRSIM = "224.5.23.2"   # mesmo grupo que o sim_one.py passa ao visionNode
PORTA_VISAO_GRSIM = 10020


def _abrir_escuta_visao_crua():
    """Escuta a visao do grSim DIRETO, por fora do tracker do time.

    POR QUE ISTO E NECESSARIO
    -------------------------
    O /visionTopic nao carrega o que o grSim viu: carrega o que o filtro de
    Kalman do tracker ACREDITA. E medimos que, no voo da bola, essas duas
    coisas sao muito diferentes - a bola ficou parada em (2500,0) no topico
    por 1,2 s DEPOIS de o grSim confirmar o disparo.

    Qualquer medicao de velocidade de saida feita em cima do topico mede o
    filtro, nao a bola. Aqui pegamos o SSL_DetectionFrame antes de qualquer
    filtragem, com o t_capture do proprio simulador - que e o unico relogio
    confiavel que temos, ja que o nosso e o do laco de gravacao.

    Isto e so leitura de um grupo multicast que ja esta sendo transmitido:
    nao muda nada no ambiente e nao concorre com o visionNode (os dois
    recebem, e para isso serve o SO_REUSEADDR).

    O HANDOVER §12 ja recomendava isto em outras palavras: "confirme lendo a
    verdade direto do multicast do grSim antes de concluir qualquer coisa".
    """
    try:
        from vision.proto.messages_robocup_ssl_wrapper_pb2 import SSL_WrapperPacket
    except ImportError as e:
        print(f"   (sem visao crua: {e})")
        return None, None
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(("", PORTA_VISAO_GRSIM))
        mreq = struct.pack("4sl", socket.inet_aton(IP_VISAO_GRSIM),
                           socket.INADDR_ANY)
        s.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        s.setblocking(False)
        return s, SSL_WrapperPacket
    except OSError as e:
        print(f"   (sem visao crua: porta {PORTA_VISAO_GRSIM} {e})")
        return None, None


def _abrir_escuta_status():
    """Escuta o estado REAL do chutador que o grSim devolve.

    POR QUE ISTO VALE MAIS QUE QUALQUER INFERENCIA
    ----------------------------------------------
    Ate aqui o teste so sabia que a ESTRATEGIA pediu chute (campo 'kick' no
    /commandTopic). Se o disparo aconteceu de fato, e quando, era deduzido do
    pico de velocidade da bola - o mesmo pico que ja nos enganou uma vez, quando
    26584 mm/s de uma colisao foram lidos como chute (HANDOVER §6.8).

    O grSim responde a verdade. Em sslworld.cpp:678-692, sempre que 'infrared'
    (bola encostada na placa) ou o KickStatus mudam, ele monta um Robots_Status
    e manda para o endereco de quem enviou o comando, na porta fixa
    BlueStatusSendPort (30011 no nosso XML). Quem envia os comandos azuis e o
    grsim_publisher_node, de 127.0.0.1 - e o container 'vice' usa rede 'host',
    entao o datagrama chega em 127.0.0.1:30011 e podemos simplesmente escutar.

    Nao interferimos em nada: so lemos. Nao mandamos comando azul nenhum, o que
    pararia o robo (sslworld.cpp:613 aplica setSpeed do pacote recebido).

    Se a porta estiver ocupada, o teste segue sem esse dado, apenas com a
    medicao por visao.
    """
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(("0.0.0.0", PORTA_STATUS_AZUL))
        s.setblocking(False)
        return s
    except OSError as e:
        print(f"   (sem status do chutador: porta {PORTA_STATUS_AZUL} {e})")
        return None


def _criar_gravador():
    """Cria o node que escuta visao e comandos. rclpy ja deve estar inicializado."""
    from rclpy.node import Node
    from system_interfaces.msg import VisionMessage, TeamCommand

    class Gravador(Node):
        def __init__(self):
            super().__init__("gravador_cenarios")
            self.amostras = []
            self.kick_por_robo = {}
            self.gravando = False
            self.bola = None
            self.amarelos = {}
            self.gol = None          # "nosso", "contra" ou None
            # velocidade de cada aliado, usada por esperar_assentar
            self.velocidades = {}
            self.bola_vel = (0.0, 0.0)
            self.contato = {}
            # Relogio MONOTONICO: o dt entre quadros e a base da medicao de
            # velocidade da bola, e time.time() pode dar passo para tras se o
            # NTP ajustar o relogio no meio da gravacao.
            self.t0 = time.monotonic()
            # Instante em que a ESTRATEGIA pediu chute pela primeira vez.
            self.t_pedido_chute = {}
            # Eventos vindos do proprio grSim (verdade do simulador).
            self.eventos_chutador = []
            self.sock_status = _abrir_escuta_status()
            self.estado_chutador = {}
            # Visao crua do grSim, por fora do tracker: [(t_capture, x, y, t_nosso)]
            self.sock_visao, self._WrapperPacket = _abrir_escuta_visao_crua()
            self.visao_crua = []
            self.robos_crus = []
            self.amarelos_crus = []
            # SEGUNDA FONTE DE VISAO NO MESMO GRUPO MULTICAST: ancora do relogio
            # de captura, para rejeitar quadros que nao sao desta execucao.
            self._tc0 = None
            self._tn0 = None
            self.quadros_fora_do_relogio = 0
            # Todas as transicoes do pedido de chute, nao so a primeira.
            self.janelas_kick = []
            self._kick_anterior = {}
            self.create_subscription(VisionMessage, "visionTopic", self._visao, 10)
            self.create_subscription(VisionMessage, "visionTopic", self._contato, 10)
            self.create_subscription(TeamCommand, "commandTopic", self._comando, 10)
            # COMANDO DO ARBITRO: serve para NAO gravar quando ele vem vazio.
            #
            # Com comando vazio a arvore recusa todas as jogadas e o time fica
            # imovel - e o replay engana, porque parece tatica ruim. Ver a
            # conferencia antes de 'gravando por ...'.
            self.comando_arbitro = None
            self.create_subscription(RefereeMessage, "refereeTopic",
                                     self._arbitro_cmd, 10)
            # setpoint que o driver esta perseguindo, em mm. E o que diz se a
            # ancora dele ja voltou a coincidir com a realidade.
            self.setpoints = {}
            self.alvos = []
            # SETPOINT: /movement_tracker/control_reference (TrajectoryPoint).
            #
            # O topico antigo (/control_command, do driver) ACABOU: a dev
            # removeu o driver.py junto com o rename new_movement -> movement, e
            # a mensagem ControlCommand saiu do system_interfaces. Nao existe
            # mais caminho alternativo para tratar aqui.
            from movement_interfaces.msg import TrajectoryPoint as _TP
            self.create_subscription(_TP, "movement_tracker/control_reference",
                                     self._setpoint_novo, 10)

        def _setpoint_novo(self, msg):
            """Mesma serie temporal, vinda do tracker_node da movimentacao nova.

            O TrajectoryPoint ja vem em MILIMETROS - ao contrario do
            ControlCommand do driver, que vem em metros e por isso e multiplicado
            por 1000 no _setpoint. Confundir os dois da erro de rastreio na casa
            de 2.186.200 mm, que foi exatamente o que apareceu na primeira
            medicao.
            """
            rid = int(getattr(msg, "robot_id", 0))
            alvo = (float(msg.pos.x), float(msg.pos.y))
            self.setpoints[rid] = alvo
            if self.gravando:
                self.alvos.append((round(time.monotonic() - self.t0, 4), rid,
                                   alvo[0], alvo[1],
                                   float(msg.vel.x), float(msg.vel.y)))

        def _contato(self, msg):
            """Mede a geometria do CHUTADOR no ponto de maior aproximacao.

            Reproduz robot.cpp:120-128: a bola precisa estar a menos de 31,5 mm
            da placa (xx) e 40 mm do eixo do corpo (yy). Sem medir isto, um
            teste que falha nao distingue "chegou torto" de "nao chegou".
            """
            if not msg.balls or not msg.blue_robots:
                return
            b = msg.balls[0]
            for r in msg.blue_robots:
                d = math.hypot(b.position_x - r.position_x,
                               b.position_y - r.position_y)
                if d >= self.contato.get(r.id, (1e9,))[0]:
                    continue
                dx, dy = math.cos(r.orientation), math.sin(r.orientation)
                kx = r.position_x + 75.5 * dx
                ky = r.position_y + 75.5 * dy
                ex, ey = kx - b.position_x, ky - b.position_y
                xx = abs(ex * dx + ey * dy)
                yy = abs(-ex * dy + ey * dx)
                self.contato[r.id] = (d, xx, yy)

        def _setpoint_novo(self, msg):
            """Mesma serie temporal, vinda do tracker_node da movimentacao nova.

            O TrajectoryPoint ja vem em MILIMETROS - ao contrario do
            ControlCommand do driver, que vem em metros e por isso e multiplicado
            por 1000 no _setpoint. Confundir os dois da erro de rastreio na casa
            de 2.186.200 mm, que foi exatamente o que apareceu na primeira
            medicao.
            """
            rid = int(getattr(msg, "robot_id", 0))
            alvo = (float(msg.pos.x), float(msg.pos.y))
            self.setpoints[rid] = alvo
            if self.gravando:
                self.alvos.append((round(time.monotonic() - self.t0, 4), rid,
                                   alvo[0], alvo[1],
                                   float(msg.vel.x), float(msg.vel.y)))

        def _drenar_visao_crua(self):
            """Le os quadros de visao do grSim sem passar pelo tracker."""
            if self.sock_visao is None:
                return
            while True:
                try:
                    dados, _ = self.sock_visao.recvfrom(65536)
                except (BlockingIOError, OSError):
                    return
                if not self.gravando:
                    continue
                try:
                    pkt = self._WrapperPacket()
                    pkt.ParseFromString(dados)
                except Exception:
                    continue
                if not pkt.HasField("detection"):
                    continue
                tc = round(float(pkt.detection.t_capture), 6)

                # UM SO RELOGIO DE CAPTURA POR EXECUCAO.
                #
                # DEFEITO QUE ISTO CORRIGE, e ele invalidou metricas de 17 dos
                # 30 replays do lote de 03/10/2026. O grupo multicast da visao
                # nao e nosso: qualquer outro grSim ou ssl-vision na maquina (ou
                # na rede) manda pacotes para ca, com o RELOGIO DELE. Medido nos
                # replays: depois de t = 24,5 s aparecia um bloco de quadros com
                # t_capture ~45.000 s, SEIS robos amarelos (a configuracao
                # default do grSim, nao a do cenario) e a bola parada em
                # (-4778, 1980), fora do campo.
                #
                # O que isso fazia com os numeros: 'a bola andou 6359 mm' no
                # cenario 'protecao_frontal', quando a bola real nao saiu do
                # lugar (deslocamento liquido de 7 mm em 24,5 s). A conclusao
                # tirada dai - "a protecao nao tem efeito visivel" - estava
                # certa por acidente; a de que "a orbita e pior" estava ERRADA,
                # porque comparava modulo de deslocamento poluido.
                #
                # O criterio nao e o valor do t_capture (nao sabemos a epoca que
                # o remetente usa) e sim a TAXA: o relogio de captura tem de
                # andar junto com o nosso. Quem divergir mais que TOL_RELOGIO
                # desde o primeiro pacote aceito e outro remetente.
                _tn = time.monotonic() - self.t0
                if self._tc0 is None:
                    self._tc0, self._tn0 = tc, _tn
                elif abs((tc - self._tc0) - (_tn - self._tn0)) > TOL_RELOGIO:
                    self.quadros_fora_do_relogio += 1
                    continue

                # A BOLA E OPCIONAL NESTE PACOTE - os ROBOS nao sao.
                #
                # BUG QUE ISTO CORRIGE, e ele apagava metade do campo: o grSim
                # manda UM PACOTE POR CAMERA. A camera que enxerga o campo de
                # ataque nao ve a bola quando ela esta no campo de defesa, entao
                # o pacote dela vem sem 'balls' - e a guarda antiga descartava o
                # pacote INTEIRO, com todos os robos que estavam nele.
                #
                # Consequencia medida no cenario 'jogo': o goleiro adversario
                # (amarelo id 0) aparecia em (4380, 339) no /game_state e estava
                # AUSENTE de 'amarelos_crus' - zero amostras em 25 s. O replay,
                # que le a visao crua, nao tinha como desenha-lo; e como ele
                # sumia, o rotulo "GK adv" caia por engano num jogador de linha
                # no meio do campo.
                #
                # Ou seja: todo robo do lado oposto ao da bola era invisivel
                # para o replay e para qualquer analise feita sobre a visao
                # crua. Agora os robos sao gravados sempre; so a serie da BOLA
                # depende de o pacote traze-la.
                if pkt.detection.balls:
                    b = pkt.detection.balls[0]
                    self.visao_crua.append((
                        tc, float(b.x), float(b.y),
                        round(time.monotonic() - self.t0, 4),
                    ))
                # Os NOSSOS robos, tambem crus. Sem a orientacao verdadeira nao
                # da para saber se a janela do chutador esteve aberta - e a
                # orientacao do topico e justamente a que atrasa 17-23 graus.
                for r in pkt.detection.robots_blue:
                    self.robos_crus.append((tc, int(r.robot_id), float(r.x),
                                            float(r.y), float(r.orientation)))
                # OS ADVERSARIOS TAMBEM, crus.
                #
                # Sem isto o replay nao tinha como desenhar o goleiro deles: os
                # quadros so carregavam robots_blue. Era por isso que o goleiro
                # "nao aparecia" - nao era estilo de desenho, o dado nunca foi
                # gravado.
                for r in pkt.detection.robots_yellow:
                    self.amarelos_crus.append((tc, int(r.robot_id), float(r.x),
                                               float(r.y), float(r.orientation)))

        def _drenar_status(self):
            """Le tudo que o grSim mandou desde a ultima vez, sem bloquear."""
            self._drenar_visao_crua()
            if self.sock_status is None:
                return
            while True:
                try:
                    dados, _ = self.sock_status.recvfrom(4096)
                except (BlockingIOError, OSError):
                    return
                agora = time.monotonic() - self.t0
                for r in _decodificar_status(dados):
                    antes = self.estado_chutador.get(r["id"])
                    novo = (r["infrared"], r["flat_kick"], r["chip_kick"])
                    self.estado_chutador[r["id"]] = novo
                    if not self.gravando or antes == novo:
                        continue
                    self.eventos_chutador.append({
                        "t": round(agora, 4),
                        "id": r["id"],
                        "infrared": r["infrared"],
                        "flat_kick": r["flat_kick"],
                        "chip_kick": r["chip_kick"],
                    })

        def _visao(self, msg):
            self._drenar_status()
            if msg.balls:
                b = msg.balls[0]
                self.bola = (b.position_x, b.position_y)
                # o goleiro precisa da VELOCIDADE para ter reflexo: so a
                # posicao o faz correr atras da bola em vez de interceptar
                self.bola_vel = (float(getattr(b, "velocity_x", 0.0) or 0.0),
                                 float(getattr(b, "velocity_y", 0.0) or 0.0))
                # Gol: a bola cruzou a linha dentro da largura da meta?
                if self.gravando and self.gol is None and abs(b.position_y) <= GOL_MEIA_LARGURA:
                    if b.position_x >= GOL_X:
                        self.gol = "nosso"
                    elif b.position_x <= -GOL_X:
                        self.gol = "contra"
            # guarda tambem a ORIENTACAO: os comandos de velocidade do grSim
            # sao no referencial DO ROBO, entao sem o angulo nao da para
            # converter uma direcao de campo em veltangent/velnormal.
            self.amarelos = {r.id: (r.position_x, r.position_y, r.orientation)
                             for r in msg.yellow_robots}
            self.azuis_pos = {r.id: (r.position_x, r.position_y)
                              for r in msg.blue_robots}
            self.velocidades = {
                r.id: math.hypot(r.velocity_x, r.velocity_y)
                for r in msg.blue_robots
            }
            if not self.gravando:
                return
            self.amostras.append(
                {
                    # 4 casas: com a bola a 6 m/s, 1 ms vale 6 mm. Arredondar em
                    # 3 casas ja era ruido comparavel ao que queremos medir.
                    "t": round(time.monotonic() - self.t0, 4),
                    "bola": [
                        # vx/vy vem do Kalman do tracker: suavizado, atrasa no
                        # degrau do disparo. Fica gravado para comparacao, mas
                        # quem manda na medicao e a derivada das posicoes.
                        {"x": b.position_x, "y": b.position_y,
                         "vx": float(getattr(b, "velocity_x", 0.0) or 0.0),
                         "vy": float(getattr(b, "velocity_y", 0.0) or 0.0)}
                        for b in msg.balls
                    ],
                    "azuis": [
                        {
                            "id": r.id,
                            "x": r.position_x,
                            "y": r.position_y,
                            "ang": r.orientation,
                        }
                        for r in msg.blue_robots
                    ],
                    "amarelos": [
                        {"id": r.id, "x": r.position_x, "y": r.position_y}
                        for r in msg.yellow_robots
                    ],
                }
            )

        def _arbitro_cmd(self, m):
            # RefereeMessage traz 'command' DIRETO (system_interfaces/msg/
            # RefereeMessage.msg:3). Quem tem 'referee.command' e o GameState.
            # Trocar os dois faz o callback estourar AttributeError a cada
            # mensagem, e o guard aborta por falta de comando - recusando o lote
            # pelo motivo errado.
            self.comando_arbitro = m.command

        def _comando(self, msg):
            if not self.gravando:
                return
            for r in msg.robots:
                anterior = self.kick_por_robo.get(r.robot_id, 0.0)
                self.kick_por_robo[r.robot_id] = max(anterior, float(r.kick))
                # Instante do PRIMEIRO pedido de chute. E a referencia temporal
                # para procurar a partida da bola: sem ela, achar "o quadro
                # seguinte ao disparo" seria adivinhacao.
                if float(r.kick) > 0.0 and r.robot_id not in self.t_pedido_chute:
                    self.t_pedido_chute[r.robot_id] = round(
                        time.monotonic() - self.t0, 4)
                # e TODAS as trocas liga/desliga, para saber em que intervalos a
                # estrategia estava de fato armada
                armado = float(r.kick) > 0.0
                if self._kick_anterior.get(r.robot_id) != armado:
                    self._kick_anterior[r.robot_id] = armado
                    self.janelas_kick.append(
                        (round(time.monotonic() - self.t0, 4), r.robot_id, armado))

    return Gravador()


#  Limiares de leitura da velocidade da bola (HANDOVER §6.8, medidos)
CENTRO_ATE_PLACA_MM = 75.5   # mesmo valor usado no _contato do gravador
LIM_XX_GRSIM = 31.5          # KickerThickness*2 + BallRadius (robot.cpp:128)
LIM_YY_GRSIM = 40.0          # KickerWidth/2

VEL_EMPURRAO = 2500.0    # ate aqui e empurrao de corpo, nao chute
VEL_CHUTE_MIN = 4000.0   # dai para cima houve disparo de verdade
VEL_ABSURDA = 9000.0     # acima do teto fisico do grSim: colisao ou glitch
VEL_PARTIU = 1000.0      # a partir daqui consideramos que a bola partiu


def energia_do_chute(amostras, eventos, t_pedido, visao_crua=None):
    """Mede a velocidade com que a bola REALMENTE sai, quadro a quadro.

    A PERGUNTA QUE ISTO RESPONDE (HANDOVER §24, pendencia 1)
    --------------------------------------------------------
    Com o chute disparando e a bola bem centrada, ela percorre 1306-1679 mm
    quando 6,4 m/s deveriam render 2362 mm. Faltam ~40% e nunca soubemos de
    onde: o handover registra que o passo decisivo - "registrar a velocidade da
    bola no quadro seguinte ao disparo" - nunca foi dado, e que estivemos
    inferindo pela distancia percorrida.

    Sao duas explicacoes com previsoes numericas OPOSTAS:

      a) a bola sai a ~6,4 m/s e perde energia no caminho -> o alcance e
         fisico, e mexer na forca nao resolve nada;
      b) a bola sai a ~5,3 m/s -> nao e atrito, e o disparo que ja nasce fraco,
         e a correcao e de forca.

    Uma execucao separa as duas.

    POR QUE NAO USAR O vx/vy DA VISAO COMO MEDIDA PRINCIPAL
    -------------------------------------------------------
    Aquilo vem do filtro de Kalman do tracker, que existe justamente para
    suavizar degraus - e um chute e o degrau mais brusco que a bola sofre. Ele
    vai reportar a subida com atraso e amortecida. A derivada das POSICOES
    entre quadros consecutivos nao tem esse vies. Gravamos os dois e mostramos
    lado a lado; se discordarem muito, e o filtro atrasando, nao a medida.

    JANELA CURTA, DE PROPOSITO
    --------------------------
    A desaceleracao medida em alta velocidade e ~8,9 m/s² (§19), ou seja
    ~0,15 m/s perdidos por quadro a 60 Hz. Uma media longa subestimaria a
    saida; por isso a estimativa principal usa ~50 ms logo apos a partida.
    """
    # FONTE. Preferimos sempre a visao crua do grSim: ela tem o t_capture do
    # proprio simulador (relogio confiavel) e nao passou pelo filtro de Kalman
    # do tracker, que mede-se atrasar o voo da bola em centenas de milimetros.
    # O /visionTopic fica como reserva, e o relatorio diz qual foi usada.
    if visao_crua and len(visao_crua) >= 3:
        fonte = "visao crua do grSim (sem tracker, relogio t_capture)"
        # O grSim emite um quadro por camera. A bola aparece na camera cuja
        # regiao a contem, e ao cruzar a fronteira ela pode ser reportada por
        # DUAS no mesmo t_capture. Ordenar e ficar com uma leitura por instante
        # evita ler essa duplicata como um salto de posicao.
        vistos, limpo = set(), []
        for tc, x, y, _tn in sorted(visao_crua):
            if tc in vistos:
                continue
            vistos.add(tc)
            limpo.append((tc, {"x": x, "y": y}))
        faixa = limpo
        # Referencia temporal, no NOSSO relogio, traduzida para t_capture pelo
        # quadro cru mais proximo. Preferimos o instante em que o grSim disse
        # que DISPAROU (flat_kick) ao instante em que a estrategia PEDIU: entre
        # um e outro pode haver segundos de aproximacao, e o primeiro pedido
        # costuma cair antes de qualquer contato.
        disparo = next((x["t"] for x in (eventos or []) if x.get("flat_kick")), None)
        alvo = disparo if disparo is not None else (
            min(t_pedido.values()) if t_pedido else None)
        if alvo is not None:
            tc_ref = min(visao_crua, key=lambda r: abs(r[3] - alvo))[0]
            t_pedido = {"ref": tc_ref}
    else:
        fonte = "/visionTopic (filtrado pelo tracker - leia com desconfianca)"
        faixa = [(a["t"], a["bola"][0]) for a in amostras if a.get("bola")]
    if len(faixa) < 3:
        return None

    # velocidade quadro a quadro, pela derivada das posicoes
    quadros = []
    for (t0, b0), (t1, b1) in zip(faixa, faixa[1:]):
        dt = t1 - t0
        if dt <= 1e-4:
            continue
        d = math.hypot(b1["x"] - b0["x"], b1["y"] - b0["y"])
        quadros.append((t1, d / dt, d))

    if not quadros:
        return None

    # a partida da bola: primeiro quadro claramente em movimento, nao antes do
    # pedido de chute (se houve pedido)
    t_ref = min(t_pedido.values()) if t_pedido else 0.0
    partida = None
    for i, (t, v, _) in enumerate(quadros):
        if t >= t_ref - 0.05 and v >= VEL_PARTIU:
            partida = i
            break
    if partida is None:
        return None

    primeiros = [round(v, 1) for _, v, _ in quadros[partida:partida + 4]]

    # estimativa principal: deslocamento acumulado nos ~50 ms apos a partida
    # ATENCAO ao indice: quadros[i] e o intervalo faixa[i] -> faixa[i+1], e o
    # 't' guardado nele e o do FIM do intervalo. Entao a janela comeca em
    # faixa[partida][0], nao em quadros[partida][0]. Somar a distancia de todos
    # os intervalos e dividir pela duracao medida a partir do fim do primeiro
    # conta um intervalo a mais de espaco do que de tempo - um teste sintetico
    # com 6400 mm/s injetados devolvia 8237.
    t_ini = faixa[partida][0]
    janela = [q for q in quadros[partida:] if q[0] - t_ini <= 0.05] \
        or [quadros[partida]]
    dist_janela = sum(d for _, _, d in janela)
    dur_janela = janela[-1][0] - t_ini
    v_janela = dist_janela / dur_janela if dur_janela > 1e-4 else quadros[partida][1]

    v_pico = max(v for _, v, _ in quadros)
    com_bola = [a for a in amostras if a.get("bola")]
    v_kalman = max(
        (math.hypot(a["bola"][0].get("vx", 0.0), a["bola"][0].get("vy", 0.0))
         for a in com_bola), default=0.0)

    # quanto a bola andou da partida ate parar
    # Percurso do chute: da partida ate a bola PARAR (ou ate o fim da
    # gravacao). Medir ate o ultimo quadro incluiria qualquer toque posterior
    # - o robo alcancando a bola de novo, um adversario rebatendo - e o numero
    # deixaria de ser "o alcance daquele chute".
    p_ini = faixa[partida][1]
    p_fim, t_parou = faixa[-1][1], None
    lentos = 0
    for t, v, _ in quadros[partida:]:
        if v < 150.0:
            lentos += 1
            if lentos >= 5:
                t_parou = t
                p_fim = next(b for tt, b in faixa if tt >= t)
                break
        else:
            lentos = 0
    percorrido = math.hypot(p_fim["x"] - p_ini["x"], p_fim["y"] - p_ini["y"])

    if v_pico > VEL_ABSURDA:
        veredito = "COLISAO/GLITCH (acima do teto fisico do grSim)"
    elif v_pico >= VEL_CHUTE_MIN:
        veredito = "CHUTE de verdade"
    elif v_pico >= VEL_EMPURRAO:
        veredito = "chute fraco ou raspao"
    else:
        veredito = "EMPURRAO de corpo, nao chute"

    return {
        "fonte": fonte,
        "t_pedido_chute": t_ref if t_pedido else None,
        "t_partida": round(t_ini, 4),
        "v_primeiros_quadros": primeiros,
        "v_saida_janela_50ms": round(v_janela, 1),
        "v_pico_quadro": round(v_pico, 1),
        "v_pico_kalman": round(v_kalman, 1),
        "percorrido_mm": round(percorrido, 1),
        "veredito": veredito,
        "eventos_chutador": eventos,
    }


def janela_do_chutador(visao_crua, robos_crus, janelas_kick, eventos):
    """Quantos quadros a janela do chutador esteve ABERTA, e o que faziamos neles.

    A PERGUNTA QUE ISTO RESPONDE
    ---------------------------
    Uma execucao sem gol nao distingue duas falhas opostas:

      a) o robo nunca chegou a uma geometria que permitisse chutar
         -> o problema e a APROXIMACAO;
      b) a geometria esteve boa e a estrategia nao estava armada naquele
         instante -> o problema e a DECISAO (ou a visao em que ela se apoia).

    As duas pedem correcoes em lugares diferentes, e ate agora estivemos
    escolhendo no palpite.

    Aqui reproduzimos o teste do grSim (robot.cpp:120-128) quadro a quadro, com
    posicao e orientacao CRUAS - sem o tracker, cuja orientacao atrasa 17-23
    graus, o que sozinho vale 31-42 mm de yy.

    Devolve as contagens e, quando a janela abriu sem estarmos armados, o
    tamanho da maior oportunidade perdida.
    """
    if not visao_crua or not robos_crus:
        return None
    bolas = {}
    for tc, x, y, tn in visao_crua:
        bolas.setdefault(tc, (x, y, tn))

    # intervalos em que a estrategia pediu chute, no NOSSO relogio
    armado_ate = []
    aberto = {}
    for t, rid, ligou in sorted(janelas_kick or []):
        if ligou:
            aberto[rid] = t
        elif rid in aberto:
            armado_ate.append((aberto.pop(rid), t))
    for rid, t in aberto.items():
        armado_ate.append((t, float("inf")))

    def estava_armado(tn):
        return any(a <= tn <= b for a, b in armado_ate)

    abertos, abertos_armados, seq, melhor_seq = 0, 0, 0, 0
    melhor = None
    for tc, rid, rx, ry, ori in sorted(robos_crus):
        if tc not in bolas:
            continue
        bx, by, tn = bolas[tc]
        dx, dy = math.cos(ori), math.sin(ori)
        kx = rx + CENTRO_ATE_PLACA_MM * dx
        ky = ry + CENTRO_ATE_PLACA_MM * dy
        ex, ey = kx - bx, ky - by
        xx = abs(ex * dx + ey * dy)
        yy = abs(-ex * dy + ey * dx)
        if xx < LIM_XX_GRSIM and yy < LIM_YY_GRSIM:
            abertos += 1
            seq += 1
            melhor_seq = max(melhor_seq, seq)
            if estava_armado(tn):
                abertos_armados += 1
            elif melhor is None or seq > melhor[0]:
                melhor = (seq, round(tc, 3), round(xx, 1), round(yy, 1))
        else:
            seq = 0
    return {
        "quadros_com_janela_aberta": abertos,
        "desses_com_chute_armado": abertos_armados,
        "maior_sequencia_aberta": melhor_seq,
        "maior_perdida": melhor,
        "disparou": any(x.get("flat_kick") for x in (eventos or [])),
    }


def imprimir_janela(j):
    if not j:
        print("   JANELA DO CHUTADOR: sem dados crus para avaliar")
        return
    a, ar = j["quadros_com_janela_aberta"], j["desses_com_chute_armado"]
    print("   JANELA DO CHUTADOR (teste do grSim refeito na visao crua):")
    if a == 0:
        print("     nunca abriu - a geometria nunca permitiu chutar")
        print("     -> o gargalo e a APROXIMACAO")
        return
    print(f"     abriu em {a} quadros (maior sequencia: {j['maior_sequencia_aberta']})")
    print(f"     desses, com o chute ARMADO: {ar}")
    if ar == 0:
        p = j["maior_perdida"]
        print("     -> OPORTUNIDADE PERDIDA: a geometria permitia e nao estavamos armados")
        if p:
            print(f"        melhor momento: t_capture={p[1]}  xx={p[2]}  yy={p[3]}")
        print("     -> o gargalo e a DECISAO / a visao em que ela se apoia")


_MODELO_REPLAY = r"""<!doctype html><html lang="pt-BR"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Replay &middot; Ararabots</title><style>
/* ==========================================================================
   VISUALIZADOR DE REPLAY. Autocontido de proposito: um arquivo, nenhuma
   dependencia externa, abre em qualquer navegador e sobrevive a um zip no
   e-mail. O payload da execucao e injetado no lugar de /*DADOS*/.

   A LEITURA TEM UMA ORDEM, e o layout a segue:
     1. QUE execucao e esta (cenario, rotulo, hora)  -> cabecalho
     2. O QUE deu                                     -> veredito + numeros
     3. COMO deu                                      -> campo + linha do tempo
   ========================================================================== */
:root{
  --bg:#12161f; --carta:#1b2029; --carta2:#232935; --linha:#333b49;
  --fg:#e8eef5; --dim:#8b98a8; --fraco:#6b7687;
  --ok:#3ddc84; --ruim:#ff6b6b; --alvo:#ffd166; --azul:#4da3ff; --campo:#0d2818;
}
*{box-sizing:border-box}
html,body{height:100%}
body{margin:0;background:var(--bg);color:var(--fg);display:flex;flex-direction:column;
  font:14px/1.5 -apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,Ubuntu,sans-serif}
.mono,.num{font-family:ui-monospace,SFMono-Regular,Menlo,Consolas,monospace;
  font-variant-numeric:tabular-nums}

/* -------------------------------------------------------------- cabecalho */
header{flex:none;display:flex;align-items:center;gap:10px 18px;flex-wrap:wrap;
  padding:13px 20px;background:var(--carta);border-bottom:1px solid var(--linha)}
.cen{font-size:17px;font-weight:650;letter-spacing:.2px}
.rotulo{font-size:12.5px;padding:3px 9px;border-radius:5px;background:var(--carta2);
  color:var(--dim);font-family:ui-monospace,Menlo,monospace}
.quando{font-size:12.5px;color:var(--fraco)}
.selos{margin-left:auto;display:flex;gap:9px;flex-wrap:wrap}
.selo{padding:4px 11px;border-radius:999px;font-size:12.5px;font-weight:600;
  border:1px solid transparent}
.selo.bom{background:rgba(61,220,132,.16);border-color:rgba(61,220,132,.5);color:#b9f0d0}
.selo.mau{background:rgba(255,107,107,.15);border-color:rgba(255,107,107,.5);color:#ffc9c9}
.selo.neutro{background:var(--carta2);border-color:var(--linha);color:var(--dim)}

/* ---------------------------------------------------------------- numeros */
.kpis{flex:none;display:grid;gap:1px;background:var(--linha);
  grid-template-columns:repeat(auto-fit,minmax(150px,1fr));
  border-bottom:1px solid var(--linha)}
.kpi{background:var(--carta);padding:11px 16px}
.kpi dt{font-size:11px;text-transform:uppercase;letter-spacing:.6px;color:var(--fraco);
  margin:0 0 3px}
.kpi dd{margin:0;font-size:18px;font-weight:600;font-family:ui-monospace,Menlo,monospace;
  font-variant-numeric:tabular-nums}
.kpi dd small{font-size:11.5px;font-weight:400;color:var(--fraco);margin-left:4px;
  font-family:inherit}
.kpi.bom dd{color:var(--ok)} .kpi.mau dd{color:var(--ruim)}

/* ------------------------------------------------------------------ campo */
main{flex:1;min-height:0;display:flex;flex-direction:column;padding:14px 20px;gap:12px}
.palco{flex:1;min-height:0;display:flex;align-items:center;justify-content:center}
svg{width:100%;height:100%;max-height:100%;display:block;background:var(--campo);
  border-radius:10px;border:1px solid var(--linha)}

/* ----------------------------------------------------------- linha do tempo */
.tempo{flex:none;background:var(--carta);border:1px solid var(--linha);
  border-radius:10px;padding:12px 16px}
.marcas{position:relative;height:16px;margin:0 0 -4px}
.marca{position:absolute;top:0;width:2px;height:11px;border-radius:1px;cursor:pointer}
.marca.chute{background:var(--ok)}
.marca.gol{background:var(--alvo);width:3px;height:16px}
.marca::after{content:attr(data-r);position:absolute;top:-16px;left:50%;
  transform:translateX(-50%);font-size:10px;color:var(--fraco);white-space:nowrap}
input[type=range]{width:100%;margin:0;accent-color:var(--azul)}
.ctl{display:flex;align-items:center;gap:10px;flex-wrap:wrap;margin-top:10px}
button{background:var(--carta2);color:var(--fg);border:1px solid var(--linha);
  border-radius:7px;padding:8px 14px;cursor:pointer;font:inherit;font-size:13.5px}
button:hover{background:#2b3240}
button:focus-visible{outline:2px solid rgba(77,163,255,.7);outline-offset:2px}
button.toca{min-width:112px;font-weight:600;background:var(--azul);color:#06213d;
  border-color:transparent}
button.toca:hover{background:#63b0ff}
.vels{display:flex;border:1px solid var(--linha);border-radius:7px;overflow:hidden}
.vels button{border:0;border-radius:0;padding:8px 11px;font-size:13px}
.vels button.sel{background:var(--azul);color:#06213d;font-weight:600}
.relogio{font-size:14px;color:var(--dim)}
.relogio b{color:var(--fg);font-size:15px}
.agora{margin-left:auto;display:flex;gap:16px;flex-wrap:wrap;font-size:13px;color:var(--dim)}
.agora b{color:var(--fg)}
.sit{padding:2px 9px;border-radius:5px;font-size:12.5px;font-weight:600}
.sit-NOSSA{background:rgba(61,220,132,.2);color:#b9f0d0}
.sit-DELES{background:rgba(255,107,107,.2);color:#ffc9c9}
.sit-DISPUTA{background:rgba(255,209,102,.2);color:#ffe6b0}
.sit-SOLTA{background:var(--carta2);color:var(--dim)}

/* ---------------------------------------------------------------- legenda */
details.leg{flex:none;background:var(--carta);border:1px solid var(--linha);
  border-radius:10px}
details.leg>summary{padding:10px 16px;cursor:pointer;font-size:13px;color:var(--dim);
  list-style:none;display:flex;align-items:center;gap:9px}
details.leg>summary::-webkit-details-marker{display:none}
details.leg>summary::before{content:"▸";color:var(--fraco)}
details.leg[open]>summary::before{content:"▾"}
.leg-corpo{display:grid;gap:7px 22px;padding:4px 16px 14px;
  grid-template-columns:repeat(auto-fit,minmax(290px,1fr));
  font-size:12.5px;color:var(--dim)}
.leg-corpo i{display:inline-block;width:10px;height:10px;border-radius:50%;
  margin-right:7px;vertical-align:-1px}
kbd{background:var(--carta2);border:1px solid var(--linha);border-bottom-width:2px;
  border-radius:4px;padding:1px 6px;font-size:11.5px;font-family:ui-monospace,Menlo,monospace}
@media (max-width:760px){
  .selos{margin-left:0} .agora{margin-left:0; width:100%}
  main{padding:10px 12px}
}
</style></head><body>

<header>
  <span class="cen" id="cen">execucao</span>
  <span class="rotulo" id="rotulo" hidden></span>
  <span class="quando" id="quando"></span>
  <span class="selos">
    <span class="selo neutro" id="selo-gol">—</span>
    <span class="selo neutro" id="selo-disp">—</span>
    <span class="selo neutro" id="selo-ajuste" hidden></span>
  </span>
</header>

<dl class="kpis">
  <div class="kpi"><dt>duração</dt><dd id="k-dur">—<small>s</small></dd></div>
  <div class="kpi"><dt>pico da bola</dt><dd id="k-pico">—<small>mm/s</small></dd></div>
  <div class="kpi" id="kpi-rastreio"><dt>erro de rastreio (mediana)</dt>
    <dd id="k-med">—<small>mm</small></dd></div>
  <div class="kpi"><dt>rastreio p90</dt><dd id="k-p90">—<small>mm</small></dd></div>
  <div class="kpi"><dt>disparos do chutador</dt><dd id="k-chutes">—</dd></div>
  <div class="kpi"><dt>situação dominante</dt><dd id="k-sit">—</dd></div>
  <!-- ADVERSARIO EM CAMPO: o confundidor que mais custou leitura errada.
       Um lote de 21/09 marcou 5 gols em 6 no cenario 'jogo' e a planilha nao
       dizia que ele rodou com ARARABOTS_SO_NOSSOS - zero quadros com amarelo
       em campo. "5 em 6" sem adversario e a linha de base conhecida; com
       adversario sao 0 em ~40. Agora o replay diz, sozinho, em qual dos dois
       mundos ele foi gravado. -->
  <div class="kpi" id="kpi-adv"><dt>adversário em campo</dt><dd id="k-adv">—</dd></div>
</dl>

<main>
  <div class="palco"><svg id="campo" viewBox="-5000 -3400 10000 6800"></svg></div>

  <div class="tempo">
    <div class="marcas" id="marcas"></div>
    <input type="range" id="sl" min="0" value="0" aria-label="linha do tempo">
    <div class="ctl">
      <button class="toca" id="play">▶ reproduzir</button>
      <button id="passo-tras" title="quadro anterior (←)">◀ı</button>
      <button id="passo-frente" title="quadro seguinte (→)">ı▶</button>
      <div class="vels" id="vels">
        <button data-v="0.1">0,1×</button>
        <button data-v="0.25">0,25×</button>
        <button data-v="1" class="sel">1×</button>
        <button data-v="2">2×</button>
      </div>
      <span class="relogio num">t = <b id="t">0.00</b> / <span id="tfim">—</span> s</span>
      <span class="agora">
        <span>situação <span class="sit sit-SOLTA" id="sit">—</span></span>
        <span class="num">erro <b id="err">—</b> mm</span>
      </span>
    </div>
  </div>

  <details class="leg">
    <summary>Como ler este campo &middot; atalhos</summary>
    <div class="leg-corpo">
      <span><i style="background:#ff9f1c"></i>bola</span>
      <span><i style="background:#4da3ff"></i>nossos robôs — a <b>face chanfrada</b> é o chutador</span>
      <span><i style="background:#ffd166"></i>adversários (mesma forma)</span>
      <span><i style="background:#ffd166"></i>setpoint comandado — a linha tracejada é o <b>erro de rastreio</b></span>
      <span><i style="background:#3ddc84"></i>disparo do chutador (verdade do grSim)</span>
      <span>o risco claro saindo do robô é a <b>velocidade</b> (1 m/s = 200 mm)</span>
      <span>a cor do anel diz o <b>papel</b>: <i style="background:#3ddc84"></i>portador
        <i style="background:#ffd166;margin-left:6px"></i>apoio
        <i style="background:#8b98a8;margin-left:6px"></i>cobertura</span>
      <span>trilhas = últimos 3 s</span>
      <span>na linha do tempo: <i style="background:#3ddc84"></i>disparo
        <i style="background:#ffd166;margin-left:6px"></i>gol — clique para pular</span>
      <span><kbd>espaço</kbd> toca/pausa &nbsp; <kbd>←</kbd> <kbd>→</kbd> quadro
        &nbsp; <kbd>Home</kbd> início</span>
    </div>
  </details>
</main>

<script>
const D = /*DADOS*/;
const svg = document.getElementById('campo');
const NS = 'http://www.w3.org/2000/svg';
function el(n, at){const e=document.createElementNS(NS,n);for(const k in at)e.setAttribute(k,at[k]);return e;}
// campo Division B: 9000 x 6000, gols em x=+-4500 com 1000 de largura
svg.appendChild(el('rect',{x:-4500,y:-3000,width:9000,height:6000,fill:'none',stroke:'#3a6b52','stroke-width':20}));
svg.appendChild(el('line',{x1:0,y1:-3000,x2:0,y2:3000,stroke:'#3a6b52','stroke-width':14}));
svg.appendChild(el('circle',{cx:0,cy:0,r:500,fill:'none',stroke:'#3a6b52','stroke-width':14}));
for(const s of [-1,1]){
  svg.appendChild(el('rect',{x:s>0?4500:-4680,y:-500,width:180,height:1000,fill:'none',stroke:'#5b8f74','stroke-width':16}));
  svg.appendChild(el('rect',{x:s>0?3500:-4500,y:-1000,width:1000,height:2000,fill:'none',stroke:'#3a6b52','stroke-width':12}));
}
// TRILHA DO GOLEIRO ADVERSARIO.
// O replay so desenhava a bola e UM robo nosso: o goleiro deles nunca apareceu
// porque o dado nem era gravado. Agora ele tem trilha propria, grossa, porque e
// ele que decide se o gol sai.
const trilhaGK = el('path',{fill:'none',stroke:'#ffd166','stroke-width':22,opacity:.75}); svg.appendChild(trilhaGK);
const gAmarelos = el('g'); svg.appendChild(gAmarelos);
const gCompanheiros = el('g'); svg.appendChild(gCompanheiros);
const trilhaB = el('path',{fill:'none',stroke:'#ff9f1c','stroke-width':16,opacity:.45}); svg.appendChild(trilhaB);
const trilhaR = el('path',{fill:'none',stroke:'#4da3ff','stroke-width':14,opacity:.35}); svg.appendChild(trilhaR);
const linhaErro = el('line',{stroke:'var(--alvo)','stroke-width':12,'stroke-dasharray':'40 30',opacity:.9}); svg.appendChild(linhaErro);

// A LINHA ROBO - BOLA - GOL, e o erro dela.
//
// Pedido do Felipe (09/10/2026): "trace uma linha entre o robo, a bola e o gol,
// para ver o erro". Sao dois segmentos e nao um: bola->gol e a linha de tiro
// (onde a bola TEM de sair) e robo->bola e de onde ele vai empurrar. Quando os
// dois viram uma reta so, o posicionamento esta certo; o angulo entre eles E o
// erro, e aparece sozinho no desenho.
const linhaTiro = el('line',{stroke:'#8ef5a0','stroke-width':6,
  'stroke-dasharray':'60 40',opacity:.55}); svg.appendChild(linhaTiro);
const linhaEmpurrao = el('line',{stroke:'#f58e8e','stroke-width':6,
  opacity:.7}); svg.appendChild(linhaEmpurrao);
const alvo = el('g'); svg.appendChild(alvo);
alvo.appendChild(el('circle',{r:70,fill:'none',stroke:'#ffd166','stroke-width':14}));
alvo.appendChild(el('line',{x1:-110,y1:0,x2:110,y2:0,stroke:'#ffd166','stroke-width':10}));
alvo.appendChild(el('line',{x1:0,y1:-110,x2:0,y2:110,stroke:'#ffd166','stroke-width':10}));
// FORMA REAL DO ROBO SSL, e nao "circulo com uma seta comprida".
//
// O robo da SSL e um cilindro de 90 mm de raio com a FRENTE CHANFRADA: a placa
// do chutador fica a 73 mm do centro (e o mesmo CENTRO_ATE_PLACA que a tatica
// usa). A corda dessa face tem meia-altura sqrt(90^2 - 73^2) = 52,6 mm.
//
// Desenhar um circulo com uma linha branca de 170 mm saindo dele deixava a
// orientacao ambigua (a seta apontava para longe do corpo, parecendo um vetor
// de movimento) e nao parecia um robo. Com a face chanfrada, para onde o
// chutador aponta e OBVIO sem precisar de seta.
// SITUACAO E PAPEIS, recalculados aqui com as MESMAS regras da tatica.
//
// POR QUE RECALCULAR e nao gravar da estrategia: as regras sao puramente
// geometricas e sem estado (ver situacao_de_jogo e distribuir_papeis em
// tatics/running.py), entao os mesmos dados dao o mesmo resultado. A unica
// diferenca e a fonte: a tatica le o /game_state (filtrado pelo Kalman) e aqui
// usamos a visao CRUA. Para ler o replay isso e melhor, nao pior - mostra onde
// os robos estavam de verdade.
//
// ⚠️ Se as regras mudarem na tatica, mudem AQUI TAMBEM, senao o replay passa a
// mentir. E o tipo de duplicacao que ja nos custou caro; fica aqui porque o
// valor de ver o papel desenhado supera o risco, mas o risco existe.
const RAIO_POSSE = 250;

function situacaoDoQuadro(fr){
  const b = fr.b, nossos = (fr.r||[]).filter(r => r[0] !== 0), deles = fr.y||[];
  if (!nossos.length) return null;
  const dn = Math.min(...nossos.map(r => Math.hypot(b[0]-r[1], b[1]-r[2])));
  const dd = deles.length
    ? Math.min(...deles.map(a => Math.hypot(b[0]-a[1], b[1]-a[2]))) : 1e9;
  if (dn <= RAIO_POSSE && dd <= RAIO_POSSE) return "DISPUTA";
  if (dn <= RAIO_POSSE) return "NOSSA";
  if (dd <= RAIO_POSSE) return "DELES";
  return "SOLTA";
}

function papeisDoQuadro(fr){
  const b = fr.b, linha = (fr.r||[]).filter(r => r[0] !== 0);
  const papeis = {};
  if (!linha.length) return papeis;
  // portador: distancia QUANTIZADA em faixas de 500 mm, desempate por id -
  // e a mesma regra do eleger_atacante, e existe porque a distancia crua
  // alterna o vencedor a cada ciclo.
  const chave = r => {
    const d = Math.hypot(b[0]-r[1], b[1]-r[2]);
    return [Math.round(d/500), r[0]];
  };
  const ord = linha.slice().sort((x,y) => {
    const a = chave(x), c = chave(y);
    return a[0] !== c[0] ? a[0]-c[0] : a[1]-c[1];
  });
  papeis[ord[0][0]] = "portador";
  const resto = linha.filter(r => r[0] !== ord[0][0])
    .sort((x,y) => Math.hypot(b[0]-x[1], b[1]-x[2]) - Math.hypot(b[0]-y[1], b[1]-y[2]));
  if (resto.length) papeis[resto[0][0]] = "apoio";
  for (let k=1; k<resto.length; k++) papeis[resto[k][0]] = "cobertura";
  return papeis;
}

const COR_PAPEL = {portador:"#3ddc84", apoio:"#ffd166", cobertura:"#8b98a8"};

const R_ROBO = 90, FRENTE = 73, MEIA_FACE = Math.sqrt(90*90 - 73*73);
function formaRobo(){
  return 'M ' + FRENTE + ' ' + (-MEIA_FACE) +
         ' A ' + R_ROBO + ' ' + R_ROBO + ' 0 1 0 ' + FRENTE + ' ' + MEIA_FACE + ' Z';
}
// Velocidade desenhada como vetor a partir do centro. ESCALA: 1 m/s = 200 mm de
// seta, entao um robo a 1,5 m/s (o teto do controlador) mostra 300 mm - visivel
// sem cobrir o campo. Sem isto nao da para distinguir um robo parado de um robo
// cruzando o campo: os dois eram um circulo igual.
const ESC_VEL = 0.2;

// Velocidade de um robo no quadro i, em mm/s, a partir da propria gravacao.
// Procura o quadro anterior em que o MESMO id aparece e divide pelo dt real -
// nao pelo indice, que nao e tempo (ver a nota da reproducao).
function velRobo(i, rid){
  const q = Q[i]; const a = (q.r||[]).find(x=>x[0]===rid); if(!a) return null;
  for (let k=i-1; k>=0 && k>i-40; k--){
    const b = (Q[k].r||[]).find(x=>x[0]===rid);
    const dt = q.t - Q[k].t;
    if (b && dt > 0.02) return [(a[1]-b[1])/dt, (a[2]-b[2])/dt];
  }
  return null;
}

// ULTIMA POSICAO CONHECIDA, para o robo nao PISCAR.
//
// O vanishing esta em 5% por quadro (e de proposito - simula perda de deteccao
// real). Sem memoria, o robo some e reaparece varias vezes por segundo e o
// replay fica ilegivel; pior, da a impressao de que ele teleporta.
//
// Guardamos ate 0,25 s: acima disso ele SOME mesmo, porque uma ausencia longa e
// informacao de verdade - foi a visao que o perdeu, e a estrategia tambem.
const MEM_MS = 0.25;
function comMemoria(i){
  const q = Q[i]; const vistos = {}; const saida = [];
  (q.r||[]).forEach(r => { vistos[r[0]] = true; saida.push(r); });
  for (let k=i-1; k>=0 && (q.t - Q[k].t) <= MEM_MS; k--){
    (Q[k].r||[]).forEach(r => {
      if (!vistos[r[0]]) { vistos[r[0]] = true; saida.push(r); }
    });
  }
  return saida;
}

const gAzuis = el('g'); svg.appendChild(gAzuis);
const robo = el('g'); svg.appendChild(robo);
robo.appendChild(el('circle',{r:90,fill:'#4da3ff'}));
const seta = el('line',{x1:0,y1:0,x2:170,y2:0,stroke:'#fff','stroke-width':22,'stroke-linecap':'round'});
robo.appendChild(seta);
const bola = el('circle',{r:45,fill:'#ff9f1c'}); svg.appendChild(bola);
const marcaChute = el('circle',{r:0,fill:'none',stroke:'var(--ok)','stroke-width':22}); svg.appendChild(marcaChute);

// ------------------------------------------------------------- identidade
// O arquivo se chama <cenario>__<rotulo>__<HHMMSS>.html; o rotulo e a hora
// dizem QUAL execucao e esta, e so existiam no nome do arquivo. Um replay
// aberto do e-mail nao dizia nem de quando era.
(function(){
  document.getElementById('cen').textContent = D.cenario || 'execucao';
  const m = (location.pathname.split('/').pop() || '').match(/^(.+?)__(.+?)__(\d{6})\.html$/);
  if (m){
    const r = document.getElementById('rotulo');
    r.textContent = m[2]; r.hidden = false;
    document.getElementById('quando').textContent =
      m[3].slice(0,2) + ':' + m[3].slice(2,4) + ':' + m[3].slice(4,6);
  }
  document.title = 'Replay · ' + (D.cenario || 'execucao') + (m ? ' · ' + m[2] : '');
})();

// QUEM E O COBRADOR. Vem do proprio resultado; se faltar, cai no robo que mais
// se aproximou da bola ao longo da execucao - nunca em "o primeiro da lista",
// que era o criterio anterior e mudava de quadro em quadro.
const COBRADOR = (D.cobrador !== null && D.cobrador !== undefined) ? D.cobrador : (function(){
  const dist={};
  D.quadros.forEach(q=>{ (q.r||[]).forEach(r=>{
    const d=Math.hypot(q.b[0]-r[1], q.b[1]-r[2]);
    if(dist[r[0]]===undefined || d<dist[r[0]]) dist[r[0]]=d;
  });});
  let melhor=null;
  for(const k in dist) if(melhor===null || dist[k]<dist[melhor]) melhor=k;
  return melhor===null?null:+melhor;
})();

// ------------------------------------------------------------------ selos
(function(){
  const g = document.getElementById('selo-gol');
  g.textContent = D.gol === 'nosso' ? 'GOL A FAVOR'
                : D.gol === 'contra' ? 'GOL SOFRIDO' : 'sem gol';
  g.className = 'selo ' + (D.gol === 'nosso' ? 'bom' : D.gol === 'contra' ? 'mau' : 'neutro');
  const d = document.getElementById('selo-disp');
  d.textContent = D.disparou ? 'chutador disparou' : 'não disparou';
  d.className = 'selo ' + (D.disparou ? 'bom' : 'neutro');
  // EM QUE CONDICAO O FEEDFORWARD RODOU, quando gravado.
  //
  // Sem isto um numero velho nao dizia em que condicao rodou: o 'preparar'
  // alterava o controlador e nada registrava. Replays antigos nao tem o campo -
  // ai o selo nao aparece, em vez de mentir um default. Os de 03/10 e antes
  // trazem 'ajuste_pid' booleano, que e a forma antiga do mesmo dado.
  {
    const ff = (D.feedforward !== undefined && D.feedforward !== null)
      ? 'PID: feedforward ' + D.feedforward
      : ((D.ajuste_pid !== undefined && D.ajuste_pid !== null)
          ? (D.ajuste_pid ? 'PID: sem feedforward' : 'PID: feedforward cru')
          : null);
    if (ff){
      const a = document.getElementById('selo-ajuste');
      a.textContent = ff;
      a.className = 'selo neutro'; a.hidden = false;
    }
  }
})();

// ------------------------------------------------------------------- KPIs
//
// Os numeros que decidem a leitura, no alto e sem clique. Sao os mesmos que o
// 'validar' grava no CSV - aqui ficam ao lado do video que os produziu.
const KPI = (function(){
  const Qd = D.quadros;
  const dur = Qd.length ? Qd[Qd.length-1].t : 0;

  // pico de velocidade da bola, pelo dt REAL (o indice do quadro nao e tempo)
  let pico = 0;
  for (let i=1;i<Qd.length;i++){
    const dt = Qd[i].t - Qd[i-1].t;
    if (dt > 0.005){
      const v = Math.hypot(Qd[i].b[0]-Qd[i-1].b[0], Qd[i].b[1]-Qd[i-1].b[1]) / dt;
      if (v > pico && v < 12000) pico = v;       // 12 m/s = ruido de deteccao
    }
  }

  // erro de rastreio DO COBRADOR, pareando robo com o setpoint dele.
  // A versao anterior comparava q.r[0] com q.a - o primeiro robo da lista
  // contra o ultimo setpoint de qualquer robo, dois objetos sem relacao.
  const es=[];
  Qd.forEach(q=>{
    if(!q.a || !q.r) return;
    const alvo=q.a.find(a=>a[0]===COBRADOR), rb=q.r.find(r=>r[0]===COBRADOR);
    if(alvo && rb) es.push(Math.hypot(alvo[1]-rb[1], alvo[2]-rb[2]));
  });
  es.sort((x,y)=>x-y);
  const med = es.length ? es[es.length>>1] : null;
  const p90 = es.length ? es[Math.floor(.9*es.length)] : null;

  // disparos: transicoes de flat_kick para verdadeiro (nao o numero de eventos)
  let chutes = 0, antes = false;
  (D.eventos||[]).forEach(e=>{ const f=!!e.flat_kick; if(f && !antes) chutes++; antes=f; });

  // situacao dominante, com as MESMAS regras da tatica (ver situacaoDoQuadro)
  const conta={};
  Qd.forEach(q=>{ const sq=situacaoDoQuadro(q); if(sq) conta[sq]=(conta[sq]||0)+1; });
  const total=Object.values(conta).reduce((a,b)=>a+b,0);
  let dom=null; for(const k in conta) if(!dom||conta[k]>conta[dom]) dom=k;

  // quantos quadros tinham ao menos um adversario detectado
  const comAdv = Qd.filter(q => (q.y || []).length).length;
  const pctAdv = Qd.length ? 100 * comAdv / Qd.length : 0;

  const põe=(id,txt)=>{document.getElementById(id).innerHTML=txt;};
  põe('k-dur', dur.toFixed(1) + '<small>s</small>');
  põe('k-pico', pico ? Math.round(pico) + '<small>mm/s</small>' : '—');
  põe('k-med', med===null ? '—' : Math.round(med) + '<small>mm</small>');
  põe('k-p90', p90===null ? '—' : Math.round(p90) + '<small>mm</small>');
  põe('k-chutes', String(chutes));
  põe('k-sit', dom ? dom.toLowerCase() +
      '<small>' + Math.round(100*conta[dom]/total) + '% do tempo</small>' : '—');
  põe('k-adv', pctAdv >= 95 ? 'sim<small>100% do tempo</small>'
             : pctAdv < 1 ? 'NÃO<small>campo sem adversário</small>'
             : Math.round(pctAdv) + '%<small>do tempo</small>');
  // sem adversario nao e erro - e outro teste. Mas tem de ficar OBVIO, porque
  // um numero dali nao se compara com um de jogo disputado.
  document.getElementById('kpi-adv').className = 'kpi ' + (pctAdv < 1 ? 'mau' : '');
  // chute real e 5000+; empurrao fica na faixa de 2500
  document.getElementById('k-pico').parentElement.className =
    'kpi ' + (pico >= 5000 ? 'bom' : pico >= 2500 ? '' : 'mau');
  // as execucoes que marcaram tiveram mediana de 76 e 153 mm; a que falhou, 306
  if (med !== null)
    document.getElementById('kpi-rastreio').className = 'kpi ' + (med > 150 ? 'mau' : 'bom');
  return {dur: dur};
})();

const Q = D.quadros;
const sl = document.getElementById('sl'); sl.max = Q.length-1;
let pB='', pR='';
function desenha(i){
  const q = Q[i];
  document.getElementById('t').textContent = q.t.toFixed(2);
  bola.setAttribute('cx', q.b[0]); bola.setAttribute('cy', q.b[1]);
  pB = (i===0?'M':'L') + q.b[0] + ' ' + q.b[1] + (i===0?'':' ');
  // TODOS OS NOSSOS ROBOS, cada um com O SEU setpoint.
  //
  // Antes desenhava-se UM robo (q.r[0], que mudava de identidade entre quadros)
  // e UM alvo (o ultimo de qualquer robo). Com o time completo em campo isso
  // nao mostrava a jogada - mostrava uma colagem.
  //
  // O cobrador vai cheio e com seta; os companheiros vao esmaecidos e com o
  // proprio alvo em risco fino. A linha de erro grossa e a legenda numerica
  // seguem SO o cobrador, que e quem decide a cobranca.
  const papeisFr = papeisDoQuadro(q);
  const sit = situacaoDoQuadro(q);
  const bs = document.getElementById('sit');
  if (bs){
    bs.textContent = sit || '-';
    bs.className = 'sit sit-' + (sit || 'SOLTA');
  }
  trilhaB.setAttribute('d', trilhaDe(i, null, true));
  trilhaR.setAttribute('d', trilhaDe(i, COBRADOR, false));
  gAzuis.textContent = '';
  const nossos = comMemoria(i);
  const meu = nossos.find(r => r[0] === COBRADOR) || null;
  nossos.forEach(r => {
    const ehCob = (r[0] === COBRADOR);
    const g = el('g', {transform:'translate('+r[1]+','+r[2]+') rotate('+(r[3]*180/Math.PI)+')',
                       opacity: ehCob ? 1 : 0.55});
    g.appendChild(el('path',{d:formaRobo(), fill:'#4da3ff',
                             stroke: ehCob?'#dff0ff':'none','stroke-width':ehCob?14:0}));
    // risco curto sobre a face: reforca de que lado esta o chutador
    g.appendChild(el('line',{x1:FRENTE-6,y1:-MEIA_FACE,x2:FRENTE-6,y2:MEIA_FACE,
                             stroke:'#0d2818','stroke-width':16}));
    if (!ehCob){
      gAzuis.appendChild(el('path',{fill:'none',stroke:'#4da3ff','stroke-width':8,
                                    opacity:.25, d: trilhaDe(i, r[0], false)}));
    }
    gAzuis.appendChild(g);
    // numero SEMPRE, inclusive no cobrador - "quem e quem" e a primeira
    // pergunta ao olhar o replay, e antes so os companheiros tinham rotulo.
    const t = el('text',{x:r[1], y:r[2]-140,'text-anchor':'middle',
                         'font-size':160, fill: ehCob?'#dff0ff':'#4da3ff'});
    t.textContent = r[0];
    gAzuis.appendChild(t);
    // PAPEL, embaixo do robo. Era a pergunta que o replay nao respondia:
    // "quem esta fazendo o que agora".
    const pp = papeisFr[r[0]];
    if (pp){
      const tp = el('text',{x:r[1], y:r[2]+250,'text-anchor':'middle',
                            'font-size':135, fill: COR_PAPEL[pp] || '#8b98a8'});
      tp.textContent = pp;
      gAzuis.appendChild(tp);
    }
    // VETOR DE VELOCIDADE, medido entre quadros vizinhos da visao crua.
    const v = velRobo(i, r[0]);
    if (v && (Math.abs(v[0])+Math.abs(v[1])) > 60){
      gAzuis.appendChild(el('line',{x1:r[1],y1:r[2],
        x2:r[1]+v[0]*ESC_VEL, y2:r[2]+v[1]*ESC_VEL,
        stroke:'#dff0ff','stroke-width':ehCob?16:9,opacity:.85,
        'stroke-linecap':'round'}));
    }
    // setpoint DESTE robo
    const a = (q.a || []).find(x => x[0] === r[0]);
    if (a){
      gAzuis.appendChild(el('line',{x1:r[1],y1:r[2],x2:a[1],y2:a[2],
        stroke:'#ffd166','stroke-width': ehCob?0:8, opacity: ehCob?0:0.5}));
      if(!ehCob) gAzuis.appendChild(el('circle',{cx:a[1],cy:a[2],r:35,fill:'none',
        stroke:'#ffd166','stroke-width':8, opacity:0.5}));
    }
  });
  robo.style.display='none';   // o robo unico virou o laco acima

  // os dois segmentos da linha de mira, do cobrador
  if (meu && q.b){
    const golX = (D.sentido_ataque === undefined || D.sentido_ataque === null)
      ? 4500 : 4500 * (D.sentido_ataque >= 0 ? 1 : -1);
    linhaTiro.setAttribute('x1',q.b[0]); linhaTiro.setAttribute('y1',q.b[1]);
    linhaTiro.setAttribute('x2',golX);   linhaTiro.setAttribute('y2',0);
    linhaTiro.style.display='';
    linhaEmpurrao.setAttribute('x1',meu[1]); linhaEmpurrao.setAttribute('y1',meu[2]);
    linhaEmpurrao.setAttribute('x2',q.b[0]); linhaEmpurrao.setAttribute('y2',q.b[1]);
    linhaEmpurrao.style.display='';
  } else { linhaTiro.style.display='none'; linhaEmpurrao.style.display='none'; }

  const alvoCob = meu ? (q.a || []).find(x => x[0] === COBRADOR) : null;
  if (alvoCob && meu){
    alvo.setAttribute('transform','translate('+alvoCob[1]+','+alvoCob[2]+')');
    alvo.style.display='';
    linhaErro.setAttribute('x1',meu[1]); linhaErro.setAttribute('y1',meu[2]);
    linhaErro.setAttribute('x2',alvoCob[1]); linhaErro.setAttribute('y2',alvoCob[2]);
    linhaErro.style.display='';
    const e = Math.hypot(alvoCob[1]-meu[1], alvoCob[2]-meu[2]);
    const ee = document.getElementById('err');
    ee.textContent = e.toFixed(0);
    ee.style.color = e>200 ? 'var(--ruim)' : 'var(--ok)';
  } else { alvo.style.display='none'; linhaErro.style.display='none'; }
  // ADVERSARIOS: todos em amarelo; o mais proximo da meta e o goleiro e ganha
  // circulo maior, rotulo e trilha.
  gAmarelos.textContent = '';
  const ams = q.y || [];
  // GOLEIRO ADVERSARIO PELO ID, nao por "quem esta mais perto do gol".
  //
  // A regra desta base e fixa: o robo 0 e sempre o goleiro (o comandar_amarelos
  // do proprio ararabots.py trata o id 0 como goleiro). Eleger pelo mais
  // proximo do gol funciona numa cobranca, em que os amarelos ficam parados,
  // mas no cenario 'jogo' eles se movem - e vimos o rotulo "GK adv" aparecer no
  // MEIO DO CAMPO, colado num jogador de linha. Rotulo errado e pior que
  // nenhum: quem le o replay conclui que o goleiro abandonou a meta.
  let gk = ams.find(a => a[0] === 0) || null;
  if (!gk) for (const a of ams){ if (!gk || Math.abs(a[1]-4500) < Math.abs(gk[1]-4500)) gk = a; }
  for (const a of ams){
    const ehGK = gk && a[0]===gk[0];
    // mesma forma dos nossos: o adversario e um robo igual, so muda a cor.
    // Antes era um circulo liso - ficava impossivel ver para onde ele apontava,
    // e no goleiro isso e justamente o que interessa.
    const ga = el('g',{transform:'translate('+a[1]+','+a[2]+') rotate('+(a[3]*180/Math.PI)+')',
                       opacity: ehGK?.95:.6});
    ga.appendChild(el('path',{d:formaRobo(), fill:'#ffd166',
                              stroke: ehGK?'#1c1c1c':'none','stroke-width':ehGK?14:0}));
    ga.appendChild(el('line',{x1:FRENTE-6,y1:-MEIA_FACE,x2:FRENTE-6,y2:MEIA_FACE,
                              stroke:'#1c1c1c','stroke-width':16}));
    gAmarelos.appendChild(ga);
    if (ehGK){
      const tx = el('text',{x:a[1],y:a[2]-190,'text-anchor':'middle','font-size':150,fill:'#ffd166'});
      tx.textContent = 'GK adv';
      gAmarelos.appendChild(tx);
      // JANELA DESLIZANTE, e nao a execucao inteira.
      //
      // Acumulando 25 s de varredura poste a poste, a trilha vira um bloco
      // rabiscado que esconde o campo e nao informa nada. Com os ultimos ~2 s
      // da para LER o movimento: para onde ele estava indo no instante do
      // chute, que e a pergunta que importa.
      // JANELA DE TEMPO, calculada do zero a cada quadro.
      //
      // Antes era um array que crescia conforme os quadros eram DESENHADOS.
      // Arrastando a barra de tempo, ele acumulava pontos de instantes nao
      // contiguos e a trilha virava um risco reto atravessando o campo - um
      // caminho que o goleiro nunca fez. Agora e a mesma funcao das outras
      // trilhas: so o que aconteceu nos ultimos JAN_TRILHA segundos.
      const pg = [];
      for (let k=i; k>=0 && (q.t - Q[k].t) <= JAN_TRILHA; k--){
        const g2 = (Q[k].y||[]).find(z => z[0] === a[0]);
        if (g2) pg.push([g2[1], g2[2]]);
      }
      pg.reverse();
      trilhaGK.setAttribute('d',
        pg.length<2 ? '' : pg.map((p,k)=>(k?'L':'M')+p[0]+' '+p[1]).join(' '));
    }
  }
  // COMPANHEIROS: todo robo nosso que nao e o cobrador, e se a bola chegou perto
  // dele (raio de recepcao) o circulo fica verde - e a leitura de "recebeu".
  // SO O INDICADOR DE RECEPCAO - o robo em si ja foi desenhado acima.
  //
  // Este bloco redesenhava TODOS os companheiros como circulos lisos por cima
  // das formas novas (dois desenhos do mesmo robo, um sobre o outro) e voltava
  // a percorrer q.r POR INDICE, a identidade instavel que ja tinha nos
  // enganado. Agora ele so acrescenta o anel verde de "a bola chegou nele",
  // que e a leitura util no cenario de passe.
  gCompanheiros.textContent = '';
  nossos.forEach(c => {
    if (c[0] === COBRADOR) return;
    const d = Math.hypot(q.b[0]-c[1], q.b[1]-c[2]);
    if (d > 250) return;
    gCompanheiros.appendChild(el('circle',{cx:c[1],cy:c[2],r:150,fill:'none',
      stroke:'var(--ok)','stroke-width':20,opacity:.9}));
    const tc = el('text',{x:c[1],y:c[2]+300,'text-anchor':'middle','font-size':140,
                          fill:'var(--ok)'});
    tc.textContent = 'recebeu (' + d.toFixed(0) + ' mm)';
    gCompanheiros.appendChild(tc);
  });
  const ev = D.eventos.find(x => x.flat_kick && Math.abs(x.t - q.t) < 0.25);
  marcaChute.setAttribute('r', ev ? 260 : 0);
  if (ev){ marcaChute.setAttribute('cx', q.b[0]); marcaChute.setAttribute('cy', q.b[1]); }
}
// trilhas completas, desenhadas de uma vez

// TRILHA DO COBRADOR, nao de "q.r[0]".
//
// q.r vem de um dicionario: QUAL robo esta na posicao 0 muda de quadro em
// quadro. A trilha pulava entre o cobrador e o apoio e riscava o campo inteiro
// com um leque de linhas que nao era o caminho de ninguem. Era a parte mais
// visivel do "replay falso".
// TRILHAS COM JANELA DE TEMPO, desenhadas a cada quadro.
//
// Antes eram fixas e cobriam a execucao INTEIRA. Numa cobranca de 25 s com um
// robo isso ainda se lia; no cenario 'jogo', com tres robos nossos, tres deles
// e a bola, virou um novelo que escondia o campo - foi o "confuso".
//
// JAN_TRILHA = 3 s: tempo suficiente para ver de onde o robo veio e para onde
// vai, sem arrastar a historia toda. E a mesma ideia ja aplicada na trilha do
// goleiro adversario.
const JAN_TRILHA = 3.0;
function trilhaDe(i, rid, ehBola){
  const t1 = Q[i].t, pts = [];
  for (let k=i; k>=0 && (t1 - Q[k].t) <= JAN_TRILHA; k--){
    if (ehBola) pts.push(Q[k].b);
    else { const r=(Q[k].r||[]).find(x=>x[0]===rid); if(r) pts.push([r[1],r[2]]); }
  }
  pts.reverse();
  return pts.length<2 ? '' : pts.map((p,k)=>(k?'L':'M')+p[0]+' '+p[1]).join(' ');
}
const trilhasOutros = {};

// setInterval, e nao requestAnimationFrame: o rAF nao dispara quando a pagina
// esta em aba oculta ou num painel que a renderiza sem foco - o botao alternava
// e nada acontecia na tela.
// REPRODUCAO NO TEMPO REAL DA GRAVACAO, e nao "um quadro a cada 16 ms".
//
// ERA ISTO O "fora da realidade". A visao crua vem de QUATRO cameras do grSim,
// cada uma no seu ritmo: os quadros NAO sao igualmente espacados no tempo.
// Avancando um indice a cada 16 ms, dois quadros separados por 2 ms na gravacao
// apareciam com o mesmo intervalo de dois quadros separados por 30 ms - o
// movimento saltava, acelerava e freava sozinho, e nada daquilo aconteceu.
//
// Agora o relogio manda: a cada tique calculamos o INSTANTE de reproducao e
// escolhemos o quadro cujo q.t e o mais proximo. Um robo parado fica parado; um
// robo a 1,5 m/s atravessa a tela no tempo que levou de verdade.
let vel=1, timer=null, tPlay=0, tUltimo=0;
const T_FIM = Q.length ? Q[Q.length-1].t : 0;

// indice do quadro mais proximo de um instante (busca binaria)
function idxDoTempo(t){
  let lo=0, hi=Q.length-1;
  while (lo < hi){
    const m = (lo+hi) >> 1;
    if (Q[m].t < t) lo = m+1; else hi = m;
  }
  if (lo > 0 && Math.abs(Q[lo-1].t - t) < Math.abs(Q[lo].t - t)) lo--;
  return lo;
}

// --------------------------------------------------- reproducao e controles
//
// O TEMPO E O DA GRAVACAO, nao o do indice: a tela avanca a 30 Hz e o quadro
// escolhido e o que corresponde ao instante acumulado. Sem isso, um replay com
// perda de deteccao (quadros espacados) corre mais rapido nos trechos ruins.
const bt = document.getElementById('play');
function tique(){
  const agora = performance.now();
  tPlay += (agora - tUltimo) / 1000 * vel;
  tUltimo = agora;
  let i = idxDoTempo(tPlay);
  if (i >= Q.length - 1){ i = Q.length - 1; para(); }
  sl.value = i; desenha(i);
}
function toca(){
  if (+sl.value >= Q.length - 1){ sl.value = 0; tPlay = Q[0].t; }
  para();
  tUltimo = performance.now();
  timer = setInterval(tique, 33);
  bt.textContent = '❚❚ pausar';
}
function para(){
  if (timer){ clearInterval(timer); timer = null; }
  bt.textContent = '▶ reproduzir';
}
function vaiPara(i){
  i = Math.max(0, Math.min(Q.length - 1, i));
  sl.value = i; tPlay = Q[i].t; desenha(i);
}
bt.onclick = () => timer ? para() : toca();
document.getElementById('passo-tras').onclick = () => { para(); vaiPara(+sl.value - 1); };
document.getElementById('passo-frente').onclick = () => { para(); vaiPara(+sl.value + 1); };
document.getElementById('vels').querySelectorAll('button').forEach(b => {
  b.onclick = () => {
    vel = +b.dataset.v;
    document.getElementById('vels').querySelectorAll('button')
      .forEach(o => o.classList.toggle('sel', o === b));
  };
});
sl.onmousedown = para;
sl.oninput = () => { tPlay = Q[+sl.value] ? Q[+sl.value].t : 0; desenha(+sl.value); };

// ATALHOS: quem analisa replay passa o tempo pulando quadro a quadro.
document.addEventListener('keydown', (e) => {
  if (e.target.tagName === 'INPUT' && e.key !== ' ') return;
  if (e.key === ' '){ e.preventDefault(); timer ? para() : toca(); }
  else if (e.key === 'ArrowLeft'){ para(); vaiPara(+sl.value - (e.shiftKey ? 10 : 1)); }
  else if (e.key === 'ArrowRight'){ para(); vaiPara(+sl.value + (e.shiftKey ? 10 : 1)); }
  else if (e.key === 'Home'){ para(); vaiPara(0); }
  else if (e.key === 'End'){ para(); vaiPara(Q.length - 1); }
});

// ------------------------------------------------------- MARCAS DO TEMPO
//
// Os instantes que interessam ficavam escondidos: para achar o disparo era
// preciso arrastar o controle procurando. Agora cada disparo e o gol tem uma
// marca clicavel sobre a linha do tempo.
(function(){
  const faixa = document.getElementById('marcas');
  const tFim = T_FIM || 1;
  document.getElementById('tfim').textContent = tFim.toFixed(1);

  const poe = (t, classe, titulo) => {
    const m = document.createElement('div');
    m.className = 'marca ' + classe;
    m.style.left = (100 * t / tFim) + '%';
    m.title = titulo + ' · t = ' + t.toFixed(2) + ' s';
    m.onclick = () => { para(); vaiPara(idxDoTempo(t)); };
    faixa.appendChild(m);
  };

  let antes = false;
  (D.eventos || []).forEach(e => {
    const f = !!e.flat_kick;
    if (f && !antes) poe(e.t, 'chute', 'disparo do chutador (robo ' + e.id + ')');
    antes = f;
  });

  // O GOL NAO TEM INSTANTE no payload - 'gol_em' e a POSICAO do cruzamento.
  // O tempo sai do primeiro quadro em que a bola passa da linha, que e a
  // mesma conta que o resumo usa para contar gol.
  if (D.gol){
    const q = Q.find(q => Math.abs(q.b[0]) >= 4500);
    if (q) poe(q.t, 'gol', D.gol === 'nosso' ? 'gol a favor' : 'gol sofrido');
  }
})();

desenha(0);
toca();   // comeca tocando: nada de clicar para ver a execucao
</script></body></html>"""


def _estado_do_feedforward():
    """Em que condicao o feedforward do PID rodou nesta execucao.

    POR QUE ISTO E REGISTRADO: sem o campo, um numero velho nao dizia em que
    condicao rodou - o 'preparar' alterava o controlador e nada ficava gravado.

    Tres estados possiveis, e os tres ja existiram nesta arvore:
        "guarda de sinal"  codigo de 07/10/2026: o feedforward vale so onde
                           concorda com o erro de posicao
        "desligado"        o ajuste de 25/08 aplicado (o termo era zerado)
        "cru"              o codigo sem ajuste nenhum, feedforward inteiro
    Devolve None se nao der para ler - melhor nao registrar nada do que
    registrar um palpite.
    """
    alvo = "/root/ssl-VICE/src/control/control/pid_controller.py"
    try:
        with open(alvo, encoding="utf-8") as fp:
            txt = fp.read()
    except OSError:
        return None
    if "if feedforward * position_error > 0.0:" in txt:
        # a guarda pode estar como codigo ou comentada por um ajuste antigo
        for linha in txt.split("\n"):
            if linha.strip().startswith("if feedforward * position_error"):
                return "guarda de sinal"
    if any(l.strip().startswith("#ORIG#") for l in txt.split("\n")):
        return "desligado"
    return "cru"


def erro_de_rastreio(resultado):
    """Distancia entre o robo e o setpoint que o driver esta comandando.

    E a unica grandeza medida que separa gol de erro: as execucoes que marcaram
    tiveram mediana de 76 e 153 mm, a que falhou teve 306 mm. Como sai de UMA
    execucao, serve para iterar onde contar gols e caro demais.

    O setpoint vem do /control_command (driver.py:33, trajectory.get_state) e a
    posicao do robo vem da visao CRUA do grSim, sem passar pelo tracker.
    """
    alvos = sorted(resultado.get("alvos") or [])
    rc = sorted(resultado.get("robos_crus") or [])
    vc = sorted(resultado.get("visao_crua") or [])
    if not alvos or not rc or not vc:
        return None
    pares = [(r[3], r[0]) for r in vc]          # (t_nosso, t_capture)
    por_tc = {}
    for tc, rid, x, y, _ori in rc:
        por_tc.setdefault(rid, []).append((tc, x, y))
    erros = []
    for reg in alvos:
        tn, rid, ax, ay = reg[0], reg[1], reg[2], reg[3]
        serie = por_tc.get(rid)
        if not serie:
            continue
        tc = min(pares, key=lambda p: abs(p[0] - tn))[1]
        _t, rx, ry = min(serie, key=lambda r: abs(r[0] - tc))
        erros.append(math.hypot(ax - rx, ay - ry))
    if not erros:
        return None
    erros.sort()
    # Taxa real do laco do driver DURANTE esta execucao. Vai junto do resultado
    # de proposito: um erro de rastreio so quer dizer alguma coisa se a gente
    # souber a que taxa o laco estava rodando quando ele foi medido. O timer do
    # driver e 100 Hz; ja medimos 19 Hz com a maquina carregada e 93 Hz com ela
    # livre, e comparar numeros dessas duas situacoes nao significa nada.
    # TAXA POR ROBO, nao a soma de todos.
    #
    # O driver antigo publicava UMA ControlCommand com a lista de todos os
    # robos: uma mensagem por ciclo. O tracker novo publica um TrajectoryPoint
    # POR ROBO. Contando mensagens, um cenario com 3 robos aparecia como
    # "300 Hz" com o timer em 100 - e o relatorio ainda avisava "(timer e 100)",
    # como se a maquina estivesse acelerada. Era so aritmetica errada.
    ts = [x[0] for x in alvos]
    n_robos = len({x[1] for x in alvos}) or 1
    hz = (len(ts) / (ts[-1] - ts[0]) / n_robos) if len(ts) > 2 and ts[-1] > ts[0] else 0.0

    # ERRO POR ROBO, e nao um numero so.
    #
    # A mediana agregada mistura quem importa com quem nao importa: o cobrador
    # faz passos curtos perto da bola, enquanto apoio e goleiro atravessam o
    # campo - e um robo em trajetoria longa fica naturalmente atras do seu
    # setpoint, sem que isso seja defeito. Medido no cenario 'ataque': agregado
    # 41 mm, mas o cobrador sozinho fica bem abaixo e o apoio (que anda 2426 mm)
    # e quem carrega a cauda.
    #
    # Julgar a cobranca pelo numero agregado e olhar o robo errado.
    por_robo = {}
    for reg in alvos:
        tn, rid, ax, ay = reg[0], reg[1], reg[2], reg[3]
        serie = por_tc.get(rid)
        if not serie:
            continue
        tc = min(pares, key=lambda p: abs(p[0] - tn))[1]
        _t, rx, ry = min(serie, key=lambda r: abs(r[0] - tc))
        por_robo.setdefault(rid, []).append(math.hypot(ax - rx, ay - ry))
    detalhe = {}
    for rid, lista in por_robo.items():
        lista.sort()
        detalhe[rid] = {
            "mediana": round(lista[len(lista) // 2], 1),
            "p90": round(lista[int(0.9 * len(lista))], 1),
            "maximo": round(lista[-1], 1),
            "n": len(lista),
        }

    return {
        "hz_controle": round(hz, 1),
        "n_robos": n_robos,
        "n": len(erros),
        "mediana": round(erros[len(erros) // 2], 1),
        "p90": round(erros[int(0.9 * len(erros))], 1),
        "maximo": round(erros[-1], 1),
        "pct_acima_200": round(100.0 * sum(1 for e in erros if e > 200) / len(erros), 1),
        "por_robo": detalhe,
    }


def gerar_replay(resultado, destino):
    """Reconstroi a execucao a partir da visao CRUA, num HTML que roda sozinho.

    POR QUE ISTO EXISTE
    -------------------
    Ver a execucao ao vivo e assistir por um dos dois caminhos que mentem:

      - a janela do grSim congela a fisica (glwidget.cpp:392 chama step() dentro
        do paintGL), entao o que se ve nao e o que a estrategia enfrentaria;
      - a ssl-gui desenha o /visionTopic, que e a saida do filtro de Kalman - o
        mesmo que manteve a bola parada em (2500,0) por 1,2 s DEPOIS do disparo.

    Aqui a fonte e o multicast do grSim, sem filtro nenhum, com o t_capture do
    proprio simulador. E mostra o que nenhuma visualizacao ao vivo mostra: o
    SETPOINT que o driver estava comandando, desenhado junto do robo. A distancia
    entre os dois e o erro de rastreio, que e o que separa gol de erro.
    """
    import json as _json
    # uma leitura de bola por instante (o grSim emite um quadro por camera)
    _vistos = set()
    vc = []
    for r in sorted(resultado.get("visao_crua") or []):
        if r[0] in _vistos:
            continue
        _vistos.add(r[0])
        vc.append(r)
    rc = sorted(resultado.get("robos_crus") or [])
    ac = sorted(resultado.get("amarelos_crus") or [])
    if not vc:
        return None

    t0 = vc[0][0]
    # mapa t_nosso -> t_capture, para trazer alvos e eventos ao mesmo relogio
    pares = [(r[3], r[0]) for r in vc]

    def para_tc(tn):
        melhor = min(pares, key=lambda p: abs(p[0] - tn))
        return melhor[1]

    # CASAR POR PROXIMIDADE, nao por igualdade de instante.
    #
    # O grSim emite um quadro por camera: a bola aparece na camera que a contem
    # e os robos nas suas. Os t_capture sao diferentes, entao casar por chave
    # exata devolve zero quadros completos - foi o que aconteceu na primeira
    # versao (1050 quadros, nenhum com robo E setpoint juntos).
    import bisect
    rob_t = sorted({t for t, _, _, _, _ in rc})
    rob_por_t = {}
    for t, rid, x, y, ori in rc:
        rob_por_t.setdefault(t, []).append([rid, round(x), round(y), round(ori, 4)])

    # mesma indexacao por instante, para os adversarios
    am_t = sorted({t for t, _, _, _, _ in ac})
    am_por_t = {}
    for t, rid, x, y, ori in ac:
        am_por_t.setdefault(t, []).append([rid, round(x), round(y), round(ori, 4)])

    def _perto(t, chaves, mapa, tol=0.03):
        if not chaves:
            return []
        i = bisect.bisect_left(chaves, t)
        cand = [chaves[j] for j in (i - 1, i) if 0 <= j < len(chaves)]
        if not cand:
            return []
        melhor = min(cand, key=lambda x: abs(x - t))
        if abs(melhor - t) > tol:
            return []
        unicos = {}
        for r in mapa[melhor]:
            unicos.setdefault(r[0], r)
        return list(unicos.values())

    def amarelos_perto(t):
        return _perto(t, am_t, am_por_t)

    def robos_perto(t, tol=0.03):
        # (o mesmo robo pode vir em mais de uma camera; um por id basta)
        if not rob_t:
            return []
        i = bisect.bisect_left(rob_t, t)
        cand = [rob_t[j] for j in (i - 1, i) if 0 <= j < len(rob_t)]
        if not cand:
            return []
        melhor = min(cand, key=lambda x: abs(x - t))
        if abs(melhor - t) > tol:
            return []
        unicos = {}
        for r in rob_por_t[melhor]:
            unicos.setdefault(r[0], r)
        return list(unicos.values())

    alvos_t = sorted((para_tc(tn), [rid, round(ax), round(ay)])
                     for tn, rid, ax, ay in
                     ((r[0], r[1], r[2], r[3]) for r in (resultado.get("alvos") or [])))
    eventos = [{"t": round(para_tc(e["t"]) - t0, 4), **e}
               for e in (resultado.get("eventos_chutador") or [])]

    # UM SETPOINT POR ROBO, e nao "o ultimo que passou".
    #
    # BUG QUE ISTO CORRIGE: 'ultimo_alvo' guardava o ultimo alvo visto,
    # QUALQUER que fosse o robo. Com 3 robos publicando setpoint, o alvo
    # desenhado trocava de dono a cada mensagem - e a linha saia do robo A ate o
    # setpoint do robo B. O replay mostrava um erro de rastreio que nao existia,
    # variando a 300 Hz. Era isso o "extremamente falso".
    #
    # Some-se o outro defeito: o desenho usava q.r[0], "o primeiro robo da
    # lista", e essa lista vem de um dicionario - ou seja, QUAL robo aparecia
    # como 'o nosso' mudava de quadro em quadro.
    quadros = []
    ia, alvo_por_robo = 0, {}
    for t, x, y, _tn in vc:
        while ia < len(alvos_t) and alvos_t[ia][0] <= t:
            rid, ax, ay = alvos_t[ia][1]
            alvo_por_robo[rid] = [rid, ax, ay]
            ia += 1
        quadros.append({
            "t": round(t - t0, 4),
            "b": [round(x), round(y)],
            "r": robos_perto(t),
            "y": amarelos_perto(t),
            "a": list(alvo_por_robo.values()),
        })

    dados = _json.dumps({
        "quadros": quadros,
        "eventos": eventos,
        "cenario": resultado.get("cenario"),
        "gol": resultado.get("gol"),
        "gol_em": resultado.get("gol_em"),
        "disparou": resultado.get("disparou"),
        "bola_inicial": resultado.get("bola_inicial"),
        # Quem cobrou, para o replay destacar o robo certo em vez de adivinhar.
        "cobrador": resultado.get("cobrador"),
        # EM QUE CONDICAO ISTO RODOU - o replay tem de dizer sozinho.
        #
        # Estes quatro campos existiam no JSON do resultado e NAO estavam aqui,
        # entao o replay - que e o que o 'medir' le - nao sabia o lado do
        # ataque, as chaves ligadas, nem se houve quadro de outra visao. O
        # 'medir' caia no default (+x) e um lote com o lado trocado teria todo o
        # sinal invertido sem ninguem notar. Mesma familia de armadilha do lote
        # de 21/09, que rodou sem adversario sem a planilha dizer.
        "sentido_ataque": resultado.get("sentido_ataque"),
        "feedforward": resultado.get("feedforward"),
        "chaves_desligadas": resultado.get("chaves_desligadas"),
        "quadros_fora_do_relogio": resultado.get("quadros_fora_do_relogio"),
    }, separators=(",", ":"))

    with open(destino, "w") as f:
        f.write(_MODELO_REPLAY.replace("/*DADOS*/", dados))
    return destino


def gol_pela_visao_crua(visao_crua):
    """Decide o gol pela visao CRUA, nao pelo topico filtrado.

    POR QUE ISTO IMPORTA PARA O CRITERIO DE APROVACAO
    -------------------------------------------------
    O criterio do Felipe e "bola cruzando x>4500 com |y|<500". Ate aqui isso
    era conferido no /visionTopic - ou seja, na saida do filtro de Kalman.

    Medimos o filtro ULTRAPASSAR a posicao real em ate 431 mm ao frear a bola
    (ele estima velocidade e continua projetando depois que ela ja parou). Com
    431 mm de ultrapassagem, uma bola que morre em x=4300 pode ser publicada
    cruzando x=4500 - e contada como gol que nao houve. O erro tambem funciona
    ao contrario, escondendo um gol de raspao.

    Contando na fonte crua, gol e gol.

    Devolve ("nosso"|"contra"|None, x, y) do primeiro cruzamento.
    """
    for tc, x, y, _tn in sorted(visao_crua or []):
        if abs(y) <= GOL_MEIA_LARGURA:
            if x >= GOL_X:
                return "nosso", x, y
            if x <= -GOL_X:
                return "contra", x, y
    return None, 0.0, 0.0


def imprimir_energia(e):
    if not e:
        print("   ENERGIA DO CHUTE: a bola nao se moveu o bastante para medir")
        return
    print("   ENERGIA DO CHUTE (derivada das posicoes):")
    print(f"     fonte: {e['fonte']}")
    if e["t_pedido_chute"] is not None:
        print(f"     estrategia pediu chute em t={e['t_pedido_chute']:.3f}s"
              f"   bola partiu em t={e['t_partida']:.3f}s")
    seq = "  ".join(f"{v:.0f}" for v in e["v_primeiros_quadros"])
    print(f"     primeiros quadros apos a partida: {seq} mm/s")
    print(f"     VELOCIDADE DE SAIDA (janela 50 ms): {e['v_saida_janela_50ms']:.0f} mm/s"
          f"  = {e['v_saida_janela_50ms']/1000.0:.2f} m/s")
    print(f"     pico quadro a quadro: {e['v_pico_quadro']:.0f} mm/s"
          f"   pico pelo Kalman: {e['v_pico_kalman']:.0f} mm/s")
    print(f"     a bola percorreu {e['percorrido_mm']:.0f} mm")
    print(f"     -> {e['veredito']}")
    ev = e.get("eventos_chutador") or []
    if ev:
        print("   CHUTADOR, pelo proprio grSim (verdade, nao inferencia):")
        for x in ev[:12]:
            marcas = []
            if x["infrared"]:
                marcas.append("bola encostada")
            if x["flat_kick"]:
                marcas.append("DISPAROU")
            if x["chip_kick"]:
                marcas.append("chip")
            print(f"     t={x['t']:6.3f}s  robo {x['id']}: "
                  f"{', '.join(marcas) if marcas else 'solto'}")
        if len(ev) > 12:
            print(f"     ... e mais {len(ev) - 12} eventos (veja o JSON)")
    else:
        print("   CHUTADOR: nenhum evento recebido do grSim "
              f"(porta {PORTA_STATUS_AZUL})")


def _girar(no, segundos):
    import rclpy

    fim = time.time() + segundos
    while time.time() < fim:
        rclpy.spin_once(no, timeout_sec=0.05)


def esperar_assentar(no, limite=8.0, vel_max=25.0, piso=0.0, erro_max=120.0):
    """Espera o driver REANCORAR, em vez de dormir um tempo fixo.

    POR QUE ESPERA ATIVA
    --------------------
    Isto era 'durma 8 segundos'. O numero foi escolhido com folga, para o pior
    caso - e o pior caso e raro. Na maioria das execucoes os robos ja estao
    parados em 1 ou 2 segundos, e os 6 restantes eram desperdicio puro,
    multiplicado por cada repeticao de cada cenario.

    O QUE ESTA ESPERANDO DE VERDADE
    -------------------------------
    O driver cria a trajetoria de um robo uma vez e dali em diante replaneja a
    partir do setpoint do plano anterior, NUNCA da posicao medida. Depois de um
    teleporte ele fica planejando a partir de uma origem fantasma. Sob HALT, a
    tatica manda cada robo para a PROPRIA posicao lida da visao: o driver
    persegue esse alvo e, quando o setpoint dele alcanca a posicao real, a
    ancora esta corrigida.

    ERRO QUE ISTO CORRIGE
    ---------------------
    A primeira versao esperava os robos PARAREM. So que, logo depois de um
    teleporte, eles ja estao parados - nunca se moveram. A condicao era
    satisfeita em 0,3 s, o driver nao recebia o tempo de reancorar, e o
    resultado era o robo andando 112 mm em 20 s: a ancora continuava na posicao
    anterior ao teleporte. O sintoma parecia "os robos travaram".

    A condicao certa nao e o robo estar parado, e o SETPOINT DO DRIVER ter
    convergido para a posicao medida. E isso que medimos agora.
    """
    import rclpy
    inicio = time.time()
    fim = inicio + limite
    ok_desde = None
    while time.time() < fim:
        rclpy.spin_once(no, timeout_sec=0.05)
        if time.time() - inicio < piso:
            continue
        sp = getattr(no, "setpoints", None)
        pos = getattr(no, "azuis_pos", None)
        vels = getattr(no, "velocidades", None)
        if not sp or not pos or not vels:
            continue
        comuns = [i for i in sp if i in pos]
        if not comuns:
            continue
        erro = max(math.hypot(sp[i][0] - pos[i][0], sp[i][1] - pos[i][1])
                   for i in comuns)
        parado = max(vels.values()) < vel_max
        if erro < erro_max and parado:
            # exige estabilidade por 3 decimos: uma leitura isolada boa
            # acontece a toa quando um quadro de visao se repete.
            ok_desde = ok_desde or time.time()
            if time.time() - ok_desde > 0.3:
                return time.time() - inicio
        else:
            ok_desde = None
    return limite


def conferir_teleporte(no, alvo, tolerancia=200.0, limite=45.0):
    """Espera a bola aparecer no alvo em /visionTopic. Devolve (ok, pos, atraso).

    Serve de controle positivo E de medida do atraso da visao.

    O limite e generoso de proposito: o visionNode do ssl-VICE le UM datagrama
    por callback do timer (vision_node.py:74 + vision_client.py:76). Com ~21
    pacotes/s chegando do grSim e o callback nao acompanhando, o socket acumula
    e o atraso cresce sem parar - ja medimos mais de 20s. Para comparacao, o
    teleporte medido direto contra o grSim, sem ROS no meio, leva 0,2s.
    """
    alvo_x, alvo_y = alvo
    t0 = time.time()
    fim = t0 + limite
    while time.time() < fim:
        _girar(no, 0.3)
        if no.bola is None:
            continue
        if math.hypot(no.bola[0] - alvo_x, no.bola[1] - alvo_y) <= tolerancia:
            return True, no.bola, time.time() - t0
    return False, no.bola, time.time() - t0


# ==========================================================================
#  Execucao de um cenario
# ==========================================================================
def rodar(nome, duracao=12.0):
    if nome not in CENARIOS:
        print(f"Cenario desconhecido: {nome}", file=sys.stderr)
        return 2

    cen = CENARIOS[nome]
    perfil = os.environ.get("CAMPO", "original")

    print(f"\n>> {cen['titulo']}")
    print(f"   {cen['descricao']}")
    print(f"   bola em x={cen['bola'][0]:.0f} y={cen['bola'][1]:.0f} mm")

    import rclpy

    alvo = cen["bola"]

    rclpy.init()
    no = _criar_gravador()

    try:
        pedido = alvo
        ok, onde, atraso = conferir_teleporte(no, alvo, limite=25.0)
        if ok:
            print(f"   cenario confirmado pela visao em {atraso:.1f}s")
            print("   assentando sob HALT (driver reancora as trajetorias)...")
            enviar_comando_arbitro("HALT")
            gasto = esperar_assentar(no, limite=ESPERA_HALT, piso=1.5)
            print(f"   assentado em {gasto:.1f}s (teto era {ESPERA_HALT:.0f}s)")

        if not ok:
            visto = (
                f"({onde[0]:.0f}, {onde[1]:.0f})" if onde else "nenhuma leitura"
            )
            print()
            print("   XX A BOLA NAO CHEGOU AO LUGAR PEDIDO.")
            print(f"      pedido: ({pedido[0]:.0f}, {pedido[1]:.0f})"
                  f"   visto: {visto}")
            print()
            print("   Duas causas possiveis:")
            print()
            print("   1) SIMULACAO CONGELADA. O grSim so avanca a fisica")
            print("      quando a janela dele e redesenhada (glwidget.cpp:392")
            print("      chama step() dentro de paintGL). Janela minimizada,")
            print("      coberta ou tela bloqueada = mundo parado, e ele segue")
            print("      reenviando o ultimo quadro, entao parece normal.")
            print("      -> traga a janela do grSim para a frente, ou suba com")
            print("         ./ararabots.sh grsim --headless")
            print()
            print("   2) O visionNode nao esta publicando (ver HANDOVER §15).")
            print("      -> ./ararabots.sh parar  e depois  ./ararabots.sh preparar")
            print()
            return 3

        print(f"   bola confirmada em ({no.bola[0]:.0f}, {no.bola[1]:.0f})")

        tipo, cor = cen["comando"]
        tipo_cen = tipo.upper()
        cor_cen = cor.upper()

        # 3. SEÇÃO MODIFICADA: Mapeamento do comportamento do adversário
        if tipo_cen in ("DIRECT", "INDIRECT"):
            modo_adv = "nossa_falta" if cor_cen == "BLUE" else "falta_deles"
        elif tipo_cen in ("KICKOFF", "PREPARE_KICKOFF"):
            modo_adv = "nosso_kickoff" if cor_cen == "BLUE" else "kickoff_deles"
        else:
            modo_adv = "jogo"

        adversario_ligado = os.environ.get("ADVERSARIO", "1") != "0"
        print(f"   adversario: {'ativo (' + modo_adv + ')' if adversario_ligado else 'desligado'}")

        # AQUECIMENTO DO ADVERSARIO - o goleiro ja entra varrendo.
        #
        # Pedido do Felipe: "o goleiro deve se mover 5 segundos antes e
        # continuar movendo". Antes, o laco que comanda os amarelos so comecava
        # junto com a gravacao, e o grSim ZERA a velocidade do robo em todo
        # passo de fisica sem comando novo (§9-0000000, item 4): o goleiro
        # partia parado exatamente no instante em que a jogada comecava.
        #
        # Aqui ele recebe comando a ~50 Hz durante PRE_VARREDURA segundos ANTES
        # do comando do arbitro. Quando a cobranca comeca ele ja esta em
        # movimento, no meio do percurso, com inercia - que e a condicao contra
        # a qual o numero de gol precisa ser medido.
        #
        # O AQUECIMENTO ACONTECE SOB HALT, NAO SOB STOP - e isto NAO e detalhe.
        #
        # BUG QUE EU MESMO CRIEI E MEDI: na primeira versao o aquecimento vinha
        # DEPOIS do STOP. Sob STOP a arvore roda a tatica Stop, que manda todos
        # os robos para um alvo FIXO em (2000,1400) (o defeito do HANDOVER §6.5,
        # que continua la). Cinco segundos extras de STOP e tempo de sobra para
        # o time inteiro sair do lugar em que o cenario o colocou.
        #
        # MEDIDO no cenario 'passe': no instante t=0 da gravacao o cobrador ja
        # estava em (852,976) em vez de (-400,0) e o receptor em (1996,1366) em
        # vez de (1600,400), os dois a caminho de (2000,1400). A bola nao saiu
        # do lugar em 6 de 6 execucoes - nao porque o passe falhasse, mas porque
        # nao havia cobranca nenhuma acontecendo.
        #
        # Sob HALT, halt.py manda cada robo para a PROPRIA posicao lida da
        # visao, ou seja, os segura parados (HANDOVER §7). O goleiro adversario
        # e comandado por nos, por fora da arvore, entao ele varre normalmente.
        # Depois do aquecimento vem o STOP curto de sempre, tambem comandando os
        # amarelos, e so entao o comando da falta.
        if adversario_ligado and modo_adv != "jogo":
            print(f"   aquecendo o adversario por {PRE_VARREDURA:.0f}s "
                  f"(goleiro ja varrendo quando a jogada comecar)...")
        # O AQUECIMENTO NAO VALE PARA O JOGO CORRIDO.
        #
        # Ele existe para a BOLA PARADA: o goleiro adversario precisa ja estar
        # varrendo quando a cobranca comeca, porque o grSim zera a velocidade de
        # quem nao recebe comando novo a cada passo de fisica.
        #
        # Em jogo corrido o perfil do adversario e ATACAR A BOLA. Aquecer por 5 s
        # antes de comecar a gravar significa deixa-lo jogar sozinho cinco
        # segundos: medido nos replays, a partida comecava com a bola ja em
        # x=-1438 e -1311, e num deles havia um CHUTE amarelo no instante 0,0 s.
        # Nao ha cenario nenhum sendo medido ali - o jogo ja aconteceu.
        aquecer = adversario_ligado and modo_adv != "jogo"
        fim_pre = time.time() + (PRE_VARREDURA if aquecer else 0.0)
        while time.time() < fim_pre:
            _girar(no, 0.02)
            if adversario_ligado and no.bola:
                comandar_amarelos(no.bola, no.amarelos,
                                  getattr(no, "azuis_pos", {}), modo_adv,
                                  getattr(no, "bola_vel", (0.0, 0.0)))

        # Transicao obrigatoria para STOP, curta como sempre foi, ja com os
        # amarelos sendo comandados para o goleiro nao parar entre as fases.
        enviar_comando_arbitro("STOP")
        fim_stop = time.time() + 1.5
        while time.time() < fim_stop:
            _girar(no, 0.02)
            if adversario_ligado and no.bola:
                comandar_amarelos(no.bola, no.amarelos,
                                  getattr(no, "azuis_pos", {}), modo_adv,
                                  getattr(no, "bola_vel", (0.0, 0.0)))

        # A GEOMETRIA DO CENARIO TEM DE VALER NO INSTANTE t=0.
        #
        # DEFEITO QUE ISTO CORRIGE, medido em 07/10/2026 no lote do portador.
        # Entre a confirmacao da bola e o inicio da gravacao havia duas janelas
        # em que o jogo ja estava rodando: 1,5 s de STOP - e sob STOP a arvore
        # manda todos os robos para um alvo FIXO em (2000,1400), ver o comentario
        # do aquecimento - e ate 3 s esperando o comando aparecer no
        # /refereeTopic. Com o robo longe da bola isso nao aparece; com ele
        # perto, ele JOGA nessa janela.
        #
        # MEDIDO, deriva da bola entre o que o cenario pede e o quadro t=0:
        #     portador_bola_atras_150     496 e 597 mm   (pedido: 150)
        #     portador_bola_atras_400     289 e 398 mm   (pedido: 400)
        #     os outros seis cenarios     0 a 6 mm
        #
        # Nos dois primeiros o robo chegava a t=0 JA EM CONTATO com a bola
        # (115 e 144 mm), e as duas condicoes do par partiam de geometrias
        # diferentes - o antes/depois media duas largadas, nao duas taticas.
        #
        # Reposicionar aqui, depois do STOP e antes de gravar, zera as duas
        # janelas de uma vez: o que o cenario escreveu e o que o replay ve.
        print("   reposicionando o cenario (a geometria tem de valer no t=0)")
        posicionar(cen)
        for _ in range(10):
            _girar(no, 0.02)

        # A GRAVACAO COMECA ANTES DO COMANDO, NAO DEPOIS.
        #
        # A espera pelo comando no /refereeTopic leva ate 3 s, e nessa janela o
        # jogo JA ESTA RODANDO - era a maior das duas fontes de deriva medidas
        # acima. Gravando antes, o replay guarda a geometria limpa em t=0 e a
        # jogada inteira; a espera entra como um pre-rolo de poucos decimos em
        # que nada se move, e o tempo gasto nela e devolvido ao fim da gravacao.
        no.t0 = time.monotonic()
        no.gravando = True

        # 2. SEÇÃO MODIFICADA: Envio do comando de arbitragem conforme a regra
        if tipo_cen in ("KICKOFF", "PREPARE_KICKOFF"):
            print(f"   comando do arbitro: PREPARE_KICKOFF {cor_cen} -> NORMAL_START")
            enviar_comando_arbitro("PREPARE_KICKOFF", cor_cen)
            # Comandando os amarelos tambem AQUI: um _girar seco deixaria o
            # goleiro parado por 2 s bem na vespera do NORMAL_START, desfazendo
            # o aquecimento que acabou de acontecer.
            fim_pk = time.time() + 2.0   # tempo para o posicionamento de kickoff
            while time.time() < fim_pk:
                _girar(no, 0.02)
                if adversario_ligado and no.bola:
                    comandar_amarelos(no.bola, no.amarelos,
                                      getattr(no, "azuis_pos", {}), modo_adv,
                                      getattr(no, "bola_vel", (0.0, 0.0)))
            enviar_comando_arbitro("NORMAL_START")
        else:
            print(f"   comando do arbitro: {tipo} {cor}")
            enviar_comando_arbitro(tipo, cor)

        # O COMANDO CHEGOU? Se nao, NAO grava.
        #
        # O referee_node as vezes publica comando VAZIO mesmo com o ssl-gc de pe
        # e respondendo na 8081. Quando isso acontece o CheckState da arvore
        # recusa todas as jogadas - ele espera FORCE_START ou NORMAL_START e
        # recebe '' - e o time inteiro fica imovel.
        #
        # O replay resultante e uma armadilha: a bola para em x=-1085 porque
        # BATE no nosso robo 1 parado onde nasceu, e a analise diz "nenhum chute
        # nosso, o adversario domina". Isso queimou QUATRO lotes desta fase, e em
        # dois deles o resultado foi reportado como efeito da tatica.
        #
        # Medido na ultima vez: 1328 ciclos com cmd=''. Agora falha na cara.
        cmd_visto = no.comando_arbitro if hasattr(no, "comando_arbitro") else None
        # QUANTO SE ESPEROU PELO ARBITRO: a gravacao ganha esse tempo de volta.
        # Sem isto, cada segundo de espera era um segundo a menos de jogada
        # gravada, e isso variava de execucao para execucao.
        _t_ini_esp = time.time()
        t_esp = _t_ini_esp + 3.0
        while time.time() < t_esp:
            rclpy.spin_once(no, timeout_sec=0.05)
            cmd_visto = getattr(no, "comando_arbitro", None)
            if cmd_visto:
                break
        if not cmd_visto:
            print()
            print("   XX O ARBITRO NAO ESTA PUBLICANDO COMANDO (veio vazio).")
            print("      A arvore recusa TODAS as jogadas e o time fica imovel;")
            print("      o replay pareceria 'tatica ruim'. Nao vou gravar.")
            print()
            print("      ./ararabots.sh parar && ./ararabots.sh preparar --headless")
            print()
            return 3
        espera_cmd = time.time() - _t_ini_esp

        print(f"   gravando por {duracao:.0f}s (olhe a janela do grSim)...")
        fim = time.time() + duracao + espera_cmd
        proximo_cmd = 0.0
        while time.time() < fim:
            # 0,02 s, e nao 0,1: a CADENCIA DO LACO e quem manda na taxa de
            # comando do adversario, nao o portao que existia embaixo.
            #
            # Eu tinha tirado o portao de 10 Hz achando que resolvia, e nao
            # mudou nada - porque _girar(0,1) bloqueia 100 ms por volta e o
            # laco continuava a 10 Hz. Com 0,02 o laco vai a ~50 Hz.
            _girar(no, 0.02)
            # COMANDA A CADA VOLTA.
            #
            # O grSim ZERA a velocidade do robo no passo de fisica em que nao
            # chega comando novo. A 10 Hz o adversario andava um passo e ficava
            # parado os outros 90 ms: o resultado e o tremor de 0,000-0,001 que
            # o Felipe viu, e nao movimento.
            #
            # MEDIDO, com o mesmo codigo nos dois casos: comandando a 10 Hz o
            # goleiro deslocou 7 mm em 5 s; um laco enviando a ~50 Hz moveu o
            # mesmo robo 250 mm em 3 s.
            #
            # Enviar todo ciclo custa um datagrama de ~30 bytes por volta -
            # nada perto de mandar o teste inteiro por agua abaixo.
            if adversario_ligado and no.bola:
                # DIAGNOSTICO temporario: quantos amarelos o gravador enxerga e
                # se a patrulha esta ligada AQUI DENTRO. Ja perdemos horas com o
                # goleiro parado por a variavel nao atravessar o docker exec.
                if os.environ.get("DIAG_ADV") and int(time.time() * 2) % 4 == 0:
                    print("   [diag] amarelos=%d patrulha=%r bola=%s"
                          % (len(no.amarelos), os.environ.get("GOLEIRO_PATRULHA"),
                             tuple(round(v) for v in no.bola)), flush=True)
                comandar_amarelos(no.bola, no.amarelos,
                                  getattr(no, "azuis_pos", {}), modo_adv,
                                  getattr(no, "bola_vel", (0.0, 0.0)))
        no.gravando = False

        enviar_comando_arbitro("HALT")
        amostras, kicks = no.amostras, no.kick_por_robo
        eventos_chutador = list(no.eventos_chutador)
        pedido_chute = dict(no.t_pedido_chute)
        visao_crua = list(no.visao_crua)
        alvos = list(no.alvos)
        robos_crus = list(no.robos_crus)
        amarelos_crus = list(no.amarelos_crus)
        janelas_kick = list(no.janelas_kick)
        for sk in (no.sock_status, no.sock_visao):
            if sk is not None:
                sk.close()
    finally:
        no.destroy_node()
        rclpy.shutdown()

    kick_ativado = any(v > 0 for v in kicks.values())
    energia = energia_do_chute(amostras, eventos_chutador, pedido_chute,
                               visao_crua)
    disparou = next((x for x in eventos_chutador if x.get("flat_kick")), None)
    janela = janela_do_chutador(visao_crua, robos_crus, janelas_kick,
                                eventos_chutador)
    _res_parcial = {"alvos": alvos, "robos_crus": robos_crus, "visao_crua": visao_crua}
    rastreio_calc = erro_de_rastreio(_res_parcial)
    gol_topico = no.gol
    gol_cru, gol_x, gol_y = gol_pela_visao_crua(visao_crua)
    gol_final = gol_cru if visao_crua else gol_topico

    perfis = {
        "original": {"goal_x": 2250.0, "kick_threshold": 1125.0},
        "divb": {"goal_x": 4500.0, "kick_threshold": 2250.0},
        "codigo": {"goal_x": 4500.0, "kick_threshold": 2250.0},
    }
    const = perfis.get(perfil, perfis["codigo"])
    # Quem cobrou NESTA execucao: o robo que mais se aproximou da bola.
    # Calculado aqui, antes do resultado, porque tanto o relatorio quanto o
    # replay dependem dele. E inferencia do que aconteceu, nao palpite.
    cobrador_medido = None
    _cont = getattr(no, "contato", None)
    if _cont:
        cobrador_medido = min(_cont, key=lambda r: _cont[r][0])

    resultado = {
        "cenario": nome,
        "branch": os.environ.get("BRANCH", "?"),
        "ordem": list(CENARIOS).index(nome),
        "titulo": cen["titulo"],
        "descricao": cen["descricao"],
        "perfil_campo": perfil,
        "goal_x": const["goal_x"],
        "kick_threshold": const["kick_threshold"],
        "bola_inicial": {"x": cen["bola"][0], "y": cen["bola"][1]},
        "comando": f"{tipo} {cor}",
        "duracao": duracao,
        "kick_por_robo": {str(k): v for k, v in kicks.items()},
        # Quem cobrou: o robo que mais se aproximou da bola. O replay usa isto
        # para destacar o robo certo em vez de desenhar "o primeiro da lista".
        "cobrador": cobrador_medido,
        "kick_ativado": kick_ativado,
        "disparou": bool(disparou),
        "gol": gol_final,
        "gol_pelo_topico": gol_topico,
        "gol_pela_visao_crua": gol_cru,
        "gol_em": {"x": gol_x, "y": gol_y} if gol_cru else None,
        "amostras": amostras,
        "eventos_chutador": eventos_chutador,
        "visao_crua": visao_crua,
        "robos_crus": robos_crus,
        "amarelos_crus": amarelos_crus,
        "alvos": alvos,
        "janela_chutador": janela,
        "janelas_kick": janelas_kick,
        "rastreio": rastreio_calc,
        "t_pedido_chute": {str(k): v for k, v in pedido_chute.items()},
        "energia": energia,
        "registrado_em": time.strftime("%Y-%m-%d %H:%M:%S"),
        # EM QUE CONDICAO ISTO RODOU.
        #
        # O 'preparar' aplica um ajuste em src/control/ que DESLIGA o
        # feedforward do PID, e nada registrava isso: um numero de hoje nao
        # dizia se veio do codigo commitado ou do codigo com o patch. Agora o
        # proprio replay diz. Replays antigos nao tem o campo - o visualizador
        # entao nao mostra o selo, em vez de inventar um default.
        "feedforward": _estado_do_feedforward(),
        # PARA QUE LADO ATACAMOS nesta execucao, para o avanco ter sinal.
        "sentido_ataque": 1.0 if const["goal_x"] >= 0 else -1.0,
        # QUANTOS QUADROS VIERAM DE OUTRO REMETENTE (ver TOL_RELOGIO). Zero e o
        # normal; qualquer numero grande invalida as metricas de deslocamento.
        "quadros_fora_do_relogio": getattr(no, "quadros_fora_do_relogio", 0),
        # QUAIS CHAVES DE EXPERIMENTO ESTAVAM LIGADAS.
        #
        # A planilha nao registrava a condicao do codigo, e foi assim que um
        # lote de 5 gols em 6 ficou tres semanas sem ninguem saber que tinha
        # rodado sem adversario. Agora a propria execucao carrega as chaves.
        "chaves_desligadas": sorted(
            k[len("ARARABOTS_SEM_"):] for k in os.environ
            if k.startswith("ARARABOTS_SEM_") and os.environ[k]),
    }

    os.makedirs(SAIDA_DIR, exist_ok=True)
    branch = os.environ.get("BRANCH", perfil)
    destino = os.path.join(SAIDA_DIR, f"{nome}__{branch}.json")
    with open(destino, "w") as f:
        json.dump(resultado, f)

    # --- resumo no terminal ---
    print(f"   amostras gravadas: {len(amostras)}")
    if gol_final == "nosso":
        print(f"   *** GOL A FAVOR ***  (cruzou em x={gol_x:.0f} y={gol_y:.0f})")
    elif gol_final == "contra":
        print(f"   *** GOL CONTRA ***  (cruzou em x={gol_x:.0f} y={gol_y:.0f})")
    else:
        print("   sem gol")
    if visao_crua and gol_topico != gol_cru:
        print(f"   !! o /visionTopic dizia '{gol_topico}' e a visao crua diz "
              f"'{gol_cru}' - vale a crua")
    if getattr(no, "contato", None):
        print("   CONTATO (limites do grSim: xx<31,5  yy<40):")
        for rid, (d, xx, yy) in sorted(no.contato.items()):
            veredito = "PODIA CHUTAR" if (xx < 31.5 and yy < 40.0) else (
                "torto (yy=%.0f)" % yy if yy >= 40.0 else "longe (xx=%.0f)" % xx)
            print("      robo %d: aproximou %4.0f mm  xx=%5.1f  yy=%5.1f  -> %s"
                  % (rid, d, xx, yy, veredito))
    print(f"   CHUTE: {'ATIVADO' if kick_ativado else 'nao ativado'}", end="")
    if kicks:
        detalhe = ", ".join(f"r{k}={v:.1f}" for k, v in sorted(kicks.items()))
        print(f"  ({detalhe})")
    else:
        print()

    if disparou:
        print(f"   DISPARO (grSim): SIM, robo {disparou['id']} em t={disparou['t']:.3f}s")
    else:
        print("   DISPARO (grSim): NAO - a estrategia pediu, o chutador nao disparou")

    rastreio = rastreio_calc
    # Quem foi o cobrador NESTA execucao: o robo que mais se aproximou da bola.
    # E uma inferencia do resultado, nao um palpite - serve so para o relatorio
    # apontar qual linha olhar.
    if rastreio:
        print("   RASTREIO (robo x setpoint): agregado mediana %.0f mm  p90 %.0f  max %.0f"
              % (rastreio["mediana"], rastreio["p90"], rastreio["maximo"]))
        # POR ROBO, com o cobrador destacado.
        #
        # O agregado mistura o cobrador (passos curtos junto da bola) com apoio
        # e goleiro (travessias de campo inteiro, que ficam naturalmente atras
        # do setpoint). Julgar a cobranca pelo agregado e olhar o robo errado.
        det = rastreio.get("por_robo") or {}
        if det:
            for rid in sorted(det):
                d = det[rid]
                marca = ""
                if cobrador_medido is not None and rid == cobrador_medido:
                    marca = "  <- COBRADOR (e este que decide a jogada)"
                print("      robo %d: mediana %4.0f  p90 %4.0f  max %5.0f%s"
                      % (rid, d["mediana"], d["p90"], d["maximo"], marca))
        print("      acima de 200 mm em %.0f%% do tempo, agregado  (alvo p/ o cobrador:"
              " mediana <100, p90 <250)" % rastreio["pct_acima_200"])
        print("      laco a %.0f Hz por robo, %d robos (timer do controle e 100)"
              % (rastreio["hz_controle"], rastreio.get("n_robos", 1)))
    imprimir_janela(janela)
    imprimir_energia(energia)

    if amostras:
        prim, ult = amostras[0], amostras[-1]
        print("   deslocamento dos nossos robos (mm):")
        pos_ini = {r["id"]: r for r in prim["azuis"]}
        for r in ult["azuis"]:
            ini = pos_ini.get(r["id"])
            if not ini:
                continue
            d = math.hypot(r["x"] - ini["x"], r["y"] - ini["y"])
            papel = " (goleiro)" if r["id"] == 0 else ""
            print(
                f"      robo {r['id']}{papel}: "
                f"({ini['x']:7.0f},{ini['y']:7.0f}) -> "
                f"({r['x']:7.0f},{r['y']:7.0f})   andou {d:6.0f}"
            )
        if prim["bola"] and ult["bola"]:
            b0, b1 = prim["bola"][0], ult["bola"][0]
            db = math.hypot(b1["x"] - b0["x"], b1["y"] - b0["y"])
            print(f"      BOLA: ({b0['x']:.0f},{b0['y']:.0f}) -> "
                  f"({b1['x']:.0f},{b1['y']:.0f})   andou {db:.0f}")
            # AVANCO ASSINADO, no eixo de ataque.
            #
            # 'andou' e modulo, e modulo nao distingue "a bola foi para o gol
            # deles" de "a bola foi para o NOSSO". Foi exatamente o que escondeu
            # o resultado do lote de 03/10: a orbita aparecia como 'pior'
            # (139 mm contra 1302) quando o que ela tinha feito era deixar de
            # empurrar a bola 1087 mm PARA O NOSSO CAMPO.
            _sent = 1.0 if resultado.get("sentido_ataque", 1.0) >= 0 else -1.0
            print(f"      AVANCO (eixo de ataque): "
                  f"{(b1['x'] - b0['x']) * _sent:+.0f} mm")

    _fora = resultado.get("quadros_fora_do_relogio") or 0
    if _fora:
        print(f"   !! {_fora} quadros de visao DESCARTADOS: outro remetente no "
              f"grupo multicast (relogio de captura divergente).")
        print("      Se este numero for grande, ha um segundo grSim/ssl-vision "
              "de pe - ver TOL_RELOGIO.")

    print(f"   salvo em {destino}")
    try:
        html = destino[:-5] + ".html"
        if gerar_replay(resultado, html):
            print(f"   REPLAY VISUAL: {html}")
            print("      (visao CRUA do grSim, com o setpoint do driver desenhado)")
    except Exception as exc:
        print(f"   (replay nao gerado: {exc})")
    return 0
    
def _ferramenta_cadeia():
    """Conta mensagens em cada topico e diz o que esta vivo na cadeia."""



    JANELA = 5.0


    class Verificador(Node):
        def __init__(self):
            super().__init__("verificador_cadeia")
            self.n = {"visao": 0, "estado": 0, "arbitro": 0, "controle": 0, "time": 0}
            self.visao = None
            self.estado = None
            self.comando_arbitro = None

            self.topicos = ["visionTopic", "game_state", "refereeTopic",
                            "control_command", "commandTopic"]
            self.subs = [
                self.create_subscription(VisionMessage, "visionTopic", self._visao, 10),
                self.create_subscription(GameState, "game_state", self._estado, 10),
                self.create_subscription(RefereeMessage, "refereeTopic", self._arbitro, 10),
                _assinar_setpoint(self, self._controle_novo),
                self.create_subscription(TeamCommand, "commandTopic", self._time, 10),
            ]

        def _visao(self, m):
            self.n["visao"] += 1
            self.visao = (len(m.blue_robots), len(m.yellow_robots), len(m.balls))

        def _estado(self, m):
            self.n["estado"] += 1
            self.estado = (len(m.ally_robots), len(m.enemy_robots), len(m.balls),
                           m.geometry.field_length)
            self.comando_arbitro = m.referee.command

        def _arbitro(self, m):
            self.n["arbitro"] += 1

        def _controle(self, m):
            self.n["controle"] += 1

        def _controle_novo(self):
            # _assinar_setpoint entrega um callback SEM argumento (ele ja
            # aplicou o filtro de 'command' quando o topico e o antigo).
            self.n["controle"] += 1

        def _time(self, m):
            self.n["time"] += 1


    def main():
        rclpy.init()
        no = Verificador()

        # ESPERAR A DESCOBERTA ANTES DE CONTAR.
        #
        # BUG QUE ISTO CORRIGE: este node nascia e comecava a contar na mesma
        # hora. Mas um node recem-criado ainda nao descobriu os publicadores -
        # e sob carga isso leva segundos. Topicos publicando normalmente
        # apareciam como "SEM DADOS  0.0 Hz", e o diagnostico mandava reiniciar
        # o grSim e recriar o container do arbitro sem que houvesse problema
        # nenhum. Confirmado: com /visionTopic acusando 0.0 Hz aqui, um
        # 'ros2 topic echo /visionTopic --once' devolvia dados na hora.
        #
        # Agora esperamos cada assinatura enxergar pelo menos um publicador,
        # e so entao a janela de contagem comeca.
        limite = time.time() + 15.0
        while time.time() < limite:
            rclpy.spin_once(no, timeout_sec=0.05)
            # rclpy Humble: a contagem de publicadores vem do NODE, por topico
            # (a Subscription nao expoe get_publisher_count).
            if all(no.count_publishers(t) > 0 for t in no.topicos):
                break
        # zera o que tiver chegado durante a espera: a janela tem de ser limpa
        for k in no.n:
            no.n[k] = 0

        fim = time.time() + JANELA
        while time.time() < fim:
            rclpy.spin_once(no, timeout_sec=0.02)

        def linha(nome, chave, detalhe=""):
            qtd = no.n[chave]
            marca = "✓" if qtd > 0 else "✗"
            hz = qtd / JANELA
            print(f"      {marca} {nome:<24} {hz:6.1f} Hz  {detalhe}")

        v = no.visao
        linha("/visionTopic", "visao",
              f"azuis={v[0]} amarelos={v[1]} bolas={v[2]}" if v else "SEM DADOS")

        e = no.estado
        if e:
            campo = f"campo={e[3]}mm" if e[3] else "campo=? (geometria nao chegou)"
            linha("/game_state", "estado", f"aliados={e[0]} inimigos={e[1]} {campo}")
        else:
            linha("/game_state", "estado", "SEM DADOS")

        linha("/refereeTopic", "arbitro", f"comando={no.comando_arbitro!r}")
        # Um caminho so desde que a dev removeu o driver.
        linha("/movement_manager/commands", "controle",
              "(a estrategia comandando -> manager)")
        linha("/commandTopic", "time", "(velocidades -> grSim)")

        print()
        if no.n["visao"] == 0:
            print("      ! Visao parada. Causas comuns:")
            print("        - grSim com a janela coberta/minimizada (a fisica congela)")
            print("        - grSim precisa ser reiniciado apos suspensao da maquina")
        if no.n["arbitro"] == 0:
            print("      ! Arbitro nao chega ao ROS 2. Recrie o container:")
            print("        docker rm -f ssl-gc  (o ararabots.sh preparar ja faz isso)")
        if e and e[0] == 0:
            print("      ! game_state sem aliados: a estrategia nao tem sobre quem agir.")

        no.destroy_node()
        rclpy.shutdown()

    # O arquivo original chamava isto no seu 'if __name__'; aqui a chamada
    # precisa ser explicita, senao o subcomando define tudo e nao executa nada.
    main()

def _ferramenta_esperar():
    """Bloqueia ate a estrategia COMANDAR de verdade (nao apenas existir)."""

    LIMITE = float(sys.argv[2]) if len(sys.argv) > 2 else 60.0


    class Espera(Node):
        def __init__(self):
            super().__init__("espera_estrategia")
            self.comandos = 0
            _assinar_setpoint(self, self._contar)

        def _contar(self):
            self.comandos += 1


    rclpy.init()
    no = Espera()
    fim = time.time() + LIMITE
    while time.time() < fim and no.comandos < 5:
        rclpy.spin_once(no, timeout_sec=0.05)
    ok = no.comandos >= 5
    print(f"   estrategia {'ATIVA' if ok else 'INATIVA'} "
          f"({no.comandos} comandos em {LIMITE - (fim - time.time()):.0f}s)")
    no.destroy_node()
    sys.exit(0 if ok else 1)


def _ferramenta_resumo():
    """Resume N execucoes do mesmo cenario: avanco, y final, pico, gols."""

    DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "saida")
    ROTULO = sys.argv[2] if len(sys.argv) > 2 else "disp"
    GOL_X, MEIA_TRAVE = 4500.0, 500.0


    def cruzamento_y(amostras):
        """Onde a bola cruzou x=4500 (interpolado), ou None se nunca cruzou."""
        ant = None
        for a in amostras:
            if not a.get("bola"):
                continue
            b = a["bola"][0]
            if ant is not None and ant["x"] < GOL_X <= b["x"]:
                f = (GOL_X - ant["x"]) / (b["x"] - ant["x"] or 1.0)
                return ant["y"] + f * (b["y"] - ant["y"])
            ant = b
        return None


    arqs = sorted(glob.glob(os.path.join(DIR, "**", "*__%s_*.json" % ROTULO), recursive=True))
    if not arqs:
        print("nenhum resultado com o rotulo '%s' em %s" % (ROTULO, DIR))
        sys.exit(1)

    print("%-4s %10s %10s %9s %10s %8s %8s" %
          ("rep", "bola_x", "bola_y", "avanco", "pico_v", "cruz_y", "gol"))
    print("-" * 66)

    linhas = []
    for arq in arqs:
        d = json.load(open(arq))
        am = d.get("amostras", [])
        com_bola = [a for a in am if a.get("bola")]
        if not com_bola:
            continue
        b0, b1 = com_bola[0]["bola"][0], com_bola[-1]["bola"][0]
        pico = 0.0
        for i in range(1, len(com_bola)):
            p, q = com_bola[i - 1], com_bola[i]
            dt = (q.get("t", 0) - p.get("t", 0)) or (1 / 30.0)
            v = math.hypot(q["bola"][0]["x"] - p["bola"][0]["x"],
                           q["bola"][0]["y"] - p["bola"][0]["y"]) / dt
            # picos absurdos sao teleporte/glitch de visao, nao fisica
            if v < 12000.0:
                pico = max(pico, v)
        cy = cruzamento_y(com_bola)
        rep = os.path.basename(arq).split("__")[1].replace(".json", "")
        gol = d.get("gol") or ("-" if cy is None else
                               ("SIM" if abs(cy) < MEIA_TRAVE else "larga"))
        linhas.append((rep, b1["x"], b1["y"], b1["x"] - b0["x"], pico, cy, gol))
        print("%-4s %10.0f %10.0f %9.0f %10.0f %8s %8s" %
              (rep.split("_")[-1], b1["x"], b1["y"], b1["x"] - b0["x"], pico,
               "-" if cy is None else "%.0f" % cy, gol))

    if not linhas:
        sys.exit(1)


    def espalha(vals, nome, unid="mm"):
        if not vals:
            return
        m = sum(vals) / len(vals)
        dp = (sum((v - m) ** 2 for v in vals) / len(vals)) ** 0.5
        print("  %-16s media %8.0f   desvio %7.0f   min %8.0f   max %8.0f  %s"
              % (nome, m, dp, min(vals), max(vals), unid))


    print()
    print("DISPERSAO entre %d execucoes IDENTICAS:" % len(linhas))
    espalha([l[3] for l in linhas], "avanco da bola")
    espalha([l[2] for l in linhas], "y final")
    espalha([l[4] for l in linhas], "pico de velocidade", "mm/s")
    gols = sum(1 for l in linhas if l[6] == "SIM" or l[6] == "nosso")
    print()
    print("  GOLS: %d de %d" % (gols, len(linhas)))
    print("  criterio do Felipe: 2 de 3 -> %s"
          % ("ATINGIDO" if gols * 3 >= 2 * len(linhas) else "NAO atingido"))


def _ferramenta_atrito():
    """Mede o alcance real da bola no grSim, isolado de robo e estrategia."""



    def carregar():
        import importlib
        for mod in ("grSim_Packet_pb2",):
            try:
                return importlib.import_module(mod)
            except ImportError:
                pass
        sys.path.insert(0, "/root/ssl-VICE/install/grsim_messenger/lib/python3.10/site-packages/grsim_messenger/protobuf")
        return importlib.import_module("grSim_Packet_pb2")


    pb = carregar()
    SOCK = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    DEST = ("127.0.0.1", 20011)


    def lancar(vx, vy):
        p = pb.grSim_Packet()
        p.replacement.ball.x = 0.0
        p.replacement.ball.y = 0.0
        p.replacement.ball.vx = vx
        p.replacement.ball.vy = vy
        SOCK.sendto(p.SerializeToString(), DEST)


    class Obs(Node):
        def __init__(self):
            super().__init__("atrito_bola")
            self.b = None
            self.create_subscription(GameState, "game_state", self.a, 10)

        def a(self, m):
            if m.balls:
                self.b = (m.balls[0].position_x, m.balls[0].position_y)


    rclpy.init()
    n = Obs()
    vels = [float(x) for x in sys.argv[2:]] or [3.0, 4.0, 5.0, 6.0, 6.5]

    print("%8s %12s %14s" % ("v(m/s)", "alcance(mm)", "desaceleracao"))
    print("-" * 38)
    for v in vels:
        lancar(0.0, 0.0)
        t = time.time()
        while time.time() - t < 1.5:
            rclpy.spin_once(n, timeout_sec=0.02)
        lancar(v, 0.0)
        t0 = time.time()
        xmax = 0.0
        parado_desde = None
        ant = None
        while time.time() - t0 < 12.0:
            rclpy.spin_once(n, timeout_sec=0.02)
            if n.b is None:
                continue
            xmax = max(xmax, n.b[0])
            if ant is not None and abs(n.b[0] - ant) < 1.0:
                if parado_desde is None:
                    parado_desde = time.time()
                elif time.time() - parado_desde > 1.0:
                    break
            else:
                parado_desde = None
            ant = n.b[0]
        a = (v * v) / (2 * (xmax / 1000.0)) if xmax > 1 else float("nan")
        print("%8.1f %12.0f %10.2f m/s2" % (v, xmax, a))

    print()
    print("Para a falta em x=2500 chegar ao gol em x=4500 sao 2000 mm de rolagem.")
    rclpy.shutdown()


def _ferramenta_fps():
    """Mede a taxa REAL de quadros que o grSim esta emitindo.

    POR QUE ISTO E OBRIGATORIO ANTES DE QUALQUER TESTE
    --------------------------------------------------
    Em modo janela o grSim so avanca a fisica quando a janela e redesenhada
    (glwidget.cpp:392 chama step() dentro de paintGL). Com a janela coberta,
    minimizada, ou apenas disputando CPU, ele nao para - ele fica LENTO. Medimos
    4,1 Hz contra os 147,5 Hz do modo headless: 1/36 do tempo real.
    
    O sintoma e traicoeiro: nada acusa erro. A gravacao de 25 s passa a cobrir
    menos de 2 segundos simulados, o robo "nao chega na bola", e a conclusao
    natural e que a estrategia esta errada. Perdemos horas assim - comparando
    versoes de codigo cujas diferencas eram, na verdade, velocidade de simulacao.
    """
    porta = 10020
    grupo = "224.5.23.2"
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("", porta))
    s.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP,
                 struct.pack("4sl", socket.inet_aton(grupo), socket.INADDR_ANY))
    s.settimeout(1.0)
    n, t0 = 0, time.time()
    while time.time() - t0 < 5.0:
        try:
            s.recv(4096); n += 1
        except socket.timeout:
            pass
    hz = n / 5.0
    if hz >= 45.0:
        print("      ok  grSim a %.0f Hz" % hz)
        return 0
    print("      !!  grSim a apenas %.1f Hz (esperado 60+)." % hz)
    print("          A simulacao esta rodando em camera lenta e QUALQUER teste")
    print("          feito assim nao vale. Use --headless, ou deixe a janela")
    print("          do grSim visivel e sem nada por cima.")
    return 1


def _ferramenta_pronto():
    """Espera um elo da cadeia ficar PRONTO de verdade, e sai assim que estiver.

    Substitui os 'sleep 7' / 'sleep 5' / 'sleep 8' do shell. Aqueles numeros
    foram escolhidos para o pior caso; na maioria das execucoes o node responde
    em menos de um segundo, e o resto era espera jogada fora - multiplicada por
    cada repeticao de cada cenario.

    A espera acontece TODA aqui dentro, num unico 'docker exec'. Fazer o laco no
    shell custaria um exec por tentativa, e o custo do proprio exec passaria a
    dominar o tempo que estamos tentando economizar.

        pronto servicos [s]   os servicos que a estrategia precisa existirem
        pronto arbitro  [s]   /refereeTopic entregando mensagem
        pronto visao    [s]   /game_state com robos e geometria
        pronto tudo     [s]   os tres acima + a estrategia ja comandando
    """
    alvo = sys.argv[2] if len(sys.argv) > 2 else "servicos"
    limite = float(sys.argv[3]) if len(sys.argv) > 3 else 30.0

    rclpy.init()
    no = Node("pronto")
    t0 = time.time()
    achou = False
    try:
        if alvo == "servicos":
            # Sem estes tres a estrategia fica presa esperando o driver, e o
            # sintoma e mudo: nenhum comando e gerado, os robos nao se mexem.
            precisa = _servicos_exigidos()
            while time.time() - t0 < limite:
                nomes = {n for n, _ in no.get_service_names_and_types()}
                if precisa.issubset(nomes):
                    achou = True
                    break
                time.sleep(0.2)

        elif alvo == "arbitro":
            estado = {"ok": False}
            no.create_subscription(RefereeMessage, "refereeTopic",
                                   lambda _m: estado.__setitem__("ok", True), 10)
            while time.time() - t0 < limite and not estado["ok"]:
                rclpy.spin_once(no, timeout_sec=0.1)
            achou = estado["ok"]

        elif alvo == "visao":
            estado = {"ok": False}

            def cb(m):
                if m.ally_robots and getattr(m.geometry, "field_length", 0):
                    estado["ok"] = True

            no.create_subscription(GameState, "game_state", cb, 10)
            while time.time() - t0 < limite and not estado["ok"]:
                rclpy.spin_once(no, timeout_sec=0.1)
            achou = estado["ok"]
        elif alvo == "tudo":
            # TODAS as checagens num processo so.
            #
            # Cada 'docker exec' com rclpy custa ~4 s so para subir. Fazendo uma
            # chamada por elo, esse custo passaria a dominar justamente o tempo
            # que estamos tentando economizar. Aqui pagamos uma vez.
            precisa = _servicos_exigidos()
            estado = {"arbitro": False, "comandos": 0}
            no.create_subscription(RefereeMessage, "refereeTopic",
                                   lambda _m: estado.__setitem__("arbitro", True), 10)
            _assinar_setpoint(
                no, lambda: estado.__setitem__("comandos", estado["comandos"] + 1))
            servicos = False
            while time.time() - t0 < limite:
                rclpy.spin_once(no, timeout_sec=0.05)
                if not servicos:
                    servicos = precisa.issubset(
                        {n for n, _ in no.get_service_names_and_types()})
                # a estrategia precisa COMANDAR, nao so existir: a arvore leva
                # dezenas de segundos para resolver a cor do time, e gravar
                # antes disso mede o nada.
                if servicos and estado["arbitro"] and estado["comandos"] >= 5:
                    achou = True
                    break
            if not achou:
                print("   ! faltou: %s%s%s" % (
                    "" if servicos else "servicos ",
                    "" if estado["arbitro"] else "arbitro ",
                    "" if estado["comandos"] >= 5 else "estrategia-comandando"))

        else:
            print("alvo desconhecido: %s" % alvo, file=sys.stderr)
    finally:
        no.destroy_node()
        rclpy.shutdown()

    gasto = time.time() - t0
    if achou:
        print("   %s pronto em %.1fs" % (alvo, gasto))
        sys.exit(0)
    print("   ! %s NAO ficou pronto em %.0fs" % (alvo, limite))
    sys.exit(1)


def _ferramenta_decisao():
    """Confere a DECISAO da jogada sem simulador algum: chuta, passa, ou nada.

    Deterministico. Quando estiver em duvida se a logica esta certa, use isto e
    nao a simulacao - metade dos bugs que cacamos na estrategia estavam no
    ambiente, nao no codigo.
    """
    sys.path.insert(0, "/root/ssl-VICE/install/strategy/lib/python3.10/site-packages")
    from strategy.tatics.freekick import OurFreekick

    class R:
        def __init__(s, i, x, y):
            s.id = i; s.position_x = x; s.position_y = y
            s.velocity_x = 0.0; s.velocity_y = 0.0
            s.orientation = 0.0; s.velocity_orientation = 0.0

    class B:
        def __init__(s, x, y):
            s.position_x = x; s.position_y = y
            s.velocity_x = 0.0; s.velocity_y = 0.0

    casos = [
        ("ataque livre -> CHUTA no canto",      B(2500, 0),     {1: R(1, 2200, 0), 2: R(2, 3500, 600)},       {0: R(0, 4300, 900)}),
        ("linha de tiro bloqueada -> nao arma", B(2500, 0),     {1: R(1, 2200, 0), 2: R(2, 3500, -600)},      {0: R(0, 3200, -150)}),
        ("defesa, apoio a frente -> PASSA",     B(-2800, -600), {1: R(1, -3100, -600), 2: R(2, -1200, -200)}, {0: R(0, 4300, 0)}),
        ("apoio so ATRAS -> passa para tras",   B(500, 800),    {1: R(1, -100, 900), 2: R(2, -300, 2000)},    {0: R(0, 4300, 0)}),
        ("encurralado -> passa ao goleiro",     B(-3500, 1200), {0: R(0, -4300, 0), 1: R(1, -3800, 1200)},    {0: R(0, -2500, 1200), 1: R(1, -2600, 600)}),
        ("sozinho em campo -> nao arma",        B(500, 800),    {1: R(1, 200, 800)},                          {0: R(0, 4300, 0)}),
    ]
    for nome, bola, aliados, inimigos in casos:
        fk = OurFreekick(aliados, bola, False, enemy_robots=inimigos)
        x, y, tipo = fk._alvo_da_jogada(1)
        print("%-38s -> %-5s alvo=(%7.0f,%6.0f)  forca=%.2f m/s  arma=%s"
              % (nome, tipo.upper(), x, y, fk._forca_do_chute(1), fk._tem_alvo_valido()))


def _assinar_setpoint(no, ao_receber):
    """Assina o topico de setpoint do caminho de movimento em uso.

    ANTIGO: /control_command, publicado pelo driver.
    NOVO:   /movement_tracker/control_reference, publicado pelo tracker_node.

    Isto NAO e cosmetico. Depois do merge da dev o control.py passou a escutar
    'movement_tracker/control_reference' e o 'control_command' fica MUDO no
    caminho novo. Todos os portoes que perguntam "a estrategia esta comandando?"
    olhavam so o topico antigo - em modo novo eles reprovariam sempre, e o lote
    inteiro seria bloqueado por um teste do instrumento, nao por falha da
    estrategia.
    """
    # Um topico so: o driver acabou (ver a nota no gravador).
    #
    # 'movement_tracker/control_reference' so existe quando ha TRAJETORIA
    # valida. 'movement_manager/commands' e a saida da PROPRIA estrategia, e e
    # a prova certa de que o primeiro elo esta vivo - o portao "a estrategia
    # esta comandando?" olha os dois.
    from movement_interfaces.msg import TrajectoryPoint as _TP
    from movement_interfaces.msg import MovementCommandArray as _MCA
    no.create_subscription(_TP, "movement_tracker/control_reference",
                           lambda _m: ao_receber(), 10)
    return no.create_subscription(
        _MCA, "movement_manager/commands",
        lambda m: ao_receber() if m.commands else None, 10)


def _servicos_exigidos():
    """Servicos sem os quais a estrategia fica presa, por caminho de movimento.

    MOVIMENTO ANTIGO: 'strategy_command' e 'update_obstacles' sao do driver.
    MOVIMENTO NOVO: o driver nao sobe; quem recebe alvo e o MovementManager, e
    ele SO publica depois de SetStaticObstacles e SetGoalKeeper - se faltarem, o
    manager recebe comando e nao publica alvo nenhum, sem erro e sem log.
    'set_orientation' vale nos dois: orientacao e do pacote control, que a dev
    nao mexeu.
    """
    # O driver acabou: nao ha mais 'strategy_command' nem 'update_obstacles'.
    return {"/SetStaticObstacles", "/SetGoalKeeper", "/set_orientation"}


def _pasta_replays_padrao():
    """<raiz>/Replays_GrSim - a pasta que o proprio replay ja usa.

    A raiz e descoberta subindo a arvore a partir deste arquivo, do mesmo jeito
    que o ararabots.sh faz. Sem caminho fixo: mover o projeto nao quebra nada.
    """
    aqui = os.path.dirname(os.path.abspath(__file__))

    # 1) Se ja existe uma Replays_GrSim subindo a arvore, e ela.
    d = aqui
    for _ in range(6):
        cand = os.path.join(d, "Replays_GrSim")
        if os.path.isdir(cand):
            return cand
        d = os.path.dirname(d)

    # 2) Nao existe: CRIA no lugar certo, que e UM NIVEL ACIMA do ssl-VICE -
    #    a mesma raiz que o ararabots.sh usa (_descobrir_raiz), onde ficam os
    #    quatro repositorios irmaos. Achamos a raiz procurando quem contem um
    #    diretorio 'ssl-VICE'.
    #
    #    POR QUE CRIAR, e nao so avisar: o lado do shell (copiar_replays) ja faz
    #    'mkdir -p "$RAIZ/Replays_GrSim"'. Se o painel caisse na pasta do script
    #    quando ela ainda nao existe, ele leria um lugar e o teste gravaria em
    #    outro - e o painel apareceria vazio sem explicacao, numa maquina nova.
    d = aqui
    for _ in range(6):
        if os.path.isdir(os.path.join(d, "ssl-VICE")):
            destino = os.path.join(d, "Replays_GrSim")
            os.makedirs(destino, exist_ok=True)
            return destino
        d = os.path.dirname(d)

    # 3) Ultimo caso (script movido para fora da arvore): ao lado do proprio
    #    ssl-VICE, deduzido de docs/ -> ssl-VICE/ -> raiz.
    destino = os.path.join(os.path.dirname(os.path.dirname(aqui)), "Replays_GrSim")
    os.makedirs(destino, exist_ok=True)
    return destino


def _ler_replay(caminho):
    """Extrai o payload de um replay HTML. Devolve None se nao for um."""
    try:
        with open(caminho, encoding="utf-8") as fh:
            txt = fh.read()
        i = txt.index("const D = ")
        j = txt.index(";\n", i)
        return json.loads(txt[i + 10:j])
    except Exception:
        return None


# ============================================================================
#  MEDIR UM REPLAY: as metricas de RESULTADO de uma execucao, numa linha.
# ============================================================================
#
# POR QUE ISTO EXISTE, e por que nao usa o CSV: o CSV guarda 'andou', que e o
# MODULO do deslocamento da bola - e modulo nao distingue "a bola foi para o gol
# deles" de "a bola foi para o NOSSO". Foi isso que inverteu a conclusao do lote
# de 03/10/2026: a orbita aparecia como pior (139 mm contra 1302) quando o que
# ela tinha feito era deixar de empurrar a bola 1087 mm para o nosso campo.
#
# As duas armadilhas que esta funcao resolve:
#
#   1. SEGUNDA FONTE DE VISAO. 17 dos 30 replays daquele lote tem, depois do
#      fim da gravacao, um bloco de quadros com outro relogio de captura, seis
#      amarelos e a bola fora do campo - outro grSim no mesmo grupo multicast.
#      Aqui o replay e cortado no primeiro SALTO de tempo; replays gravados a
#      partir de 07/10 ja nascem limpos (ver TOL_RELOGIO).
#   2. TREMOR DA VISAO SOMADO. Somar o passo da bola quadro a quadro da ~7200 mm
#      numa execucao em que ela nao saiu do lugar (~5 mm por quadro de tremor a
#      58 quadros/s). Por isso o numero que vale e o LIQUIDO e o AVANCO.
SALTO_REPLAY = 5.0          # s; acima disto, outro relogio
CONTATO_RAIO = 130.0        # mm; casco 90 + bola 21 = 111, com folga de rastreio


def _trecho_valido(quadros):
    """Quadros ate o primeiro salto de tempo, e quantos ficaram de fora."""
    fim = len(quadros)
    for i, (a, b) in enumerate(zip(quadros, quadros[1:])):
        if b["t"] - a["t"] > SALTO_REPLAY:
            fim = i + 1
            break
    return quadros[:fim], len(quadros) - fim


def metricas_do_replay(caminho):
    """Metricas de resultado de UMA execucao, lidas do replay. Dicionario."""
    d = _ler_replay(caminho)
    if not d or not d.get("quadros"):
        return None
    q, descartados = _trecho_valido(d["quadros"])
    if not q:
        return None
    sent = 1.0 if (d.get("sentido_ataque") or 1.0) >= 0 else -1.0
    bx0, by0 = q[0]["b"][0], q[0]["b"][1]
    bxf, byf = q[-1]["b"][0], q[-1]["b"][1]
    rid = d.get("cobrador")
    dists, contato = [], 0
    # RECUO: o quanto a bola chegou a andar para o NOSSO campo em algum momento.
    #
    # E a metrica do contorno, e nenhuma outra serve. 'avanco' olha so o fim:
    # uma execucao que empurra a bola 1 m para tras e depois a traz 1,2 m para a
    # frente aparece como +200 mm, igual a uma que nunca errou o lado. O defeito
    # que o contorno existe para evitar e exatamente o primeiro empurrao errado.
    recuo = 0.0
    # GIRO: quanto o robo andou EM VOLTA da bola, em graus acumulados. Diz se
    # ele contornou de fato, em vez de inferir pelo resultado.
    giro, ang_ant = 0.0, None
    for f in q:
        bx, by = f["b"][0], f["b"][1]
        recuo = min(recuo, (bx - bx0) * sent)
        perto = min((math.hypot(r[1] - bx, r[2] - by) for r in f["r"]),
                    default=9e9)
        if perto < CONTATO_RAIO:
            contato += 1
        if rid is not None:
            for r in f["r"]:
                if r[0] == rid:
                    dists.append(math.hypot(r[1] - bx, r[2] - by))
                    ang = math.atan2(r[2] - by, r[1] - bx)
                    if ang_ant is not None:
                        dif = ang - ang_ant
                        while dif > math.pi:
                            dif -= 2 * math.pi
                        while dif < -math.pi:
                            dif += 2 * math.pi
                        giro += abs(dif)
                    ang_ant = ang
    # DERIVA: a bola no quadro t=0 estava onde o cenario pediu?
    #
    # Sem isto um par antes/depois pode comparar duas LARGADAS diferentes em vez
    # de duas taticas - foi o que aconteceu em 'portador_bola_atras_400', com a
    # bola 289 e 398 mm fora do lugar nas duas rodadas e o robo chegando ao t=0
    # ja em contato com ela. Qualquer valor acima de umas poucas dezenas de mm
    # invalida a comparacao.
    ped = d.get("bola_inicial") or {}
    deriva = None
    if ped.get("x") is not None:
        deriva = round(math.hypot(bx0 - ped["x"], by0 - ped["y"]))
    # O ERRO DA LINHA ROBO-BOLA-GOL, no fim da execucao.
    #
    # Pedido do Felipe (09/10/2026): "faca ele parar em frente a bola e tracar
    # uma linha entre o robo, a bola e o gol, para ver o erro". Com o portador
    # parado atras da bola (ARARABOTS_SO_POSICIONAR=1), estes dois numeros sao
    # TODO o posicionamento, sem o contato por cima:
    #
    #   erro_ang   quantos graus a direcao bola->robo esta de 'exatamente atras
    #              da bola em relacao ao gol'. Zero = os tres em linha.
    #   desvio     a que distancia o robo esta da RETA bola->gol, em mm. E o
    #              erro que manda a bola para o lado quando ele encosta.
    #
    # Medidos no ultimo segundo gravado, que e onde ele ja parou.
    erro_ang = desvio = None
    gol_x = 4500.0 * sent
    ult = [f for f in q if f["t"] >= q[-1]["t"] - 1.0]
    amostras_fim = []
    for f in ult:
        bx, by = f["b"][0], f["b"][1]
        for r in f["r"]:
            if rid is not None and r[0] == rid:
                amostras_fim.append((r[1], r[2], bx, by))
    if amostras_fim:
        angs, desvs = [], []
        for rx, ry, bx, by in amostras_fim:
            a_gol = math.atan2(0.0 - by, gol_x - bx)
            a_robo = math.atan2(ry - by, rx - bx)
            dif = a_robo - (a_gol + math.pi)
            while dif > math.pi:
                dif -= 2 * math.pi
            while dif < -math.pi:
                dif += 2 * math.pi
            angs.append(abs(math.degrees(dif)))
            # distancia do robo a reta bola->gol
            vx, vy = gol_x - bx, 0.0 - by
            n = math.hypot(vx, vy) or 1.0
            desvs.append(abs((rx - bx) * (vy / n) - (ry - by) * (vx / n)))
        angs.sort(); desvs.sort()
        erro_ang = round(angs[len(angs) // 2], 1)
        desvio = round(desvs[len(desvs) // 2])
    return {
        "cenario": d.get("cenario"),
        "quadros": len(q),
        "deriva": deriva,
        "erro_ang": erro_ang,
        "desvio": desvio,
        "descartados": descartados,
        "dur": round(q[-1]["t"], 1),
        "avanco": round((bxf - bx0) * sent),
        "recuo": round(recuo),
        "giro": round(math.degrees(giro)),
        "liquido": round(math.hypot(bxf - bx0, byf - by0)),
        "contato_pct": round(100.0 * contato / len(q)),
        "portador_min": round(min(dists)) if dists else None,
        "portador_max": round(max(dists)) if dists else None,
        "disparou": bool(d.get("disparou")),
        "gol": d.get("gol"),
        "chaves": ",".join(d.get("chaves_desligadas") or []) or "-",
    }


def _ferramenta_medir():
    """medir <replay.html> [...]  - uma linha por replay, para o 'lotes'."""
    alvos = sys.argv[2:]
    if not alvos:
        print("uso: ararabots.py medir <replay.html> [...]")
        return 1
    for c in alvos:
        m = metricas_do_replay(c)
        if m is None:
            print("ERRO|%s" % c)
            continue
        print("|".join("%s=%s" % (k, v) for k, v in m.items()))
    return 0


def _ferramenta_painel():
    """Painel HTML com o historico de execucoes, lido dos PROPRIOS replays.

    POR QUE LER OS REPLAYS, e nao so os validacao.csv
    -------------------------------------------------
    O CSV e sobrescrito a cada lote e so existe para os cenarios rodados por
    'validar'. Os replays ficam TODOS em Replays_GrSim, com carimbo de hora no
    nome, e carregam o registro completo - a serie da bola, os robos, os eventos
    do chutador. E o unico historico que nao se perde.

    Uso:
        ./ararabots.sh painel            usa <raiz>/Replays_GrSim
        ./ararabots.sh painel -definir   pergunta a pasta
        ./ararabots.sh painel <pasta>    usa a pasta dada
    """
    import collections
    import glob as _glob

    arg = sys.argv[2] if len(sys.argv) > 2 else ""
    if arg == "-definir":
        padrao = _pasta_replays_padrao()
        print("   pasta com os replays [%s]: " % padrao, end="", flush=True)
        try:
            digitado = sys.stdin.readline().strip()
        except Exception:
            digitado = ""
        pasta = os.path.expanduser(digitado) if digitado else padrao
    elif arg:
        pasta = os.path.expanduser(arg)
    else:
        pasta = _pasta_replays_padrao()

    if not os.path.isdir(pasta):
        print("   XX pasta inexistente: %s" % pasta)
        return 1

    arquivos = sorted(_glob.glob(os.path.join(pasta, "*.html")))
    arquivos = [a for a in arquivos if os.path.basename(a) != "painel.html"]
    if not arquivos:
        print("   XX nenhum replay .html em %s" % pasta)
        print("      (rode um cenario primeiro: ./ararabots.sh validar 1 um_so_cobrador)")
        return 1

    print("   lendo %d replays de %s ..." % (len(arquivos), pasta))
    regs = []
    for arq in arquivos:
        D = _ler_replay(arq)
        if not D:
            continue
        base = os.path.basename(arq)[:-5]
        partes = base.split("__")
        cen = D.get("cenario") or (partes[0] if partes else "?")
        quando = partes[-1] if len(partes) > 2 else ""
        lote = partes[1] if len(partes) > 2 else "avulso"

        Q = D.get("quadros") or []
        vmax = 0.0
        andou = 0.0
        if len(Q) > 2:
            b0 = Q[0]["b"]
            for k in range(1, len(Q)):
                dt = Q[k]["t"] - Q[k - 1]["t"]
                if dt > 0.005:
                    d = math.hypot(Q[k]["b"][0] - Q[k - 1]["b"][0],
                                   Q[k]["b"][1] - Q[k - 1]["b"][1])
                    vmax = max(vmax, d / dt)
            andou = max(math.hypot(fr["b"][0] - b0[0], fr["b"][1] - b0[1]) for fr in Q)
        regs.append({
            "cen": cen, "lote": lote, "quando": quando, "arq": base + ".html",
            "gol": D.get("gol") == "nosso",
            "contra": D.get("gol") == "contra",
            "disparou": bool(D.get("disparou")),
            "vmax": vmax, "andou": andou,
        })

    if not regs:
        print("   XX os .html existem mas nenhum tem payload de replay.")
        return 1

    por_cen = collections.OrderedDict()
    for r in regs:
        por_cen.setdefault(r["cen"], []).append(r)

    def barra(k, n):
        if not n:
            return ""
        pct = 100.0 * k / n
        cor = "ok" if pct >= 66 else ("meio" if pct >= 33 else "ruim")
        return ('<div class="b"><div class="f %s" style="width:%.0f%%"></div>'
                '<span>%d/%d</span></div>' % (cor, pct, k, n))

    def mediana(v):
        v = sorted(v)
        return v[len(v) // 2] if v else 0.0

    corpo = []
    # visao geral primeiro
    corpo.append('<h2>visao geral</h2><table><tr><th>cenario</th>'
                 '<th>execucoes</th><th>disparou</th><th>GOL</th>'
                 '<th>bola andou (mediana)</th><th>pico da bola (mediana)</th></tr>')
    for cen in sorted(por_cen):
        L = por_cen[cen]
        corpo.append("<tr><td><b>%s</b></td><td>%d</td><td>%s</td><td>%s</td>"
                     "<td>%.0f mm</td><td>%.0f mm/s</td></tr>"
                     % (cen, len(L), barra(sum(r["disparou"] for r in L), len(L)),
                        barra(sum(r["gol"] for r in L), len(L)),
                        mediana([r["andou"] for r in L]),
                        mediana([r["vmax"] for r in L])))
    corpo.append("</table>")

    # depois, por lote dentro de cada cenario
    for cen in sorted(por_cen):
        L = por_cen[cen]
        porlote = collections.OrderedDict()
        for r in sorted(L, key=lambda x: x["quando"]):
            porlote.setdefault(r["lote"], []).append(r)
        corpo.append('<h2>%s <span class="dim">(%d execucoes)</span></h2>'
                     '<table><tr><th>lote</th><th>quando</th><th>n</th>'
                     '<th>disparou</th><th>GOL</th><th>bola andou</th>'
                     '<th>pico</th></tr>' % (cen, len(L)))
        for lote, rs in porlote.items():
            q = rs[0]["quando"]
            quando = ("%s:%s" % (q[:2], q[2:4])) if len(q) >= 4 else "-"
            corpo.append("<tr><td>%s</td><td class=\"dim\">%s</td><td>%d</td>"
                         "<td>%s</td><td>%s</td><td>%.0f mm</td><td>%.0f mm/s</td></tr>"
                         % (lote, quando, len(rs),
                            barra(sum(r["disparou"] for r in rs), len(rs)),
                            barra(sum(r["gol"] for r in rs), len(rs)),
                            mediana([r["andou"] for r in rs]),
                            mediana([r["vmax"] for r in rs])))
        corpo.append("</table>")

    html = (_PAINEL_HTML.replace("/*CORPO*/", "\n".join(corpo))
                        .replace("/*QTD*/", str(len(regs)))
                        .replace("/*ARQS*/", str(len(arquivos)))
                        .replace("/*PASTA*/", pasta))
    destino = os.path.join(pasta, "painel.html")
    with open(destino, "w", encoding="utf-8") as fh:
        fh.write(html)
    print("   painel gerado: %s" % destino)
    print("   %d execucoes, %d cenarios" % (len(regs), len(por_cen)))
    return 0


_PAINEL_HTML = r"""<!doctype html><html lang="pt-BR"><head><meta charset="utf-8">
<title>Painel de resultados - Ararabots</title><style>
:root{--bg:#11151a;--fg:#e8eef5;--dim:#8b98a8;--linha:#39485a;--ok:#3ddc84;--meio:#ffd166;--ruim:#ff6b6b}
*{box-sizing:border-box}body{margin:0;padding:20px;background:var(--bg);color:var(--fg);
font:14px/1.6 ui-monospace,Menlo,Consolas,monospace}
h1{font-size:19px;margin:0 0 4px}h2{font-size:15px;margin:26px 0 6px;color:var(--meio)}
.sub{color:var(--dim);font-size:12px;margin-bottom:18px}
table{border-collapse:collapse;width:100%;max-width:1000px}
th,td{text-align:left;padding:6px 10px;border-bottom:1px solid var(--linha);font-size:13px}
th{color:var(--dim);font-weight:600;font-size:11px;text-transform:uppercase;letter-spacing:.5px}
.b{position:relative;background:#1c2530;border-radius:3px;height:18px;width:120px}
.f{height:100%;border-radius:3px}.f.ok{background:var(--ok)}.f.meio{background:var(--meio)}
.f.ruim{background:var(--ruim)}
.b span{position:absolute;left:8px;top:0;line-height:18px;font-size:11px;color:#0b0f14;font-weight:700}
.alerta{color:var(--ruim);font-size:11px}.dim{color:var(--dim);font-size:11px}
.nota{margin-top:30px;color:var(--dim);font-size:12px;max-width:760px;border-top:1px solid var(--linha);padding-top:14px}
</style></head><body>
<h1>Painel de resultados &mdash; cobranca de falta</h1>
<div class="sub">/*QTD*/ execucoes lidas de /*ARQS*/ replays em <b>/*PASTA*/</b></div>
/*CORPO*/
<div class="nota">
<b>Como ler.</b> <i>disparou</i> e o chutador do grSim ter disparado de verdade;
<i>GOL</i> e a bola ter cruzado a linha. Sao coisas diferentes e a diferenca
importa: um lote pode ter 6 de 6 disparos e 1 gol &mdash; nesse caso o problema
esta na mira ou no goleiro, nao na aproximacao. O contrario (poucos disparos)
aponta para a aproximacao.<br><br>
<b>Uma execucao nao diz nada.</b> Compare lotes de 6; diferencas de 1 ou 2 em 6
estao dentro do ruido. E confira em que configuracao cada lote rodou &mdash;
mira, goleiro, movimentacao e ajustes mudam o resultado, e lote medido sem saber
disso nao vale.
</div>
</body></html>"""



def _ferramenta_sonda_chute():
    """Segue o COMANDO DE CHUTE ao longo da cadeia, ate o grSim.

    POR QUE ISTO EXISTE
    -------------------
    A estrategia arma o chute ('arma=True' no DIAG_JOGO) dezenas de vezes por
    partida, o robo chega a 92-96 mm da bola - dentro da janela de disparo do
    grSim, que exige o centro da bola a ate ~104,5 mm - e NENHUM disparo sai.
    O adversario, comandado direto no simulador, dispara a 105 mm.

    Entre a nossa ordem e o chutador ha esta cadeia:

        strategy --update_kick--> control --commandTopic--> grsim_messenger

    Sondas avulsas por 'docker exec' deram medidas erradas (viam zero mensagem
    em topicos que existiam), entao a sonda vive aqui, no mesmo caminho que
    todos os outros testes usam.

    Uso:  ./ararabots.sh sonda-chute [segundos]
          (rode um cenario em outro terminal, ou logo apos 'cenario jogo')
    """
    segundos = float(sys.argv[2]) if len(sys.argv) > 2 else 25.0
    rclpy.init()
    no = Node("sonda_chute")
    est = {"cmd": 0, "ref": 0, "kick_pos": 0, "kick_max": 0.0, "robos": set()}

    def _cmd(m):
        est["cmd"] += 1
        for r in m.robots:
            est["robos"].add(int(r.robot_id))
            k = float(r.kick)
            if k > 0.0:
                est["kick_pos"] += 1
            est["kick_max"] = max(est["kick_max"], k)

    def _ref(m):
        est["ref"] += 1

    no.create_subscription(TeamCommand, "commandTopic", _cmd, 10)
    try:
        from movement_interfaces.msg import TrajectoryPoint as _TP
        no.create_subscription(_TP, "movement_tracker/control_reference",
                               _ref, 10)
    except Exception:
        pass

    t0 = time.time()
    while time.time() - t0 < segundos:
        rclpy.spin_once(no, timeout_sec=0.05)
    dur = max(time.time() - t0, 0.001)

    print()
    print("   CADEIA DO CHUTE, em %.0f s" % dur)
    print("   %-34s %6.1f Hz" % ("movement_tracker/control_reference",
                                 est["ref"] / dur))
    print("   %-34s %6.1f Hz   robos=%s"
          % ("commandTopic", est["cmd"] / dur, sorted(est["robos"])))
    print("   %-34s %6d amostras   maior valor=%.1f"
          % ("kick > 0 no commandTopic", est["kick_pos"], est["kick_max"]))
    print()
    if est["cmd"] == 0:
        print("   >> O commandTopic esta MUDO: nada chega ao grSim por aqui.")
        print("      control.py so publica se 'control_references' nao estiver")
        print("      vazio (control.py:120) - ou seja, so enquanto o rastreador")
        print("      emite referencia para o robo.")
    elif est["kick_pos"] == 0:
        print("   >> O commandTopic publica, mas o KICK nunca sai positivo:")
        print("      o problema esta entre 'update_kick' e o kick_cache.")
    else:
        print("   >> O chute CHEGA ao grSim. Se nao dispara, e geometria:")
        print("      bola a mais de ~104,5 mm do centro, ou fora da face.")
    no.destroy_node()
    rclpy.shutdown()
    return 0


def _ferramenta_narrar():
    """NARRA o replay quadro a quadro: o que aconteceu, em ordem, e por quem.

    POR QUE ISTO EXISTE
    -------------------
    O 'jogo-analise' devolve os numeros do FIM - x_max, v_pico, posse. Esses
    numeros esconderam o essencial durante varias rodadas: 'v_pico 5389' foi
    lido como "nosso time chuta" quando na verdade TODO arranco da bola era do
    adversario, sempre para tras. Agregado nao distingue quem fez o que.

    Aqui o replay e lido como jogo: cada toque, cada chute, quem tocou, para
    onde a bola foi e quanto andou. E o que permite dizer "aos 4,2 s o amarelo 1
    chutou para tras e a bola parou 1,1 m depois" em vez de "mediana 341 mm".

    Uso:  ./ararabots.sh narrar [n]     (n = quantos replays, padrao 1)
    """
    import glob as _glob

    quantos = int(sys.argv[2]) if len(sys.argv) > 2 else 1
    pasta = _pasta_replays_padrao()
    arqs = sorted(_glob.glob(os.path.join(pasta, "jogo__*.html")),
                  key=os.path.getmtime)[-quantos:]
    if not arqs:
        print("   XX nenhum replay do cenario 'jogo' em %s" % pasta)
        return 1

    TOQUE = 130.0        # raio de contato: robo 90 + bola 21, com folga
    MOVE = 400.0         # mm/s a partir do qual a bola esta "andando"

    for arq in arqs:
        D = _ler_replay(arq)
        Q = (D or {}).get("quadros") or []
        if len(Q) < 10:
            continue
        print()
        print("   " + "=" * 68)
        # CONTA OS ROBOS EM TODOS OS QUADROS, nao so no primeiro.
        #
        # Era 'len(Q[0]["r"]) - 1'. O rastreio perde robos em quadros isolados -
        # medido: os quatro azuis aparecem em ~1345 de 1407 quadros, e o azul 3
        # simplesmente nao esta no quadro 0. O relatorio dizia "2 de linha" e eu
        # tratei tres lotes como contaminados por isso, procurando um defeito de
        # simulacao que nao existia.
        _ids = set()
        for _f in Q:
            for _r in _f.get("r", []):
                _ids.add(_r[0])
        print("   %s   (%d quadros, %d de linha por time)"
              % (os.path.basename(arq), len(Q), max(len(_ids) - 1, 0)))
        print("   " + "=" * 68)

        def _t(i):
            return Q[i].get("t", i / 60.0)

        def _do_corpo(f, vx, vy):
            """A bola saiu no eixo do corpo de alguem em contato?"""
            saida = math.atan2(vy, vx)
            b = f["b"]
            for chave in ("r", "y"):
                for r in f.get(chave, []):
                    if math.hypot(r[1] - b[0], r[2] - b[1]) > 160.0:
                        continue
                    dif = abs((saida - r[3] + math.pi) % (2 * math.pi) - math.pi)
                    if dif < 0.45:          # ~25 graus do eixo do corpo
                        return True
            return False

        def _dono_chute(f, vx, vy):
            """Quem disparou: contato + corpo alinhado com a saida da bola."""
            saida = math.atan2(vy, vx)
            b = f["b"]
            melhor = (None, None, 9e9)
            for time, chave in (("AZUL", "r"), ("AMARELO", "y")):
                for r in f.get(chave, []):
                    dd = math.hypot(r[1] - b[0], r[2] - b[1])
                    if dd > 160.0:
                        continue
                    dif = abs((saida - r[3] + math.pi) % (2 * math.pi) - math.pi)
                    if dif < 0.45 and dif < melhor[2]:
                        melhor = (time, r[0], dif)
            return melhor

        def _dono(f):
            """(time, id, dist) do robo mais proximo da bola."""
            b = f["b"]
            melhor = (None, None, 1e9)
            for time, chave in (("AZUL", "r"), ("AMARELO", "y")):
                for r in f.get(chave, []):
                    d = math.hypot(r[1] - b[0], r[2] - b[1])
                    if d < melhor[2]:
                        melhor = (time, r[0], d)
            return melhor

        def _vel(i):
            if i == 0:
                return 0.0, 0.0, 0.0
            dt = max(_t(i) - _t(i - 1), 1e-3)
            vx = (Q[i]["b"][0] - Q[i - 1]["b"][0]) / dt
            vy = (Q[i]["b"][1] - Q[i - 1]["b"][1]) / dt
            return vx, vy, math.hypot(vx, vy)

        eventos = []
        ult_dono = None
        i = 1
        while i < len(Q):
            f = Q[i]
            time, rid, d = _dono(f)
            vx, vy, v = _vel(i)
            _, _, v0 = _vel(i - 1)

            # CHUTE: a bola sai do repouso para velocidade alta junto de um robo
            # ATRIBUICAO POR EIXO DO CORPO, nao por proximidade.
            #
            # BUG QUE ISTO CORRIGE: o chute era atribuido ao robo mais proximo
            # dentro de 400 mm. Se um adversario chutava e um robo nosso estava
            # mais perto naquele quadro, o relatorio dizia "CHUTE AZUL" - e eu
            # reportei tres vezes como avanco nosso o que era chute deles.
            #
            # O disparo do grSim sai no EIXO DO CORPO (robot.cpp:157). Uma
            # trombada sai na linha robo->bola. Entao o dono do chute e quem
            # tem o corpo alinhado com a saida da bola E esta em contato.
            if v > 2000.0 and v0 < 600.0 and d < 400.0 and _do_corpo(Q[i - 1], vx, vy):
                # ate onde ela foi antes de parar ou ser tocada de novo
                j = i
                while j + 1 < len(Q) and _vel(j + 1)[2] > MOVE:
                    j += 1
                perc = math.hypot(Q[j]["b"][0] - Q[i - 1]["b"][0],
                                  Q[j]["b"][1] - Q[i - 1]["b"][1])
                dxt = Q[j]["b"][0] - Q[i - 1]["b"][0]
                rumo = ("para o gol DELES" if dxt > 150 else
                        "para o NOSSO gol" if dxt < -150 else "para o lado")
                # o contato se ve no quadro ANTERIOR: no quadro do
                # salto de velocidade a bola ja saiu do chutador.
                _tm, _rd, _ = _dono_chute(Q[i - 1], vx, vy)
                time = _tm if _tm is not None else time
                rid = _rd if _rd is not None else rid
                eventos.append((_t(i), "CHUTE", time, rid,
                                "%4.0f mm/s, andou %4.0f mm %s, parou em x=%+5.0f y=%+5.0f"
                                % (v, perc, rumo, Q[j]["b"][0], Q[j]["b"][1])))
                i = j + 1
                ult_dono = (time, rid)
                continue

            # TOQUE: mudou de dono estando em contato
            if d < TOQUE and (time, rid) != ult_dono:
                eventos.append((_t(i), "toque", time, rid,
                                "bola em x=%+5.0f y=%+5.0f" % (f["b"][0], f["b"][1])))
                ult_dono = (time, rid)
            i += 1

        # PARADAS LONGAS: onde o jogo morreu
        paradas = []
        i = 1
        while i < len(Q):
            if _vel(i)[2] < 60.0:
                j = i
                while j + 1 < len(Q) and _vel(j + 1)[2] < 60.0:
                    j += 1
                if _t(j) - _t(i) > 1.5:
                    time, rid, d = _dono(Q[i])
                    paradas.append((_t(i), _t(j) - _t(i), Q[i]["b"][0],
                                    Q[i]["b"][1], time, rid, d))
                i = j + 1
            else:
                i += 1

        for ev in eventos:
            if ev[1] == "CHUTE":
                print("   %6.1fs  CHUTE  %-7s %d   %s" % (ev[0], ev[2], ev[3], ev[4]))
            else:
                print("   %6.1fs  toque  %-7s %d   %s" % (ev[0], ev[2], ev[3], ev[4]))
        if not eventos:
            print("   (nenhum toque nem chute: ninguem encostou na bola)")

        if paradas:
            print()
            print("   BOLA PARADA (mais de 1,5 s sem andar):")
            for t0, dur, bx, by, time, rid, d in paradas:
                print("   %6.1fs  por %4.1fs em x=%+5.0f y=%+5.0f  "
                      "(mais proximo: %s %d a %.0f mm)"
                      % (t0, dur, bx, by, time, rid, d))
        chutes_n = [e for e in eventos if e[1] == "CHUTE" and e[2] == "AZUL"]
        chutes_d = [e for e in eventos if e[1] == "CHUTE" and e[2] == "AMARELO"]
        print()
        print("   RESUMO: %d chutes nossos, %d deles, %d toques, %d paradas longas"
              % (len(chutes_n), len(chutes_d),
                 len([e for e in eventos if e[1] == "toque"]), len(paradas)))
    return 0


def _ferramenta_jogo_analise():
    """Mede o JOGO CORRIDO nos replays do cenario 'jogo'.

    POR QUE ISTO EXISTE
    -------------------
    Em bola parada o criterio e simples: disparou? foi gol? Em jogo corrido nao
    existe criterio unico, e olhar o replay "no olho" nao distingue progresso de
    agitacao - medimos um jogo em que os robos percorreram 12 metros cada e a
    bola nao saiu do nosso campo.

    As cinco perguntas que decidem se o jogo funciona:
      1. ALGUEM VAI A BOLA?     menor distancia de um robo nosso a bola
      2. AMONTOAM?              tempo com DOIS nossos a menos de 600 mm dela
      3. A BOLA ANDA PARA A FRENTE?  x maximo que ela alcancou
      4. O TIME ATACA?          quadros com robo nosso alem de x=3500
      5. ALGUEM CHUTA?          pico de velocidade da bola (chute = 5000+)

    Uso:  ./ararabots.sh jogo-analise [n]   (n = quantos replays recentes, padrao 3)
    """
    import glob as _glob

    quantos = int(sys.argv[2]) if len(sys.argv) > 2 else 3
    pasta = _pasta_replays_padrao()
    arqs = sorted(_glob.glob(os.path.join(pasta, "jogo__*.html")),
                  key=os.path.getmtime)[-quantos:]
    if not arqs:
        print("   XX nenhum replay do cenario 'jogo' em %s" % pasta)
        print("      rode:  ./ararabots.sh validar 3 jogo")
        return 1

    print("   %d replays de 'jogo' (mais recentes)" % len(arqs))
    print()
    print("   %-9s %8s %8s %9s %8s %9s %8s" %
          ("quando", "d_min", "amontoa", "x_max", "ataque", "v_pico", "posse"))
    resumo = []
    for arq in arqs:
        D = _ler_replay(arq)
        if not D:
            continue
        Q = D.get("quadros") or []
        if len(Q) < 10:
            continue
        dmin = 1e9
        amontoa = amostras = 0
        xmax = -9999.0
        ataque = 0
        vmax = 0.0
        posse = 0
        for k, fr in enumerate(Q):
            b = fr["b"]
            nossos = fr.get("r") or []
            xmax = max(xmax, b[0])
            for r in nossos:
                d = math.hypot(b[0] - r[1], b[1] - r[2])
                dmin = min(dmin, d)
                if r[1] > 3500:
                    ataque += 1
            if len(nossos) >= 2:
                ds = sorted(math.hypot(b[0] - r[1], b[1] - r[2]) for r in nossos)
                amostras += 1
                if ds[1] < 600.0:
                    amontoa += 1
                if ds[0] < 200.0:
                    posse += 1
            if k:
                dt = fr["t"] - Q[k - 1]["t"]
                if dt > 0.005:
                    vmax = max(vmax, math.hypot(b[0] - Q[k - 1]["b"][0],
                                                b[1] - Q[k - 1]["b"][1]) / dt)
        pa = (100.0 * amontoa / amostras) if amostras else 0.0
        pp = (100.0 * posse / amostras) if amostras else 0.0
        quando = os.path.basename(arq)[-11:-5]
        print("   %-9s %7.0f %7.0f%% %9.0f %8d %9.0f %7.0f%%"
              % (quando, dmin, pa, xmax, ataque, vmax, pp))
        resumo.append((dmin, pa, xmax, ataque, vmax, pp))

    if not resumo:
        return 1
    n = len(resumo)
    med = lambda i: sorted(x[i] for x in resumo)[n // 2]
    print()
    print("   MEDIANAS e o que cada uma quer dizer")
    print("   1. alguem vai a bola   : d_min  %6.0f mm   (contato e 111 mm)"
          % med(0))
    print("   2. amontoam            : %6.0f%% do tempo com DOIS a menos de 600 mm"
          % med(1))
    print("   3. bola avanca         : x_max  %6.0f mm   (gol deles em +4500)"
          % med(2))
    print("   4. time ataca          : %6.0f quadros alem de x=3500" % med(3))
    print("   5. alguem chuta        : v_pico %6.0f mm/s  (chute real e 5000+)"
          % med(4))
    print("   posse (alguem a menos de 200 mm da bola): %.0f%% do tempo" % med(5))
    return 0



def _ferramenta_posse():
    """Separa a execucao em SITUACOES DE JOGO e mede quanto tempo em cada uma.

    POR QUE ISTO EXISTE
    -------------------
    A tatica de jogo so distingue "o eleito e os outros". Ela reage a bola, nao
    joga: nao existe 'eles estao com a bola', 'a bola esta solta', 'nos temos a
    posse'. Sem separar os casos nao da para dar papel a ninguem - e sem medir
    quanto tempo o jogo passa em cada um, nao da para saber quais casos valem a
    pena tratar primeiro.

    CRITERIO, todo geometrico e sem estado:
      - DOMINIO: robo a menos de RAIO_POSSE da bola. Contato fisico acontece a
        111 mm (raio do robo 90 + raio da bola 21,5); 250 mm cobre "esta com
        ela" sem exigir toque perfeito, que a visao com ruido nao confirma.
      - NOSSA / DELES / DISPUTA / SOLTA, nessa ordem de prioridade.

    Uso:  ./ararabots.sh posse [n]     (n replays recentes de 'jogo', padrao 6)
    """
    import glob as _glob

    RAIO_POSSE = 250.0
    quantos = int(sys.argv[2]) if len(sys.argv) > 2 else 6
    pasta = _pasta_replays_padrao()
    arqs = sorted(_glob.glob(os.path.join(pasta, "jogo__*.html")),
                  key=os.path.getmtime)[-quantos:]
    if not arqs:
        print("   XX nenhum replay de 'jogo' em %s" % pasta)
        return 1

    import collections
    total = collections.Counter()
    campo = collections.Counter()
    amostras = 0
    for arq in arqs:
        D = _ler_replay(arq)
        for fr in (D.get("quadros") or []):
            b = fr["b"]
            nossos = fr.get("r") or []
            deles = fr.get("y") or []
            if not nossos:
                continue
            dn = min((math.hypot(b[0] - r[1], b[1] - r[2]) for r in nossos),
                     default=9e9)
            dd = min((math.hypot(b[0] - a[1], b[1] - a[2]) for a in deles),
                     default=9e9)
            amostras += 1
            if dn <= RAIO_POSSE and dd <= RAIO_POSSE:
                estado = "DISPUTA"
            elif dn <= RAIO_POSSE:
                estado = "NOSSA"
            elif dd <= RAIO_POSSE:
                estado = "DELES"
            else:
                estado = "SOLTA"
            total[estado] += 1
            # onde a bola esta: nosso campo (x<0) ou o deles
            campo[(estado, "ataque" if b[0] > 0 else "defesa")] += 1

    if not amostras:
        print("   XX replays sem quadros utilizaveis")
        return 1

    print("   %d replays de 'jogo', %d quadros" % (len(arqs), amostras))
    print()
    print("   SITUACAO         tempo      no nosso campo   no campo deles")
    for estado in ("NOSSA", "DELES", "DISPUTA", "SOLTA"):
        n = total[estado]
        if not n:
            continue
        print("   %-14s %5.0f%%      %11.0f%%   %13.0f%%"
              % (estado, 100.0 * n / amostras,
                 100.0 * campo[(estado, "defesa")] / n,
                 100.0 * campo[(estado, "ataque")] / n))
    print()
    print("   (posse = alguem a menos de %.0f mm da bola; contato fisico e 111 mm)"
          % RAIO_POSSE)
    return 0



def _ferramenta_mov_bruto():
    """Comanda a movimentacao NOVA sozinha, sem estrategia nenhuma.

    POR QUE ISTO EXISTE
    -------------------
    Depois do merge da dev o robo saia do campo em linha reta enquanto o
    setpoint publicado era sao (oscilando junto da bola). Havia duas
    explicacoes possiveis e nenhuma forma de escolher entre elas:
        a) a cadeia planner -> tracker -> control da dev nao segue a referencia;
        b) a NOSSA estrategia manda alvo de um jeito que ela nao aceita.

    Aqui a estrategia nao participa. Este processo faz o minimo que o
    MovementManager exige - SetStaticObstacles, SetGoalKeeper e um
    MovementCommandArray com UM alvo fixo - e mede se o robo chega la.

    Se o robo chegar, o defeito e nosso (b). Se sair do campo do mesmo jeito, o
    defeito e da cadeia deles (a), e nenhum conserto no nosso lado adianta.

    Uso:  ./ararabots.sh mov-bruto [x] [y]
    """
    import rclpy
    from rclpy.node import Node
    from movement_interfaces.msg import MovementCommandArray, MovementCommand
    from movement_interfaces.srv import SetStaticObstacles, SetGoalKeeper
    from system_interfaces.msg import GameState

    alvo_x = float(sys.argv[2]) if len(sys.argv) > 2 else 2250.0
    alvo_y = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
    robo = int(sys.argv[4]) if len(sys.argv) > 4 else 1
    dur = float(sys.argv[5]) if len(sys.argv) > 5 else 12.0

    rclpy.init()
    no = Node("mov_bruto")
    pub = no.create_publisher(MovementCommandArray, "movement_manager/commands", 10)
    est_cli = no.create_client(SetStaticObstacles, "SetStaticObstacles")
    gk_cli = no.create_client(SetGoalKeeper, "SetGoalKeeper")

    pos = {}
    ref = {}
    no.create_subscription(
        GameState, "game_state",
        lambda m: pos.update({r.id: (r.position_x, r.position_y,
                                     r.velocity_x, r.velocity_y)
                              for r in m.ally_robots}), 10)
    try:
        from movement_interfaces.msg import TrajectoryPoint
        no.create_subscription(
            TrajectoryPoint, "movement_tracker/control_reference",
            lambda m: ref.update({int(m.robot_id): (m.pos.x, m.pos.y,
                                                    m.vel.x, m.vel.y)}), 10)
    except Exception:
        pass

    # TIRAR DO HALT - senao o teste mede o nada.
    #
    # control.py zera o comando quando is_halt (o 'if self.is_halt: vel_cmd =
    # Vector2D(0,0)'), e o ambiente fica em HALT depois do preparar. A primeira
    # versao desta sonda nao mandava comando de arbitro nenhum: o robo ficava
    # parado e eu quase registrei isso como defeito da cadeia da dev.
    print("   tirando do HALT (STOP -> FORCE_START)...")
    try:
        enviar_comando_arbitro("STOP")
        time.sleep(1.0)
        enviar_comando_arbitro("FORCE_START")
        time.sleep(0.5)
    except Exception as e:
        print("   !! nao consegui falar com o arbitro: %s" % e)

    print("   esperando os servicos do MovementManager...")
    for cli, nome in ((est_cli, "SetStaticObstacles"), (gk_cli, "SetGoalKeeper")):
        if not cli.wait_for_service(timeout_sec=15.0):
            print("   XX servico %s ausente - o manager nao esta no ar" % nome)
            no.destroy_node(); rclpy.shutdown(); return 2

    r1 = SetStaticObstacles.Request(); r1.border_area = True; r1.center_area = False
    est_cli.call_async(r1)
    r2 = SetGoalKeeper.Request(); r2.robot_id = 0
    gk_cli.call_async(r2)
    for _ in range(20):
        rclpy.spin_once(no, timeout_sec=0.05)

    msg = MovementCommandArray()
    cmd = MovementCommand()
    cmd.robot_id = robo
    cmd.target_pos.x = alvo_x
    cmd.target_pos.y = alvo_y
    msg.commands = [cmd]

    print("   alvo do robo %d: (%.0f, %.0f)   gravando %.0fs" % (robo, alvo_x, alvo_y, dur))
    inicio = time.time()
    amostras = []
    while time.time() - inicio < dur:
        pub.publish(msg)                     # republica: o manager guarda o ultimo
        rclpy.spin_once(no, timeout_sec=0.02)
        if robo in pos:
            amostras.append((time.time() - inicio, pos[robo], ref.get(robo)))

    no.destroy_node(); rclpy.shutdown()

    if not amostras:
        print("   XX nenhuma leitura de game_state para o robo %d" % robo)
        return 3

    print()
    print("   %-6s %-22s %-22s %s" % ("t", "robo (x,y)", "referencia (x,y)", "dist ao alvo"))
    passo = max(1, len(amostras) // 12)
    for t, p, r in amostras[::passo]:
        dist = math.hypot(alvo_x - p[0], alvo_y - p[1])
        sref = "(%8.0f,%8.0f)" % (r[0], r[1]) if r else "        --        "
        print("   %5.1f  (%8.0f,%8.0f)  %s  %7.0f mm" % (t, p[0], p[1], sref, dist))

    t_fim, p_fim, _ = amostras[-1]
    d0 = math.hypot(alvo_x - amostras[0][1][0], alvo_y - amostras[0][1][1])
    df = math.hypot(alvo_x - p_fim[0], alvo_y - p_fim[1])
    print()
    print("   distancia ao alvo: %.0f mm -> %.0f mm" % (d0, df))
    if df < 150.0:
        print("   >> CHEGOU. A cadeia da dev segue a referencia; o defeito e do")
        print("      nosso lado (como a estrategia manda o alvo).")
    elif df > d0:
        print("   >> AFASTOU-SE. A cadeia planner->tracker->control da dev nao")
        print("      segue a referencia nem sem estrategia nenhuma.")
    else:
        print("   >> aproximou mas nao chegou - veja a serie acima.")
    return 0



def _ferramenta_sonda():
    """Tabula as linhas [FK] que a tatica emite com DIAG_FK=1.

    POR QUE ISTO MORA AQUI, e nao num script solto: a regra da ferramenta unica.
    Ja tivemos a receita duplicada em tres arquivos que divergiram em silencio.

    O QUE ELA RESPONDE. O setpoint do cobrador saia para um ponto que nao
    correspondia a nenhum alvo calculavel de fora - nem o ponto de encaixe
    (250 mm atras da bola) nem o raio de contorno (700 mm). Inferir qual ramo
    pedia aquilo falhou tres vezes seguidas. Esta sonda faz o ramo se
    identificar: cada acao da tatica registra o alvo que pediu, e aqui se ve a
    sequencia de fases, de ramos e a distancia ate a bola ao longo do tempo.

    Uso:
        DIAG_FK=1 ./ararabots.sh validar 1 passe
        ./ararabots.sh sonda
    """
    import collections
    saida = subprocess.run(
        ["docker", "exec", "vice", "cat", "/tmp/strategy.log"],
        capture_output=True, text=True).stdout
    linhas = [l for l in saida.split("\n") if l.startswith("[FK]")]
    if not linhas:
        print("Nenhuma linha [FK] no /tmp/strategy.log.")
        print("A sonda so emite com DIAG_FK=1 - e a variavel precisa atravessar")
        print("o ros_d, que e quem sobe o strategyNode (ver HANDOVER, o item da")
        print("variavel que nao atravessa o docker exec).")
        print("   DIAG_FK=1 ./ararabots.sh validar 1 <cenario>")
        return 1

    def campos(l):
        d = {}
        for par in l[5:].split():
            if "=" in par:
                k, v = par.split("=", 1)
                d[k] = v
        return d

    regs = [campos(l) for l in linhas]
    print("linhas [FK]: %d" % len(regs))

    # quanto tempo em cada (fase, ramo), e a distancia tipica ali
    conta = collections.Counter()
    dist = collections.defaultdict(list)
    for r in regs:
        ch = (r.get("fase", "?"), r.get("ramo", "?"))
        conta[ch] += 1
        try:
            dist[ch].append(float(r.get("d", "0")))
        except ValueError:
            pass
    print("\nCICLOS POR FASE E RAMO (o ramo e quem PEDIU o alvo):")
    print("  %-12s %-14s %7s  %s" % ("fase", "ramo", "ciclos", "dist ate a bola"))
    for ch, n in conta.most_common():
        ds = dist[ch]
        faixa = "%5.0f a %5.0f mm" % (min(ds), max(ds)) if ds else ""
        print("  %-12s %-14s %7d  %s" % (ch[0], ch[1], n, faixa))

    # trocas de fase: e o vaivem que queremos ver
    trocas = 0
    ant = None
    for r in regs:
        f = r.get("fase")
        if ant is not None and f != ant:
            trocas += 1
        ant = f
    print("\ntrocas de fase: %d em %d ciclos" % (trocas, len(regs)))

    # o momento em que o robo mais se afasta, com o ramo que mandou
    pior = None
    for r in regs:
        try:
            d = float(r.get("d", "0"))
        except ValueError:
            continue
        if pior is None or d > pior[0]:
            pior = (d, r)
    if pior:
        r = pior[1]
        print("\nMAIOR AFASTAMENTO DA BOLA: %.0f mm" % pior[0])
        print("  fase=%s  ramo=%s  pedido=%s  robo=%s  tipo=%s  alvo=%s"
              % (r.get("fase"), r.get("ramo"), r.get("pedido"),
                 r.get("robo"), r.get("tipo"), r.get("alvo")))
        print("  Este e o ramo a investigar: foi ele que pediu o ponto.")

    # amostra cronologica, para ver a sequencia
    print("\nSEQUENCIA (1 a cada %d linhas):" % max(1, len(regs) // 25))
    passo = max(1, len(regs) // 25)
    for r in regs[::passo]:
        print("  t=%-8s fase=%-10s ramo=%-13s d=%-6s pedido=%-16s alvo=%s"
              % (r.get("t"), r.get("fase"), r.get("ramo"), r.get("d"),
                 r.get("pedido"), r.get("alvo")))
    return 0



if __name__ == "__main__":
    acao = sys.argv[1] if len(sys.argv) > 1 else ""

    if acao == "robos":
        # QUANTOS ROBOS POR TIME O CENARIO EXIGE.
        #
        # O shell tinha isto fixo: 'jogo' => 4, todo o resto 3. Qualquer cenario
        # novo com quatro robos rodava com TRES, e o quarto simplesmente nao
        # existia em campo - medido no primeiro lote de 'orientacao_meio': o
        # replay mostrou os robos 0, 1 e 2, e o 3 nunca apareceu. Com um robo a
        # menos a distribuicao de papeis muda (sem apoio nao ha passe), ou seja
        # o cenario media outra coisa.
        #
        # Quem sabe a resposta e o proprio cenario: o maior id usado, mais um.
        # Nunca MENOS que o default, para nao mudar a configuracao dos cenarios
        # historicos - a cobranca de falta foi medida com tres.
        nome = sys.argv[2] if len(sys.argv) > 2 else ""
        cen = CENARIOS.get(nome)
        piso = int(os.environ.get("ARARABOTS_ROBOS", "3"))
        if not cen:
            print(piso); sys.exit(0)
        ids = [r[0] for r in cen.get("azuis", [])] + [r[0] for r in cen.get("amarelos", [])]
        print(max(piso, (max(ids) + 1) if ids else piso))
        sys.exit(0)

    if acao == "listar":
        for nome, c in CENARIOS.items():
            print("%s|%s" % (nome, c["titulo"]))

    elif acao == "posicionar":
        if len(sys.argv) < 3 or sys.argv[2] not in CENARIOS:
            print("cenario invalido", file=sys.stderr); sys.exit(2)
        cen = CENARIOS[sys.argv[2]]
        print(">> %s" % cen["titulo"])
        print("   %s" % cen["descricao"])
        print("   bola em x=%.0f y=%.0f mm" % (cen["bola"][0], cen["bola"][1]))
        enviar_comando_arbitro("HALT")
        time.sleep(1.5)
        print("   montando o cenario (robos nao usados sao desligados)...")
        posicionar(cen)
        print("   pronto - o shell vai reiniciar os nodes agora")

    elif acao == "rodar":
        if len(sys.argv) < 3 or sys.argv[2] not in CENARIOS:
            print("cenario invalido", file=sys.stderr); sys.exit(2)
        dur = float(sys.argv[3]) if len(sys.argv) > 3 else 12.0
        sys.exit(rodar(sys.argv[2], dur))

    elif acao == "fps":      sys.exit(_ferramenta_fps())
    elif acao == "pronto":   _ferramenta_pronto()
    elif acao == "cadeia":   _ferramenta_cadeia()
    elif acao == "esperar":  _ferramenta_esperar()
    elif acao == "resumo":   _ferramenta_resumo()
    elif acao == "atrito":   _ferramenta_atrito()
    elif acao == "decisao":  _ferramenta_decisao()
    elif acao == "sonda":    sys.exit(_ferramenta_sonda())
    elif acao == "mov-bruto": sys.exit(_ferramenta_mov_bruto())
    elif acao == "painel":   sys.exit(_ferramenta_painel())
    elif acao == "medir":    sys.exit(_ferramenta_medir())
    elif acao == "sonda-chute": sys.exit(_ferramenta_sonda_chute())
    elif acao == "narrar": sys.exit(_ferramenta_narrar())
    elif acao == "jogo-analise": sys.exit(_ferramenta_jogo_analise())
    elif acao == "posse":    sys.exit(_ferramenta_posse())
    else:
        print(__doc__); sys.exit(2)
