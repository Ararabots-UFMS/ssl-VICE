"""Camada de SKILLS: as acoes atomicas da estrategia.

Tres camadas, e esta e a de baixo:

    plays/    quando uma jogada se aplica e quem faz o que
    tatics/   o comportamento de um robo dentro da jogada
    skills/   as acoes atomicas - este pacote

REGRAS DESTA CAMADA
-------------------
1. NADA aqui importa rclpy. Toda funcao recebe dados (robos, bola, pontos) e
   devolve dados (pontos, booleanos, forcas). Isso e o que permite testar
   decisao sem simulador - foi uma sonda desse tipo que encontrou a inversao de
   papeis de 02/10/2026.
2. As constantes MEDIDAS moram junto da skill que as usa, com o numero e a
   fonte no comentario. Constante medida que aparece em dois lugares e defeito.
3. Estado, quando inevitavel (a trava de armamento do chute), entra como
   parametro - nunca como atributo de modulo. As taticas sao reconstruidas a
   cada ciclo; so o dicionario 'estado' da jogada sobrevive.

O QUE AINDA NAO ESTA AQUI
-------------------------
A cobranca de falta (tatics/freekick.py) mantem as suas proprias versoes de
aproximacao e armamento. Ela e o codigo com resultado comprovado da base
(5 gols em 6) e migra-la muda numeros que ninguem mediu de novo - a migracao
esta prevista e exige lote medido. Ver docs/auditoria-papeis-e-testes.md §6.3.
"""

from strategy.skills.skills import Skill, Skills

from strategy.skills import aproximacao, bola, chute, geometria, posicionamento

__all__ = [
    "Skill", "Skills",
    "aproximacao", "bola", "chute", "geometria", "posicionamento",
]
