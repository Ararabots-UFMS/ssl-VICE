"""Camada de SKILLS: as acoes atomicas da estrategia.

Esta e a camada de baixo; as outras estao em docs/estrategia-regras.md:

    plays/      quando uma jogada se aplica
    tatics/     a ordem das etapas dentro da jogada
    analysis/   os fatos do ciclo, calculados uma vez
    roles/      para onde cada robo vai, por papel
    skills/     as acoes atomicas - este pacote

REGRAS DESTA CAMADA
-------------------
1. NADA aqui importa rclpy. Toda funcao recebe dados (robos, bola, pontos) e
   devolve dados (pontos, booleanos, forcas). Isso e o que permite testar
   decisao sem simulador - foi uma sonda desse tipo que encontrou a inversao de
   papeis de 02/10/2026.
2. As constantes MEDIDAS moram junto da skill que as usa, com o numero e a
   fonte no comentario. Constante medida que aparece em dois lugares e defeito.
3. Estado, quando inevitavel (a trava de armamento do chute), entra como
   parametro - nunca como atributo de modulo. Quem guarda e a jogada, numa
   dataclass de memoria (ver roles/ e analysis/).

O QUE AINDA NAO ESTA AQUI
-------------------------
A cobranca de falta (tatics/freekick.py) mantem as suas proprias versoes de
aproximacao e armamento. Ela e o codigo com resultado comprovado da base
(5 gols em 6) e migra-la muda numeros que ninguem mediu de novo - a migracao
esta prevista e exige lote medido. Ver docs/auditoria-papeis-e-testes.md §6.3.
"""

from strategy.skills.skills import Skill, Skills

from strategy.skills import aproximacao, avaliacao_chute, bola, chute, geometria, posicionamento

__all__ = [
    "Skill", "Skills",
    "aproximacao", "avaliacao_chute", "bola", "chute", "geometria", "posicionamento",
]
