# Regras de projeto da estratégia

Como a estratégia (`src/strategy/strategy/`) é organizada e o que todo PR que mexe
nela tem de seguir. Vale para papel novo, jogada nova, ajuste de comportamento e
refatoração.

A estrutura segue a mesma divisão do Sumatra (TIGERs Mannheim): uma **análise**
calculada uma vez por ciclo, uma **atribuição** que entrega papéis aos robôs, e
**papéis** que só decidem para onde o seu robô vai. A diferença é de escala: lá
são ~80 calculadores e dezenas de papéis em Java; aqui são poucos módulos Python
puros, que dá para ler numa tarde.

---

## 1. O ciclo, de ponta a ponta

```
game_state ──► strategy.py ──► TickContext ──► RootTree ──► play ──► tática ──► [Skill] ──► MovementHandler
 (tópico)      (único nó ROS)   (foto imutável)   (Selector)   (BT)    (análise +    (um por     (planejador,
                                                                       papéis)       robô)       chute, giro)
```

A cada 100 ms, o `strategy.py`:

1. congela a última mensagem em um `TickContext` (`context.py`);
2. roda a árvore (`root.py`) com esse contexto;
3. entrega as `Skill` devolvidas ao `MovementHandler`, que publica **uma**
   `MovementCommandArray` com o time inteiro e chama os serviços de chute e giro.

No jogo corrido (`FORCE_START` / `NORMAL_START`), a tática `JogoCorrido`
(`tatics/running.py`) faz, em ordem:

```
analysis/mundo.analisar_mundo     situação (NOSSA/DELES/DISPUTA/SOLTA), gols, sentido
roles/atribuicao.distribuir_papeis  quem é PORTADOR, APOIO, COBERTURA
analysis/alvo_chute.alvo_do_chute   para onde a bola vai: gol, passe, saída, alívio
──────────────────────────────────── tudo isso vira um AnaliseJogo (congelado)
roles/goleiro, roles/<papel>.comando   uma Skill por robô
```

---

## 2. As camadas

| camada | pasta / arquivo | decide | **não pode** |
|---|---|---|---|
| Nó ROS | `strategy.py`, `movement_handler.py` | nada de jogo: assina, consulta serviço, publica | conter geometria ou regra de jogo |
| Contexto | `context.py` | o que a árvore sabe do mundo neste ciclo | ser alterado por ninguém dentro do ciclo |
| Árvore | `behaviour.py`, `root.py`, `plays/` | **qual** jogada vale, pelo comando do árbitro e pela configuração | fazer conta de campo; guardar mundo em atributo |
| Tática | `tatics/` | ordem das etapas dentro da jogada | decidir o que já é decidido por análise ou papel |
| Análise | `analysis/` | fatos do ciclo que mais de um papel usa | devolver comando; saber de robô específico |
| Atribuição | `roles/atribuicao.py` | quem joga qual papel | existir em outro lugar |
| Papel | `roles/<papel>.py` | para onde **o seu** robô vai, e em que fase está | recalcular análise; mexer em outro robô |
| Skill | `skills/` | ações atômicas e geometria (aproximação, chute, posicionamento) | importar ROS; guardar estado em módulo |

---

## 3. Regras

**R1 — Só o `strategy.py` fala com ROS.** Nenhum nó da árvore, tática, papel ou
skill importa `rclpy`, assina tópico ou cria cliente de serviço. Um dado novo do
mundo entra como campo do `TickContext`, preenchido em `from_game_state`. O teste
`test_tree_builds_without_rclpy` quebra se alguém esquecer.

> Por quê: com cada folha sendo um nó ROS havia 18 assinaturas de `game_state`
> e 6 clientes de `get_game_config`, cada um com a sua ideia de que lado do campo
> é o nosso.

**R2 — Tudo que um nó sabe vem do `context` recebido no `run()`.** Nada de
singleton, de variável de módulo com o "último estado", de callback. Duas leituras
no mesmo ciclo veem o mesmo mundo.

**R3 — A árvore escolhe a jogada; não faz geometria.** Plays têm `CheckState` e
ações que chamam uma tática. "Atacar" e "defender" no jogo corrido **não** são
ramos da árvore: são papéis dentro do `JogoCorrido`, escolhidos pela análise.

**R4 — A análise é calculada uma vez por ciclo, e os papéis só a leem.** Se dois
papéis precisam do mesmo fato (quem prensa a bola, qual adversário é a ameaça, se
a linha de passe está livre), ele vira campo da análise. Um papel nunca chama
`alvo_do_chute`, `distribuir_papeis` ou `situacao_de_jogo`.

**R5 — Um papel por arquivo, subclasse de `Papel`.** O papel implementa
`alvo(a, rid, ordem) -> (x, y)` e, se precisar, os ganchos:

| gancho | padrão | para quê |
|---|---|---|
| `fase()` | `None` | fase entregue à trava de chute (`chute.chutar_em`) |
| `velocidade_chegada(a, rid, x, y)` | `(0, 0)` | chegar em movimento em vez de parar |
| `evita_bola(a, rid)` | `True` | planejador contorna a bola |
| `diagnosticar(a, rid, cmd)` | nada | linhas extras com `DIAG_JOGO=1` |

O resto — orientação, armar o chute, força, bandeiras de obstáculo — é **igual
para todo papel** e mora em `Papel.comando`. Não sobrescreva `comando` (o goleiro
é a exceção documentada, porque tem regra própria de chute e área).

**R6 — Papéis são entregues em um lugar só: `roles/atribuicao.py`.** Todo nome
devolvido por `distribuir_papeis` tem de estar registrado em
`JogoCorrido.papeis`; o teste `test_every_role_handed_out_has_an_implementation`
confere isso para 1 a 5 robôs de linha.

**R7 — Memória é uma `dataclass` ao lado de quem a usa, e a jogada é a dona.**

- Papel com memória guarda em `self.memoria` (ex.: `MemoriaPortador`). O
  `JogoCorrido` cria cada papel **uma vez**, então a memória sobrevive entre
  ciclos.
- Memória da análise vai em `MemoriaAnalise` (`analysis/jogo.py`).
- **Proibido:** dicionário com chave em string (`estado["fase_portador"]`),
  atributo de módulo, atributo de classe mutável.
- Valor que só vale no ciclo que o decidiu (como a fase do portador) é zerado no
  começo do ciclo, de propósito, e isso está escrito no docstring do papel.

> Por quê: o defeito da fase velha ligando a bola como obstáculo para o alvo
> errado veio de uma chave do dicionário que um ramo escrevia e outro não limpava.

**R8 — Fases são nomeadas e listadas no topo do módulo do papel.** O docstring do
módulo diz o que cada fase faz e o que faz mudar de fase. Quem lê o arquivo sabe
o diagrama de estados sem caçar `if`.

**R9 — Constante medida mora ao lado da skill ou do papel que a usa**, em
`MAIÚSCULAS`, com o número e a fonte em **uma ou duas linhas** de comentário:

```python
# 2 s de mira sem girar e empurra assim mesmo: parado, perde a bola.
CICLOS_MAX_MIRANDO = 20
```

O histórico completo da medição (tabelas, lotes, datas) vai na **mensagem do
commit** ou em `docs/`, não no código. Constante repetida em dois arquivos é
defeito: importe.

**R10 — Chaves de experimento só por `skills/experimento.py`.**

- `experimento.desligado("NOME")` — desliga uma modificação já adotada (o padrão
  é o comportamento novo). Liga com `ARARABOTS_SEM_NOME=1` ou
  `/tmp/ararabots_sem_nome`.
- `experimento.ligado("NOME")` — liga algo que foi medido como pior e ficou só
  para remedir. Liga com `ARARABOTS_NOME=1` ou `/tmp/ararabots_nome`.
- Nome novo entra na tupla `CHAVES` / `CHAVES_LIGA` com uma linha dizendo o que
  faz; nome desconhecido levanta erro.
- Nunca `os.environ.get(...)` direto em tática, papel ou skill. Lembre de
  exportar a variável no `ros_d` do `docs/ararabots.sh`.

**R11 — Diagnóstico só por `strategy/diagnostico.py`** (`ativo()`, `log()`,
`log_e_arquivo()`), atrás do `DIAG_JOGO=1`. Diagnóstico nunca muda o que o robô
faz. Use as etiquetas existentes (`[JG]` jogo, `[TR]` troca de papel, `[BT]`
árvore, `[LG]` largada, `[AB]` teste A/B, `[FK]` falta) e não mude o formato de
uma linha que uma ferramenta lê (`[JG] MOV`, `[JG] CheckState cmd=`).

**R12 — Goleiro e campo têm fonte única.** O id do goleiro é
`context.GOALKEEPER_ID` (o mesmo vai para o `SetGoalKeeper`). Gols e medidas
saem de `campo.py`. Nada de `rid == 0` ou `4500.0` solto.

**R13 — Nomes:** pastas e infraestrutura em inglês (como já eram); termos de jogo
em português (`portador`, `alvo`, `fase`). Não misture os dois no mesmo nome.

**R14 — Testes.** Ver seção 5.

---

## 4. Receitas

### 4.1 Adicionar um papel novo

Exemplo: um `MARCADOR` que gruda no adversário mais perigoso.

1. **Nome**, em `roles/atribuicao.py`:

   ```python
   PAPEL_MARCADOR = "marcador"
   ```

2. **Quem recebe**, em `distribuir_papeis`. Diga no docstring do módulo em que
   ordem de prioridade ele entra quando faltam robôs.

3. **O papel**, em `roles/marcador.py`:

   ```python
   """MARCADOR: fica entre a ameaça e o nosso gol, encostado."""

   from strategy.analysis.jogo import AnaliseJogo
   from strategy.roles.atribuicao import PAPEL_MARCADOR
   from strategy.roles.base import Papel
   from strategy.skills import posicionamento


   class Marcador(Papel):
       nome = PAPEL_MARCADOR

       def alvo(self, a: AnaliseJogo, rid: int, ordem: int):
           ameaca = a.ameaca                     # vem da análise (R4)
           if ameaca is None:
               return posicionamento.cobertura_na_linha(
                   a.bola.position_x, a.bola.position_y, a.mundo.nosso_gol)
           return posicionamento.marcar(ameaca, a.mundo.nosso_gol)
   ```

4. **O fato novo na análise** (aqui, `ameaca`): calcule em `analysis/` e
   acrescente o campo em `AnaliseJogo`. Ver 4.3.

5. **Registro**, em `tatics/running.py`:

   ```python
   self.papeis = {..., PAPEL_MARCADOR: Marcador()}
   ```

6. **Teste**, em `test/test_jogo_corrido.py` ou num arquivo próprio: monte um
   `TickContext` à mão e confira o alvo e as bandeiras do comando.

### 4.2 Mudar o comportamento de um papel

1. Mexa só no arquivo do papel (ou na skill que ele usa).
2. Constante nova: R9. Comportamento antigo que precisa continuar medível: R10.
3. Rode os testes (seção 5) e meça no simulador antes do PR — a mudança
   **deve** mudar a saída do `gravar_saida`; diga no PR o que mudou e o número
   medido.

### 4.3 Acrescentar um fato à análise

1. Função pura em `analysis/<assunto>.py`, recebendo `Mundo` (e o que mais
   precisar já calculado), devolvendo dado — nunca `Skill`.
2. Se precisar lembrar de ciclos anteriores, a memória é uma `dataclass` nova
   dentro de `MemoriaAnalise`.
3. Campo novo em `AnaliseJogo`, preenchido em `analisar()`.

### 4.4 Acrescentar uma jogada (estado do árbitro)

1. `plays/<jogada>.py`: uma `Sequence` com `CheckState(...)` e uma ação
   (`LeafNode`) que devolve `RUNNING` enquanto faltar mundo ou configuração.
2. Memória da jogada (como `EstadoFreekick`) é criada no `__init__` da play e
   injetada na ação.
3. Registre em `root.py`. **A ordem importa:** o `Selector` para no primeiro
   filho que não devolve `FAILURE`, e um `RUNNING` bloqueia os seguintes.
4. Teste em `test/test_behaviour_tree.py` (há um parametrizado por comando).

### 4.5 Acrescentar um dado do mundo

Campo novo em `TickContext`, preenchido em `TickContext.from_game_state`. Se vier
de serviço (como `GameConfig`), quem consulta é o `strategy.py`, uma vez.

---

## 5. Testes e medição

| o quê | como | quando |
|---|---|---|
| Testes sem ROS | `cd src/strategy && PYTHONPATH=.:../utils python3 -m pytest -q test` | sempre |
| Testes no container | `scripts/vice test strategy` | antes do PR |
| Refatoração não muda nada | `tools/gravar_saida.py` antes e depois, `cmp` (instruções no topo do arquivo) | toda refatoração |
| Mudança de comportamento | `docs/ararabots.sh validar ...` ou `campanha` | toda mudança de comportamento |

O `gravar_saida` roda a árvore sobre milhares de mundos aleatórios com semente
fixa e grava cada comando de cada ciclo. Se o arquivo de antes e o de depois são
idênticos, a refatoração não mudou o que os robôs fazem. Foi assim que a
reorganização em papéis foi verificada: 112 500 ciclos, saída idêntica, byte a
byte.

---

## 6. Checklist do PR

- [ ] Nenhum `import rclpy`, `create_subscription` ou `create_client` fora do `strategy.py` / `movement_handler.py`.
- [ ] Nenhum `os.environ` fora de `experimento.py` e `diagnostico.py`.
- [ ] Nenhuma chave de dicionário nova como memória; memória em `dataclass`.
- [ ] Papel novo registrado e com teste.
- [ ] Constante nova com número e fonte em ≤ 2 linhas.
- [ ] `pytest` passando.
- [ ] Refatoração: `gravar_saida` idêntico. Mudança de comportamento: número medido no PR.

---

## 7. Trazendo um branch antigo para esta estrutura

Para quem tem trabalho feito sobre o `tatics/running.py` antigo (antes de
20/10/2026). A lógica não mudou de comportamento — só de lugar — então a
migração é mover o seu diff para o arquivo novo correspondente.

| antes | agora |
|---|---|
| `situacao_de_jogo`, `RAIO_POSSE`, `SITUACAO_*` | `analysis/mundo.py` |
| `distribuir_papeis`, `PAPEL_*`, `VANTAGEM_TROCA`, `CICLOS_MIN_PAPEL` | `roles/atribuicao.py` |
| `alvo_do_chute`, `mira_firme`, `_alvo_do_chute_cru`, `_mira_existe` | `analysis/alvo_chute.py` |
| saída de bola e alívio (dentro de `montar_comandos`) | `analysis/alvo_chute.py` (`_saida_de_bola`, `_alivio`) |
| `alvo_do_papel`, ramo `PAPEL_PORTADOR` | `roles/portador.py` (`Portador.alvo`) |
| velocidade de chegada e `cmd.ball` do portador (em `montar_comandos`) | `Portador.velocidade_chegada`, `Portador.evita_bola` |
| `alvo_do_papel`, ramo `PAPEL_APOIO` | `roles/apoio.py` |
| `alvo_do_papel`, ramo `PAPEL_COBERTURA` | `roles/cobertura.py` |
| laço por robô de `montar_comandos` (`chutar_em`, bandeiras) | `roles/base.py` (`Papel.comando`) |
| goleiro em `Atack.execute` / `Defense.execute` | `roles/goleiro.py` |
| `Atack`, `Defense`, `montar_comandos` | `tatics/running.py` (`JogoCorrido`) |
| `CheckAtack` / `AtackAction` / `DefenseAction` | `plays/running.py` (`JogoCorridoAction`): as duas rodavam o mesmo código |
| `EstadoJogo.registrar`, `create_client(GetGameConfig)` nas folhas | `context` no `run()` (R1, R2) |
| `CenterGoal` de `stop.py`, `kickoff.py`, `running.py` | `campo.py` |

E as chaves do antigo dicionário `estado`:

| `estado[...]` | agora |
|---|---|
| `portador`, `portador_ciclos`, `apoio`, `apoio_ciclos` | `MemoriaPapeis` |
| `mira_tipo`, `mira_ponto`, `mira_ciclos`, `mira_crua`, `mira_segurada`, `alvo_passe` | `MemoriaMira` |
| `fase_portador`, `ciclos_mirando`, `erro_mira` | `MemoriaPortador` (`Portador.memoria`) |
| `chute_armado` | `MemoriaJogo.travas_chute` |

---

## 8. O que ainda não segue estas regras

Dívida conhecida, para quem for mexer ali:

- **`tatics/freekick.py`** (≈2 600 linhas): tem a sua própria aproximação e armamento
  de chute, e um `OurFreekick` de ≈1 600 linhas. É o código com resultado medido
  (5 gols em 6); migrar para papéis exige lote medido. Lê `MIRA_CANTO`, `DIAG_FK` e
  `MOVIMENTO_NOVO` direto do ambiente.
- **`tatics/kickoff.py` e `tatics/stop.py`**: cada um tem a sua cópia de
  `GoalkeeperKickoff`.
- **`tatics/goalkeeper.py`**: o goleiro ainda é chamado como tática, por fora da
  interface `Papel`.
- Comentários longos herdados nas skills: quando mexer numa função, encurte o
  comentário dela para o formato da R9 e leve o histórico para o commit.
