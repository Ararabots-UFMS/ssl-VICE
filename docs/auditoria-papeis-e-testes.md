# Estratégia Ararabots — análise por papel, plano de camadas e bateria de testes

Documento de trabalho da coordenação de estratégia, para a equipe.

**Base:** branch `refactor/referee-commands`, após a correção de papéis de
**02/10/2026**. Caminhos relativos a `src/strategy/strategy/` salvo indicação em
contrário.

**Método:** leitura integral do pacote `strategy`, com citação `arquivo:linha`.
Onde há número, a fonte é medição registrada em `documentacao/` ou no comentário
do próprio código, e está dita. Onde um comportamento nunca foi medido, está
escrito.

**Para que serve:** organizar a estratégia em papéis e camadas, separar o que
funciona do que não funciona, e fixar a bateria de testes que decide cada caso.
É a referência para distribuir trabalho e para julgar se uma mudança melhorou
algo.

---

## 1. O modelo de papéis e o que o código fazia

A estratégia passa a ser descrita por **quatro papéis** — portador, apoio,
cobertura e goleiro — em **três camadas**:

| camada | o que decide |
|---|---|
| **Plays** | decisões de nível de time: qual jogada se aplica, quem faz o quê, quantos vão à bola |
| **Tactics** | comportamento de um robô dentro da play: para onde ir, como chegar |
| **Skills** | ações atômicas: ir à bola, apresentar a face, armar/chutar, posicionar-se, interceptar |

### 1.1 Correção aplicada em 02/10/2026: os papéis estavam com os nomes trocados

Até essa data o rótulo `PAPEL_APOIO` era atribuído a quem ia **à bola** (chamado
"buscador" nos comentários) e `PAPEL_PORTADOR` a quem ficava adiantado esperando.
A troca não era cosmética: `alvo_do_chute` escolhe o receptor do passe entre os
`PAPEL_APOIO` — isto é, escolhia justamente quem estava indo disputar a bola.

Medição por sonda de decisão offline, sem simulador (as funções envolvidas são
puras e recebem dicionários de robôs):

```
receptor do passe == robô mais próximo da bola        3 de 3 cenários
cenário 'posse nossa': receptor a 100 mm da bola
```

Como **todas** as direções do ciclo derivam de `alvo_chute` — orientação do corpo,
ponto de atravessar a bola, "espaço à frente" do apoio — a consequência era o time
apontar e deslocar-se para trás:

| | antes | depois |
|---|---|---|
| posse nossa, corpo do apoio | **−139,9°** (aponta para a própria meta) | passe mira o apoio em (2400, 800) |
| largada, alvo do apoio | **(−1500, 0)**, com a bola em (0,0) | (1500, 0) |
| largada, tipo de alvo | `passe` (degenerado) | `bloqueado` → saída/alívio |

A trava de segurança então recusava armar o chute, porque a direção de saída
apontava para a própria meta. O laço era fechado: **gol bloqueado → passe
degenerado → direções invertidas → sem chute.** Isso explica o `alvo_chute=passe`
em 250 de 250 ciclos melhor do que a folga de linha do item B1, porque o segmento
bola→robô-mais-próximo é curto e quase sempre livre.

O item B1 (nota contínua de chute) **continua valendo**, mas deixou de ser a única
explicação para a ausência de gols.

> ⚠️ **A correção não foi medida em simulador.** Verificação feita por análise
> sintática e pela sonda de decisão offline. O primeiro lote de 6 em `jogo` é o
> que decide.

### 1.2 Divergências estruturais que permanecem

1. **A camada Skills passou a existir em 02/10/2026** (seção 6). Antes,
   `skills/skills.py` era um DTO e as mesmas cinco ações atômicas estavam
   implementadas três vezes, em `tatics/running.py`, `tatics/freekick.py` e
   `tatics/goalkeeper.py`. O jogo corrido e o goleiro já consomem a camada; a
   cobrança de falta mantém a sua lógica de armamento e aproximação, por ser o
   código com resultado comprovado (5 gols em 6) — migração prevista, com lote
   medido.
2. **A camada Plays quase não decide.** `root.py:14` é um `Selector` que escolhe
   jogada **só pelo comando do árbitro**. Não há play por situação de jogo. Toda a
   decisão de time — quantos vão à bola, quem ataca, quando passar — está dentro
   de uma função de tática, `montar_comandos` (`tatics/running.py:785`),
   compartilhada por `Atack` e `Defense`. A única decisão de time fora dela é
   `CheckAtack` (`plays/running.py:79`).
3. **O goleiro não passa pela distribuição de papéis.** É sempre o robô 0, fixado
   em três lugares independentes: `tatics/running.py:67`, o `rid == 0` de
   `montar_comandos`, e `strategy.py:389` (`SetGoalKeeper(0)`). O código dele é
   separado e tem problemas próprios — por isso ganha seção própria aqui.

---

## 2. Papel: PORTADOR

Quem tem a bola ou vai buscá-la. Escolhido primeiro, o mais próximo dela
(`distribuir_papeis`, `tatics/running.py:143`).

| Camada | O que existe hoje | Arquivo:linha | Funciona? |
|---|---|---|---|
| Plays | Eleição do portador: o mais próximo da bola, com histerese de `VANTAGEM_TROCA = 1200 mm` **e** `CICLOS_MIN_PAPEL = 20` ciclos | `tatics/running.py:251-278` | **OK** — medido: 133 trocas em 250 ciclos para 1 |
| Plays | Quantos vão à bola: um em `SOLTA`, dois em `DELES`/`DISPUTA` (portador + cobertura), um em `NOSSA` | `tatics/running.py:143` (docstring) e os ramos de `alvo_do_papel` | **OK** — a regra é coerente com a medição de que eles chegam com três e o time com um |
| Plays | Decisão de alvo da bola: gol → passe no apoio → nada | `tatics/running.py:391` (`alvo_do_chute`) | **parcial** — o receptor foi corrigido (1.1), mas `linha_livre` (`skills/geometria.py:66`) segue **binário**, com folga fixa de 180 mm. Com o goleiro adversário sobre a reta (medido: 6 mm), "gol" raramente é alvo válido. É o item B1 |
| Plays | Rede de segurança sem alvo: **saída de bola** no terço defensivo (direção de maior folga) e **alívio** lateral sob pressão | `tatics/running.py:848-917` | **OK** — `bloqueado` caiu de 142 para 11 ciclos |
| Tactics | Aproximação contínua em dois regimes interpolados pela distância: longe vale o avanço, perto vale atravessar a janela de disparo; desvio lateral de contorno que **zera na chegada** | `skills/aproximacao.py:82` (`ponto_de_aproximacao`) | **OK; é o trecho mais maduro do jogo corrido.** Cada constante tem medição no comentário (`PONTO_CHUTE = 94`, `VIES_LATERAL = -27`, `ATRAVESSA_CHUTE = -53`) |
| Tactics | Não larga a bola em `NOSSA` (corrigido em 02/10) | ramo PORTADOR, `tatics/running.py:529-546` | **corrigido, não medido** — antes quem estava a 100 mm da bola era enviado 1,8 m à frente enquanto o adiantado vinha buscá-la |
| Tactics | Interceptar em vez de perseguir quando a bola já saiu (`VEL_BOLA_CHUTADA = 250 mm/s`) | `skills/aproximacao.py:64` (`ponto_de_interceptacao`) | **OK** — era código inalcançável (escrito depois de um `return`); corrigido |
| Tactics | "Nunca chegar": o alvo final fica **além** da bola | `ATRAVESSA_CHUTE`, `skills/aproximacao.py:54` | **contorno de defeito externo** — `control.py:120,139` só comanda robô com trajetória ativa, então chegar ao ponto cala o canal de chute |
| Skills | Portão de chute: arma com `d < 260 mm`, mantém armado até 420 mm, exige bola à frente da placa, proíbe saída na direção da própria meta; força 6,0 / 4,5 / 2,5 conforme o alvo | `skills/chute.py:109` (`armar_chute`) | **parcial** — o portão está correto e as travas vieram de medição; o conjunto não fecha: chute sai em campo limpo (3 de 3, 5555-5944 mm/s) e **nenhum gol com adversário** em ~40 execuções |
| Skills | Orientação: a menos de `RAIO_ORIENTA_CHUTE = 700 mm` o corpo aponta na direção do chute; além disso, para a bola | `tatics/running.py:952-975` | **parcial** — a regra está certa, mas a orientação viaja por serviço separado e não é aplicada a robô sem trajetória ativa (`control.py:120,139`) |
| Skills | Conduzir, driblar | — | **ausente** — `spinner = 0` em `grsim_messenger/grsim_publisher.py:57`: o dribbler nunca é acionado. Conduzir é empurrar com o casco |

### Adições previstas — PORTADOR

**Plays**
- **Nota contínua de chute (item B1).** Substituir `linha_livre` por varredura
  angular da boca do gol, mirando a bissetriz do maior vão livre. É a causa medida
  da ausência de gols e é validável sem simulador.
- **Nota de passe multiplicativa (item B2).** Hoje o passe é binário e o primeiro
  apoio livre vence; passa a ser pontuado por distância, ângulo de recepção e
  pressão sobre o receptor.
- **Decidir entre chutar, passar e conduzir por comparação de notas**, em vez de
  ordem fixa de preferência.

**Tactics**
- **Condução deliberada.** Hoje "levar a bola para frente" é efeito colateral de
  atravessá-la na direção do alvo. Falta tratar progressão como intenção, com
  limite de quantos metros se conduz antes de reavaliar.
- **Proteção de posse sob pressão.** Com adversário a menos de `PRESSAO_RAIO`, o
  corpo entre o adversário e a bola, em vez de só mirar o alívio lateral.

**Skills**
- ✅ **`chute.armar_chute(robô, bola, alvo, …, travas, rid)`** — feito: o portão do
  jogo corrido virou skill, com as três travas (armamento, bola à frente da placa,
  direção segura). A bola parada **ainda não** usa — ver 6.3.
- ✅ **`aproximacao.ponto_de_aproximacao(…)`** — feito para o jogo corrido
  (contorno contínuo + ponto de chute). A bola parada mantém as fases discretas;
  unificar as duas é o próximo passo e exige lote medido.
- ✅ **`aproximacao.ponto_de_interceptacao(…)`** — feito: o cálculo estava repetido
  em três ramos.
- ⬜ **`chutar_em(robô, ponto)`** — não é expressável hoje (três canais distintos;
  ver 6.2). Só depois de o laço do `control` ser corrigido.

---

## 3. Papel: APOIO

Quem ajuda sem a bola. O mais adiantado entre os que sobram. Desde 02/10 o ramo
decide por cenário: ataque, marcação, espaço ou recuo.

| Camada | O que existe hoje | Arquivo:linha | Funciona? |
|---|---|---|---|
| Plays | Eleição do apoio: o mais adiantado entre os que não são portador, com a mesma histerese dupla | `tatics/running.py:284-305` | **OK** |
| Plays | O apoio é o **receptor do passe**, e precisa estar a `AVANCO_MINIMO_PASSE = 600 mm` à frente da bola | `tatics/running.py:391-487` | **corrigido em 02/10, não medido** — ver 1.1. A guarda de avanço vem da lição já paga em `freekick.py` (`_companheiro_para_passe`) |
| Plays | Alvo de passe **congelado** no instante da decisão, re-verificado contra a posição atual do receptor | `tatics/running.py:438-462` | **OK** — sem o congelamento, o ponto de encaixe saltava a cada ciclo. Medido: janela do chutador aberta 1272 quadros com o chute armado em zero deles |
| Plays | Nota contínua de posição ("onde se oferecer") | — | **ausente** — a posição do apoio é fórmula geométrica fixa, não avaliação (itens B2/B9) |
| Tactics | **Ataque:** 1800 mm à frente da bola e 1600 mm aberto, alternando o lado pela ordem | `skills/posicionamento.py:72` (`oferta_de_passe`) | **OK, calibrado por regressão** — com 1200/900 ficava dentro do corredor do empurrão e o chute caiu de 5525 para 1536 mm/s. Medido e revertido duas vezes |
| Tactics | **Marcação:** adversário com a posse → planta-se entre bola e própria meta a `BLOQUEIO_DIST = 600`; sem dono → marca a ameaça (adversário mais próximo da própria meta) a `MARCACAO_DIST = 400` | `tatics/running.py:698-731` | **novo em 02/10, não medido** — o gatilho é `DELES` **ou** zona de perigo (3000 mm). `DISPUTA` foi deliberadamente excluída: é 51% do tempo e marcaria metade do jogo, esvaziando o ataque |
| Tactics | **Espaço:** em `SOLTA`, 1500 mm adiante da bola, 30% disso em y, na direção do **gol de ataque** | `tatics/running.py:733-744` | **corrigido em 02/10** — a direção saía de `alvo_chute` e, com o passe degenerado, apontava para trás |
| Tactics | **Recuo:** via obstruída (`alvo_chute == bloqueado`) → 900 mm atrás da bola | `tatics/running.py:746-764` | **novo em 02/10** — usa `APOIO_RECUO` e o parâmetro `bloqueado`, que existiam e nunca eram lidos |
| Skills | Receber, dominar, primeiro toque | — | **ausente** — sem dribbler, "receber" é estar parado no lugar certo e a bola chegar |
| Skills | Giro para encarar o passador | `tatics/running.py:952-975` | **parcial** — regra correta; robô parado não gira, por `control.py:120,139` |

### Adições previstas — APOIO

**Plays**
- **Mapa de oferta.** Avaliar alguns pontos candidatos por (linha de passe livre,
  distância ao gol, pressão) e escolher o melhor, em vez de uma fórmula fixa.
- **Dosagem do gatilho de marcação.** Hoje é binário (`DELES` ou zona de perigo).
  Deve virar uma nota de risco, para o apoio recuar proporcionalmente ao perigo em
  vez de alternar entre dois extremos.

**Tactics**
- **Marcação por ameaça real (item B6)**, no lugar de "o adversário mais próximo
  da nossa meta": ponderar quem tem linha de recepção livre.
- **Recepção orientada:** chegar ao ponto de passe já com o corpo na direção da
  continuação da jogada, não voltado para o passador.

**Skills**
- ✅ **`posicionamento.oferta_de_passe` / `recuo_de_apoio` / `ocupar_espaco`** —
  feitos. A versão da bola parada (`_posicao_de_apoio`) segue separada.
- ✅ **`posicionamento.marcar` / `bloquear_linha` / `ameaca_mais_perigosa`** —
  feitos, com as duas constantes (`BLOQUEIO_DIST`, `MARCACAO_DIST`) num só lugar.
- ⬜ **`oferecer_linha` com busca** — a oferta atual é fórmula fixa; falta
  escolher entre candidatos por linha livre (ver Plays deste papel).
- ⬜ **`receber(robô, ponto, direção_de_continuação)`** — só faz sentido pleno com
  dribbler; no escopo atual, posicionar e orientar.

---

## 4. Papel: COBERTURA

Entre a bola e a própria meta. Em `DELES`/`DISPUTA` é a segunda a ir à bola.

| Camada | O que existe hoje | Arquivo:linha | Funciona? |
|---|---|---|---|
| Plays | Todo robô que não é portador nem apoio é cobertura; com menos robôs que papéis, a cobertura tem prioridade sobre o apoio | `tatics/running.py:280-283, 299-305` | **novo em 02/10** — antes, com dois robôs de linha, saíam portador e apoio e **ninguém cobria** |
| Plays | Segunda a ir à bola em `DELES`/`DISPUTA`, entrando pelo lado oposto ao do portador | `tatics/running.py:604-608` | **OK no desenho** — dois ângulos de ataque dão duas saídas possíveis, o que é o correto sem dribbler. É também o mecanismo que mais contribui para amontoamento |
| Tactics | Posicionamento **sobre** a reta bola→própria meta, a `COBERTURA_FRACAO = 45%` do caminho, com os demais espalhados **ao longo** dela | `skills/posicionamento.py:116` (`cobertura_na_linha`) | **OK; correção medida** — o deslocamento antigo era perpendicular e tirava a cobertura do corredor do chute. Medido: a bola percorria 4122-5437 mm em linha reta até a linha de fundo e **só o goleiro a tocava** |
| Tactics | "Matar a jogada": com a bola a menos de `COBERTURA_MATA = 2500 mm` da própria meta, abandona a linha e vai **na** bola | `tatics/running.py:642-652` | **parcial** — a lógica é correta (bloquear a 45% não chega a tempo contra chute de 0,8 s), mas só o robô de ordem 0 faz isso, e três comportamentos disputam a mesma região do campo (`COBERTURA_MATA = 2500`, `TERCO_DEFENSIVO = 3000`) sem um lugar único que decida |
| Skills | Bloquear linha, interpor corpo | `skills/posicionamento.py:116` (`cobertura_na_linha`) e o ramo de marcação do apoio | **parcial** — duas implementações, constantes diferentes, e **nenhuma usa a velocidade da bola** |
| Skills | Interceptação por tempo-até-chegar | — | **ausente** — a cobertura escolhe ponto por fração de distância, não por quem chega antes |

### Adições previstas — COBERTURA

**Plays**
- **Atribuição por ameaça, não por sobra.** Hoje é cobertura quem não virou
  portador nem apoio. Com mais de um adversário perigoso, a segunda cobertura
  deveria ser alocada a uma ameaça específica (item B6).
- **Regra única para a região defensiva.** Unificar `COBERTURA_MATA`,
  `TERCO_DEFENSIVO` e o gatilho de marcação do apoio numa só definição de "zona de
  perigo", hoje três números próximos em três lugares.

**Tactics**
- **Bloqueio por tempo de chegada**, usando a velocidade da bola: escolher o ponto
  da reta que o robô alcança antes da bola, em vez de uma fração fixa.
- **Barreira de falta (item B5).** Não existe, e é conformidade com a regra 5.3.3,
  não só tática.

**Skills**
- ✅ **`posicionamento.cobertura_na_linha` e `bloquear_linha`** — feitos; a
  cobertura e o apoio usam a mesma implementação.
- ⬜ **critério por TEMPO de chegada**, e não por fração de distância: exige a
  velocidade da bola, que nenhuma das duas usa hoje.
- ⬜ **`interceptar_trajetoria`** com a geometria da trajetória da bola (hoje só
  existe o ponto previsto a 0,5 s, conservador por construção).

---

## 5. Papel: GOLEIRO

Sempre o robô 0. Não passa por `distribuir_papeis`; o comando dele é prefixado em
`Atack.execute` e `Defense.execute` (`tatics/running.py:1346`, `:1391`).

| Camada | O que existe hoje | Arquivo:linha | Funciona? |
|---|---|---|---|
| Plays | O robô 0 é o goleiro, fixado em três lugares independentes | `tatics/running.py:67`, `montar_comandos` (`rid == 0`), `strategy.py:389` | **parcial** — funciona, mas não há um lugar único que defina isso; trocar o goleiro exige editar três arquivos |
| Plays | O goleiro nunca é escolhido para cobrar falta nem para disputar | `tatics/running.py:67`, cenário `um_so_goleiro` | **OK** — há cenário de teste dedicado |
| Plays | Decisão de sair jogando: passe no companheiro livre → chute no espaço → empurrão | `tatics/goalkeeper.py:166` | **parcial** — a ordem de preferência é correta; a avaliação que a alimenta está quebrada (linha abaixo) |
| Tactics | Área de defesa correta da Division B: `AREA_PROFUNDIDADE = 1000`, `AREA_MEIA_LARGURA = 1000`, fonte `~/.grsim.xml`; `padding` que **aumenta** a área | `tatics/goalkeeper.py:20-21, 124-165` | **OK; correção medida** — a condição anterior (`\|y\| < 600`) dava **16,2% de falso negativo** em 8472 quadros: bola na área e goleiro parado na linha, numa execução por 987 quadros seguidos |
| Tactics | Posicionamento fora da área: centro da meta alinhado ao y da bola | `tatics/goalkeeper.py:166-202` | **parcial** — não antecipa pela orientação do atacante nem pela velocidade da bola (item B7) |
| Tactics | Varredura de cinco pontos na meia altura para escolher onde chutar | `tatics/goalkeeper.py:39-72` | **quebrado** — ver skills |
| Skills | Checagem de linha para escolher companheiro e espaço | `tatics/goalkeeper.py:48` e `:94` | **OK** — passa as coordenadas do destino. O defeito registrado em `strategy-analysis/ours/current-strategy.md` (problema 4), em que se passava um **versor** no lugar do destino e a checagem não conferia linha nenhuma, **foi corrigido em `ef74a5e`** e o comentário no código documenta o conserto. Nunca foi medido o efeito da correção |
| Skills | Empurrão para fora da área (ponto 70 mm atrás da bola, direção meta→bola) | `tatics/goalkeeper.py:166-202` | **parcial** — funciona (medido: `CHUTE AZUL 0, 5463 mm/s, bola andou 2558 mm`), mas é uma terceira implementação de "chegar atrás da bola e empurrar" |
| Skills | `AVANCO_MINIMO_PASSE = 600` duplicado | `tatics/goalkeeper.py:10` (agora um alias) e `skills/posicionamento.py:38` | **duplicação** — mesmo valor, mesmo motivo, dois arquivos |

### Adições previstas — GOLEIRO

**Plays**
- **Fonte única do id do goleiro.** Uma definição consumida pelos três pontos que
  hoje fixam `0` independentemente, e pelo serviço `SetGoalKeeper`.
- **Decisão de sair da meta.** Hoje é binária (bola na área ou não). Deve
  considerar se há companheiro em condição de receber e se o adversário chega
  antes.
- **Tratamento de pênalti.** `PREPARE_PENALTY_*` não tem play; o goleiro não tem
  comportamento definido para a cobrança contra.

**Tactics**
- **Antecipação pela orientação do atacante (item B7)** e pela velocidade da bola,
  em vez de só alinhar o y.
- **Limite explícito de saída.** Medido: o goleiro nunca passou de x = −3500, mas
  não há limite no código — é consequência da geometria, não decisão.

**Skills**
- **Medir o efeito da correção da checagem de linha** (feita em `ef74a5e`, nunca
  medida). O cenário C-4 existe para isso.
- **Reusar `espaço_mais_aberto` da camada compartilhada**, em vez da varredura
  local de cinco pontos — hoje o goleiro já usa `skills.geometria.linha_livre`,
  mas a varredura e o critério de folga continuam dele.
- ⬜ **Reusar `aproximacao.ponto_de_aproximacao`** para o empurrão, eliminando a
  terceira cópia — hoje o goleiro ainda tem a sua (ponto 70 mm atrás da bola).
- ✅ **Deixou de importar de outra tática:** usava
  `from strategy.tatics.running import linha_livre` dentro de dois métodos; agora
  consome `skills.geometria`.

---

## 6. A camada de Skills: inventário, causas e plano

### 6.1 As mesmas ações, em três cópias incompatíveis

| ação atômica | `tatics/running.py` | `tatics/freekick.py` | `tatics/goalkeeper.py` |
|---|---|---|---|
| ir à bola / aproximar | maquinário contínuo do ramo PORTADOR (contorno + ponto de chute interpolados) | `_aproximar_da_bola:1786`, `_contornar_a_bola:2280`, `_fase_da_cobranca:2125`, `_passo_ate:1651` | ponto 70 mm atrás da bola, em `execute:164` |
| apresentar a face | `_off`/`_lat` que zeram na chegada | `_geometria_do_chutador:895`, `_corpo_alinhado:1053`, `_atras_da_bola:2311` | — |
| armar / chutar | portão em `montar_comandos` + trava em `estado["chute_armado"]` | `_chute_deve_estar_armado:933`, `_armar_se_der:982`, `_forca_do_chute:1107` | `kick` direto no comando |
| escolher alvo | `alvo_do_chute:391` + `linha_livre:559` | `_ponto_de_mira:1209`, `_folga_do_tiro:1171`, `_decidir_alvo_da_jogada:1451` | `_espaco_livre_a_frente:37`, `_companheiro_livre:71` |
| posicionar-se / receber | ramos APOIO e COBERTURA | `_posicao_de_apoio:1589`, `_companheiro_para_passe:1351` | — |
| recuar após o chute | — | `_recuar_apos_chute:2018` | — |
| segurança (duplo toque, timeout) | — | `_check_double_touch:816`, `_parada_segura:822` | — |

E `skills/skills.py` — o arquivo com "skills" no nome — é um DTO: `move_to`,
`move_with_angle`, `set_orientation`, `obstacles`, `stop`, mais `activate_kick`
(1,5 m/s, valor da `dev`, nunca chamado).

### 6.2 Por que a camada não nasceu

1. **O canal de comando é pobre e tripartido.** Uma skill `chutar_em(ponto)` não é
   expressável numa chamada: `MovementCommand` tem `target_pos`, `target_vel` e
   `planning_options`; **orientação** vai pelo serviço `set_orientation` e
   **chute** pelo `update_kick`, ambos direto ao pacote `control`.
2. **O terceiro canal vaza para dentro da skill.** `control.py:120,139` só publica
   comando para robô com referência de trajetória ativa; logo, chegar ao destino
   **cala** o canal de chute e de orientação. A solução no código é o alvo ficar
   além da bola (`ATRAVESSA_CHUTE`) — ou seja, a assinatura da skill teria de
   incluir "nunca chegue ao destino". É defeito de outro pacote virando interface.
3. **As skills são stateful; as táticas são recriadas a cada ciclo.** A janela de
   disparo do grSim dura **uma amostra**, então armar exige trava entre ciclos,
   mas `Atack`/`Defense` são construídas de novo a cada tick. Até o dicionário
   `estado` injetado existir, não havia onde o estado morar.
4. **Falta atuador.** `spinner = 0`: o dribbler nunca é acionado. Metade do
   vocabulário de skills da divisão pressupõe dribbler.
5. **Duas linhagens que nunca puderam ser fundidas.** A bola parada cresceu como
   métodos acoplados ao `EstadoFreekick`; o jogo corrido, como fórmulas dentro de
   `alvo_do_papel`. Extrair uma exigia o estado da outra. O custo está medido: a
   trava de armamento, a guarda de "bola à frente da placa" e o alvo de passe
   congelado foram descobertos na bola parada e redescobertos meses depois no jogo
   corrido.

Razão estrutural de fundo: cada folha da árvore **é um nó ROS**
(`behaviour.py:13`), o que desencoraja unidades pequenas e puras. `context.py`
(`TickContext` imutável, sem `rclpy`) é o começo do conserto e está escrito,
testado e **não ligado** — `commons/check_state.py:3` importa `RunResult`, símbolo
que não existe no `behaviour.py` vigente.

### 6.3 A camada, como ficou (02/10/2026)

```
strategy/skills/
  geometria.py       linha_livre, livre_do_lado, norm_ang, versor, no_campo,
                     projecao_no_eixo, folga_lateral          (nivel 1)
  bola.py            bola_ja_saiu, onde_a_bola_vai, VEL_BOLA_CHUTADA   (nivel 1)
  chute.py           geometria_do_chutador, na_janela_de_disparo,
                     direcao_para_frente, armar_chute, forca_por_alvo  (nivel 2)
  aproximacao.py     ponto_de_aproximacao, ponto_de_interceptacao      (nivel 2)
  posicionamento.py  oferta_de_passe, recuo_de_apoio, ocupar_espaco,
                     bloquear_linha, marcar, ameaca_mais_perigosa,
                     cobertura_na_linha, pode_receber                  (nivel 2)
  skills.py          o DTO `Skill`/`Skills`, como antes
```

**Três regras da camada**, escritas no `skills/__init__.py`:

1. nada ali importa `rclpy` — toda função recebe dados e devolve dados, o que
   permite testar decisão sem simulador;
2. a constante **medida** mora junto da skill que a usa, com o número e a fonte
   no comentário; constante medida escrita em dois lugares é defeito (era o caso
   de `AVANCO_MINIMO_PASSE`, em `goalkeeper.py` e `running.py`, e dos cinco
   números da geometria do chutador, em `freekick.py` e `running.py`);
3. estado, quando inevitável (a trava de armamento), entra como parâmetro.

**Quem já consome a camada:** `tatics/running.py` (jogo corrido, inteiro),
`tatics/goalkeeper.py` (predicado de linha e constante de passe — e deixou de
importar de outra tática) e `tatics/freekick.py` (constantes físicas do chutador
e predicado de linha).

**Verificação:** teste-ouro de 49 cenários × 3 saídas (papéis, comandos do time,
comando do goleiro) = **147 verificações, zero diferenças** antes e depois da
extração, com e sem `DIAG_JOGO`. Isto prova que a extração **não mudou
comportamento** — não que o comportamento esteja certo.

**O que ficou de fora, e por quê:** a lógica de armamento e de aproximação da
cobrança de falta. Ela usa um ponto de referência próprio (centro da placa, não a
face) e é o código com resultado comprovado da base. Migrar muda números que
ninguém mediu de novo; a migração exige lote medido. A única divergência conhecida
na delegação do predicado de linha é o caso degenerado (origem e destino a menos
de 1 mm), verificado em 200 mil casos aleatórios: **87 divergências, todas nesse
caso**, impossível em campo porque o robô tem 180 mm de diâmetro.

### 6.4 Ordem de construção adotada

1. **Geometria e predicados** (`linha_livre`, versores, normalização de ângulo,
   limites de campo) — sem estado, sem ROS, testável direto.
2. **Chute** (geometria do chutador no referencial do robô, janela de disparo do
   grSim, trava de armamento, força por tipo de alvo) — é onde as duas táticas já
   concordam no critério físico.
3. **Aproximação** (ponto de contorno contínuo, ponto de interceptação).
4. **Posicionamento** (oferta, marcação, bloqueio de linha).
5. Migração da bola parada **somente com lote medido**, por ser o código com
   resultado comprovado (5 gols em 6).

---

## 7. Bateria de testes comportamentais

### Como rodar

```bash
cd Arara_Bots/ssl-VICE/docs
./ararabots.sh preparar --headless     # conferir as quatro taxas antes de medir
```

Convenção dos cenários (`docs/ararabots.py:74-89`): time **azul**, defende x
negativo, ataca +x; gol adversário em x = +4500 com boca |y| ≤ 500; robô **0** é
goleiro. Coordenadas em mm.

Regras de medição (`documentacao/PROMPT_IA.md` §6): lote de 6, headless, uma
mudança por vez, e conferência do sinal de vida do experimento **antes** de olhar
o resultado:

```bash
docker exec vice bash -lc 'grep -c Traceback /tmp/strategy.log'
docker exec vice bash -lc 'grep -o "CheckState cmd=[^ ]*" /tmp/strategy.log | sort | uniq -c'
```

Sete cenários exigem entradas novas no dict `CENARIOS` (`docs/ararabots.py:90`),
especificadas abaixo no schema do arquivo. Os utilitários ausentes estão na seção 9.

### P-1 · Portador · gol do meio-campo com espaço aberto

**Pergunta:** o portador faz gol a partir do meio-campo quando não há ninguém
entre ele e o gol?

```python
"jogo_gol_livre": {
    "titulo": "Portador no meio-campo, caminho livre para o gol",
    "bola": (0.0, 0.0),
    "azuis": [(0, -4300, 0, 0), (1, -600, 0, 0)],
    "amarelos": [(0, 4300, 1200, 180)],     # goleiro deles FORA da reta bola->gol
    "comando": ("FORCE_START", "BLUE"),
},
```

**Execução:** `DIAG_JOGO=1 ./ararabots.sh validar 6 jogo_gol_livre`

**Sucesso:** `gol=SIM` em ≥ 4 de 6; `v_saida_mms ≥ 5000`; `[JG] alvo_chute=gol` em
≥ 80% dos ciclos.

**Falha:** `disparou=NAO` com `arma=True` ⇒ **skill** (janela de disparo; conferir
`[JG] PLACA xx yy` contra 0 ≤ xx < 31,5 e |yy| < 40). `alvo_chute` diferente de
`gol` com o gol livre ⇒ **play**. Bola freada após o disparo ⇒ **tactic**.

### P-2 · Portador · gol com o goleiro adversário na frente

**Pergunta:** com o goleiro deles sobre a reta bola→centro do gol, e a boca aberta
dos dois lados, o portador acha o vão ou desiste de chutar?

```python
"jogo_goleiro_na_linha": {
    "titulo": "Goleiro deles EM CIMA da reta bola->centro do gol",
    "bola": (1500.0, 0.0),
    "azuis": [(0, -4300, 0, 0), (1, 900, 0, 0), (2, 1200, 1500, 0)],
    "amarelos": [(0, 4400, 0, 180)],
    "comando": ("FORCE_START", "BLUE"),
},
```

**Execução:** `DIAG_JOGO=1 ARARABOTS_INIMIGO_PARADO=1 ./ararabots.sh validar 6 jogo_goleiro_na_linha`

**Sucesso:** `alvo_chute=gol` em ≥ 30% dos ciclos **e** `gol=SIM` em ≥ 2 de 6.

**Falha:** `bloqueado`/`saida` em ~100% dos ciclos ⇒ **play**, item B1 confirmado
como causa remanescente. Este é o cenário mais informativo da bateria: separa o
que a correção de 02/10 resolveu do que o B1 ainda deve resolver.

### P-3 · Portador · a bola já chutada não é perseguida nem freada

**Pergunta:** com a bola viajando, o portador intercepta à frente ou corre atrás e
a segura?

```python
"jogo_bola_viajando": {
    "titulo": "Bola em movimento rapido; nosso robo ao lado da trajetoria",
    "bola": (0.0, 0.0), "bola_vel": (3000.0, 0.0),   # CAMPO NOVO (ver seção 9)
    "azuis": [(0, -4300, 0, 0), (1, 300, 700, 0)],
    "amarelos": [(0, 4300, 0, 180)],
    "comando": ("FORCE_START", "BLUE"),
},
```

**Sucesso:** a bola não perde mais de 30% da velocidade por contato nosso antes de
x = 3000; o alvo publicado está **à frente** da bola em ≥ 80% dos ciclos com
`v_bola > 250`.

**Falha:** bola desacelerando com um robô nosso colado ⇒ **tactic**; é o teste de
regressão do tratamento `bola_ja_saiu`, que era código inalcançável.

### P-4 · Portador/Apoio · passe com o gol fechado

**Pergunta:** com o gol bloqueado e o apoio adiantado e livre, sai passe com força
de passe, para o robô certo?

```python
"jogo_passe": {
    "titulo": "Jogo corrido: gol fechado, apoio livre a frente",
    "bola": (500.0, -800.0),
    "azuis": [(0, -4300, 0, 0), (1, 100, -900, 0), (2, 2400, 800, 0)],
    "amarelos": [(0, 4300, 0, 180), (1, 1600, -500, 180), (2, 2000, -200, 180)],
    "comando": ("FORCE_START", "BLUE"),
},
```

**Sucesso:** o receptor é o **r2** (o adiantado), não o robô que está na bola;
`v_saida_mms` entre 1800 e 3200; a bola chega a menos de 300 mm do r2 em ≥ 4 de 6.

**Falha:** receptor igual ao robô na bola ⇒ regressão da correção de 02/10. Passe a
5000+ ⇒ **skill** (força não seguiu o tipo de alvo).

### C-1 · Cobertura · o bloqueio sobrevive à troca de lado

```python
"cobertura_troca_lado": {
    "titulo": "Adversario cruza o campo com a bola; cobertura acompanha",
    "bola": (-500.0, 2000.0),
    "azuis": [(0, -4300, 0, 0), (1, -1500, 1500, 0), (2, -2000, 0, 0), (3, -2500, -1000, 0)],
    "amarelos": [(0, 4300, 0, 180), (1, -300, 2000, 180), (2, 500, 0, 180), (3, 800, -1500, 180)],
    "comando": ("FORCE_START", "BLUE"),
},
```

**Sucesso:** a cobertura fica a menos de 250 mm da reta bola→própria meta em ≥ 70%
dos quadros com a bola além de 2500 mm da meta; nenhum gol sofrido em ≥ 4 de 6.

**Falha:** cobertura ao lado do corredor ⇒ **tactic** (regressão do deslocamento
perpendicular). Papel trocando a cada travessia ⇒ **play** (histerese).

### C-2 · Cobertura · defender o chute de longe

```python
"chute_de_longe": {
    "titulo": "Chute deles do meio-campo; so o goleiro tocava na bola",
    "bola": (0.0, 0.0),
    "azuis": [(0, -4300, 0, 0), (1, -1200, 800, 0), (2, -2200, -600, 0), (3, -3000, 300, 0)],
    "amarelos": [(0, 4300, 0, 180), (1, 300, 0, 180), (2, 1500, 1000, 180), (3, 1500, -1000, 180)],
    "comando": ("FORCE_START", "BLUE"),
},
```

**Sucesso:** em ≥ 3 de 6, um robô de linha toca na bola antes de x = −3500; gols
sofridos ≤ 2 em 6.

**Falha:** bola atravessando o time e só o goleiro tocando ⇒ **tactic**, e indica
a necessidade da skill de interceptação por tempo de chegada (nenhum mecanismo
atual usa a velocidade da bola).

### C-3 · Goleiro · bola na área, entre 600 e 1000 mm do centro

```python
"goleiro_area_lateral": {
    "titulo": "Bola na area, |y| entre 600 e 1000 (o falso negativo de 16,2%)",
    "bola": (-4100.0, 800.0),
    "azuis": [(0, -4300, 0, 0), (1, -2500, 1200, 0)],
    "amarelos": [(0, 4300, 0, 180), (1, -3400, 900, 180)],
    "comando": ("FORCE_START", "BLUE"),
},
```

**Sucesso:** o goleiro sai da linha e toca na bola em ≤ 3 s em ≥ 5 de 6; a bola
termina com x > −3000 em ≥ 4 de 6.

**Falha:** goleiro parado na linha ⇒ **tactic**; é regressão direta da correção da
área.

### C-4 · Goleiro · a saída de bola escolhe um alvo que existe

```python
"goleiro_sai_jogando": {
    "titulo": "Goleiro com a bola na area; UM companheiro livre e UM marcado",
    "bola": (-4000.0, 200.0),
    "azuis": [(0, -4300, 0, 0), (1, -2000, 1800, 0), (2, -2000, -1800, 0)],
    "amarelos": [(0, 4300, 0, 180), (1, -3000, -900, 180), (2, -2600, -1500, 180)],
    "comando": ("FORCE_START", "BLUE"),
},
```

**Sucesso:** a bola vai na direção do **r1** (o livre) em ≥ 4 de 6 e chega a menos
de 400 mm dele em ≥ 3 de 6.

**Falha:** escolha do lado marcado, ou alternância sem critério ⇒ **skill**: a
checagem de linha foi corrigida em `ef74a5e` (antes recebia um versor no lugar
das coordenadas de destino) e o efeito nunca foi medido. Este cenário é o teste
de regressão dessa correção.

### A-1 · Apoio · oferecer-se sem entrar no corredor, e marcar quando deve

```python
"apoio_corredor": {
    "titulo": "Bola nossa no meio; o apoio deve abrir, nao entrar no corredor",
    "bola": (800.0, 0.0),
    "azuis": [(0, -4300, 0, 0), (1, 600, 0, 0), (2, 1400, 200, 0)],
    "amarelos": [(0, 4300, 0, 180), (1, 3000, 0, 180)],
    "comando": ("FORCE_START", "BLUE"),
},
```

**Sucesso:** o apoio a mais de 900 mm da reta bola→alvo em ≥ 80% dos ciclos;
`v_saida_mms ≥ 4500` quando o chute sai; amontoamento (`jogo-analise`, coluna
`amontoa`) ≤ 10%. **Critério adicional desde 02/10:** o apoio deve **marcar**
quando a bola entra na zona de perigo e **não** marcar em disputa no campo
adversário.

**Falha:** apoio dentro do corredor ⇒ **tactic** (constantes com histórico de duas
reversões). Apoio indo à bola ⇒ **play** (atribuição de papéis).

### A-2 · Apoio · giro com o robô parado no lugar

```python
"apoio_parado_gira": {
    "titulo": "Apoio chega ao ponto e fica; a bola se move ao lado dele",
    "bola": (0.0, -1500.0),
    "azuis": [(0, -4300, 0, 0), (1, -300, -1500, 0), (2, 1800, 1600, 90)],
    "amarelos": [(0, 4300, 0, 180)],
    "comando": ("FORCE_START", "BLUE"),
},
```

**Sucesso:** erro de orientação do r2 para a bola < 20° em ≥ 80% dos quadros após
3 s.

**Falha esperada, e é o resultado útil:** r2 congelado na orientação inicial ⇒ não
é da estratégia, é `control.py:120,139`. Este cenário produz o número que falta
para levar o defeito à equipe de movimentação.

### X-1 · Plays · falta indireta a favor não congela o time

```python
"indireta_favor": {
    "titulo": "Falta INDIRETA a nosso favor: existe play?",
    "bola": (2500.0, 0.0),
    "azuis": [(0, -4300, 0, 0), (1, 1900, 150, 0), (2, 1700, 1400, 0)],
    "amarelos": [(0, 4300, 0, 180), (1, 3050, 0, 180)],
    "comando": ("INDIRECT", "BLUE"),      # exige fluxo novo (seção 9)
},
```

**Sucesso:** deslocamento somado dos nossos > 500 mm em 25 s, e a bola sai do
lugar.

**Falha:** `[BT] RootStrategy parou em RUNNING`, ou nenhum comando publicado ⇒
**play ausente** (item A6). O teste mede o custo: segundos de jogo parado por
partida.

---

## 8. Prognóstico

**Falha com causa isolada:**

- **A-2** — defeito conhecido em `control.py`; a falha é o resultado desejado.
- **X-1** — ausência de play para `INDIRECT_FREE_*`.

**Provável falha:**

- **C-2** — nenhum mecanismo de cobertura usa a velocidade da bola.
- **P-1** — o chute deve sair (medido: 3 de 3 em campo limpo, 5555-5944 mm/s), mas
  o gol depende de a mira cair em |y| ≤ 500 a 4,5 m, com orientação atrasada pela
  visão (4,7° ≈ 37 cm de desvio nessa distância).

**Deve passar:**

- **C-4** — a checagem de linha está correta desde `ef74a5e`; o cenário confirma.
- **P-4** — teste de regressão da correção do receptor do passe.
- **A-1** — constantes calibradas por duas reversões medidas.
- **C-3** — regressão de correção feita com 8472 quadros de comparação.

**Incerto, e por isso informativo:** **P-2** (separa a correção de 02/10 do item
B1), **P-3** e **C-1** (exercitam correções recentes, nunca medidas em
isolamento).

**Risco novo a vigiar:** a marcação do apoio pode reduzir os quadros com robô
nosso além de x = 3500 — métrica 4 do `jogo-analise`. Queda em relação à linha de
base indica gatilho de marcação largo demais.

**Ordem recomendada:** P-2 e A-2 primeiro. Cada um fecha uma pergunta aberta do
`ESTADO_ATUAL.md` sem depender do restante do time.

---

## 9. Utilitários ausentes

1. **Comandos de árbitro além dos três fluxos.** `FLUXOS_ARBITRAGEM`
   (`docs/ararabots.py:66`) só conhece `freekick` e `kickoff`. X-1 exige
   `INDIRECT_FREE_KICK_*`; o mesmo vale para `BALL_PLACEMENT_*` e
   `PREPARE_PENALTY_*`.
2. **Velocidade inicial da bola no `posicionar`.** O replacement do grSim aceita
   velocidade; os cenários só declaram posição. P-3 exige um campo `"bola_vel"`.
3. **Distância do robô à reta bola→própria meta, por quadro.** C-1 e C-2 dependem
   dela; o `jogo-analise` não a tem. Os replays HTML já contêm as posições.
4. **Erro de orientação por robô ao longo do tempo.** A-2 depende disso.
5. **Papel do robô no replay.** Hoje só aparece em `/tmp/strategy.log`, que guarda
   apenas a última execução do lote.
6. **Sonda de decisão do jogo corrido** (`./ararabots.sh decisao-jogo`), espelhando
   o `decisao` que já existe para a bola parada. Foi uma sonda offline desse tipo
   que encontrou a inversão de papéis descrita em 1.1; sem ela, o item B1 exige um
   lote inteiro para cada tentativa.
