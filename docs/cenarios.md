# Os cenários de teste — o que cada um mede, e por que existe

Documento vivo da coordenação de estratégia. **Cenário novo entra aqui junto com
o código.** Um cenário sem explicação vira, meses depois, um nome numa lista que
ninguém sabe se ainda mede alguma coisa — foi o que aconteceu com os dois que
saíram em 03/10/2026.

Os cenários vivem no dict `CENARIOS`, em `docs/ararabots.py`. Cada um tem
`tipo`, e a interface pede **primeiro o tipo, depois o cenário**.

```bash
./ararabots.sh validar 6 <cenario>        # lote de 6
./ararabots.sh cenario  <cenario>         # monta um só, sem gravar
```

No painel (aba `Tool` da GUI): bloco **2 · escolher e rodar** → tipo → cenário.

---

## Os dez tipos

| tipo | para que serve | quantos |
|---|---|---|
| `jogo` | jogo corrido — o que a competição é | 4 |
| `bola_parada` | cobrança de falta: a parte mais madura da base (5 gols em 6) | 9 |
| `kickoff` | início de partida, a favor e contra | 2 |
| `orientacao` | teste da correção de 03/10: o corpo não vira as costas para a bola | 3 |
| `orbita` | teste da correção de 03/10: bola atrás de nós, contorna e pega por trás | 3 |
| `pressao` | teste da correção de 03/10: pressão medida na bola, não no corpo | 3 |
| `protecao` | teste da correção de 03/10: saída sob pressão fugindo de quem prensa | 3 |
| `mira` | teste da correção de 07/10: a mira não gira a cada ciclo | 3 |
| `empurrao` | teste da correção de 07/10: no contato, o alvo fica além da bola | 3 |
| `robustez` | contagem de robôs: casos limite que já quebraram o nó | 4 |

---

## `jogo` — jogo corrido

| cenário | o que mede | por que existe |
|---|---|---|
| `jogo` | partida livre, quatro contra quatro, bola ao centro | É o cenário que decide se o time joga. Não tem critério de gol embutido: o que vale é o replay e o `jogo-analise` (alguém vai à bola, amontoam, a bola avança, o time ataca, alguém chuta). Quatro robôs por time, e não três, porque com dois de linha não há papéis — um é o portador e o outro faz todo o resto |
| `regressao_chute_livre` | chute com o gol livre, campo limpo | Resultado conhecido: **3 de 3**, 5555–5944 mm/s. Serve para detectar quebra na cadeia do chute sem o ruído da disputa |
| `regressao_goleiro_central` | goleiro centralizado na meta, portador e apoio disponíveis | O chute reto está bloqueado. Inspecione o replay para ver se a estratégia mira um canto livre ou escolhe o passe; não se exige gol, pois o goleiro pode defender |
| `regressao_passe` | gol fechado, apoio adiantado e livre | Tem de sair **passe** (~2,5 m/s), não chute de 6. Três robôs de linha de propósito: com dois, a prioridade de papéis (portador → cobertura → apoio) deixa o time **sem apoio**, e sem apoio não existe receptor — o time nunca passa |

---

## `bola_parada` — cobrança de falta

A fase mais madura: aproximação angulada em três fases, mira por varredura da
boca do gol, travas de mira e de armamento, guarda de duplo toque. Encerrada com
**gol em 5 de 6**.

| cenário | o que mede | por que existe |
|---|---|---|
| `ataque` | cobrança no terço de ataque | O caso canônico: cobrador posiciona atrás da bola, empurra rumo ao gol e **ativa** o chute |
| `passe` | falta longe do gol, companheiro adiantado e com linha limpa | Tem de sair passe, não condução solitária. Complementa o `ataque`: nem toda falta é chute |
| `defesa` | cobrança no nosso campo | O chute tem de ficar **desativado** e o goleiro não pode deixar a meta. Mede o risco de gol contra na saída |
| `lateral` | bola junto à linha lateral | A geometria de aproximação muda: metade dos ângulos de chegada sai do campo |
| `canto` | bola no canto do campo de ataque | O ângulo para o gol é agudo; a mira por varredura tem de achar o vão ou desistir |
| `deles_meio` | cobrança **deles** no meio-campo | Mede o nosso posicionamento defensivo em bola parada contra. A barreira não existe — este cenário é onde isso aparece |
| `deles_perto_gol` | cobrança **deles** rente à nossa área | Perigo máximo: bloqueio concentrado entre a bola e o gol, goleiro na linha |
| `um_so_cobrador` | só o robô 1 em campo | O caso mínimo em que a cobrança ainda deve acontecer. Se falhar aqui, o problema é da execução da falta, não do time. É a referência dos **5 gols em 6** |
| `cobrador_na_frente` | cobrador do **lado errado** da bola | Exige contornar. Foi o buraco de cobertura descoberto na rev. 19: os outros 14 cenários punham o cobrador sempre atrás da bola |

---

## `kickoff`

| cenário | o que mede | por que existe |
|---|---|---|
| `kickoff_favor` | nosso kickoff | Fluxo STOP → PREPARE_KICKOFF → NORMAL_START, com a formação de ataque |
| `kickoff_contra` | kickoff deles | Posicionamento legal (fora do círculo, no nosso campo) e a transição para o jogo |

---

## `orientacao` — o corpo não vira as costas para a bola *(03/10/2026)*

**O defeito.** Dentro de 700 mm da bola o ângulo do corpo era `atan2(alvo − bola)`
— calculado no referencial da **bola**, idêntico para qualquer posição do robô.
Metade do círculo em volta dela fica do lado errado dessa direção, e ali a ordem é
ficar de costas. Medido por sonda offline: **18 de 36 posições** em todos os raios
abaixo de 700 mm, erro de até **180°**. Com a correção: **0 de 576**.

Os três põem o robô a **632–650 mm** da bola: dentro do raio de orientação (700) e
**fora** do raio da órbita (600), onde a órbita provadamente não dispara. É o que
isola esta modificação da seguinte.

| cenário | setup | o que o lote deve mostrar |
|---|---|---|
| `orientacao_meio` | bola no centro, robô entre ela e o gol deles | com a chave: corpo a 0° (costas); sem: 180° (olhando a bola) |
| `orientacao_lateral` | bola fora do eixo, em (800, 1900) | o alvo do chute muda de direção e a ordem antiga gira o corpo junto, sem olhar onde o robô está |
| `orientacao_terco` | bola no nosso terço | o caso perigoso: de costas para a bola na frente da nossa área, com a saída de bola escolhendo a direção |

---

## `orbita` — bola atrás de nós, contorna e pega por trás *(03/10/2026)*

**O defeito.** Perto da bola o alvo era o ponto na linha de tiro, e o desvio
lateral do contorno zerava de propósito (para o robô se comprometer com a janela
de disparo). Do lado errado, esse alvo só é alcançável **atravessando a bola** — e
sem dribbler a bola sai na direção robô→bola, isto é, para trás. Medido: **22 de
72 largadas** terminavam empurrando a bola de volta, 175° no pior caso. Com a
correção: **0 de 72**.

| cenário | setup | o que o lote deve mostrar |
|---|---|---|
| `orbita_frontal` | robô a 260 mm, exatamente entre a bola e o gol | o pior caso. Com a chave a bola vai para trás; sem ela o robô dá a volta |
| `orbita_diagonal` | lado errado por 135°, a 400 mm | o contorno tem de escolher o **sentido mais curto** do arco, não só inverter |
| `orbita_colado` | lado errado a 150 mm, quase em contato | o risco é empurrar enquanto decide; o arco tem raio 260, então o primeiro alvo **afasta** antes de contornar |

Estes três mudaram o código: a 260 e 400 mm a situação é `SOLTA` (o raio de posse
é 250), e o ramo `SOLTA` devolvia o ponto previsto **direto**, sem passar pelo
maquinário de aproximação — ou seja, **com a bola solta o portador ia reto nela
pelo lado errado**. A bola solta é justamente quando mais se ganha contornando,
porque ninguém disputa.

---

## `pressao` — a pressão se mede na bola *(03/10/2026)*

**O defeito.** A contagem de adversários "perto" usava a posição do **robô**. Com
o portador atrás da bola — a posição certa para empurrar — quem prensa a **bola**
fica a mais de 400 mm do robô e a conta dá **zero**: numa prensa frontal com dois
adversários a 304 e 348 mm da bola, o alívio nunca disparava e o portador
posicionava para sempre.

Nos três, os amarelos ficam a menos de 400 mm da **bola** e a mais de 400 mm do
**nosso robô**. A assimetria é o teste.

| cenário | setup | o que o lote deve mostrar |
|---|---|---|
| `pressao_na_bola` | dois prensando a bola, longe do nosso corpo | com a chave: `alvo_chute=bloqueado` quase sempre; sem ela: `alivio` |
| `pressao_na_bola_lateral` | mesma assimetria junto à linha | com a bola na lateral a saída tem menos opções, o que torna o disparo mais decisivo |
| `pressao_na_bola_terco` | prensa no nosso terço | **controle negativo**: ali a saída de bola tem precedência sobre o alívio, então o esperado é **não mudar nada**. Se mudar, a precedência está errada |

---

## `protecao` — proteção de posse sob pressão *(03/10/2026)*

**O que mudou.** O alívio era fixo ("joga na lateral", sempre no mesmo y limite,
sem olhar onde está quem prensa). Agora a direção sai de uma **varredura angular**
pontuada pela folga, com viés de ataque, o cone da própria meta proibido **e as
direções que empurram a bola para o lado de quem prensa descartadas**.

É essa última restrição que põe o corpo no meio — descoberto medindo, depois de
errar o raciocínio: sem ela o escudo media 74° a 163° (metade das vezes o robô ia
para o lado **oposto** e não protegia nada). Com ela: **66° de mediana, mínimo
39°**.

| cenário | setup | o que o lote deve mostrar |
|---|---|---|
| `protecao_frontal` | adversário a 240 mm do portador e da bola | o alívio dispara nas duas rodadas; o que muda é a direção e, com ela, a posição do corpo |
| `protecao_dois_lados` | dois adversários em flancos opostos | a varredura tem de achar a única direção com folga; a lateral fixa pode jogar em cima de um deles |
| `protecao_contra_tres` | três em cima da bola | o caso medido em jogo ("eles chegam com três e nós com um"). Nenhuma direção está limpa: mede se escolher a **menos pior** vale a pena |

---

## `mira` — a mira não gira a cada ciclo *(07/10/2026)*

**O defeito.** Toda direção do ciclo sai de `alvo_do_chute`: a orientação do
corpo, o lado por onde se chega, o ponto de atravessar, o alívio sob pressão.
Existia uma trava de 2 s para isso — e ela excluía justamente a transição que
acontecia:

```python
and tipo_alvo not in (None, "bloqueado")
and _ant_t not in (None, "bloqueado")
```

Qualquer troca que passasse por `bloqueado` passava livre. E é por `bloqueado`
que ela passa: com o gol fechado e o apoio entrando e saindo da linha de passe, o
tipo alterna `passe → bloqueado → passe`. Medido com a sonda de decisão guiada
pelos replays do lote de 03/10 (a tática roda sobre os quadros gravados, sem ROS
e sem simulador):

| cenário medido | ciclos `passe` | ciclos `bloqueado` | trocas de mira |
|---|---:|---:|---:|
| `orientacao_terco` | 132 | 148 | **65 em 245 ciclos** |
| `protecao_dois_lados` | 67 | 202 | 6 |
| `protecao_frontal` | 35 | 242 | 2 |

Com a mira girando, o alinhamento do portador nunca fecha: mediana **t = 0,11**
(≈160° de erro), alvo da aproximação **atrás** da bola em 54% dos ciclos, e o
portador orbitando a **314–652 mm** dela por 24,5 s sem nunca encostar. O
deslocamento líquido da bola nos três cenários de proteção foi de **0 a 8 mm**.

**É por isso que nenhuma das quatro correções de 03/10 mostrou ganho de resultado
nesses cenários:** as três camadas decidiam sobre uma direção que trocava sozinha.

Com a correção, `orientacao_terco` cai de 65 trocas para **8**. A mira segurada
vale para o corpo e para a aproximação, **mas não para o gatilho** — `armar_chute`
não confere linha, e o defeito medido era chutar em quem estava no caminho (14 de
19 instantes de chute). Ver `chute.chutar_em(pode_armar=...)`.

| cenário | setup | o que o lote deve mostrar |
|---|---|---|
| `mira_gol_fechado` | gol tapado; apoio à frente e um amarelo rente à reta bola→apoio | com a chave: dezenas de trocas de mira e o portador nunca encosta; sem ela: mira estável e contato |
| `mira_apoio_que_entra` | apoio a 620 mm à frente, rente ao limite de `AVANCO_MINIMO_PASSE` (600) | a fronteira que mais pisca: qualquer recuo dele invalida o passe |
| `mira_dois_apoios` | dois aliados elegíveis à frente, gol tapado | o papel de apoio tem histerese; o **alvo do passe** não tinha |

---

## `empurrao` — no contato, o alvo fica além da bola *(07/10/2026)*

**O defeito.** O cabeçalho de `skills/aproximacao.py` diz, desde que foi escrito,
que "o alvo próximo é **além** da bola, não o ponto de chute" — e a constante
dizia o contrário: `ATRAVESSA_CHUTE = -53`, isto é, 53 mm **aquém** dela. Medido
com a sonda guiada por replay, em todos os quadros de contato:

| execução | offset mediano do alvo | além da bola |
|---|---:|---:|
| `orbita_frontal` antes | −58,3 mm | 0% |
| `orbita_frontal` depois | −56,1 mm | 3% |
| `orbita_colado` depois | −53,0 mm | 9% |
| `pressao_na_bola` | −54,9 mm | 0% |

O casco tem 90 mm e a bola 21: o **centro** do robô não chega a menos de 111 mm
do centro da bola. Um alvo a −53 mm é inalcançável por construção, e o que sobra
é erro residual de ~58 mm. Com `kp = 2,3` isso dá **0,13 m/s** — e o feedforward
está desligado pelo ajuste do PID, que é a condição de medida desta fase. O robô
encosta na bola e a bola não sai do lugar.

Com a correção o alvo desliza até **+180 mm além** da bola conforme o alinhamento
(`EMPURRAO`, acima de `TOL_EMPURRAO = 0,80`, ou ~36° de erro), o que dá 291 mm de
erro e ~0,67 m/s de comando. Sonda fechada (a decisão guiando um modelo da cadeia
de movimento, sem simulador): avanço da bola de 2885 → 3747, 2623 → 4397 e
4515 → 5579 mm.

| cenário | setup | o que o lote deve mostrar |
|---|---|---|
| `empurrao_reto` | portador 400 mm atrás da bola, alinhado, gol livre | a geometria mais simples que existe: se a bola não anda aqui, não anda em lugar nenhum |
| `empurrao_terco` | bola no nosso terço, gol fechado | a direção vem do afastamento; mede quantos mm a bola avança no eixo de ataque |
| `empurrao_apos_contorno` | lado errado a 300 mm | órbita e **depois** empurrão: o que o lote de 03/10 não conseguiu separar |

---

## `robustez` — contagem de robôs

A lógica trata o robô 0 como goleiro e distribui papéis entre os demais. Estes
cenários são os casos limite que já quebraram o nó ou escondiam comportamento.

| cenário | o que mede |
|---|---|
| `dois_goleiro_e_cobrador` | goleiro + um de linha: há cobrança e a meta fica guardada? |
| `dois_sem_goleiro` | dois de linha, nenhum goleiro: ninguém pode assumir a meta por engano |
| `um_so_goleiro` | só o robô 0: ele **não** pode assumir a cobrança |
| `campo_vazio` | nenhum robô nosso: a árvore não pode quebrar nem comandar robô inexistente |

---

## As chaves de experimento

As correções de 03/10 e de 07/10 têm chave para desligar, de modo que o **mesmo
binário** rode o antes e o depois. O default é o comportamento **novo**.

| chave | desliga |
|---|---|
| `ARARABOTS_SEM_ORIENTACAO_LADO` | o corpo não vira as costas para a bola |
| `ARARABOTS_SEM_ORBITA` | contorna e pega por trás |
| `ARARABOTS_SEM_PRESSAO_BOLA` | pressão medida na bola |
| `ARARABOTS_SEM_PROTECAO` | saída por varredura (volta à lateral fixa) |
| `ARARABOTS_SEM_MIRA_FIRME` | a trava da mira volta a excluir `bloqueado` |
| `ARARABOTS_SEM_EMPURRAO` | o alvo de contato volta a 53 mm aquém da bola |

```bash
ARARABOTS_INIMIGO_PARADO=1 ARARABOTS_SEM_ORBITA=1 ./ararabots.sh validar 3 orbita_frontal   # antes
ARARABOTS_INIMIGO_PARADO=1                        ./ararabots.sh validar 3 orbita_frontal   # depois
```

### A campanha inteira, num comando

Não é preciso montar os pares à mão. O subcomando `lotes` roda as cinco
funcionalidades em aberto — três cenários cada, antes e depois —, imprime cada
par **assim que ele termina** e escreve `docs/lotes-<data>.md`:

```bash
./ararabots.sh lotes
```

```bash
./ararabots.sh lotes MIRA_FIRME EMPURRAO
```

```bash
N=3 ./ararabots.sh lotes
```

Ele resolve sozinho as três armadilhas que custaram lotes antes: a trava órfã do
`validar` (que faz o lote seguinte sair vazio em silêncio), a chave de
experimento vazando de uma condição para a outra, e o árbitro emudecendo no meio
(remonta e segue). A métrica da tabela é o **avanço assinado** no eixo de ataque.

No painel elas aparecem no bloco **3 · bandeiras**, marcadas com ⏻.

**Adversário parado nos testes das modificações.** Os cenários posicionam os
amarelos numa geometria exata; o perfil que a ferramenta usa os faz se mover, e a
geometria dura poucos décimos de segundo. Com lote pequeno isso vira ruído.

**Protocolo:** ao testar uma modificação, as outras ficam no estado **novo** nas
duas rodadas, para o delta medir só a que está sendo testada.

**A métrica de resultado é o AVANÇO ASSINADO no eixo de ataque**, não o módulo do
deslocamento. Foi o módulo que escondeu o resultado do lote de 03/10: a órbita
aparecia como "pior" (139 mm contra 1302) quando o que ela havia feito era deixar
de empurrar a bola **1087 mm para o nosso campo**. O `rodar` agora imprime
`AVANCO (eixo de ataque)` e o CSV traz `bola_ini_x`/`bola_fim_x`.

**Confira os quadros descartados.** O grupo multicast da visão não é nosso:
qualquer outro grSim ou ssl-vision na máquina manda pacotes para lá com o relógio
dele. Isso invalidou as métricas de 17 dos 30 replays do lote de 03/10 (um bloco
de quadros com `t_capture` 12 h adiantado, seis amarelos e a bola fora do campo).
O `rodar` agora rejeita esses quadros e **diz quantos** — número grande significa
um segundo simulador de pé.

---

## Ao acrescentar um cenário

1. entre no `CENARIOS` de `docs/ararabots.py` com **`tipo`** preenchido;
2. escreva a `descricao` dizendo **o que o cenário prova**, não só onde os robôs
   estão — ela aparece no painel quando alguém seleciona o cenário;
3. acrescente a linha aqui, na tabela do tipo, com a coluna "por que existe";
4. confira a geometria antes de rodar lote: robôs sobrepostos, bola dentro de
   robô ou robô fora do campo fazem o grSim aceitar e o teste nascer inválido;
5. confira que o cenário **discrimina**: se a decisão não muda entre as duas
   condições que você quer comparar, o lote não vai mostrar nada.

---

## Removidos em 03/10/2026

| cenário | por que saiu |
|---|---|
| `limiar` | testava o "limiar de chute" (`kick_threshold`), que virou código morto: a decisão de chutar hoje é de `alvo_do_chute` + `linha_livre` |
| `meio` | mesmo motivo — o critério dele era "o chute deve ficar **desativado** aquém do limiar", e esse limiar não existe mais |
| `regressao_cobranca` | era cópia exata de `um_so_cobrador` (mesma bola, mesmos robôs). A regressão da bola parada usa o original |

---

## O que ainda não existe

- **`penalty`** — `PREPARE_PENALTY_*` não tem play nenhuma: o time fica parado.
  Um cenário aqui mediria o vazio. Primeiro a play, depois o cenário.
- **`INDIRECT_FREE_*`** — mesma situação, e é falta a nosso favor.
- **`BALL_PLACEMENT_*`** — idem, e é exigência de regra para a Division B.
