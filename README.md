# agri_gazebo

A plataforma de simulação agrícola: mundos de fazenda de citros, a metade Gazebo de cada veículo
(plugins, sensores, bridge) e os launches que põem qualquer um deles em qualquer mundo.
Compartilhada por todo projeto do workspace; nada aqui sabe de um projeto em particular.

```
agri_gz_worlds/               mundos comuns a todos os veículos
├── worlds/
│   ├── empty.sdf             chão, sol, os sistemas: o teste de fumaça
│   └── citrus_farm{,_90,_60,_30}.sdf      uma propriedade, uma rede de estradas cada
├── models/                   os adereços que esses campos <include>em
├── scripts/                  os geradores: os mundos e as malhas dos citros
└── docs/citrus/              as fotografias a partir das quais as fazendas foram feitas
agri_gz_bringup/              os launches que todo veículo compartilha (um veículo, um comboio),
│                             o módulo Python vehicles por trás deles, os macros gz de sensores
autonomous_tractor_gz/        o trator no Gazebo: tags gz, plugins, bridge, altura de spawn, launch
pickup_gz/                    o mesmo, para a pickup
```

Dirigir um veículo não é daqui: a teleoperação pelo teclado está no repositório
`vehicle_control`, e o seguidor de estrada por câmera está no repositório `ftl`, que o usa para
dirigir o líder.

O `agri_gz_bringup` é o que impede um segundo veículo de ser uma cópia do primeiro. O seu
`sim.launch.py` sobe qualquer um deles — ele acha `urdf/<veiculo>.gz.xacro` e `config/bridge.yaml`
dentro de `<veiculo>_gz` por convenção — e o seu `urdf/sensors.gz.xacro` guarda a metade Gazebo de
todo sensor: taxa, ruído, campo de visão. A metade da descrição (onde o sensor fica) mora no
repositório `robot_description`, e as duas se juntam pelo nome do link.

O alvo é o **Gazebo Harmonic**, a versão pareada com o ROS 2 Jazzy. Os comandos são `gz sim` (o
antigo `ign gazebo` é do Fortress, e `gzserver`/`gzclient` do Gazebo Classic, que chegou ao fim da
vida em janeiro de 2025).

## Rodando

```bash
ros2 launch autonomous_tractor_gz sim.launch.py                # mundo + trator + bridge, GUI
ros2 launch autonomous_tractor_gz sim.launch.py gui:=false     # headless, para CI
ros2 launch autonomous_tractor_gz sim.launch.py world:=empty.sdf
ros2 launch autonomous_tractor_gz sim.launch.py world:=citrus_farm.sdf

# O mesmo, pelo launch genérico, mais onde pôr a máquina:
ros2 launch agri_gz_bringup sim.launch.py robot:=autonomous_tractor \
    world:=citrus_farm_60.sdf x:=-40 y:=6 yaw:=1.5708
```

`world:` é um arquivo em `agri_gz_worlds/worlds/`. Um `gz sim citrus_farm_90.sdf` puro também
funciona, de qualquer diretório: o hook de ambiente põe aquele diretório no
`GZ_SIM_RESOURCE_PATH`.

Dirija de um segundo shell. Entre no container *que está rodando* em vez de abrir outro, para os
dois shells compartilharem o mesmo grafo ROS:

```bash
docker exec -it ros2-ws-dev bash
```

Depois dirija com o nó de teclado do repositório `vehicle_control` (veja no README de lá por que
ele esterça por curvatura, e não por taxa de yaw):

```bash
ros2 run vehicle_control teleop_key      # teclado: o acelerador trava, o volante volta ao centro
```

Ou comande direto, que é o que um script faria:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0}, angular: {z: 0.3}}" -r 20
ros2 topic echo /odom
ros2 control list_controllers           # os três devem estar active alguns segundos após o spawn
```

`angular.z` é uma **taxa de yaw**, não um ângulo de direção: o controlador de direção calcula o que
as rodas dianteiras precisam fazer para produzi-la, e as duas acabam em ângulos diferentes — é isso
que Ackermann quer dizer. Peça mais do que a geometria dá e ele satura no limite da direção em vez
de recusar.

### Como os veículos são dirigidos

Pelo **ros2_control**: o bloco `<ros2_control>` de cada veículo (do
`vehicle_common_description`) recebe o plugin de hardware do Gazebo, o `gz_ros2_control` roda um
controller manager para ele dentro do simulador, no namespace do veículo, e o `vehicles.py` cria o
`joint_state_broadcaster`, o `ackermann_steering_controller` (direção e rodas traseiras) e o
`front_drive_controller` (rodas dianteiras: tração nas quatro) e inicia o `cmd_vel_limiter` na
frente deles. Os controladores, seus parâmetros e os limites são do `vehicle_control`, os mesmos do
mock e da máquina; nada de condução é configurado aqui. Os tópicos não mudaram: `cmd_vel` entra,
`odom`, `tf` e `joint_states` saem.

Os controladores ficam ativos alguns segundos depois que um veículo é criado; até lá ele não anda,
e um `cmd_vel` enviado antes é simplesmente descartado pelo limiter depois de 0,5 s. Num comboio
cada veículo tem o seu manager, `/<namespace>/controller_manager`, e cada um trava o simulador por
um momento enquanto carrega, então a partida de cinco veículos leva boa parte de um minuto. Rode o
Gazebo com GPU: sem ela o `citrus_farm` roda a cerca de um décimo do tempo real.

O plugin Ackermann do Gazebo, que dirigia os veículos antes, saiu em 2026-11-12 (com a seção de
tração do `bridge.yaml`), depois de uma volta completa do FTL nos dois caminhos; a comparação está
no README do `vehicle_control`. O `bridge.yaml` agora só leva o relógio e os sensores.

### Um comboio

```bash
ros2 launch agri_gz_bringup convoy.launch.py                      # pickup + 4 tratores
ros2 launch agri_gz_bringup convoy.launch.py spacing:=15.0 followers:=2
ros2 launch agri_gz_bringup convoy.launch.py leader:=autonomous_tractor follower:=pickup
```

Por padrão, uma **pickup** na frente e **quatro tratores autônomos** atrás, em fila pelo carreador
principal a dez metros, o líder na origem do mundo virado para leste. `leader:=` e `follower:=`
aceitam qualquer tipo de veículo que tenha um pacote `<veiculo>_gz`; os seguidores se chamam
`<follower_prefix>_<n>` (`tractor_1`… por padrão). O comboio só alinha as máquinas: o que elas
fazem depois é do projeto que o lançou. Dez metros entre `base_link`s são 6,4 m de espaço livre
entre um trator de 3,6 m e o seguinte.

Duas coisas fazem cinco veículos num mundo funcionarem, e as duas valem ser conhecidas antes de
criar um sexto:

- **Cada máquina é o seu próprio namespace ROS.** O controller manager, os controladores e os
  tópicos dos sensores dela vivem nele; sem isso os cinco chegariam em `/cmd_vel` e `/odom` e
  brigariam. Dirigir o terceiro é
  `ros2 run vehicle_control teleop_key --ros-args -r __ns:=/tractor_3`. Cada um também mantém a
  sua **árvore de TF no seu namespace** (`/tractor_3/tf`, `/tractor_3/tf_static`): todo veículo
  usa os mesmos nomes de frame, e num `/tf` global um sobrescreveria o outro. Um nó que lê o TF de
  um veículo remapeia `/tf` e `/tf_static` para os nomes relativos, como o `ftl_bringup` faz.
- **Cada veículo leva a própria tag, e só os seguidores levam câmera.** O líder tem a tag 0 e o
  seguidor `k` tem a tag `k`. Todos têm GNSS, IMU e a pose real; os seguidores têm câmera (a
  câmera é a coisa mais cara de qualquer um destes mundos: 16,8 dos 62,1 segundos de parede para
  simular 20 s do `citrus_farm_60`), e a metade câmera da configuração da bridge do líder cai
  junto com a câmera, então nada assina um tópico que nunca vai ter mensagem.

Argumentos do comboio além dos de posição:

| argumento        | padrão    | o que faz                                                       |
|------------------|-----------|-----------------------------------------------------------------|
| `tag_family`     | `36h11`   | família AprilTag de todos os veículos                           |
| `tag_size`       | `0.5`     | lado do quadrado preto da tag, em metros                        |
| `camera_mount`   | `single`  | `single` (uma câmera) ou `dual` (duas câmeras de 90 graus)      |
| `sensor_profile` | `default` | perfil de GNSS e IMU: `default`, `nominal` ou `ideal`           |

Cada câmera sobe com um nó `camera_distortion` no namespace do veículo, que publica
`<veiculo>/camera_front/image_raw` (ou as das duas câmeras na montagem `dual`).

A **verdade de solo** de cada veículo, líder inclusive, chega em
`/ground_truth/<nome>/pose` (`geometry_msgs/msg/PoseStamped`), por exemplo
`/ground_truth/pickup/pose`.

**Sem tela:** com `gui:=false` e sem `DISPLAY`, o `gz sim` recebe `--headless-rendering`. Se
`/usr/share/glvnd/egl_vendor.d/10_nvidia.json` existir, o launch define
`__EGL_VENDOR_LIBRARY_FILENAMES` com ele; sem isso o EGL escolhe o Mesa e a câmera não renderiza
no container. O `sim.launch.py` (um veículo só) usa a mesma regra, com a tag 0 e câmera se o tipo
de veículo tiver.

O que o bringup de um veículo é mora no módulo Python `agri_gz_bringup.vehicles`
(`agri_gz_bringup/agri_gz_bringup/vehicles.py`, instalado com `ament_python_install_package`) e é
usado pelo `sim.launch.py`, pelo `convoy.launch.py` e por qualquer projeto que monte o próprio
cenário, porque um veículo são alguns nós e uma regra sobre nomes de tópicos, e ter isso escrito
duas vezes é ter duas definições que vão discordar:

```python
from agri_gz_bringup.vehicles import vehicle_nodes
nodes = vehicle_nodes("pickup", "pickup", "/pickup", (0.0, 0.0, None, 0.0))   # z None: spawn.yaml
# tag_id, tag_family, tag_size, camera, camera_mount e sensor_profile são opcionais
```

Cada veículo cai da sua própria altura, declarada em `<veiculo>_gz/config/spawn.yaml` (`z: 0.4`
para o trator, em rodas de 0,75 m, `0.3` para a pickup), então nenhum launch precisa saber o
tamanho de uma máquina.

### Dirigindo sozinho, pelas bordas da estrada

O `road_follow`, que dirige um veículo pelos carreadores pela câmera, mora agora no repositório
`ftl`: o FTL o usa para dirigir o líder. O README de lá conta a história inteira de como ele acha a
estrada. Ele roda com qualquer veículo lançado daqui:

```bash
ros2 launch agri_gz_bringup sim.launch.py robot:=autonomous_tractor world:=citrus_farm.sdf
ros2 run road_follow road_follow                     # segundo shell
```

O que um veículo publica: `/odom`, `/joint_states` e `/tf` pelos controladores, e pela bridge
`/clock`, `/imu/data`, `/gnss/fix`, `/camera_front/image_raw` e `/camera_front/camera_info`. Sem
lidar — veja abaixo.

O veículo começa **no centro do mundo**, no carreador que passa pela origem e apontado ao longo
dele, em qualquer um destes mundos e sem pose por mundo: veja as convenções abaixo. O `/gnss/fix`
é reportado no `gnss_link`, que fica sobre o **centro da máquina** a 2,685 m, e a
`/camera_front/image_raw` vem da **face dianteira** dela a 1,460 m — os dois posicionados e
explicados no `robot_description`, que é o dono de onde um sensor fica. A média de 285 fixes com o
trator parado na origem põe a antena a 1,122 ± 0,058 m a leste e 0,018 ± 0,058 m ao norte dela,
contra os 1,075 m e 0 que a descrição pede.

## Mundos agrícolas

`agri_gz_worlds/worlds/` guarda uma propriedade em quatro cortes e o mundo vazio para recorrer. O
campo é disposto como um de verdade: talhões de fileiras plantadas, carreadores largos o bastante
para manobrar uma máquina, uma cabeceira no fim das fileiras e uma divisa que tem de ser
respeitada.

As quatro fazendas compartilham o chão, o plantio, a mata e o pátio. A **única** diferença entre
elas são as curvas que as estradas fazem, e é isso que as torna dignas de comparação: pergunte o
que uma de 30 custa a um robô que uma de 90 não custa, e o mundo é a única variável. Três delas
usam um ângulo só; a quarta mistura os três, para quando o campo deve ter a variedade de uma
propriedade real, e não a variável única que um experimento quer.

| Mundo | Campo | Para que serve |
|-------|-------|----------------|
| `citrus_farm_90.sdf` | 410 × 410 m, 5421 árvores, 10 curvas | **Estradas que atravessam a propriedade**, e não anéis dentro dela: quatro carreadores vão de borda a borda e se cruzam, e os talhões são o que os cruzamentos deixam. Toda curva é de 90°, divisa incluída. O trator e a carreta de bins ficam estacionados no pátio ao lado do galpão, a 20 m da origem. Os quatro modelos de árvore estão plantados nele, como nos dois irmãos. |
| `citrus_farm_60.sdf` | a mesma propriedade, 4835 árvores, três carreadores | Cada estrada faz uma curva para fora e uma de volta. Uma divisa que se fecha em si girou 360°, então a 60° ela é um hexágono. Este também é o mundo com o único talhão que **dá fruta** — seis fileiras, 131 árvores, a primeira a 11 m ao sul da origem. |
| `citrus_farm_30.sdf` | a mesma propriedade, 4538 árvores, dois carreadores | Uma de 30 mal muda para onde a estrada aponta, então uma esquina leva três curvas seguidas, e duas estradas é tudo o que a propriedade comporta: mais que isso e elas correm lado a lado em vez de se cruzar. O ângulo aparecendo no traçado, e não como enfeite. |
| `citrus_farm.sdf` | um **quadrado de 560 m**, um cruzamento, **quatro talhões** de 266 m, 13 600 árvores, **mais quatro estradas oblíquas**: 60° em cada talhão de cima, 30° em cada um de baixo | A propriedade de que este workspace realmente trata. Quatro cantos de divisa de 90°, todos arredondados a 24 m e todos pintados; dois carreadores de borda a borda cruzando na origem em ângulo reto, cortados em esquadro, sem número em nenhum — um cruzamento não é uma curva. O plantio são os quatro quadrantes que essas estradas deixam e nada mais: **todo talhão é cercado por carreadores nos quatro lados**, fileiras leste-oeste em dois e norte-sul nos outros dois, e **uma copa por talhão** — duas lisas e duas irregulares, uma de cada par com fruta. A mata é um **cinturão em volta de tudo**. Cada estrada oblíqua de cima é **60° nos seus três pontos**: sai da estrada da divisa a 60° em vez de em esquadro, vira 60° no **meio do seu talhão** e encontra o carreador principal a 60°. Cada uma de baixo é **30° nos dois entroncamentos**, saindo do principal e chegando à divisa lateral, com a curva a 72 m do meio do talhão: a 30° nenhum retângulo comporta as duas coisas, e a seção abaixo mostra por quê. Cada par se espelha, leste e oeste, e as quatro têm o ângulo pintado. |
| `empty.sdf` | chão, sol, os sistemas | O teste de fumaça. Se um veículo se comporta mal aqui, o mundo não é o motivo. |

**O que fica solto na cabeceira: árvores nativas e pedras.** Havia cupinzeiros aqui — quarenta
montes de 1 m feitos de elipsoides marrons, espalhados ao lado dos carreadores — e eles saíram:
nesse tamanho ninguém olhando o mundo sabia o que eles deviam ser, e um caroço marrom ao lado de uma
estrada parece algo que caiu, e não algo que cresceu. Agora são `forest_tree`, de pé onde os montes
estavam, a 7,5 m da linha central do carreador, o que põe o tronco 1,5 m fora da terra nua e deixa
a copa inclinada um metro sobre o acostamento, como uma árvore ao lado de uma estrada de fazenda. O
que ela não pode cobrir são os 7 m de pista de rodagem, e com 2,6 m de copa a 7,5 m ela os livra
por 1,4 m.

Árvores e pedras são postas **alternadas**, e não um tipo e depois o outro, o que importa no
`citrus_farm`: uma propriedade plantada até os carreadores tem lugar para vinte e duas coisas soltas,
onde os mundos cujos talhões deixam folga levam as cinquenta e oito, e o tipo que vinha primeiro
levava tudo. Revezando, o `citrus_farm` termina com 14 árvores e 8 pedras e os outros três com as
40 e 18 completas.

### Quatro copas, e as fileiras são pré-assadas

Uma propriedade é plantada com **quatro modelos de árvore**, e qual talhão recebe qual é uma
tabela, `PLANTING`, perto do topo do gerador. A mesma tabela nos três mundos, então as copas nunca
são o que difere entre eles:

| Talhão | Copa | Por quê |
|--------|------|---------|
| onze de catorze | `citrus_tree` | A copa poligonal, pré-assada oito por trecho. O padrão, e o único quase de graça. |
| **e** (noroeste), **l** (ao sul do pátio) | `citrus_round` | Um elipsoide, o mesmo envelope. O talhão de controle. |
| **n** (no cruzamento) | `citrus_tree_fruit` | 131 árvores com laranjas, a 11 m ao sul da origem. |
| as quatro matas | `forest_tree` | Dois elipsoides aninhados num tronco, espalhados, sem fileira. |

Eles estão espalhados, e não lado a lado, de propósito: nenhuma rota pela propriedade deixa de
passar por eles, e nenhum pode ser confundido com uma mudança no chão. No mapa do cabeçalho de cada
mundo eles são `:`, `o`, `*` e `#`.

Estes mundos desenhavam um citros como **um elipsoide flutuando a 1,75 m sem nada embaixo**. A
defesa era o custo: um tronco dobraria a contagem de visuais com 2275 árvores. Essa defesa era sobre
um tronco como *segunda forma*. A copa em malha leva **o tronco dentro da mesma malha**, então o
tronco sai de graça — o Gazebo cobra por **entidade**, não por triângulo — e o `citrus_round` é o
elipsoide antigo trazido de volta de propósito, com a saia até o chão em vez de pairar sobre o nada.
A forma nunca foi o problema; a flutuação era.

O que *não* é de graça é a entidade. O servidor percorre todo `<visual>` a cada passo, se mova ou
não e haja algo renderizando ou não, e o `citrus_farm_60` tinha **7255** deles, 4966 sendo árvores.
Tirar os visuais por completo o levou de 312 passos de física por segundo a 1428 — era para lá que o
fator de tempo real estava indo.

Então as árvores são **pré-assadas em trechos**: oito por malha, depois quatro, depois duas, de uma
biblioteca em `models/citrus_tree/meshes/`, deixando no máximo uma árvore de qualquer fileira
plantada sozinha. Medido em sequência na mesma máquina, `citrus_farm_60`:

| | visuais | passos/s | RTF no passo de 3 ms do mundo |
|---|---|---|---|
| um visual por árvore | 7255 | 142, 147 | 0,43, 0,44 |
| trechos pré-assados | 3545 | 277, 285 | 0,83, 0,86 |

**1,95× nas duas repetições**, com a geometria inalterada — as mesmas copas, nos mesmos lugares, com
o mesmo giro, escala e jitter por árvore, gravados na malha em vez de aplicados por visual. (As duas
execuções dividiam a máquina com outra simulação, por isso os números absolutos são baixos; a razão
é o que importa. Sozinho, o mundo antigo media 312 passos/s.)

Fileiras inteiras seriam ainda melhores — 339 visuais em vez de 620 — e são impossíveis: um OBJ de
uma copa tem 87 KB, então fileiras únicas poriam **430 MB** de geometria num mundo e 1,3 GB nos
três. Seis trechos de cada comprimento são 10 MB para todos, e 48 árvores distintas bastam para uma
fileira de quarenta não se repetir visivelmente. Oito copas também são 24 m de cerca viva, o
comprimento em que uma seção padrão de fileira sempre foi cortada.

Três talhões são plantados árvore por árvore mesmo assim, e esse é o propósito deles. O talhão logo
ao sul do cruzamento — seis fileiras, 131 árvores, a primeira a 11 m da origem — dá **fruta**, e a
malha da fruta pertence a uma copa, então não há trecho pré-assado com fruta. Os dois talhões
redondos abrem mão dos trechos pelo motivo oposto: uma primitiva não pode ser pré-assada numa malha.
É o `PLANTING` que põe cada um onde está.

**O que os sensores custam, medido.** Com o trator criado no `citrus_farm_60`, cronometrando 20 s
fixos de tempo *simulado* (repetível a 2%; amostrar o `/stats` não é — variou de 0,19 a 0,78 para
uma mesma configuração):

| | relógio para 20 s de simulação | RTF |
|---|---|---|
| lidar 360° a 10 Hz + câmera, plantas colidindo | 62,1 s | 0,32 |
| sem o lidar | 32,8 s | 0,61 |
| sem o lidar, plantas sem colisão | 27,0 s | **0,74** |
| também sem as estradas (não feito — veja abaixo) | 25,0 s | 0,80 |

As partes se somam: física e tudo o que não é sensor, 14,9 s; o lidar, 30,4 s; a câmera, 16,8 s. O
lidar era metade do orçamento para um sensor só, e por isso o trator não leva mais um. A física
sozinha rodaria a RTF 1,34.

Duas coisas que parecem soluções e não são. **Aumentar o passo de física**: 3 ms → 6 ms compra 5%,
e 10 ms compra os mesmos 5%, porque a física é um quarto do custo e os sensores são cronometrados em
tempo simulado. **Baixar a resolução dos sensores**: 1280×720 → 640×480 e 900 → 450 amostras de
lidar não mudaram nada mensurável. O custo é uma travessia da cena por quadro, e escala com o número
de *visuais*, não de pixels: 3563 visuais → 62,1 s, 2925 → 49,8 s, 2128 → 35,4 s.

**O que estava na mesa, e foi tomado.** A superfície das estradas era cortada em ladrilhos pelas
dobras do chão, sem colisão nenhuma: 1435 visuais no `citrus_farm_60`, 40% daquele mundo, contra
1097 no de 30 e 740 no de 90. Apagá-los de vez levou a configuração acima de 27,0 s para 25,0 s de
relógio, e, antes de o lidar sair, de 62,1 s para 35,4 s. Agora são uma malha por mundo, que é essa
economia sem apagar nada. (Uma nota anterior aqui dizia que a medição estava dentro do ruído.
Estava — porque foi tirada sem nenhum sensor renderizando, que é exatamente quando visuais parecem
de graça.) Aumentar o passo de física de 3 ms compraria o resto quase linearmente: medido no
pequeno rover com que estes mundos foram testados primeiro, de 1 ms a 8 ms em terreno plano não fez
diferença nenhuma (sem movimento vertical), então a folga existe se um dia for necessária — uma
máquina de 4 t em pneus macios merece ser medida de novo antes de se contar com isso.

Quatro convenções valem nos quatro, e vale mantê-las num quinto:

- **+X ao longo das fileiras, +Y através delas.** Tudo fica mais fácil de ler com isso fixo.
- **A origem fica num carreador, em terreno plano, em z = 0.** O perfil é nivelado entre
  x = ±25 m em todos estes mundos, então o meio da propriedade é um pátio nivelado faça o relevo o
  que fizer mais adiante. É isso que deixa o `sim.launch.py` criar um veículo em qualquer um deles
  **no centro da simulação, sem pose por mundo**: os padrões são `x:=0 y:=0 yaw:=0`, e a altura de
  queda é do próprio veículo (`config/spawn.yaml`), não uma propriedade do mundo. Medido, com o
  trator criado em cada um dos quatro: ele assenta em (0,0000, 0,0000, 0,0000) com roll, pitch e
  yaw todos 0,00°.
- **Marcas no chão não têm colisão.** O solo trabalhado de um talhão e a superfície de um carreador
  são pintados no chão, não construídos sobre ele: um degrau de 2 cm não é nada para um trator e é
  um muro para um robô pequeno em rodízios de 4 cm.
- **Toda planta é estática.** Um campo de corpos dinâmicos é um campo para o qual o motor de física
  resolve contatos a cada passo, e uma barreira que é empurrada para o lado no primeiro esbarrão.

**Nenhuma planta colide, em lugar nenhum.** Nem nos mundos, de onde o gerador as tira desde que o
padrão `--collisions ground` entrou, nem nos quatro modelos de árvore, que agora não têm
`<collision>` nenhum — então um mundo que `<include>` um deles recebe o mesmo tratamento. Uma câmera
e um lidar de GPU trabalham sobre *visuais*, então a folhagem continua sendo exatamente o que um
sensor vê; o solver estava sendo cobrado por cilindros de tronco que nada numa execução de
percepção toca. Doze colisões sobram em cada mundo e são todas chão ou construção: as onze placas de
terreno e o galpão. Atravessar uma árvore é a troca; cair pelo chão não é.
`generate_agriculture_worlds.py --collisions full` ainda existe e devolve as colisões das plantas ao
mundo — os arquivos de modelo não têm nenhuma a devolver.

### A biblioteca de modelos

`agri_gz_worlds/models/` está no `GZ_SIM_RESOURCE_PATH`, então qualquer mundo do workspace pode
`<include>` estes por `model://<nome>`. Nove deles: quatro árvores, as estradas, três máquinas e um
tanque.

Nenhum deles colide. Os tamanhos são medidos na geometria, não nos raios de que ela foi feita.

| Árvore | Tamanho | Visuais | Notas |
|--------|---------|---------|-------|
| `citrus_tree` | 3,95 × 2,83 m | 1 | Copa poligonal, tronco na mesma malha. Seis variantes em `meshes/`, mais os trechos de 2, 4 e 8 com que as fileiras são de fato montadas — é o que faz um visual servir para oito árvores. **Do que a maior parte de todo talhão é plantada.** |
| `citrus_tree_fruit` | 3,95 × 2,83 m | 2 | A mesma copa com uma segunda malha de laranjas dentro. Nenhum trecho pré-assado tem fruta, então uma fileira com fruta é plantada uma árvore por vez e ainda paga a fruta por cima: dezesseis vezes as entidades da mesma fileira em trechos. Um talhão a leva. |
| `citrus_round` | 3,12 × 2,76 m | 2 | Tronco e um elipsoide — a esfera de que a malha é empurrada para fora, arredondada em planta, sobre 0,44 m de tronco visível. O **controle**: o que uma pilha de percepção faz de diferente entre um talhão redondo e um em malha vem da folhagem, e não do campo. |
| `forest_tree` | 5,16 × 6,71 m | 3 | Dois elipsoides, um dentro do outro, num tronco — copa a partir de 2,59 m, então uma barra passa por baixo. Do que a **mata** é feita, espalhada em vez de em fileira. |
| `carreador` | — | 0 | Não é um modelo que alguém posicione: as estradas pré-assadas. Um OBJ por mundo com todo carreador dele como uma faixa nomeada — terra nua e pista de rodagem — que cada mundo escolhe pelo nome com `<submesh>`. Escrito junto com os mundos, nunca à mão. |

| Máquina | Notas |
|---------|-------|
| `bin_trailer` | Quatro rodas, três bins. Atrás de um trator dá 9 m de veículo, que é o que de fato decide quão larga uma estrada e quão aberta uma curva têm de ser. Estacionada no pátio, engatada na barra de tração do trator. |
| `tractor` | 3,6 × 2,15 × 2,6 m. A régua pela qual todo carreador foi traçado — e o mesmo objeto que o workspace dirige: o `autonomous_tractor_description` copia a carroceria dele painel por painel no `params.xacro`, até o azul. Os dois num mundo são uma máquina em dois estados, a estacionada junto ao galpão e a dirigida no carreador. |
| `harvest_bin` | Bin de campo deixado num corredor: em mapa nenhum, e sumido na semana que vem. |
| `water_tank` | O único marco alto e isolado num campo em que toda fileira parece igual. |

Só quatro dos nove são `<include>`ídos por um mundo — as máquinas e o tanque. Um modelo também é
uma **definição**, e essa é a outra metade do motivo deste diretório: os mundos referenciam as
malhas do `citrus_tree` por URI e reproduzem o elipsoide do `citrus_round` e o par do `forest_tree`
como geometria inline. Os números vivem num lugar só, e o arquivo do modelo é onde eles são
explicados.

Por que inline: medido em 300 objetos estáticos idênticos, **300 modelos custam 0,8 ms por passo, os
mesmos 300 como links dentro de um modelo 0,5 ms, como formas simples dentro de um único link
0,3 ms**. O Gazebo cobra por entidade e um modelo são várias delas, então onde a contagem chega a
centenas — toda árvore, toda árvore solta, toda pedra — a geometria vai inline, num modelo por
quadrante. Onde a contagem é cinco, o `model://` fica: é o jeito melhor de escrever um mundo e a
diferença não aparece num passo.

### Estradas que fazem curva, e fileiras cortadas por elas

Os mundos são todos construídos do mesmo jeito, e vale saber disso antes de editar um. Um carreador
é escrito do jeito que é dirigido:

```python
Path(0, 0, 0).straight(44).turn(-30, 26).straight(34).turn(-60, 20)
```

Ângulos positivos viram à esquerda, e o segundo número é o raio da linha central — o que decide se
um conjunto contorna a curva ou tem de manobrar. A linha central vira uma espinha — as duas pontas
de cada reta, uma estação a cada 2,5 m de arco, e uma estação por vértice agudo levando os dois
rumos — e cada camada da estrada é uma fita de triângulos ao longo dela. Uma estrada inteira é um
modelo, e cada camada dela um visual, por mais longa que seja.

As fileiras então **não** são traçadas e desenhadas. Cada linha de fileira é percorrida a cada
0,5 m e mantida só onde o chão é plantável: dentro do seu talhão, fora de todo corredor de estrada,
fora da mata, longe do pátio e a 4,5 m do talhão vizinho. Os trechos contínuos que sobram viram
fileiras; um trecho curto demais para três árvores fica vazio. É isso que faz o campo terminar em
esquadro contra uma curva, e quer dizer que editar uma estrada recorta automaticamente todo talhão
ao lado dela.

O cabeçalho de cada mundo traz um mapa dele, rasterizado pelo gerador a 5 m por caractere. É o
jeito mais rápido de ver o que uma mudança fez sem abrir o simulador.

Toda curva recebe o ângulo pintado automaticamente, a partir de `Path.marks()`: **no meio do
próprio arco**, algarismos escuros numa placa clara, lado a lado, e **todo número disposto com o
norte para cima**, qualquer que seja a direção da estrada — orientação consistente é o que deixa um
conjunto deles legível do alto. Sete caixas por algarismo, porque o SDF não tem geometria de texto
e estes mundos não levam texturas.

**Toda placa tem o mesmo tamanho, sempre.** Ela encolhia para 68% de si quando a busca não achava um
lugar livre de uma dobra do chão, e isso é um defeito pior que o que corrigia: numa propriedade com
números de dois tamanhos, os pequenos parecem querer dizer outra coisa — uma curva menor, outro tipo
de marca — quando só queriam dizer que o chão embaixo deles tinha um vinco. Tamanho não é informação
aqui; os algarismos são. Então a busca move a placa e nunca a redimensiona, e tem mais lugares para
onde movê-la: nove pontos ao longo do arco em vez de três, depois alguns metros além de cada ponta.
Nos quatro mundos ela agora acha um lugar limpo para os 48 números sem recorrer uma vez sequer.

Ela ficava 28 m antes, na reta que levava à curva, como uma placa de trânsito, e esse é o modelo
errado: uma placa é um aviso e precisa do recuo; um número no chão é um rótulo. Recuado, ele tem de
ser associado à curva no olho, e numa propriedade de ângulos misturados o número antes de uma curva
e o número depois dela ficam ambos à vista de um mesmo ponto sem nada que diga qual é qual. No arco
não há dúvida. Dos 48 números nos quatro mundos, 34 ficam bem no centro do seu arco; os outros 14
são deslocados ao longo dele, ou alguns metros além da ponta, para livrar um entroncamento ou uma
dobra do chão sobre a qual a placa não deitaria plana. `Path.close_to_y()` calcula a última reta de
cada laço de talhão, então mudar um raio ainda faz a estrada voltar ao carreador de onde saiu.

Nenhuma estrada termina em lugar nenhum a não ser em outra estrada. Os carreadores divisórios vão de
borda a borda e terminam numa divisa que se fecha em si, e o `check_network()` garante isso na hora
de gerar — ele se recusa a escrever um mundo cujas estradas quebrem qualquer uma de três regras:

| Regra | O que pega |
|-------|------------|
| **dentro da divisa** | Uma estrada cuja linha central sai mais de 6 m para fora do perímetro. |
| **as duas pontas nela** | Uma estrada que para no meio de um campo, que é uma estrada de onde uma máquina tem de sair de ré. |
| **não paralelas** | Duas estradas a menos de 32 m uma da outra apontando para o mesmo lado por mais de 25 m. |

Cada uma delas foi um defeito real antes de ser uma verificação. O terceiro carreador do
`citrus_farm_60` começava na ponta de baixo do hexágono e seguia para leste, onde a propriedade tem
**35 m de largura**: 180 dos seus 270 m corriam fora da divisa e voltavam, o que no mapa é uma
estrada e uma divisa se cruzando duas vezes com uma lente entre elas — um oito — e no mundo é uma
estrada pelo campo de ninguém. Ela sobreviveu porque o `close_to_boundary()` só olha para onde uma
estrada *termina*, e nada olhava para onde ela tinha passado. Essa estrada agora vai da borda
sudeste à noroeste e cruza as outras duas no caminho.

Cruzar outro carreador é reportado, e não imposto. O `south` do `citrus_farm_90` vai de borda a
borda e não encontra nada além do perímetro, o que é mais ralo do que o projeto quer, mas não é
errado como as três acima são, então gerá-lo imprime uma nota e segue.

### Onde uma de 90 cabe, e por que uma curva precisa de espaço

Duas descobertas, ambas de olhar o `citrus_farm` depois de construído pela primeira vez, e ambas
agora regras, e não opiniões.

**Uma curva precisa de reta dos dois lados.** Uma curva de ângulo θ e raio *r* come `r·tan(θ/2)` da
reta em cada ponta: 6,4 m para uma de 30 com r=24, 13,9 m para uma de 60, **24 m para uma de 90**. O
primeiro corte daquele mundo pôs uma de 60 e uma de 90 a sessenta metros uma da outra no papel, o
que dá 60 − 13,9 − 24 = **22 m** de estrada real entre os dois arcos — duas larguras de estrada. Elas
se emendavam e o par parecia uma curva longa só, então a de 90 parecia uma estrada que se desviou,
e não uma de 90. `check_network(bend_spacing=45)` é essa regra agora, e o `citrus_farm` deixa 50 m
entre cada par de curvas. Os mundos de ângulo único deliberadamente não seguem a regra: três de 30
seguidas é como uma estrada de 30 faz uma esquina ali.

**Uma de 90 cabe onde a estrada muda o que está fazendo.** Uma de 30 ou de 60 desvia uma estrada e
ela segue atravessando a propriedade; uma de 90 a põe no outro eixo. Então uma de 90 parece certa
onde um carreador para de correr num sentido e começa a correr em outro, e parece errada largada no
meio de uma travessia. O raio não é a alavanca que parece: o mínimo é **22 m** de linha central,
dado por um trator com uma bin_trailer carregada precisando de 18,5 m no lado de dentro da curva,
então uma de 90 não pode ser muito mais fechada do que é. Espaçamento e posição são as alavancas.

**Duas estradas se cruzando são duas superfícies na mesma altura, e a ordem de empilhamento é por
camada, não por estrada.** Todo carreador era desenhado em z = 0,013, então um cruzamento punha duas
faces planas sobre o mesmo chão na mesma altura, e faces coplanares são o que um renderizador não
consegue escolher: o entroncamento pisca conforme a câmera se move, o que parece um defeito no
entroncamento e é um defeito no z. Escalonar as estradas resolveu a piscada e trouxe um defeito
pior, que precisou de uma câmera para ser achado. Empilhar cada estrada como uma unidade — terra,
pista, marcas, a próxima estrada um milímetro acima — põe a **pista de um carreador sobre as marcas
de outro**. Sete metros de pista cruzando a 30° cobrem **catorze metros** do que cruzam, então a marca
no meio da estrada principal simplesmente parava em todo entroncamento. O trator guiado por câmera
seguia o que sobrava, que era da estrada que chegava, e saía pela estrada de 30° para dentro do
talhão — duas vezes, antes de se suspeitar do mundo, e não do seguidor.

Então o empilhamento é **por camada, e não por estrada**. O `ROAD_LAYERS` são duas faixas — terra a
10 mm e pista a 14, com os números pintados a 18 — e o lugar de uma estrada é um degrau de meio
milímetro **dentro** da sua faixa. Toda pista fica então acima de toda faixa de terra, seja de quem
for, então um cruzamento mostra as duas estradas patroladas, e não o acostamento de uma estendido
sobre a pista da outra. O gerador confere que as faixas ainda comportam o escalonamento, o que com
sete estradas são 3 mm de escalonamento contra 4 mm de faixa, e não sobreviveria a nove.

**Um canto agudo é uma meia-esquadria, não um remendo.** Duas faixas de estrada se encontrando num
vértice deixam o quadrante de fora descoberto — 6 m de mordida numa estrada de 12 m a 90° — e isso
era coberto com um quadrado da largura da estrada, em esquadro com a estrada, e não com a bissetriz
(na bissetriz ele preenche o entalhe também, e passa das duas bordas por `largura/√2`: **8,5 m de
estrada apontando para o mato como uma ponta de flecha**). O remendo saiu. Um vértice agora é uma
estação cujos dois rumos diferem, e a borda da fita sai ao longo da **meia-esquadria** — o ponto
onde as duas linhas deslocadas se cruzam, que é a normal esticada por `1/cos(metade do giro)`, 1,41
num ângulo reto. É o mesmo canto em esquadro que o remendo desenhava, com as mesmas duas linhas de
aritmética de toda outra estação, sem nada para sobrepor e sem peça separada para recortar em volta
das dobras.

**O pátio é escolhido antes do plantio.** A clareira que mantém as fileiras fora dele é derivada de
onde ele cai, então os dois não podem se separar. Já se separaram: o pátio buscava chão livre depois
de o plantio estar feito, se movia, e deixava a clareira para trás — um galpão com citros a **4 m da
parede**. Agora são 31 m, e o `check_props()` mantém toda peça do pátio, o maquinário e os bins a
**15 m** de qualquer carreador, porque as estradas são a única parte destes mundos a que o veículo
tem direito.

**Ângulos retos por fora, os outros ângulos por dentro.** Uma propriedade real é esquadrejada na
borda, porque é ali que um agrimensor a desenhou, e as estradas que cortam os talhões correm no
ângulo que for preciso para atravessá-los. O `citrus_farm` é traçado assim: um **retângulo de
360 × 420 m** cujos quatro cantos viram 90 — dois em esquadro, dois arredondados a 24 m —, dois
carreadores de moldura cruzando-o de borda a borda e se encontrando em esquadro no meio, e duas
**diagonais** cortando os talhões que eles deixam, com as de 30 e as de 60. Um cruzamento não é algo
em que se vira, então as estradas de moldura não levam número pintado nenhum; toda curva que vira
uma estrada leva um.

**Uma fazenda vira dos dois jeitos.** Uma estrada patrolada é arredondada onde há espaço e
simplesmente cortada onde dois talhões se encontram em esquadro, e as fotografias aéreas estão
cheias das duas — principalmente da segunda. Então `Path.turn(graus, raio)` aceita **raio 0**: sem
arco, um vértice. Não custa reta também, onde um arco come `r·tan(θ/2)` em cada ponta, e é assim que
uma de 90 aguda cabe onde uma arredondada não caberia. Duas faixas de estrada se encontrando num
vértice deixam um entalhe triangular no lado de fora da curva, que a meia-esquadria daquela estação
preenche exatamente, e que é o pedaço de chão que uma patrola esquadreja de qualquer jeito. O
`check_network(pairs=True)` então exige que **todo ângulo apareça dos dois jeitos**: o mundo tem uma
de 30, uma de 60 e uma de 90 agudas e uma de 30, uma de 60 e uma de 90 arredondadas, e não compila
sem as seis.

**Os carreadores pertencem ao veículo.** Nada que uma máquina tivesse de contornar pode ficar a menos
de **15 m** da linha central de uma estrada: os 6 m de terra nua que a estrada tem, mais espaço para
passar pelo que estiver estacionado sem pôr uma roda na cultura. O `check_props()` garante isso, e o
pátio — galpão, tanque, bins, o trator estacionado e a carreta dele — é posto como um bloco cuja
âncora é buscada até toda peça livrar. Isso pegou o mundo para o qual foi escrito: o tanque d'água
estava a **1 m** de uma linha central e os bins de campo a 4,8 m, o que é um tanque e dois bins
estacionados numa estrada que um trator sem motorista deveria ter só para si.

**E o veículo começa nela, apontado ao longo dela.** O `sim.launch.py` cria em 0 0 com yaw 0 e sem
pose por mundo, então o carreador que passa pela origem tem de correr ao longo de **+x**: uma
máquina que começa 20° atravessada na própria estrada passa o primeiro movimento se corrigindo. O
`check_network` mede o rumo da estrada sob a origem e recusa qualquer coisa a mais de 3°.

**O número vai no arco, e fora do entroncamento.** Vinte e dois metros de folga a qualquer outra
estrada: os 5 m da própria placa, mais os 11 m de terra nua que a outra estrada tem, mais o bastante
para ver a que estrada o número pertence. A busca pega o meio do arco, depois os dois lados dele ao
longo do arco, e só então até 12 m além das pontas — nunca longe o bastante para parecer o número da
reta. Nos quatro mundos o número mais apertado agora livra **20,2 m**, contra **5,1 m** antes, que
era uma de 90 deitada sobre um cruzamento.

### Uma propriedade, quatro redes de estradas

Os quatro saem de um construtor, `_build_farm()`, que recebe as estradas dos talhões como uma
especificação e nada mais. O terreno, os talhões, a mata, o pátio e o jeito de cortar as fileiras
são código comum, então a única diferença entre os quatro mundos é a geometria dos seus carreadores
— que é o que os torna dignos de comparação.

Três deles tiram o ângulo de `through_network()`, que o aplica a toda curva; o mundo do cruzamento
usa `mixed_network()`, que recebe cada curva em graus, porque uma mistura é todo o propósito dele.

Os quatro são traçados como **rotas de passagem**: cada estrada vai de uma borda da propriedade a
outra e elas se cruzam no caminho, então os talhões são o que os cruzamentos deixam. Um corte
anterior circundava cada quarto da fazenda com um laço, o que produzia um retângulo dentro de um
retângulo dentro de um retângulo, em que toda rota contornava algo e nenhuma ia a lugar nenhum.

Quantas estradas uma propriedade comporta acaba sendo função do ângulo. A 90° cabem quatro; a 60°,
três; a 30°, duas, porque uma de 30 mal muda para onde a estrada aponta, então qualquer terceira
estrada acaba correndo ao lado de uma das outras em vez de cruzá-la.

Os traçados são declarativos — `dogleg([(x, y), ...], raio)` recebe os cantos e calcula as retas
para que cada arco seja tangente às duas —, então mudar uma rede é mover um canto, e não refazer
trigonometria.

Duas regras mantêm as redes legíveis, a primeira imposta pelo `check_network()` acima:

- **Dois carreadores não correm lado a lado.** Cruzar é um entroncamento, e entroncamentos são o
  propósito; um laço passando 5 m fora do perímetro é uma bagunça larga com uma tira de cerca viva
  encalhada no meio. Todo laço é dimensionado para que nada paralelo chegue a menos de **32 m**.
- **O ângulo de uma curva é pintado na curva.** O arco é o trecho mais largo e plano de estrada que
  existe — 11 m de terra nua pela curva, contra uma placa de 7,8 por 6,0 m — e é o único lugar onde
  um rótulo não pode ser lido como pertencendo à próxima curva.

Duas restrições fazem o trabalho de projeto nos mundos de ângulo único. Um laço que sai do carreador
leste-oeste rumo norte e volta a ele rumo sul girou **180°**, então o ângulo fixa quantas curvas ele
tem: duas de 90, três de 60, seis de 30, sendo cada laço meio polígono regular. Uma divisa que se
fecha em si girou **360°**, então o perímetro é um polígono de `360/ângulo` lados. Nada é plantado
fora dele.

### Sessenta graus cabem no meio de um talhão; trinta não

O `citrus_farm` acrescenta quatro estradas oblíquas ao cruzamento, e elas são a mesma ideia em dois
ângulos: sair de um carreador no ângulo, virar o ângulo em campo aberto, chegar a outro carreador no
ângulo. Três entroncamentos de três tipos diferentes — um T saindo da divisa, uma curva no aberto, um
T chegando ao principal — todos com a mesma volta de volante.

**A 60° a aritmética fecha.** Sessenta graus a partir de uma estrada horizontal põem o primeiro
trecho a −60; virar 60 põe o segundo a −120; e tanto −60 quanto −120 estão a 60° da horizontal,
então o primeiro trecho pode terminar numa estrada horizontal e o segundo também. Nada fica
sobredeterminado, e a curva fica livre para ficar onde se quiser — aqui, **a 3 m do centro do seu
talhão de 266 m**, que é a outra metade do propósito: uma máquina entrando pelo norte encontra a
curva com o talhão inteiro em volta, e não contra uma cerca, e as fileiras dos dois lados são
cortadas por ela.

**A 30° não fecha, e nenhuma largura de talhão faz fechar.** Um trecho a 30° da horizontal encontra
uma estrada *vertical* a 60°; um trecho a 60° encontra uma estrada *horizontal* a 60°. Os dois
trechos de uma curva de 30° são −30 e −60, então um quer terminar num carreador horizontal e o outro
num vertical — um pede um talhão alto, o outro um largo, e nenhum retângulo é as duas coisas. É por
isso que alargar os talhões de baixo não resolve: a restrição é na *proporção* que os dois trechos
exigem ao mesmo tempo, e um retângulo tem uma proporção só.

Então as estradas de baixo mantêm o ângulo e abrem mão da posição: **30° saindo do principal, 30°
chegando à divisa lateral, curva a 72 m do meio** do talhão — ainda dentro do terço central dele. O
outro lado da troca é uma curva bem no centro com um entroncamento a 60°, o que abre mão do ângulo
que o mundo existe para testar. Qualquer um dos dois é uma linha de especificação; a escolha é qual
das duas propriedades importa mais.

### Chão que não é plano

Estes mundos têm relevo, e como vale saber, porque os jeitos óbvios não funcionam. **O DART do
Gazebo Harmonic não colide nem com heightmap nem com trimesh nesta build** — os dois foram tentados e
medidos, e uma esfera largada em qualquer um cai direto (até z = −1686 e z = −1093,
respectivamente, a malha carregando sem erro antes). Caixas colidem, então o chão é feito de placas
inclinadas, e como o topo de uma caixa é um plano, a superfície é planar por partes. Cada placa só
pode inclinar em **um eixo**: incline em dois e os topos se projetam em paralelogramos que não se
encontram mais nos cantos, o que nestes tamanhos são 10–20 cm de fresta. Daí relevo ao longo de X e
nível ao longo de Y. As onze placas se encontram a menos de **1,3 mm**.

**O chão vai além da cerca, e o `citrus_farm` vai mais longe.** `extent` é até onde o terreno vai a
partir da origem, e as rampas das pontas do perfil são continuadas em vez de se inventar um novo
relevo. Os 260 m padrão servem às propriedades de 410 m; o `citrus_farm` pede **460** para uma
propriedade que chega a 332, porque uma vista aérea mostrou por que 340 não bastava — chão que acaba
8 m depois da última árvore é um penhasco na borda do mundo, e a fazenda inteira parecia flutuar
sobre o nada. Não custa entidades: as placas são as mesmas onze caixas, mais compridas, e o mundo
ainda roda 20 s de simulação em 22,7 s.

Tudo é posto sobre essa superfície: as fileiras são quebradas nas dobras para que cada cerca viva
fique num plano, e tudo o que se apoia no chão acompanha a inclinação local.

Estradas pedem mais cuidado que isso, e errar é fácil. As dobras correm de norte a sul, então um
carreador que corre de norte a sul tem uma dobra **no meio dele**, e não atravessada, e um desenhado
como uma tábua plana acaba com uma borda enterrada e a outra no ar — 55 cm nas bordas de 11 m de
terra nua onde a rampa vai de −7% a +3%, o que parece uma estrada partida ao meio no comprimento.

Caixas foram a primeira resposta e são a forma errada para isso. Uma caixa é um plano, então a
estrada tinha de ser cortada em volta de cada vinco: no comprimento, onde uma dobra cruzava a linha
central; na largura, onde uma passava por baixo; e depois picada a cada 3 m perto da dobra, onde a
estrada a encontrava num ângulo e o vinco atravessava o retângulo na diagonal, o que nenhum dos dois
cortes sozinho resolve. Isso deixava **6 a 19 cm** de pior canto, contra 55 a 79 antes — uma
aproximação, com uma granularidade a ajustar, e cara: veja a seção de custo abaixo, onde ela se
revelou a maior parte do que o `citrus_farm` estava pagando.

**Uma estrada agora é uma malha, e a dobra é uma aresta dela.** Uma malha não tem obrigação de ser
planar, então o vinco não precisa ser evitado — pode ser parte da geometria, que é o que ele é no
chão. O `ribbon()` percorre a estrada como uma tira de quadriláteros, recorta cada um por toda dobra
que passa embaixo (`_one_plane_each()`, um polígono convexo cortado por uma linha vertical) e
triangula os pedaços. Todo triângulo que sai fica, portanto, em exatamente um plano do terreno, e
todo vértice fica no chão no seu próprio x. Medido nos quatro mundos amostrando o interior de todos
os 3046 triângulos contra o terreno: **pior desvio 0,000 mm**, onde as caixas deixavam 19 cm. Exato,
sem nada a ajustar, e um visual em vez de várias centenas.

**Um carreador são duas faixas e nada mais.** Doze metros de terra nua, e 7 m de pista patrolada no
meio, algumas contagens mais clara. Sem marcas: duas foram tentadas e as duas saíram — uma linha
branca pintada, porque ninguém pinta uma estrada de fazenda, e depois um par de sulcos de roda,
porque duas faixas de borda dura a 2,1 m uma da outra também pareciam tinta e não batiam com a
bitola de máquina nenhuma da propriedade. A lição da linha sobrevive às duas e está na nota sobre a
ordem em z acima: uma marca só é referência se a estrada que chega à sua não a enterrar.

A estrada pela qual a câmera esterça agora é a própria pista, cujas duas bordas são simétricas em
relação à linha central por construção. Isso não é prêmio de consolação — é uma característica que
a estrada realmente tem, e não precisa ser mantida em dois lugares.

### Coisas dimensionadas para um mundo grande

410 m de propriedade, quatro a cinco mil árvores e dezesseis metros de relevo são pagos em cinco
lugares, e vale entender todos antes de copiá-los:

- eles avançam a **3 ms** por passo, onde o `empty.sdf` avança a 1;
- o citros deles é **uma copa por árvore sem tronco separado** — uma árvore crescida tem a saia até
  o chão de qualquer jeito — e as fileiras são pré-assadas oito copas por malha;
- as **cercas vivas são caixas de colisão e copas dentro de um link por talhão**, e não um modelo
  por fileira;
- a mata e os objetos soltos do chão são **inline num modelo por quadrante**, e não 300 includes;
- os **carreadores são malhas pré-assadas**, uma faixa por camada por estrada, e não um piso de
  caixas em ladrilhos.

Os dois últimos são medidos, em 300 objetos estáticos idênticos: **300 modelos custam 0,8 ms por
passo, os mesmos 300 como links num modelo 0,5 ms, como formas simples num único link 0,3 ms.** O
Gazebo cobra por entidade, e um modelo são várias delas. Juntos, eles são o motivo de cinco mil
árvores e relevo completo chegarem como trinta e poucas entidades e acompanharem o relógio:

| Mundo | visuais | colisões | modelos + includes | 20 s de simulação, headless | RTF |
|---|---|---|---|---|---|
| `citrus_farm.sdf` | 3778 | 14 | 32 + 8 | 22,7 s | 0,88 |
| `citrus_farm_90.sdf` | 2138 | 12 | 32 + 8 | 22,5 s | 0,89 |
| `citrus_farm_60.sdf` | 1915 | 12 | 32 + 8 | 22,4 s | 0,89 |
| `citrus_farm_30.sdf` | 1971 | 12 | 36 + 8 | 22,4 s | 0,89 |
| `empty.sdf` | 1 | 1 | 1 | 22,1 s | 0,90 |

Todo mundo agora fica a 0,7 s do vazio, que é o ponto dos dois parágrafos abaixo: as estradas eram
a maior parte da diferença.

**Um visual estático não é de graça, mesmo sem nada renderizando.** Isso foi suposto ao contrário
por um tempo — o servidor não tem câmera, então por que se importaria — e o `citrus_farm` provou o
contrário. A versão controlada: pegue aquele mundo, mantenha os 39 modelos exatamente onde estão e
reduza os oito modelos de estrada oblíqua de 1139 ladrilhos de caixa a uma caixa cada. Nada mais
muda, e na mesma sessão ele vai de **33,0 s para 24,6 s** de relógio por 20 s de simulação. Contra
o mesmo mundo sem aquelas quatro estradas na sessão limpa, 3904 visuais custaram 23,6 s e 5243
custaram 28,8 s.

Resolva isso contra o mundo vazio, que roda 20 s de simulação em 22,1 s com um visual, e a forma é
simples: cerca de **1,1 ms por passo de custo fixo e 0,6 µs por passo por visual estático**. Num
passo de 3 ms, isso punha 3,2 ms de orçamento de visuais no mundo do cruzamento contra um passo de
3 ms — toda a diferença entre RTF 0,85 e RTF 0,69, por estradas com que um robô nem consegue
colidir.

**Então as estradas foram pré-assadas em malhas, e o custo sumiu.** Um OBJ por mundo em
`models/carreador/meshes/`, toda faixa de todo carreador um objeto nomeado dentro dele, cada visual
SDF escolhendo um com `<submesh>`: 1486 visuais de estrada no `citrus_farm` viraram **23**, e as
estradas da propriedade inteira nos quatro mundos são 3046 triângulos e 153 kB. Headless, aquele
mundo foi de 28,8 s para **22,7 s**; interativamente, de um RTF de 0,59–0,66 para **0,91**. A mesma
troca das árvores, e a mesma lição duas vezes: a 0,6 µs por visual, o que uma cena custa é decidido
por em quantas entidades ela é cortada, e não por quanta geometria contém. Treze mil árvores com um
visual cada também nunca teriam rodado.

**O cinturão cede chão aos talhões, e o mundo fica do tamanho que era.** A mata começou como quatro
manchas dentro da propriedade, virou uma mancha fora dela, depois um cinturão em volta, e foi
aparada duas vezes desde então: 122 000 m² de copa, depois 60 000, agora **24 000** — uma faixa de
**18 m** onde começou com 40. Nada disso foi para aumentar o mundo. Nada nele passa de **332 m** da
origem, que é exatamente onde o cinturão chegava quando a propriedade era um quadrado de 400 m; o que
mudou foi a cerca, que saiu de 200 m para **280** por chão onde a mata ficava. Um cinturão é cenário
por onde ninguém dirige e um talhão é aquilo para o que a máquina está lá, então quando os dois
querem os mesmos metros o talhão leva — e, para o que a mata serve, fechar o horizonte, 18 m dela
fazem tão bem quanto 40.

**Pré-asse a variedade, não a racione.** A variedade de copas era algo que um mundo mal podia pagar.
Medido no mundo do cruzamento com o quadrante noroeste plantado liso e o sudeste com fruta, ambos de
9 hectares:

| | visuais, antes | visuais, agora |
|---|---|---|
| o talhão com fruta | 3194 | 244 |
| o talhão liso | 2986 | 213 |
| os dois talhões de copa poligonal | 922 | 918 |
| todo o resto — estradas, mata, terreno | 896 | 669 |
| **mundo inteiro** | **7998**, RTF 0,45 | **2044**, RTF 1,00 |

A fruta e a copa lisa nunca foram árvores caras. Eram árvores que não podiam ser **pré-assadas oito
por malha**, então cada uma custava um visual próprio e a fruta custava dois, e a conta crescia com
o talhão. O `generate_citrus_meshes.py` agora escreve **quatro bibliotecas** em vez de uma — duas
copas, irregular e lisa, cada uma com e sem fruta —, todas em trechos de oito, quatro e dois, e um
talhão é plantado com a que quiser. Um talhão de fruta custa o que um talhão de qualquer coisa custa,
então a variedade de copas é agora uma escolha livre, e não um orçamento. A biblioteca de malhas foi
de 10 MB para 47 MB, que é todo o preço: a fruta pré-assada é cortada para 18 laranjas por árvore
contra as 60 da árvore avulsa, porque um trecho é aquilo com que um talhão de nove hectares é
plantado, e ninguém conta laranjas a essa distância.

Só servidor — `gz sim -s -r --iterations`, nenhum veículo criado e nada renderizando —, então a
diferença entre os três está dentro do ruído, e o número do mundo vazio não é a pechincha que
parece: ele compra seu 0,90 em **20000** passos de 1 ms contra os 6667 de 3 ms das fazendas, um
terço do custo por passo. O número interativo é o que importa no dia a dia, e ele se sustenta: o
`citrus_farm_60` com a GUI aberta, o trator criado e a câmera dele publicando, em **renderização
por software do Mesa sem GPU**, fica em **RTF 0,69 a 0,80** em seis amostras. Medido do mesmo jeito
a cada etapa: 0,92 a 1,10 com todo talhão em trechos pré-assados, 0,74 a 1,07 quando dois deles
ficaram redondos e um com fruta, 0,69 a 0,80 quando as árvores redondas ganharam troncos. Essa
sequência, e não a headless, é o que um visual custa de fato — ele é cobrado quando algo o
renderiza, e a câmera é o que o renderiza.

Se quiser recuperar mais, os troncos redondos são o lugar para olhar: são 580 dos 4658 visuais
daquele mundo e existem pela aparência, não pela condução. Pré-assar a árvore redonda em trechos de
oito como a árvore em malha é pré-assada levaria um talhão redondo de um visual por árvore para um
por oito, ao custo de a copa redonda deixar de ser uma primitiva.

Maquinário estacionado vai no pátio, e maquinário em movimento nos carreadores; nenhum dos dois cabe
num corredor. Um corredor são 6,5 m de carreira menos **3,95 m** de copa, então um trator de 2,15 m
passa com 0,2 m de cada lado — cabe, por pouco, e também bloqueia o corredor por inteiro, que é todo
o argumento. Esse número era citado como 3,5 m de copa e 0,4 m de cada lado, e os dois estavam
errados: ninguém tinha medido a malha, que saía do gerador com **4,6 m**, deixando 1,9 m de corredor
por onde o trator não passava de jeito nenhum. Cortar as copas em um sétimo foi o que tornou a
aritmética verdadeira.

### Mudando um campo

Os mundos são gerados, e o SDF gerado vai no commit — a build nunca roda o gerador, e o Gazebo só lê
o SDF. Edite as constantes de traçado no topo do construtor correspondente em
`agri_gz_worlds/scripts/generate_agriculture_worlds.py` (espaçamento de fileiras, largura de
corredor, inclinação, número de fileiras) e gere de novo. Duas tabelas perto do topo desse arquivo
são as que vale conhecer: `THROUGH_BLOCKS`, que é onde ficam os talhões, e `PLANTING`, que é com o
que cada um é plantado — mudar um talhão para `"round"` ou `"fruit"` ali é tudo o que trocar uma
copa exige:

```bash
python3 agri_gz_worlds/scripts/generate_agriculture_worlds.py          # reescreve no lugar
python3 agri_gz_worlds/scripts/generate_agriculture_worlds.py --out /tmp   # olhar antes
```

As árvores em si saem de um segundo gerador, que escreve as copas — com tronco e tudo — e a fruta
delas em `models/citrus_tree/meshes/` como OBJ. Mude ali a forma de um citros — quantos lóbulos,
quão funda a saia, quanta fruta — e rode de novo; os mundos não precisam ser gerados de novo depois,
já que só nomeiam os arquivos de malha.

```bash
python3 agri_gz_worlds/scripts/generate_citrus_meshes.py               # reescreve no lugar
python3 agri_gz_worlds/scripts/generate_citrus_meshes.py --out /tmp    # olhar antes
```

Os dois geradores só precisam de Python 3, e o OBJ leva as cores num `.mtl` ao lado, e não no mundo:
uma malha tem uma copa verde e um tronco marrom, e um `<material>` do SDF só consegue sobrescrever
uma malha com uma cor só.

### A textura da tag

`agri_gz_bringup.tags.tag_texture(family, tag_id, folder=None)` grava o PNG do quadrado preto de uma
tag AprilTag (512 px, com o anel preto de fora e sem a margem branca, que vem da placa do
veículo) e devolve o caminho. As famílias são `16h5`, `25h9`, `36h10` e `36h11`; família ou ID
desconhecido dá `ValueError`. O arquivo se chama `tag<família>_<ID>.png` e, sem `folder`, vai para
`agri_gz_tags` dentro do diretório temporário; se já existe, não é regravado.

### A distorção da câmera

O Gazebo Harmonic publica os coeficientes de Brown no `camera_info`, mas o ogre2 não distorce a
imagem. O nó `camera_distortion` assina `<câmera>/image_ideal` e `<câmera>/camera_info` e publica
`<câmera>/image_raw` distorcida, com o mesmo cabeçalho. O nome da câmera é o parâmetro `camera`
(padrão `camera_front`), relativo ao namespace do nó. Com `D` nulo, a imagem é repassada sem
alteração.

`agri_gz_bringup.distortion.distortion_maps(K, D, width, height)` calcula os mapas do `cv2.remap`.

### Perfis de sensor, câmera, textura e pose real

O argumento `sensor_profile` do xacro de cada veículo escolhe os sensores de navegação:

| perfil    | GNSS                  | IMU                  |
|-----------|-----------------------|----------------------|
| `default` | 5 Hz, 1 m e 2 m       | 100 Hz, com ruído    |
| `nominal` | 1 Hz, 2 cm e 4 cm     | 20 Hz, sem ruído     |
| `ideal`   | 1 Hz, sem ruído       | 20 Hz, sem ruído     |

Os dois valores do GNSS são o desvio horizontal e o vertical, em metros.

O trator sempre leva câmera: 120 graus, 1280x720, a 10 Hz, sem `<noise>` e com `<distortion>`
(`camera_k1`, `camera_k2`, `camera_k3`, `camera_p1` e `camera_p2`; padrão -0,10, 0,01, 0, 0 e 0).
Com `camera_mount:=dual` são duas câmeras de 90 graus, `camera_front_left` e `camera_front_right`.
A ponte leva a imagem para `camera_front/image_ideal`, e o nó `camera_distortion` publica
`camera_front/image_raw`. A pickup não tem câmera.

A textura da tag (o argumento `tag_texture` do xacro, o PNG que a função `tag_texture()` grava) vai
num link `tag_rear_texture`, filho de `tag_rear` por junta fixa: uma caixa de 1 mm logo à frente do
quadrado da descrição, e o único visual com `albedo_map`. Os argumentos `tag_family`, `tag_id`
e `tag_size` vão para a descrição.

A pose real de cada modelo sai do plugin `gz-sim-pose-publisher-system` em `/model/<nome>/pose` e
a ponte a entrega em `/ground_truth/<nome>/pose` (`geometry_msgs/msg/PoseStamped`).

### Memória compartilhada para as imagens

O quadro da câmera tem 2,7 MB. No padrão do Fast DDS ele não cabe no segmento de memória
compartilhada e cai para o UDP, que perde fragmentos: a imagem chegava a 5 a 7 Hz em vez de 10 Hz.
O perfil `agri_gz_bringup/config/fastdds_shm.xml` dá ao segmento 64 MB. O `convoy.launch.py` o
aplica em `FASTRTPS_DEFAULT_PROFILES_FILE`, salvo se a variável já estiver definida; o teste de
simulação do `autonomous_tractor_gz` a define pelo CMake. Um nó que assine a imagem fora do launch
precisa da mesma variável.

### Fator de tempo real

O `measure_rtf.py` sobe o comboio sem GUI com `followers:=4` (1 líder e 4 tratores com câmera) e
divide o tempo simulado do `/clock` pelo tempo de relógio, contados a partir do primeiro `/clock`:

```bash
ros2 run agri_gz_bringup measure_rtf.py --seconds 60
```

Medida em 2026-11-21, no mundo `citrus_farm.sdf`, na GPU NVIDIA GeForce RTX 3060 (driver
595.99.02), com a renderização sem tela pelo EGL, sem `rectify_node` nem `apriltag_node`:

- fator de tempo real: 0,54 (32,67 s simulados em 60,00 s de relógio)

## Adicionando um veículo

Copie `autonomous_tractor_gz/` para `<veiculo>_gz/` e aponte-o para o `<veiculo>_description`
correspondente (em `robot_description`). Três coisas não são copiadas:

- os mundos, que ficam em `agri_gz_worlds/` — por isso é um pacote próprio;
- o launch, que fica em `agri_gz_bringup/` — a cópia guarda só o wrapper que dá nome ao veículo, e o
  wrapper tem uma dúzia de linhas;
- o comportamento dos sensores, que fica em `agri_gz_bringup/urdf/sensors.gz.xacro` — uma segunda
  máquina com o mesmo lidar não deve carregar uma segunda opinião sobre o ruído dele.

Mantenha os três nomes de arquivo que os launches genéricos procuram: `urdf/<veiculo>.gz.xacro`,
`config/bridge.yaml` e `config/spawn.yaml` (o `z:` dele é a altura de onde o veículo cai). O veículo
também precisa do `config/<veiculo>.yaml` no `vehicle_control`, com os controladores dele. Depois de
compilar (`scripts/compile.sh <veiculo>_gz` na raiz do workspace), o veículo pode ser criado sozinho
(`sim.launch.py robot:=<veiculo>`) ou assumir qualquer papel num comboio (`leader:=` / `follower:=`),
em todo projeto.

## Por que as tags do Gazebo estão aqui e não na descrição

O `autonomous_tractor_gz/urdf/autonomous_tractor.gz.xacro` inclui a descrição pura do
`autonomous_tractor_description` e acrescenta os blocos `<gazebo>` — plugins, atrito, ruído de
sensor. A dependência aponta para um lado só: `autonomous_tractor_gz` →
`autonomous_tractor_description`, nunca o contrário. Um pacote de descrição que depende do Gazebo
não pode mais ser usado só com o rviz, com o MoveIt ou na máquina real.

## Duas armadilhas em que este workspace já caiu

**O tópico relativo de um plugin não tem o escopo do modelo.** `<topic>cmd_vel</topic>` dentro de um
plugin publica em `/cmd_vel`, na raiz, e não em `/model/<nome>/cmd_vel` — então duas máquinas num
mundo andam pelo mesmo tópico e publicam a mesma odometria. Todo tópico no
`autonomous_tractor.gz.xacro` é, por isso, escrito por extenso, começando com `/model/<nome>/`, e o
`bridge.yaml` é um template com `{name}` para poder nomear os mesmos.

**O ruído horizontal de um GNSS é em graus.** O Gazebo o soma à latitude e à longitude como estão,
então `<stddev>1.0</stddev>` não é um metro de dispersão, mas um grau — 111 km. O macro `gz_navsat`
recebe metros e divide; o sintoma, se ele for contornado, é um fix que parece plausível e passeia
por outros estados. Um mundo também precisa de `<spherical_coordinates>`, ou não há datum contra o
qual converter: todos estes mundos levam o mesmo, Bebedouro/SP.
