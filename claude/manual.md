# Manual do Agrobot (Webots + Nav2 + VSLAM)

Resumo de como o projeto funciona hoje: como rodar, o que cada parte faz e por quê.
Não é um histórico de alterações; o histórico está no `git log` e nos PRs.

- **Simulador oficial:** Webots (pacote `agrobot_webots`).
- **Localização oficial:** VSLAM com RTAB-Map e a câmera RGB-D (`ekf:=true vslam:=true`).
- **Ambiente:** ROS 2 Jazzy no WSL2 (Ubuntu 24.04), Webots R2025a instalado no Windows
  (`WEBOTS_HOME=/mnt/c/Program Files/Webots`), `webots_ros2_driver` 2025.0.0, Nav2 1.3.x,
  rtabmap_ros 0.23.

## 1. Instalar e compilar

```bash
sudo apt install ros-jazzy-webots-ros2 ros-jazzy-navigation2 ros-jazzy-rviz2 \
                 ros-jazzy-robot-state-publisher ros-jazzy-robot-localization \
                 ros-jazzy-rtabmap-ros ros-jazzy-slam-toolbox
cd ~/projects/ros2_gazebo_mestrado
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select agrobot_description agrobot_webots
source install/setup.bash
```

- Recompile depois de mexer em qualquer arquivo do `agrobot_webots`: os launches leem as
  cópias instaladas (`install/agrobot_webots/share/...`). Inclua o `agrobot_description`
  quando o URDF mudar.
- Se o `colcon build` falhar com `File exists` (por exemplo em `meshes/chassis.stl`) ou ao
  trocar de branch: `rm -rf build/agrobot_webots install/agrobot_webots` e compile de novo.
  O colcon não apaga links de arquivos que saíram do pacote.
- Se um launch foi morto à força, sobram nós do Nav2 que atrapalham o próximo:
  `pkill -9 -f nav2_` antes de rodar de novo.

## 2. Como rodar

Tudo sai de um launch:

```bash
ros2 launch agrobot_webots simulation.launch.py [argumentos]
```

Ele abre o Webots com o mundo, o driver do robô, o `robot_state_publisher`, o Nav2 e o RViz.
Fechar o Webots encerra o launch inteiro.

| Argumento | Padrão | O que faz |
|---|---|---|
| `world` | `obstacle_arena.wbt` | Mundo em `agrobot_webots/worlds/` (`plantation_field.wbt` é a plantação) |
| `nav` | `true` | Sobe o Nav2 |
| `rviz` | `true` | Abre o RViz (`nav.rviz`, ou `nav_vslam.rviz` com `vslam:=true`, `nav_slam.rviz` com `slam:=true`) |
| `ekf` | `false` | EKF do `robot_localization` (rodas + IMU) publica `/odom` e `odom → base_link`. Obrigatório para o VSLAM |
| `camera` | `true` | Câmera RGB-D do VSLAM. `false` alivia a CPU nas rodadas sem VSLAM |
| `vslam` | `false` | RTAB-Map publica `map → odom` e o mapa. Exige `ekf:=true` e `camera:=true` (o launch para com erro se faltar) |
| `vslam_localization` | `false` | Com `vslam:=true`: localiza num mapa salvo em vez de mapear |
| `vslam_db` | `dados/vslam/<mundo>.db` | Banco do RTAB-Map. **O mapeamento apaga o banco ao começar** |
| `rtabmap_viz` | `false` | Abre a janela do `rtabmap_viz` |
| `data_camera` | `false` | Câmera RGB de aquisição de dados (seção 6) |
| `data_record` | `true` | Com `data_camera:=true`, grava as fotos com a pose de cada uma (seção 6). `false` só mostra as imagens |
| `data_dir` | `dados/fotos/` | Pasta das fotos gravadas |
| `data_min_distance` | `0.3` | Distância mínima (m) entre duas fotos gravadas |
| `waypoints` | vazio | Missão de waypoints: nome de uma lista em `config/waypoints/`, caminho de um `.yaml`, ou `rviz` para clicar os pontos |
| `slam` | `false` | SLAM por LiDAR (slam_toolbox). Só para comparação (seção 8) |

### Usos mais comuns

```bash
# Mapear a plantação com o VSLAM (grava dados/vslam/plantation_field.db)
ros2 launch agrobot_webots simulation.launch.py world:=plantation_field.wbt ekf:=true vslam:=true

# Localizar no mapa salvo (o arquivo não é alterado; roda numa cópia)
ros2 launch agrobot_webots simulation.launch.py world:=plantation_field.wbt ekf:=true vslam:=true \
    vslam_localization:=true

# Missão de waypoints de um caso de estudo
ros2 launch agrobot_webots simulation.launch.py world:=plantation_field.wbt ekf:=true vslam:=true \
    waypoints:=caso3_plantacao_serpentina

# Com a câmera de aquisição ligada, gravando as fotos
ros2 launch agrobot_webots simulation.launch.py world:=plantation_field.wbt ekf:=true vslam:=true \
    data_camera:=true
```

- **Onde ficam os dados:** tudo o que a simulação grava fica em `dados/`, na raiz do projeto
  (fora do git): `dados/missions/` (CSVs das missões), `dados/fotos/` (câmera de aquisição) e
  `dados/vslam/` (bancos do RTAB-Map). O launch acha a raiz pelo link do
  `colcon build --symlink-install`; sem ele, usa a pasta que tem o `install/`.
- **Objetivo avulso:** no RViz, ferramenta **Nav2 Goal**. Sem SLAM/VSLAM as coordenadas
  estão em `odom`; com `vslam:=true`, em `map`. Os dois nascem na pose inicial do robô, não
  no centro do mundo.
- **Durante o mapeamento, deixe o RViz aberto:** a grade `/rtabmap/map` só é montada
  enquanto alguém a assina.
- Para guardar um mapa, copie o `.db` antes de mapear de novo o mesmo mundo, ou use
  `vslam_db:=...`.
- A localização exige que o robô comece onde começou o mapeamento (os mundos já fixam essa
  pose).

## 3. O robô no Webots

Skid-steer de 4 rodas, raio 0,22 m, bitola 0,96 m, footprint 1,2 × 1,12 m. Modelo em
`agrobot_webots/protos/agrobot.proto`; frames no URDF `agrobot_description/urdf/agrobot.urdf`
(o PROTO e o URDF têm que bater nas poses dos sensores, porque o Nav2 e o RTAB-Map projetam
os dados pelo TF).

| Sensor | Onde (em relação ao `base_link`) | Tópico | Para quê |
|---|---|---|---|
| LiDAR frontal `front_lidar` | (0,65; 0; -0,10), ~13 cm do chão, 172°, 300 raios, 0,1–8 m | `/scan` | Obstáculos do Nav2 e ICP do RTAB-Map |
| IMU (`InertialUnit`, `Gyro`, `Accelerometer`) | (0; 0; 0,35) | `/imu/data` | Yaw do EKF |
| Encoders das rodas | juntas das rodas | `/wheel/odom` | Velocidade do EKF |
| Câmera RGB-D do VSLAM (modelo Intel RealSense D455) | (0,45; 0; 0,38), 15° para baixo, 424 × 240, 87°, 0,5–6 m, ~10 Hz | `/vslam_camera/color/image_raw`, `/vslam_camera/aligned_depth_to_color/image_raw`, `camera_info` | Entrada do RTAB-Map |
| Câmera RGB de aquisição | (0; 0; 0,66), ~0,90 m do chão, olhando para a esquerda, 1280 × 960, 90°, 1 foto a cada ~1 s | `/data_camera/image_raw`, `/data_camera/camera_info` | Monitorização das plantas |
| Pose verdadeira (Supervisor) | `base_link` | `/ground_truth/odom` | Só para medir erro; nenhum estimador usa |

Por quê:
- **LiDAR baixo e próprio do Webots** (`front_lidar_link`): o `lidar_link` antigo fica a
  0,57 m do chão e não vê caixas de 0,2–0,3 m. A 8 cm o plano batia no chão quando o robô
  inclinava na plantação e o Nav2 via obstáculos falsos; a 13 cm isso quase some.
- **Yaw pelo IMU, distância pelas rodas:** no skid-steer as rodas escorregam nas curvas e o
  yaw das rodas erra a cada giro.
- **Links novos em vez de mexer nos antigos** (`lidar_link`, `camera_link`): o Gazebo usa o
  mesmo URDF.

### O driver (`agrobot_webots/agrobot_driver.py`)

Plugin Python do `webots_ros2_driver` (carregado pelo `<plugin>` do URDF). A cada passo
de 32 ms ele: converte `/cmd_vel` em velocidade das rodas; publica `/clock`, `/odom` (sem
EKF), `/wheel/odom`, `/imu/data`, `/scan`, `/joint_states`, as câmeras e o
`/ground_truth/odom`.

- **Por que o driver publica os sensores, e não o `webots_ros2_driver`:** os publicadores
  dele carimbam com o relógio do próprio nó, que fica parado ou atrasado. O Nav2 descartava
  todos os scans em silêncio, e o RTAB-Map casa cor, profundidade e odometria pelo carimbo.
  Tudo aqui sai com o tempo do Webots (`robot.getTime()`), no passo em que o dado foi feito;
  os devices correspondentes estão desligados no URDF (`<ros><enabled>false</enabled>`).
- **`/clock` publicado pelo driver:** sem ele, os nós com `use_sim_time` (o EKF, por exemplo)
  ficam parados em 0.
- **`wheelSeparationScale` 1,33:** o skid-steer gira menos que a geometria prevê; tratar a
  bitola como mais larga faz o giro comandado bater com o real.
- **Câmeras por contador inteiro de ms:** evita publicar uma imagem de um passo diferente do
  carimbo.
- As propriedades do plugin (`publishOdom`, `vslamCameraPeriod`, `dataCameraPeriod`) vêm do
  URDF, não de parâmetros ROS. Por isso o launch grava uma cópia do URDF com os valores de
  `ekf`, `camera` e `data_camera`.

### Malha e visual

`agrobot_webots/meshes/chassis.stl` é gerada por `scripts/fix_chassis_mesh.py` a partir do
CAD em `agrobot_description/meshes/` (motores alinhados às rodas). Se o CAD mudar, rode o
script de novo. Eixos, hastes, mastro e garras das rodas são só visuais (sem massa nem
colisão). A colisão do corpo é uma caixa de 1,2 × 0,8 × 0,35 m, maior que o quadro visível;
foi mantida para não mudar os resultados.

## 4. Mundos

| Mundo | O que é | Uso |
|---|---|---|
| `obstacle_arena.wbt` | Arena 5 × 5 m com paredes, 4 caixotes e 3 caixas. Robô em (-1,0; -0,3) do mundo | Testes rápidos; controle negativo do VSLAM (paredes e chão lisos) |
| `plantation_field.wbt` | 6 fileiras de plantas (milho) de 9 m, entrelinhas de 1,8 m, terreno ondulado com camalhões, buracos, torrões, pedras e galhos; atrito 0,7. Robô em (-6; 0), no começo da entrelinha do meio | Mundo principal de teste em terreno não estruturado |

A plantação é gerada por `scripts/generate_plantation_field.py` (semente fixa, `--seed`,
`--roughness`). **Não edite o `.wbt` à mão**; mude o script e gere de novo. No `odom`, as
fileiras vão de x = 1,5 a 10,5 e as entrelinhas ficam em y = 0, ±1,8, ±3,6.

## 5. Localização e navegação

```
rodas (/wheel/odom) + IMU (/imu/data) ─► EKF ─► /odom, TF odom → base_link
câmera RGB-D + /scan + odom do EKF ─► RTAB-Map ─► TF map → odom, /rtabmap/map
Nav2: planeja em map, controla em odom ─► /cmd_vel ─► driver
```

### EKF (`config/ekf.yaml`, `ekf:=true`)

`robot_localization` a 30 Hz, 2D. Usa vx/vy das rodas e yaw + velocidade de yaw do IMU.
- O yaw das rodas fica de fora (escorregamento) e a aceleração também (dupla integração
  piora a posição).
- `process_noise_covariance` de x/y em 0 e covariância das rodas proporcional ao movimento:
  a elipse cresce com o deslocamento e na direção dele, e não com o tempo parado.
- O frame `odom` deriva por definição; nenhuma entrada do EKF mede posição. Quem corrige é o
  VSLAM.
- Na simulação o EKF está favorecido: o `InertialUnit` não tem ruído nem deriva. Para uma
  comparação justa com o robô real, ligar ruído nos sensores do PROTO.

### VSLAM (`config/rtabmap.yaml`, `vslam:=true`)

`rgbd_sync` + `rtabmap` no namespace `rtabmap`, a 1 Hz, 3DoF (x, y, yaw). Escolhas e motivos:
- **Odometria do EKF, sem odometria visual:** economiza CPU; o RTAB-Map só corrige o
  `map → odom`.
- **Um só dono de `map → odom`:** dois publicando fazem o robô pular entre duas poses.
- **`Reg/Strategy` 2 (visual + ICP com o `/scan`) e `Optimizer/Robust` (Vertigo):** a
  textura do solo se repete a cada 2 m e gerava fechamentos falsos; o ICP e a otimização
  robusta os derrubam.
- **Fechamento global desligado (`Rtabmap/LoopThr` e `RGBD/AggressiveLoopThr` em 1,01):** as
  entrelinhas são iguais para a câmera e para o LiDAR (1,8 m de deslocamento dá o mesmo
  scan). Só valem fechamentos por proximidade, perto da pose do EKF (câmera até 1 m e 45°,
  ICP corrige até 0,2 m). Funciona enquanto a deriva do EKF entre dois fechamentos ficar
  abaixo de ~0,8 m. Para religar a detecção global, volte os dois valores para `0.11` e
  `0.05`.
- **Localização:** roda numa cópia do banco (`Mem/IncrementalMemory` false) com
  `RGBD/StartAtOrigin`, então o robô começa no primeiro nó do mapa.
- Na plantação, as passadas repetidas são em sentidos opostos e a câmera não reconhece o
  lugar visto de costas; os fechamentos vêm sobretudo do LiDAR. Por isso o erro do VSLAM fica
  perto do EKF sozinho nesses casos.

### Nav2 (`config/nav2_params.yaml`, `launch/navigation.launch.py`)

- Launch próprio, e não o `nav2_bringup`: no Jazzy ele sobe o `docking_server`, que falha
  sem plugins e derruba o lifecycle.
- Com `vslam:=true` (ou `slam:=true`), o planner, o bt_navigator e o costmap global ficam em
  `map`; o controlador e o costmap local ficam sempre em `odom` (REP-105).
- Regulated Pure Pursuit com giro no lugar, NavFn com `allow_unknown`, velocidade até
  0,5 m/s e 1,0 rad/s, tolerância 0,15 m / 0,3 rad.
- Footprint 1,2 × 1,12 m (as rodas saem do chassi) e inflação 0,7 m.
- `min_obstacle_height: -1.0` na camada e na fonte: o LiDAR fica abaixo do `base_link`.
- Costmaps sem camada estática, janela global de 24 × 24 m centrada no robô: objetivos a
  mais de ~12 m são recusados.
- `default_server_timeout` 1000 ms: com câmera e RTAB-Map a CPU fica cheia e o planner
  demora a aceitar o pedido.

## 6. Câmera RGB de aquisição de dados (`data_camera:=true`)

Câmera separada da RGB-D, só para monitorizar as plantas; não entra no EKF nem no RTAB-Map.
Desligada por padrão, para não pesar nas rodadas de navegação.

- Olha para a esquerda (+y), no centro do robô a ~0,90 m do chão. Com o robô no meio da
  entrelinha, a fileira fica a 0,9 m e cada foto cobre ~1,80 m de fileira por 1,35 m de
  altura (~0,23 a 1,58 m do chão), ~1,4 mm por pixel. O robô monitora **um lado por
  passada**.
- 1280 × 960 (4:3 para caber a planta inteira na altura), uma foto a cada 1024 ms: a
  0,4 m/s dá uma foto a cada ~0,41 m e ~77% de sobreposição. Cada imagem tem 4,9 MB.
- Carimbo do passo em que a imagem foi feita, para georreferenciar cada foto pelo TF
  `map → data_camera_optical_frame`.
- Ligar e desligar com a simulação rodando:
  ```bash
  ros2 service call /data_camera/enable std_srvs/srv/SetBool "{data: false}"
  ros2 service call /data_camera/enable std_srvs/srv/SetBool "{data: true}"
  ```
- No RViz, o painel "Data Camera" vem desmarcado.

**Gravação das fotos (nó `data_recorder`, sobe com `data_camera:=true`; `data_record:=false`
desliga).** Uma pasta por execução,
`dados/fotos/<mundo>_<modo>_<data>/` (o modo é o mesmo rótulo dos CSVs das
missões, como `ekf+vslam`), com:
- `000001.png`, `000002.png`, ...: as fotos em RGB, sem perdas (~2 a 3 MB cada);
- `fotos.csv`: uma linha por foto, com o tempo de simulação, a pose **estimada** da câmera
  (`est_*`, pelo TF `map → data_camera_optical_frame` no carimbo da foto; `odom` sem
  SLAM/VSLAM), a pose **verdadeira** (`real_*`, do `/ground_truth/odom` do mesmo carimbo,
  trazida para o frame da partida como no `waypoint_mission`) e o erro entre as duas
  (`erro_xy_m`, `erro_z_m`, `erro_yaw_rad`). Cada pose tem x, y, z do centro óptico,
  roll/pitch/yaw do `data_camera_link` (yaw ≈ yaw do robô + 90°) e o quaternion do frame
  óptico, para projetar pontos na imagem;
- `camera_info.yaml`: intrínsecos, no formato do `camera_calibration_parsers`.

Regras do gravador:
- Só grava uma foto se a câmera andou pelo menos `data_min_distance` (0,3 m) desde a última,
  então o robô parado (nas paradas dos waypoints, por exemplo) não enche o disco.
- Espera até 1 s de simulação pelo TF do carimbo da foto. Se não chegar, grava a foto com as
  colunas `est_*` vazias e avisa no log.
- A pose verdadeira é relativa à primeira mensagem do `/ground_truth/odom` que o nó recebe; por
  isso ele tem que subir junto com a simulação (o launch faz isso).
- Ainda não está ligado às missões de waypoints: grava enquanto a câmera estiver ligada e o
  robô andando. Para gravar só um trecho, use o serviço `/data_camera/enable`.

A câmera e o gravador ainda não foram rodados no Webots (só conferidos fora do simulador).

## 7. Missões de waypoints e casos de estudo

O nó `waypoint_mission` manda a lista inteira para o `FollowWaypoints` do Nav2, com parada
de 3 s em cada ponto (`waypoint_pause_duration`; é ali que entra a coleta de dados) e pula
um waypoint que falha. Sem `yaw` no YAML, o robô chega olhando para o próximo ponto. Se o
`world:` do YAML não bater com o do launch, a missão é cancelada.

| Lista | Mundo | Rota | O que testa |
|---|---|---|---|
| `caso1_arena_circuito` | arena | ~10 m em volta do caixote do centro | Ambiente fechado; controle negativo do VSLAM |
| `caso2_plantacao_ida_volta` | plantação | ~26 m: entrelinha y = 0 e volta pela y = 1,8 | Fileiras repetitivas, duas cabeceiras |
| `caso3_plantacao_serpentina` | plantação | ~51 m por quatro entrelinhas | Deriva acumulada, aliasing entre entrelinhas |

As coordenadas estão no frame da partida, então a mesma lista vale com e sem VSLAM.

**Waypoints clicados (`waypoints:=rviz`):** no RViz, ferramenta **Publish Point**, um
clique por ponto; depois `ros2 service call /waypoint_mission/start std_srvs/srv/Trigger`.
Também há `undo`, `clear` e `save` (grava a lista em `dados/missions/` no formato
dos casos, para rodar de novo com `waypoints:=<arquivo>`).

**Resultados:** cada rodada grava em `dados/missions/` um
`<caso>_<modo>_<data>_waypoints.csv` (um por waypoint) e um `_trajetoria.csv` (pose estimada
e verdadeira a 5 Hz). O modo é `ekf`, `ekf+vslam`, `ekf+vslam-loc` ou `ekf+slam`. Para a
tabela (atingidos, tempo, distância, erro final, ATE, erro máximo):

```bash
python3 agrobot_webots/scripts/summarize_missions.py   # lê dados/missions/; ou passe pastas/CSVs
```

Compare pelo ATE e pelo erro máximo, não pelo erro final: o robô chega perto de onde ele
*acha* que o alvo está, mesmo com a estimativa errada. Rode cada caso várias vezes; com a
CPU no limite os resultados variam entre rodadas.

## 8. Legado e comparação

- **SLAM por LiDAR (`slam:=true`, slam_toolbox):** não é mais usado nem corrigido; fica só
  para comparação nos resultados. Na plantação ele perde o yaw nas cabeceiras e o robô vai
  para a entrelinha errada. Com `vslam:=true slam:=true`, o slam_toolbox roda sem TF (mapa em
  `/slam_toolbox/map`) e o RTAB-Map continua dono de `map → odom`.
- **`goal_navigation.launch.py` (`goal_navigator`):** vai até um objetivo do RViz (2D Goal
  Pose) sem Nav2 nem mapa, só desviando do que está à frente. Funciona na arena, não entra
  nas entrelinhas da plantação.
- **Gazebo (`agrobot_gazebo`, `agrobot_control`):** versão antiga, não é mais usada nem
  mantida.

## 9. Limites conhecidos

- Quase tudo foi validado na nuvem Linux, headless e sem GPU; os testes no Windows/WSL e no
  RViz na tela ainda são poucos. A câmera de aquisição ainda não rodou no Webots.
- A profundidade da RGB-D e o IMU não têm ruído na simulação.
- O Nav2 às vezes não acha caminho na travessia da cabeceira leste da plantação
  ("Failed to create plan with tolerance of 0.3") e perde waypoints ali.
- Ao girar no lugar na arena, a quina do robô pode empurrar a `cardboard box(2)`.
