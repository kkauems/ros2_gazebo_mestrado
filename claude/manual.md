# Manual do Agrobot (Webots + Nav2 + VSLAM)

Resumo de como o projeto funciona hoje: como rodar, o que cada parte faz e por quê.
Não é um histórico de alterações; o histórico está no `git log` e nos PRs.

- **Simulador oficial:** Webots (pacote `agrobot_webots`).
- **Localização oficial:** VSLAM com RTAB-Map, a câmera RGB-D e o LiDAR 3D do topo
  (`ekf:=true vslam:=true`).
- **Segurança:** o LiDAR de segurança (2D, 360°, 15 cm do chão) e o `collision_monitor` do Nav2
  param ou reduzem o robô antes de qualquer comando chegar às rodas (`safety:=true`, padrão).
- **Ambiente:** ROS 2 Jazzy no WSL2 (Ubuntu 24.04), Webots R2025a instalado no Windows
  (`WEBOTS_HOME=/mnt/c/Program Files/Webots`), `webots_ros2_driver` 2025.0.0, Nav2 1.3.x,
  rtabmap_ros 0.23.

## 1. Instalar e compilar

```bash
sudo apt install ros-jazzy-webots-ros2 ros-jazzy-navigation2 ros-jazzy-rviz2 \
                 ros-jazzy-robot-state-publisher ros-jazzy-robot-localization \
                 ros-jazzy-rtabmap-ros ros-jazzy-slam-toolbox \
                 ros-jazzy-joy ros-jazzy-teleop-twist-joy \
                 ros-jazzy-rosbridge-suite python3-opencv
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

Ele abre o Webots com o mundo, o driver do robô, o `robot_state_publisher`, a camada de
segurança (`collision_monitor`), o Nav2 e o RViz.
Fechar o Webots encerra o launch inteiro.

| Argumento | Padrão | O que faz |
|---|---|---|
| `world` | `obstacle_arena.wbt` | Mundo em `agrobot_webots/worlds/` (`plantation_field.wbt` é a plantação) |
| `nav` | `true` | Sobe o Nav2 |
| `rviz` | `true` | Abre o RViz (`nav.rviz`, ou `nav_vslam.rviz` com `vslam:=true`, `nav_slam.rviz` com `slam:=true`) |
| `ekf` | `false` | EKF do `robot_localization` (rodas + IMU) publica `/odom` e `odom → base_link`. Obrigatório para o VSLAM |
| `camera` | `true` | Câmera RGB-D do VSLAM. `false` alivia a CPU nas rodadas sem VSLAM |
| `vslam` | `false` | RTAB-Map publica `map → odom` e o mapa. Exige `ekf:=true`, `camera:=true` e `lidar3d:=true` (o launch para com erro se faltar) |
| `lidar3d` | `true` | LiDAR 3D do topo (`/lidar3d/points`), usado pelo RTAB-Map. `false` alivia a CPU nas rodadas sem VSLAM |
| `safety` | `true` | Camada de segurança: dois `collision_monitor` (limite e parada) com o LiDAR de segurança entre todos os comandos e as rodas (seção 5). `false` só para comparar: o robô anda sem parada de segurança |
| `vslam_localization` | `false` | Com `vslam:=true`: localiza num mapa salvo em vez de mapear |
| `vslam_db` | `dados/vslam/<mundo>.db` | Banco do RTAB-Map (o mapa). Um nome sem pasta (`vslam_db:=olival_mapa1`) vai para `dados/vslam/<nome>.db`. O mapeamento começa num banco vazio; um mapa que já existia com o mesmo nome vira `<nome>_<data>.db` |
| `rtabmap_viz` | `false` | Abre a janela do `rtabmap_viz` |
| `stable_map` | `true` | Com `vslam:=true`: a grade `/rtabmap/map` é montada pelo nó `stable_map`, que compara cada passada com o mapa já feito (seção 5). `false`: a grade do próprio rtabmap, como antes (para comparar) |
| `data_camera` | `false` | Câmera RGB de aquisição de dados (seção 6) |
| `data_record` | `true` | Com `data_camera:=true`, grava as fotos com a pose de cada uma (seção 6). `false` só mostra as imagens |
| `data_dir` | `dados/fotos/` | Pasta das fotos gravadas |
| `data_min_distance` | `0.3` | Distância mínima (m) entre duas fotos gravadas |
| `waypoints` | vazio | Missão de waypoints: nome de uma lista em `config/waypoints/`, caminho de um `.yaml`, ou `rviz` para clicar os pontos |
| `waypoint_check` | `true` | Recusa waypoints onde o robô não cabe, não gira ou não chega (seção 7) |
| `teleop` | vazio | Teleoperação: `joy` (joystick) ou `keyboard` (teclado, num terminal próprio). O operador tem prioridade sobre o Nav2 (seção 5) |
| `slip_check` | `true` | Checagem de patinagem das rodas pela odometria visual (seção 5). Só age com `ekf:=true` e `camera:=true` |
| `row_heading` | `auto` | Rumo pelas fileiras: o yaw do EKF vem do ângulo das fileiras no LiDAR 3D, no lugar do yaw do IMU (seção 5). `auto` liga só no olival; `true`/`false` forçam. Só com `ekf:=true` e `lidar3d:=true`; acrescenta `-rumo` ao rótulo dos CSVs |
| `wheel_radius_scale` | `auto` | Raio efetivo / nominal na odometria das rodas (seção 5). `auto`: o de `config/wheel_calibration.yaml` para o mundo e a semente (hoje só o olival, sementes 101 e 303), 1,0 se não houver; um número força. Diferente de 1, acrescenta `-cal` ao rótulo |
| `lateral_correction` | `auto` | Correção do escorregamento lateral no EKF (seção 5): `imu` (pela inclinação), `rows` (pela posição das fileiras) ou `none`. `auto`: `imu` no olival quando há calibração para a semente, `none` no resto. Acrescenta `-incl` ou `-lat` ao rótulo |
| `wheel_calibration` | `false` | Rodada de calibração das rodas (seção 5): grava `dados/calibracao/<mundo>_semente<N>.yaml` |
| `slam` | `false` | SLAM por LiDAR (slam_toolbox). Só para comparação (seção 10) |
| `sensor_noise` | `true` | Ruído realista na IMU, nos encoders, nos dois LiDARs e na câmera RGB-D (seção 3). `false`: sensores ideais do Webots, com `-ideal` no rótulo dos CSVs |
| `noise_seed` | `0` | Semente do ruído. `0` sorteia uma nova (sai no log do driver); outro valor repete o ruído de uma rodada |
| `operator` | `false` | Interface web do operador em http://localhost:8080 (seção 8) |
| `operator_port`, `rosbridge_port` | `8080`, `9090` | Portas da página e do rosbridge (WebSocket) |

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

# Mapear conduzindo o robô (no outro terminal: ros2 run agrobot_webots teleop_keyboard)
ros2 launch agrobot_webots simulation.launch.py world:=olive_grove.wbt ekf:=true vslam:=true \
    vslam_db:=olival_mapa1 nav:=false teleop:=keyboard

# Com a câmera de aquisição ligada, gravando as fotos
ros2 launch agrobot_webots simulation.launch.py world:=plantation_field.wbt ekf:=true vslam:=true \
    data_camera:=true

# Operar pelo navegador (http://localhost:8080 no Windows): mapa, waypoints, varreduras,
# joystick, câmeras e as fotos gravadas
ros2 launch agrobot_webots simulation.launch.py world:=olive_grove.wbt ekf:=true vslam:=true \
    data_camera:=true operator:=true rviz:=false
```

- **Onde ficam os dados:** tudo o que a simulação grava fica em `dados/`, na raiz do projeto
  (fora do git): `dados/missions/` (CSVs das missões), `dados/fotos/` (câmera de aquisição) e
  `dados/vslam/` (bancos do RTAB-Map). O launch acha a raiz pelo link do
  `colcon build --symlink-install`; sem ele, usa a pasta que tem o `install/`.
- **Objetivo avulso:** no RViz, ferramenta **Nav2 Goal**. Sem SLAM/VSLAM as coordenadas
  estão em `odom`; com `vslam:=true`, em `map`. Os dois nascem na pose inicial do robô, não
  no centro do mundo.
- A grade `/rtabmap/map` é montada o tempo todo pelo nó `stable_map` (com `stable_map:=false`,
  a do rtabmap só é montada enquanto alguém a assina, como o RViz).
- **O mapa é salvo quando a simulação fecha** (feche o Webots ou Ctrl-C no launch): o
  rtabmap grava o banco ao sair e o launch espera até 2 min por ele. A linha "Mapa salvo em
  ..." no log confirma. Dê nomes aos mapas com `vslam_db:=<nome>`.
- A localização exige que o robô comece onde começou o mapeamento (os mundos já fixam essa
  pose).

## 3. O robô no Webots

Skid-steer de 4 rodas, raio 0,22 m, bitola 0,96 m, footprint 1,2 × 1,12 m. Modelo em
`agrobot_webots/protos/agrobot.proto`; frames no URDF `agrobot_description/urdf/agrobot.urdf`
(o PROTO e o URDF têm que bater nas poses dos sensores, porque o Nav2 e o RTAB-Map projetam
os dados pelo TF).

| Sensor | Onde (em relação ao `base_link`) | Tópico | Para quê |
|---|---|---|---|
| LiDAR de segurança `front_lidar` (modelo 3irobotix Delta-2A) | (0,65; 0; -0,09), na frente, 15 cm do chão, 360°, 360 raios (1°), 0,13–8 m, 6,25 Hz | `/scan` | Parada de segurança (`collision_monitor`) e obstáculos dos costmaps do Nav2 |
| LiDAR 3D `lidar3d` (modelo RoboSense RS-Helios-16P) | (0; 0; 0,80), no topo do mastro, ~1,04 m do chão; 16 canais de -15° a +15° (2° entre canais), 360° com 0,2° (1800 colunas), 0,2–150 m, ~10 Hz | `/lidar3d/points` (nuvem, frame `lidar3d_link`) | ICP dos fechamentos de laço e grade do RTAB-Map |
| IMU (`InertialUnit`, `Gyro`, `Accelerometer`) | (0; 0; 0,35) | `/imu/data` | Yaw do EKF |
| Encoders das rodas | juntas das rodas | `/wheel/odom` | Velocidade do EKF |
| Câmera RGB-D do VSLAM (modelo Intel RealSense D455) | (0,45; 0; 0,38), 15° para baixo, 424 × 240, 87°, 0,5–6 m, ~10 Hz | `/vslam_camera/color/image_raw`, `/vslam_camera/aligned_depth_to_color/image_raw`, `camera_info` | Entrada do RTAB-Map |
| Câmera RGB de aquisição | (0; 0; 0,66), ~0,90 m do chão, olhando para a esquerda, 1280 × 960, 90°, 1 foto a cada ~1 s | `/data_camera/image_raw`, `/data_camera/camera_info` | Monitorização das plantas |
| Pose verdadeira (Supervisor) | `base_link` | `/ground_truth/odom` | Só para medir erro; nenhum estimador usa |

Por quê:
- **Dois LiDARs com papéis separados** (pedido do orientador): o de segurança, baixo, vê
  pernas de pessoas, pedras e caixas perto do chão e só serve para não bater; o 3D, no alto,
  vê troncos, copas e o relevo a dezenas de metros e só serve para a localização.
- **LiDAR de segurança a 15 cm** (`front_lidar_link`, na frente): o `lidar_link` antigo fica
  a 0,57 m do chão e não vê caixas de 0,2–0,3 m nem pernas. A 8 cm o plano batia no chão
  quando o robô inclinava na plantação e o Nav2 via obstáculos falsos; a 13–15 cm isso quase
  some. Ele gira 360°, mas, na frente do robô, para trás o plano passa por baixo do chassi
  (que fica a ~17 cm do chão) e entre as rodas: os raios que batem nas rodas e nos motores
  (dentro de uma caixa de x -0,65 a 0,62 m e y ±0,58 m no `base_link`) saem como NaN, senão o
  `collision_monitor` veria o próprio robô.
- **LiDAR 3D a 0,80 m do `base_link`:** acima de tudo no robô (mastro da câmera de aquisição
  e base de 1 cm), então nenhum raio bate nele mesmo. O canal mais baixo (-15°) só toca o
  chão a ~3,9 m: perto do robô quem vê o chão é o LiDAR de segurança.
- **Especificações dos dois modelos:** Delta-2A de 4 a 10 Hz (6,2 típico), 2 a 5 mil pontos/s,
  0,13–8 m, exatidão < 1% a 5 m, 175 g; Helios-16P 16 canais, ±15° com 2°, 0,2° a 10 Hz
  (288 mil pontos/s), até 150 m, ±2 cm, 97,5 × 100 mm, ~0,9 kg. Vêm do manual do Helios e de
  listas de LiDARs (os sites dos fornecedores não abriram na nuvem); confira com o
  datasheet antes de citar.
- **Yaw pelo IMU, distância pelas rodas:** no skid-steer as rodas escorregam nas curvas e o
  yaw das rodas erra a cada giro.
- **Links novos em vez de mexer nos antigos** (`lidar_link`, `camera_link`): o Gazebo usa o
  mesmo URDF.

### O driver (`agrobot_webots/agrobot_driver.py`)

Plugin Python do `webots_ros2_driver` (carregado pelo `<plugin>` do URDF). A cada passo
de 32 ms ele: converte `/cmd_vel` em velocidade das rodas; publica `/clock`, `/odom` (sem
EKF), `/wheel/odom`, `/imu/data`, `/scan`, `/lidar3d/points`, `/joint_states`, as câmeras
e o `/ground_truth/odom`.

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
- **Para sem comando:** se o `/cmd_vel` parar de chegar por 1 s de simulação, o robô para.
  O Nav2 e a teleoperação publicam continuamente enquanto andam; sem isso, um nó que caísse
  deixaria o robô seguindo o último comando.
- **Nuvem do LiDAR 3D:** a range image do Webots (16 × 1800 distâncias) é lida de uma vez
  com o numpy e vira pontos x, y, z pelos ângulos de cada raio; só os pontos válidos vão no
  `PointCloud2` (~14 mil por volta na plantação). Custa ~1,2 ms por volta no driver. O que
  pesa é o Webots desenhar os 360° do LiDAR 3D: na nuvem, sem GPU, a simulação ficou ~35%
  mais lenta (de 0,37 para 0,23 do tempo real, sem câmera e sem Nav2).
- **Ângulos dos raios:** num Lidar de 360° do Webots cada raio fica no meio da sua fatia (o
  primeiro em +180° − meio passo). Com `AGROBOT_LIDAR_CHECK=1` no ambiente, o driver compara
  uma vez os pontos calculados com a nuvem do próprio Webots e escreve a diferença no log
  (até 2 cm no LiDAR 3D, abaixo do ruído).
- As propriedades do plugin (`publishOdom`, `vslamCameraPeriod`, `dataCameraPeriod`,
  `safetyLidarPeriod`, `lidar3dPeriod`, `sensorNoise`, `noiseSeed`) vêm do URDF, não de
  parâmetros ROS. Por isso o launch grava uma cópia do URDF com os valores de `ekf`,
  `camera`, `data_camera`, `lidar3d`, `sensor_noise` e `noise_seed`.

### Ruído dos sensores (`agrobot_webots/sensor_noise.py`, `sensor_noise:=true`)

Os sensores do Webots são perfeitos, o que favorecia o EKF (ele segue o yaw do IMU). O driver
aplica o erro de sensores reais de baixo custo antes de publicar; os valores vêm de
datasheets e estão justificados no próprio arquivo.

| Sensor | Modelo de erro (1σ) | Referência |
|---|---|---|
| Giroscópio | ruído branco 0,01 °/s/√Hz; bias fixo 0,01 °/s (resíduo da calibração parada); bias que vaga 0,005 °/s (Gauss-Markov, 2 min); escala 1% | BMI160, MPU-6050, ICM-20948 |
| Orientação da IMU | yaw = integral do erro do giroscópio (sem magnetômetro: deriva e não volta); roll/pitch com a inclinação fixa do offset do acelerômetro + 0,1° | IMU de 6 eixos com fusão a bordo |
| Acelerômetro | ruído 200 µg/√Hz; offset 20 mg | BMI160, MPU-6050 |
| Encoders | 4096 contagens por volta; raio efetivo 1% errado, mais 0,3% de diferença entre os lados | encoder de 1024 pulsos; pneu sob carga |
| LiDAR de segurança | 1% da distância (mín. 1 cm): 5 cm a 5 m; 1% dos raios sem leitura (NaN) | 3irobotix Delta-2A (triangulação, < 1% a 5 m) |
| LiDAR 3D | 2 cm por ponto; 0,5% dos pontos perdidos | RoboSense Helios-16P (±2 cm) |
| Profundidade RGB-D | σ = z² · 0,1 px / (447 px · 0,095 m): 2,4 mm a 1 m, 3,8 cm a 4 m | Intel RealSense D455 |
| Cor RGB-D | 2 níveis de 255 | sensor CMOS de dia |

- **Deriva do yaw:** parado, mediana de ~2° em 5 min e ~4° em 9 min (até ~9° no pior
  décimo). Nos giros, o erro de escala soma ~1,8° a cada 180°. Numa entrelinha de 9 m, 4° de
  yaw viram ~0,6 m de erro lateral: quem corrige é o VSLAM.
- **Efeito medido** (caso2 na plantação, EKF+VSLAM, 3 rodadas de cada): com ruído, 15/15
  waypoints, ATE 0,16 ± 0,01 m, erro máximo médio 0,34 m e yaw até 3,4°; sem ruído, 15/15,
  ATE 0,11 ± 0,03 m, 0,21 m e 2,3°. Nenhum alarme falso de patinagem nas duas.
- **Custo:** a câmera do VSLAM passa de ~1,6 para ~3,5 ms por quadro no driver (o ruído da cor
  sai de uma tabela sorteada uma vez, lida de um ponto sorteado a cada quadro).
- **Sem magnetômetro de propósito:** no campo ele sofre com motores, cercas e implementos.
- **As variâncias publicadas** (`IMU_*_VARIANCE` e as das rodas no driver) já cobriam esses
  erros e não mudaram; a configuração do EKF também não.
- **A pose verdadeira** (`/ground_truth/odom`) e a câmera de aquisição continuam sem ruído.
- **Resultados anteriores** (passo 3 da comparação, outubro de 2026) são com sensores ideais;
  para compará-los com rodadas novas, rode as novas com `sensor_noise:=false` ou refaça as
  antigas.

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
| `olive_grove.wbt` | Olival tradicional do planalto do Douro Internacional (referência: Vilarinho dos Galegos): 65 oliveiras em grade de 7 × 7 m, encosta de 4 a 5% subindo para +y, chão irregular (ruído e montículos rasos), solo avermelhado com lascas de pedra e galhos, montes de poda, moroiços e blocos nas entrelinhas e cabeceiras, vinha, estrada de terra e tanque de rega; atrito 0,8. Robô em (-36; -3,5), na cabeceira oeste | Lugar aberto: o LiDAR vê troncos esparsos e os obstáculos; desvio nas entrelinhas; declive lateral; aquisição de dados nas fileiras |

A plantação é gerada por `scripts/generate_plantation_field.py` (semente fixa, `--seed`,
`--roughness`). **Não edite o `.wbt` à mão**; mude o script e gere de novo. No `odom`, as
fileiras vão de x = 1,5 a 10,5 e as entrelinhas ficam em y = 0, ±1,8, ±3,6.

O olival é gerado por `scripts/generate_olive_grove.py` (`--seed`, `--slope`, `--noise`; a
oliveira é o PROTO `OliveTree`, em que só o tronco colide). No `odom`, as entrelinhas ficam em y = -14,
-7, 0, 7, 14 e 21, as árvores vão de x = 4,5 a 67,5 e os vãos para atravessar as fileiras
(subir ou descer a encosta) ficam em x = 8, 15, 22, 29... Pedras pequenas só na linha das
árvores e lascas de 2 a 4 cm no resto: maiores que isso, inclinam o robô e o plano do LiDAR
(~13 cm do chão) bate no chão à frente. Pelo mesmo motivo o relevo irregular é largo e raso
(calombos de poucos metros com mais de 2 a 3 cm já davam obstáculo falso); `--noise` o
escala. Os obstáculos ficam a 2,5 m ou mais dos waypoints do `caso4` e do `caso5`.

## 5. Localização e navegação

```
rodas (/wheel/odom) + IMU (/imu/data) ─► EKF ─► /odom, TF odom → base_link
LiDAR 3D (/lidar3d/points) + LiDAR de segurança (/scan) ─► lidar_merge ─► /lidar3d/merged
câmera RGB-D + /lidar3d/merged + odom do EKF ─► RTAB-Map ─► TF map → odom, /rtabmap/map
Nav2 (costmaps com o /scan): planeja em map, controla em odom ─► /cmd_vel_raw
/cmd_vel_raw ─► collision_monitor_limit ─► collision_monitor ─► /cmd_vel ─► driver
                (lentidão, aproximação)     (parada)      (os dois com o /scan)
```

### EKF (`config/ekf.yaml`, `ekf:=true`)

`robot_localization` a 30 Hz, 2D. Usa vx/vy das rodas e yaw + velocidade de yaw do IMU.
- O yaw das rodas fica de fora (escorregamento) e a aceleração também (dupla integração
  piora a posição).
- `process_noise_covariance` de x/y em 0 e covariância das rodas proporcional ao movimento:
  a elipse cresce com o deslocamento e na direção dele, e não com o tempo parado.
- **Ruído de processo de vx em 1,0** (o padrão é 0,025): na entrelinha o Nav2 para e arranca
  várias vezes por segundo (o RPP vê as plantas ao lado como colisão à frente), e o robô vai
  de 0,4 m/s a 0 num passo de 32 ms. Como a variância das rodas parado é quase zero, com
  0,025 o EKF zerava a velocidade na hora ao parar, mas levava ~5 passos para voltar a 0,4 ao
  arrancar, e andava 4 a 8% menos que as rodas. No caso 3 isso dava 0,3 a 0,45 m de erro ao
  longo da fileira (que o ICP não corrige, porque a fileira é igual em todo o comprimento), e
  o robô ficava preso na saída da entrelinha. Com 1,0 o EKF segue as rodas no mesmo passo
  (perde ~0,4%). Na mesma semente de ruído, o caso 3 foi de 7/11 com 15 paradas para 11/11
  sem paradas (ATE 0,30 → 0,09 m) e o caso 4 do olival de ATE 1,01 para 0,73 m (15/15 nos
  dois); resultados em `resultados/erro_pose_serpentina/` nos arquivos do projeto.
- **Rumo pelas fileiras (`row_heading.py`, `config/ekf_row_heading.yaml`, `row_heading:=auto`,
  ligado no olival):** o yaw do giroscópio deriva (~0,6° por minuto com o ruído da seção 3, e
  ~1° a cada giro de 90° pelo erro de escala) e, numa ida pela entrelinha de 65 m do olival, não
  há nada já visto para o RTAB-Map fechar laço. Num pomar as fileiras são retas e paralelas:
  o nó mede a direção delas na nuvem do LiDAR 3D (pontos de 0,3 a 4 m acima do chão local, até
  40 m; para cada ângulo a até 8° do yaw previsto, projeta os pontos na normal e procura o
  ângulo em que eles se juntam em faixas de 0,2 m) e publica o yaw em `/row_heading/pose`. O
  EKF usa essa medida como yaw absoluto e o IMU só como velocidade de giro (o yaw do IMU deriva
  e não volta). A direção das fileiras no `odom` é medida nas 5 primeiras nuvens, então o robô
  não precisa partir alinhado; no olival a grade de 7 × 7 m tem linhas nas duas direções, e
  vale o múltiplo de 90° mais perto. Só mede com o robô quase sem girar, com o pico nítido e a
  até 3° do EKF. Hipótese: fileiras retas e paralelas.
  - Medido no banco de um caso 4 (pose verdadeira, trechos retos): 0,5° rms por nuvem, sem
    viés, contra 1,2° do EKF só com o IMU.
  - O ruído de processo do yaw cai de 0,06 para 1e-5 rad²/s: com 0,06 cada medida substituía o
    yaw, ele pulava a cada 0,4 s, o robô ziguezagueava (desvio do yaw na ida de 0,5 para 6°) e
    escorregava encosta abaixo (2,3 m de erro lateral). Com 1e-5 a medida corrige aos poucos.
  - Na plantação fica desligado: o VSLAM já segura o yaw ali, e no caso 2 o yaw rms foi de 0,6
    para 1,0° com ele ligado (uma rodada).
- **Raio efetivo das rodas calibrado (`wheel_radius_scale`, `config/wheel_calibration.yaml`):**
  o pneu real roda com raio diferente do nominal (carga, pressão, solo), e o ruído da seção 3
  sorteia esse erro por semente (~1%; 1,8% a menos na semente 101). Na ida de 66 m do caso 4
  isso dava 1,1 m de erro ao longo da fileira, que o RTAB-Map não corrige (a entrelinha é igual
  em todo o comprimento). O driver multiplica o raio da odometria pelo fator calibrado (o
  comando dos motores fica com o nominal). No simulador a semente faz o papel do robô, por
  isso a tabela é por mundo e por semente; com `noise_seed:=0` não há calibração.
  - **Como calibrar** (uma rodada de ~45 min na nuvem, ida e volta de 33 m na entrelinha da
    partida):
    ```bash
    ros2 launch agrobot_webots simulation.launch.py world:=olive_grove.wbt ekf:=true \
        vslam:=true rviz:=false noise_seed:=101 wheel_radius_scale:=1.0 \
        lateral_correction:=none wheel_calibration:=true \
        waypoints:=calibracao_rodas_olival
    # ou: python3 agrobot_webots/scripts/bateria.py --headless \
    #   --cases calibracao_rodas_olival --seeds 101 -- wheel_calibration:=true \
    #   wheel_radius_scale:=1.0 lateral_correction:=none
    ```
    O `wheel_calibration.py` usa só os trechos retos: raio = distância horizontal verdadeira
    / distância das rodas; offset do roll da IMU = média do roll na ida e na volta (a encosta
    inverte o sinal, o offset não); ganho do escorregamento por mínimos quadrados. Copie os
    valores de `dados/calibracao/` para `config/wheel_calibration.yaml`. A referência é a pose
    verdadeira do Webots; no campo seria uma trena, um GNSS RTK ou marcos na entrelinha.
  - Medido: x1,0174 (101) e x1,0036 (303), contra 1,0180 e 1,0039 sorteados; o offset do roll
    a 0,04° do verdadeiro.
- **Escorregamento encosta abaixo pela inclinação (`slope_slip.py`, `lateral_correction:=imu`,
  padrão no olival):** de través na encosta o skid-steer escorrega para baixo, e as rodas
  dizem vy = 0. O nó fica entre as rodas e o EKF e soma
  `vy = ganho · sin(roll − offset) · |vx|`, com o roll da IMU e o ganho e o offset da
  calibração (ganho −0,17 e −0,15 nas duas sementes: é do solo). O modelo explica 33 a 43% da
  velocidade lateral verdadeira (o resto é o vaivém do controlador), e a variância de vy cresce
  com a correção.
- **Posição lateral pelas fileiras (`lateral_correction:=rows`, fica como opção):** o
  `row_heading` também mede onde o robô está entre as fileiras (a fase do padrão de 7 m na
  normal delas) e manda ao EKF a posição corrigida só nessa direção
  (`config/ekf_row_position.yaml`). A referência das linhas no `odom` é a média dos primeiros
  20 m. Funciona no `odom` (erro lateral do `odom` ≤ 0,4 m nas 4 rodadas), mas a medida erra 0,2
  a 0,4 m onde as oliveiras saem da grade (até 0,4 m no mundo, como num pomar real), e a direção
  das fileiras medida na partida já vem 0,3 a 0,6° torta pelo mesmo motivo; nos casos 4 e 5
  ficou pior que `imu` (seção 7). Para melhorar, precisaria de um mapa das árvores (posição de
  cada uma), não só da hipótese de grade.
- O frame `odom` deriva por definição; nenhuma entrada do EKF mede posição (a não ser com
  `lateral_correction:=rows`). Quem corrige é o VSLAM.
- Com `sensor_noise:=true` (padrão) o yaw do IMU deriva como o de uma IMU MEMS real (seção 3);
  com `sensor_noise:=false` ele é perfeito e o EKF fica favorecido.

### Checagem de patinagem (`wheel_slip_monitor`, `slip_check:=true`)

Preso numa pedra ou num mourão com as rodas girando, as rodas dizem que o robô anda e o EKF
vai junto; o IMU não percebe (parado e velocidade constante dão aceleração zero). Por isso um
`rgbd_odometry` (odometria visual da câmera RGB-D, `/vo/odom`) mede a velocidade sem olhar as
rodas, e o `wheel_slip_monitor` fica entre o driver e o EKF (`/wheel/odom` →
`/wheel/odom_checked`). Normalmente as rodas passam sem mudança; durante a patinagem o EKF
recebe a velocidade da câmera no lugar delas, e `/wheel/slip` fica `true`.

A odometria visual erra de vez em quando, e um alarme falso faz o EKF seguir uma velocidade
errada. A lógica (em `slip_detector.py`) se protege assim:
- **Descarta leituras impossíveis:** acima de 0,8 m/s (o robô anda até 0,5) e as 3 primeiras
  depois de a câmera se perder. Logo depois de recomeçar (`Odom/ResetCountdown: 1`) ela dava
  picos de ~2 m/s para trás, que eram a origem dos alarmes falsos.
- **Decide por janela:** só entra em patinagem quando, em 1,5 s, há pelo menos 8 leituras
  válidas e 80% delas discordam das rodas; sai quando 80% concordam em 1 s. Os alarmes falsos
  duravam ~1 s; preso, o robô fica vários segundos.
- **Patinando, usa a mediana da câmera** do último meio segundo; se ela some, segura o último
  valor com a incerteza crescendo por até 3 s, e depois devolve as rodas.

Por que não pôr a câmera direto no EKF (`twist_rejection_threshold`): na patinagem as rodas
(30 Hz) puxam o estado, e o teste de Mahalanobis rejeitaria justamente a câmera, que é quem
está certa; e as variâncias do `rgbd_odometry` às vezes saem quase nulas.

Na plantação, contra um mourão da cerca (0,4 m/s por 22 s, ~8 s preso): detectou em ~1,3 s
e o erro final da pose ficou em 0,47 m, contra 2,8 m sem a checagem. O que o EKF anda antes
da detecção (~0,45 m) fica no erro.
Nas missões normais (caso2 duas vezes, caso4 do olival uma vez, com EKF+VSLAM) não houve
nenhum alarme, com 2 a 7 perdas da câmera por corrida.

### VSLAM (`config/rtabmap.yaml`, `vslam:=true`)

`rgbd_sync` + `rtabmap` no namespace `rtabmap`, a 1 Hz, 3DoF (x, y, yaw), com a câmera RGB-D e
a nuvem do LiDAR 3D. Escolhas e motivos:
- **Odometria do EKF, sem odometria visual:** economiza CPU; o RTAB-Map só corrige o
  `map → odom`. (A odometria visual da checagem de patinagem não entra no RTAB-Map.)
- **Um só dono de `map → odom`:** dois publicando fazem o robô pular entre duas poses.
- **`Reg/Strategy` 2 (visual + ICP com a nuvem do LiDAR 3D) e `Optimizer/Robust`
  (Vertigo):** a textura do solo se repete a cada 2 m e gerava fechamentos falsos; o ICP e a
  otimização robusta os derrubam.
- **LiDAR 3D no ICP, no lugar do `/scan`:** o scan a 13 cm só via colmos e troncos num corte fino, e
  as entrelinhas davam o mesmo scan. A nuvem 3D tem o chão, os troncos, as copas e o relevo,
  que travam x, y e yaw no ICP ponto-a-plano (`Icp/PointToPlane`, voxel de 10 cm, até 0,5 m
  entre correspondências, correção de até 0,3 m sobre o chute do EKF). Com 16 canais a
  nuvem é esparsa: dois nós da mesma entrelinha casam só 6 a 9% dos pontos, por isso o mínimo
  casado é baixo (`Icp/CorrespondenceRatio` 2,5%) e quem barra fechamento errado é o limite de
  correção (com 0,5 m o ICP escorregava ao longo da fileira e o mapa piorava). Cada nó guarda a nuvem reduzida
  a 10 cm (`Mem/LaserScanVoxelSize`). O LiDAR 3D **não** vira odometria nem localização
  principal: a pose continua sendo o EKF corrigido pelos fechamentos.
- **Grade só pelo LiDAR de segurança, LiDAR 3D só no ICP:** o RTAB-Map assina uma nuvem só
  (`/lidar3d/merged`, do nó `lidar_merge`): a do LiDAR 3D com o scan de segurança junto. O
  `lidar_merge` tira da nuvem 3D os pontos da faixa de altura que a grade usa, e a grade
  (`Grid/MinGroundHeight`, `MaxGroundHeight`, `MaxObstacleHeight`) só vê o scan: obstáculo
  onde ele bate, livre ao longo dos raios, e cada raio sem retorno vira chão a 5 m (como o
  `Grid/Scan2dUnknownSpaceFilled` do scan 2D). Assim a grade mostra o mesmo que os costmaps
  do Nav2. Com a grade pela nuvem 3D, as fileiras sumiam do mapa em ~1 minuto de mapeamento:
  o LiDAR 3D, a ~1 m do chão, só vê um colmo numa faixa a 2,6–4 m, e os raios dele até o chão
  passam por cima das plantas e, na grade 2D, apagavam o que a passada anterior tinha
  marcado. E o relevo (camalhões) a 5–6 m virava obstáculo falso no meio das entrelinhas, e a
  checagem recusava waypoints.
- **Toques do scan no chão:** o LiDAR de segurança é um plano a 15 cm; com o relevo, ele bate
  no chão (ou num galho num camalhão) a alguns metros. Antes de entrar na grade, o
  `lidar_merge` confere cada ponto do scan na nuvem 3D: sem nada mais alto que ele (+5 cm) a
  até 15 cm, é chão e sai; além de 3 m sem nenhum ponto 3D por perto, também sai (um
  obstáculo de verdade aparece na nuvem). Perto do robô (a nuvem 3D não vê abaixo de ~3 m),
  o scan vale sozinho. No caso 3, ~20% dos pontos do scan eram chão. Esse filtro vale só
  para a grade: a camada de segurança e os costmaps usam o `/scan` cru.
- **Ocupado pesa mais que livre (`GridGlobal/ProbMiss` 0,47):** um colmo de 2 a 4 cm nem
  sempre é atingido, e com o padrão (0,4) dois ou três raios que passavam ao lado o apagavam.
  Agora são ~7 raios livres para desfazer um acerto.
- **Grade conferida (`stable_map.py`, `stable_map:=true`):** a grade do rtabmap soma a grade
  local de todos os nós; quando uma passada volta com a pose deslocada da anterior (a deriva do
  EKF entre fechamentos de laço passa de 0,3 m na serpentina), as fileiras aparecem de novo ao
  lado e os raios livres da passada torta apagam as fileiras certas. O nó `stable_map` monta
  `/rtabmap/map` com os mesmos dados (`/rtabmap/mapData`: grafo otimizado + grade local de
  cada nó) e o mesmo log-odds, mas compara cada nó novo com o mapa já feito
  (a grade do rtabmap continua em `/rtabmap/map_rtabmap`):
  - célula vista de perto (até 4 m) por menos de 3 nós ainda está no mapeamento inicial e é
    atualizada normalmente; o que o nó vê longe só preenche o que ainda não foi assentado;
  - **conferência do nó:** os pontos ocupados do nó em lugar já mapeado são comparados com o
    mapa na pose dele e deslocados até 0,8 m e 2,3°. Se uma pose deslocada (≥ 0,25 m) bate
    25 pontos percentuais melhor, a passada está desalinhada e o nó fica de fora da grade;
  - nó conferido: ponto ocupado a até 0,3 m de um obstáculo do mapa é o mesmo obstáculo (não
    engorda nem duplica a fileira); longe, é obstáculo novo e entra com ~3 nós, como no
    rtabmap. Raio livre só apaga um obstáculo do mapa se o nó não tem ponto ocupado a até
    0,3 m dali.
  A camada de segurança e os costmaps do Nav2 não usam essa grade (leem o `/scan` cru), então
  um obstáculo novo é sempre visto na hora; na grade (checagem dos waypoints, RViz, página do
  operador) ele aparece depois de alguns nós. Na localização, o mapa salvo é carregado do
  rtabmap no início e o que mudar na grade vale só para a sessão (o banco não muda).
  **Medido** (plantação, mapeando 3 voltas no mesmo trajeto, mesmo banco reprocessado com a
  pose verdadeira como referência): sem deslocamento, as duas grades ficam iguais (o ruído
  cai com as voltas nas duas). Com as voltas 2 e 3 deslocadas 0,4 m/1,7° e 0,6 m/-1,7°
  (simulando a deriva), a grade do rtabmap perdeu 62% das fileiras e as células fora do lugar
  passaram de 202 para 393; a conferida ficou igual à da primeira volta (213 fora do lugar,
  23% das fileiras faltando). Com 0,2 e 0,3 m: rtabmap 202 → 436 fora do lugar e 60%
  faltando; conferida 213 → 221 e 28%. Testes em `test/test_stable_map.py`.
  **Limite:** a conferência só pega desalinhamento de até 0,8 m (menos que meia entrelinha
  da plantação, para não casar com a fileira vizinha) e não conserta o que já entrou errado
  na primeira passada. Uma deriva lenta e grande é problema da localização: no olival
  (2 voltas mapeando, 2026-11-18) a pose derivou até 3 m na lateral, e as duas grades ficaram
  com as oliveiras deslocadas.
- **Ajustes do olival (`config/rtabmap_olive_grove.yaml`):** o launch carrega
  `config/rtabmap_<mundo>.yaml` por cima do `rtabmap.yaml` quando existe. No olival,
  `Icp/MaxTranslation` 1,0 m (0,3 na plantação) e `RGBD/ProximityMaxGraphDepth` 0 (padrão 50).
  Na volta do caso 4, pela entrelinha a 7 m da de ida, a deriva já passava de 0,3 m e todo
  fechamento com a passada de ida era recusado; e a ida fica a mais de 50 elos no grafo. Os 0,3 m
  existem por causa das entrelinhas a 1,8 m da plantação; no olival elas ficam a 7 m. No banco de
  2 voltas do olival reprocessado, o erro online da pose caiu de 1,76 m (0,3 m) para 0,61 (0,6),
  0,40 (0,8), 0,34 (1,0) e 0,36 m (1,5).
- **Fechamento global desligado (`Rtabmap/LoopThr` e `RGBD/AggressiveLoopThr` em 1,01):** as
  entrelinhas são iguais para a câmera e para o LiDAR (1,8 m de deslocamento dava o mesmo
  scan 2D). Só valem fechamentos por proximidade, perto da pose do EKF (câmera até 1 m e 45°,
  ICP corrige até 0,3 m). Funciona enquanto a deriva do EKF entre dois fechamentos ficar
  abaixo de ~0,8 m. Para religar a detecção global, volte os dois valores para `0.11` e
  `0.05`.
- **Localização:** roda numa cópia do banco (`Mem/IncrementalMemory` false) com
  `RGBD/StartAtOrigin`, então o robô começa no primeiro nó do mapa.
- Na plantação, as passadas repetidas são em sentidos opostos e a câmera não reconhece o
  lugar visto de costas; os fechamentos vêm sobretudo do LiDAR. Por isso o erro do VSLAM fica
  perto do EKF sozinho nesses casos.
- **Com o LiDAR 3D e a camada de segurança** (caso2, nuvem, 2026-11-16), 3 rodadas (a primeira com o ICP anterior) com 5/5
  waypoints: ATE 0,23, 0,58 e 0,18 m (com o scan 2D, 0,16 ± 0,01 m). A rodada de 0,58 m teve
  deriva do EKF já na primeira passada, antes de qualquer fechamento possível; o mesmo banco
  reprocessado com o ICP antigo dá o mesmo erro, então não veio dos fechamentos. A variação
  entre rodadas ainda precisa de mais repetições para comparar com o scan 2D.

### Nav2 (`config/nav2_params.yaml`, `launch/navigation.launch.py`)

- Launch próprio, e não o `nav2_bringup`: no Jazzy ele sobe o `docking_server`, que falha
  sem plugins e derruba o lifecycle.
- Com `vslam:=true` (ou `slam:=true`), o planner, o bt_navigator e o costmap global ficam em
  `map`; o controlador e o costmap local ficam sempre em `odom` (REP-105).
- Regulated Pure Pursuit com giro no lugar, NavFn com `allow_unknown`, velocidade até
  0,5 m/s e 1,0 rad/s, tolerância 0,15 m / 0,3 rad.
- Footprint 1,2 × 1,12 m (as rodas saem do chassi) e inflação 0,7 m.
- `min_obstacle_height: -1.0` na camada e na fonte: o LiDAR fica abaixo do `base_link`.
- `inf_is_valid: true` na fonte do scan: raio sem retorno (+inf do Webots) também limpa o
  costmap. Sem isso, em lugar aberto (olival) quase tudo ficava desconhecido, o caminho
  serpenteava entre as fatias livres e o escorregamento nas curvas levava o EKF a ~1 m de
  erro em 50 m.
- Costmaps sem camada estática, janela global de 24 × 24 m centrada no robô: objetivos a
  mais de ~12 m são recusados.
- `default_server_timeout` 1000 ms: com câmera e RTAB-Map a CPU fica cheia e o planner
  demora a aceitar o pedido.

### Camada de segurança (`collision_monitor`, `safety:=true`)

O LiDAR de segurança (seção 3) tem prioridade sobre o planejamento: pode haver uma pessoa no
campo. Quem aplica isso são dois `collision_monitor` do Nav2 em série
(`config/collision_monitor.yaml`), os últimos nós antes das rodas. Tudo o que comanda o robô
(Nav2, comportamentos de recuperação, joystick, teclado, página web) publica em
`/cmd_vel_raw`; o `collision_monitor_limit` limita a velocidade (`/cmd_vel_limited`) e o
`collision_monitor` para (`/cmd_vel` do driver). Eles não planejam: só reduzem ou cortam a
velocidade pedida quando há pontos do `/scan` dentro de zonas que andam com o robô. O Nav2 continua usando o mesmo `/scan` nos costmaps
para desviar; a camada de segurança é o que garante a parada quando o desvio não basta (uma
pessoa que entra na frente de repente, o operador conduzindo mal).

| Zona | Nó | Forma (no `base_link`; robô de x ±0,6, y ±0,56) | Ação |
|---|---|---|---|
| Lentidão (`slowdown_zone`) | limite | De 0 a 1,4 m à frente do robô, 10 cm de cada lado | Limita a 0,2 m/s |
| Aproximação (`approach_zone`) | limite | Simula a pegada (5 cm de folga) andando com o comando | Reduz para que nada fique a menos de 1,5 s de colisão (curvas e giros) |
| Parada (`stop_zone`) | parada | Pela velocidade que chega do 1º nó: até 0,2 m/s, 0,5 m à frente do robô; mais rápido, 0,7 m; 6 cm de cada lado; para trás, o mesmo atrás; girando no lugar, a pegada com 5 cm de folga | Velocidade zero |

- **Parar x desviar:** a camada não sabe se é uma pessoa, uma planta ou um galho. A zona de
  parada depende da velocidade, como os campos de proteção dos veículos autônomos
  industriais (ISO 3691-4): perto de qualquer obstáculo o robô anda a 0,2 m/s, e nessa
  velocidade ele para em ~6 cm, então a zona de parada pode ser curta e não pega as plantas
  por onde o Nav2 passa raspando nas manobras. Quem desvia é o Nav2 (costmaps); a parada fica
  para o que chega perto demais, como uma pessoa que entra na frente. Com uma zona fixa de
  0,6 m, o robô parava de frente para as plantas na cabeceira e ficava travado.
- **Por que dois nós:** o `collision_monitor` escolhe a zona de parada pela velocidade que
  recebe, não pela que sai. Num nó só, o Nav2 pedindo 0,4 m/s perto das plantas usaria a zona
  longa mesmo com a lentidão agindo.
- **Tamanho das zonas:** o LiDAR a 15 cm vê a canela; o corpo e os pés da pessoa vão ~0,25 m
  além dela, e entre o scan e a roda passam ~0,25 s. A 0,2 m/s o robô anda ~6 cm até parar;
  a 0,5 m/s, ~15 cm.
- **Por que zonas que dependem do movimento:** parado na frente de um obstáculo, o robô
  ainda consegue girar ou recuar para sair (a zona olha para onde ele quer ir), mas não
  avançar sobre ele.
- **6 pontos para agir** (`min_points`): com 1° por raio, uma perna de pessoa a 0,7 m dá ~10
  pontos e um mourão de 10 cm a 0,5 m dá ~11; um colmo de milho ou a ponta de uma folha caída
  dá de 1 a 4. Com 3 pontos (o padrão do Nav2), as plantas da cabeceira paravam o robô em
  toda manobra e o Nav2 desistia do objetivo; desviar das plantas é trabalho do Nav2.
- **Falha segura:** sem scan novo por 1 s, o robô para (`source_timeout`). Se o
  `collision_monitor` cair, nada chega às rodas e o driver para o robô em 1 s.
- **`safety:=false`** volta ao Nav2 e ao operador mandando direto nas rodas. Serve só para
  comparar (controle dos testes).
- **Na página do operador** aparece "PARADA DE SEGURANÇA" (ou "segurança: devagar") quando
  uma zona age; o estado sai em `/collision_monitor_state` (parada) e
  `/collision_monitor_limit_state` (lentidão e aproximação), e as zonas em
  `/collision_monitor/stop_zone` e `/collision_monitor/slowdown_zone` (RViz).
- **Não é um sistema de segurança certificado:** o Delta-2A não é um sensor de segurança e
  nada aqui segue a ISO 13849 ou a IEC 61496. Na dissertação, "camada de segurança da
  navegação". Ele também só vê o que corta o plano a 15 cm: uma pessoa deitada, ou algo
  pendurado acima disso, passa. E com o relevo o plano pode bater no chão ou num galho num
  camalhão: perto do robô (até 0,7 m) isso exige uma rampa forte; mais longe, só limita a
  velocidade.

**Teste com uma pessoa no caminho (`scripts/teste_pessoa.py`).** Põe um `Pedestrian` do
Webots (com o controlador `controllers/pessoa_teste`, que o anima e grava a posição) num
cenário, comanda o robô pelo Nav2 ou como operador e mede pela pose verdadeira a menor folga
entre a pessoa (círculo de 0,25 m) e a pegada do robô:

```bash
python3 agrobot_webots/scripts/teste_pessoa.py mundo plantacao_cruzando   # gera o mundo
ros2 launch agrobot_webots simulation.launch.py world:=plantation_field_pessoa.wbt \
    ekf:=true vslam:=true vslam_db:=teste_pessoa teleop:=keyboard
python3 agrobot_webots/scripts/teste_pessoa.py rodar plantacao_cruzando   # logo em seguida
```

O mundo gerado fica só no `install/` (não no git). Rode o `rodar` logo depois do launch: a
pessoa é um robô `<extern>` e o Webots espera o controlador dela para andar. Cenários:
`plantacao_parada` e `olival_parada` (pessoa parada na entrelinha, o Nav2 vai a um objetivo
depois dela), `plantacao_cruzando` e `olival_cruzando` (ela atravessa a entrelinha a 1 m/s
quando o robô chega a ~2 m, para aparecer a ~0,6 m do para-choque) e `plantacao_operador` (o operador manda 0,4 m/s para cima dela).
Os resultados vão para `dados/seguranca/` (CSV a 10 Hz e uma linha por rodada em
`resumo.md`); `--rotulo sem-seguranca` com `safety:=false` no launch dá o controle.

Resultados na nuvem (2026-11-17, EKF + VSLAM, folga entre o círculo de 0,25 m da pessoa e a
pegada; `dados/seguranca/resumo.md`):

| Cenário | Folga mínima | O que aconteceu |
|---|---|---|
| `plantacao_operador`, sem a camada (`safety:=false`) | contato | o robô passa por cima da pessoa |
| `plantacao_operador` | 0,13 m | limite a 0,2 m/s a ~1,4 m, parada; o operador continua mandando e o robô não anda (com a zona devagar de 0,45 m, 0,08 m) |
| `plantacao_cruzando` | 0,70 m | limite enquanto ela atravessa; o Nav2 chega ao objetivo |
| `olival_cruzando` | 0,49 m | limite; o Nav2 chega ao objetivo |

Missões com a camada (sem pessoa): no caso 3 (serpentina na plantação), as paradas com o robô
torto na entrelinha vinham da pose errada ao longo da fileira (o EKF perdia distância nas
arrancadas; corrigido no `ekf.yaml`, seção 5). Depois da correção (nuvem, 2026-11-18, 3
rodadas): 11/11 com ATE 0,09 m na semente que antes parava 15 vezes; 8/11 numa semente com
giroscópio 2,4% fora de escala (o yaw do EKF derivou 15° e o VSLAM segurou em 2–4°); e 8/11
numa semente nova, preso no contorno da pedra da cabeceira leste com a pose certa (erro de
0,07 m): o vão de ~1,5 m ali, com a camada de segurança, às vezes não deixa o robô passar.
No olival (caso 4), 15/15 e nenhuma parada.

### Teleoperação e mapeamento manual (`teleop:=joy|keyboard`)

O primeiro mapa do campo real vai ser feito com um operador conduzindo o robô (plano em
`planejamento/primeiro_mapa_campo.md` nos arquivos do projeto); a simulação serve de ensaio.

```
joystick / teclado ─► /cmd_vel_teleop ─┐
Nav2 (velocity_smoother) ─► /cmd_vel_nav_out ─┴► cmd_vel_mux ─► /cmd_vel_raw ─► collision_monitor_limit ─► collision_monitor ─► /cmd_vel ─► driver
```

- **`cmd_vel_mux`:** o operador tem prioridade. Enquanto chegam comandos dele, o Nav2 é
  ignorado; 0,5 s depois que ele para, o Nav2 volta a comandar **sem cancelar a missão**.
  Assim dá para tomar o controle no meio de uma missão e devolvê-lo. Quem comanda sai em
  `/cmd_vel_mux/source`. Sem `teleop`, o mux não sobe e o Nav2 manda direto no
  `/cmd_vel_raw`. O operador também passa pela camada de segurança: ele não consegue jogar o
  robô em cima de uma pessoa.
- **Joystick (`teleop:=joy`):** `game_controller_node` (pacote `joy`, mesmo mapeamento para
  Xbox, PlayStation e genéricos) + `teleop_twist_joy`, configurados em
  `config/teleop_joy.yaml`. Segure **LB/L1** para andar (botão de homem-morto: soltou,
  parou) ou **RB/R1** para o modo rápido; analógico esquerdo para frente/ré e giro. Até
  0,3 m/s e 0,6 rad/s (rápido: 0,5 m/s e 1,0 rad/s). No WSL, o controle USB precisa ser
  repassado ao Linux com o `usbipd-win`; sem isso, use o teclado.
- **Teclado (`teleop:=keyboard`):** o launch não tem terminal, então rode
  `ros2 run agrobot_webots teleop_keyboard` num outro. Funciona como piloto automático: cada
  tecla muda a velocidade e o robô a mantém, sem precisar segurar (a repetição da tecla
  segurada dá trancos). `w`/`s` ou setas: ±0,1 m/s; `a`/`d`: ±0,2 rad/s; espaço: parar e
  devolver ao Nav2; `p`: segurar parado (o Nav2 não comanda); `q`: sair.

**Do mapa às missões de aquisição** (exemplo no olival, mapa `olival_mapa1`):

1. **Mapear conduzindo:**
   ```bash
   ros2 launch agrobot_webots simulation.launch.py world:=olive_grove.wbt ekf:=true \
       vslam:=true vslam_db:=olival_mapa1 nav:=false teleop:=keyboard
   ros2 run agrobot_webots teleop_keyboard      # no outro terminal
   ```
   A grade aparece no RViz (`/rtabmap/map`) enquanto o robô anda. Protocolo de condução
   para o RTAB-Map fechar laços: parado ~10 s na partida, uma volta pelo perímetro voltando
   à partida, depois as entrelinhas, revisitando de tempos em tempos uma entrelinha **no
   mesmo sentido** de antes (a câmera não reconhece o lugar visto de costas), a 0,3 a
   0,4 m/s e terminando parado na partida.
2. **Salvar:** opcionalmente, guarde também a grade como imagem (para a dissertação ou para
   conferir), com a simulação ainda aberta:
   `ros2 run nav2_map_server map_saver_cli -t /rtabmap/map -f dados/vslam/olival_mapa1`
   (gera `.pgm` e `.yaml`). Depois feche o Webots e espere a linha "Mapa salvo em
   dados/vslam/olival_mapa1.db" no log. O mapa é o `.db`; a imagem não é usada para navegar.
3. **Marcar os waypoints no mapa:**
   ```bash
   ros2 launch agrobot_webots simulation.launch.py world:=olive_grove.wbt ekf:=true \
       vslam:=true vslam_localization:=true vslam_db:=olival_mapa1 waypoints:=rviz
   ```
   O robô começa no primeiro nó do mapa e o mapa salvo aparece no RViz. Marque os pontos com
   **Publish Point** (no máximo ~12 m entre dois seguidos) e grave com
   `ros2 service call /waypoint_mission/save std_srvs/srv/Trigger`, que cria
   `dados/missions/rviz_<data>.yaml` com `map: olival_mapa1`. Pode já rodar com `start`.
4. **Rodar a missão de aquisição** quantas vezes quiser, no mesmo mapa:
   ```bash
   ros2 launch agrobot_webots simulation.launch.py world:=olive_grove.wbt ekf:=true \
       vslam:=true vslam_localization:=true vslam_db:=olival_mapa1 \
       waypoints:=dados/missions/rviz_<data>.yaml data_camera:=true
   ```
   A missão é recusada se o `map:` do arquivo não for o mapa em uso: as coordenadas só
   valem no mapa em que foram marcadas. Com `teleop:=keyboard` junto, dá para tomar o
   controle no meio da missão.

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
desliga).**
- **Com missão** (`waypoints:=...`): a câmera fica desligada fora da missão. O
  `waypoint_mission` liga a câmera quando o Nav2 aceita a missão e desliga no fim, e as fotos
  vão para `dados/fotos/<caso>_<modo>_<data>/`, o **mesmo prefixo** dos CSVs da missão em
  `dados/missions/` (a data é a do início da missão). Assim cada pasta de fotos casa com a
  sua trajetória. Com `waypoints:=rviz`, cada `start` abre uma pasta nova.
- **Sem missão:** grava a execução inteira em `dados/fotos/<mundo>_<modo>_<data>/`.

```bash
# Missão com aquisição de fotos (localização oficial: EKF+VSLAM)
ros2 launch agrobot_webots simulation.launch.py world:=plantation_field.wbt ekf:=true vslam:=true \
    waypoints:=caso3_plantacao_serpentina data_camera:=true
```

Cada pasta tem:
- `000001.png`, `000002.png`, ...: as fotos em RGB, sem perdas (~2 a 3 MB cada);
- `fotos.csv`: uma linha por foto, com o tempo de simulação, a pose **estimada** da câmera
  (`est_*`, pelo TF `map → data_camera_optical_frame` no carimbo da foto; `odom` sem
  SLAM/VSLAM), a pose **verdadeira** (`real_*`, do `/ground_truth/odom` do mesmo carimbo,
  trazida para o frame da partida como no `waypoint_mission`) e o erro entre as duas
  (`erro_xy_m`, `erro_z_m`, `erro_yaw_rad`). Cada pose tem x, y, z do centro óptico,
  roll/pitch/yaw do `data_camera_link` (yaw ≈ yaw do robô + 90°) e o quaternion do frame
  óptico, para projetar pontos na imagem;
- `camera_info.yaml`: intrínsecos, no formato do `camera_calibration_parsers`;
- `origem.yaml`: pose verdadeira da partida no mundo do Webots (x, y, z, yaw), para levar as
  plantas do `.wbt` ao frame das colunas `real_*` na avaliação.

Regras do gravador:
- Só grava uma foto se a câmera andou pelo menos `data_min_distance` (0,3 m) desde a última,
  então o robô parado (nas paradas dos waypoints, por exemplo) não enche o disco.
- Espera até 1 s de simulação pelo TF do carimbo da foto. Se não chegar, grava a foto com as
  colunas `est_*` vazias e avisa no log.
- A pose verdadeira é relativa à primeira mensagem do `/ground_truth/odom` que o nó recebe; por
  isso ele tem que subir junto com a simulação (o launch faz isso).
- O início e o fim vêm do `waypoint_mission` pelo tópico `/data_recorder/session` (o prefixo
  no início, uma string vazia no fim). Fora de missão, para gravar só um trecho, use o
  serviço `/data_camera/enable`.

Conferido no Webots (nuvem, headless) no caso 2 da plantação com EKF+VSLAM: a câmera ficou
desligada até a missão, gravou 57 fotos em 96 s (erro xy da câmera de 0,13 m em média, 0,22 m
no máximo) e desligou no fim, com a pasta e os CSVs com o mesmo prefixo. Atenção ao cruzar as
duas: o `t_s` do `fotos.csv` é o tempo de simulação, e o dos CSVs da missão conta do início da
missão.

**Avaliação das fotos (`scripts/avaliar_aquisicao.py`, fora da simulação).** Responde quantas
plantas foram fotografadas e quão bem cada foto está georreferenciada:

```bash
python3 agrobot_webots/scripts/avaliar_aquisicao.py dados/fotos/<pasta> [<pasta> ...] --folha 12
python3 agrobot_webots/scripts/exportar_plantas.py plantation_field -o plantas.csv  # só o inventário
```

- **Inventário:** as plantas são lidas do próprio `.wbt` (cada `CropPlant` e `OliveTree`: base,
  altura, raios), não regeneradas pelo gerador, para ser sempre o mundo que rodou. O mundo sai
  do nome da pasta (`plantation_field`/`plantacao`, `olive_grove`/`olival`) ou de `--mundo`.
- **Cobertura**, pela pose verdadeira (onde a foto foi feita de fato): uma planta está
  **inteira** numa foto se o pé (`--corte-base`, 0,25 m acima da base), o topo e, na oliveira,
  as duas bordas da copa caem dentro da imagem, a menos de `--dist-max` na horizontal (padrão
  1,8 m no milho, que deixa só a fileira vizinha, porque as de trás ficam tapadas; 7 m no
  olival). No milho as folhas não contam: "inteira" é o colmo até o pendão. O corte da base
  existe porque, nas imagens do Webots, a câmera não vê os ~20 cm de baixo do colmo da
  fileira vizinha; e os milhos mais altos (topo acima de ~1,55 m do pé) saem com o pendão
  cortado. Saem as frações das plantas do mundo inteiras em ≥ 1 e ≥ 3 fotos e as vistas (meio
  da planta na imagem) em ≥ 1. Com uma câmera que vê um lado por passada, a cobertura do mundo
  inteiro é baixa por construção; a tabela por fileira mostra quais a rota cobriu.
- **No olival**, do meio da entrelinha (árvore a 3,5 m), a câmera vê o tronco e a base da copa,
  mas o alto da copa sai da imagem: a árvore só aparece inteira de longe, em diagonal, antes
  ou depois de passar por ela. Se a dissertação quiser a copa inteira, a câmera precisa
  inclinar para cima ou ter FOV vertical maior.
- **Erro de georreferência**, nas fotos com pose estimada: o da câmera (m e graus); o da
  planta em metros (onde ela iria parar no mapa usando a pose estimada da foto, o que soma o
  erro de posição e o de yaw vezes a distância); e o mesmo em pixels (meio da planta projetado
  com a pose estimada contra a verdadeira). É o número que diz se dá para achar a planta na
  foto pela pose do VSLAM.
- **Saída:** uma linha por pasta na tela (Markdown) e, em `<pasta>/avaliacao/`, `resumo.md`,
  `plantas.csv` (fotos em que cada planta aparece), `projecoes.csv` (cada planta inteira em
  cada foto, com os pixels e os erros) e, com `--folha N`, `folha_contato.png` (N fotos com o
  eixo de cada planta projetado: verde pela pose verdadeira, magenta pela estimada; pede
  `python3-matplotlib`).
- Sem `origem.yaml` (fotos gravadas antes dele), a partida vem da pose do robô no `.wbt` e
  avisa: o x/y fica certo, o z pode errar alguns centímetros.
- **Conferido no Webots** (nuvem, 2026-11-13): com fotos da `data_camera` nos dois mundos, o
  eixo projetado de cada planta pela pose verdadeira cai sobre o colmo/tronco na imagem (erro
  de até ~10 px, provavelmente o robô andando ~1 cm no passo da foto).

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

**Cabeceira leste da plantação (caso 3):** a pedra grande em (11,8; -1,5) fica na saída da
entrelinha y = -1,8 e deixa só ~1,1 m até o fim da fileira y = -0,9 (o robô tem 1,12 m), então
não dá para subir direto para a y = 0. Por isso o caso 3 sai dessa entrelinha em diagonal pelo
vão sul da pedra até `contorno_pedra_sul` (11,6; -2,85), onde há folga para girar, chega
olhando para o vão leste (`yaw: 0.7`) e contorna a pedra pelo leste. Parar no bolsão em
(10,8; -1,8), como era antes, deixava o robô olhando para a pedra sem conseguir girar, e o Nav2
perdia esse waypoint e o seguinte. Hoje a checagem dos waypoints (abaixo) recusa esse ponto.

**Checagem dos waypoints (`waypoint_check:=true`, padrão):** antes de a missão começar, e a
cada clique no RViz, o `waypoint_mission` confere cada ponto numa grade de ocupação com as
medidas do robô (1,2 x 1,12 m, mais 5 cm de folga; `waypoint_check.py`):

- **não cabe:** a pegada bate num obstáculo chegando ao ponto;
- **não gira:** o robô para em cada waypoint e gira no lugar até o yaw de saída (olhando para
  o próximo); a pegada tem de caber em todos os ângulos desse giro;
- **não chega:** não há caminho até o ponto por onde o robô passe (centro a pelo menos meia
  largura dos obstáculos, como o planejador do Nav2).

A direção de chegada sai de um caminho A* estimado na grade, do waypoint anterior até o
ponto. Com um ponto ruim a missão não começa: o log diz qual, por quê e o ponto livre mais
perto (até 1 m), e o ponto fica roxo no RViz. Um clique ruim é recusado com a mesma
explicação. A grade é o `/rtabmap/map` com VSLAM e o costmap global sem ele. Só dá para
conferir o que já está no mapa: localizando num mapa salvo (`vslam_localization:=true`) vale
para a rota toda; mapeando do zero, os pontos longe da partida ainda são desconhecidos e
passam sem checagem (o log avisa quais). A folga de 5 cm é pequena de propósito: com o mapa
da plantação ela separa o ponto antigo da cabeceira leste (recusado) dos pontos que o robô
cumpre nas rodadas; o erro de localização (0,1 a 0,3 m) não entra na conta.

Antes da checagem, a grade perde os **ciscos** (`check_speck_radius`, 0,3 m; 0 desliga): uma
célula ocupada com menos de 2 outras ocupadas a até 0,3 m vira livre. No `/rtabmap/map` da
plantação os torrões, as pedrinhas e os galhos das entrelinhas (o robô passa por cima) e os
toques do LiDAR no chão ondulado viram células soltas no meio da entrelinha, e uma só bastava
para recusar o ponto: num mapa real, só 37 de 95 pontos no meio das entrelinhas passavam
olhando ao longo delas; sem os ciscos, 94. As fileiras (plantas a cada 0,3 m), as pedras
grandes, os troncos e os mourões têm vizinhas e ficam: em cima de uma fileira e girando 90°
entre elas o ponto continua recusado.
| `caso4_olival_ida_volta` | olival | ~160 m: entrelinha y = 0 até a cabeceira leste e volta pela y = 7 | Lugar aberto, poucos retornos do LiDAR, declive lateral nas manobras de cabeceira |
| `caso5_olival_encosta` | olival | ~115 m: sobe a encosta atravessando as fileiras em x = 8 e desce em x = 29 | Subida e descida, grade de troncos vista de frente |

As coordenadas estão no frame da partida, então a mesma lista vale com e sem VSLAM.

**Waypoints clicados (`waypoints:=rviz`):** no RViz, ferramenta **Publish Point**, um
clique por ponto; depois `ros2 service call /waypoint_mission/start std_srvs/srv/Trigger`.
Também há `undo`, `clear` e `save` (grava a lista em `dados/missions/` no formato
dos casos, para rodar de novo com `waypoints:=<arquivo>`). Com VSLAM, o arquivo leva o nome
do mapa (`map:`), e a missão só roda nesse mapa. O passo a passo do mapa às missões está
na seção 5 (teleoperação).

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

**Bateria de avaliação (várias rodadas com estatística):**

```bash
python3 agrobot_webots/scripts/bateria.py            # casos 2 a 5, sementes 101 202 303 404 505
python3 agrobot_webots/scripts/bateria.py --cases caso3_plantacao_serpentina --seeds 1 2 3
python3 agrobot_webots/scripts/estatistica_bateria.py dados/bateria/<data> --por-rodada
```

O `bateria.py` roda cada caso com `ekf:=true vslam:=true rviz:=false noise_seed:=<semente>`,
uma rodada por vez, espera a missão terminar, fecha o launch e passa à próxima (por semente,
para que uma bateria interrompida tenha todos os casos; rodar de novo com `--out` na mesma
pasta continua de onde parou). Grava em `dados/bateria/<data>/` os CSVs, o log de cada
rodada e `rodadas.csv`, e apaga os bancos do mapeamento. `--headless` usa o `xvfb-run`
(nuvem); argumentos a mais vão para o launch (`vslam_localization:=true`). O
`estatistica_bateria.py` dá, por caso, média ± desvio padrão do ATE, do erro máximo, do RMSE
do yaw, dos waypoints atingidos e no alvo (≤ 0,5 m de verdade), das paradas (robô parado
≥ 1 s fora da parada programada nos waypoints) e do tempo.

Resultado da bateria final (nuvem, 2026-11-20, 5 sementes; detalhes em
`resultados/bateria_final/resumo.md` nos arquivos do projeto):

| Caso | ATE (m) | Erro máx. (m) | Atingidos | No alvo ≤ 0,5 m | Paradas |
|---|---|---|---|---|---|
| caso2 plantação | 0,10 ± 0,04 | 0,20 ± 0,08 | 100% | 100% | 0,8 ± 1,1 |
| caso3 plantação | 0,22 ± 0,18 | 0,46 ± 0,29 | 93 ± 8% | 91 ± 11% | 6,2 ± 4,9 |
| caso4 olival | 1,48 ± 1,24 | 3,11 ± 2,94 | 100% | 52 ± 24% | 0,6 ± 1,3 |
| caso5 olival | 1,07 ± 0,32 | 1,94 ± 0,72 | 100% | 32 ± 11% | 0 |

Na plantação a localização segura (os waypoints perdidos do caso 3 são quase todos o
contorno da pedra da cabeceira leste). No olival o Nav2 cumpre tudo, mas o robô chega onde
*acha* que é o alvo: o erro de yaw de 4 a 6° nas manobras de cabeceira vira metros de desvio
lateral na volta (até 8 m na semente 303 do caso 4), e o RTAB-Map não fecha laço para corrigir.

**Olival depois das correções de yaw** (rumo pelas fileiras e ajustes do RTAB-Map no olival,
seção 5; nuvem, 2026-11-23, mesmas sementes; CSVs em `resultados/yaw_olival/` nos arquivos
do projeto):

| Caso, semente | ATE (m) antes → depois | Erro máx. (m) | No alvo ≤ 0,5 m |
|---|---|---|---|
| caso4, 303 | 3,51 → 0,72 | 8,04 → 1,37 | 8/15 → 5/15 |
| caso4, 101 | 1,13 → 0,96 | 2,12 → 1,66 | 5/15 → 5/15 |
| caso5, 101 | 1,08 → 0,54 | 1,95 → 1,09 | 5/12 → 7/12 |
| caso5, 303 | 1,57 → 0,59 | 3,12 → 1,13 | 3/12 → 7/12 |

O yaw na ida do caso 4 ficou com 0,4 a 0,5° rms (antes até 1,7°) e a volta não diverge mais.
O que sobra no caso 4 não é yaw: na semente 101 o EKF anda 1,1 m a menos que o robô em 66 m
(o raio das rodas sorteado pelo ruído erra ~1,7%) e o robô escorrega 0,3 a 0,5 m encosta abaixo
na ida; nenhum dos dois é visto pelas rodas ou pelo IMU, e na ida não há com que fechar laço.

**Olival com o raio calibrado e a correção lateral** (seção 5; nuvem, 2026-11-25, mesmas
sementes, uma rodada por célula; CSVs em `resultados/distancia_escorregamento_olival/` nos
arquivos do projeto). ATE / erro máximo (m) / no alvo ≤ 0,5 m:

| Caso, semente | Só yaw (antes) | + raio calibrado | + raio + `imu` (padrão) | + raio + `rows` |
|---|---|---|---|---|
| caso4, 101 | 0,96 / 1,66 / 5 de 15 | 0,93 / 1,73 / 4 | 0,57 / 1,05 / 5 | 0,33 / 0,79 / 12 |
| caso4, 303 | 0,72 / 1,37 / 5 de 15 | 1,81 / 3,32 / 2 | 0,40 / 1,00 / 11 | 1,68 / 3,01 / 5 |
| caso5, 101 | 0,54 / 1,09 / 7 de 12 | 0,67 / 1,22 / 6 | 0,62 / 1,06 / 6 | 0,82 / 1,58 / 5 |
| caso5, 303 | 0,59 / 1,13 / 7 de 12 | 0,75 / 1,27 / 5 | 0,76 / 1,33 / 6 | 1,03 / 1,93 / 5 |
| média | 0,70 / 1,31 / 24 de 54 | 1,04 / 1,89 / 17 | **0,59 / 1,11 / 28** | 0,97 / 1,83 / 27 |

- O raio calibrado tira o erro ao longo da fileira (na ida do caso 4, semente 101, de 1,1 m
  para 0,1 m), mas sozinho não melhorou a média: o erro lateral sobra, e numa rodada (caso 4,
  303) o RTAB-Map desviou o `map` 2 m na volta.
- `imu` foi a melhor no conjunto e virou o padrão no olival. No caso 5 (que sobe e desce a
  encosta quase sem andar de través) ficou igual ao de antes.
- `rows` deu o melhor resultado de todos numa rodada (caso 4, 101: 12 de 15 no alvo) e o pior
  em outra (caso 4, 303: o `odom` ficou a 0,4 m, mas o RTAB-Map desviou o `map` 2,5 m na volta,
  como na rodada só com o raio). Uma rodada por célula não separa bem o método da sorte do
  RTAB-Map; a bateria de 5 sementes resolveria.

## 8. Interface web do operador (`operator:=true`)

Página para operar o robô de um computador qualquer pelo navegador, sem ROS instalado nele
(o Kauê usa Windows): ver o mapa e o robô, marcar waypoints, montar varreduras, rodar e
cancelar missões, conduzir com um joystick na tela, ver as câmeras e baixar o que foi gravado.

```bash
sudo apt install ros-jazzy-rosbridge-suite python3-opencv   # uma vez
ros2 launch agrobot_webots simulation.launch.py world:=plantation_field.wbt ekf:=true \
    vslam:=true data_camera:=true operator:=true rviz:=false
```

Abra **http://localhost:8080** no navegador do Windows (o WSL2 repassa as portas do
`localhost`). Em outro computador da rede, use o IP da máquina que roda o ROS (no WSL, isso
exige liberar a porta no Windows, `netsh interface portproxy`); num robô real com Linux, o IP
dele direto. Dá para usar junto com o RViz (`rviz:=true`) e com `waypoints:=<caso>` (a
página mostra a missão desse arquivo).

```
navegador ─WebSocket 9090─► rosbridge ─► /cmd_vel_teleop ─► cmd_vel_mux ─► collision_monitors ─► driver
          │                           ─► /waypoint_mission/command ─► waypoint_mission ─► Nav2
          │                           ◄─ /operator/* (operator_bridge) e /waypoint_mission/status
          └─HTTP 8080─► operator_web: a página (agrobot_webots/web/) e dados/ só para leitura
```

- **Por que web com rosbridge:** o operador só precisa de um navegador, e a página fala com os
  tópicos e serviços que já existem. Foxglove resolveria a visualização, mas não a lógica de
  missão; um app PyQt/rqt exigiria ROS no PC do operador.
- **`operator_bridge`:** mandar a grade, o scan e as imagens cruas pelo rosbridge (JSON) pesaria
  demais, então ele resume tudo: `/operator/state` (pose pelo TF no frame global, velocidade,
  quem comanda, patinagem, caminho do Nav2 e o scan já no frame global, a 5 Hz),
  `/operator/map` (a grade em PNG: `/rtabmap/map` com VSLAM, o costmap global sem) e as
  câmeras em JPEG (`/operator/<câmera>/compressed`). Ele só assina uma câmera enquanto a
  página mostra a prévia dela (a de aquisição tem 4,9 MB por imagem), e os timers são de
  relógio de parede, para a página não travar quando a simulação anda mais devagar que o
  tempo real.
- **`operator_web`:** servidor HTTP da página e de `dados/` (missões, fotos, mapas) só para
  leitura, na aba **Dados**. Não tem senha: quem alcança as portas 8080 e 9090 comanda o
  robô. Em rede aberta (campo real), restrinja as portas ou ponha senha antes de usar.
- **Joystick (aba Controle):** arraste o círculo ou segure **W A S D** / setas. A página
  manda `/cmd_vel_teleop` a 10 Hz só enquanto o operador comanda e, ao soltar, alguns zeros
  para o robô parar na hora. O `cmd_vel_mux` sobe sozinho com `operator:=true` e dá
  prioridade ao operador; 0,5 s depois que ele solta, o Nav2 volta sem cancelar a missão
  (seção 5). **PARAR** (ou Espaço) manda zeros sem parar, segurando o robô e deixando o Nav2
  de fora até soltar. Se a página fechar ou a rede cair, os comandos param de chegar e em
  0,5 s o mux devolve o controle ao Nav2 (ou para o robô, sem missão).
- **Camada de segurança:** o joystick passa pelo `collision_monitor` como o Nav2 (seção 5):
  o operador não consegue avançar sobre uma pessoa ou um obstáculo a 15 cm do chão. Quando
  uma zona age, aparece o aviso "PARADA DE SEGURANÇA" (vermelho) ou "segurança: devagar" /
  "aproximação" (amarelo) no topo da página.
- **Waypoints e missões (aba Missão):** a lista é do `waypoint_mission`, que sobe sozinho
  com `operator:=true` (sem arquivo, como no `waypoints:=rviz`). A página manda a lista
  inteira pelo tópico `/waypoint_mission/command` (JSON: `set`, `load`, `start`, `cancel`,
  `clear`, `undo`, `save`, `probe`) e mostra o `/waypoint_mission/status`. Ferramentas do
  mapa: **+ Waypoint** (clique acrescenta, botão direito apaga), **Ir até** (um ponto só, já
  inicia) e **Varredura**.
- **Arrastar pontos no mapa:** em qualquer ferramenta (também em Mover e Ir até), clicar num
  waypoint ou num ponto da prévia da varredura e arrastar o reposiciona; só a missão em
  andamento trava os pontos. Enquanto arrasta, a página pergunta ao `waypoint_mission` se o
  robô cabe e chega ali (`{"op": "probe", "x", "y", "prev", "next", "yaw"}`, resposta em
  `/waypoint_mission/probe`, a mesma checagem da lista, sem mudar nada) e desenha a pegada do
  robô no ponto: verde (cabe e chega), roxa (com o motivo e o ponto livre mais perto) ou
  cinza (área não mapeada). Uma pergunta por vez, sempre com a posição mais nova. Esc desfaz
  o arrasto. **Cancelar** para a missão; os CSVs são gravados com
  os waypoints restantes como `cancelado`.
- **Editar missões gravadas:** **Carregar para editar** traz uma missão de
  `config/waypoints/` ou de `dados/missions/` (só as deste mundo e deste mapa). Na lista, ✎
  abre nome, x, y e orientação (em graus; vazio = chega olhando para o próximo ponto), ↑ ↓
  mudam a ordem, + insere um ponto no meio do caminho até o próximo e × apaga; no mapa,
  arrastar move. **Salvar alterações** grava por cima do arquivo carregado, mas só dos de
  `dados/missions/`: os casos de estudo do projeto (`config/waypoints/`) não são
  sobrescritos, para os resultados continuarem comparáveis; use **Salvar como**, que cria
  `dados/missions/<nome>.yaml` (recusa um nome que já existe sem "substituir" e o nome de
  um caso do projeto). Essas missões também rodam com `waypoints:=dados/missions/<nome>.yaml`.
  O comando é `{"op": "save", "name": ..., "overwrite": ...}` no `/waypoint_mission/command`.
- **Checagem pelas medidas do robô:** cada ponto passa pelo `waypoint_check` (seção 7) assim
  que entra na lista e de novo quando a grade muda. Em roxo, com o motivo e, quando existe,
  o botão **usar ponto livre** (o ponto livre mais perto, até 1 m). "Área ainda não
  mapeada" (cinza) não bloqueia: no mapeamento o RTAB-Map só conhece o que o robô já viu.
- **Varredura (aba Varredura):** com a ferramenta Varredura, arraste do começo da primeira
  passada até o canto oposto. As passadas seguem x (as fileiras dos dois mundos) ou y, em
  serpentina, com o espaçamento escolhido (1,8 m na plantação, 7 m no olival) e um ponto a
  cada 10 m no máximo, porque o costmap global só aceita objetivos a até ~12 m. A câmera
  de aquisição olha para a esquerda, então cada passada fotografa o lado oposto da
  anterior. Cada ponto leva o yaw da própria passada (olhando ao longo da fileira): sem ele,
  o fim de cada passada teria de girar 90° entre as fileiras, onde o robô não cabe girando,
  e a checagem recusava esses pontos; com ele, a curva para a passada seguinte é do
  planejador do Nav2, pela cabeceira. Os pontos da prévia podem ser arrastados (com a
  pegada verde/roxa) e apagados (botão direito) antes de **Usar como missão**, que manda a
  lista ao `waypoint_mission` com o nome `varredura` (os CSVs e a pasta das fotos saem com
  esse prefixo). Mudar espaçamento, direção ou distância refaz a prévia e perde os ajustes.
- **Câmeras:** a cor da RGB-D também aparece na aba Controle. A de aquisição só existe com
  `data_camera:=true` e, com o `waypoint_mission` rodando, fica desligada fora das missões
  (os botões Ligar/Desligar chamam `/data_camera/enable`).
- **Conferido no Webots** (nuvem, headless, 2026-11-16): na plantação, carregar o caso 2,
  iniciar, tomar o controle no meio (o mux passou para o operador e devolveu ao Nav2) e
  cancelar (CSVs e pasta de fotos fechados); no olival, **Ir até** 4 m à frente (chegou com
  0,12 m de erro real) e uma varredura de duas entrelinhas desenhada na página, com as fotos
  gravadas em `dados/fotos/varredura_*`.

## 9. Site explicativo em 3D (`site/`)

Página para apresentações e para entender o projeto: o robô em 3D (medidas do
`agrobot.proto` e a malha do chassi), os sensores com o que cada um vê, os nós ROS 2
num grafo (o que entra, o que sai, por quê, onde age no robô), um roteiro de
apresentação, os resultados do passo 3 e os próximos passos.

- **Abrir:** dois cliques em `site/index.html`. No Windows, pelo Explorer em `\\wsl$\`
  (distro → pasta do projeto → `site`), ou copie a pasta `site/` para o Windows. Não precisa
  de servidor nem de internet: o three.js vai junto em `site/vendor/`.
- **Online:** https://kkauems.github.io/ros2_gazebo_mestrado/ (GitHub Pages). O workflow
  `.github/workflows/pages.yml` publica a pasta `site/` sozinho a cada push na `main` que
  mexa nela; para atualizar o site basta mudar `site/` e fazer merge. Precisa estar ligado
  uma vez em Settings → Pages → Source: **GitHub Actions**.
- **No uso:** setas ← → passam o roteiro, espaço pausa, `g` alterna 3D e grafo, `p` esconde
  o painel. Clicar numa parte do robô mostra os nós que a usam.
- **A missão na plantação é a real:** as rodadas do caso 2 (uma por método) vêm dos CSVs
  do passo 3; a trilha verde é a pose verdadeira, a colorida é a estimada. No olival o robô
  só segue a rota do caso 4 (ainda não há rodadas medidas).
- **Mudar textos** (nós, roteiro, próximos passos, resultados): `site/js/dados.js`.
- **Se o chassi ou as rodadas mudarem:** `python3 site/gerar_dados.py <pasta dos CSVs>`
  regenera `js/malha_chassi.js` e `js/trajetorias.js` (o navegador não deixa uma página
  aberta do disco ler outros arquivos, por isso os dados vão embutidos em `.js`).

## 10. Legado e comparação

- **SLAM por LiDAR (`slam:=true`, slam_toolbox):** não é mais usado nem corrigido; fica só
  para comparação nos resultados. Na plantação ele perde o yaw nas cabeceiras e o robô vai
  para a entrelinha errada. Com `vslam:=true slam:=true`, o slam_toolbox roda sem TF (mapa em
  `/slam_toolbox/map`) e o RTAB-Map continua dono de `map → odom`.
- **`goal_navigation.launch.py` (`goal_navigator`):** vai até um objetivo do RViz (2D Goal
  Pose) sem Nav2 nem mapa, só desviando do que está à frente. Funciona na arena, não entra
  nas entrelinhas da plantação.
- **Gazebo (`agrobot_gazebo`, `agrobot_control`):** versão antiga, não é mais usada nem
  mantida.

## 11. Limites conhecidos

- Quase tudo foi validado na nuvem Linux, headless e sem GPU; os testes no Windows/WSL e no
  RViz na tela ainda são poucos. A câmera de aquisição só rodou no Webots na nuvem; falta medir o custo no Windows.
- O LiDAR 3D é o sensor mais caro da simulação: sem GPU, o Webots anda a ~0,23× o tempo real
  com ele e sem a câmera (~0,37× sem ele) e a ~0,15× com câmera, RTAB-Map e Nav2. No
  Windows com GPU deve pesar bem menos; `lidar3d:=false` o desliga quando não há VSLAM.
- As especificações do Delta-2A e do Helios-16P vieram de fichas técnicas de revendas (os
  sites dos links não abriram daqui); confira alcance, frequência e ruído com as fichas
  oficiais antes de citar na dissertação.
- O ruído dos sensores (ligado em todas as rodadas da bateria final, nos quatro casos) é branco e gaussiano, menos o bias do giroscópio: faltam buracos e bordas falsas na profundidade, reflexos no LiDAR e deriva por temperatura.
- A interface web só foi conferida na nuvem (Chromium headless); falta abrir no navegador do
  Windows com o WSL.
- O joystick ainda não rodou no Webots (o teclado sim). O salvamento do mapa com nome, o
  aviso "Mapa salvo" e o `map:` nas missões só foram conferidos fora do simulador.
- Ao girar no lugar na arena, a quina do robô pode empurrar a `cardboard box(2)`.
- No olival, o robô escorrega encosta abaixo no declive lateral (nas manobras, nos desvios
  dos obstáculos e ao passar por cima de galhos), e as rodas e o IMU não veem isso. O erro de yaw
  (rumo pelas fileiras), o de distância (raio calibrado) e parte do escorregamento
  (`lateral_correction:=imu`) estão corrigidos (seção 5); sobra 0,4 a 0,8 m de ATE e cerca de 1 m
  de erro máximo, metade dos waypoints a mais de 0,5 m do alvo, e rodadas em que o RTAB-Map
  desvia o `map` 1 a 2 m na volta mesmo com o `odom` bom (caso 4, semente 303, sem correção
  lateral e com `rows`). `--slope 0.03` gera a encosta mais suave (2 a 3%) e `--slope 0` o
  olival plano.
- A calibração das rodas e do escorregamento é feita por quem desenvolve: uma rodada com
  `wheel_calibration:=true` e os valores copiados à mão para `config/wheel_calibration.yaml`,
  medidos contra a pose verdadeira do Webots. **Trabalho futuro:** uma opção de calibração para
  o operador (na interface web): andar uma distância conhecida, ou ida e volta entre dois
  marcos, e o sistema calcula e grava o fator do raio e o ganho do escorregamento daquele robô
  naquele solo, sem editar arquivo e sem a pose verdadeira. Também falta calibrar a plantação
  (lá o VSLAM fecha laço a cada entrelinha de 9 m e o erro de distância é menor).
- O rumo pelas fileiras supõe fileiras retas e paralelas, e ele, a calibração e a correção
  pela inclinação foram validados em 4 rodadas no olival (sementes 101 e 303); falta a bateria
  completa de 5 sementes (e calibrar as sementes 202, 404 e 505).
