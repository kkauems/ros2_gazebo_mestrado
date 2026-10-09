# Manual: Webots + Nav2 for the Agrobot

What changed since commit `f4a0384 agrobot webots v1`, and why. The original plan and
status log are in [instructions.md](instructions.md). This file explains the reasoning
behind the result.

## Goal

Before this work, `agrobot_webots` only listened to `/cmd_vel`. It published no odometry,
TF or sensor data, so nothing could navigate. The goal was the smallest setup where you
**click a point in RViz and the robot drives there, going around obstacles**.

Scope decision: **no map**. No SLAM, AMCL or map_server. Nav2 runs with `odom` as its
global frame, and both costmaps are rolling windows built from the lidar. This is the
minimum Nav2 needs to accept a `NavigateToPose` goal. A `map` frame can be added later
without changing the rest.

## What was done, file by file

### 1. Driver plugin — [agrobot_driver.py](../agrobot_webots/agrobot_webots/agrobot_driver.py)

The driver is a **webots_ros2_driver Python plugin**. `webots_ros2_driver` loads it from
the URDF `<plugin>` tag and calls `init()` once, then `step()` every simulation step. It now
does five jobs.

**a) Motors (as before, plus a separation scale).** `cmd_vel` → left/right wheel speeds
with differential-drive kinematics. Wheel separation is now
`wheelSeparation × wheelSeparationScale`.
*Why:* a skid-steer robot doesn't turn like an ideal differential drive. The wheels drag
sideways, so the robot turns less than the geometry predicts. Treating the track as wider
(scale 1.33, tuned in simulation) makes commanded and actual turn rates match. All three
values are now URDF properties, not hard-coded constants.

**b) Odometry `/odom` + TF `odom → base_link`.** The plugin reads the four wheel
`PositionSensor`s (`*_wheel_joint_sensor`), averages each side, and integrates x and y with
the midpoint method.
*Why:* Nav2 can't run without `odom → base_link`, and the robot had no source for it.

**c) Yaw comes from the IMU, not the wheels.** An `InertialUnit` (`imu`) was added to the
PROTO. When it is present, `d_yaw` comes from the IMU (with its initial offset removed)
instead of `(d_right - d_left) / separation`.
*Why:* wheel-based yaw was the worst error source. Skid-steer slip makes it wrong after
every turn, and one degree of yaw error grows into a large position error over distance.
The IMU yaw matched Webots ground truth. Distance still comes from the wheels.

**d) `/scan` is published by the plugin.** It reads `front_lidar` directly
(`getRangeImage()`) and publishes a `LaserScan` every 3 simulation steps.
*Why:* this was the hardest bug. webots_ros2_driver has its own lidar publisher
(`Ros2Lidar`), but it stamped every scan with **time 0**, because its node clock never
advanced. Nav2's costmap TF message filter **silently dropped** every scan, so the
costmaps stayed empty and the robot drove into crates without any error. Publishing from
the plugin, with the same `robot.getTime()` clock as the odometry, fixed it. For the same
reason, the URDF now disables the driver's own lidar publisher
(`<device reference="front_lidar"><ros><enabled>false</enabled>`).
Details:
- Webots orders rays left to right (+fov/2 → -fov/2). ROS expects counter-clockwise from
  `angle_min`, so the array is reversed.
- The first read is delayed by 2 lidar periods. Reading the range image before the first
  sample exists crashed the controller.
- QoS is `sensor_data` (best effort), the standard for scans and what the costmaps expect.

**e) `/joint_states` for the wheels.** *Why:* `robot_state_publisher` only publishes TF
for continuous joints when it receives their positions. Without this, the wheel frames are
missing and RViz shows the RobotModel with errors.

**Timestamps:** everything is stamped with `robot.getTime()` (Webots sim time), so the
plugin doesn't depend on its private node receiving `/clock`.

**Kept on purpose:** `rclpy.init()` + a private node inside `init()`. The first plan said
to remove them, but this is the official webots_ros2 Python plugin pattern. `webots_node`
only exposes `.robot`, not a ROS node.

**Removed:** `controllers/agrobot_driver/agrobot_driver.py`, a standalone Webots
controller that duplicated the plugin. The world uses `controller "<extern>"`, so only the
plugin runs, and the duplicate just caused confusion. Its install rule in `setup.py` was
removed too.

### 2. Robot model — [agrobot.proto](../agrobot_webots/protos/agrobot.proto) and [agrobot.urdf](../agrobot_description/urdf/agrobot.urdf)

**New low front lidar** (`front_lidar` in the PROTO, `front_lidar_link` in the URDF) at
(0.65, 0, -0.15) from `base_link`: 3.0 rad (~172°) FOV, 300 rays, 0.1–8 m.
*Why:* the existing `lidar_link` sits about 0.57 m above the floor. The crates are 0.3 m
tall and the boxes 0.2 m, so a lidar there sees **nothing** in this arena. The new one is
about 0.14 m above the floor. It sits ahead of the wheels, and its FOV stays under 180°, so
it never hits the robot's own wheels.
*Why a new link instead of moving `lidar_link`:* Gazebo uses the same URDF and its
`gpu_lidar` is attached to `lidar_link`. Leaving that link alone keeps the Gazebo setup
unchanged.

**New `InertialUnit`** in the PROTO (see 1c). The URDF already had an `imu_link` for
Gazebo. The Webots device sits on the robot's root body.

**The URDF `<webots>` block** now passes `wheelRadius`, `wheelSeparation` and
`wheelSeparationScale` to the plugin, and disables the driver's built-in lidar publisher
(see 1d).

### 3. World — [obstacle_arena.wbt](../agrobot_webots/worlds/obstacle_arena.wbt)

- **Four walls** at ±2.55 m (0.4 m tall, 0.1 m thick) around the 5 × 5 m floor.
  *Why:* without walls the robot could drive off the floor, and the lidar had nothing to
  see at the edges.
- **Spawn moved from (0, 0) to (-1.0, -0.3).**
  *Why:* at (0, 0), the robot's 1.2 × 1.12 m footprint overlapped `plastic crate(4)` at
  (0.19, 0.37). It spawned inside an obstacle, and the costmap marked the robot's own
  position as lethal.

Side effect: `odom` starts at the spawn, so goals are relative to (-1.0, -0.3) in world
coordinates.

### 4. Launch — [simulation.launch.py](../agrobot_webots/launch/simulation.launch.py)

New nodes alongside Webots and the driver:
- **`robot_state_publisher`** with the URDF contents and `use_sim_time`. It publishes the
  static TF `base_link → wheels / front_lidar_link / imu_link / ...`. *Why:* Nav2 has to
  transform `/scan` from `front_lidar_link` into `odom`.
- **`navigation.launch.py`**, included when `nav:=true` (default).
- **RViz** with `rviz/nav.rviz`, when `rviz:=true` (default).
- **Shutdown handler:** when the Webots process exits, the whole launch shuts down.
  *Why:* otherwise closing Webots leaves Nav2 and RViz running with no simulation behind
  them.

### 5. Nav2 launch — [navigation.launch.py](../agrobot_webots/launch/navigation.launch.py) (new)

Starts only `controller_server`, `smoother_server`, `planner_server`, `behavior_server`,
`velocity_smoother`, `bt_navigator` and one `lifecycle_manager` (autostart).

*Why not `nav2_bringup/navigation_launch.py`:* in Jazzy it also starts `docking_server`,
`collision_monitor` and `waypoint_follower`. `docking_server` fails to configure without
dock plugins, which aborts the lifecycle manager, so nothing ever becomes active. Writing
our own launch file avoids that and keeps the node set minimal.

Velocity chain (the standard Nav2 one):
`controller_server` / `behavior_server` → `/cmd_vel_nav` → `velocity_smoother` → `/cmd_vel` → plugin.

### 6. Nav2 parameters — [nav2_params.yaml](../agrobot_webots/config/nav2_params.yaml) (new)

| Choice | Why |
|---|---|
| `global_frame: odom` everywhere, rolling-window costmaps, no static layer | No map in this scope |
| Regulated Pure Pursuit, `use_rotate_to_heading`, no reversing | Simple and predictable path follower that suits a skid-steer (it can turn in place) |
| NavFn planner with `allow_unknown` | Most of the rolling global costmap is unknown at the start |
| Footprint 1.2 × 1.12 m | The wheels stick out past the chassis: y = ±0.48 ± 0.07, x = ±0.35 ± 0.22. The first estimate of 1.2 × 0.8 would let the wheels hit obstacles |
| Inflation radius 0.7 m | Must be at least the inscribed radius (~0.56 m) or the planner routes too close to obstacles |
| `min_obstacle_height: -1.0` at **both** layer and source level | The lidar is 0.15 m **below** `base_link`, so its points have z < 0 in `odom`. The default (0) filters them all out. Setting it only on the `scan` source was not enough: clearing worked but nothing was marked. The layer-level value is a separate filter |
| Goal tolerance 0.15 m / 0.3 rad, `stateful` goal checker | Reachable with this odometry quality. Once xy is reached, the robot only rotates to the final heading |
| Velocity limits 0.5 m/s, 1.0 rad/s | Moderate speeds for a 1.2 m robot in a 5 m arena |

### 7. RViz config — [nav.rviz](../agrobot_webots/rviz/nav.rviz) (new)

Fixed frame `odom`, top-down view. Displays: Grid, RobotModel (from
`/robot_description`), TF, LaserScan, global and local costmaps, footprint, global plan,
odometry arrows. The **Nav2 Goal** tool (SetGoal) publishes to `/goal_pose`, and
`bt_navigator` turns that into a `NavigateToPose` goal.

### 8. Packaging — [setup.py](../agrobot_webots/setup.py) and [package.xml](../agrobot_webots/package.xml)

- `setup.py` installs `config/*.yaml` and `rviz/*.rviz`, and no longer installs the
  removed controller.
- `package.xml` adds the new runtime dependencies: `builtin_interfaces`, `launch_ros`,
  `nav_msgs`, `navigation2`, `robot_state_publisher`, `rviz2`, `tf2_ros`. `sensor_msgs` is
  missing even though the driver imports it. It comes in through `navigation2`, but it
  should be listed explicitly.

## How it was verified (2026-10-02)

In Webots, end to end:
1. Teleop with `/cmd_vel`: `/odom` follows the robot. After calibrating the separation
   scale and switching to IMU yaw, odometry yaw matched Webots ground truth.
2. `/scan` arrives at about 10 Hz with sim-time stamps, and the crates appear in both
   costmaps.
3. All lifecycle nodes become `active`.
4. Nav2 goals to (2.2, 1.2) and back to (0, 0) in `odom`: **both SUCCEEDED**. The path went
   around `plastic crate(4)` and no object was pushed.

## Limits and what to do next

- **Position drift:** about 0.3 m true error per goal and about 0.4 m after a round trip.
  The cause is sideways slip in turns, which neither the encoders nor the IMU yaw can see.
  Next step: a `robot_localization` EKF (wheel odom + IMU), then GPS or SLAM/AMCL to get a
  real `map` frame. When that exists, change `global_frame` to `map` in the global costmap
  and `bt_navigator`, and add a static layer if you use a map.
- **Testing gotcha:** killing `ros2 launch` with SIGKILL leaves orphan Nav2 nodes. Their
  `lifecycle_manager` deactivates the next run's nodes. Run `pkill -9 -f nav2_` before
  relaunching.
- The work is **uncommitted**. Only the deletion of the old controller is staged.

## Registro de alterações

### 2026-10-06 — Remoção do `agrobot.urdf.xacro` antigo

**O que mudou**
- Apagado `agrobot_description/urdf/agrobot.urdf.xacro`.
- `README.md`: a linha do layout do repositório deixou de citar o `.xacro`.

**Por quê.** Havia a suspeita de que o plugin de tração diferencial usava medidas de roda
erradas (raio 0,22 m e separação 0,96 m contra 0,165 m e 0,86 m "do modelo"). A conferência
mostrou que o plugin está certo:
- Todos os launches (`spawn_agrobot`, `agrobot_gazebo/simulation`,
  `stability_simulation`, `agrobot_webots/simulation`) carregam `agrobot.urdf`.
- Em `agrobot.urdf` as rodas têm raio 0,22 m e as juntas ficam em y = ±0,48 m
  (separação 0,96 m). O plugin `gz-sim-diff-drive-system`, o `<plugin>` do Webots e o
  `agrobot.proto` usam esses mesmos valores.
- Os valores 0,165 m e 0,86 m (y = ±0,43 m) só existiam no `.xacro`, um modelo antigo e
  simplificado, sem plugin, que nenhum launch carrega.

O `.xacro` foi removido para não gerar esse diagnóstico errado de novo. Nenhum parâmetro de
roda foi alterado. O `agrobot.urdf.backup` foi mantido.

**Se o `/odom` do Gazebo divergir na prática**, a causa provável é o skid-steer: o
`DiffDrive` calcula o giro pelas rodas, que deslizam lateralmente nas curvas. No Webots isso
já foi resolvido com o yaw do IMU e o `wheelSeparationScale` (seção 1).

**Validação.** O `CMakeLists.txt` instala o diretório `urdf/` inteiro, então nada depende
do arquivo pelo nome; `grep` no repositório não encontra outra referência ao `.xacro`.
O `colcon build` não foi executado porque o ambiente de nuvem não tem ROS 2 instalado.

### 2026-10-06 — Agente de refinamento de prompts

**O que mudou**
- Novo arquivo `.claude/agents/prompt-refiner.md`: um subagente do Claude Code chamado
  `prompt-refiner`.

**O que ele faz.** Recebe um pedido bruto (curto, ambíguo ou com vários assuntos) e devolve
um prompt refinado por assunto, com objetivo, contexto (caminhos reais do repositório),
passos, restrições, critérios de pronto, suposições e perguntas abertas. Ele já embute o
contexto fixo do projeto: ROS 2 Jazzy no WSL2, Webots + Nav2 sem mapa, Gazebo Harmonic, os
pacotes `agrobot_*`, a falta de ROS 2 no ambiente de nuvem (o que validar na nuvem e o que
testar localmente) e a regra de registrar toda alteração neste manual.

**Por quê.** Pedidos curtos geram resultados que adivinham escopo e critério de pronto.
Refinar o pedido antes de executar deixa explícitos os defaults escolhidos e como verificar
o resultado.

**Como usar.** No Claude Code, dentro do repositório:
- Explícito: "use o agente prompt-refiner para refinar: <pedido>".
- Ou `/agents` para ver e editar o agente.
O Claude também pode chamá-lo sozinho quando o pedido for vago (critério na `description`
do arquivo). O agente só tem as ferramentas `Read`, `Grep` e `Glob`: lê o repositório, mas
não altera nada. Depois, o prompt refinado pode ser executado na mesma sessão.

**Validação.** Frontmatter YAML conferido (`name`, `description`, `tools`, `model`). Os
caminhos citados no agente e no exemplo existem no repositório. Nenhum código ROS foi
alterado.

### 2026-10-06 — Objetivo pelo RViz sem Nav2 (`goal_navigator`, Webots)

**Pedido.** Iniciar a simulação já com o RViz e o robô, marcar um objetivo no RViz e o
robô ir até ele, sem reconhecimento do mundo (sem mapa nem costmaps do Nav2), apenas
desviando de objetos diretamente à frente.

**Como usar**
```bash
colcon build --symlink-install --packages-select agrobot_description agrobot_webots
source install/setup.bash
ros2 launch agrobot_webots goal_navigation.launch.py      # rviz:=false para não abrir o RViz
```
No RViz, ferramenta **2D Goal Pose** → clicar e arrastar no grid (a seta dá a orientação
final). O Fixed Frame é `odom`, ou seja, as coordenadas são relativas ao ponto onde o robô
nasceu, como no Nav2. O log do `goal_navigator` mostra cada decisão: objetivo novo,
obstáculo no caminho, lado do desvio, caminho livre de novo e chegada.

**O que mudou, arquivo por arquivo**
- `agrobot_webots/agrobot_webots/goal_navigator.py` (novo). Nó que assina `/goal_pose`,
  `/odom` e `/scan` e publica `/cmd_vel` a 10 Hz:
  - **Direto ao objetivo** quando a faixa da largura do robô (0,56 m + 0,10 m de margem para
    cada lado), saindo do centro dele na direção do objetivo, está livre por
    `stop_distance` = 0,6 m à frente da carroceria (ou até o objetivo, se estiver mais
    perto). Gira parado se o erro de direção passa de 0,6 rad; senão anda com
    v = min(0,3; 0,5·dist)·cos(erro) e w = 1,5·erro (limitado a 0,8 rad/s).
  - **Desvio** quando essa faixa está bloqueada: procura, de 5° em 5° dentro do campo de
    visão do LiDAR, a direção livre por `clear_distance` = 1,0 m mais próxima da do
    objetivo, com uma penalidade para trocar de lado. O rumo escolhido é guardado (no frame
    `odom`) enquanto continuar livre, para o robô não ficar virando de um lado para o
    outro. Quando o caminho direto abre de novo, ele volta a mirar o objetivo. Se nenhuma
    direção estiver livre, gira para o lado mais livre até abrir uma.
  - **Memória curta de pontos.** O LiDAR do Webots fica na frente (0,65 m à frente do
    `base_link`) e vê só 3,0 rad (~172°), então não enxerga a lateral nem a traseira. O nó
    guarda os pontos que o LiDAR viu a até 2 m (grade de 5 cm, no frame `odom`) e os
    descarta quando o robô se afasta mais de 3 m deles (ou depois de 5 min). Dentro do campo de visão vale só o
    scan atual. Isso não é um mapa: serve para lembrar o que acabou de passar ao lado do
    robô.
  - **Checagem da carroceria.** Antes de girar parado, o nó simula um giro de 0,25 rad e
    vê se algum ponto cairia dentro do retângulo do robô (1,2 × 1,12 m + margem). Se cair,
    tenta avançar um pouco, recuar um pouco (até 0,8 m sem progresso) ou girar para o
    outro lado. Andando, se a curva levaria a lateral para cima de um ponto, segue reto.
  - **Desistência.** Para e avisa no log se não chega 0,1 m mais perto do objetivo em
    40 s, ou se fica 5 s sem conseguir girar nem andar.
  - **Chegada.** Dentro de 0,25 m gira até a orientação pedida (±0,15 rad). Se não houver
    espaço para girar, para ali e avisa.
  - Velocidades, ganhos, tolerâncias, dimensões do robô, `stop_distance`,
    `clear_distance`, os 40 s de desistência, os 0,8 m de recuo e a memória (2 m, 5 min)
    são parâmetros ROS (`ros2 param list /goal_navigator`). O passo de 5°, o giro simulado
    de 0,25 rad, a grade de 5 cm, o descarte a 1 m além do raio da memória, os 5 s sem
    conseguir girar e os 10 Hz são constantes no código. Os parâmetros são lidos só quando o nó inicia: `ros2 param
    set` com ele rodando não muda nada. Para mudar, ponha-os em `parameters=[{...}]` do
    `goal_navigator` no `goal_navigation.launch.py`, sempre com ponto decimal (`60.0`, não
    `60`: com um inteiro o nó não inicia).
- `agrobot_webots/launch/goal_navigation.launch.py` (novo): inclui o `simulation.launch.py`
  com `nav:=false rviz:=false` (Webots, driver e `robot_state_publisher`, sem Nav2) e
  acrescenta o RViz com `rviz/goal.rviz` e o `goal_navigator` (sem `use_sim_time`; veja
  a entrada "Robô não andava até o objetivo").
- `agrobot_webots/rviz/goal.rviz` (novo): Fixed Frame `odom`, vista de cima, RobotModel,
  LaserScan (`/scan`, Best Effort), Odometry, Pose do objetivo e a ferramenta 2D Goal
  Pose publicando em `/goal_pose`. Sem os costmaps nem o caminho do Nav2.
- `agrobot_webots/setup.py`: entry point `goal_navigator`.
- `agrobot_webots/package.xml`: `sensor_msgs` como dependência de execução (o driver já o
  usava e faltava, conforme a seção 8).
- `README.md`: tabela de estado, layout e seção de uso.

**Por que não reaproveitar o Nav2.** O `simulation.launch.py` com Nav2 já aceita o mesmo
2D Goal Pose (o `bt_navigator` também escuta `/goal_pose`), mas ele monta costmaps do
ambiente e planeja um caminho, que é o "reconhecimento do mundo" que o pedido excluiu. O
modo novo não substitui o Nav2: os dois continuam disponíveis, em launches separados.

**Validação.** O ambiente de nuvem não tem ROS 2 nem Webots, então nada foi rodado no
simulador. O que foi feito:
- `flake8` sem erros no nó e no launch; YAML do RViz conferido com parser.
- O nó foi rodado numa simulação 2D em Python que reproduz a arena do Webots: robô
  diferencial nascendo em (-1,0; -0,3), LiDAR a 0,65 m à frente com 300 raios em 3,0 rad e
  alcance de 8 m, paredes em ±2,5 m, as 4 caixas plásticas (0,3 m) e as 3 de papelão
  (0,2 m), e a carroceria como retângulo de 1,2 × 1,12 m.
- 64 objetivos aleatórios dentro da arena, saindo do ponto inicial: **56 alcançados sem
  encostar em nada**, 7 em que o robô parou com o aviso de "sem chegar mais perto" e 1 em
  que a lateral encostou numa caixa ao contornar. Os 7 casos sem chegada são quase todos
  objetivos ao norte do ponto inicial, atrás do vão entre a caixa 3 e a caixa 4. Esse vão
  tem ~1,3 m, menos que a largura do robô com a margem.
- As primeiras versões mostraram por que a memória e a checagem da carroceria são
  necessárias: sem elas, o robô batia a lateral ou a traseira nas caixas que já tinham
  saído do campo de visão em quase todos os objetivos.

**Limites conhecidos**
- É um desvio reativo, sem planejamento: em becos ou vãos estreitos ele para e avisa em
  vez de achar outro caminho. Para isso existe o modo Nav2.
- A posição é só odometria (rodas + yaw do IMU); os desvios de ~0,3 m por objetivo citados
  acima valem aqui também, e a memória de pontos herda esse erro.

### 2026-10-06 — Malha do chassi não encontrada pelo Webots com `--symlink-install`

**Sintoma.** Com `colcon build --symlink-install`, o Webots avisava
`Unable to find resource at '//wsl.localhost/.../agrobot_webots/meshes/chassis.stl'` e o
robô aparecia sem o chassi.

**Causa.** O `agrobot.proto` aponta para `../meshes/chassis.stl`, relativo à pasta do PROTO.
O `setup.py` copiava a malha de `agrobot_description/meshes/` para
`install/.../share/agrobot_webots/meshes/`. Com `--symlink-install`, porém, o PROTO
instalado é um link para `agrobot_webots/protos/agrobot.proto` no código-fonte. O Webots
(no Windows, lendo o WSL) segue o link e procura a malha em `agrobot_webots/meshes/` do
código-fonte, que não existia. Sem `--symlink-install` funcionava, porque o PROTO é
copiado para junto da malha instalada.

**O que mudou**
- `agrobot_webots/meshes/chassis.stl` (novo): cópia de `agrobot_description/meshes/chassis.stl`
  (440 KB), para o caminho relativo do PROTO valer tanto no código-fonte quanto no
  `install/`. Preferi a cópia a um link simbólico para não depender de como o Webots, no
  Windows, resolve links do WSL.
- `agrobot_webots/setup.py`: instala `meshes/*.stl` do próprio pacote.

**Atenção.** Se a malha do chassi mudar em `agrobot_description`, copie-a também para
`agrobot_webots/meshes/`.

**Validação.** O ambiente de nuvem não tem ROS 2 nem Webots. Foi conferido que o PROTO só
referencia `../meshes/chassis.stl` e que o arquivo agora existe nesse caminho relativo à
pasta `protos/`.

### 2026-10-06 — RViz não abria no `goal_navigation.launch.py`

**Sintoma.** `ros2 launch agrobot_webots goal_navigation.launch.py` abria o Webots, mas não
o RViz.

**Causa.** O launch inclui o `simulation.launch.py` com `rviz:=false`, para não abrir o
`nav.rviz` do Nav2. No `launch` do ROS 2, os argumentos de um `IncludeLaunchDescription`
viram configurações do contexto inteiro e continuam valendo depois do include. Assim, o
`rviz` do próprio `goal_navigation.launch.py` também passava a valer `false`, e o RViz com
`goal.rviz` não era iniciado.

**O que mudou**
- `agrobot_webots/launch/goal_navigation.launch.py`: o include fica dentro de um
  `GroupAction` (que por padrão é `scoped=True`). Os argumentos passados ao include valem
  só dentro do grupo, e o `rviz` deste launch volta ao valor dele (`true`, ou o que for
  passado na linha de comando).

**Validação.** Reproduzido com o pacote `launch` do ROS 2 Jazzy (código do branch `jazzy`
de `ros2/launch`) num launch mínimo com a mesma estrutura: sem o `GroupAction`, a ação
condicionada a `rviz` do arquivo pai não rodava; com ele, rodava, e a do arquivo incluído
continuava desligada. No mesmo teste, o `OnProcessExit` → `Shutdown` registrado dentro do
include (o que fecha tudo quando o Webots fecha) continuou funcionando dentro do grupo.

### 2026-10-06 — Robô não andava até o objetivo (`goal_navigator`)

**Sintoma.** O RViz publicava o objetivo (`Setting goal pose: Frame:odom, ...`), mas o
robô ficava parado.

**Causa.** O `goal_navigator` rodava com `use_sim_time: true`, e o laço de controle era
um timer de 10 Hz no relógio do ROS. Com `use_sim_time`, esse relógio fica parado em 0 até
alguém publicar `/clock`, e nesta simulação ninguém publica: no `webots_ros2_driver`
2025.0.0, o `/clock` sai do nó `Ros2Supervisor`, que só existe com
`WebotsLauncher(..., ros2_supervisor=True)`, e o `simulation.launch.py` não usa essa opção
(conferido em `webots_ros2_driver/webots_launcher.py` e `ros2_supervisor.py` da tag
2025.0.0). O objetivo chegava ao nó, mas o timer nunca disparava e nada era publicado em
`/cmd_vel`. A revisão automática do PR chegou à mesma causa de forma independente.

**O que mudou**
- `goal_navigator.py`: o timer do laço de controle usa o relógio do sistema
  (`Clock(clock_type=ClockType.STEADY_TIME)`), então roda com ou sem `use_sim_time`.
- `goal_navigation.launch.py`: o `goal_navigator` sobe sem `use_sim_time`. O nó não usa o
  tempo do ROS para nada; a memória de pontos usa o carimbo do próprio `/scan`, que é o
  tempo do Webots.
- `goal_navigator.py`, `main()`: o `rclpy.init` passa a ser feito sem os tratadores de
  sinal do rclpy (`SignalHandlerOptions.NO`). No Jazzy, com eles, o Ctrl+C fecha o contexto
  antes do `finally`, o `publish` da parada falha com erro e o último comando de
  velocidade fica valendo no Webots. Agora o Ctrl+C manda velocidade zero antes de sair.
- `README.md`: a nota sobre `use_sim_time` diz que nada publica `/clock` nesta simulação.

**Observação.** Os outros nós (`robot_state_publisher`, Nav2, RViz) também rodam com
`use_sim_time` sem `/clock`. Isso não foi alterado: o modo Nav2 foi testado e funciona
assim, e as mensagens são carimbadas com o tempo do Webots pelo driver.

**Validação.** Assinatura de `Node.create_timer(..., clock=...)`, `Clock(clock_type=...)`,
`rclpy.init(..., signal_handler_options=...)` e `rclpy.try_shutdown` conferidas no código do
`rclpy` branch `jazzy`. `flake8` sem erros. Não foi possível rodar no Webots daqui.

### 2026-10-06 — Desvio do `goal_navigator`: colisões em curva e robô tremendo

Correções de uma revisão do PR. A revisão rodou o nó em sequências de objetivos, em que cada
objetivo começa de onde o anterior terminou e a memória de pontos se acumula. Os testes
anteriores partiam sempre do ponto inicial.

**Problemas encontrados**
- **Colisão ao seguir reto.** Andando, se a curva levaria a lateral para cima de um ponto,
  o nó zerava o giro e seguia reto sem conferir se reto também batia. Quando batia, o robô
  entrava na caixa.
- **Colisão no meio da curva.** A checagem da curva olhava só a posição final do trecho
  simulado (0,5 s à frente). Numa curva fechada, o canto traseiro passava por cima de uma
  caixa no meio do trecho e saía do outro lado, e a checagem não via.
- **Robô tremendo.** O giro para o lado do objetivo batia e não dava para avançar nem
  recuar. O nó então girava um passo (0,1 s) para o outro lado. No passo seguinte o
  controle pedia de novo o lado do objetivo, ainda bloqueado. O robô ficava alternando
  ±0,8 rad/s no mesmo lugar até a desistência de 40 s. O aviso de "Robô cercado" (5 s)
  nunca disparava.
- **Manual.** A entrada "Objetivo pelo RViz sem Nav2" dizia que todos os números eram
  parâmetros ROS e omitia o limite de 0,8 rad/s em w.

**O que mudou**
- `agrobot_webots/agrobot_webots/goal_navigator.py`
  - `motion_hits()` (novo): confere a carroceria em 1/4, 1/2, 3/4 e no fim do trecho de
    0,5 s, e não só no fim.
  - Andando: se a curva bate, confere o movimento reto. Se reto também bate, o robô não
    anda e passa a girar parado, com a checagem de giro de antes.
  - `rotate()`: ao trocar para o outro lado do giro, o lado fica travado (`turn_sign`) até
    o robô voltar a andar. Se esse lado também bloquear e não houver como avançar ou
    recuar, o nó conta os 5 s e para com "Robô cercado". A trava é zerada a cada objetivo
    novo.
  - A mensagem de "Robô cercado" agora diz que ele não consegue continuar girando nem
    manobrar para liberar o giro, e pede outro objetivo.
- `claude/manual.md`: a entrada "Objetivo pelo RViz sem Nav2" foi corrigida. Ela agora diz
  que w é limitado a 0,8 rad/s, separa os parâmetros ROS das constantes do código e explica
  como mudar os parâmetros.
- `README.md`: a seção do `goal_navigator` cita a parada por "Robô cercado".

**Validação.** Sem ROS 2 nem Webots na nuvem, tudo foi testado na simulação 2D da arena em
Python, a mesma da entrada original.
- **64 objetivos saindo do ponto inicial.**
  - Antes: 56 alcançados, 7 paradas e 1 colisão.
  - Agora: 56 alcançados, 8 paradas e 0 colisões.
- **15 sequências de 5 objetivos (75 objetivos), sem erro de odometria.**
  - Antes: 44 alcançados, 23 paradas e 8 colisões.
  - Agora: 49 alcançados, 23 paradas e 3 colisões.
  - As trocas de sentido de giro caíram de 6505 para 348 no total.
  - Cada parada leva em média 21 s, em vez de 42 s.
- **As mesmas sequências com erro de odometria simulado (escorregamento nas curvas).**
  - Colisões caíram de 7 para 4.
  - Alcançados caíram de 38 para 33. Parte dos alcançados de antes veio depois de uma
    colisão: na simulação 2D a caixa não segura o robô, e ele atravessa e segue.
- **As 3 colisões que sobraram.** Duas são pontos cegos do LiDAR. A face da caixa que fica
  ao lado da carroceria nunca entrou no campo de visão, então não está na memória: numa,
  o robô bate ao girar na chegada; na outra, numa curva. A terceira é o objetivo seguinte a
  uma dessas, que já começa encostado. Pontos cegos só se resolvem com sensores laterais
  ou traseiros.
- **Onde estão as paradas.** Elas se concentram em lugares em que o robô fica entre duas
  caixas, sem espaço para girar até o lado do objetivo. Em algumas sequências os objetivos
  seguintes também param, porque ele continua no mesmo lugar apertado. Nesse caso, tire-o
  dali com o teleop (seção "Manual driving" do README) e marque o objetivo de novo.
- **Lint.** `flake8` sem erros.
### 2026-10-06 — `prompt-refiner`: Webots como alvo padrão, Gazebo como legado

**O que mudou** (`.claude/agents/prompt-refiner.md`)
- O contexto fixo agora diz que o **Webots (`agrobot_webots`) é o simulador oficial**: todo
  pedido mira o Agrobot do Webots, mesmo sem citar o simulador, e é concluído lá.
- `agrobot_gazebo` e `agrobot_control` passaram a ser descritos como **legado**: o agente não
  os propõe como alvo nem como exemplo, a menos que o pedido cite o Gazebo pelo nome.
- O agente só lê os pacotes do Gazebo quando o pedido citar o Gazebo.
- O exemplo do agente foi reescrito: o desvio reativo de obstáculos agora mira o Webots
  (nó novo em `agrobot_webots`, `/scan` do plugin, launch com `nav:=false`).

**Por quê.** O Kauê usa Windows; o Gazebo era uma versão antiga que não está mais em uso.
A primeira versão do agente tratava o Gazebo como opção atual e usava um exemplo no Gazebo,
o que levaria outros agentes a trabalhar no simulador errado.

**Validação.** Frontmatter YAML conferido; os caminhos citados no exemplo existem.
Nenhum código ROS foi alterado.

### 2026-10-07 — `colcon build` falhava com "File exists" em `meshes/chassis.stl`

**Sintoma.** Depois de atualizar para o PR, `colcon build --symlink-install
--packages-select agrobot_webots` falhava com
`error: [Errno 17] File exists: '.../build/agrobot_webots/meshes/chassis.stl' ->
'.../install/agrobot_webots/share/agrobot_webots/meshes/chassis.stl'`.

**Causa.** É uma sobra do build antigo, não um erro do `setup.py` novo. No `main`, o
`setup.py` instalava a malha a partir de `../agrobot_description/meshes/chassis.stl`. Com
`--symlink-install`, o colcon deixou em `install/.../meshes/chassis.stl` um link para a malha
do `agrobot_description`. O `setup.py` novo instala `agrobot_webots/meshes/chassis.stl`, que
é outro arquivo (mesmo conteúdo, outro inode). O comando `symlink_data` do colcon-core (a
partir da 0.16.0) chama `os.symlink`, e ele falha porque o destino já existe; o colcon não
apaga links antigos nesse caso. Só acontece em workspaces compilados com
`--symlink-install` antes da entrada "Malha do chassi não encontrada pelo Webots"; um
clone novo compila sem erro.

**Como resolver (uma vez por workspace)**
```bash
cd ~/projects/ros2_gazebo_mestrado
rm install/agrobot_webots/share/agrobot_webots/meshes/chassis.stl
colcon build --symlink-install --packages-select agrobot_webots
source install/setup.bash
```
Se der outro erro, ou ao trocar de branch (por exemplo, voltar para o `main`) ou alternar
entre build com e sem `--symlink-install`, limpe o pacote:
`rm -rf build/agrobot_webots install/agrobot_webots` e compile de novo. O colcon não remove
links de arquivos que saíram do pacote, e eles quebram o build seguinte.

**Observação.** A malha é o último item de `data_files`. No build que falhava, o script
`goal_navigator`, o `goal_navigation.launch.py` e o `goal.rviz` já tinham sido instalados
antes do erro, e o link velho aponta para uma malha idêntica. Por isso os testes anteriores
rodaram o código novo mesmo com o colcon mostrando `Failed`.

**O que mudou**
- `README.md`: item em "Known issues" com o erro e os dois comandos.
- Nenhuma mudança de código. Voltar a instalar a malha de `../agrobot_description` traria
  de volta o erro do Webots com `--symlink-install` e criaria o conflito inverso para quem
  já compilou a versão nova. Renomear a malha evitaria o erro, mas deixaria um link órfão
  em cada workspace existente, por um problema que acontece uma vez só.

**Validação.** Reproduzido num workspace com o mesmo layout (pacotes na raiz), Python 3.12
e setuptools 68.1.2 (versões do Ubuntu 24.04), com colcon-core 0.16.0, 0.18.4, 0.20.1 e
0.21.3. Em todas: o build do `main` com `--symlink-install` seguido do build do PR deu o
mesmo `[Errno 17]`, e tanto o `rm` do arquivo quanto o `rm -rf` do pacote resolveram. O
build seguinte também passou, e o `meshes/chassis.stl` instalado passou a apontar para
`agrobot_webots/meshes/chassis.stl`. A colcon-core 0.15.2 não tem `symlink_data` (copia os
arquivos) e não dá o erro.

### 2026-10-07 — EKF do `robot_localization` (rodas + IMU), `/clock` e estudo de VSLAM

**Pedido.** Usar o Nav2 para planejar a rota, com a odometria vinda do `robot_localization`
(EKF com encoders das rodas e IMU, sugestão do orientador), e estudar o uso de VSLAM com a
câmera RGB-D que o robô vai ter.

**Ponto de partida.** O Nav2 já existia no Webots (`simulation.launch.py`, seções 4 a 6
acima), sem mapa e com `odom` como frame global. A odometria era calculada dentro do driver
(distância das rodas + yaw do IMU). O que faltava era o EKF e um relógio coerente.

**Como usar**
```bash
sudo apt install ros-jazzy-robot-localization
cd ~/projects/ros2_gazebo_mestrado
colcon build --symlink-install --packages-select agrobot_description agrobot_webots
source install/setup.bash
ros2 launch agrobot_webots simulation.launch.py ekf:=true       # Nav2 + EKF
ros2 launch agrobot_webots goal_navigation.launch.py ekf:=true  # goal_navigator + EKF
```
Sem `ekf:=true` tudo funciona como antes (o driver publica `/odom`). Conferir:
`ros2 topic hz /clock`, `ros2 topic echo /imu/data --once`, `ros2 topic hz /odom` (~30 Hz
com o EKF) e `ros2 run tf2_ros tf2_echo odom base_link`.

**O que mudou, arquivo por arquivo**
- `agrobot_webots/agrobot_webots/agrobot_driver.py`
  - **`/clock`** com o tempo do Webots (`robot.getTime()`) a cada passo. *Por quê:* nada
    publicava `/clock` (sem `Ros2Supervisor`), e os nós com `use_sim_time` ficavam com o
    relógio parado em 0. O `ekf_node` nem começa assim: ele fica em
    `Waiting for clock to start...` (visto no teste). Também era a causa do `/scan` com
    carimbo 0 do `Ros2Lidar` (seção 1d) e afetaria a câmera RGB-D, que o
    `webots_ros2_driver` publica com o relógio do nó dele.
  - **`/imu/data`** (`sensor_msgs/Imu`, frame `imu_link`, QoS `sensor_data`): orientação do
    `InertialUnit`, velocidade angular do `Gyro` e aceleração do `Accelerometer`, com
    variâncias fixas (constantes no topo do arquivo). Mensagem pulada enquanto algum sensor
    devolve NaN.
  - **`/wheel/odom`**: odometria só das rodas, inclusive o yaw. É a entrada do EKF. vy = 0
    com variância pequena (o robô não anda de lado) e variância alta em wz (skid-steer).
  - **Propriedade `publishOdom`** (padrão `true`). Com `false`, o driver não publica `/odom`
    nem o TF `odom → base_link`, porque quem publica é o EKF. Com `true`, o `/odom` é o de
    antes (rodas + yaw do IMU).
- `agrobot_webots/protos/agrobot.proto`: `Gyro` (`gyro`) e `Accelerometer`
  (`accelerometer`) ao lado do `InertialUnit`. Os três passaram para (0, 0, 0,35), a posição
  do `imu_link` do URDF. A leitura de orientação não muda com a posição.
- `agrobot_description/urdf/agrobot.urdf`: `<publishOdom>true</publishOdom>` no bloco
  `<webots><plugin>`. O Gazebo ignora esse bloco.
- `agrobot_webots/config/ekf.yaml` (novo): `ekf_filter_node`, 30 Hz, `two_d_mode`,
  `world_frame: odom`, publica o TF `odom → base_link`.
  - Rodas (`/wheel/odom`): só vx e vy. O yaw das rodas fica de fora: no skid-steer ele erra
    a cada curva (foi por isso que o driver passou a usar o IMU, seção 1c).
  - IMU (`/imu/data`): yaw e velocidade angular em z. `imu0_relative: true` zera o yaw na
    primeira leitura, como o driver fazia, então o `odom` continua nascendo alinhado com a
    frente do robô e os objetivos valem igual.
  - Aceleração linear fora: dupla integração de acelerômetro piora a posição mais do que
    ajuda com rodas disponíveis.
- `agrobot_webots/launch/simulation.launch.py`: argumento `ekf` (padrão `false`). Com
  `true`, inicia o `ekf_node` (saída `odometry/filtered` remapeada para `/odom`) e entrega
  ao driver uma cópia do URDF com `publishOdom` = `false`, gravada em
  `/tmp/agrobot_webots_ekf.urdf`. O driver do Webots lê as propriedades do plugin do URDF,
  e não de parâmetros ROS; por isso a cópia (um `OpaqueFunction` decide na hora do launch).
  Como o EKF publica `/odom`, o Nav2 e o `goal_navigator` não precisaram de nenhuma mudança.
- `agrobot_webots/launch/goal_navigation.launch.py`: repassa o mesmo argumento `ekf`.
- `agrobot_webots/package.xml`: `robot_localization` e `rosgraph_msgs`.
- `claude/estudo_vslam.md` (novo): estudo de VSLAM com RGB-D (resumo abaixo).
- `README.md`: estado, dependências (`ros-jazzy-robot-localization`), argumento `ekf`,
  tópicos novos, nota do `/clock`, próximos passos.

**Por que `ekf:=false` continua o padrão.** O modo antigo foi testado no Webots e funciona;
o novo só foi testado fora dele (veja Validação). Depois de testar, se der certo, vale trocar
o padrão para `true`.

**Por que sem `map → odom` agora.** Só rodas e IMU não dão referência global, então não há
o que publicar em `map`. O Nav2 continua em `odom`. Quem vai publicar `map → odom` é o VSLAM
(ou SLAM com o LiDAR, ou GPS), e aí o `global_costmap` e o `bt_navigator` passam para `map`.

**Estudo de VSLAM (resumo de `claude/estudo_vslam.md`).** Viável. Recomendação: RTAB-Map
(`ros-jazzy-rtabmap-ros`, tem pacote no Jazzy, roda em CPU) com a odometria do EKF como
entrada, publicando `map → odom` e o `/map` 2D para o Nav2. Riscos: banda das imagens entre
o Webots no Windows e o WSL (começar em 320 × 240 a 10 Hz e medir), arena sem textura (VSLAM
precisa de features) e as condições de campo. O `/clock` deste PR é pré-requisito. O estudo
traz o plano em 6 passos e a comparação com ORB-SLAM3, Isaac ROS e slam_toolbox.

**Validação.** O ambiente de nuvem não tem Webots. O que foi feito:
- ROS 2 Jazzy instalado via RoboStack (conda) com `robot_localization` 3.8.3 e Nav2
  1.3.12, as mesmas séries do apt do Jazzy.
- O **driver real** (`AgrobotDriver`) rodou contra um robô falso em Python que imita a API
  do Webots: motores, encoders, `InertialUnit`, `Gyro`, `Accelerometer` e o LiDAR com
  raios contra as paredes e caixas da arena. A cinemática tem escorregamento: o giro real
  usa uma bitola efetiva maior que a do driver (1,45 contra 1,33) e o avanço real é 4% menor
  que o das rodas. Junto rodaram `robot_state_publisher`, o EKF com `config/ekf.yaml` e o
  `navigation.launch.py` com o `nav2_params.yaml` do repositório.
- Resultado com `ekf:=true`: todos os nós ativos, sem erro de TF nem descarte de `/scan`.
  Os objetivos (2,2; 1,2), (0; 0) e (3,0; -1,0) em `odom` deram **SUCCEEDED**. O erro
  entre a pose verdadeira e o `/odom` ficou em 0,10 a 0,12 m nos pontos mais distantes e
  quase 0 de volta à origem. É o escorregamento de 4% no avanço, que nem rodas nem IMU
  enxergam.
- Com `ekf:=false` na mesma simulação, os números foram praticamente iguais. Esperado: o
  driver já usava o yaw do IMU, e o IMU simulado não tem ruído. O ganho do EKF é de
  estrutura: pesa cada sensor pela covariância e aceita novas fontes (VSLAM, GPS) sem mexer
  no driver.
- Nos dois modos, o Nav2 reclamou de "collision ahead" perto das caixas e abortou objetivos
  colados nas paredes. Isso é da sintonia do Nav2 (já existente) e da simulação simplificada,
  não do EKF.
- O `simulation.launch.py` e o `goal_navigation.launch.py` foram executados com o
  `webots_ros2_driver` substituído por stubs: com `ekf:=true` o `ekf_node` sobe e o driver
  recebe o URDF com `publishOdom` = `false`; com `ekf:=false`, o URDF original e nenhum EKF.
- `flake8` sem erros no driver e nos launches.
- **Falta testar no Webots:** que o `Gyro` e o `Accelerometer` do PROTO aparecem, que o
  `/clock` não atrapalha o modo Nav2 antigo, e o EKF com o robô real do simulador.

### 2026-10-07 — Círculos roxos no RViz com `ekf:=true`

**Sintoma.** Com `simulation.launch.py ekf:=true`, o RViz (`nav.rviz`) desenhava círculos
roxos enormes sobrepostos, que cobriam os costmaps e o LiDAR.

**Causa.** São as elipses de covariância de posição do display **Odometry** (`/odom`). No
`nav.rviz` a opção `Covariance` não estava definida, e o padrão do RViz é ligada, com uma
elipse para cada uma das 50 setas guardadas (`Keep: 50`). Com a odometria do driver isso
não aparecia, porque ele publica uma covariância fixa e pequena (0,01 m²). O EKF publica a
covariância real: como ele só recebe velocidades das rodas e o yaw do IMU, sem nenhuma
medida absoluta de posição, a incerteza de x e y cresce sem limite enquanto o robô anda.
Isso é o comportamento correto do filtro (é a deriva que o VSLAM vai corrigir), não um erro.

**O que mudou**
- `agrobot_webots/rviz/nav.rviz`: `Covariance: Value: false` no display Odometry, como já
  estava no `goal.rviz`. As setas da odometria continuam.

**Para ver a covariância de novo**, marque `Odometry → Covariance` no painel do RViz. Para
acompanhar o crescimento sem o RViz: `ros2 topic echo /odom --field pose.covariance --once`
(índices 0 e 7 são as variâncias de x e y).

**Validação.** YAML conferido com parser; o nome da propriedade (`Covariance`) conferido no
`librviz_default_plugins.so` do Jazzy. Não foi possível abrir o RViz aqui.

### 2026-10-08 — Covariância do EKF: círculos que só cresciam

**Pergunta (reunião com o orientador).** Com `ekf:=true`, as elipses de covariância do
`/odom` no RViz (1) continuavam crescendo mesmo com o robô perto de uma parede, onde se
esperava que diminuíssem, e (2) eram círculos, sem diferença entre x e y.

**Causa de (2), os círculos: ruído de processo padrão.** O `ekf.yaml` não definia
`process_noise_covariance`, então o `robot_localization` usava o padrão do pacote
(`filter_base.cpp:110-125` da 3.8.3): 0,05 em x e 0,05 em y. A cada passo de predição o
filtro faz `P = J·P·Jᵀ + Q·Δt` (`ekf.cpp:431-436`), com Q no frame `odom`, sem rotação e,
com `dynamic_process_noise_covariance` desligado (padrão), também com o robô parado. Esse
termo soma 0,05 m²/s igual em x e em y: σ = 0,22·√t m, ou 0,7 m em 10 s e 1,7 m em 1 min.
A parte que dependeria do movimento (incerteza das velocidades das rodas e do yaw do IMU,
propagada pelo modelo) é cerca de 300 vezes menor e some no meio. Resultado: círculos
iguais em x e y, crescendo com o tempo, andando ou parado.

**Causa de (1): nenhuma entrada do EKF enxerga a parede.** O EKF recebe só vx e vy das
rodas e yaw e velocidade de yaw do IMU. Nenhuma delas mede posição. O LiDAR alimenta só os
costmaps do Nav2 e o `goal_navigator`, não a localização. Sem medida de posição, a
variância de x e y só pode crescer. Isso não é erro: no REP-105 o frame `odom` é contínuo e
deriva por definição. A elipse só diminui perto de paredes quando uma localização compara o
que o sensor vê com um mapa: AMCL (precisa de mapa pronto), SLAM com o LiDAR
(slam_toolbox) ou VSLAM (RTAB-Map), publicando `map → odom` e uma pose com covariância no
frame `map`. Mesmo assim, perto de **uma** parede reta ela estreita só na direção
perpendicular à parede (a distância fica conhecida, a posição ao longo dela não): vira uma
elipse comprida paralela à parede, não um círculo menor. Num canto, ela fica pequena nos
dois eixos.

**O que mudou**
- `agrobot_webots/config/ekf.yaml`: `process_noise_covariance` explícito, com x e y em
  0 (os outros valores são os padrões do pacote), e `dynamic_process_noise_covariance:
  true`. A incerteza de posição passa a vir só da incerteza das velocidades e do yaw,
  propagada pelo modelo de movimento: cresce com o movimento e na direção dele. Definir
  a matriz também evita um bug da 3.8.3 em que o serviço `reset` do nó zera Q quando o
  parâmetro não está no arquivo.
- `agrobot_webots/agrobot_webots/agrobot_driver.py`: o desvio-padrão das velocidades em
  `/wheel/odom` deixou de ser fixo (0,071 m/s em vx e 0,032 m/s em vy, mesmo parado) e
  passou a ser um piso de 0,002 m/s mais uma parte proporcional ao movimento: 10% de |v|
  em vx; 5% de |v| mais 0,05 m/s por rad/s de giro em vy (deriva lateral do skid-steer).
  É a ideia do modelo de odometria do *Probabilistic Robotics* (erro proporcional ao
  movimento). Sem isso, mesmo com Q de x/y zerado, a elipse continuaria crescendo com o
  robô parado, por causa da incerteza fixa das velocidades. Os quatro números são
  constantes no topo do arquivo.
- `claude/img/ekf_covariancia_antes_depois.png` (novo): figura da medição abaixo.

**Medição** (driver real + robô falso em Python + `ekf_node` 3.8.3, mesmo roteiro nos dois
casos: 10 s parado, 6 m em linha reta no eixo x, giro de 90°, 6 m no eixo y, 16 s
parado; desvio-padrão de 1σ):

| | Antes | Depois |
|---|---|---|
| Parado, 10 s | σx = σy = 0,71 m | σx = σy = 0,008 m |
| Fim da 1ª reta (eixo x) | σx = σy = 1,10 m | σx = 0,030 m, σy = 0,020 m |
| Fim da 2ª reta (eixo y) | σx = σy = 1,49 m | σx = σy = 0,037 m |
| Parado mais 16 s no fim | sobe para 1,73 m | sobe cerca de 0,001 m |

Na 1ª reta a elipse fica comprida no sentido do movimento (x); na 2ª, y cresce mais e
alcança x, e no fim de um trajeto em L ela volta a ficar quase redonda, porque acumulou
erro de avanço nos dois eixos. Com Nav2 (4 objetivos), todos SUCCEEDED, com o mesmo erro
de posição de antes.

**Limite importante para a dissertação.** O EKF trata o erro das rodas como ruído branco,
e a covariância cresce com √distância. Um erro sistemático (raio de roda errado,
escorregamento constante) cresce linearmente e o filtro não o representa: no robô falso,
com 4% de escorregamento no avanço, o erro real chegou a 0,11 m com σ = 0,03 m. Os
coeficientes de 10% e 5% são um ponto de partida; o certo é calibrá-los comparando o
`/odom` com a pose verdadeira do Webots (Supervisor), por exemplo exigindo que o erro
fique dentro de 2σ na maior parte do tempo.

**Para ver as elipses de novo no RViz**, marque `Odometry → Covariance`. Agora elas têm
poucos centímetros; aumente `Covariance → Position → Scale` para enxergá-las ao lado do
robô.

### 2026-10-08 — SLAM com o LiDAR (`slam_toolbox`) e a elipse no frame `map`

**Pedido.** Depois da explicação de que a elipse do `/odom` não tem como diminuir (nenhuma
entrada do EKF mede posição), Kauê pediu para configurar o slam_toolbox, para a elipse
diminuir perto das paredes.

**O que mudou** (tudo só com `slam:=true`; o padrão `slam:=false` não muda nada)
- `agrobot_webots/config/slam_toolbox.yaml` (novo): slam_toolbox 2.8.5 em modo mapping com
  o `/scan`. Monta o `/map`, publica `map → odom` e, a cada scan processado, `/pose`
  (`PoseWithCovarianceStamped` no frame `map`) com a covariância do scan matching.
  `base_frame: base_link` (o padrão do pacote, `base_footprint`, não existe no URDF).
  Processa um scan a cada 0,2 m ou 0,2 rad de giro, no máximo a cada 0,25 s.
- `agrobot_webots/config/ekf_map.yaml` (novo): um segundo `ekf_node`, `ekf_map_node`, no
  frame `map`. Funde vx e vy das rodas, a velocidade de yaw do IMU e x, y e yaw do `/pose`,
  e publica `/odometry/map`. Não publica TF: `map → odom` continua sendo do slam_toolbox.
- `agrobot_webots/launch/simulation.launch.py`: argumento `slam` (padrão `false`). Sobe o
  slam_toolbox (nó lifecycle: o launch o configura e ativa) e o `ekf_map_node`, com os
  serviços `set_pose`, `reset`, `enable` e `toggle` remapeados para `/ekf_map_node/...` para
  não colidir com o `ekf_filter_node`. Com `slam:=true` o RViz abre `nav_slam.rviz`.
- `agrobot_webots/rviz/nav_slam.rviz` (novo): cópia do `nav.rviz` com Fixed Frame `map`, o
  `/map`, o robô translúcido e a elipse do `/odometry/map` (só a pose atual, ampliada 20×).
  Há também a elipse do `/odom` para comparar, desligada; ela só faz sentido com `ekf:=true`
  (com `ekf:=false` o driver publica covariância fixa de 0,01 m², um disco constante).
- `agrobot_webots/package.xml`: dependências `slam_toolbox` e `lifecycle_msgs`.
- `README.md`: `apt install ros-jazzy-slam-toolbox`, argumento `slam`, tópicos novos.
- `claude/img/slam_elipse_campo_arena.png` (novo): figura da medição abaixo.

**Como a elipse se forma.** O slam_toolbox compara cada scan com os 10 scans anteriores
numa busca grossa de ±0,25 m (passo de 2 cm, refinada depois a 1 cm) e ±20°. A
covariância de x e y é o espalhamento das posições da busca grossa que casam quase tão bem
quanto a melhor (`Mapper.cpp:874-966`), com piso de σ = 6,3 mm: se as
paredes vistas prendem uma direção, ela é estreita nessa direção; ao longo de uma parede
reta todas as posições casam igual e ela fica longa (limitada pela janela de busca); sem
nenhum ponto no alcance vale 500 m². O `ekf_map_node` funde essa pose: entre um scan e
outro a elipse cresce com o movimento; a cada scan ela encolhe na direção que as paredes
prendem.

**Ruído de processo de x e y no `ekf_map_node`: 0,01, não 0 como no `ekf.yaml`.** Com 0, o
filtro confia demais nas rodas (o modelo delas não tem o escorregamento sistemático) e
quase ignora o LiDAR. Medição na arena (mesmo roteiro, robô falso com 4% de
escorregamento):

| x/y em `process_noise_covariance` | Erro médio / máximo | σ da elipse | Erro dentro de 2σ |
|---|---|---|---|
| 0 (como no `ekf.yaml`) | 3,7 / 7,4 cm | 0,6 a 1,7 cm | 15% das amostras |
| 0,01 (escolhido) | 1,7 / 5,2 cm | 1,5 a 4,9 cm | 100% |
| 0,05 (padrão do pacote) | 2,1 / 6,7 cm | 3,4 a 8,7 cm | 100% |

O valor é escalado pela velocidade (`dynamic_process_noise_covariance`): parado, essa folga
não soma nada, e a elipse só cresce alguns milímetros por minuto, como a do `/odom`. Ele deve ser calibrado com a pose verdadeira do Webots, como os coeficientes
das rodas.

**Medição** (driver real, `simulation.launch.py` real com o Webots trocado por um robô falso
em Python, slam_toolbox 2.8.5, `ekf_map_node` 3.8.3; figura
`claude/img/slam_elipse_campo_arena.png`):

| Situação | Elipse (1σ) | Erro real |
|---|---|---|
| Arena 5 × 5 m, área aberta | cerca de 3 × 3 cm | 0,5 a 5 cm |
| Arena, de frente para a parede leste (frente do robô a 0,3 m) | 2,6 cm perpendicular × 4,5 cm ao longo | menos de 1 cm |
| Campo aberto, 4,4 m sem nada no alcance | cresce até 12 × 16 cm | de 0 a 15 cm |
| Campo, a parede (a 13 m) entra no alcance | perpendicular cai de 12 para 4 cm | continua 17 a 22 cm |
| Campo, andando 5 m ao longo da parede | 6 × 8 cm | sobe até 75 cm |
| Campo, de costas para a parede | volta a crescer, até 23 cm | 52 cm |

Com Nav2 e `slam:=true`, os 4 objetivos de teste dados no frame `map` terminaram em
SUCCEEDED, com erro do `/odometry/map` de 1 a 2 cm na chegada. Com `slam:=false` o Nav2 se
comporta como antes: nos testes, às vezes uma das 4 metas aborta porque o controlador vê
colisão à frente ("collision ahead"), e isso também acontece no commit anterior a esta
mudança.

**O que a elipse mostra (para a conversa com o orientador)**
1. Na arena ela não diminui perto das paredes, porque o LiDAR (8 m) vê pelo menos duas
   paredes de qualquer ponto de uma arena de 5 m: a elipse fica em ~3 cm em todo lugar.
   O que muda é a forma: de frente para uma parede ela fica estreita na direção da parede
   e comprida ao longo dela (o scan matching sozinho dá 2,6 × 14 cm ali).
2. A intuição "perto da parede o robô sabe melhor onde está" aparece quando há lugares
   sem nada no alcance: no campo aberto a elipse cresce e, quando a parede entra no
   alcance, encolhe na direção perpendicular a ela.
3. A elipse é a incerteza em relação ao mapa, não ao ponto de partida. No campo, a parede
   foi mapeada com os 18 cm de erro que o robô já tinha quando a viu pela primeira vez; o
   robô fica bem localizado em relação à parede mapeada, mas o erro em relação à origem
   não cai. Isso é próprio do SLAM: só um mapa ou referência externa (GPS, marco
   conhecido) corrige o erro anterior.
4. Ao longo de uma parede longa e uniforme o scan matching não tem o que segurar e
   desliza: 0,5 a 0,9 m em 5 m nos testes, pior que as rodas, enquanto a elipse diz
   6 a 8 cm. A covariância do slam_toolbox é local (comparação com os últimos scans), não
   acumula esse deslize. Isso importa para o vinhedo: uma fileira longa é o mesmo caso.

**Escolhas e armadilhas**
- slam_toolbox dono de `map → odom` e o `ekf_map_node` só para a elipse: a elipse é a
  mesma que se o EKF publicasse o TF, sem risco de dois nós publicarem `map → odom`. Para
  o arranjo de dois EKFs do REP-105, troque `publish_tf: true` no `ekf_map.yaml` e
  `transform_publish_period: 0.0` no `slam_toolbox.yaml`.
- `minimum_travel_distance: 0.2`, não 0. Com 0 o slam_toolbox processa scans com o robô
  parado; nos testes a pose "andou" 10 a 16 cm em 40 s parada, e cada scan repetido faria
  a elipse encolher sem informação nova.
- O EKF do mapa usa só a velocidade de yaw do IMU: o yaw absoluto vem do slam_toolbox.
  Duas fontes de yaw absoluto brigariam no filtro.
- `smooth_lagged_data` com `history_length: 1.0`: a pose do slam_toolbox chega atrasada
  (carimbo do scan); o filtro volta no tempo e reaplica rodas e IMU.
- No RViz, `Keep: 1` e tolerâncias 0 no display da elipse: com os padrões (0,1 m e
  0,1 rad) ela congela com o robô parado. `Scale: 20` desenha 20σ, porque 1σ (~3 cm) some
  embaixo do robô de 1,2 m.
- O Nav2 continua planejando no frame `odom`. Um objetivo clicado no RViz (frame `map`) é
  convertido para `odom` uma vez, quando o Nav2 o aceita; se o SLAM corrigir `map → odom`
  depois, o robô vai para o ponto antigo.

**Para usar**
```bash
sudo apt install ros-jazzy-slam-toolbox
cd ~/projects/ros2_gazebo_mestrado
git pull
colcon build --symlink-install --packages-select agrobot_webots
source install/setup.bash
ros2 launch agrobot_webots simulation.launch.py ekf:=true slam:=true
```
No RViz, a elipse azul-clara é a do `/odometry/map`. Para salvar o mapa:
`ros2 run nav2_map_server map_saver_cli -f ~/mapa_arena`.
