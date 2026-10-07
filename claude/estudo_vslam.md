# Estudo: VSLAM com câmera RGB-D no Agrobot (Webots + ROS 2 Jazzy)

Data: 2026-10-07. Pergunta: dá para usar VSLAM com a câmera RGB-D que o robô vai ter,
junto com o Nav2 e o EKF do `robot_localization`, no setup atual (ROS 2 Jazzy no WSL2 e
Webots no Windows)?

**Resposta curta: sim.** A opção recomendada é o **RTAB-Map** (`rtabmap_ros`), usando a
odometria do EKF como entrada. Ele publica o `map → odom` que hoje não existe, gera o mapa 2D
para o Nav2 e um mapa 3D. O maior risco não é o algoritmo, e sim o desempenho da câmera
simulada na ligação WSL2 ↔ Windows, que precisa ser medido antes (passo 2 do plano).

## 1. Onde o VSLAM entra na arquitetura

```
rodas (/wheel/odom) ─┐
                     ├─ EKF (robot_localization) ─► odom → base_link  (local, contínuo, deriva)
IMU (/imu/data) ─────┘
RGB-D + odom do EKF ─── VSLAM (RTAB-Map) ────────► map → odom        (global, corrige a deriva)
                                         └───────► /map (grade 2D) → camada static do Nav2
```

É a divisão do REP-105, a mesma que o `robot_localization` documenta: o EKF fica com o
`odom → base_link` (suave, sem saltos, bom para o controlador) e quem tem referência global
publica `map → odom`. Nada do que foi feito agora precisa ser desfeito. Quando o VSLAM entrar:
- o Nav2 passa a usar `global_frame: map` no `global_costmap` e no `bt_navigator`;
- o `global_costmap` ganha a camada `static_layer` lendo o `/map` do RTAB-Map;
- o `local_costmap` continua em `odom`.

Opcional, depois: a odometria visual do RTAB-Map (`rgbd_odometry`) pode entrar no EKF como
`odom1` (só velocidades). Isso ajuda quando as rodas patinam, que é o caso de solo agrícola.

## 2. Opções avaliadas

| Opção | RGB-D | Jazzy (apt) | GPU | Mapa para o Nav2 | Avaliação |
|---|---|---|---|---|---|
| **RTAB-Map** (`ros-jazzy-rtabmap-ros`) | Sim | Sim (pacote lançado no Jazzy) | Não precisa | Sim: `/map` 2D e nuvem 3D | **Recomendada.** Aceita odometria externa (EKF), fecha laços, tem modo de localização num mapa salvo. Integração com Nav2 documentada |
| ORB-SLAM3 | Sim | Não (compilar do código, wrappers da comunidade) | Não | Não (só pontos esparsos) | Bom para comparar precisão na dissertação, mas dá muito mais trabalho e não entrega mapa para navegar |
| NVIDIA Isaac ROS Visual SLAM (cuVSLAM) | Sim (e estéreo) | Não (Docker da NVIDIA) | **Exige GPU NVIDIA com CUDA** | Não diretamente | Só vale se a máquina tiver GPU NVIDIA; no WSL2 adiciona CUDA no WSL + Docker. Não recomendo agora |
| slam_toolbox (LiDAR 2D) | Não usa câmera | Sim | Não | Sim | Não é VSLAM, mas é a forma mais rápida de ter `map → odom` com o LiDAR que já existe. Serve de base de comparação |

O que foi conferido: o `rtabmap_ros` tem release para o Jazzy no `rosdistro` (branch
`jazzy-devel`) e o README dele lista o Jazzy/Ubuntu 24.04. As demais linhas (GPU do Isaac ROS,
falta de pacote do ORB-SLAM3) são do meu conhecimento, não conferidas hoje.

## 3. A câmera no Webots

O Webots monta uma RGB-D como um par `Camera` + `RangeFinder`. Já existem PROTOs prontos no
Webots R2025a, por exemplo o **Orbbec Astra** (`projects/devices/orbbec/protos/Astra.proto`:
640 × 480, FOV 1,04 rad, profundidade de 0,6 a 8 m, ruído configurável) e o Kinect. Também dá
para declarar os dois dispositivos direto no `agrobot.proto`.

O `webots_ros2_driver` publica `Camera` e `RangeFinder` sozinho (imagem, `camera_info` e
profundidade), sem código nosso. Antes deste PR isso não funcionaria: ele carimba as
mensagens com o relógio do nó, que ficava parado em 0 sem `/clock` (o mesmo problema que o
LiDAR teve). **O `/clock` que o driver passou a publicar resolve isso também para a câmera.**
Isso é pré-requisito para qualquer VSLAM, que depende de sincronizar imagem, profundidade e
odometria pelo carimbo de tempo.

## 4. Riscos no setup WSL2 + Webots no Windows

1. **Banda entre Windows e WSL.** O controlador roda no WSL e recebe cada imagem do Webots
   por TCP. Uma RGB-D em 640 × 480 a 15 Hz é cerca de 1,2 MB (cor) + 1,2 MB (profundidade
   float) por quadro, uns 35 MB/s. Deve funcionar, mas a simulação pode ficar abaixo do
   tempo real (estimativa minha, precisa ser medida). Começar com 320 × 240 a 10 Hz, que é
   suficiente para o RTAB-Map. Com o `/clock`, rodar mais lento que o tempo real não quebra
   nada, só demora mais.
2. **Mundo sem textura.** VSLAM precisa de pontos visuais. A arena atual tem chão e paredes
   lisos e poucas caixas, e vai falhar por falta de features. Será preciso texturizar o
   chão e as paredes ou montar um cenário agrícola (fileiras, troncos, vegetação).
3. **Ambiente agrícola real.** Texturas repetitivas (fileiras iguais), folhas que mexem com o
   vento, luz do sol forte e poeira degradam o VSLAM. Isso é tema de dissertação por si só:
   vale medir onde ele falha e quanto o EKF segura nesses trechos.
4. **CPU.** O RTAB-Map roda em CPU; em 320 × 240 a carga é moderada. Webots, Nav2, RViz e
   RTAB-Map juntos na mesma máquina podem disputar CPU.
5. **DDS no WSL.** O próprio README do RTAB-Map recomenda o Cyclone DDS
   (`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`) quando os tópicos de imagem ficam lentos.

## 5. Plano sugerido (cada passo é testável sozinho)

1. **Câmera RGB-D no robô.** `Camera` + `RangeFinder` no PROTO (ou o PROTO do Astra),
   `camera_link` e `camera_optical_frame` no URDF, conferir `/camera/...` e `camera_info` no
   RViz.
2. **Medir o desempenho no WSL.** Fator de tempo real do Webots com a câmera ligada, em
   320 × 240 e 640 × 480. Se ficar muito lento, reduzir resolução ou taxa.
3. **Texturizar o mundo** (ou criar o cenário agrícola).
4. **RTAB-Map em modo mapeamento** (`sudo apt install ros-jazzy-rtabmap-ros`), com
   `odom_frame_id: odom` (do EKF), `subscribe_rgbd`, sincronização aproximada e publicação
   de `map → odom`. Conduzir o robô e salvar o banco do mapa.
5. **Nav2 sobre o mapa.** `global_frame: map`, `static_layer` lendo `/map`, RTAB-Map em
   modo localização.
6. **Avaliação para a dissertação.** Pose verdadeira do Webots (Supervisor) contra a do EKF
   sozinho, a do EKF + VSLAM e a do slam_toolbox. Métrica: erro absoluto de trajetória (ATE).

## 6. O que a câmera RGB-D dá além do VSLAM

Mesmo sem VSLAM, a profundidade serve para o costmap do Nav2: obstáculos fora do plano do
LiDAR (mais altos, mais baixos, galhos) entram como nuvem de pontos numa `VoxelLayer` ou
`spatio_temporal_voxel_layer`. Isso tende a ser útil num ambiente não estruturado, onde o
LiDAR 2D baixo atual não vê tudo.
