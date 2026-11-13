// Conteúdo do site: nós ROS 2, roteiro da apresentação e próximos passos.
// Para mudar um texto, edite aqui; o resto do site lê destes objetos.
// Fonte dos fatos: claude/manual.md, launch/simulation.launch.py,
// config/*.yaml e os planejamentos do projeto.

// Partes do robô que o site sabe destacar no 3D.
window.PARTES = {
  chassi: 'Chassi e quadro',
  rodas: 'Rodas e encoders',
  lidar: 'LiDAR frontal',
  rgbd: 'Câmera RGB-D (VSLAM)',
  camdados: 'Câmera RGB de aquisição',
  imu: 'IMU',
};

// Grupos de nós (cores no grafo e na lista).
window.GRUPOS = {
  sim: { nome: 'Simulação e ponte', cor: '#8b95a7' },
  loc: { nome: 'Localização', cor: '#4aa3ff' },
  nav: { nome: 'Navegação (Nav2)', cor: '#2fbf71' },
  missao: { nome: 'Missão e dados', cor: '#f2c94c' },
  op: { nome: 'Operador', cor: '#f29e38' },
  vis: { nome: 'Visualização', cor: '#b08cff' },
  legado: { nome: 'Só comparação', cor: '#6b6f7a' },
};

// Nós. x, y: posição no grafo (viewBox 1160 x 660). partes: o que acende no
// robô. camadas: o que liga no 3D quando o nó é escolhido.
window.NOS = [
  {
    id: 'webots', nome: 'Webots R2025a', grupo: 'sim', x: 20, y: 300,
    pacote: 'simulador (Windows)', quando: 'sempre',
    faz: 'Simula a física (rodas, atrito, terreno) e o que cada sensor mede. É o "mundo real" de mentira: plantação ou olival, com o robô do PROTO <code>agrobot.proto</code>.',
    entradas: ['velocidade das rodas (do driver)'],
    saidas: ['leituras dos sensores, pose verdadeira (Supervisor)'],
    porque: 'Roda bem no Windows, onde o Kauê trabalha; o Gazebo ficou como versão antiga.',
    arquivo: 'agrobot_webots/worlds/*.wbt, protos/agrobot.proto',
    partes: ['chassi', 'rodas', 'lidar', 'rgbd', 'camdados', 'imu'], camadas: [],
  },
  {
    id: 'driver', nome: 'agrobot_driver', grupo: 'sim', x: 210, y: 300,
    pacote: 'agrobot_webots (plugin do webots_ros2_driver)', quando: 'sempre',
    faz: 'A ponte entre o Webots e o ROS 2. A cada passo de 32 ms converte <code>/cmd_vel</code> em velocidade das 4 rodas e publica todos os sensores com o tempo da simulação.',
    entradas: ['/cmd_vel'],
    saidas: ['/clock', '/wheel/odom', '/imu/data', '/scan', '/joint_states', '/vslam_camera/*', '/data_camera/*', '/ground_truth/odom'],
    porque: 'Os publicadores do webots_ros2_driver carimbavam com o relógio do nó (parado ou atrasado): o Nav2 descartava os scans e o RTAB-Map não casava cor com profundidade. Aqui tudo sai com robot.getTime(). Se o /cmd_vel parar por 1 s, o robô para.',
    arquivo: 'agrobot_webots/agrobot_webots/agrobot_driver.py',
    partes: ['rodas', 'lidar', 'rgbd', 'camdados', 'imu'], camadas: ['lidar'],
  },
  {
    id: 'rsp', nome: 'robot_state_publisher', grupo: 'sim', x: 210, y: 480,
    pacote: 'robot_state_publisher', quando: 'sempre',
    faz: 'Lê o URDF e publica os frames fixos: onde cada sensor está em relação ao <code>base_link</code>.',
    entradas: ['URDF (agrobot.urdf)', '/joint_states'],
    saidas: ['TF base_link → sensores e rodas', '/robot_description'],
    porque: 'O Nav2 e o RTAB-Map projetam os dados dos sensores pelo TF; por isso o PROTO e o URDF têm que bater nas poses dos sensores.',
    arquivo: 'agrobot_description/urdf/agrobot.urdf',
    partes: ['chassi'], camadas: ['tf'],
  },
  {
    id: 'sync', nome: 'rgbd_sync', grupo: 'loc', x: 400, y: 150,
    pacote: 'rtabmap_sync', quando: 'vslam:=true (ou checagem de patinagem)',
    faz: 'Junta a imagem colorida, a profundidade e o camera_info do mesmo instante numa mensagem só.',
    entradas: ['/vslam_camera/color/image_raw', '/vslam_camera/aligned_depth_to_color/image_raw', 'camera_info'],
    saidas: ['/rtabmap/rgbd_image'],
    porque: 'O RTAB-Map precisa de cor e profundidade do mesmo quadro; o driver já publica as duas com o mesmo carimbo.',
    arquivo: 'config/rtabmap.yaml',
    partes: ['rgbd'], camadas: ['rgbd'],
  },
  {
    id: 'vo', nome: 'rgbd_odometry', grupo: 'loc', x: 400, y: 60,
    pacote: 'rtabmap_odom', quando: 'ekf:=true camera:=true (slip_check, ligado por padrão)',
    faz: 'Odometria visual: estima quanto a câmera andou entre dois quadros. Não entra direto no EKF; serve para conferir as rodas.',
    entradas: ['/rtabmap/rgbd_image'],
    saidas: ['/vo/odom'],
    porque: 'Quando o robô fica preso e as rodas giram em falso, só a câmera percebe que ele não saiu do lugar.',
    arquivo: 'config/rtabmap.yaml',
    partes: ['rgbd'], camadas: ['rgbd'],
  },
  {
    id: 'slip', nome: 'wheel_slip_monitor', grupo: 'loc', x: 400, y: 330,
    pacote: 'agrobot_webots', quando: 'ekf:=true camera:=true (slip_check)',
    faz: 'Compara a velocidade das rodas com a da câmera. Normalmente repassa as rodas sem mudar; se discordam (patinando ou arrastado), troca a velocidade das rodas pela da câmera até voltarem a concordar.',
    entradas: ['/wheel/odom', '/vo/odom'],
    saidas: ['/wheel/odom_checked (para o EKF)', '/wheel/slip'],
    porque: 'Nem o EKF nem a IMU percebem rodas girando em falso: parado e velocidade constante dão a mesma aceleração (zero).',
    arquivo: 'agrobot_webots/agrobot_webots/wheel_slip_monitor.py',
    partes: ['rodas', 'rgbd'], camadas: [],
  },
  {
    id: 'ekf', nome: 'ekf_filter_node', grupo: 'loc', x: 590, y: 330,
    pacote: 'robot_localization', quando: 'ekf:=true',
    faz: 'Filtro de Kalman estendido a 30 Hz, em 2D. Junta a velocidade das rodas com o yaw e a velocidade de giro da IMU e publica uma pose suave e contínua.',
    entradas: ['/wheel/odom (ou /wheel/odom_checked)', '/imu/data'],
    saidas: ['/odom', 'TF odom → base_link'],
    porque: 'No skid-steer as rodas escorregam nas curvas, então o yaw vem da IMU e a distância vem das rodas. Nenhuma entrada mede posição: o odom deriva por definição, e quem corrige é o VSLAM.',
    arquivo: 'config/ekf.yaml',
    partes: ['rodas', 'imu'], camadas: ['trilhas'], metodo: 'ekf',
  },
  {
    id: 'rtabmap', nome: 'rtabmap (VSLAM)', grupo: 'loc', x: 590, y: 150,
    pacote: 'rtabmap_slam', quando: 'vslam:=true (exige ekf:=true e camera:=true)',
    faz: 'SLAM visual, a 1 Hz, em 3DoF. Cria um nó do mapa por segundo com a imagem RGB-D e o scan, e, quando reconhece um lugar perto de onde o EKF acha que está, corrige a deriva. Grava o mapa num banco .db; com vslam_localization:=true localiza num mapa salvo.',
    entradas: ['/rtabmap/rgbd_image', '/scan', 'TF odom → base_link (do EKF)'],
    saidas: ['TF map → odom', '/rtabmap/map', 'dados/vslam/<mapa>.db'],
    porque: 'Usa a odometria do EKF (sem odometria visual, economiza CPU). Fechamento só por proximidade: as entrelinhas são iguais para a câmera e para o LiDAR, e a detecção global dava fechamentos falsos de 1,8 m. Visual + ICP com o /scan e otimização robusta (Vertigo) derrubam os que sobram.',
    arquivo: 'config/rtabmap.yaml',
    partes: ['rgbd', 'lidar'], camadas: ['rgbd', 'lidar', 'trilhas'], metodo: 'ekf+vslam',
  },
  {
    id: 'mission', nome: 'waypoint_mission', grupo: 'missao', x: 780, y: 40,
    pacote: 'agrobot_webots', quando: 'waypoints:=<caso> ou waypoints:=rviz',
    faz: 'Lê a lista de waypoints (YAML ou cliques no RViz) e manda tudo para o FollowWaypoints do Nav2, com parada de 3 s em cada ponto. Grava a pose estimada e a verdadeira em CSV para medir o erro.',
    entradas: ['config/waypoints/*.yaml ou /clicked_point', '/ground_truth/odom (só para medir)'],
    saidas: ['ação FollowWaypoints', '/waypoints/markers', 'dados/missions/*_trajetoria.csv'],
    porque: 'As coordenadas são do frame da partida, então a mesma lista vale com e sem VSLAM. A missão é recusada se o mundo ou o mapa não baterem.',
    arquivo: 'agrobot_webots/agrobot_webots/waypoint_mission.py',
    partes: [], camadas: ['waypoints', 'trilhas'],
  },
  {
    id: 'wpf', nome: 'waypoint_follower', grupo: 'nav', x: 780, y: 130,
    pacote: 'nav2_waypoint_follower', quando: 'nav:=true',
    faz: 'Vai a um waypoint por vez (NavigateToPose); para 3 s em cada um e, se um falhar, pula para o próximo.',
    entradas: ['ação FollowWaypoints'], saidas: ['ação NavigateToPose'],
    porque: 'A parada é onde entra a coleta de dados.',
    arquivo: 'config/nav2_params.yaml',
    partes: [], camadas: ['waypoints'],
  },
  {
    id: 'bt', nome: 'bt_navigator', grupo: 'nav', x: 780, y: 220,
    pacote: 'nav2_bt_navigator', quando: 'nav:=true',
    faz: 'Árvore de comportamento do Nav2: pede um caminho ao planner, manda o controller segui-lo e chama as recuperações quando algo dá errado.',
    entradas: ['ação NavigateToPose'], saidas: ['ComputePathToPose', 'FollowPath', 'recuperações'],
    porque: 'Com vslam:=true trabalha no frame map; sem, no odom.',
    arquivo: 'config/nav2_params.yaml',
    partes: [], camadas: ['caminho'],
  },
  {
    id: 'planner', nome: 'planner_server', grupo: 'nav', x: 780, y: 310,
    pacote: 'nav2_planner (NavFn)', quando: 'nav:=true',
    faz: 'Planeja o caminho global no costmap global (janela de 24 × 24 m centrada no robô, em map com VSLAM).',
    entradas: ['/scan', 'TF map → base_link'], saidas: ['/plan'],
    porque: 'NavFn com allow_unknown. Sem camada estática: objetivos a mais de ~12 m são recusados.',
    arquivo: 'config/nav2_params.yaml',
    partes: ['lidar'], camadas: ['caminho', 'lidar'],
  },
  {
    id: 'controller', nome: 'controller_server', grupo: 'nav', x: 780, y: 400,
    pacote: 'nav2_controller (Regulated Pure Pursuit)', quando: 'nav:=true',
    faz: 'Segue o caminho a 20 Hz no costmap local (4 × 4 m, sempre em odom), com giro no lugar. Até 0,5 m/s e 1,0 rad/s.',
    entradas: ['/plan', '/odom', '/scan'], saidas: ['/cmd_vel_nav'],
    porque: 'Fica em odom (REP-105): os saltos do map → odom atrapalhariam o seguimento. Footprint 1,2 × 1,12 m e inflação de 0,7 m.',
    arquivo: 'config/nav2_params.yaml',
    partes: ['lidar', 'rodas'], camadas: ['costmap', 'lidar', 'caminho'],
  },
  {
    id: 'behavior', nome: 'behavior_server', grupo: 'nav', x: 780, y: 490,
    pacote: 'nav2_behaviors (e o smoother_server, do nav2_smoother)', quando: 'nav:=true',
    faz: 'Recuperações (girar, dar ré, esperar) e suavização do caminho, chamadas pela árvore do bt_navigator.',
    entradas: ['pedidos do bt_navigator'], saidas: ['/cmd_vel_nav'],
    porque: 'Launch próprio do Nav2 (e não o nav2_bringup): no Jazzy ele sobe o docking_server, que falha sem plugins.',
    arquivo: 'launch/navigation.launch.py',
    partes: [], camadas: [],
  },
  {
    id: 'velsm', nome: 'velocity_smoother', grupo: 'nav', x: 970, y: 400,
    pacote: 'nav2_velocity_smoother', quando: 'nav:=true',
    faz: 'Limita aceleração e velocidade do comando do Nav2 antes de chegar ao robô.',
    entradas: ['/cmd_vel_nav'], saidas: ['/cmd_vel (ou /cmd_vel_nav_out com teleop)'],
    porque: 'Evita trancos nas rodas e no terreno irregular.',
    arquivo: 'config/nav2_params.yaml',
    partes: ['rodas'], camadas: [],
  },
  {
    id: 'mux', nome: 'cmd_vel_mux', grupo: 'op', x: 970, y: 490,
    pacote: 'agrobot_webots', quando: 'teleop:=joy ou teleop:=keyboard',
    faz: 'Junta o comando do operador e o do Nav2 num /cmd_vel só. O operador tem prioridade; 0,5 s depois que ele solta, o Nav2 volta sem cancelar a missão.',
    entradas: ['/cmd_vel_teleop', '/cmd_vel_nav_out'], saidas: ['/cmd_vel', '/cmd_vel_mux/source'],
    porque: 'Dá para tomar o controle no meio de uma missão e devolvê-lo. É assim que o primeiro mapa do campo real vai ser feito.',
    arquivo: 'agrobot_webots/agrobot_webots/cmd_vel_mux.py',
    partes: ['rodas'], camadas: [],
  },
  {
    id: 'teleop', nome: 'joystick / teclado', grupo: 'op', x: 970, y: 580,
    pacote: 'joy + teleop_twist_joy, ou teleop_keyboard', quando: 'teleop:=joy | keyboard',
    faz: 'Joystick: segure LB/L1 para andar (homem-morto). Teclado (num terminal próprio): cada tecla muda a velocidade e o robô a mantém, como piloto automático.',
    entradas: ['joystick ou teclado'], saidas: ['/cmd_vel_teleop'],
    porque: 'O mapa do campo real vai ser feito com um operador conduzindo, sem GPS.',
    arquivo: 'config/teleop_joy.yaml, agrobot_webots/teleop_keyboard.py',
    partes: [], camadas: [],
  },
  {
    id: 'recorder', nome: 'data_recorder', grupo: 'missao', x: 400, y: 560,
    pacote: 'agrobot_webots', quando: 'data_camera:=true (data_record:=true)',
    faz: 'Grava as fotos da câmera de aquisição em PNG e, para cada uma, a pose estimada da câmera (TF map → data_camera_optical_frame no instante da foto), a pose verdadeira e o erro entre as duas. Só grava se a câmera andou 0,3 m desde a última.',
    entradas: ['/data_camera/image_raw', '/data_camera/camera_info', 'TF', '/ground_truth/odom'],
    saidas: ['dados/fotos/<mundo>_<modo>_<data>/*.png', 'fotos.csv', 'camera_info.yaml'],
    porque: 'Cada foto fica georreferenciada: dá para saber de que planta ela é e medir quanto a localização errou ali.',
    arquivo: 'agrobot_webots/agrobot_webots/data_recorder.py',
    partes: ['camdados'], camadas: ['camdados'],
  },
  {
    id: 'rviz', nome: 'RViz / rtabmap_viz', grupo: 'vis', x: 590, y: 580,
    pacote: 'rviz2, rtabmap_viz', quando: 'rviz:=true (rtabmap_viz:=true)',
    faz: 'Mostra o mapa, os costmaps, o caminho, os waypoints e as câmeras. É onde se clica um objetivo (Nav2 Goal) ou os waypoints (Publish Point).',
    entradas: ['quase tudo'], saidas: ['/goal_pose', '/clicked_point'],
    porque: 'A grade /rtabmap/map só é montada enquanto alguém a assina: deixe o RViz aberto no mapeamento.',
    arquivo: 'rviz/nav_vslam.rviz',
    partes: [], camadas: ['tf'],
  },
  {
    id: 'slam', nome: 'slam_toolbox', grupo: 'legado', x: 590, y: 480,
    pacote: 'slam_toolbox (+ ekf_map_node)', quando: 'slam:=true (só para comparação)',
    faz: 'SLAM só com o LiDAR (scan matching). Com vslam:=true junto, roda sem publicar TF.',
    entradas: ['/scan'], saidas: ['/map, map → odom (sem VSLAM)'],
    porque: 'Ótimo na arena (0,03 m), mas perde o yaw nas fileiras iguais da plantação. Não é mais usado nem corrigido; fica como base de comparação.',
    arquivo: 'config/slam_toolbox.yaml',
    partes: ['lidar'], camadas: ['lidar', 'trilhas'], metodo: 'ekf+slam',
  },
];

// Ligações do grafo: [de, para, tópico, tracejada?]
window.LIGACOES = [
  ['webots', 'driver', 'sensores e motores'],
  ['driver', 'sync', '/vslam_camera/*'],
  ['sync', 'vo', 'rgbd_image'],
  ['sync', 'rtabmap', '/rtabmap/rgbd_image'],
  ['vo', 'slip', '/vo/odom'],
  ['driver', 'slip', '/wheel/odom'],
  ['slip', 'ekf', '/wheel/odom_checked'],
  ['driver', 'ekf', '/imu/data'],
  ['driver', 'rtabmap', '/scan'],
  ['ekf', 'rtabmap', 'TF odom→base_link'],
  ['rsp', 'rtabmap', 'TF dos sensores'],
  ['rtabmap', 'planner', 'TF map→odom, /rtabmap/map'],
  ['ekf', 'controller', '/odom'],
  ['driver', 'planner', '/scan'],
  ['driver', 'controller', '/scan'],
  ['mission', 'wpf', 'FollowWaypoints'],
  ['wpf', 'bt', 'NavigateToPose'],
  ['bt', 'planner', 'ComputePathToPose'],
  ['bt', 'controller', 'FollowPath'],
  ['bt', 'behavior', 'recuperações'],
  ['controller', 'velsm', '/cmd_vel_nav'],
  ['velsm', 'mux', '/cmd_vel_nav_out'],
  ['teleop', 'mux', '/cmd_vel_teleop'],
  ['mux', 'driver', '/cmd_vel'],
  ['driver', 'recorder', '/data_camera/image_raw'],
  ['ekf', 'recorder', 'TF da câmera'],
  ['driver', 'mission', '/ground_truth/odom', true],
  ['driver', 'slam', '/scan', true],
];

// Roteiro da apresentação. camera: posição pronta da câmera 3D. camadas:
// o que fica ligado. destaque: partes do robô. no: nó escolhido no grafo.
window.ROTEIRO = [
  {
    titulo: 'O Agrobot',
    texto: '<p>Robô agrícola de quatro rodas com tração <b>skid-steer</b> (gira como um trator de esteira), simulado no <b>Webots</b> e controlado por <b>ROS 2 Jazzy</b>.</p><p>Objetivo: percorrer plantações e olivais, parar em pontos definidos e <b>fotografar as plantas</b>, sabendo onde cada foto foi tirada.</p><ul><li>1,2 × 1,12 m, rodas de 0,44 m de diâmetro com garras</li><li>bitola de 0,96 m (cabe nas entrelinhas de 1,8 m)</li><li>37 kg no modelo</li></ul>',
    camera: 'geral', camadas: ['rotulos'], destaque: ['chassi', 'rodas'], mundo: 'plantacao', parado: true,
  },
  {
    titulo: 'Os sensores',
    texto: '<p>Cada sensor tem uma função própria:</p><ul><li><b>LiDAR frontal</b> (13 cm do chão, 172°, 8 m): obstáculos do Nav2 e conferência do VSLAM</li><li><b>IMU</b>: o ângulo (yaw) do robô</li><li><b>Encoders das rodas</b>: a distância percorrida</li><li><b>Câmera RGB-D</b> (como a RealSense D455, 15° para baixo): o VSLAM</li><li><b>Câmera RGB de aquisição</b> (0,9 m do chão, olhando para a esquerda): as fotos das plantas</li></ul>',
    camera: 'sensores', camadas: ['rotulos', 'lidar', 'rgbd', 'camdados'], destaque: ['lidar', 'rgbd', 'camdados', 'imu'], mundo: 'plantacao', parado: true,
  },
  {
    titulo: 'A ponte: agrobot_driver',
    texto: '<p>O <b>driver</b> é o único que conversa com o Webots. A cada passo de 32 ms ele lê os sensores, publica nos tópicos do ROS 2 e aplica o <code>/cmd_vel</code> nos motores.</p><p>Tudo sai carimbado com o <b>tempo da simulação</b>. Sem isso o Nav2 descartava os scans e o RTAB-Map não casava cor com profundidade.</p>',
    camera: 'geral', camadas: ['rotulos', 'lidar'], destaque: ['rodas', 'lidar', 'rgbd', 'camdados', 'imu'], no: 'driver', mundo: 'plantacao',
  },
  {
    titulo: 'Onde estou? 1ª camada: EKF',
    texto: '<p>O <b>filtro de Kalman estendido</b> junta a distância das rodas com o ângulo da IMU, 30 vezes por segundo.</p><p>É como andar de olhos fechados contando passos: a pose é suave, mas o erro <b>acumula</b>. Na rodada mostrada (caso 2), o erro médio (ATE) foi de 0,23 m.</p><p><span class="chip" style="--c:var(--verdade)">verde</span> = onde o robô estava de verdade; <span class="chip" style="--c:var(--ekf)">laranja</span> = onde o EKF achava que estava.</p>',
    camera: 'cima', camadas: ['trilhas', 'fantasma'], destaque: ['rodas', 'imu'], no: 'ekf', metodo: 'ekf', mundo: 'plantacao',
  },
  {
    titulo: '2ª camada: VSLAM (RTAB-Map)',
    texto: '<p>O <b>RTAB-Map</b> usa a câmera RGB-D, conferida pelo LiDAR, para montar um mapa e <b>reconhecer lugares</b> por onde já passou. Quando reconhece, corrige o erro acumulado do EKF.</p><p>Escolha importante: só aceita reconhecer lugares <b>perto</b> de onde o EKF acha que está, porque as entrelinhas são todas iguais.</p><p>Resultado: ATE de <b>0,13 m</b> na rodada mostrada, cerca de metade do EKF.</p>',
    camera: 'segue', camadas: ['trilhas', 'fantasma', 'rgbd', 'lidar'], destaque: ['rgbd', 'lidar'], no: 'rtabmap', metodo: 'ekf+vslam', mundo: 'plantacao',
  },
  {
    titulo: 'A árvore de frames',
    texto: '<p><code>map → odom → base_link → sensores</code></p><ul><li><b>odom → base_link</b>: o EKF. Suave, sem saltos, mas deriva.</li><li><b>map → odom</b>: o RTAB-Map. É a correção da deriva.</li><li><b>base_link → sensores</b>: fixo, do URDF.</li></ul><p>O controlador do Nav2 usa o <code>odom</code> (não gosta de saltos); o planejador usa o <code>map</code>.</p>',
    camera: 'geral', camadas: ['tf', 'rotulos'], destaque: ['chassi'], no: 'rsp', mundo: 'plantacao', parado: true,
  },
  {
    titulo: 'Navegação: Nav2',
    texto: '<p>O <b>Nav2</b> é o motorista:</p><ul><li><b>NavFn</b> planeja o caminho no mapa</li><li><b>Regulated Pure Pursuit</b> segue o caminho (até 0,5 m/s, gira no lugar)</li><li>os <b>costmaps</b>, montados com o LiDAR, marcam obstáculos e mantêm 0,7 m de folga</li></ul><p>O quadrado em volta do robô é o costmap local (4 × 4 m); a linha tracejada, o caminho até o próximo waypoint.</p>',
    camera: 'segue', camadas: ['costmap', 'caminho', 'lidar', 'waypoints'], destaque: ['lidar'], no: 'controller', metodo: 'ekf+vslam', mundo: 'plantacao',
  },
  {
    titulo: 'Missão de waypoints',
    texto: '<p>O <code>waypoint_mission</code> lê uma lista de pontos (YAML ou cliques no RViz) e a entrega ao Nav2. O robô <b>para 3 s em cada ponto</b>: é ali que entra a coleta de dados.</p><p>Ao mesmo tempo grava a pose estimada e a <b>pose verdadeira</b> (do simulador), o que permite medir o erro de cada método.</p><p>Aqui: caso 2, ida pela entrelinha do meio e volta pela vizinha (~26 m).</p>',
    camera: 'cima', camadas: ['waypoints', 'trilhas', 'caminho'], destaque: [], no: 'mission', metodo: 'ekf+vslam', mundo: 'plantacao',
  },
  {
    titulo: 'Aquisição de dados',
    texto: '<p>A <b>câmera RGB de aquisição</b> (1280 × 960, uma foto por segundo) olha para a fileira à esquerda. A 0,9 m da fileira, cada foto cobre ~1,8 m de comprimento por 1,35 m de altura.</p><p>O <code>data_recorder</code> grava cada foto com a <b>pose estimada</b> e a <b>verdadeira</b> da câmera, para saber de que planta ela é e quanto a localização errou ali.</p><p class="aviso">Código pronto, ainda não rodado no Webots.</p>',
    camera: 'lado', camadas: ['camdados', 'rotulos'], destaque: ['camdados'], no: 'recorder', metodo: 'ekf+vslam', mundo: 'plantacao',
  },
  {
    titulo: 'Operador no comando',
    texto: '<p>Com <code>teleop:=joy</code> ou <code>keyboard</code>, um operador pode conduzir o robô. O <code>cmd_vel_mux</code> dá prioridade a ele; quando ele solta, o Nav2 volta sem cancelar a missão.</p><p>É assim que vai ser feito o <b>primeiro mapa do campo real</b>: um operador conduz com o RTAB-Map mapeando, e depois marca os waypoints em cima do mapa salvo.</p>',
    camera: 'geral', camadas: ['rotulos'], destaque: ['rodas'], no: 'mux', mundo: 'olival',
  },
  {
    titulo: 'Os mundos de teste',
    texto: '<p><b>Plantação</b>: 6 fileiras de milho de 9 m, entrelinhas de 1,8 m, terreno com camalhões, buracos, torrões e pedras. Fileiras iguais confundem o LiDAR.</p><p><b>Olival</b> (planalto do Douro Internacional): oliveiras em grade de 7 × 7 m numa encosta de 2 a 3%. Lugar aberto: o LiDAR só vê troncos esparsos.</p>',
    camera: 'alto', camadas: ['lidar'], destaque: [], mundo: 'olival',
  },
  {
    titulo: 'Resultados até aqui',
    texto: '<p>32 rodadas comparando três métodos de localização:</p><ul><li><b>Na plantação</b>, o VSLAM reduziu o erro do EKF <b>pela metade</b> (ATE 0,13 a 0,15 m) e atingiu 15/15 waypoints.</li><li>O <b>SLAM por LiDAR</b> se perdeu nas fileiras repetitivas (ATE acima de 2 m, yaw errado em até 51°): por isso a câmera.</li></ul><p>Troque o método no painel do 3D para ver as três rodadas.</p>',
    camera: 'cima', camadas: ['trilhas', 'fantasma'], destaque: [], no: 'slam', metodo: 'ekf+slam', mundo: 'plantacao', aba: 'resultados',
  },
  {
    titulo: 'O que vem a seguir',
    texto: '<p>Os próximos passos estão na aba <b>Próximos passos</b>: rodar a câmera de aquisição no Webots, ligar a gravação às missões, medir a cobertura das plantas, ruído nos sensores e, por fim, o <b>primeiro mapa no campo real</b>.</p>',
    camera: 'geral', camadas: ['rotulos', 'camdados'], destaque: ['camdados'], mundo: 'olival', aba: 'futuro', parado: true,
  },
];

// Próximos passos. estado: feito | agora | proximo | futuro
window.FUTURO = [
  {
    titulo: 'Já funciona',
    estado: 'feito',
    itens: [
      ['Robô no Webots com LiDAR, IMU, encoders, câmera RGB-D e pose verdadeira', ''],
      ['EKF (rodas + IMU) e VSLAM com RTAB-Map: mapeamento e localização em mapa salvo', 'vslam:=true, vslam_localization:=true'],
      ['Nav2 com missões de waypoints por YAML ou clicadas no RViz, gravando a trajetória', 'waypoints:=<caso> | rviz'],
      ['Comparação EKF × SLAM LiDAR × VSLAM (passo 3, 32 rodadas)', 'resultados/passo3_comparacao'],
      ['Mundos da plantação e do olival, gerados por script', 'plantation_field.wbt, olive_grove.wbt'],
      ['Teleoperação com prioridade sobre o Nav2 e mapas com nome', 'teleop:=joy | keyboard, vslam_db:=<nome>'],
      ['Checagem de patinagem das rodas pela odometria visual', 'wheel_slip_monitor'],
    ],
  },
  {
    titulo: 'Aquisição de dados (em andamento)',
    estado: 'agora',
    itens: [
      ['Rodar a câmera de aquisição e o data_recorder no Webots pela primeira vez', 'código pronto, conferido só fora do simulador'],
      ['Medir o custo no Windows (fator de tempo real) com as duas câmeras e o RTAB-Map', 'se pesar: foto a cada 2 s, 1024 × 768 ou JPEG'],
      ['Ligar a gravação às missões: a missão liga a câmera no início e desliga no fim, com o mesmo prefixo dos CSVs', 'etapa 5 do plano da câmera'],
      ['Inventário das plantas e avaliação: cobertura (plantas vistas inteiras em ≥ 1 e ≥ 3 fotos) e erro de georreferência em metros e em pixels', 'etapa 6'],
      ['Rodadas de comparação da aquisição: casos 2 e 3, EKF × VSLAM', 'etapa 7'],
    ],
  },
  {
    titulo: 'Simulação e resultados',
    estado: 'proximo',
    itens: [
      ['Rodar os casos do olival (caso 4 ida e volta, caso 5 encosta)', 'ainda sem rodadas de comparação'],
      ['Mapear o olival conduzindo (protocolo do operador) e comparar com o mapa da missão automática', 'ATE e erro máximo'],
      ['Ligar ruído nos sensores (IMU com deriva, profundidade) para uma comparação justa com o robô real', 'hoje a IMU é perfeita e favorece o EKF'],
      ['Resolver a navegação na cabeceira leste da plantação (caso 3)', 'mistura erro de navegação com erro de localização'],
      ['Validar no Windows/WSL com o RViz na tela', 'quase tudo foi validado na nuvem, sem GPU'],
      ['Gravar rosbag e refazer o mapa a partir dele', ''],
    ],
  },
  {
    titulo: 'Campo real',
    estado: 'proximo',
    itens: [
      ['Planejar o percurso na imagem de satélite (QGIS ou Google Earth)', ''],
      ['Conferir o robô parado: TF, scan, profundidade da RGB-D ao sol, IMU e odometria numa reta de 10 m', ''],
      ['Primeiro mapa com joystick, sem GPS, seguindo o protocolo de condução', 'vslam:=true teleop:=joy'],
      ['Sobrepor o mapa à imagem de satélite e marcar os waypoints do operador', ''],
    ],
  },
  {
    titulo: 'Trabalho futuro e opcionais',
    estado: 'futuro',
    itens: [
      ['Segunda câmera de aquisição (lado direito), para fotografar as duas fileiras numa passada', ''],
      ['Rótulos automáticos (planta × solo) com o Recognition do Webots', 'conjunto de dados rotulado sem anotação manual'],
      ['Plantas "doentes" num mundo variante, para ter o que monitorizar', ''],
      ['Mapeamento semiautônomo a partir de um polígono desenhado na imagem de satélite', 'rota gerada + filtro keepout do Nav2'],
      ['Relocalização global: começar em qualquer lugar do mapa, e não só na partida', 'hoje a detecção global está desligada'],
      ['Marcar waypoints sem o robô ligado e avisar pontos a mais de ~12 m', ''],
    ],
  },
];

// Resultados do passo 3 (resultados/passo3_comparacao/compare.md).
window.RESULTADOS = [
  // caso, método, atingidos, no alvo, ATE, dp, erro máx. médio, yaw máx.
  ['Caso 2: plantação, ida e volta (~26 m)', 'ekf', '13/15', '12/15', 0.29, 0.17, 0.72, 2.4],
  ['', 'ekf+slam', '10/15', '7/15', 2.12, 1.65, 4.47, 51.5],
  ['', 'ekf+vslam', '15/15', '15/15', 0.15, 0.04, 0.33, 4.3],
  ['', 'ekf+vslam-loc', '14/15', '14/15', 0.13, 0.06, 0.27, 4.5],
  ['Caso 3: plantação, serpentina (~51 m)', 'ekf', '30/33', '26/33', 0.47, 0.32, 1.01, 3.4],
  ['', 'ekf+slam', '12/22', '8/22', 2.97, 2.41, 5.52, 38.9],
  ['', 'ekf+vslam', '28/33', '20/33', 1.05, 0.89, 1.86, 8.3],
];

window.METODOS = {
  'ekf': { nome: 'EKF (rodas + IMU)', cor: '--ekf' },
  'ekf+vslam': { nome: 'EKF + VSLAM (oficial)', cor: '--vslam' },
  'ekf+vslam-loc': { nome: 'EKF + VSLAM, mapa salvo', cor: '--vslam' },
  'ekf+slam': { nome: 'EKF + SLAM LiDAR (comparação)', cor: '--slam' },
};
