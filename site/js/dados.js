// Conteúdo do site: nós ROS 2, roteiro da apresentação e próximos passos.
// Para mudar um texto, edite aqui; o resto do site lê destes objetos.
// Fonte dos fatos: claude/manual.md, launch/simulation.launch.py,
// config/*.yaml e os planejamentos do projeto.

// Partes do robô que o site sabe destacar no 3D.
window.PARTES = {
  chassi: 'Chassi e quadro',
  rodas: 'Rodas e encoders',
  lidar: 'LiDAR de segurança',
  lidar3d: 'LiDAR 3D',
  rgbd: 'Câmera RGB-D (VSLAM)',
  camdados: 'Câmera RGB de aquisição',
  imu: 'IMU',
};

// Grupos de nós (cores no grafo e na lista).
window.GRUPOS = {
  sim: { nome: 'Simulação e ponte', cor: '#8b95a7' },
  seg: { nome: 'Segurança', cor: '#e5484d' },
  loc: { nome: 'Localização', cor: '#4aa3ff' },
  nav: { nome: 'Navegação (Nav2)', cor: '#2fbf71' },
  missao: { nome: 'Missão e dados', cor: '#f2c94c' },
  op: { nome: 'Operador', cor: '#f29e38' },
  vis: { nome: 'Visualização', cor: '#b08cff' },
  legado: { nome: 'Só comparação', cor: '#6b6f7a' },
};

// Nós. x, y (e h, a altura, se não for 44): posição no grafo, em colunas de
// 200 em 200 (caixas de 166 de largura). partes: o que acende no robô.
// camadas: o que liga no 3D quando o nó é escolhido.
window.NOS = [
  {
    id: 'webots', nome: 'Webots R2025a', grupo: 'sim', x: 20, y: 300,
    pacote: 'simulador (Windows)', quando: 'sempre',
    faz: 'Simula a física (rodas, atrito, terreno) e o que cada sensor mede. É o "mundo real" de mentira: plantação ou olival, com o robô do PROTO <code>agrobot.proto</code>.',
    entradas: ['velocidade das rodas (do driver)'],
    saidas: ['leituras dos sensores, pose verdadeira (Supervisor)'],
    porque: 'Roda bem no Windows, onde o Kauê trabalha; o Gazebo ficou como versão antiga.',
    arquivo: 'agrobot_webots/worlds/*.wbt, protos/agrobot.proto',
    partes: ['chassi', 'rodas', 'lidar', 'lidar3d', 'rgbd', 'camdados', 'imu'], camadas: [],
  },
  {
    id: 'driver', nome: 'agrobot_driver', grupo: 'sim', x: 220, y: 284, h: 104,
    pacote: 'agrobot_webots (plugin do webots_ros2_driver)', quando: 'sempre',
    faz: 'A ponte entre o Webots e o ROS 2. A cada passo de 32 ms converte <code>/cmd_vel</code> em velocidade das 4 rodas e publica todos os sensores com o tempo da simulação. Antes de publicar, aplica o erro de sensores reais de baixo custo (sensor_noise:=true, padrão).',
    entradas: ['/cmd_vel'],
    saidas: ['/clock', '/wheel/odom', '/imu/data', '/scan', '/lidar3d/points', '/joint_states', '/vslam_camera/*', '/data_camera/*', '/ground_truth/odom'],
    porque: 'Os publicadores do webots_ros2_driver carimbavam com o relógio do nó (parado ou atrasado): o Nav2 descartava os scans e o RTAB-Map não casava cor com profundidade. Aqui tudo sai com robot.getTime(). O ruído (IMU que deriva, encoders, LiDARs, profundidade) vem de datasheets: sem ele a IMU era perfeita e favorecia o EKF. Se o /cmd_vel parar por 1 s, o robô para.',
    arquivo: 'agrobot_webots/agrobot_webots/agrobot_driver.py, sensor_noise.py',
    partes: ['rodas', 'lidar', 'lidar3d', 'rgbd', 'camdados', 'imu'], camadas: ['lidar', 'lidar3d'],
  },
  {
    id: 'cm', nome: 'collision_monitor', grupo: 'seg', x: 220, y: 470,
    pacote: 'nav2_collision_monitor', quando: 'safety:=true (padrão)',
    faz: 'Último nó antes das rodas: zera a velocidade quando há pontos do LiDAR de segurança na zona de parada. A zona anda com o robô e depende da velocidade: 0,5 m à frente até 0,2 m/s, 0,7 m acima disso; para trás, o mesmo atrás; girando, a pegada com 5 cm de folga.',
    entradas: ['/cmd_vel_limited', '/scan'], saidas: ['/cmd_vel', '/collision_monitor_state'],
    porque: 'Pode haver uma pessoa no campo, e o LiDAR a 15 cm vê a canela dela. Parado na frente de um obstáculo, o robô ainda gira ou recua, mas não avança. Sem scan novo por 1 s, para. Não é um sistema de segurança certificado: é a "camada de segurança da navegação".',
    arquivo: 'config/collision_monitor.yaml',
    partes: ['lidar'], camadas: ['seguranca', 'lidar'],
  },
  {
    id: 'cmlim', nome: 'collision_monitor_limit', grupo: 'seg', x: 220, y: 560,
    pacote: 'nav2_collision_monitor', quando: 'safety:=true (padrão)',
    faz: 'Primeiro dos dois: limita a 0,2 m/s quando há obstáculo até 1,4 m à frente, e reduz a velocidade para nada ficar a menos de 1,5 s de colisão nas curvas e giros.',
    entradas: ['/cmd_vel_raw (Nav2, joystick, teclado, página web)', '/scan'], saidas: ['/cmd_vel_limited'],
    porque: 'Dois nós em série porque o collision_monitor escolhe a zona de parada pela velocidade que recebe: com a velocidade já limitada, a zona de parada pode ser curta e não pega as plantas por onde o Nav2 passa raspando. A 0,2 m/s o robô para em ~6 cm.',
    arquivo: 'config/collision_monitor.yaml',
    partes: ['lidar'], camadas: ['seguranca', 'lidar'],
  },
  {
    id: 'rsp', nome: 'robot_state_publisher', grupo: 'sim', x: 420, y: 20,
    pacote: 'robot_state_publisher', quando: 'sempre',
    faz: 'Lê o URDF e publica os frames fixos: onde cada sensor está em relação ao <code>base_link</code>.',
    entradas: ['URDF (agrobot.urdf)', '/joint_states'],
    saidas: ['TF base_link → sensores e rodas', '/robot_description'],
    porque: 'Todo mundo usa esses frames: o EKF (a IMU está no imu_link), o RTAB-Map (câmera e LiDAR 3D), os costmaps e a camada de segurança (LiDAR de segurança) e o data_recorder (câmera). Por isso o PROTO e o URDF têm que bater nas poses dos sensores.',
    arquivo: 'agrobot_description/urdf/agrobot.urdf',
    partes: ['chassi'], camadas: ['tf'],
  },
  {
    id: 'sync', nome: 'rgbd_sync', grupo: 'loc', x: 420, y: 110,
    pacote: 'rtabmap_sync', quando: 'vslam:=true (ou checagem de patinagem)',
    faz: 'Junta a imagem colorida, a profundidade e o camera_info do mesmo instante numa mensagem só.',
    entradas: ['/vslam_camera/color/image_raw', '/vslam_camera/aligned_depth_to_color/image_raw', 'camera_info'],
    saidas: ['/rtabmap/rgbd_image'],
    porque: 'O RTAB-Map precisa de cor e profundidade do mesmo quadro; o driver já publica as duas com o mesmo carimbo.',
    arquivo: 'config/rtabmap.yaml',
    partes: ['rgbd'], camadas: ['rgbd'],
  },
  {
    id: 'vo', nome: 'rgbd_odometry', grupo: 'loc', x: 420, y: 200,
    pacote: 'rtabmap_odom', quando: 'ekf:=true camera:=true (slip_check, ligado por padrão)',
    faz: 'Odometria visual: estima quanto a câmera andou entre dois quadros. Não entra direto no EKF; serve para conferir as rodas.',
    entradas: ['/rtabmap/rgbd_image'],
    saidas: ['/vo/odom'],
    porque: 'Quando o robô fica preso e as rodas giram em falso, só a câmera percebe que ele não saiu do lugar.',
    arquivo: 'config/rtabmap.yaml',
    partes: ['rgbd'], camadas: ['rgbd'],
  },
  {
    id: 'merge', nome: 'lidar_merge', grupo: 'loc', x: 420, y: 290,
    pacote: 'agrobot_webots', quando: 'vslam:=true',
    faz: 'Junta numa nuvem só a do LiDAR 3D e o scan de segurança. Tira da nuvem 3D a faixa de altura da grade (que fica só com o scan) e tira do scan os toques no chão, conferindo cada ponto na nuvem 3D.',
    entradas: ['/lidar3d/points', '/scan'], saidas: ['/lidar3d/merged'],
    porque: 'O RTAB-Map assina uma nuvem só. O LiDAR 3D vai para o ICP dos fechamentos de laço; a grade 2D sai do LiDAR de segurança, como nos costmaps do Nav2. Com a grade pela nuvem 3D, as fileiras sumiam do mapa e o relevo virava obstáculo falso.',
    arquivo: 'agrobot_webots/agrobot_webots/lidar_merge.py',
    partes: ['lidar', 'lidar3d'], camadas: ['lidar3d', 'lidar'],
  },
  {
    id: 'slip', nome: 'wheel_slip_monitor', grupo: 'loc', x: 420, y: 390,
    pacote: 'agrobot_webots', quando: 'ekf:=true camera:=true (slip_check)',
    faz: 'Compara a velocidade das rodas com a da câmera. Normalmente repassa as rodas sem mudar; se discordam por uma janela de 1,5 s (patinando ou preso), troca a velocidade das rodas pela mediana da câmera até voltarem a concordar.',
    entradas: ['/wheel/odom', '/vo/odom'],
    saidas: ['/wheel/odom_checked (para o EKF)', '/wheel/slip'],
    porque: 'Nem o EKF nem a IMU percebem rodas girando em falso: parado e velocidade constante dão a mesma aceleração (zero). Preso num mourão por ~8 s, o erro final ficou em 0,47 m, contra 2,8 m sem a checagem; nas missões normais, nenhum alarme falso.',
    arquivo: 'agrobot_webots/agrobot_webots/wheel_slip_monitor.py, slip_detector.py',
    partes: ['rodas', 'rgbd'], camadas: [],
  },
  {
    id: 'ekf', nome: 'ekf_filter_node', grupo: 'loc', x: 620, y: 390,
    pacote: 'robot_localization', quando: 'ekf:=true',
    faz: 'Filtro de Kalman estendido a 30 Hz, em 2D. Junta a velocidade das rodas com o yaw e a velocidade de giro da IMU e publica uma pose suave e contínua.',
    entradas: ['/wheel/odom_checked (ou /wheel/odom)', '/imu/data'],
    saidas: ['/odom', 'TF odom → base_link'],
    porque: 'No skid-steer as rodas escorregam nas curvas, então o yaw vem da IMU e a distância vem das rodas. Nenhuma entrada mede posição: o odom deriva por definição (com o ruído, ~4° de yaw viram ~0,6 m numa entrelinha de 9 m), e quem corrige é o VSLAM.',
    arquivo: 'config/ekf.yaml',
    partes: ['rodas', 'imu'], camadas: ['trilhas'], metodo: 'ekf',
  },
  {
    id: 'rtabmap', nome: 'rtabmap (VSLAM)', grupo: 'loc', x: 620, y: 110,
    pacote: 'rtabmap_slam', quando: 'vslam:=true (exige ekf, camera e lidar3d)',
    faz: 'SLAM visual, a 1 Hz, em 3DoF. Cria um nó do mapa por segundo com a imagem RGB-D e a nuvem do LiDAR 3D e, quando reconhece um lugar perto de onde o EKF acha que está, corrige a deriva. Grava o mapa num banco .db; com vslam_localization:=true localiza num mapa salvo.',
    entradas: ['/rtabmap/rgbd_image', '/lidar3d/merged', 'TF odom → base_link (do EKF)'],
    saidas: ['TF map → odom', '/rtabmap/map', 'dados/vslam/<mapa>.db'],
    porque: 'Usa a odometria do EKF (sem odometria visual, economiza CPU). Fechamento só por proximidade: as entrelinhas são iguais para a câmera e para o LiDAR. Cada fechamento é conferido por ICP ponto-a-plano com a nuvem 3D (chão, troncos, copas, relevo), que pode corrigir até 0,3 m, e pela otimização robusta (Vertigo).',
    arquivo: 'config/rtabmap.yaml',
    partes: ['rgbd', 'lidar3d'], camadas: ['rgbd', 'lidar3d', 'trilhas'], metodo: 'ekf+vslam',
  },
  {
    id: 'mission', nome: 'waypoint_mission', grupo: 'missao', x: 820, y: 20,
    pacote: 'agrobot_webots', quando: 'waypoints:=<caso>, rviz, ou operator:=true',
    faz: 'Lê a lista de waypoints (YAML, cliques no RViz ou a página do operador), confere cada ponto pelas medidas do robô e manda tudo para o FollowWaypoints do Nav2, com parada de 3 s em cada ponto. Grava a pose estimada e a verdadeira em CSV e, com data_camera:=true, liga a câmera de aquisição só durante a missão.',
    entradas: ['config/waypoints/*.yaml, /clicked_point ou /waypoint_mission/command', 'grade do mapa', '/ground_truth/odom (só para medir)'],
    saidas: ['ação FollowWaypoints', '/waypoint_mission/status', '/data_recorder/session', 'dados/missions/*_trajetoria.csv'],
    porque: 'As coordenadas são do frame da partida, então a mesma lista vale com e sem VSLAM. Um ponto onde o robô não cabe, não consegue girar ou não chega é recusado antes de a missão começar, com o ponto livre mais perto. A missão também é recusada se o mundo ou o mapa não baterem.',
    arquivo: 'agrobot_webots/agrobot_webots/waypoint_mission.py, waypoint_check.py',
    partes: [], camadas: ['waypoints', 'trilhas'],
  },
  {
    id: 'wpf', nome: 'waypoint_follower', grupo: 'nav', x: 820, y: 110,
    pacote: 'nav2_waypoint_follower', quando: 'nav:=true',
    faz: 'Vai a um waypoint por vez (NavigateToPose); para 3 s em cada um e, se um falhar, pula para o próximo.',
    entradas: ['ação FollowWaypoints'], saidas: ['ação NavigateToPose'],
    porque: 'A parada é onde entra a coleta de dados.',
    arquivo: 'config/nav2_params.yaml',
    partes: [], camadas: ['waypoints'],
  },
  {
    id: 'bt', nome: 'bt_navigator', grupo: 'nav', x: 820, y: 200,
    pacote: 'nav2_bt_navigator', quando: 'nav:=true',
    faz: 'Árvore de comportamento do Nav2: pede um caminho ao planner, manda o controller segui-lo e chama as recuperações quando algo dá errado.',
    entradas: ['ação NavigateToPose'], saidas: ['ComputePathToPose', 'FollowPath', 'recuperações'],
    porque: 'Com vslam:=true trabalha no frame map; sem, no odom.',
    arquivo: 'config/nav2_params.yaml',
    partes: [], camadas: ['caminho'],
  },
  {
    id: 'planner', nome: 'planner_server', grupo: 'nav', x: 820, y: 300,
    pacote: 'nav2_planner (NavFn)', quando: 'nav:=true',
    faz: 'Planeja o caminho global no costmap global (janela de 24 × 24 m centrada no robô, em map com VSLAM).',
    entradas: ['/scan', 'TF map → base_link'], saidas: ['/plan'],
    porque: 'NavFn com allow_unknown. Sem camada estática: objetivos a mais de ~12 m são recusados.',
    arquivo: 'config/nav2_params.yaml',
    partes: ['lidar'], camadas: ['caminho', 'lidar'],
  },
  {
    id: 'controller', nome: 'controller_server', grupo: 'nav', x: 820, y: 390,
    pacote: 'nav2_controller (Regulated Pure Pursuit)', quando: 'nav:=true',
    faz: 'Segue o caminho a 20 Hz no costmap local (4 × 4 m, sempre em odom), com giro no lugar. Até 0,5 m/s e 1,0 rad/s.',
    entradas: ['/plan', '/odom', '/scan'], saidas: ['/cmd_vel_nav'],
    porque: 'Fica em odom (REP-105): os saltos do map → odom atrapalhariam o seguimento. Footprint 1,2 × 1,12 m e inflação de 0,7 m. Os costmaps usam o LiDAR de segurança, o mesmo da camada de segurança: o Nav2 desvia, a camada para.',
    arquivo: 'config/nav2_params.yaml',
    partes: ['lidar', 'rodas'], camadas: ['costmap', 'lidar', 'caminho'],
  },
  {
    id: 'behavior', nome: 'behavior_server', grupo: 'nav', x: 1020, y: 200,
    pacote: 'nav2_behaviors (e o smoother_server, do nav2_smoother)', quando: 'nav:=true',
    faz: 'Recuperações (girar, dar ré, esperar) e suavização do caminho, chamadas pela árvore do bt_navigator.',
    entradas: ['pedidos do bt_navigator'], saidas: ['/cmd_vel_nav'],
    porque: 'Launch próprio do Nav2 (e não o nav2_bringup): no Jazzy ele sobe o docking_server, que falha sem plugins.',
    arquivo: 'launch/navigation.launch.py',
    partes: [], camadas: [],
  },
  {
    id: 'velsm', nome: 'velocity_smoother', grupo: 'nav', x: 1020, y: 390,
    pacote: 'nav2_velocity_smoother', quando: 'nav:=true',
    faz: 'Limita aceleração e velocidade do comando do Nav2 antes de chegar ao robô.',
    entradas: ['/cmd_vel_nav'], saidas: ['/cmd_vel_raw (ou /cmd_vel_nav_out com o mux)'],
    porque: 'Evita trancos nas rodas e no terreno irregular.',
    arquivo: 'config/nav2_params.yaml',
    partes: ['rodas'], camadas: [],
  },
  {
    id: 'mux', nome: 'cmd_vel_mux', grupo: 'op', x: 1020, y: 480,
    pacote: 'agrobot_webots', quando: 'teleop:=joy | keyboard, ou operator:=true',
    faz: 'Junta o comando do operador e o do Nav2 num só. O operador tem prioridade; 0,5 s depois que ele solta, o Nav2 volta sem cancelar a missão.',
    entradas: ['/cmd_vel_teleop', '/cmd_vel_nav_out'], saidas: ['/cmd_vel_raw', '/cmd_vel_mux/source'],
    porque: 'Dá para tomar o controle no meio de uma missão e devolvê-lo. O operador também passa pela camada de segurança: não consegue jogar o robô em cima de uma pessoa.',
    arquivo: 'agrobot_webots/agrobot_webots/cmd_vel_mux.py',
    partes: ['rodas'], camadas: [],
  },
  {
    id: 'teleop', nome: 'joystick / teclado', grupo: 'op', x: 1220, y: 480,
    pacote: 'joy + teleop_twist_joy, ou teleop_keyboard', quando: 'teleop:=joy | keyboard',
    faz: 'Joystick: segure LB/L1 para andar (homem-morto). Teclado (num terminal próprio): cada tecla muda a velocidade e o robô a mantém, como piloto automático.',
    entradas: ['joystick ou teclado'], saidas: ['/cmd_vel_teleop'],
    porque: 'O mapa do campo real vai ser feito com um operador conduzindo, sem GPS.',
    arquivo: 'config/teleop_joy.yaml, agrobot_webots/teleop_keyboard.py',
    partes: [], camadas: [],
  },
  {
    id: 'web', nome: 'interface web', grupo: 'op', x: 1220, y: 570,
    pacote: 'operator_bridge, operator_web, rosbridge', quando: 'operator:=true (http://localhost:8080)',
    faz: 'Página para operar pelo navegador, sem ROS no computador: mapa e robô, waypoints (marcar, arrastar, editar e salvar missões), varreduras em serpentina, joystick na tela, câmeras, avisos da camada de segurança e os dados gravados.',
    entradas: ['/operator/state, /operator/map e as câmeras em JPEG (do operator_bridge)', '/waypoint_mission/status'],
    saidas: ['/cmd_vel_teleop', '/waypoint_mission/command', '/data_camera/enable'],
    porque: 'O Kauê usa Windows e, no campo, o operador só precisa de um navegador. O operator_bridge resume a grade, o scan e as imagens (PNG e JPEG) porque mandá-los crus pelo rosbridge pesaria demais. Sem senha: em rede aberta, restringir as portas.',
    arquivo: 'agrobot_webots/web/, operator_bridge.py, operator_web.py',
    partes: [], camadas: ['waypoints'],
  },
  {
    id: 'recorder', nome: 'data_recorder', grupo: 'missao', x: 620, y: 480,
    pacote: 'agrobot_webots', quando: 'data_camera:=true (data_record:=true)',
    faz: 'Grava as fotos da câmera de aquisição em PNG; com missão, só entre o início e o fim dela, numa pasta com o mesmo prefixo dos CSVs. Para cada foto, a pose estimada da câmera (TF map → data_camera_optical_frame no instante da foto), a pose verdadeira e o erro entre as duas. Só grava se a câmera andou 0,3 m desde a última.',
    entradas: ['/data_camera/image_raw', '/data_camera/camera_info', 'TF', '/ground_truth/odom', '/data_recorder/session'],
    saidas: ['dados/fotos/<caso>_<modo>_<data>/*.png', 'fotos.csv', 'camera_info.yaml', 'origem.yaml'],
    porque: 'Cada foto fica georreferenciada: dá para saber de que planta ela é e medir quanto a localização errou ali. Depois, scripts/avaliar_aquisicao.py conta as plantas fotografadas inteiras e mede o erro de georreferência em metros e em pixels.',
    arquivo: 'agrobot_webots/agrobot_webots/data_recorder.py',
    partes: ['camdados'], camadas: ['camdados'],
  },
  {
    id: 'rviz', nome: 'RViz / rtabmap_viz', grupo: 'vis', x: 20, y: 560,
    pacote: 'rviz2, rtabmap_viz', quando: 'rviz:=true (rtabmap_viz:=true)',
    faz: 'Mostra o mapa, os costmaps, o caminho, as zonas de segurança, os waypoints e as câmeras. É onde se clica um objetivo (Nav2 Goal) ou os waypoints (Publish Point).',
    entradas: ['quase tudo'], saidas: ['/goal_pose', '/clicked_point'],
    porque: 'A grade /rtabmap/map só é montada enquanto alguém a assina: deixe o RViz (ou a página do operador) aberto no mapeamento.',
    arquivo: 'rviz/nav_vslam.rviz',
    partes: [], camadas: ['tf'],
  },
  {
    id: 'slam', nome: 'slam_toolbox', grupo: 'legado', x: 620, y: 570,
    pacote: 'slam_toolbox (+ ekf_map_node)', quando: 'slam:=true (só para comparação)',
    faz: 'SLAM só com o LiDAR 2D (scan matching). Com vslam:=true junto, roda sem publicar TF.',
    entradas: ['/scan'], saidas: ['/map, map → odom (sem VSLAM)'],
    porque: 'Ótimo na arena (0,03 m), mas perde o yaw nas fileiras iguais da plantação. Não é mais usado nem corrigido; fica como base de comparação.',
    arquivo: 'config/slam_toolbox.yaml',
    partes: ['lidar'], camadas: ['lidar', 'trilhas'], metodo: 'ekf+slam',
  },
];

// Ligações do grafo. rota (opcional; sem ela, curva automática):
//   ['h', y]                reta horizontal na altura y
//   ['v', x]                reta vertical (x opcional: centro da caixa)
//   ['l', sai, x, entra]    sai pela direita em y=sai, desce/sobe em x, entra pela esquerda em y=entra
//   ['u', sai, x, entra]    sai pela direita e volta pela direita do destino
//   ['p', [[x, y], ...]]    pontos explícitos
// As rotas foram escolhidas para nenhuma linha passar por trás de uma caixa.
window.LIGACOES = [
  { de: 'webots', para: 'driver', topico: 'sensores e motores', rota: ['h', 322] },
  { de: 'driver', para: 'rsp', topico: '/joint_states', rota: ['l', 290, 391, 42] },
  { de: 'driver', para: 'sync', topico: '/vslam_camera/*', rota: ['l', 297, 397, 132] },
  { de: 'driver', para: 'merge', topico: '/lidar3d/points, /scan', rota: ['h', 312] },
  { de: 'driver', para: 'mission', topico: '/ground_truth/odom', tracejada: true, rota: ['l', 342, 797, 42] },
  { de: 'driver', para: 'planner', topico: '/scan', rota: ['l', 349, 805, 322] },
  { de: 'driver', para: 'controller', topico: '/scan', rota: ['l', 356, 812, 402] },
  { de: 'driver', para: 'ekf', topico: '/imu/data', rota: ['l', 363, 610, 402] },
  { de: 'driver', para: 'slip', topico: '/wheel/odom', rota: ['l', 370, 415, 412] },
  { de: 'driver', para: 'recorder', topico: '/data_camera/image_raw', rota: ['l', 377, 409, 502] },
  { de: 'driver', para: 'slam', topico: '/scan', tracejada: true, rota: ['l', 384, 403, 592] },
  { de: 'driver', para: 'cm', topico: '/scan', rota: ['v', 262] },
  { de: 'driver', para: 'cmlim', topico: '/scan', rota: ['p', [[220, 372], [210, 372], [210, 594], [220, 594]]] },
  { de: 'cm', para: 'driver', topico: '/cmd_vel', rota: ['v', 345] },
  { de: 'cmlim', para: 'cm', topico: '/cmd_vel_limited', rota: ['v'] },
  { de: 'mux', para: 'cmlim', topico: '/cmd_vel_raw', rota: ['p', [[1103, 524], [1103, 712], [203, 712], [203, 574], [220, 574]]] },
  { de: 'rsp', para: 'rtabmap', topico: 'TF dos sensores', rota: ['l', 42, 596, 120] },
  { de: 'sync', para: 'vo', topico: 'rgbd_image', rota: ['v'] },
  { de: 'sync', para: 'rtabmap', topico: '/rtabmap/rgbd_image', rota: ['h', 132] },
  { de: 'vo', para: 'slip', topico: '/vo/odom', rota: ['p', [[586, 232], [592, 232], [592, 400], [586, 400]]] },
  { de: 'merge', para: 'rtabmap', topico: '/lidar3d/merged', rota: ['l', 300, 603, 144] },
  { de: 'slip', para: 'ekf', topico: '/wheel/odom_checked', rota: ['h', 420] },
  { de: 'ekf', para: 'rtabmap', topico: 'TF odom→base_link', rota: ['v'] },
  { de: 'ekf', para: 'controller', topico: '/odom', rota: ['h', 420] },
  { de: 'ekf', para: 'recorder', topico: 'TF da câmera', rota: ['v'] },
  { de: 'rtabmap', para: 'planner', topico: 'TF map→odom, /rtabmap/map', rota: ['l', 132, 805, 306] },
  { de: 'mission', para: 'wpf', topico: 'FollowWaypoints', rota: ['v'] },
  { de: 'wpf', para: 'bt', topico: 'NavigateToPose', rota: ['v'] },
  { de: 'bt', para: 'planner', topico: 'ComputePathToPose', rota: ['v'] },
  { de: 'bt', para: 'controller', topico: 'FollowPath', rota: ['u', 236, 1000, 400] },
  { de: 'bt', para: 'behavior', topico: 'recuperações', rota: ['h', 222] },
  { de: 'controller', para: 'velsm', topico: '/cmd_vel_nav', rota: ['h', 420] },
  { de: 'velsm', para: 'mux', topico: '/cmd_vel_nav_out', rota: ['v'] },
  { de: 'teleop', para: 'mux', topico: '/cmd_vel_teleop', rota: ['p', [[1220, 502], [1186, 502]]] },
  { de: 'web', para: 'mux', topico: '/cmd_vel_teleop', rota: ['p', [[1220, 582], [1203, 582], [1203, 514], [1186, 514]]] },
  { de: 'web', para: 'mission', topico: '/waypoint_mission/command', rota: ['p', [[1220, 596], [1210, 596], [1210, 30], [986, 30]]] },
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
    texto: '<p>Cada sensor tem uma função própria:</p><ul><li><b>LiDAR de segurança</b> (Delta-2A, 15 cm do chão, 8 m): parar diante de pessoas e obstáculos, e os costmaps do Nav2</li><li><b>LiDAR 3D</b> (Helios-16P, 16 canais, no topo): troncos, copas e relevo para a localização</li><li><b>IMU</b>: o ângulo (yaw) do robô</li><li><b>Encoders das rodas</b>: a distância percorrida</li><li><b>Câmera RGB-D</b> (como a RealSense D455, 15° para baixo): o VSLAM</li><li><b>Câmera RGB de aquisição</b> (0,9 m do chão, olhando para a esquerda): as fotos das plantas</li></ul><p>Os dois LiDARs têm papéis separados: o baixo só serve para não bater; o alto, só para saber onde o robô está.</p>',
    camera: 'sensores', camadas: ['rotulos', 'lidar', 'lidar3d', 'rgbd', 'camdados'], destaque: ['lidar', 'lidar3d', 'rgbd', 'camdados', 'imu'], mundo: 'plantacao', parado: true,
  },
  {
    titulo: 'A ponte: agrobot_driver',
    texto: '<p>O <b>driver</b> é o único que conversa com o Webots. A cada passo de 32 ms ele lê os sensores, publica nos tópicos do ROS 2 e aplica o <code>/cmd_vel</code> nos motores.</p><p>Tudo sai carimbado com o <b>tempo da simulação</b>. Sem isso o Nav2 descartava os scans e o RTAB-Map não casava cor com profundidade.</p><p>Antes de publicar, ele soma o <b>erro de sensores reais</b> de baixo custo, tirado de datasheets: a IMU deriva, os encoders erram o raio da roda, os LiDARs e a profundidade têm ruído. O simulador perfeito favorecia o EKF.</p>',
    camera: 'geral', camadas: ['rotulos', 'lidar'], destaque: ['rodas', 'lidar', 'lidar3d', 'rgbd', 'camdados', 'imu'], no: 'driver', mundo: 'plantacao',
  },
  {
    titulo: 'Onde estou? 1ª camada: EKF',
    texto: '<p>O <b>filtro de Kalman estendido</b> junta a distância das rodas com o ângulo da IMU, 30 vezes por segundo.</p><p>É como andar de olhos fechados contando passos: a pose é suave, mas o erro <b>acumula</b>. Com o ruído realista, a IMU deriva uns 2° em 5 minutos, e 4° de yaw viram ~0,6 m de erro lateral numa entrelinha de 9 m.</p><p><span class="chip" style="--c:var(--verdade)">verde</span> = onde o robô estava de verdade; <span class="chip" style="--c:var(--ekf)">laranja</span> = onde o EKF achava que estava (rodada do caso 2, ATE 0,23 m).</p>',
    camera: 'cima', camadas: ['trilhas', 'fantasma'], destaque: ['rodas', 'imu'], no: 'ekf', metodo: 'ekf', mundo: 'plantacao',
  },
  {
    titulo: '2ª camada: VSLAM (RTAB-Map)',
    texto: '<p>O <b>RTAB-Map</b> monta um mapa com a câmera RGB-D e <b>reconhece lugares</b> por onde já passou. Quando reconhece, corrige o erro acumulado do EKF.</p><p>Cada reconhecimento é conferido com a nuvem do <b>LiDAR 3D</b> (ICP): o chão, os troncos, as copas e o relevo travam x, y e o ângulo. Só aceita lugares <b>perto</b> de onde o EKF acha que está, porque as entrelinhas são todas iguais.</p><p>Na rodada mostrada (caso 2, ainda com o LiDAR 2D), ATE de <b>0,13 m</b>, cerca de metade do EKF.</p>',
    camera: 'segue', camadas: ['trilhas', 'fantasma', 'rgbd', 'lidar3d'], destaque: ['rgbd', 'lidar3d'], no: 'rtabmap', metodo: 'ekf+vslam', mundo: 'plantacao',
  },
  {
    titulo: 'A árvore de frames',
    texto: '<p><code>map → odom → base_link → sensores</code></p><ul><li><b>odom → base_link</b>: o EKF. Suave, sem saltos, mas deriva.</li><li><b>map → odom</b>: o RTAB-Map. É a correção da deriva.</li><li><b>base_link → sensores</b>: fixo, do URDF.</li></ul><p>O controlador do Nav2 usa o <code>odom</code> (não gosta de saltos); o planejador usa o <code>map</code>.</p>',
    camera: 'geral', camadas: ['tf', 'rotulos'], destaque: ['chassi'], no: 'rsp', mundo: 'plantacao', parado: true,
  },
  {
    titulo: 'Navegação: Nav2',
    texto: '<p>O <b>Nav2</b> é o motorista:</p><ul><li><b>NavFn</b> planeja o caminho no mapa</li><li><b>Regulated Pure Pursuit</b> segue o caminho (até 0,5 m/s, gira no lugar)</li><li>os <b>costmaps</b>, montados com o LiDAR de segurança, marcam obstáculos e mantêm 0,7 m de folga</li></ul><p>O quadrado em volta do robô é o costmap local (4 × 4 m); a linha tracejada, o caminho até o próximo waypoint.</p>',
    camera: 'segue', camadas: ['costmap', 'caminho', 'lidar', 'waypoints'], destaque: ['lidar'], no: 'controller', metodo: 'ekf+vslam', mundo: 'plantacao',
  },
  {
    titulo: 'Camada de segurança',
    texto: '<p>Pode haver uma pessoa no campo. Entre <b>todos os comandos</b> (Nav2, joystick, teclado, página web) e as rodas ficam dois <code>collision_monitor</code> com o LiDAR de segurança:</p><ul><li><span class="chip" style="--c:#f2c94c">amarelo</span> obstáculo até 1,4 m à frente: no máximo <b>0,2 m/s</b></li><li><span class="chip" style="--c:#e5484d">vermelho</span> obstáculo na zona de parada: <b>para</b>. São 0,5 m até 0,2 m/s e 0,7 m mais rápido</li></ul><p>Quem desvia é o Nav2; a camada para o que chega perto demais. Com o operador mandando 0,4 m/s para cima de uma pessoa, o robô parou a 0,13 m dela; sem a camada, passou por cima.</p>',
    camera: 'segue', camadas: ['seguranca', 'lidar'], destaque: ['lidar'], no: 'cm', metodo: 'ekf+vslam', mundo: 'plantacao',
  },
  {
    titulo: 'Missão de waypoints',
    texto: '<p>O <code>waypoint_mission</code> lê uma lista de pontos (YAML, cliques no RViz ou a página do operador) e a entrega ao Nav2. O robô <b>para 3 s em cada ponto</b>: é ali que entra a coleta de dados.</p><p>Antes de começar, confere cada ponto pelas medidas do robô: recusa onde ele <b>não cabe</b>, <b>não gira</b> ou <b>não chega</b>, e sugere o ponto livre mais perto.</p><p>Grava a pose estimada e a <b>verdadeira</b> (do simulador), para medir o erro de cada método. Aqui: caso 2, ida pela entrelinha do meio e volta pela vizinha (~26 m).</p>',
    camera: 'cima', camadas: ['waypoints', 'trilhas', 'caminho'], destaque: [], no: 'mission', metodo: 'ekf+vslam', mundo: 'plantacao',
  },
  {
    titulo: 'Aquisição de dados',
    texto: '<p>A <b>câmera RGB de aquisição</b> (1280 × 960, uma foto por segundo) olha para a fileira à esquerda. A 0,9 m da fileira, cada foto cobre ~1,8 m de comprimento por 1,35 m de altura.</p><p>Com uma missão, ela liga no início e desliga no fim. O <code>data_recorder</code> grava cada foto com a <b>pose estimada</b> e a <b>verdadeira</b> da câmera, numa pasta com o mesmo nome dos CSVs da trajetória.</p><p>Depois, um script conta as <b>plantas fotografadas inteiras</b> e mede o <b>erro de georreferência</b> em metros e em pixels. No caso 2: 57 fotos em 96 s, erro da câmera de 0,13 m em média.</p>',
    camera: 'lado', camadas: ['camdados', 'rotulos'], destaque: ['camdados'], no: 'recorder', metodo: 'ekf+vslam', mundo: 'plantacao',
  },
  {
    titulo: 'Operador no comando',
    texto: '<p>Um operador pode conduzir o robô com joystick, teclado ou pela <b>página web</b> (<code>operator:=true</code>), que abre em qualquer navegador sem ROS instalado: mapa e robô, waypoints que se arrastam e se salvam, varreduras em serpentina, joystick na tela e as câmeras.</p><p>O <code>cmd_vel_mux</code> dá prioridade ao operador; quando ele solta, o Nav2 volta sem cancelar a missão. O operador também passa pela camada de segurança.</p><p>É assim que vai ser feito o <b>primeiro mapa do campo real</b>.</p>',
    camera: 'geral', camadas: ['rotulos', 'seguranca'], destaque: ['rodas'], no: 'web', mundo: 'olival',
  },
  {
    titulo: 'Os mundos de teste',
    texto: '<p><b>Plantação</b>: 6 fileiras de milho de 9 m, entrelinhas de 1,8 m, terreno com camalhões, buracos, torrões e pedras. Fileiras iguais confundem o LiDAR 2D.</p><p><b>Olival</b> (planalto do Douro Internacional): 65 oliveiras em grade de 7 × 7 m numa encosta de 4 a 5%, com montes de poda, moroiços, vinha e estrada de terra. Lugar aberto: o LiDAR de segurança só vê troncos esparsos; o LiDAR 3D vê as copas e o relevo.</p>',
    camera: 'alto', camadas: ['lidar3d'], destaque: [], mundo: 'olival',
  },
  {
    titulo: 'Resultados até aqui',
    texto: '<p>32 rodadas comparando três métodos de localização (sensores ainda sem ruído):</p><ul><li><b>Na plantação</b>, o VSLAM reduziu o erro do EKF <b>pela metade</b> (ATE 0,13 a 0,15 m) e atingiu 15/15 waypoints.</li><li>O <b>SLAM por LiDAR</b> se perdeu nas fileiras repetitivas (ATE acima de 2 m, yaw errado em até 51°): por isso a câmera.</li></ul><p>Com o <b>ruído realista</b>, o VSLAM continua em 15/15 no caso 2, com ATE de 0,16 m. A aba Resultados tem também a camada de segurança.</p>',
    camera: 'cima', camadas: ['trilhas', 'fantasma'], destaque: [], no: 'slam', metodo: 'ekf+slam', mundo: 'plantacao', aba: 'resultados',
  },
  {
    titulo: 'O que vem a seguir',
    texto: '<p>Os próximos passos estão na aba <b>Próximos passos</b>: as rodadas de comparação da aquisição de fotos, refazer a comparação com o ruído e o LiDAR 3D, o olival e, por fim, o <b>primeiro mapa no campo real</b>.</p>',
    camera: 'geral', camadas: ['rotulos', 'camdados'], destaque: ['camdados'], mundo: 'olival', aba: 'futuro', parado: true,
  },
];

// Próximos passos. estado: feito | agora | proximo | futuro
window.FUTURO = [
  {
    titulo: 'Já funciona',
    estado: 'feito',
    itens: [
      ['Robô no Webots com LiDAR de segurança, LiDAR 3D, IMU, encoders, câmera RGB-D e pose verdadeira', ''],
      ['Ruído realista nos sensores (IMU com deriva, encoders, LiDARs, profundidade), tirado de datasheets', 'sensor_noise:=true (padrão)'],
      ['EKF (rodas + IMU) e VSLAM com RTAB-Map, com o LiDAR 3D no ICP: mapeamento e localização em mapa salvo', 'vslam:=true, vslam_localization:=true'],
      ['Camada de segurança: lentidão e parada com o LiDAR de segurança, testada com uma pessoa no caminho', 'safety:=true (padrão)'],
      ['Nav2 com missões de waypoints, e checagem de cada ponto pelas medidas do robô', 'waypoints:=<caso> | rviz'],
      ['Câmera RGB de aquisição ligada às missões, e avaliação das fotos: cobertura das plantas e erro de georreferência', 'data_camera:=true, scripts/avaliar_aquisicao.py'],
      ['Interface web do operador: mapa, waypoints editáveis, varreduras, joystick, câmeras e dados', 'operator:=true'],
      ['Teleoperação com prioridade sobre o Nav2 e mapas com nome', 'teleop:=joy | keyboard, vslam_db:=<nome>'],
      ['Checagem de patinagem das rodas pela odometria visual', 'wheel_slip_monitor'],
      ['Comparação EKF × SLAM LiDAR × VSLAM (passo 3, 32 rodadas, sensores sem ruído)', 'resultados/passo3_comparacao'],
      ['Mundos da plantação e do olival, gerados por script', 'plantation_field.wbt, olive_grove.wbt'],
    ],
  },
  {
    titulo: 'Em andamento',
    estado: 'agora',
    itens: [
      ['Rodadas de comparação da aquisição de fotos: casos 2 e 3, EKF × VSLAM', 'cobertura e erro de georreferência por método'],
      ['Refazer a comparação dos métodos com o ruído e o LiDAR 3D (mais repetições)', 'os resultados do passo 3 são com sensores ideais'],
      ['Validar no Windows/WSL: custo (fator de tempo real) com as câmeras, o LiDAR 3D e o RTAB-Map, o RViz e a página web', 'quase tudo foi validado na nuvem, sem GPU'],
    ],
  },
  {
    titulo: 'Simulação e resultados',
    estado: 'proximo',
    itens: [
      ['Waypoints e rodadas do olival', 'ainda sem rodadas de comparação'],
      ['Corrigir o deslizamento encosta abaixo no olival, que o EKF não vê', 'erro de até 1,6 m no caso 4'],
      ['Mapear o olival conduzindo e comparar com o mapa da missão automática', 'ATE e erro máximo'],
      ['Testar o joystick no Webots', 'o teclado já foi testado'],
      ['Gravar rosbag e refazer o mapa a partir dele', ''],
    ],
  },
  {
    titulo: 'Campo real',
    estado: 'proximo',
    itens: [
      ['Planejar o percurso na imagem de satélite (QGIS ou Google Earth)', ''],
      ['Conferir o robô parado: TF, os dois LiDARs, profundidade da RGB-D ao sol, IMU e odometria numa reta de 10 m', ''],
      ['Pôr senha ou restringir as portas da página do operador', 'hoje quem alcança as portas comanda o robô'],
      ['Primeiro mapa com joystick, sem GPS, seguindo o protocolo de condução', 'vslam:=true teleop:=joy'],
      ['Sobrepor o mapa à imagem de satélite e marcar os waypoints do operador', ''],
    ],
  },
  {
    titulo: 'Trabalho futuro e opcionais',
    estado: 'futuro',
    itens: [
      ['Segunda câmera de aquisição (lado direito), para fotografar as duas fileiras numa passada', ''],
      ['Câmera inclinada ou com mais FOV vertical, para a copa inteira das oliveiras', 'do meio da entrelinha o alto da copa sai da foto'],
      ['Rótulos automáticos (planta × solo) com o Recognition do Webots', 'conjunto de dados rotulado sem anotação manual'],
      ['Plantas "doentes" num mundo variante, para ter o que monitorizar', ''],
      ['Mapeamento semiautônomo a partir de um polígono desenhado na imagem de satélite', 'a varredura da página web já faz o retângulo'],
      ['Relocalização global: começar em qualquer lugar do mapa, e não só na partida', 'hoje a detecção global está desligada'],
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

// Resultados mais novos (claude/manual.md, seções 3 e 5).
window.RESULTADOS_NOVOS = [
  {
    titulo: 'Ruído realista nos sensores',
    texto: 'Caso 2 na plantação, EKF + VSLAM, 3 rodadas de cada. Com ruído, a IMU deriva e os encoders erram como os de um robô real.',
    cab: ['Sensores', 'Atingidos', 'ATE (m)', 'Erro máx. (m)', 'Yaw máx.'],
    linhas: [
      ['sem ruído', '15/15', '0,11 ± 0,03', '0,21', '2,3°'],
      ['com ruído', '15/15', '0,16 ± 0,01', '0,34', '3,4°'],
    ],
  },
  {
    titulo: 'Camada de segurança: uma pessoa no caminho',
    texto: 'EKF + VSLAM, folga mínima entre a pessoa (círculo de 0,25 m) e a pegada do robô, pela pose verdadeira.',
    cab: ['Cenário', 'Folga mínima', 'O que aconteceu'],
    linhas: [
      ['operador vai para cima dela, sem a camada', 'contato', 'passa por cima da pessoa'],
      ['operador vai para cima dela', '0,13 m', 'limita a 0,2 m/s a ~1,4 m e para'],
      ['ela cruza a entrelinha (plantação)', '0,70 m', 'limita; o Nav2 chega ao objetivo'],
      ['ela cruza a entrelinha (olival)', '0,49 m', 'limita; o Nav2 chega ao objetivo'],
    ],
  },
];
