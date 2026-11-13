import os
import re
import shutil
import tempfile
import time

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown, matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


ODOM_FROM_DRIVER = re.escape('<publishOdom>true</publishOdom>')
ODOM_FROM_EKF = '<publishOdom>false</publishOdom>'
# Qualquer período (o URDF pode ter outro múltiplo de 32 ms).
CAMERA_ON = r'<vslamCameraPeriod>\s*\d+\s*</vslamCameraPeriod>'
CAMERA_OFF = '<vslamCameraPeriod>0</vslamCameraPeriod>'
DATA_CAMERA_ON = r'<dataCameraPeriod>\s*\d+\s*</dataCameraPeriod>'
DATA_CAMERA_OFF = '<dataCameraPeriod>0</dataCameraPeriod>'


def driver_urdf(robot_description, ekf, camera, data_camera=False):
    """
    Caminho do URDF para o driver, com as trocas pedidas no launch.

    Com ekf, o driver não publica /odom; sem camera, não liga a câmera do
    VSLAM, e sem data_camera, não liga a de aquisição de dados (o Webots
    também deixa de renderizá-las).
    """
    replacements = []
    if ekf:
        replacements.append((ODOM_FROM_DRIVER, ODOM_FROM_EKF))
    if not camera:
        replacements.append((CAMERA_ON, CAMERA_OFF))
    if not data_camera:
        replacements.append((DATA_CAMERA_ON, DATA_CAMERA_OFF))
    if not replacements:
        return robot_description
    with open(robot_description) as urdf_file:
        content = urdf_file.read()
    for pattern, new in replacements:
        content, count = re.subn(pattern, new, content)
        if count == 0:
            raise RuntimeError(
                f'{pattern} não encontrado em {robot_description}')
    path = os.path.join(tempfile.gettempdir(), 'agrobot_webots_driver.urdf')
    with open(path, 'w') as urdf_file:
        urdf_file.write(content)
    return path


VSLAM_CAMERA = [
    ('rgb/image', '/vslam_camera/color/image_raw'),
    ('rgb/camera_info', '/vslam_camera/color/camera_info'),
    ('depth/image', '/vslam_camera/aligned_depth_to_color/image_raw'),
]


def project_dir():
    """
    Pasta do projeto (o repositório). Tudo o que os nós gravam fica em
    dados/ dentro dela (data_dir()).

    Com colcon build --symlink-install este arquivo é um link para o fonte, e
    o realpath cai no repositório. Sem o link, cai no install/ do workspace;
    então vale a pasta que tem o install/ (no uso normal, o próprio
    repositório). Se nada disso achar, ~/.ros.
    """
    here = os.path.dirname(os.path.realpath(__file__))
    parents = []
    while True:
        parents.append(here)
        parent = os.path.dirname(here)
        if parent == here:
            break
        here = parent
    for folder in parents:
        if os.path.isfile(
                os.path.join(folder, 'agrobot_webots', 'package.xml')):
            return folder
    for folder in parents:
        if os.path.isdir(os.path.join(folder, 'install')):
            return folder
    return os.path.join(os.path.expanduser('~'), '.ros')


def data_dir(*parts):
    """
    dados/ na raiz do projeto, fora do git: missions/ (CSVs das missões),
    fotos/ (câmera de aquisição) e vslam/ (bancos do RTAB-Map).
    """
    return os.path.join(project_dir(), 'dados', *parts)


def flag(context, name):
    return LaunchConfiguration(name).perform(context).lower() == 'true'


TELEOP_MODES = ('joy', 'keyboard')


def teleop_mode(context):
    """'joy', 'keyboard' ou '' (sem teleoperação)."""
    mode = LaunchConfiguration('teleop').perform(context).strip().lower()
    if mode in ('', 'false', 'none'):
        return ''
    if mode not in TELEOP_MODES:
        raise RuntimeError(
            f'teleop:={mode} não existe; use teleop:=joy ou teleop:=keyboard.')
    return mode


def slip_check_on(context):
    """Checagem de patinagem: pedida e com o EKF e a câmera ligados."""
    return (flag(context, 'slip_check') and flag(context, 'ekf')
            and flag(context, 'camera'))


def rgbd_sync_node(config):
    """Junta cor, profundidade e camera_info em /rtabmap/rgbd_image."""
    return Node(
        package='rtabmap_sync',
        executable='rgbd_sync',
        name='rgbd_sync',
        namespace='rtabmap',
        output='screen',
        parameters=[config, {'use_sim_time': True}],
        remappings=VSLAM_CAMERA,
    )


def vslam_db_path(context):
    """
    Caminho do banco do RTAB-Map (o mapa).

    vslam_db vazio: dados/vslam/<mundo>.db. Um nome sem pasta
    (vslam_db:=olival_mapa1) fica em dados/vslam/, com .db no fim se faltar.
    Com pasta, é o caminho do arquivo.
    """
    value = LaunchConfiguration('vslam_db').perform(context).strip()
    if not value:
        # world_name: o WebotsLauncher troca 'world' por uma cópia temporária.
        world = LaunchConfiguration('world_name').perform(context)
        value = os.path.splitext(os.path.basename(world))[0]
    if os.sep in value or '/' in value:
        return os.path.abspath(os.path.expanduser(value))
    if not value.endswith('.db'):
        value += '.db'
    return data_dir('vslam', value)


def map_name(context):
    """Nome do mapa em uso (o do banco, sem .db), ou '' sem VSLAM."""
    if not flag(context, 'vslam'):
        return ''
    return os.path.splitext(os.path.basename(vslam_db_path(context)))[0]


def backup_old_map(path):
    """
    O mapeamento começa com o banco vazio. Se já existe um mapa com esse
    nome, ele é renomeado para <nome>_<data>.db (a data do arquivo), em vez
    de apagado.
    """
    if not os.path.isfile(path):
        return None
    stamp = time.strftime(
        '%Y%m%d-%H%M%S', time.localtime(os.path.getmtime(path)))
    root, ext = os.path.splitext(path)
    backup = f'{root}_{stamp}{ext}'
    os.replace(path, backup)
    return backup


def vslam_database(context):
    """
    Banco do RTAB-Map e se ele é de mapeamento.

    O mapeamento começa num banco vazio (o mapa anterior com o mesmo nome vira
    um backup); a localização roda numa cópia, para o mapa salvo não mudar.
    """
    path = vslam_db_path(context)
    if not flag(context, 'vslam_localization'):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        return path, True
    if not os.path.isfile(path):
        raise RuntimeError(
            f'vslam_localization:=true precisa de um mapa salvo, e {path} '
            'não existe. Rode antes com vslam:=true.')
    copy = os.path.join(tempfile.gettempdir(), 'agrobot_vslam_localization.db')
    shutil.copyfile(path, copy)
    return copy, False


def generate_launch_description():
    webots_share = get_package_share_directory('agrobot_webots')
    # Arquivo em worlds/ (world:=plantation_field.wbt para a plantação).
    world_path = PathJoinSubstitution([
        webots_share,
        'worlds',
        LaunchConfiguration('world'),
    ])

    webots = WebotsLauncher(
        world=world_path,
        output='screen',
    )

    description_share = get_package_share_directory('agrobot_description')
    robot_description = os.path.join(
        description_share,
        'urdf',
        'agrobot.urdf',
    )
    with open(robot_description) as urdf_file:
        robot_description_content = urdf_file.read()

    def start_driver(context):
        ekf = LaunchConfiguration('ekf').perform(context).lower() == 'true'
        camera = (
            LaunchConfiguration('camera').perform(context).lower() == 'true')
        return [WebotsController(
            robot_name='agrobot',
            parameters=[
                {'robot_description':
                    driver_urdf(robot_description, ekf, camera,
                                flag(context, 'data_camera'))},
                {'use_sim_time': True},
            ],
            output='screen',
        )]

    agrobot_driver = OpaqueFunction(function=start_driver)

    # Rodas + IMU -> /odom e TF odom -> base_link (no lugar do driver). Com
    # a checagem de patinagem, as rodas chegam pelo wheel_slip_monitor.
    def start_ekf(context):
        if not flag(context, 'ekf'):
            return []
        parameters = [
            os.path.join(webots_share, 'config', 'ekf.yaml'),
            {'use_sim_time': True},
        ]
        if slip_check_on(context):
            parameters.append({'odom0': '/wheel/odom_checked'})
        return [Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=parameters,
            remappings=[('odometry/filtered', '/odom')],
        )]

    ekf_node = OpaqueFunction(function=start_ekf)

    # Checagem de patinagem: a odometria visual da câmera RGB-D
    # (rgbd_odometry do RTAB-Map, em /vo/odom) é comparada com as rodas pelo
    # wheel_slip_monitor. Quando discordam (robô preso com as rodas girando,
    # ou arrastado com elas paradas), o EKF recebe a velocidade da câmera no
    # lugar da das rodas. Só com ekf:=true e camera:=true.
    def start_slip_check(context):
        if not slip_check_on(context):
            return []
        config = os.path.join(webots_share, 'config', 'rtabmap.yaml')
        nodes = []
        if not flag(context, 'vslam'):
            # Com vslam:=true o rgbd_sync já sobe no start_vslam.
            nodes.append(rgbd_sync_node(config))
        nodes += [
            Node(
                package='rtabmap_odom',
                executable='rgbd_odometry',
                name='rgbd_odometry',
                namespace='rtabmap',
                output='screen',
                parameters=[config, {'use_sim_time': True}],
                remappings=[('odom', '/vo/odom')],
            ),
            Node(
                package='agrobot_webots',
                executable='wheel_slip_monitor',
                output='screen',
                parameters=[{'use_sim_time': True}],
            ),
        ]
        return nodes

    slip_check = OpaqueFunction(function=start_slip_check)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': True,
        }],
    )

    # SLAM com o LiDAR: /map, map -> odom e /pose (com covariância do scan
    # matching). O nó é lifecycle: o launch o configura e ativa.
    def start_slam(context):
        if not flag(context, 'slam'):
            return []
        vslam_on = flag(context, 'vslam')
        slam_parameters = [
            os.path.join(webots_share, 'config', 'slam_toolbox.yaml'),
            {'use_sim_time': True},
        ]
        if vslam_on:
            # Com vslam:=true o dono de map -> odom é o RTAB-Map. O
            # slam_toolbox continua, para comparação: sem TF, com o mapa em
            # /slam_toolbox/map e num frame (slam_map) que não existe no TF.
            slam_parameters.append({
                'transform_publish_period': 0.0,
                'map_frame': 'slam_map',
                'map_name': '/slam_toolbox/map',
            })
        slam_toolbox = LifecycleNode(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            namespace='',
            output='screen',
            parameters=slam_parameters,
        )
        slam_configure = EmitEvent(
            event=ChangeState(
                lifecycle_node_matcher=matches_action(slam_toolbox),
                transition_id=Transition.TRANSITION_CONFIGURE,
            ),
        )
        slam_activate = RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node=slam_toolbox,
                start_state='configuring',
                goal_state='inactive',
                entities=[EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(slam_toolbox),
                    transition_id=Transition.TRANSITION_ACTIVATE,
                ))],
            ),
        )
        actions = [slam_toolbox, slam_configure, slam_activate]
        if vslam_on:
            # A elipse no frame map vem do /pose do slam_toolbox, que com o
            # VSLAM fica em slam_map. Ela volta, com a pose do RTAB-Map, numa
            # etapa seguinte.
            return actions
        # Rodas + IMU + pose do slam_toolbox -> /odometry/map (a elipse no
        # frame map). Os serviços ganham prefixo para não colidir com o
        # ekf_filter_node.
        actions.append(Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_map_node',
            output='screen',
            parameters=[
                os.path.join(webots_share, 'config', 'ekf_map.yaml'),
                {'use_sim_time': True},
            ],
            remappings=[
                ('odometry/filtered', '/odometry/map'),
                ('set_pose', '/ekf_map_node/set_pose'),
                ('enable', '/ekf_map_node/enable'),
                ('reset', '/ekf_map_node/reset'),
                ('toggle', '/ekf_map_node/toggle'),
            ],
        ))
        return actions

    slam = OpaqueFunction(function=start_slam)

    # VSLAM com a câmera RGB-D: o rgbd_sync junta cor, profundidade e
    # camera_info numa mensagem só, e o rtabmap publica map -> odom, lendo a
    # odometria do TF odom -> base_link (do EKF).
    def start_vslam(context):
        if not flag(context, 'vslam'):
            return []
        if not flag(context, 'ekf'):
            raise RuntimeError(
                'vslam:=true precisa de ekf:=true: o RTAB-Map usa a odometria '
                'do EKF.')
        if not flag(context, 'camera'):
            raise RuntimeError('vslam:=true precisa da câmera (camera:=true).')
        database, mapping = vslam_database(context)
        actions = []
        if mapping:
            backup = backup_old_map(database)
            if backup:
                actions.append(LogInfo(msg=(
                    f'Já havia um mapa em {database}; ele foi guardado como '
                    f'{backup}.')))
        config = os.path.join(webots_share, 'config', 'rtabmap.yaml')
        rtabmap_parameters = {
            'database_path': database,
            'delete_db_on_start': mapping,
            'Mem/IncrementalMemory': 'true' if mapping else 'false',
            'Mem/InitWMWithAllNodes': 'false' if mapping else 'true',
        }
        if not mapping:
            # O robô nasce no mesmo lugar do mundo que no mapeamento, isto é,
            # no primeiro nó do mapa. Sem isso, a localização começaria onde
            # o mapeamento terminou.
            rtabmap_parameters['RGBD/StartAtOrigin'] = 'true'
        rtabmap = Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            namespace='rtabmap',
            output='screen',
            parameters=[config, rtabmap_parameters,
                        {'use_sim_time': True}],
            remappings=[('scan', '/scan')],
            # O rtabmap grava o mapa no banco ao fechar, e num mapa grande
            # isso passa dos 5 s que o launch espera antes de matá-lo.
            sigterm_timeout='120',
        )
        nodes = actions + [rgbd_sync_node(config), rtabmap]
        if mapping:
            def map_saved(event, context):
                if not os.path.isfile(database):
                    return [LogInfo(msg=(
                        f'O rtabmap fechou, mas {database} não existe: o '
                        'mapa não foi salvo.'))]
                size = os.path.getsize(database) / 1e6
                return [LogInfo(msg=(
                    f'Mapa salvo em {database} ({size:.1f} MB). Para usá-lo: '
                    'vslam_localization:=true vslam_db:='
                    f'{os.path.splitext(os.path.basename(database))[0]}'))]
            nodes.append(RegisterEventHandler(OnProcessExit(
                target_action=rtabmap, on_exit=map_saved)))
        if flag(context, 'rtabmap_viz'):
            nodes.append(Node(
                package='rtabmap_viz',
                executable='rtabmap_viz',
                name='rtabmap_viz',
                namespace='rtabmap',
                output='screen',
                parameters=[config, {'use_sim_time': True}],
                remappings=[('scan', '/scan')],
            ))
        return nodes

    vslam = OpaqueFunction(function=start_vslam)

    def global_frame(context):
        # Com SLAM (slam_toolbox ou RTAB-Map) existe map -> odom e o Nav2
        # planeja em map; sem, em odom.
        mapped = flag(context, 'slam') or flag(context, 'vslam')
        return 'map' if mapped else 'odom'

    def start_navigation(context):
        # Com teleop, o Nav2 manda para o cmd_vel_mux, e não direto ao driver.
        cmd_vel_out = 'cmd_vel_nav_out' if teleop_mode(context) else 'cmd_vel'
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(webots_share, 'launch', 'navigation.launch.py')),
            launch_arguments={
                'global_frame': global_frame(context),
                'cmd_vel_out': cmd_vel_out,
            }.items(),
            condition=IfCondition(LaunchConfiguration('nav')),
        )]

    navigation = OpaqueFunction(function=start_navigation)

    # Teleoperação (teleop:=joy ou teleop:=keyboard): o operador manda em
    # /cmd_vel_teleop e o cmd_vel_mux junta com o Nav2, com prioridade para o
    # operador. É o modo do mapeamento manual com o RTAB-Map e de tomar o
    # controle no meio de uma missão.
    def start_teleop(context):
        mode = teleop_mode(context)
        if not mode:
            return []
        nodes = [Node(
            package='agrobot_webots',
            executable='cmd_vel_mux',
            output='screen',
            parameters=[{'use_sim_time': True}],
        )]
        if mode == 'joy':
            config = os.path.join(webots_share, 'config', 'teleop_joy.yaml')
            nodes += [
                Node(
                    package='joy',
                    executable='game_controller_node',
                    name='joy_node',
                    output='screen',
                    parameters=[config],
                ),
                Node(
                    package='teleop_twist_joy',
                    executable='teleop_node',
                    name='teleop_twist_joy_node',
                    output='screen',
                    parameters=[config],
                    remappings=[('cmd_vel', '/cmd_vel_teleop')],
                ),
            ]
        else:
            # O teclado precisa de um terminal; o launch não tem um.
            nodes.append(LogInfo(msg=(
                'teleop:=keyboard: rode num outro terminal '
                '"ros2 run agrobot_webots teleop_keyboard".')))
        return nodes

    teleop = OpaqueFunction(function=start_teleop)

    def run_label(context):
        # Rótulo do modo nos nomes dos arquivos (CSVs das missões e pasta
        # das fotos): ekf, ekf+slam, ekf+vslam, ekf+vslam-loc, ...
        modes = [m for m in ('ekf', 'slam', 'vslam') if flag(context, m)]
        if slip_check_on(context):
            # EKF com a checagem de patinagem pela odometria visual.
            modes[modes.index('ekf')] = 'ekf-vo'
        if 'vslam' in modes and flag(context, 'vslam_localization'):
            # Localização num mapa salvo: outro rótulo nos CSVs, para não
            # misturar com as rodadas de mapeamento. Hífen, e não '_', porque
            # o summarize_missions.py separa caso, modo e data por '_'.
            modes[modes.index('vslam')] = 'vslam-loc'
        return '+'.join(modes) or 'odom'

    def start_mission(context):
        # waypoints:=caso2_plantacao_ida_volta (config/waypoints/) ou um
        # caminho para um .yaml; waypoints:=rviz para clicar os waypoints no
        # RViz (Publish Point). Vazio: sem missão.
        name = LaunchConfiguration('waypoints').perform(context)
        if not name:
            return []
        if name == 'rviz':
            path = ''
        else:
            path = name if os.path.isfile(name) else os.path.join(
                webots_share, 'config', 'waypoints',
                name if name.endswith('.yaml') else name + '.yaml')
            if not os.path.isfile(path):
                raise RuntimeError(
                    f'Arquivo de waypoints não encontrado: {path}')
        return [Node(
            package='agrobot_webots',
            executable='waypoint_mission',
            output='screen',
            parameters=[{
                'waypoints_file': path,
                'frame': global_frame(context),
                'run_label': run_label(context),
                'world': LaunchConfiguration('world_name').perform(context),
                'map': map_name(context),
                'output_dir': data_dir('missions'),
                'use_sim_time': True,
            }],
        )]

    mission = OpaqueFunction(function=start_mission)

    def start_recorder(context):
        # Grava as fotos da câmera de aquisição com a pose estimada e a
        # verdadeira de cada uma (data_recorder.py). Sobe junto com a câmera
        # (data_camera:=true); data_record:=false só mostra as imagens.
        if not (flag(context, 'data_camera') and flag(context, 'data_record')):
            return []
        parameters = {
            'frame': global_frame(context),
            'run_label': run_label(context),
            'world': LaunchConfiguration('world_name').perform(context),
            'min_distance': float(
                LaunchConfiguration('data_min_distance').perform(context)),
            'use_sim_time': True,
        }
        parameters['output_dir'] = (
            LaunchConfiguration('data_dir').perform(context)
            or data_dir('fotos'))
        return [Node(
            package='agrobot_webots',
            executable='data_recorder',
            output='screen',
            parameters=[parameters],
        )]

    recorder = OpaqueFunction(function=start_recorder)

    def start_rviz(context):
        # Com slam ou vslam, o RViz usa o frame map; com slam, mostra o /map
        # e as elipses, e com vslam, a grade e o caminho do RTAB-Map.
        if flag(context, 'vslam'):
            config = 'nav_vslam.rviz'
        elif flag(context, 'slam'):
            config = 'nav_slam.rviz'
        else:
            config = 'nav.rviz'
        return [Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(webots_share, 'rviz', config)],
            parameters=[{'use_sim_time': True}],
            output='screen',
            condition=IfCondition(LaunchConfiguration('rviz')),
        )]

    rviz = OpaqueFunction(function=start_rviz)

    shutdown_on_webots_exit = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=webots,
            on_exit=[EmitEvent(event=Shutdown())],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='obstacle_arena.wbt'),
        DeclareLaunchArgument('nav', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('ekf', default_value='false'),
        DeclareLaunchArgument('slam', default_value='false'),
        DeclareLaunchArgument('waypoints', default_value=''),
        # O WebotsLauncher troca 'world' pelo caminho de uma cópia temporária;
        # o nome original fica guardado para o waypoint_mission.
        SetLaunchConfiguration('world_name', LaunchConfiguration('world')),
        DeclareLaunchArgument('camera', default_value='true'),
        # Câmera RGB de aquisição de dados (/data_camera/...), desligada por
        # padrão para não pesar nas rodadas que não coletam imagens.
        DeclareLaunchArgument('data_camera', default_value='false'),
        # Gravação das fotos (data_recorder), ligada sempre que a câmera de
        # aquisição estiver: pasta (padrão dados/fotos/ no projeto) e
        # distância mínima entre fotos (m).
        DeclareLaunchArgument('data_record', default_value='true'),
        DeclareLaunchArgument('data_dir', default_value=''),
        DeclareLaunchArgument('data_min_distance', default_value='0.3'),
        DeclareLaunchArgument('vslam', default_value='false'),
        DeclareLaunchArgument('vslam_localization', default_value='false'),
        DeclareLaunchArgument('vslam_db', default_value=''),
        DeclareLaunchArgument('rtabmap_viz', default_value='false'),
        # Checagem de patinagem das rodas pela odometria visual (com
        # ekf:=true e camera:=true; sem efeito sem eles).
        DeclareLaunchArgument('slip_check', default_value='true'),
        # Teleoperação: joy (joystick) ou keyboard (teclado, num terminal
        # próprio). Vazio: sem teleoperação, o Nav2 manda direto no /cmd_vel.
        DeclareLaunchArgument('teleop', default_value=''),
        # Antes do Webots: se faltar o ekf, a câmera ou o mapa salvo, o launch
        # para com o erro sem abrir o Webots.
        vslam,
        teleop,
        webots,
        agrobot_driver,
        robot_state_publisher,
        ekf_node,
        slip_check,
        slam,
        navigation,
        mission,
        recorder,
        rviz,
        shutdown_on_webots_exit,
    ])
