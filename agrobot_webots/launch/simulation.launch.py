import os
import re
import shutil
import tempfile

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
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


def driver_urdf(robot_description, ekf, camera):
    """
    Caminho do URDF para o driver, com as trocas pedidas no launch.

    Com ekf, o driver não publica /odom; sem camera, não liga a câmera do
    VSLAM (o Webots também deixa de renderizá-la).
    """
    replacements = []
    if ekf:
        replacements.append((ODOM_FROM_DRIVER, ODOM_FROM_EKF))
    if not camera:
        replacements.append((CAMERA_ON, CAMERA_OFF))
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


VSLAM_DB_DIR = os.path.join('~', '.ros', 'agrobot_vslam')
VSLAM_CAMERA = [
    ('rgb/image', '/vslam_camera/color/image_raw'),
    ('rgb/camera_info', '/vslam_camera/color/camera_info'),
    ('depth/image', '/vslam_camera/aligned_depth_to_color/image_raw'),
]


def flag(context, name):
    return LaunchConfiguration(name).perform(context).lower() == 'true'


def vslam_database(context):
    """
    Banco do RTAB-Map e se ele é de mapeamento.

    O padrão é ~/.ros/agrobot_vslam/<mundo>.db. O mapeamento apaga o banco ao
    começar; a localização roda numa cópia, para o mapa salvo não mudar.
    """
    path = LaunchConfiguration('vslam_db').perform(context)
    if not path:
        world = LaunchConfiguration('world').perform(context)
        name = os.path.splitext(os.path.basename(world))[0]
        path = os.path.join(VSLAM_DB_DIR, name + '.db')
    path = os.path.abspath(os.path.expanduser(path))
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
                    driver_urdf(robot_description, ekf, camera)},
                {'use_sim_time': True},
            ],
            output='screen',
        )]

    agrobot_driver = OpaqueFunction(function=start_driver)

    # Rodas + IMU -> /odom e TF odom -> base_link (no lugar do driver).
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(webots_share, 'config', 'ekf.yaml'),
            {'use_sim_time': True},
        ],
        remappings=[('odometry/filtered', '/odom')],
        condition=IfCondition(LaunchConfiguration('ekf')),
    )

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
        nodes = [
            Node(
                package='rtabmap_sync',
                executable='rgbd_sync',
                name='rgbd_sync',
                namespace='rtabmap',
                output='screen',
                parameters=[config, {'use_sim_time': True}],
                remappings=VSLAM_CAMERA,
            ),
            Node(
                package='rtabmap_slam',
                executable='rtabmap',
                name='rtabmap',
                namespace='rtabmap',
                output='screen',
                parameters=[config, rtabmap_parameters,
                            {'use_sim_time': True}],
                remappings=[('scan', '/scan')],
            ),
        ]
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
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(webots_share, 'launch', 'navigation.launch.py')),
            launch_arguments={'global_frame': global_frame(context)}.items(),
            condition=IfCondition(LaunchConfiguration('nav')),
        )]

    navigation = OpaqueFunction(function=start_navigation)

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
        modes = [m for m in ('ekf', 'slam', 'vslam') if flag(context, m)]
        if 'vslam' in modes and flag(context, 'vslam_localization'):
            # Localização num mapa salvo: outro rótulo nos CSVs, para não
            # misturar com as rodadas de mapeamento. Hífen, e não '_', porque
            # o summarize_missions.py separa caso, modo e data por '_'.
            modes[modes.index('vslam')] = 'vslam-loc'
        return [Node(
            package='agrobot_webots',
            executable='waypoint_mission',
            output='screen',
            parameters=[{
                'waypoints_file': path,
                'frame': global_frame(context),
                'run_label': '+'.join(modes) or 'odom',
                'world': LaunchConfiguration('world_name').perform(context),
                'use_sim_time': True,
            }],
        )]

    mission = OpaqueFunction(function=start_mission)

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
        DeclareLaunchArgument('vslam', default_value='false'),
        DeclareLaunchArgument('vslam_localization', default_value='false'),
        DeclareLaunchArgument('vslam_db', default_value=''),
        DeclareLaunchArgument('rtabmap_viz', default_value='false'),
        # Antes do Webots: se faltar o ekf, a câmera ou o mapa salvo, o launch
        # para com o erro sem abrir o Webots.
        vslam,
        webots,
        agrobot_driver,
        robot_state_publisher,
        ekf_node,
        slam,
        navigation,
        mission,
        rviz,
        shutdown_on_webots_exit,
    ])
