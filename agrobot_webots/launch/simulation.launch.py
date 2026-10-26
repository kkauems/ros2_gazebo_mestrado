import os
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


ODOM_FROM_DRIVER = '<publishOdom>true</publishOdom>'
ODOM_FROM_EKF = '<publishOdom>false</publishOdom>'
CAMERA_OFF = '<enableVslamCamera>false</enableVslamCamera>'
CAMERA_ON = '<enableVslamCamera>true</enableVslamCamera>'

# Tópicos da câmera RGB-D publicados pelo AgrobotDriver (nomes do realsense-ros).
VSLAM_CAMERA_REMAPPINGS = [
    ('rgb/image', '/vslam_camera/color/image_raw'),
    ('rgb/camera_info', '/vslam_camera/color/camera_info'),
    ('depth/image', '/vslam_camera/aligned_depth_to_color/image_raw'),
]


def driver_urdf(robot_description, ekf, vslam):
    """
    Caminho do URDF para o driver.

    O driver do Webots lê as propriedades do plugin no URDF, não parâmetros
    ROS. Com ekf, o driver não publica /odom; com vslam, liga a câmera RGB-D.
    """
    replacements = []
    if ekf:
        replacements.append((ODOM_FROM_DRIVER, ODOM_FROM_EKF))
    if vslam:
        replacements.append((CAMERA_OFF, CAMERA_ON))
    if not replacements:
        return robot_description
    with open(robot_description) as urdf_file:
        content = urdf_file.read()
    for old, new in replacements:
        if old not in content:
            raise RuntimeError(f'{old} não encontrado em {robot_description}')
        content = content.replace(old, new)
    path = os.path.join(tempfile.gettempdir(), 'agrobot_webots_driver.urdf')
    with open(path, 'w') as urdf_file:
        urdf_file.write(content)
    return path


def is_true(context, name):
    return LaunchConfiguration(name).perform(context).lower() == 'true'


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
        ekf = is_true(context, 'ekf')
        vslam = is_true(context, 'vslam')
        if vslam and not ekf:
            # A odometria do RTAB-Map é o TF do EKF, e as variâncias dos
            # elos (rtabmap.yaml) supõem o EKF.
            raise RuntimeError('vslam:=true exige ekf:=true')
        urdf = driver_urdf(robot_description, ekf, vslam)
        return [WebotsController(
            robot_name='agrobot',
            parameters=[
                {'robot_description': urdf},
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

    def start_slam(context):
        """
        SLAM com o LiDAR: /map, map -> odom e /pose (covariância do scan matching).

        Com vslam:=true (modo combinado) o dono de map -> odom é o RTAB-Map:
        o slam_toolbox continua mapeando e publicando /pose, mas sem TF e no
        frame slam_map, que não existe no TF (só para gravar e comparar).
        """
        if not is_true(context, 'slam'):
            return []
        overrides = {'use_sim_time': True}
        vslam = is_true(context, 'vslam')
        if vslam:
            overrides.update({
                'transform_publish_period': 0.0,
                'map_frame': 'slam_map',
                'map_name': '/slam_toolbox/map',
            })
        # O nó é lifecycle: o launch o configura e ativa.
        slam_toolbox = LifecycleNode(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            namespace='',
            output='screen',
            parameters=[
                os.path.join(webots_share, 'config', 'slam_toolbox.yaml'),
                overrides,
            ],
        )
        slam_configure = EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_toolbox),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ))
        slam_activate = RegisterEventHandler(OnStateTransition(
            target_lifecycle_node=slam_toolbox,
            start_state='configuring',
            goal_state='inactive',
            entities=[EmitEvent(event=ChangeState(
                lifecycle_node_matcher=matches_action(slam_toolbox),
                transition_id=Transition.TRANSITION_ACTIVATE,
            ))],
        ))
        actions = [slam_toolbox, slam_configure, slam_activate]
        if vslam:
            # A elipse do ekf_map_node usa o /pose no frame map; no modo
            # combinado o /pose está em slam_map, então ele não sobe.
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

    def start_vslam(context):
        """
        VSLAM com a câmera RGB-D: RTAB-Map, dono de map -> odom.

        rgbd_sync junta cor, profundidade e camera_info em /rtabmap/rgbd_image;
        o rtabmap lê a odometria do TF (EKF) e monta a grade com o /scan.
        """
        if not is_true(context, 'vslam'):
            return []
        rtabmap_yaml = os.path.join(webots_share, 'config', 'rtabmap.yaml')
        database = LaunchConfiguration('vslam_db').perform(context)
        if not database:
            world = os.path.splitext(
                LaunchConfiguration('world_name').perform(context))[0]
            database = os.path.join('~', '.ros', 'agrobot_vslam', world + '.db')
        database = os.path.expanduser(database)
        os.makedirs(os.path.dirname(database), exist_ok=True)
        nodes = [
            Node(
                package='rtabmap_sync',
                executable='rgbd_sync',
                name='rgbd_sync',
                namespace='rtabmap',
                output='screen',
                parameters=[rtabmap_yaml, {'use_sim_time': True}],
                remappings=VSLAM_CAMERA_REMAPPINGS,
            ),
            Node(
                package='rtabmap_slam',
                executable='rtabmap',
                name='rtabmap',
                namespace='rtabmap',
                output='screen',
                parameters=[rtabmap_yaml, {
                    'use_sim_time': True,
                    # Mapeamento: começa um mapa novo a cada execução.
                    'database_path': database,
                    'delete_db_on_start': True,
                }],
                remappings=[('scan', '/scan')],
            ),
        ]
        if is_true(context, 'rtabmap_viz'):
            nodes.append(Node(
                package='rtabmap_viz',
                executable='rtabmap_viz',
                name='rtabmap_viz',
                namespace='rtabmap',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'subscribe_rgbd': True,
                    'subscribe_scan': True,
                    'qos_scan': 2,
                    'approx_sync': True,
                    'frame_id': 'base_link',
                    'odom_frame_id': 'odom',
                }],
                remappings=[('scan', '/scan')],
            ))
        return nodes

    vslam = OpaqueFunction(function=start_vslam)

    def global_frame(context):
        # Com SLAM ou VSLAM existe map -> odom e o Nav2 planeja em map; sem,
        # em odom.
        mapped = is_true(context, 'slam') or is_true(context, 'vslam')
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
        # caminho para um .yaml. Vazio: sem missão.
        name = LaunchConfiguration('waypoints').perform(context)
        if not name:
            return []
        path = name if os.path.isfile(name) else os.path.join(
            webots_share, 'config', 'waypoints',
            name if name.endswith('.yaml') else name + '.yaml')
        if not os.path.isfile(path):
            raise RuntimeError(f'Arquivo de waypoints não encontrado: {path}')
        modes = [m for m in ('ekf', 'slam', 'vslam') if is_true(context, m)]
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
        # Com slam, o RViz usa o frame map e mostra o /map e as elipses; com
        # vslam, o /rtabmap/map.
        if is_true(context, 'vslam'):
            config = 'nav_vslam.rviz'
        elif is_true(context, 'slam'):
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
        DeclareLaunchArgument('vslam', default_value='false'),
        # Banco do RTAB-Map; vazio: ~/.ros/agrobot_vslam/<mundo>.db.
        DeclareLaunchArgument('vslam_db', default_value=''),
        DeclareLaunchArgument('rtabmap_viz', default_value='false'),
        DeclareLaunchArgument('waypoints', default_value=''),
        # O WebotsLauncher troca 'world' pelo caminho de uma cópia temporária;
        # o nome original fica guardado (waypoint_mission e banco do RTAB-Map).
        SetLaunchConfiguration('world_name', LaunchConfiguration('world')),
        webots,
        agrobot_driver,
        robot_state_publisher,
        ekf_node,
        slam,
        vslam,
        navigation,
        mission,
        rviz,
        shutdown_on_webots_exit,
    ])
