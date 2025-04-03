import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition

def create_nodes(context, *args, **kwargs):
    model_folder = 'turtlebot3_burger'
    robot_desc_path = os.path.join(get_package_share_directory("turtlebot3_gazebo"), "urdf", "turtlebot3_burger.urdf")
    template = os.path.join(get_package_share_directory('turtlebot3_gazebo'),'models',model_folder,'turtlebot3_model_template.sdf.xacro')
    nav_launch_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch', 'nav2_bringup')
    params_file = LaunchConfiguration('nav_params_file')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    declare_params_file_cmd = DeclareLaunchArgument(
        'nav_params_file',
        default_value=os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')
    with open(robot_desc_path, 'r') as infp:
        robot_desc = infp.read()
    bots = int(LaunchConfiguration('bots').perform(context))
    x_positions_array = str(LaunchConfiguration(
        'x_positions'
    ).perform(context))
    y_positions_array = str(LaunchConfiguration(
        'y_positions',
    ).perform(context))

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time', default_value=use_sim_time, description='Use simulator time'
    )

    enable_drive = LaunchConfiguration('enable_drive', default='false')
    declare_enable_drive = DeclareLaunchArgument(
        name='enable_drive', default_value=enable_drive, description='Enable robot drive node'
    )

    enable_rviz = LaunchConfiguration('enable_rviz', default='true')
    declare_enable_rviz = DeclareLaunchArgument(
        name='enable_rviz', default_value=enable_rviz, description='Enable rviz launch'
    )

    rviz_config_file = LaunchConfiguration('rviz_config_file')
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
    default_value=os.path.join(
        get_package_share_directory('turtlebot3_gazebo'), 'rviz', 'multi_nav2_default_view.rviz'),
    description='Full path to the RVIZ config file to use')
    
    x_positions = x_positions_array.split(',')
    y_positions = y_positions_array.split(',')
    nodes =[]
    nodes.append(declare_enable_drive)
    nodes.append(declare_enable_rviz)
    nodes.append(declare_use_sim_time)
    nodes.append(declare_rviz_config_file_cmd)
    nodes.append(declare_params_file_cmd)
    names = ["tb3_"+str(i) for i in range(bots)]
    for i in range(bots):
        name = names[i]
        alter_name = ""
        modified_sdf = os.path.join(get_package_share_directory('turtlebot3_gazebo'),'models/model'+str(i)+'.sdf')
        doc = xacro.process_file(template, mappings={'robot_id': str(i)})
        with open(modified_sdf, 'w') as output_file:
            output_file.write(doc.toprettyxml())
        if i == 0:
            alter_name = name[1]
        else:
            alter_name = name[0]
        spawn_robot = Node(
            package='gazebo_ros', 
            executable='spawn_entity.py', 
            arguments=[
                '-entity', name,
                '-file', modified_sdf, 
                '-x', x_positions[i], 
                '-y', y_positions[i], 
                '-z', '0.01',
                '-robot_namespace', name,
            ],
            output='screen'
        )
        robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=name,
            output='screen',
            parameters=[{'frame_prefix': name + '/',
                        'use_sim_time': True,
                        'robot_description': robot_desc}]
        )
        async_slam_toolbox = Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='async_slam_toolbox_node',
            namespace=name,
            parameters=[{
                'use_sim_time': True,
                'odom_frame': name + '/odom',
                'base_frame': name + '/base_footprint',
                'scan_topic': 'scan',
                'map_frame': name + '/map',
                'minimum_travel_distance': 0.3,
                'minimum_travel_heading': 0.3,
                'resolution': 0.05,
            }],
            remappings=[
                ("/map", "map"),
                ("/map_metadata", "map_metadata"),
                ("/slam_toolbox/scan_visualization", "slam_toolbox/scan_visualization"),
                ("/slam_toolbox/graph_visualization", "slam_toolbox/graph_visualization"),
            ],
            output='screen',
        )
        pathFollow = Node(
            package='path_follow',
            executable='path_follow',
            name='pathFollow',
            output='screen',
            namespace=name,
            parameters=[{
                'use_sim_time': True,
            }],
            remappings=[
                ("/path", "/"+name+"/path"),
                ("/cmd_vel", "/"+name+"/cmd_vel"),
                ("/odom", "/"+name+"/odom"),
                ("/visual_path", "/"+alter_name+"/visual_path"),
            ]
        ) 
        bringup_cmd = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav_launch_dir, 'bringup_launch.py')),
                    launch_arguments={  
                                    'slam': 'False',
                                    'namespace': name,
                                    'use_namespace': 'True',
                                    'map': '',
                                    'map_server': 'False',
                                    'params_file': params_file,
                                    'default_bt_xml_filename': os.path.join(
                                        get_package_share_directory('nav2_bt_navigator'),
                                        'behavior_trees', 'navigate_w_replanning_and_recovery.xml'),
                                    'autostart': 'true',
                                    'use_sim_time': use_sim_time, 'log_level': 'warn'}.items()
                                    )
        nodes.append(spawn_robot)
        nodes.append(robot_state_publisher)
        nodes.append(async_slam_toolbox)
        nodes.append(pathFollow)
        nodes.append(bringup_cmd)
    last_action = None
    # Start rviz nodes and drive nodes after the last robot is spawned
    for i in range(bots):

        # Create a initial pose topic publish call
        message = '{header: {frame_id: map}, pose: {pose: {position: {x: ' + \
            x_positions[i]+ ', y: ' + y_positions[i] + \
            ', z: 0.1}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0000000}}, }}'

        initial_pose_cmd = ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '-1', '--qos-reliability', 'reliable', [name[i]] + ['/initialpose'],
                'geometry_msgs/PoseWithCovarianceStamped', message],
            output='screen'
        )

        rviz_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav_launch_dir, 'rviz_launch.py')),
                launch_arguments={'use_sim_time': use_sim_time, 
                                  'namespace': name[i],
                                  'use_namespace': 'True',
                                  'rviz_config': rviz_config_file, 'log_level': 'warn'}.items(),
                                   condition=IfCondition(enable_rviz)
                                    )

        drive_turtlebot3_burger = Node(
            package='turtlebot3_gazebo', executable='turtlebot3_drive',
            namespace=name[i], output='screen',
            condition=IfCondition(enable_drive),
        )

        # Use RegisterEventHandler to ensure next robot rviz launch happens 
        # only after all robots are spawned
        post_spawn_event = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=last_action,
                on_exit=[initial_pose_cmd, rviz_cmd, drive_turtlebot3_burger],
            )
        )

        # Perform next rviz and other node instantiation after the previous intialpose request done
        last_action = initial_pose_cmd

        nodes.append(post_spawn_event)
        nodes.append(declare_params_file_cmd)
    ######################
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    world = os.path.join(get_package_share_directory('turtlebot3_gazebo'),'worlds',str(LaunchConfiguration('world').perform(context)))
    # GAZEBO
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world,'verbose':"true",'extra_gazebo_args': 'verbose'}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        ),
        launch_arguments={'verbose':"true"}.items()
    )    
    nodes.append(gzserver_cmd)
    nodes.append(gzclient_cmd)
    return nodes

def generate_launch_description():
    bots = DeclareLaunchArgument(
        'bots',
        default_value='2'
    )
    ld = LaunchDescription()
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = DeclareLaunchArgument(
        'world',
        default_value='Warehouse_BharatForge.world',
        description='World',
    )
    declare_x_positions = DeclareLaunchArgument(
        'x_positions',
        default_value='[5.0, -2.0, -6.2, -4.0]',
        description='List of x positions for robots',
    )
    declare_y_positions = DeclareLaunchArgument(
        'y_positions',
        default_value='[2.5, 3.0, -3.5, 3.0]',
        description='List of y positions for robots',
    )
    
    ld.add_action(bots)
    ld.add_action(declare_x_positions)
    ld.add_action(declare_y_positions)
    ld.add_action(world)
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]
    map_server=Node(package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'map', 'map.yaml'),
                     },],
        remappings=remappings)

    map_server_lifecyle=Node(package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': True},
                        {'node_names': ['map_server']}])


    ld.add_action(map_server)
    ld.add_action(map_server_lifecyle)
   
    bot_nodes = OpaqueFunction(function=create_nodes)
    ld.add_action(bot_nodes)
    return ld