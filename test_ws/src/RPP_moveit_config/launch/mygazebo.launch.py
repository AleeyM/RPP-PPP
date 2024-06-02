import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    pkg_share = get_package_share_directory('RPP')
    world_file_name = 'my_empty_world.world'
    world_path = os.path.join(pkg_share, 'worlds', world_file_name)
    xacro_file = os.path.join(pkg_share, 'config', 'RPP.urdf.xacro')

    log_world_path = LogInfo(msg=f'World file path: {world_path}')
    log_xacro_path = LogInfo(msg=f'Xacro file path: {xacro_file}')

    moveit_config =(
    	MoveItConfigsBuilder("RPP")
    	.robot_description(file_path=("config/RPP.urdf.xacro"))
    	.trajectory_execution(file_path="config/moveit_controllers.yaml")# moveit_simple_controllers
    	.robot_description_kinematics(file_path="config/kinematics.yaml")
    	.planning_scene_monitor(
    		publish_robot_description=True, publish_robot_description_semantic=True
    	)
    	.planning_pipelines(pipelines=["ompl"])
    	.to_moveit_configs()
    )
    
    # Robot State Publisher Node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': Command(['xacro', ' ', xacro_file]), 'use_sim_time': True}]
    )

    # Gazebo Server
    gazebo_server = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_path, '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # Spawn Entity
#    spawn_entity_robot = Node(
#        package='gazebo_ros',
#        executable='spawn_entity.py',
#        arguments=['-entity', 'RPP', '-topic', 'robot_description'],
#        output='screen',
#        emulate_tty=True,
#    )

    # Controller Manager Spawner
    #controller_manager_spawner = Node(
    #    package='controller_manager',
    #    executable='ros2_control_node',
    #    parameters=[os.path.join(pkg_share, 'config', 'ros2_controllers.yaml')],
    #    output='screen',
    #    remappings=[('/joint_states', '/robot/joint_states')]
    #)

    # Joint State Broadcaster Spawner
#    joint_state_broadcaster_spawner = Node(
#        package='controller_manager',
#        executable='spawner',
#        arguments=['joint_state_broadcaster'],
#        output='screen'
#    )

    # Manipulator Controller Spawner
#    manipulator_controller_spawner = Node(
#        package='controller_manager',
#        executable='spawner',
#        arguments=['manipulator_controller'],
#        output='screen'
#    )

    # Hand Controller Spawner
#    hand_controller_spawner = Node(
#        package='controller_manager',
#        executable='spawner',
#        arguments=['hand_controller'],
#        output='screen'
#    )


    	
    # Spawn Entity
    spawn_entity_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'rpp', '-topic', '/robot_description'],
        output='screen',
        emulate_tty=True,
    )

    load_joint_state_controller =ExecuteProcess(
    	cmd=['ros2','control','load_controller','--set-state','start','joint_state_broadcaster'],
    	output='screen'
    )

    manipulator_controller_spawner = ExecuteProcess(
    	cmd=['ros2','control','load_controller','--set-state','start','manipulator_controller'],
    	output='screen'
    )
    hand_controller_spawner = ExecuteProcess(
    	cmd=['ros2','control','load_controller','--set-state','start','hand_controller'],
    	output='screen'
    )

    return LaunchDescription([

	RegisterEventHandler(
		event_handler=OnProcessExit(
			target_action=spawn_entity_robot,
			on_exit=[load_joint_state_controller],
		)
	),
	RegisterEventHandler(
		event_handler=OnProcessExit(
			target_action=load_joint_state_controller,
			on_exit=[manipulator_controller_spawner],
		)
	),
	RegisterEventHandler(
		event_handler=OnProcessExit(
			target_action=manipulator_controller_spawner,
			on_exit=[hand_controller_spawner],
		)
	),
	
        gazebo_server,
        robot_state_publisher,
	#joint_state_broadcaster_spawner, 
	#manipulator_controller_spawner, 
	#hand_controller_spawner,
#	controller_manager_spawner,
	spawn_entity_robot
	
    ])

