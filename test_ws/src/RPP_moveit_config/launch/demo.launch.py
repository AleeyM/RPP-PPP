import os
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
import launch_ros
import launch
def generate_launch_description():
    pkgPath = launch_ros.substitutions.FindPackageShare(package='RPP').find('RPP')
    urdfModelPath= os.path.join(pkgPath, 'urdf/RPP.urdf')
    
    with open(urdfModelPath, 'r') as infp:
    	robot_desc = infp.read()
    params= {'robot_description': robot_desc}
    # Command-line arguments
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value="moveit.rviz",
        description="RViz configuration file",
    )
    db_arg = DeclareLaunchArgument(
        "db", default_value="False", description="Database flag"
    )
    ros2_control_hardware_type = DeclareLaunchArgument(
        "ros2_control_hardware_type",
        default_value="mock_components",
        description="ROS 2 control hardware interface type to use for the launch file -- possible values: [mock_components, isaac]",
    )
    #moveit_config = MoveItConfigsBuilder("RPP", package_name="RPP_moveit_config").to_moveit_configs()

#    moveit_config = (
#        MoveItConfigsBuilder("RPP")
#        .robot_description(
#            file_path="config/RPP.urdf.xacro",
#            mappings={
#                "ros2_control_hardware_type": LaunchConfiguration(
#                    "ros2_control_hardware_type"
#                )
#            },
#        )
        #.robot_description_semantic(file_path="config/RPP.srdf")
#        .planning_scene_monitor(
#            publish_robot_description=True, publish_robot_description_semantic=True
#        )
#        .trajectory_execution(file_path="config/moveit_controllers.yaml")
#        .planning_pipelines(
#            pipelines=["ompl", "stomp", "pilz_industrial_motion_planner"]
#        )
#        .to_moveit_configs()
#    )
    # Define xacro mappings for the robot description file
    launch_arguments = {
        "robot_ip": "xxx.yyy.zzz.www",
        "use_fake_hardware": "true",
        #"gripper": "robotiq_2f_85",
        #"dof": "7",
    }

    # Load the robot configuration
    moveit_config = (
        MoveItConfigsBuilder(
            "RPP", package_name="RPP_moveit_config"
        )
        .robot_description(mappings=launch_arguments)
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .planning_pipelines(
            pipelines=["ompl", "stomp", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )

    # Start the actual move_group node/action server
    
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
       # arguments=["--ros-args", "--log-level", "info"],
    )
       # RViz
    rviz_base = LaunchConfiguration("rviz_config")
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("RPP_moveit_config"), "launch", rviz_base]
    )
    rviz_node = launch_ros.actions.Node(
    	package='rviz2',
    	executable='rviz2',
    	name='rviz2',
    	output='screen',
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ]
    )
#    rviz_node = Node(
#        package="rviz2",
#        executable="rviz2",
#        name="rviz2",
#        output="log",
#        arguments=["-d", rviz_config],
#        parameters=[
#            moveit_config.robot_description,
#            moveit_config.robot_description_semantic,
#            moveit_config.robot_description_kinematics,
#            moveit_config.planning_pipelines,
#            moveit_config.joint_limits,
#        ],
#    )
        # Static TF
  #  static_tf_node = Node(
  #      package="tf2_ros",
  #      executable="static_transform_publisher",
  #      name="static_transform_publisher",
  #      output="log",
  #      arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "base_link", "world"],#
  #  )
    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    
    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("RPP_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )
    #robot_state_publisher_node= launch_ros.actions.Node(
    #package='robot_state_publisher',
    #executable='robot_state_publisher',
    #output='screen',
    #parameters=[params])
    
   # joint_state_publisher_node = launch_ros.actions.Node(
    #	package='joint_state_publisher',
    #	executable='joint_state_publisher',
    #	name='joint_state_publisher',
    #	parameters=[params],
    #	condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
   # )
    #joint_state_publisher_gui_node=launch_ros.actions.Node(
    #	package='joint_state_publisher_gui',
    #	executable='joint_state_publisher_gui',
    #	name='joint_state_publisher_gui',
    #	condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
    #)
    #panda_arm_controller_spawner = Node(
       # package="controller_manager",
      #  executable="spawner",
     #   arguments=["panda_arm_controller", "-c", "/controller_manager"],
    #)

    #panda_hand_controller_spawner = Node(
    #    package="controller_manager",
    #    executable="spawner",
    #    arguments=["panda_hand_controller", "-c", "/controller_manager"],
    #) 
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["manipulator_controller", "-c", "/controller_manager"],# joint_trajectory_controller
    )

    hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["wrist_controller", "-c", "/controller_manager"],# robotiq_gripper_controller
    )
    # Warehouse mongodb server
    db_config = LaunchConfiguration("db")
    mongodb_server_node = Node(
        package="warehouse_ros_mongo",
        executable="mongo_wrapper_ros.py",
        parameters=[
            {"warehouse_port": 33829},
            {"warehouse_host": "localhost"},
            {"warehouse_plugin": "warehouse_ros_mongo::MongoDatabaseConnection"},
        ],
        output="screen",
        condition=IfCondition(db_config),
    )

    return LaunchDescription(
        [

            rviz_config_arg,
            db_arg,
            ros2_control_hardware_type,
            rviz_node,
            static_tf ,
            robot_state_publisher,
            move_group_node,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
            hand_controller_spawner,
	    mongodb_server_node,
            
            
            
        ]
    )
    #return generate_demo_launch(moveit_config)
