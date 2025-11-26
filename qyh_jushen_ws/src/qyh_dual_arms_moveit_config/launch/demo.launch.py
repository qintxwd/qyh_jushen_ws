import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Get package directories
    moveit_config_pkg = get_package_share_directory('qyh_dual_arms_moveit_config')
    description_pkg = get_package_share_directory('qyh_dual_arms_description')
    
    # Build MoveIt configuration
    moveit_config = (
        MoveItConfigsBuilder("qyh_dual_arms")
        .robot_description(
            file_path=os.path.join(description_pkg, "urdf", "dual_arms.urdf.xacro")
        )
        .robot_description_semantic(
            file_path=os.path.join(moveit_config_pkg, "config", "qyh_dual_arms.srdf")
        )
        .trajectory_execution(
            file_path=os.path.join(moveit_config_pkg, "config", "moveit_controllers.yaml")
        )
        .planning_pipelines(
            pipelines=["ompl"],
            default_planning_pipeline="ompl"
        )
        .to_moveit_configs()
    )
    
    # MoveGroup Node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "publish_robot_description": True,
                "publish_robot_description_semantic": True,
                "use_sim_time": False,
            }
        ],
    )
    
    # RViz Node
    rviz_base = os.path.join(moveit_config_pkg, "config")
    rviz_config = os.path.join(rviz_base, "moveit.rviz") if os.path.exists(os.path.join(rviz_base, "moveit.rviz")) else ""
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config] if rviz_config else [],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )
    
    # Static TF for world->base_link
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="virtual_joint_broadcaster",
        output="log",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
    )
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )
    
    # Joint State Publisher (for demo mode)
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[
            {"source_list": ["move_group/fake_controller_joint_states"]},
        ],
    )

    return LaunchDescription([
        move_group_node,
        rviz_node,
        static_tf_node,
        robot_state_publisher,
        joint_state_publisher,
    ])
