import os
from launch import LaunchDescription
from launch_ros.actions import Node
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

    return LaunchDescription([
        move_group_node,
        static_tf_node,
        robot_state_publisher,
    ])
