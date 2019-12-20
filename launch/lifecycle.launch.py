import launch
import launch.actions
import launch.events

import launch_ros
import launch_ros.actions
import launch_ros.events

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node

import lifecycle_msgs.msg


def generate_launch_description():

    ld = launch.LaunchDescription()
    
    demo_node = launch_ros.actions.LifecycleNode(
        node_name='demo_node',
        package='launch_lifecycle_demo', node_executable='lifecycle', output='screen')

    to_inactive = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(demo_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )
    
    from_unconfigured_to_inactive = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=demo_node, 
            goal_state='unconfigured',
            entities=[
                launch.actions.LogInfo(msg="<< Unconfigured >>"),
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(demo_node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                )),
            ],
        )
    )

    from_inactive_to_active = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=demo_node, 
            start_state = 'configuring',
            goal_state='inactive',
            entities=[
                launch.actions.LogInfo(msg="<< Inactive >>"),
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(demo_node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )

    from_active_to_inactive = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=demo_node, 
            goal_state='active',
            entities=[
                launch.actions.LogInfo(msg="<< Active >>"),
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(demo_node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_DEACTIVATE,
                )),
            ],
        )
    )

    from_inactive_to_finalized = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=demo_node, 
            start_state = 'deactivating',
            goal_state='inactive',
            entities=[
                launch.actions.LogInfo(msg="<< Inactive >>"),
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(demo_node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_INACTIVE_SHUTDOWN,
                )),
            ],
        )
    )
    
    from_finalized_to_exit = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=demo_node, goal_state='finalized',
            entities=[
                launch.actions.LogInfo(msg="<< Finalized >>"),
                launch.actions.EmitEvent(event=launch.events.Shutdown()),
            ],
        )
    )
    
    ld.add_action(from_unconfigured_to_inactive)
    ld.add_action(from_inactive_to_active)
    ld.add_action(from_active_to_inactive)
    ld.add_action(from_inactive_to_finalized)
    ld.add_action(from_finalized_to_exit)
    
    ld.add_action(demo_node)
    ld.add_action(to_inactive)

    return ld