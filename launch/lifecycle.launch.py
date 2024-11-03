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
    
    # 修正: node_name -> name, node_executable -> executable
    demo_node = LifecycleNode(
        name='demo_node',
        namespace='',
        package='launch_lifecycle_demo',
        executable='lifecycle',
        output='screen'
    )

    # 非アクティブ状態（inactive）への遷移イベントをエミット
    to_inactive = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(demo_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    # 「unconfigured」から「inactive」への状態遷移時のイベントハンドラ
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

    # 「configuring」から「inactive」に遷移したとき、「active」へのイベントをエミット
    from_inactive_to_active = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=demo_node, 
            start_state='configuring',
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

    # 「active」から「inactive」に遷移する際のイベントハンドラ
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

    # 「deactivating」から「inactive」に遷移したとき、シャットダウンするイベントをエミット
    from_inactive_to_finalized = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=demo_node, 
            start_state='deactivating',
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

    # 「finalized」状態に遷移したとき、システムをシャットダウンするイベントをエミット
    from_finalized_to_exit = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=demo_node,
            goal_state='finalized',
            entities=[
                launch.actions.LogInfo(msg="<< Finalized >>"),
                launch.actions.EmitEvent(event=launch.events.Shutdown()),
            ],
        )
    )
    
    # LaunchDescriptionにすべてのアクションとイベントハンドラを追加
    ld.add_action(from_unconfigured_to_inactive)
    ld.add_action(from_inactive_to_active)
    ld.add_action(from_active_to_inactive)
    ld.add_action(from_inactive_to_finalized)
    ld.add_action(from_finalized_to_exit)
    
    ld.add_action(demo_node)
    ld.add_action(to_inactive)

    return ld
