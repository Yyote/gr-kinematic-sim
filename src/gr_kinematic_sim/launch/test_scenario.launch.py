from launch import LaunchDescription
import launch_ros.actions as actions

def generate_launch_description():
    ld = LaunchDescription()
    
    robot_types = [
        'ackerman',
        'tracked',
        'omni'
    ]
    
    robot_coords = [
        1, 2, 
        1, 1,
        3, 4  
    ]
    
    
    map_name = 'map_test.tmx'
    
    ld.add_action(actions.Node(
        package="gr_kinematic_sim",
        executable="sim",
        name="test_scenario_sim",
        parameters=[
            {'robot_types' : robot_types},
            {'robot_coords' : robot_coords},
            {'map_name' : map_name},
        ],
        emulate_tty=True, output='screen'
    ))
    

    return ld
