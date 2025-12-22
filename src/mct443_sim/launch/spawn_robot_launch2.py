from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable

def generate_launch_description():
    # ABSOLUTE path to your world file
    world_path = "/home/mohammedaljamal/mct443_ws/src/mct443_sim/worlds/corridor_world_2.sdf"

    return LaunchDescription([
        # Force software rendering in VM so Gazebo doesn't crash
        SetEnvironmentVariable(name="LIBGL_ALWAYS_SOFTWARE", value="1"),

        # Start Gazebo with the corridor world
        ExecuteProcess(
            cmd=["gz", "sim", world_path],
            output="screen"
        ),
    ])

