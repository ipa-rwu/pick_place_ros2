from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch import LaunchDescription
from launch.substitutions import FindExecutable

import yaml
from yaml import SafeLoader

close_1 = """
        fun: 1
        pin: 0
        state: 0.0
    """

close_2 = """
        fun: 1
        pin: 0
        state: 0.0
    """

open = """
        fun: 1
        pin: 1
        state: 1.0
    """
# pin 0 close


def generate_launch_description():
    ld = LaunchDescription()

    req_close = yaml.load(close, Loader=SafeLoader)

    cmd_str_close = (
        'service call /io_and_status_controller/set_io ur_msgs/srv/SetIO "{}"'.format(
            str(req_close)
        )
    )

    req_open = yaml.load(open, Loader=SafeLoader)

    cmd_str_open = (
        'service call /io_and_status_controller/set_io ur_msgs/srv/SetIO "{}"'.format(
            str(req_open)
        )
    )

    # ld.add_action(
    #     ExecuteProcess(
    #         cmd=[[
    #             FindExecutable(name='ros2'),
    #             " {}".format(cmd_str_open)
    #         ]],
    #         shell=True
    #     )
    # )

    ld.add_action(
        ExecuteProcess(
            cmd=[[FindExecutable(name="ros2"), " {}".format(cmd_str_close)]], shell=True
        )
    )
    ld.add_action(
        ExecuteProcess(
            cmd=[[FindExecutable(name="ros2"), " {}".format(cmd_str_close)]], shell=True
        )
    )
    return ld
