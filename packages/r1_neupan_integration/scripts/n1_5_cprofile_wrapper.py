#!/home/pinggu/venvs/neupan_ros_jazzy/bin/python
"""Profile one ROS console-script process launched by ros2 launch."""

import cProfile
import os
import runpy
import sys


def main():
    if len(sys.argv) < 2:
        raise SystemExit('usage: n1_5_cprofile_wrapper.py NODE_SCRIPT [ARGS...]')
    node_script = sys.argv[1]
    sys.argv = sys.argv[1:]
    output = os.environ.get('N1_CPROFILE_OUTPUT', 'n1_results/neupan.profile')
    profiler = cProfile.Profile()
    profiler.enable()
    try:
        runpy.run_path(node_script, run_name='__main__')
    finally:
        profiler.disable()
        profiler.dump_stats(output)


if __name__ == '__main__':
    main()
