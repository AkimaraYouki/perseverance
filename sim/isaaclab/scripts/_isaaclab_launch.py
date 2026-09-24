"""IsaacLab 스크립트에 위임하기 전에 이 패키지를 import 한다.

IsaacLab 의 train.py/play.py 는 isaaclab_tasks 만 import 하므로,
외부 패키지의 gym.register() 가 같은 프로세스 안에서 먼저 돌지 않으면
--task Isaac-WheeledBiped-Balance-v0 이 NameNotFound 로 실패한다.
별도 프로세스에서 import 를 확인하는 것으로는 해결되지 않는다
(gym 레지스트리는 프로세스마다 따로다).

train.py 가 sibling 인 cli_args 를 import 하므로 그 디렉터리도 sys.path 에 넣는다.
"""

import os
import runpy
import sys

import wheeled_biped_isaaclab.tasks  # noqa: F401

target = sys.argv.pop(1)
sys.path.insert(0, os.path.dirname(os.path.abspath(target)))
sys.argv[0] = target
runpy.run_path(target, run_name="__main__")
