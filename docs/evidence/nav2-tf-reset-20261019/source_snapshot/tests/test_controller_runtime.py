"""运行真实 FollowPath 服务；不启动 Gazebo，不连接驱动器。"""
import json
import os
from pathlib import Path
import subprocess
import uuid


def test_tf_fault_and_repeated_reset(tmp_path):
    """错误 TF 不得报到达，连续 Reset/Startup 后仍能执行和取消新的有效动作。"""
    tool = Path(__file__).resolve().parents[3]/'tools/verify_controller.py'
    binary = Path(os.environ['SMARTCAR_TEST_CONTROLLER'])
    output = tmp_path/'controller_runtime'
    result = subprocess.run(
        ['/usr/bin/python3', str(tool), '--output', str(output), '--executable', str(binary),
         '--domain-id', str(100+uuid.uuid4().int%120), '--cycles', '3'],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=80)
    assert result.returncode == 0, result.stdout
    report = json.loads((output/'result.json').read_text())
    assert report['result'] == 'PASS', report
