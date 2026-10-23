from pathlib import Path
import sys,json,time
import rclpy
from std_msgs.msg import String
from std_srvs.srv import Trigger
p=Path(sys.argv[1]).resolve();s=json.loads((p/'mission/session.json').read_text())
report={'reason':'Serial validation restart after overlapping owned simulations triggered unchanged scan and heartbeat protection','expected_output_dir':str(p/'mission'),'state_before':s['state']}
if s['state'] in ['DONE','ERROR']:
    report['request_skipped_completed']=True
else:
    rclpy.init();node=rclpy.create_node('verified_session_finish');latest={}
    def receive(msg):
        """只接受本次独立会话的状态，防止跨启动请求发给下一轮。"""
        latest.update(json.loads(msg.data))
    node.create_subscription(String,'/exploration/status',receive,10);client=node.create_client(Trigger,'/exploration/finish');end=time.monotonic()+10
    while time.monotonic()<end and latest.get('output_dir')!=str(p/'mission'):rclpy.spin_once(node,timeout_sec=.1)
    if latest.get('output_dir')!=str(p/'mission'):raise RuntimeError('Expected session not present; no finish request sent')
    if not client.wait_for_service(timeout_sec=3.):raise RuntimeError('Expected finish service unavailable')
    future=client.call_async(Trigger.Request());rclpy.spin_until_future_complete(node,future,timeout_sec=10.)
    if not future.done():raise RuntimeError('Finish response timeout')
    response=future.result();report.update(success=response.success,message=response.message,observed_status=latest)
    node.destroy_node();rclpy.shutdown()
(p/'intervention_load_overlap.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))
