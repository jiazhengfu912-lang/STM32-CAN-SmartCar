"""编译实际 C 模块，运行故障注入、控制安全和电脑端接口测试。"""
from pathlib import Path
import os
import subprocess
import sys
import json

ROOT=Path(__file__).resolve().parents[1]
GCC=Path(os.environ.get('HOST_GCC',r'C:\msys64\ucrt64\bin\gcc.exe'))


def run(args):
    """保存每次测试的完整输出，失败立即终止并显示对应原因。"""
    result=subprocess.run([str(x) for x in args],cwd=ROOT,capture_output=True,text=True,encoding='utf-8',errors='replace',env={**os.environ,'PYTHONUTF8':'1'})
    with (ROOT/'out/tests.log').open('a',encoding='utf-8') as out: out.write(result.stdout+result.stderr)
    if result.returncode: print(result.stdout+result.stderr);raise SystemExit(result.returncode)
    print((result.stdout+result.stderr)[-450:])


def main():
    """统一编译共享库，确保 Python 测试直接运行生产代码而非替代实现。"""
    (ROOT/'out').mkdir(exist_ok=True);(ROOT/'out/tests.log').write_text('',encoding='utf-8')
    boot=ROOT/'firmware/stm32_bootloader';gateway=ROOT/'firmware/esp32_gateway';app=ROOT/'firmware/stm32_app'
    flags=['-std=c99','-Wall','-Wextra','-Werror','-O2','-I'+str(boot),'-I'+str(ROOT/'protocol'),'-I'+str(app/'Core/Inc')]
    lib=ROOT/'out/core.dll'
    run([GCC,*flags,'-shared','-o',lib,ROOT/'protocol/car_protocol.c',boot/'boot_guard.c',boot/'ota_receiver.c',gateway/'cache_store.c',gateway/'ota_sender.c'])
    for name in ['protocol','boot_guard','ota_receiver','cache_store','ota_sender']:
        run([sys.executable,'-B',ROOT/'tests'/f'test_{name}.py',lib])
    run([GCC,*flags,app/'Core/Src/drive.c',ROOT/'tests/test_drive.c','-o',ROOT/'out/test_drive.exe'])
    run([ROOT/'out/test_drive.exe'])
    run([GCC,*flags,app/'Core/Src/angle_speed.c',ROOT/'tests/test_angle_speed.c','-o',ROOT/'out/test_angle_speed.exe'])
    run([ROOT/'out/test_angle_speed.exe'])
    run([GCC,*flags,app/'Core/Src/imu.c',app/'Core/Src/navigation.c',app/'Core/Src/drive.c',app/'Core/Src/angle_speed.c',ROOT/'tests/test_imu_navigation.c','-lm','-o',ROOT/'out/test_imu_navigation.exe'])
    run([ROOT/'out/test_imu_navigation.exe'])
    run([sys.executable,'-B',ROOT/'tests/test_bsp.py'])
    run([sys.executable,'-B',ROOT/'tests/test_icm42688.py'])
    run([sys.executable,'-B',ROOT/'tests/test_imu_console.py'])
    for name in ['stop_busy','timed_trial','heartbeat_timing','first_heartbeat','navigation_ui']:
        run(['node',ROOT/'tests'/f'test_{name}.cjs'])
    run([sys.executable,'-B',ROOT/'tests/test_console.py'])
    run([sys.executable,'-B',ROOT/'tests/test_console_http.py'])
    run([sys.executable,'-B',ROOT/'tests/test_uart_tx_race.py'])
    run([sys.executable,'-B',ROOT/'tests/test_gateway_connections.py'])
    run([GCC,*flags,'-I'+str(gateway),gateway/'control_datagram.c',ROOT/'tests/test_control_datagram.c','-o',ROOT/'out/test_control_datagram.exe'])
    run([ROOT/'out/test_control_datagram.exe'])
    (ROOT/'out/test_report.json').write_text(json.dumps({'result':'PASS','scope':'HOST_SOFTWARE_ONLY','hardware_tested':False},indent=2),encoding='utf-8')


if __name__=='__main__':main()
