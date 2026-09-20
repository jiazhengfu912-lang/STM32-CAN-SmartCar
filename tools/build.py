"""构建独立 STM32 App / Bootloader，并同步 Keil 工程；不连接或烧录设备。"""
from pathlib import Path
import argparse
import re
import json
import os
import subprocess
import sys
import xml.etree.ElementTree as ET
import copy

ROOT = Path(__file__).resolve().parents[1]
APP = ROOT / 'firmware/stm32_app'
BOOT = ROOT / 'firmware/stm32_bootloader'
HAL = APP / 'Drivers/STM32F1xx_HAL_Driver'
ARM = Path(os.environ.get('ARM_BIN', r'D:\Keil_v5\ARM\ARMCLANG\bin'))
INCLUDES = [APP/'Core/Inc', HAL/'Inc', APP/'Drivers/CMSIS/Include',
            APP/'Drivers/CMSIS/Device/ST/STM32F1xx/Include', ROOT/'protocol', BOOT]
HAL_NAMES = ['hal', 'hal_can', 'hal_uart', 'hal_i2c', 'hal_dma', 'hal_rcc', 'hal_rcc_ex',
             'hal_gpio', 'hal_cortex', 'hal_flash', 'hal_flash_ex', 'hal_pwr']
APP_SOURCES = list((APP/'Core/Src').glob('*.c')) + [HAL/'Src'/f'stm32f1xx_{n}.c' for n in HAL_NAMES] + [
    ROOT/'protocol/car_protocol.c', BOOT/'update_request.c', BOOT/'ota_receiver.c']
BOOT_SOURCES = [BOOT/n for n in ['main.c','uart_service.c','boot_guard.c','ota_receiver.c','update_request.c','stm32_flash.c']] + [
    ROOT/'protocol/car_protocol.c',APP/'Core/Src/system_stm32f1xx.c']


def run(args):
    """执行工具并在首个失败处中止，避免使用旧构建产物报成功。"""
    subprocess.run([str(x) for x in args], check=True, cwd=ROOT)


def sync_keil():
    """修正 C8T6 容量并同步源列表、头文件路径和 App scatter 文件。"""
    path=APP/'MDK-ARM/SmartCar.uvprojx'
    tree=ET.parse(path); root=tree.getroot()
    changes={'TargetName':'SmartCar', 'Device':'STM32F103C8',
        'Cpu':'IRAM(0x20000000,0x5000) IROM(0x08004000,0xB800) CPUTYPE("Cortex-M3") CLOCK(72000000) ELITTLE',
        'OutputDirectory':'..\\..\\..\\out\\keil\\', 'OutputName':'SmartCar', 'ListingPath':'..\\..\\..\\out\\keil\\',
        'useUlib':'1', 'ScatterFile':'./app.sct', 'umfTarg':'0','useFile':'1'}
    for key,value in changes.items():
        for node in root.iter(key): node.text=value
    for node in root.iter():
        if node.text and 'STM32F103T6' in node.text: node.text=node.text.replace('STM32F103T6','STM32F103C8')
    # 目标内存描述也同步，避免 IDE 再生成 scatter 时恢复旧地址。
    for parent in root.findall('.//OnChipMemories/*'):
        if parent.find('StartAddress') is not None:
            address=parent.find('StartAddress')
            if address.text=='0x08000000': address.text='0x08004000'; parent.find('Size').text='0xB800'
            if address.text=='0x20000000': parent.find('Size').text='0x5000'
    c=root.find('.//Cads/VariousControls')
    c.find('Define').text='USE_HAL_DRIVER,STM32F103xB,__MICROLIB'
    c.find('IncludePath').text=';'.join(os.path.relpath(p,path.parent).replace('\\','/') for p in INCLUDES)
    c.find('MiscControls').text='-Oz -Wall'
    root.find('.//Aads/VariousControls/Define').text='__MICROLIB'
    groups=root.find('.//Groups'); groups.clear()
    group=ET.SubElement(groups,'Group'); ET.SubElement(group,'GroupName').text='SmartCar sources'
    files=ET.SubElement(group,'Files')
    for source in [APP/'MDK-ARM/startup_stm32f103xb.s']+APP_SOURCES:
        f=ET.SubElement(files,'File'); ET.SubElement(f,'FileName').text=source.name
        ET.SubElement(f,'FileType').text='2' if source.suffix=='.s' else '1'
        ET.SubElement(f,'FilePath').text=os.path.relpath(source,path.parent).replace('\\','/')
    ET.indent(tree,space='  ')
    tree.write(path,encoding='utf-8',xml_declaration=True,short_empty_elements=False)
    # 独立 Bootloader 也提供可直接打开的 MDK 工程，不依赖原仓库。
    boot_path=BOOT/'MDK-ARM/Bootloader.uvprojx';boot_path.parent.mkdir(exist_ok=True)
    bt=copy.deepcopy(tree); br=bt.getroot()
    for key,value in {'TargetName':'Bootloader','OutputName':'Bootloader',
                       'OutputDirectory':'..\\..\\..\\out\\keil_boot\\',
                       'ListingPath':'..\\..\\..\\out\\keil_boot\\',
                       'ScatterFile':'../boot.sct',
                       'Cpu':'IRAM(0x20000000,0x5000) IROM(0x08000000,0x4000) CPUTYPE("Cortex-M3") CLOCK(8000000) ELITTLE'}.items():
        for node in br.iter(key):node.text=value
    c=br.find('.//Cads/VariousControls')
    c.find('Define').text='STM32F103xB,__MICROLIB,M2_BOOT,M2_UART_UPDATE'
    c.find('IncludePath').text=';'.join(os.path.relpath(p,boot_path.parent).replace('\\','/') for p in INCLUDES)
    files=br.find('.//Groups/Group/Files');files.clear()
    for source in [APP/'MDK-ARM/startup_stm32f103xb.s']+BOOT_SOURCES:
        f=ET.SubElement(files,'File');ET.SubElement(f,'FileName').text=source.name
        ET.SubElement(f,'FileType').text='2' if source.suffix=='.s' else '1'
        ET.SubElement(f,'FilePath').text=os.path.relpath(source,boot_path.parent).replace('\\','/')
    for parent in br.findall('.//OnChipMemories/*'):
        if parent.find('StartAddress') is not None and parent.find('StartAddress').text=='0x08004000':
            parent.find('StartAddress').text='0x08000000';parent.find('Size').text='0x4000'
    ET.indent(bt,space='  ');bt.write(boot_path,encoding='utf-8',xml_declaration=True,short_empty_elements=False)


def build(kind, sources):
    """使用 Arm Compiler 6 构建并导出 HEX/BIN，链接器强制检查分区上限。"""
    output=ROOT/'out'/kind; output.mkdir(parents=True,exist_ok=True)
    objects=[output/'startup.o']
    run([ARM/'armasm.exe','--cpu=Cortex-M3','--pd','__MICROLIB SETA 1',
         APP/'MDK-ARM/startup_stm32f103xb.s','-o',objects[0]])
    flags=['-c','--target=arm-arm-none-eabi','-mcpu=cortex-m3','-mthumb','-std=c99',
           '-Oz','-g','-Wall','-Wextra','-Werror','-ffunction-sections','-fdata-sections',
           '-DSTM32F103xB','-D__MICROLIB']+['-I'+str(p) for p in INCLUDES]
    if kind=='app': flags+=['-DUSE_HAL_DRIVER']
    else: flags+=['-DM2_UART_UPDATE','-DM2_BOOT']
    for i,source in enumerate(sources):
        obj=output/f'{i}.o'; extra=['-Dmain=diagnostic_main'] if source.name=='uart_service.c' else []
        run([ARM/'armclang.exe',*flags,*extra,source,'-o',obj]); objects.append(obj)
    scatter=APP/'MDK-ARM/app.sct' if kind=='app' else BOOT/'boot.sct'
    run([ARM/'armlink.exe','--cpu=Cortex-M3','--library_type=microlib','--scatter',scatter,
         '--map','--symbols','--info=sizes','--list',output/f'{kind}.map','--output',output/f'{kind}.axf',*objects])
    for option,extension in [('--bin','bin'),('--i32combined','hex')]:
        run([ARM/'fromelf.exe',option,'--output',output/f'{kind}.{extension}',output/f'{kind}.axf'])


def main():
    """按统一源码构建两份固件，并生成版本 100 的 OTA 包和出厂安装镜像。"""
    sync_keil(); build('app',APP_SOURCES); build('boot',BOOT_SOURCES)
    sys.path.insert(0,str(BOOT)); import package
    app=(ROOT/'out/app/app.bin').read_bytes(); boot=(ROOT/'out/boot/boot.bin').read_bytes()
    package.check_vectors(app,package.APP_BASE,package.APP_CAPACITY)
    package.check_vectors(boot,package.FLASH_BASE,16384)
    # 包版本直接取自 App 定义，避免升级元数据与运行时版本不一致。
    version=int(re.search(r'#define\s+DRIVE_VERSION\s+(\d+)U', (APP/'Core/Inc/drive.h').read_text(encoding='utf-8')).group(1))
    meta=package.metadata(app,version)
    (ROOT/f'out/SmartCar-v{version}.ota').write_bytes(meta+app)
    # 到元数据页为止；参数页 0x0800FC00 不在安装镜像中。
    factory=bytearray(b'\xff'*0xFC00); factory[:len(boot)]=boot
    factory[0x4000:0x4000+len(app)]=app; factory[0xF800:0xF820]=meta
    (ROOT/'out/SmartCar-factory.bin').write_bytes(factory)
    report={'result':'PASS','scope':'BUILD_ONLY','app_bytes':len(app),'app_limit':47104,
            'boot_bytes':len(boot),'version':version,'hardware_tested':False}
    (ROOT/'out/build_report.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(report))


if __name__=='__main__': main()
