"""从实测CSV/JSON生成可追溯验收波形；不连接设备，不修改原始数据。"""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import shutil
import statistics

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[1]
plt.rcParams.update({'font.sans-serif':['Microsoft YaHei','DejaVu Sans'],
                     'axes.unicode_minus':False,'font.size':10})
COLORS={'left':'#008c7d','right':'#cc7a00'}
PARAMETERS={
    107:'P 0.7→0.3；I 0→0.010；恢复PI，无新增补偿',
    108:'P 0.3→0.6；I 常规0.010 / 反向误差释放0.050；新增快速释放',
    109:'P 0.6不变；I 0.010/0.050；仅大幅反向误差快速释放',
    110:'P 0.6→0.3；I 0.010/0.050不变；其余算法不变',
    111:'P 0.3；I 0.010/0.050；只增加诊断，不改控制算法',
    112:'P 0.3；I 0.010/0.050；新增起转时积分削减至±150',
    113:'P 0.3；I 0.010/0.050；只增加角度诊断，不改控制算法',
    114:'P 0.3；I 0.010/0.050；闭环反馈改用约30ms角度差分',
    115:'P 0.3不变；I 0.010/0.050→统一0.010；取消快速释放',
    116:'P 0.3；I 0.010；起转由限积分改为限制PI总输出约±150',
    117:'P 0.3；I 0.010；速度算法不变，只移除临时诊断',
    118:'P 0.3；I 0.010；速度算法不变，只修UART发送竞争',
    119:'P 0.3；I 0.010不变；持续低速100ms后重新启用起转保护',
    120:'P 0.3；稳态I 0.010；5–10rpm启动I 0.030，增量≤6/拍、输出≤450',
    121:'右轮反转P 0.30→0.45；其余P 0.30；稳态I 0.010不变；启动策略沿用v120',
    122:'P右轮反转0.45/其余0.30；稳态I 0.010、启动I 0.030不变；起转阈值1→0.5rpm',
    123:'P右轮反转0.45/其余0.30；I稳态0.010/启动0.030；撤回v122，起转阈值恢复1rpm',
    124:'P右轮反转0.45/其余0.30；I稳态0.010/启动0.030；控制同v123，仅新增起步诊断',
    125:'P/I同v124；仅右轮≤10rpm反转起转释放上限150→180；保留启动诊断',
}


def archive(source,output,manifest):
    """原样复制输入并登记SHA256，避免后续控制台重启覆盖历史证据。"""
    target=output/'raw'/f'{source.parent.name}_{source.name}'
    target.parent.mkdir(parents=True,exist_ok=True)
    shutil.copyfile(source,target)
    manifest.append({'source':str(source.relative_to(ROOT)),
                     'snapshot':str(target.relative_to(output)),
                     'sha256':hashlib.sha256(target.read_bytes()).hexdigest()})
    return target


def plot_series(ax,rows,field,label,color,origin,scale=1,style='-'):
    """逐点画实测值；无效反馈、时间回退或超过350ms缺口处断线。"""
    xs=[];ys=[];previous=None
    for row in rows:
        now=int(row['uptime_ms'])
        valid=str(row.get('valid',True)).lower()=='true'
        if previous is not None and (now<previous or now-previous>350):
            xs.append(float('nan'));ys.append(float('nan'))
        xs.append((now-origin)/1000)
        ys.append(float(row[field])/scale if valid else float('nan'))
        previous=now
    ax.plot(xs,ys,style,color=color,label=label,marker='.',markersize=3,linewidth=1)


def save(fig,output,name):
    """同时保存PNG预览和SVG矢量原图。"""
    if name.startswith('v'):
        version=int(name[1:].split('_')[0])
        if version in PARAMETERS:
            fig.suptitle(fig._suptitle.get_text()+'\n'+PARAMETERS[version]+'（I为每10ms积分系数，D=0）',fontsize=11)
    fig.savefig(output/(name+'.png'),dpi=150)
    fig.savefig(output/(name+'.svg'))
    plt.close(fig)


def motion_charts(source,version,output,manifest):
    """按真实非零目标段绘图，包含前后停机样本及失败短段，不挑选最佳曲线。"""
    raw=archive(source,output,manifest)
    rows=[r for r in csv.DictReader(raw.open(encoding='utf-8-sig')) if int(r['version'])==version]
    groups=[];group=[]
    for i,row in enumerate(rows):
        moving=int(row['target_left']) or int(row['target_right'])
        if moving:
            if group and (row['session']!=rows[group[-1]]['session'] or
                          (row['target_left'],row['target_right'])!=(rows[group[-1]]['target_left'],rows[group[-1]]['target_right']) or
                          not 0<=int(row['uptime_ms'])-int(rows[group[-1]]['uptime_ms'])<=1000):
                groups.append(group);group=[]
            group.append(i)
        elif group:groups.append(group);group=[]
    if group:groups.append(group)
    results=[]
    for index,ids in enumerate(groups,1):
        a=[rows[i] for i in ids];start=int(a[0]['uptime_ms']);end=int(a[-1]['uptime_ms'])
        # 仅从同一段附近顺序取样，避免复位后的同名时间混入。
        context=rows[max(0,ids[0]-20):min(len(rows),ids[-1]+21)]
        context=[r for r in context if start-1500<=int(r['uptime_ms'])<=end+1800]
        fig,axes=plt.subplots(3,1,figsize=(11,7.5),sharex=True,layout='constrained')
        for side in COLORS:
            plot_series(axes[0],context,'rpm_'+side,('左' if side=='left' else '右')+'实际',COLORS[side],start,36)
            plot_series(axes[0],context,'target_'+side,('左' if side=='left' else '右')+'目标',COLORS[side],start,36,'--')
            plot_series(axes[1],context,'output_'+side,('左' if side=='left' else '右')+'输出',COLORS[side],start)
        for field,color,label in [('fault','#c43232','故障码'),('state','#486cb0','状态'),('io_errors','#9b4db0','UART错误累计')]:
            plot_series(axes[2],context,field,label,color,start)
        axes[0].set_ylim(-22,22)
        for ax,label in zip(axes,['轮速 / rpm','控制电流命令 / 原始单位','状态 / 计数']):
            ax.set_ylabel(label);ax.grid(alpha=.22);ax.legend(loc='upper right',ncol=3,fontsize=8)
        axes[-1].set_xlabel('相对首次记录非零目标的时间 / s（非按钮点击时刻）')
        feedback='原始C610速度字段' if version<114 else '固件角度差分速度（约30ms窗口）'
        fig.suptitle(f'v{version} · 运动段 {index} · {feedback}\n实测遥测逐点绘制，无额外平滑；点间连线不代表连续测量')
        name=f'v{version}_trial_{index:02d}';save(fig,output,name)
        tail=[r for r in a if int(r['uptime_ms'])>=end-3000]
        result={'figure':name,'version':version,'samples':len(a),'duration_s':(end-start)/1000,
                'target_rpm':float(a[0]['target_left'])/36,
                'target_right_rpm':float(a[0]['target_right'])/36,
                'faults':sorted({int(r['fault']) for r in context}),
                'after_startup_3s':{}}
        for side in COLORS:
            values=[float(r['rpm_'+side])/36 for r in tail]
            result[side]={'mean':statistics.mean(values),'std':statistics.pstdev(values)}
            # 另存起步3秒后的整个区间，避免只取末3秒漏掉中途波动；短段不虚构稳态。
            settled=[float(r['rpm_'+side])/36 for r in a
                     if int(r['uptime_ms'])>=start+3000 and str(r.get('valid',True)).lower()=='true']
            result['after_startup_3s'][side]=({'samples':len(settled),
                'mean':statistics.mean(settled),'std':statistics.pstdev(settled),
                'min':min(settled),'max':max(settled)} if settled else None)
        results.append(result)
    # 即使发生故障时目标已经归零，也独立保留故障前后波形。
    for code in sorted({int(r['fault']) for r in rows}-{0}):
        at=next(i for i,r in enumerate(rows) if int(r['fault'])==code)
        center=int(rows[at]['uptime_ms']);context=[r for r in rows[max(0,at-50):at+50] if abs(int(r['uptime_ms'])-center)<=3500]
        fig,axes=plt.subplots(3,1,figsize=(11,7),sharex=True,layout='constrained')
        for side in COLORS:plot_series(axes[0],context,'rpm_'+side,side,COLORS[side],center,36)
        plot_series(axes[1],context,'sequence','STM32接收序号','#486cb0',center)
        for f,c in [('fault','#c43232'),('io_errors','#9b4db0')]:plot_series(axes[2],context,f,f,c,center)
        for ax in axes:ax.grid(alpha=.2);ax.legend()
        axes[0].set_ylabel('轮速 / rpm');axes[1].set_ylabel('序号');axes[2].set_ylabel('故障 / 错误计数');axes[2].set_xlabel('相对首次记录故障 / s')
        fig.suptitle(f'v{version} 故障{code}现场：保留零目标阶段故障及遥测缺口')
        save(fig,output,f'v{version}_fault_{code}')
    return results


def watch_chart(source,output,manifest,name):
    """绘制零速观察原始轮询快照；重复的MCU时间保留，不能当独立采样。"""
    rows=json.loads(archive(source,output,manifest).read_text(encoding='utf-8'))
    origin=rows[0]['uptime_ms'];fig,axes=plt.subplots(4,1,figsize=(11,9),sharex=True,layout='constrained')
    for side in COLORS:
        plot_series(axes[0],rows,'rpm_'+side,side,COLORS[side],origin,36)
        plot_series(axes[1],rows,'output_'+side,side,COLORS[side],origin)
    for f,c in [('fault','#c43232'),('io_errors','#9b4db0')]:plot_series(axes[2],rows,f,f,c,origin)
    plot_series(axes[3],rows,'age_ms','遥测年龄','#486cb0',origin)
    for ax,label in zip(axes,['轮速 / rpm','控制输出','故障 / UART错误','年龄 / ms']):ax.set_ylabel(label);ax.grid(alpha=.2);ax.legend()
    axes[-1].set_xlabel('相对首个MCU快照时间 / s')
    unique=len({r['uptime_ms'] for r in rows})
    fig.suptitle(f'{name} · 零目标观察（不是运动验收）\n{len(rows)}个轮询快照，{unique}个不同MCU时间；仅覆盖已保存区间')
    save(fig,output,name)


def trace_chart(source,version,output,manifest):
    """绘制控制周期原始诊断；v113另标出角度计算值，区分测量与派生数据。"""
    rows=json.loads(archive(source,output,manifest).read_text(encoding='utf-8'))
    fig,axes=plt.subplots(3,1,figsize=(11,8),sharex=True,layout='constrained')
    times=[(r[0]-rows[0][0])/1000 for r in rows]
    for side,k in [('left',0),('right',1)]:
        # 原始诊断各字段：时间、目标LR、反馈LR、输出LR、积分LR，随后才是角度/原始RPM。
        for ax,offset,label in [(axes[0],3,'反馈'),(axes[1],5,'输出'),(axes[2],7,'积分')]:
            scale=36 if offset==3 else 1
            ax.plot(times,[r[offset+k]/scale for r in rows],'.-',ms=3,lw=.8,color=COLORS[side],label=side+' '+label)
        axes[0].plot(times,[r[1+k]/36 for r in rows],'--',color=COLORS[side],lw=.8,label=side+' 目标')
        if version==113:
            positions=[0]
            for prev,cur in zip(rows,rows[1:]):
                delta=(cur[9+k]-prev[9+k]+4096)%8192-4096
                positions.append(positions[-1]+delta*(1 if k==0 else -1))
            derived=[float('nan')]*3
            for i in range(3,len(rows)):
                dt=rows[i][0]-rows[i-3][0]
                derived.append((positions[i]-positions[i-3])*60000/(8192*dt*36))
            axes[0].plot(times,derived,lw=2,color=COLORS[side],alpha=.65,label=side+' 角度派生≈30ms')
    for ax,label in zip(axes,['轮速 / rpm','控制输出 / 原始单位','积分 / 原始单位']):ax.set_ylabel(label);ax.legend(ncol=2,fontsize=8);ax.grid(alpha=.2)
    axes[-1].set_xlabel('相对诊断首个样本 / s（仅RAM保留的片段，非完整试验）')
    fig.suptitle(f'v{version} · 控制周期诊断原始记录\n所有有效样本逐点绘制；v113粗线明确为角度派生值，其余线不额外平滑')
    save(fig,output,f'v{version}_control_trace')


def comparison_chart(output):
    """用同一测速方式的两轮5rpm实测作前后示例，完整重复记录仍留在索引中。"""
    fig,axes=plt.subplots(2,2,figsize=(13,7),sharex=True,sharey='row',layout='constrained')
    for col,version in enumerate([114,117]):
        source=output/'raw'/f'tuning_v{version}_repeats.csv'
        rows=[r for r in csv.DictReader(source.open(encoding='utf-8-sig')) if int(r['version'])==version]
        groups=[];group=[]
        for row in rows:
            if int(row['target_left'])==180:group.append(row)
            elif group:groups.append(group);group=[]
        if group:groups.append(group)
        a=groups[0] if version==114 else groups[-1]
        start=int(a[0]['uptime_ms'])
        for side in COLORS:
            plot_series(axes[0,col],a,'rpm_'+side,side,COLORS[side],start,36)
            plot_series(axes[1,col],a,'output_'+side,side,COLORS[side],start)
        axes[0,col].axhline(5,color='#555555',ls='--',label='目标5rpm')
        axes[0,col].set_ylim(-2,20)
        axes[0,col].set_title(f'v{version} / '+('5rpm第1段：停转再冲起\nP=0.3，I=0.010/0.050；起转只限积分' if version==114 else '5rpm第2段：趋稳，起步仍不同步\nP=0.3，I=0.010；起转限制PI总输出'))
        axes[1,col].set_xlabel('相对首个非零目标样本 / s')
        for ax in axes[:,col]:ax.grid(alpha=.2);ax.legend(ncol=3,fontsize=8)
    axes[0,0].set_ylabel('轮速 / rpm');axes[1,0].set_ylabel('控制输出 / 原始单位')
    fig.suptitle('修改方向：取消过快积分释放 + 起转时计入比例项限制总输出\n同为约30ms角度差分反馈，无绘图平滑；两段示例不替代全部重复验收')
    save(fig,output,'comparison_5rpm_v114_v117')


def main():
    """默认补绘历史验收；也支持指定新版本CSV供后续每轮修复复用。"""
    parser=argparse.ArgumentParser();parser.add_argument('--csv',type=Path);parser.add_argument('--version',type=int)
    parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    output=args.output;output.mkdir(parents=True,exist_ok=True);manifest=[];results=[]
    if args.csv:
        if args.version is None:parser.error('--csv requires --version')
        results=motion_charts(args.csv.resolve(),args.version,output,manifest)
    else:
        for version in [107,108,109,110,112,114,115,116,117]:
            folder=ROOT/'out'/f'tuning_v{version}'
            source=folder/('repeats.csv' if (folder/'repeats.csv').exists() else 'motion.csv')
            results+=motion_charts(source,version,output,manifest)
        for folder,name in [('tuning_v117_http11','v117_UART故障观察'),('tuning_v118','v118_一分钟零速观察')]:
            watch_chart(ROOT/'out'/folder/'zero_watch.json',output,manifest,name)
        for version in [111,112,113,114]:
            filename='trace5_round3.json' if version==114 else 'trace.json'
            trace_chart(ROOT/'out'/f'tuning_v{version}'/filename,version,output,manifest)
        comparison_chart(output)
    (output/'manifest.json').write_text(json.dumps(manifest,indent=2,ensure_ascii=False),encoding='utf-8')
    (output/'measurements.json').write_text(json.dumps(results,indent=2,ensure_ascii=False),encoding='utf-8')
    lines=['# 原始实测波形索引','','图像来自随附 raw 快照，SHA256 见 manifest.json。未额外平滑、删异常点或补采样。',
           'v114之前为C610原始速度字段；v114之后为固件实际闭环所用角度差分速度，二者不能当成同一测量方式直接比较噪声。',
           '短时中断段不是完整稳态试验；统计仅辅助阅读，不自动判定验收通过。','','|版本/段|目标rpm|记录非零目标时长s|左均值/标准差|右均值/标准差|','|---|---:|---:|---:|---:|']
    for r in results:
        lines.append(f'|[{r["figure"]}]({r["figure"]}.png)|{r["target_rpm"]:g}|{r["duration_s"]:.2f}|{r["left"]["mean"]:.2f}/{r["left"]["std"]:.2f}|{r["right"]["mean"]:.2f}/{r["right"]["std"]:.2f}|')
    for image in sorted(output.glob('*.png')):lines+=['',f'## {image.stem}','',f'![{image.stem}]({image.name})',f'[SVG矢量图]({image.with_suffix(".svg").name})']
    (output/'README.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    print(f'{len(results)} motion segments; {len(list(output.glob("*.png")))} figures; {len(manifest)} archived inputs')


if __name__=='__main__':main()
