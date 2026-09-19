'use strict';
const $=id=>document.getElementById(id); // 按固定 ID 取得界面控件。
const owner=crypto.randomUUID();
let csrf='',armed=false,pending=0,move='',upgrading=false,lastValid=0,epoch=0,armGrace=0,heartbeatTimer=null;
const samples=[];
let trialTimer=null,trialSpeed=0;

/** 显示操作结果，不把请求成功等同于实物已经完成运动。 */
function notice(text){$('notice').textContent=text;}

/** 向本地服务提交带同源令牌的请求，并解码业务错误。 */
async function post(path,body,binary=false){
  const response=await fetch(path,{method:'POST',headers:{'X-Console-Token':csrf,'Content-Type':binary?'application/octet-stream':'application/json'},body:binary?body:JSON.stringify(body)});
  const data=await response.json();if(!response.ok)throw Error(data.error||'请求失败');return data;
}

/** 仅停止遇到明确忙锁时有限重试；不排队运动，每次提交新的停止时间戳。 */
async function postStop(body){
  for(let attempt=0;attempt<3;attempt++){
    try{return await post('/api/control',{...body,at:Date.now()});}
    catch(error){
      if(attempt===2||!error.message.includes('设备忙'))throw error;
      await new Promise(resolve=>setTimeout(resolve,40));
    }
  }
}

/** 将当前按压方向转换为左右电机轴转速；轮速乘 36 得到电机轴 rpm。 */
function targets(){
  const speed=(trialSpeed||Number($('speed').value))*36;
  return {forward:[speed,speed],back:[-speed,-speed],left:[-speed,speed],right:[speed,-speed]}[move]||[0,0];
}

/** 更新方向按压外观；只有明确使能且页面前台时才允许设置非零目标。 */
function direction(value){
  move=armed&&!document.hidden?value:'';
  document.querySelectorAll('[data-move]').forEach(button=>button.classList.toggle('held',button.dataset.move===move));
}

/** 发送即时控制；发生任何不确定结果即撤销本地控制权，不自动重试使能。 */
async function command(op){
  if(pending && op!==0)return;
  if(!csrf)return;
  const [left,right]=op===2?targets():[0,0];
  const startedEpoch=epoch,startedAt=Date.now();let startHeartbeat=false;pending++;
  try{
    const result=op===0?await postStop({op,owner,left,right}):await post('/api/control',{op,owner,left,right,at:Date.now()});
    if(op===1&&startedEpoch===epoch){armed=result.state===1;direction('');startHeartbeat=armed;armGrace=Date.now()+250;notice('已使能。按住方向行驶，松开回零。');}
    if(op===0||op===3){armed=false;direction('');notice(op===0?'已确认禁止驱动。':'故障已清除，请检查反馈后重新使能。');}
  }catch(error){armed=false;direction('');notice(error.message+'；已停止发送运动心跳。');}
  finally{pending--;}
  // 使能确认后立即发送首个零速心跳，避免额外等待定时器；停止后的迟到应答不能续命。
  if(startHeartbeat && armed && startedEpoch===epoch && !document.hidden)void command(2);
  else if(op===2 && startedEpoch===epoch)scheduleHeartbeat(startedAt,startedEpoch);
}

/** 根据本次发送时间安排下一心跳；慢请求完成即续发，且始终只有一个在途请求。 */
function scheduleHeartbeat(startedAt,startedEpoch){
  clearTimeout(heartbeatTimer);
  if(!armed||upgrading||document.hidden||startedEpoch!==epoch||Date.now()-lastValid>=500)return;
  heartbeatTimer=setTimeout(()=>{
    heartbeatTimer=null;
    if(armed&&!pending&&!upgrading&&!document.hidden&&startedEpoch===epoch&&Date.now()-lastValid<500)void command(2);
  },Math.max(0,100-(Date.now()-startedAt)));
}

/** 立即停止本地心跳并请求禁止驱动；请求失败时由 STM32 超时保护兜底。 */
function stop(){clearTimeout(trialTimer);trialTimer=null;trialSpeed=0;clearTimeout(heartbeatTimer);heartbeatTimer=null;epoch++;armed=false;direction('');void command(0);}

/** 在架空状态执行单次定时试验；时长仅6或20秒，不自动使能，到时禁止驱动。 */
function timedTrial(heading='forward'){
  const speed=Number($('speed').value);
  const seconds=Number($('trialDuration').value);
  if(![6,20].includes(seconds)||!['forward','back'].includes(heading)||!armed||pending||upgrading||document.hidden||Date.now()-lastValid>=300||trialTimer!==null||![5,10,12].includes(speed))return;
  trialSpeed=speed;
  trialTimer=setTimeout(stop,seconds*1000);
  direction(heading);
  notice(`架空试验进行中，${seconds} 秒后自动禁止驱动；可随时按停止。`);
}

/** 绘制固定 ±20 rpm 标尺的实际/目标曲线；无反馈时保留空白而非模拟数据。 */
function draw(){
  const canvas=$('chart'),ratio=devicePixelRatio||1,w=canvas.clientWidth,h=235;
  canvas.width=w*ratio;canvas.height=h*ratio;
  const c=canvas.getContext('2d');c.scale(ratio,ratio);c.font='10px Consolas';
  for(let rpm=-20;rpm<=20;rpm+=10){const y=15+(20-rpm)/40*(h-35);c.strokeStyle='#2b3a42';c.beginPath();c.moveTo(27,y);c.lineTo(w,y);c.stroke();c.fillStyle='#9aaeb8';c.fillText(String(rpm),0,y+3);}
  for(const [field,color,dash] of [['rpm_left','#64e1ce',[]],['rpm_right','#f4ba67',[]],['target_left','#64e1ce',[4,4]],['target_right','#f4ba67',[4,4]]]){
    c.strokeStyle=color;c.lineWidth=1.6;c.setLineDash(dash);c.beginPath();let pen=false;
    samples.forEach((sample,i)=>{if(!sample.valid){pen=false;return;}const x=27+i/299*(w-27),y=15+(20-sample[field]/36)/40*(h-35);if(pen)c.lineTo(x,y);else c.moveTo(x,y);pen=true;});c.stroke();
  }c.setLineDash([]);
}

/** 刷新状态，并对过期或丢失的反馈撤销页面使能；永不自动恢复驾驶。 */
async function poll(){
  try{
    const response=await fetch('/api/state');const data=await response.json();csrf=data.csrf;
    const t=data.telemetry,valid=t.valid===true;
    if(valid)lastValid=Date.now();
    $('connection').textContent=valid?'● 已连接 · 数据实时':'● 离线 / 数据过期';
    $('connection').style.color=valid?'var(--cyan)':'var(--amber)';
    $('state').textContent=valid?(['禁止驱动','已使能','故障锁定','正在升级'][t.state]||'未知状态'):'离线';
    $('detail').textContent=valid?`固件 v${t.version} · ${t.feedback_mask===3?'双轮反馈正常':'等待电机反馈'} · 故障码 ${t.fault}`:(t.error||'没有新鲜遥测，驾驶功能已禁用。');
    $('left').textContent=valid?(t.rpm_left/36).toFixed(1):'—';$('right').textContent=valid?(t.rpm_right/36).toFixed(1):'—';$('age').textContent=valid?t.age_ms:'—';
    $('current').textContent=valid?`反馈电流：左 ${t.current_left} / 右 ${t.current_right}（原始单位）`:'反馈电流：—';
    $('arm').disabled=!valid||t.state!==0||t.feedback_mask!==3||upgrading;
    $('trial').disabled=!valid||!armed||pending>0||trialTimer!==null||![5,10,12].includes(Number($('speed').value))||upgrading;
    $('trialBack').disabled=$('trial').disabled;
    if(trialTimer!==null && (!valid||t.state!==1||t.fault||Math.abs(t.rpm_left)>720||Math.abs(t.rpm_right)>720))stop();
    $('clear').disabled=!valid||t.state!==2||t.feedback_mask!==3||upgrading;
    if(!valid || (t.state!==1 && !pending && Date.now()>armGrace)){armed=false;direction('');}
    document.querySelectorAll('[data-move]').forEach(button=>button.disabled=!valid||!armed||upgrading);
    samples.push(valid?t:{valid:false});if(samples.length>300)samples.shift();draw();
    if(upgrading)$('upgradeStatus').textContent=({uploading:'正在上传并校验固件…',installing:'正在写入 STM32 并等待新 App…',complete:'升级成功，已确认新 App 版本。',failed:'升级失败：'+(data.upgrade.error||'')}[data.upgrade.state]||'正在准备升级…');
  }catch(error){armed=false;direction('');$('state').textContent='控制台连接中断';notice(error.message);}
  finally{setTimeout(poll,100);}
}

/** 上传用户选择的完整 OTA 包；开始后停止控制心跳并展示阶段结果。 */
async function install(){
  if(armed){notice('请先点击停止并禁止驱动。');return;}
  const file=$('firmware').files[0];if(!file){notice('请先选择 .ota 固件包。');return;}
  upgrading=true;$('install').disabled=true;$('upgradeStatus').textContent='正在准备升级…';
  try{const result=await post('/api/upgrade',await file.arrayBuffer(),true);$('upgradeStatus').textContent=`升级完成 · App v${result.running_version} · ${result.transferred} 字节`;}
  catch(error){$('upgradeStatus').textContent='升级失败：'+error.message;}
  finally{upgrading=false;$('install').disabled=false;}
}

/** 注册按住驾驶、失焦停车和空格停车事件；不提供锁定持续运动。 */
function bind(){
  $('arm').onclick=()=>command(1);$('clear').onclick=()=>command(3);$('stop').onclick=stop;$('install').onclick=install;
  $('trial').onclick=()=>timedTrial('forward');
  $('trialBack').onclick=()=>timedTrial('back');
  $('speed').oninput=()=>{$('limit').textContent=$('speed').value;};
  document.querySelectorAll('[data-move]').forEach(button=>{
    button.addEventListener('pointerdown',event=>{event.preventDefault();button.setPointerCapture(event.pointerId);direction(button.dataset.move);});
    button.addEventListener('pointerup',()=>direction(''));button.addEventListener('pointercancel',stop);button.addEventListener('lostpointercapture',()=>direction(''));
  });
  window.addEventListener('blur',stop);document.addEventListener('visibilitychange',()=>{if(document.hidden)stop();});
  window.addEventListener('keydown',event=>{if(event.code==='Space'){event.preventDefault();stop();}});
  window.addEventListener('resize',draw);
  // 心跳由使能应答启动并串行调度；页面关闭或网络丢失后 MCU 300 ms 超时。
}
bind();void poll();
