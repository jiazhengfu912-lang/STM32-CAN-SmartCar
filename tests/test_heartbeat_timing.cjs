// 用虚拟时钟重放慢请求，真实网页函数不访问网络或硬件。
const fs=require('node:fs'),vm=require('node:vm'),assert=require('node:assert/strict');
const source=fs.readFileSync(process.argv[2]||'desktop/app.js','utf8');
/** 创建虚拟网络及定时器，记录模拟 STM32 接受命令的间隔。 */
function fixture(){
 let now=1000,id=0,calls=0,last=null,fault=false;const timers=new Map(),accepted=[];
 const elements={speed:{value:5},notice:{textContent:''}};
 /** 将定时任务加入虚拟事件队列，可模拟旧版重复节拍。 */
 function schedule(fn,delay,repeat=0){const key=++id;timers.set(key,{fn,at:now+delay,repeat});return key;}
 const context=vm.createContext({console,crypto:{randomUUID:()=> 'test'},Date:{now:()=>now},
  document:{hidden:false,getElementById:key=>elements[key],querySelectorAll:()=>[]},
  setTimeout:(fn,delay)=>schedule(fn,delay),clearTimeout:key=>timers.delete(key),
  setInterval:(fn,delay)=>schedule(fn,delay,delay),
  fetch:(path,options)=>new Promise(resolve=>{
   const command=JSON.parse(options.body);const delay=++calls===2?213:101;
   schedule(()=>{if(command.op===2){if(last!==null&&now-last>=300)fault=true;last=now;accepted.push(now);}
    resolve({ok:!fault,json:async()=>fault?{error:'STM32 command timeout'}:{state:1}});},delay);
  })});
 vm.runInContext(source.split('/** 绘制固定')[0]+"\ncsrf='test';armed=true;lastValid=100000;",context);
 // 旧版的定时器位于 bind 内；保留其真实表达式以重放漏过节拍的问题。
 const interval=source.split('\n').find(line=>line.includes('setInterval('));
 if(interval)vm.runInContext(interval,context);
 /** 推进虚拟时间并排空异步应答，不等待真实时钟。 */
 async function advance(end){
  while(true){for(let i=0;i<12;i++)await Promise.resolve();
   const next=[...timers].filter(([,t])=>t.at<=end).sort((a,b)=>a[1].at-b[1].at)[0];
   if(!next)break;
   now=next[1].at;timers.delete(next[0]);if(next[1].repeat)timers.set(next[0],{...next[1],at:now+next[1].repeat});next[1].fn();
  }now=end;
 }
 return {context,advance,accepted,get fault(){return fault;},get calls(){return calls;}};
}
/** 验证慢请求不额外等待下一节拍，且失焦、过期、停止后不再续发。 */
async function main(){
 const f=fixture();vm.runInContext('void command(2)',f.context);await f.advance(2000);
 assert.equal(f.fault,false,'101/213 ms requests plus missed timer tick must not cause 300 ms timeout');
 assert.ok(f.accepted.length>=5);const count=f.calls;
 vm.runInContext('document.hidden=true',f.context);await f.advance(2400);assert.equal(f.calls,count);
 for(const cancellation of ["armed=false;epoch++", "lastValid=0", "upgrading=true"]){
  const g=fixture();vm.runInContext('void command(2)',g.context);vm.runInContext(cancellation,g.context);await g.advance(1500);assert.equal(g.calls,1);
 }
 console.log('PASS: slow requests stay within lease; hidden/stale/stopped/upgrading pages do not renew.');
}
main().catch(error=>{console.error(error);process.exitCode=1;});
