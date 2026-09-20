// 运行真实网页脚本验证导航心跳、取消和缺测显示，所有HTTP均为替身。
const fs=require('node:fs'),vm=require('node:vm'),assert=require('node:assert/strict');
const source=fs.readFileSync('desktop/app.js','utf8').replace('bind();void poll();','');
const html=fs.readFileSync('desktop/index.html','utf8');
for(const match of source.matchAll(/\$\('([^']+)'\)/g))assert(html.includes(`id="${match[1]}"`),match[1]);
/** 提供最小DOM与画布环境，禁止真实网络和定时器。 */
function fixture(fetch){
 const canvas={scale(){},fillText(){},beginPath(){},moveTo(){},lineTo(){},stroke(){},setLineDash(){}};
 const elements=new Proxy({}, {get(target,id){return target[id]||(target[id]={value:5,textContent:'',style:{},disabled:false,clientWidth:400,getContext:()=>canvas});}});
 const context=vm.createContext({console,fetch,setTimeout:()=>0,clearTimeout:()=>{},crypto:{randomUUID:()=> 'test'},Date,Math,devicePixelRatio:1,
   document:{hidden:false,getElementById:id=>elements[id],querySelectorAll:()=>[]}});
 vm.runInContext(source+"\ncsrf='test';armed=true;lastValid=Date.now();imuLatest={position_valid:true};",context);
 return {context,elements};
}
/** 验证动作只启动一次，心跳为op4，失效与手动接管不续跑旧动作。 */
async function main(){
 const calls=[];
 const {context,elements}=fixture(async(path,options)=>{calls.push({path,body:JSON.parse(options.body)});return {ok:true,json:async()=>({result:0,state:1})};});
 await vm.runInContext('navigation(4,[90])',context);await new Promise(setImmediate);
 assert.equal(calls[0].path,'/api/navigation');assert.equal(calls[0].body.op,4);
 assert.equal(calls[1].path,'/api/control');assert.equal(calls[1].body.op,4);assert.equal(calls[1].body.left,0);
 await vm.runInContext('command(2)',context);assert.equal(calls[2].body.op,4);
 vm.runInContext("direction('forward')",context);await vm.runInContext('command(2)',context);assert.equal(calls.at(-1).body.op,2);
 vm.runInContext('navMode=2;showImu(null)',context);await new Promise(setImmediate);assert.equal(calls.at(-1).body.op,0);assert.equal(elements.attitude.textContent,'姿态：—');
 assert.equal(vm.runInContext('navMode',context),0);
 const late=[];let resolve;
 const other=fixture((path,options)=>{late.push(path);if(path==='/api/navigation')return new Promise(r=>{resolve=r;});return Promise.resolve({ok:true,json:async()=>({state:0})});});
 const start=vm.runInContext('navigation(3,[5])',other.context);vm.runInContext('stop()',other.context);
 resolve({ok:true,json:async()=>({state:1})});await start;await new Promise(setImmediate);
 assert.equal(vm.runInContext('navMode',other.context),0);assert.equal(late.filter(x=>x==='/api/navigation').length,1);
 console.log('PASS: navigation heartbeat, manual takeover, stale display, late start after stop (HOST ONLY)');
}
main().catch(error=>{console.error(error);process.exitCode=1;});
