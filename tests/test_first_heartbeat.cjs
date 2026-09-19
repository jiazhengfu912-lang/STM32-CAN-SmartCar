// 使用真实网页控制函数及模拟 HTTP，验证首个心跳与迟到使能应答；不连接硬件。
const fs=require('node:fs'),vm=require('node:vm'),assert=require('node:assert/strict');
const source=fs.readFileSync(process.argv[2]||'desktop/app.js','utf8').split('/** 绘制固定')[0];
/** 创建隔离的网页环境，不运行定时器或真实网络。 */
function fixture(fetch){
 const elements={speed:{value:5},notice:{textContent:''}};
 const context=vm.createContext({setTimeout:()=>0,clearTimeout:()=>{},crypto:{randomUUID:()=> 'test'},Date,document:{hidden:false,getElementById:id=>elements[id],querySelectorAll:()=>[]},fetch});
 vm.runInContext(source+"\ncsrf='test';",context);return context;
}
/** 确认使能后立即发送零目标，停止后的迟到应答不再发送心跳。 */
async function main(){
 const calls=[];
 const context=fixture(async(path,options)=>{const body=JSON.parse(options.body);calls.push(body);return {ok:true,json:async()=>({state:1})};});
 await vm.runInContext('command(1)',context);
 await Promise.resolve();
 assert.deepEqual(calls.map(c=>c.op),[1,2]);
 assert.equal(calls[1].left,0);assert.equal(calls[1].right,0);
 const late=[];let resolveArm;
 const other=fixture((path,options)=>{const body=JSON.parse(options.body);late.push(body);if(body.op===1)return new Promise(resolve=>{resolveArm=resolve;});return Promise.resolve({ok:true,json:async()=>({state:0})});});
 const arm=vm.runInContext('command(1)',other);
 vm.runInContext('stop()',other);
 resolveArm({ok:true,json:async()=>({state:1})});await arm;await Promise.resolve();
 assert.equal(vm.runInContext('armed',other),false);
 assert.deepEqual(late.map(c=>c.op),[1,0]);
 console.log('PASS: immediate zero heartbeat; late ARM after STOP cannot restart heartbeat; no hardware commands.');
}
main().catch(error=>{console.error(error);process.exitCode=1;});
