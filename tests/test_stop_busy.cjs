const fs=require('fs'),vm=require('vm'),assert=require('assert');
const source=fs.readFileSync('desktop/app.js','utf8').split('/** 绘制固定')[0];
/** 执行真实网页命令代码；假HTTP忙响应不会接触车辆。 */
async function run(op,busyCount,message='设备忙，请停止操作后重试'){
  const calls=[],notice={};
  const ctx=vm.createContext({crypto:{randomUUID:()=> 'test'},Date,setTimeout,clearTimeout,
    document:{hidden:false,getElementById:id=>id==='notice'?notice:{value:'5'},querySelectorAll:()=>[]},
    fetch:async(path,options)=>{calls.push(JSON.parse(options.body));const busy=calls.length<=busyCount;
      return {ok:!busy,json:async()=>busy?{error:message}:{result:0,state:0}};}});
  vm.runInContext(source+"\ncsrf='test';armed=true;",ctx);
  await vm.runInContext(`command(${op})`,ctx);
  return {calls,notice,ctx};
}
/** 验证停止能跨短暂忙锁，永久忙有界，非停止命令及其他错误不重试。 */
async function main(){
  const recovered=await run(0,2);
  assert.equal(recovered.calls.length,3);
  assert(recovered.calls.every(c=>c.op===0&&c.left===0&&c.right===0));
  assert(recovered.calls[2].at>recovered.calls[0].at);
  assert.equal(recovered.notice.textContent,'已确认禁止驱动。');
  assert.equal(vm.runInContext('pending',recovered.ctx),0);
  const failed=await run(0,99);assert.equal(failed.calls.length,3);
  assert(failed.notice.textContent.includes('设备忙'));
  for(const op of [1,2,3])assert.equal((await run(op,99)).calls.length,1);
  assert.equal((await run(0,99,'令牌无效')).calls.length,1);
  console.log('PASS: only STOP retries busy; fresh timestamps; 3 attempt bound; no motion retries');
}
main().catch(error=>{console.error(error);process.exitCode=1;});
