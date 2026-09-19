const fs=require('fs'),vm=require('vm'),assert=require('assert');
const source=fs.readFileSync('desktop/app.js','utf8').split('/** 绘制固定')[0];
/** 建立虚拟时钟，验证定时试验不会越过停止或可用状态约束。 */
function fixture(){let now=0,id=0;const timers=new Map(),elements={speed:{value:'5'},trialDuration:{value:'6'},notice:{}};const ctx=vm.createContext({crypto:{randomUUID:()=> 'test'},Date:{now:()=>now},document:{hidden:false,getElementById:id=>elements[id],querySelectorAll:()=>[]},setTimeout:(fn,ms)=>{timers.set(++id,{fn,at:now+ms});return id;},clearTimeout:id=>timers.delete(id)});vm.runInContext(source+"\narmed=true;lastValid=0;",ctx);return {ctx,timers,elements,advance(t){now=t;for(const [k,v] of [...timers])if(v.at<=t){timers.delete(k);v.fn();}}};}
const f=fixture();vm.runInContext('timedTrial()',f.ctx);assert.equal(vm.runInContext('move',f.ctx),'forward');f.advance(5999);assert.equal(vm.runInContext('armed',f.ctx),true);f.advance(6000);assert.equal(vm.runInContext('armed',f.ctx),false);assert.equal(vm.runInContext('trialSpeed',f.ctx),0);
for(const condition of ['armed=false','document.hidden=true','upgrading=true','pending=1',"$('speed').value='20'"]){const g=fixture();vm.runInContext(condition+';timedTrial()',g.ctx);assert.equal(g.timers.size,0);}
const h=fixture();vm.runInContext('timedTrial();stop()',h.ctx);assert.equal(h.timers.size,0);
/* 后退12rpm必须双轮同向负目标；中途调速不改变本轮固定目标，到期清零并禁用。 */
const b=fixture();b.elements.speed.value='12';vm.runInContext("timedTrial('back')",b.ctx);
assert.equal(JSON.stringify(vm.runInContext('targets()',b.ctx)),'[-432,-432]');
b.elements.speed.value='20';assert.equal(JSON.stringify(vm.runInContext('targets()',b.ctx)),'[-432,-432]');
b.advance(6000);assert.equal(vm.runInContext('armed',b.ctx),false);
assert.equal(JSON.stringify(vm.runInContext('targets()',b.ctx)),'[0,0]');
const bad=fixture();vm.runInContext("timedTrial('invalid')",bad.ctx);assert.equal(bad.timers.size,0);
console.log('PASS: forward/reverse 6 second bound, manual cancel, speed and state gates');

/* 20秒上限固定于启动时；修改下拉框不能延长已开始的试验。 */
const longer=fixture();longer.elements.trialDuration.value='20';vm.runInContext("timedTrial('back')",longer.ctx);
longer.elements.trialDuration.value='6';longer.advance(19999);assert.equal(vm.runInContext('armed',longer.ctx),true);
longer.advance(20000);assert.equal(vm.runInContext('armed',longer.ctx),false);
for(const duration of ['0','21','NaN']){const invalid=fixture();invalid.elements.trialDuration.value=duration;vm.runInContext('timedTrial()',invalid.ctx);assert.equal(invalid.timers.size,0);}
console.log('PASS: 20 second bound, duration captured once, invalid duration rejected');
