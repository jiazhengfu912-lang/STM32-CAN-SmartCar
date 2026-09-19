"""浏览器功能验证：所有运动与升级 API 均拦截为测试应答，不触达硬件。"""
from pathlib import Path
import json
from playwright.sync_api import sync_playwright

ROOT=Path(__file__).resolve().parents[1]


def main():
    """检查离线禁止驾驶、按压/释放、失焦停车、升级结果与响应式布局。"""
    with sync_playwright() as p:
        browser=p.chromium.launch(headless=True,channel='msedge')
        page=browser.new_page(viewport={'width':1280,'height':1000})
        errors=[];page.on('pageerror',lambda error:errors.append(str(error)))
        # 页面持续 10 Hz 轮询，使用已渲染离线状态代替永远无法达到的 networkidle。
        page.goto('http://127.0.0.1:18765');page.wait_for_load_state('domcontentloaded')
        page.wait_for_function("document.getElementById('state').textContent==='离线'")
        assert page.locator('#arm').is_disabled()
        assert page.locator('#state').inner_text()=='离线'
        page.screenshot(path=str(ROOT/'out/console-offline.png'),full_page=True)
        state={'valid':True,'state':0,'fault':0,'feedback_mask':3,'age_ms':10,'uptime_ms':1000,
               'version':100,'target_left':0,'target_right':0,'rpm_left':0,'rpm_right':0,
               'current_left':0,'current_right':0}
        commands=[]; delayed=[]; delay_arm=[False]

        def route(request):
            """在浏览器边界注入假网关数据，记录实际页面发出的控制请求。"""
            path=request.request.url.split('18765')[-1]
            if path=='/api/state': result={'csrf':'test','telemetry':state,'upgrade':{'state':'idle'}}
            elif path=='/api/control':
                data=request.request.post_data_json;commands.append(data)
                if data['op']==1 and delay_arm[0]: delayed.append(request);return
                if data['op']==1:state['state']=1
                if data['op']==0:state['state']=0
                result={'result':0,'state':state['state']}
            elif path=='/api/upgrade':result={'result':'PASS','running_version':100,'transferred':9256}
            else:request.continue_();return
            request.fulfill(status=200,content_type='application/json',body=json.dumps(result))

        page.route('**/api/**',route)
        page.wait_for_function("!document.getElementById('arm').disabled")
        page.locator('#arm').click()
        page.wait_for_function("!document.querySelector('[data-move=forward]').disabled")
        forward=page.locator('[data-move=forward]');box=forward.bounding_box()
        page.mouse.move(box['x']+30,box['y']+30);page.mouse.down();page.wait_for_timeout(350)
        assert any(c['op']==2 and c['left']==180 and c['right']==180 for c in commands)
        page.mouse.up();page.wait_for_timeout(250)
        assert commands[-1]['left']==0 and commands[-1]['right']==0
        page.evaluate("window.dispatchEvent(new Event('blur'))")
        page.wait_for_timeout(200);assert commands[-1]['op']==0
        # 模拟 ARM 应答迟于停止操作到达，页面不得被迟到应答重新解锁。
        delay_arm[0]=True
        page.locator('#arm').click();page.wait_for_timeout(100)
        assert len(delayed)==1
        page.locator('#stop').click();page.wait_for_timeout(100)
        delayed[0].fulfill(status=200,content_type='application/json',body='{"result":0,"state":1}')
        page.wait_for_timeout(200)
        assert page.evaluate('armed') is False
        delay_arm[0]=False
        page.locator('#firmware').set_input_files(str(ROOT/'out/SmartCar-v100.ota'))
        page.locator('#install').click()
        page.wait_for_function("document.getElementById('upgradeStatus').textContent.includes('升级完成')")
        state['valid']=False
        page.wait_for_function("document.getElementById('arm').disabled")
        page.set_viewport_size({'width':390,'height':844});page.wait_for_timeout(200)
        assert page.evaluate('document.documentElement.scrollWidth<=window.innerWidth')
        page.screenshot(path=str(ROOT/'out/console-mobile.png'),full_page=True)
        assert not errors,errors
        report={'result':'PASS','scope':'BROWSER_WITH_MOCKED_DEVICE','commands':len(commands),'page_errors':errors}
        (ROOT/'out/ui_report.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
        print(json.dumps(report));browser.close()


if __name__=='__main__':main()
