"""隔离静态服务与浏览器替身验收；不读取本地凭据或访问真实网关。"""
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import threading
from playwright.sync_api import sync_playwright

ROOT=Path(__file__).resolve().parents[1]


def main():
    """验证真实DOM、移动布局、导航心跳及断流显示，保存测试截图。"""
    output=ROOT/'out/imu_browser';output.mkdir(exist_ok=True)
    server=ThreadingHTTPServer(('127.0.0.1',0),partial(SimpleHTTPRequestHandler,directory=str(ROOT/'desktop')))
    thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
    state={'valid':False,'state':0,'fault':0,'feedback_mask':3,'age_ms':10,'version':127,
           'rpm_left':0,'rpm_right':0,'current_left':0,'current_right':0,'target_left':0,'target_right':0}
    commands=[];errors=[]
    try:
        with sync_playwright() as playwright:
            browser=playwright.chromium.launch(headless=True,channel='msedge')
            page=browser.new_page(viewport={'width':1280,'height':1000})
            page.on('pageerror',lambda error:errors.append(str(error)))

            def route(request):
                """拦截全部API，不放行任何运动或配置网络请求。"""
                path=request.request.url.split('/api/')[-1]
                if path=='state': result={'csrf':'test','telemetry':state,'upgrade':{'state':'idle'}}
                elif path in ('control','navigation'):
                    data=request.request.post_data_json;commands.append({'path':path,**data})
                    if path=='control':
                        if data['op']==1:state['state']=1
                        if data['op']==0:state['state']=0;state.get('imu',{})['mode']=0
                    elif data['op'] in (3,4,9):state['imu']['mode']=3 if data['op']==9 else data['op']-2
                    result={'result':0,'state':state['state']}
                else:request.abort();return
                request.fulfill(status=200,content_type='application/json',body=json.dumps(result))

            page.route('**/api/**',route)
            page.goto(f'http://127.0.0.1:{server.server_port}/')
            # 生产页面持续轮询；以已渲染状态作就绪条件，避免等待永不出现的networkidle。
            page.wait_for_function("document.getElementById('state').textContent==='离线'")
            assert page.locator('#navForward').is_disabled()
            state.update(valid=True)
            state['imu']={'valid':True,'attitude_valid':True,'position_valid':True,'calibration':2,
                          'age_ms':10,'mode':0,'result':0,'origin_id':1,'version':127,'uptime_ms':1000,
                          'roll_rad':.1,'pitch_rad':-.1,'yaw_rad':.5,'heading_rad':.5,'x_m':.4,'y_m':.1}
            page.wait_for_function("!document.getElementById('arm').disabled")
            page.locator('#arm').click();page.wait_for_function("!document.getElementById('turnLeft').disabled")
            page.locator('#goalX').fill('1');page.locator('#goalY').fill('0.5')
            # 在同一浏览器事件中检查空闲并点击真实DOM，避免心跳恰好抢占。
            page.wait_for_function("() => {if(pending!==0)return false;document.getElementById('gotoStart').click();return true;}")
            page.wait_for_function('navMode===3');page.wait_for_timeout(250)
            assert any(c['path']=='control' and c['op']==4 for c in commands)
            assert len([c for c in commands if c['path']=='navigation'])==1
            assert next(c for c in commands if c['path']=='navigation')['args']==[1,0.5,3,0.05]
            assert '距目标' in page.locator('#gotoStatus').inner_text()
            state['imu']['valid']=False
            page.wait_for_function("document.getElementById('attitude').textContent==='姿态：—'")
            assert any(c['path']=='control' and c['op']==0 for c in commands)
            assert page.locator('#navForward').is_disabled()
            page.screenshot(path=str(output/'desktop.png'),full_page=True)
            page.set_viewport_size({'width':390,'height':844})
            page.locator('summary').click();page.wait_for_timeout(150)
            assert page.evaluate('document.documentElement.scrollWidth<=window.innerWidth')
            page.screenshot(path=str(output/'mobile.png'),full_page=True)
            assert not errors,errors
            report={'result':'PASS','scope':'BROWSER_MOCK_ONLY','hardware_tested':False,'page_errors':errors,'commands':commands}
            (output/'report.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
            print('PASS: real browser offline/navigation/stale/mobile; no hardware requests')
            browser.close()
    finally:
        server.shutdown();server.server_close();thread.join()


if __name__=='__main__':main()
