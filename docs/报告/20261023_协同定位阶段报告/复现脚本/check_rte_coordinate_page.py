#!/usr/bin/env python3
"""浏览器检查坐标页选项卡、单图放大、几何和响应式布局，并保存截图。

运行：PYTHONPATH=/tmp/rte_viewer_browser_deps python3 tools/check_rte_coordinate_page.py
需要Playwright和Chromium；不连接ROS或车辆。
"""
import asyncio
import json
from pathlib import Path
import threading

from playwright.async_api import async_playwright

from rte_coordinate_viewer import serve_page

ROOT = Path(__file__).resolve().parents[1]
PAGE = ROOT / 'docs/visualization/rte_coordinates.html'
OUTPUT = ROOT / 'data/rte_coordinate_validation_20261028'


async def check():
    OUTPUT.mkdir(exist_ok=True)
    errors = []
    async with async_playwright() as playwright:
        browser = await playwright.chromium.launch(headless=True)
        page = await browser.new_page(viewport={'width': 1600, 'height': 1000})
        page.on('pageerror', lambda error: errors.append(str(error)))
        await page.goto(PAGE.as_uri())
        await page.get_by_role('button', name='暂停', exact=True).click()
        await page.locator('#time').fill('12')
        await page.locator('#time').dispatch_event('input')
        await page.wait_for_timeout(150)
        assert await page.locator('#graphs-panel').is_visible()
        assert not await page.locator('#guide-panel').is_visible()
        assert not await page.locator('#data-panel').is_visible()
        dimensions = await page.locator('canvas').evaluate_all(
            '(items)=>items.map(x=>({id:x.id,width:x.clientWidth,height:x.clientHeight}))')
        assert all(item['height'] >= 460 for item in dimensions)
        await page.locator('main').screenshot(path=str(OUTPUT / '坐标对照_桌面.png'))

        before = await page.locator('#relative').evaluate('(x)=>x.clientWidth')
        await page.locator('[data-view="relative"]').click()
        await page.wait_for_timeout(150)
        after = await page.locator('#relative').evaluate('(x)=>x.clientWidth')
        assert after > before * 2
        assert not await page.locator('#triangle').is_visible()
        await page.locator('main').screenshot(path=str(OUTPUT / '坐标对照_单图.png'))
        await page.get_by_role('button', name='恢复三图').click()
        assert await page.locator('#triangle').is_visible()

        await page.get_by_role('tab', name='坐标数据').click()
        assert await page.locator('#data-panel').is_visible()
        assert await page.locator('#rows tr').count() == 3
        await page.locator('main').screenshot(path=str(OUTPUT / '坐标对照_数据页.png'))
        await page.get_by_role('tab', name='使用说明').click()
        assert await page.locator('#guide-panel').is_visible()
        assert not await page.locator('#relative').is_visible()
        await page.locator('main').screenshot(path=str(OUTPUT / '坐标对照_说明页.png'))
        await page.get_by_role('tab', name='使用说明').press('ArrowLeft')
        assert await page.locator('#tab-data').get_attribute('aria-selected') == 'true'
        await page.get_by_role('tab', name='图形视图').click()
        await page.locator('#scenario').select_option('training')
        await page.locator('#time').fill('30')
        await page.locator('#time').dispatch_event('input')
        await page.wait_for_timeout(100)

        geometry = await page.evaluate('''()=>{
          const a=coordinateDemo(0,'orbit'),b=coordinateDemo(12,'orbit');
          return {old203moves:Math.hypot(a.triangle[2].x-b.triangle[2].x,a.triangle[2].y-b.triangle[2].y),
                  fixed203moves:Math.hypot(a.relative[2].x-b.relative[2].x,a.relative[2].y-b.relative[2].y),
                  mirror:coordinateDemo(24,'mirror').relative[2].y<0,
                  positiveTriangle:coordinateDemo(24,'mirror').triangle[2].y>=0};
        }''')
        assert geometry['old203moves'] > 1 and geometry['fixed203moves'] == 0
        assert geometry['mirror'] and geometry['positiveTriangle']

        await page.set_viewport_size({'width': 390, 'height': 844})
        await page.locator('#scenario').select_option('orbit')
        await page.locator('#time').fill('12')
        await page.locator('#time').dispatch_event('input')
        await page.wait_for_timeout(100)
        assert await page.evaluate('document.documentElement.scrollWidth<=innerWidth')
        await page.locator('main').screenshot(path=str(OUTPUT / '坐标对照_手机.png'))
        for name in ('坐标数据', '使用说明', '图形视图'):
            await page.get_by_role('tab', name=name).click()
            assert await page.evaluate('document.documentElement.scrollWidth<=innerWidth')

        # 使用本机只读HTTP服务测试输入模式：图形页改版不影响状态读取。
        payload = await page.evaluate("coordinateDemo(12,'orbit')")
        payload.update(live=True, stamp_ns='1000000000', age_s=0.0)
        server = serve_page('127.0.0.1', 0, lambda: payload)
        worker = threading.Thread(target=server.serve_forever)
        worker.start()
        try:
            await page.goto(f'http://127.0.0.1:{server.server_port}')
            await page.wait_for_function("document.getElementById('scenario').disabled")
            await page.get_by_role('tab', name='坐标数据').click()
            assert await page.locator('#rows tr').count() == 3
            assert '变换有效' in await page.locator('#status').inner_text()
        finally:
            server.shutdown()
            server.server_close()
            worker.join()
        assert not errors, errors
        await browser.close()
    result = dict(tabs=True, single_plot_enlargement=True, desktop_canvas=dimensions,
                  mobile_no_horizontal_overflow=True, http_input=True, geometry=geometry,
                  browser_errors=errors)
    (OUTPUT / 'browser_checks.json').write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps(result, ensure_ascii=False))
    print(f'截图：{OUTPUT}')


if __name__ == '__main__':
    asyncio.run(check())
