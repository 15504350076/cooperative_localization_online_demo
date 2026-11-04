#!/usr/bin/env python3
"""从Markdown生成可离线搜索、默认折叠的现场排查手册。"""
from html import escape
from pathlib import Path
import re

ROOT=Path(__file__).resolve().parents[1]
SOURCE=ROOT/'docs/32_三盒协同定位现场调试排查手册.md'
OUTPUT=SOURCE.with_suffix('.html')


def inline(value):
    parts=re.split(r'(`[^`]+`)',value)
    return ''.join('<code>'+escape(p[1:-1])+'</code>' if p.startswith('`') and p.endswith('`') else escape(p) for p in parts)


def build():
    lines=SOURCE.read_text(encoding='utf-8').splitlines()
    out=[];nav=[];sections=cards=0;in_section=in_details=in_list=in_code=False;code=[]
    def close_list():
        nonlocal in_list
        if in_list:out.append('</ul>');in_list=False
    def close_card():
        nonlocal in_details
        close_list()
        if in_details:out.append('</details>');in_details=False
    def close_section():
        nonlocal in_section
        close_card()
        if in_section:out.append('</section>');in_section=False
    for line in lines:
        if line.startswith('```'):
            if in_code:
                out.append('<div class="code-wrap"><button type="button" class="copy" aria-label="复制命令">复制</button><pre><code>'+escape('\n'.join(code))+'</code></pre></div>')
                code=[];in_code=False
            else:close_list();in_code=True
            continue
        if in_code:code.append(line);continue
        if line.startswith('# '):continue
        if line.startswith('## '):
            close_section();sections+=1;title=line[3:];nav.append(f'<a href="#section-{sections}">{inline(title)}</a>')
            out.append(f'<section id="section-{sections}"><h2>{inline(title)}</h2>');in_section=True;continue
        if line.startswith('### '):
            assert in_section;close_card();cards+=1
            out.append(f'<details class="case"><summary><span class="number">{cards:03d}</span><span>{inline(line[4:])}</span></summary><div class="answer">');in_details=True;continue
        if line.startswith('- '):
            if not in_list:out.append('<ul>');in_list=True
            out.append('<li>'+inline(line[2:])+'</li>');continue
        close_list()
        if line:out.append('<p>'+inline(line)+'</p>')
    assert not in_code,'Markdown存在未闭合的代码块'
    close_section()
    assert sections>=10 and cards>=80,(sections,cards)
    body='\n'.join(out).replace('</details>', '</div></details>')
    return TEMPLATE.replace('@@NAV@@','\n'.join(nav)).replace('@@BODY@@',body).replace('@@COUNT@@',str(cards))


TEMPLATE='''<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>三盒协同定位现场调试与排查手册</title>
<style>
:root{color-scheme:light;--ink:#172b3e;--muted:#536879;--line:#dbe4eb;--accent:#145b82;--paper:#fff;--bg:#f2f6f8}
*{box-sizing:border-box}html{scroll-behavior:smooth}body{margin:0;background:var(--bg);color:var(--ink);font:16px/1.72 "Noto Sans CJK SC","Microsoft YaHei","PingFang SC",sans-serif}
header{background:#173a50;color:#fff;padding:28px max(24px,calc((100vw - 1450px)/2));}h1{font-size:clamp(23px,3vw,32px);line-height:1.3;margin:0 0 8px}header p{margin:3px 0;color:#d7e6ee}.layout{max-width:1450px;margin:0 auto;display:grid;grid-template-columns:240px minmax(0,1fr);gap:25px;padding:22px 24px 80px}
aside{align-self:start;position:sticky;top:16px;max-height:calc(100vh - 32px);overflow:auto;background:var(--paper);border:1px solid var(--line);border-radius:10px;padding:16px}aside h2{font-size:15px;margin:0 0 8px}aside a{display:block;font-size:14px;line-height:1.4;color:var(--accent);padding:5px 0;text-decoration:none}aside a:hover{text-decoration:underline}
main{min-width:0}.tools{position:sticky;top:0;z-index:5;background:rgba(242,246,248,.97);padding:0 0 12px;display:flex;gap:8px;flex-wrap:wrap;align-items:center}.tools input{flex:1 1 280px;min-width:0;border:1px solid #91aaba;border-radius:7px;padding:10px 12px;font:inherit;background:white}.tools button,.copy{font:inherit;border:1px solid #9cb0bf;background:white;color:#1a4c67;border-radius:6px;padding:8px 12px;cursor:pointer}.tools button:hover,.copy:hover{background:#e7f2f8}.result{font-size:13px;color:var(--muted);width:100%}
section{scroll-margin-top:90px;margin:0 0 28px}section>h2{font-size:21px;margin:10px 0 10px;border-left:4px solid #4f8dad;padding-left:12px}section>p{color:var(--muted)}details.case{background:var(--paper);border:1px solid var(--line);border-radius:8px;margin:8px 0;overflow:hidden}details.case[open]{border-color:#8bb1c7;box-shadow:0 2px 8px #193e5410}summary{cursor:pointer;list-style:none;display:flex;align-items:baseline;gap:10px;font-weight:600;padding:12px 15px}summary::-webkit-details-marker{display:none}summary:after{content:"＋";color:var(--accent);margin-left:auto}details[open]>summary:after{content:"−"}.number{font:12px/1.4 monospace;color:#7892a1}.answer{border-top:1px solid #edf2f5;padding:3px 19px 14px}.answer ul{margin:10px 0;padding-left:1.4em}.answer li{margin:7px 0}.answer p{margin:10px 0}code{font-family:Consolas,monospace;font-size:.93em}li code,p code{background:#eef4f7;border-radius:3px;padding:1px 4px}.code-wrap{position:relative;margin:10px 0}.code-wrap pre{overflow:auto;border-radius:7px;background:#172b39;color:#edf5f8;padding:16px;margin:0;line-height:1.55;white-space:pre}.copy{position:absolute;right:8px;top:8px;font-size:13px;padding:3px 9px;color:#21455b}.links{font-size:14px;color:var(--muted);margin:12px 0 20px}.links a{color:var(--accent)}[hidden]{display:none!important}footer{color:var(--muted);font-size:13px;margin-top:30px}
@media(max-width:850px){.layout{grid-template-columns:1fr;padding:14px 14px 55px}aside{position:static;max-height:none}aside nav{display:flex;flex-wrap:wrap;gap:3px 14px}header{padding:22px 16px}.tools{top:0}}
@media print{header{background:white;color:black;padding:0}header p{color:black}body{background:white;font-size:11pt}.layout{display:block;padding:0}aside,.tools,.copy{display:none}details.case{border:1px solid #aaa;break-inside:avoid}summary{padding:6px}summary:after{display:none}.answer{padding:0 10px}.code-wrap pre{white-space:pre-wrap;color:black;background:white;border:1px solid #aaa}}
</style></head><body>
<header><h1>三盒协同定位现场调试与排查手册</h1><p>按现象查找；点击问题展开检查步骤。共 @@COUNT@@ 条，离线可用。</p><p>现场启动与设备配置以本轮实际部署和21号文档为准。</p></header>
<div class="layout"><aside><h2>分类导航</h2><nav>@@NAV@@</nav></aside><main>
<div class="tools"><input id="query" type="search" aria-label="搜索现象、话题、命令或文件" placeholder="搜索：例如 时间回退、202、PTP、镜像、无输出"><button type="button" id="expand">展开当前结果</button><button type="button" id="collapse">收起全部</button><div class="result" id="result" role="status" aria-live="polite">默认收起，点击问题查看答案。</div></div>
<div class="links">配套文档：<a href="21_上交盒端传感器节点启动与可用性检查_精简执行版.md">现场启动</a> · <a href="31_离线数据验收与算法回放步骤.md">离线验收</a> · <a href="41_201盒UWB最小在线相对定位部署步骤.md">UWB部署</a> · <a href="32_三盒协同定位现场调试排查手册.md">可编辑源文件</a></div>
@@BODY@@
<footer>检查命令仅在标明的机器执行。页内数据和参数以项目现有文档、源码为依据，现场请核对实际安装版本。</footer>
</main></div>
<script>
const cases=[...document.querySelectorAll('details.case')],sections=[...document.querySelectorAll('main section')],input=document.getElementById('query'),result=document.getElementById('result');
function search(){const q=input.value.trim().toLocaleLowerCase();let shown=0;for(const d of cases){const hit=!q||d.textContent.toLocaleLowerCase().includes(q);d.hidden=!hit;if(hit)shown++}for(const s of sections)s.hidden=![...s.querySelectorAll('details.case')].some(d=>!d.hidden);result.textContent=q?`找到 ${shown} 条，点击问题展开。`:`共 ${cases.length} 条，默认收起，点击问题查看答案。`}
input.addEventListener('input',search);document.getElementById('expand').addEventListener('click',()=>cases.filter(x=>!x.hidden).forEach(x=>x.open=true));document.getElementById('collapse').addEventListener('click',()=>cases.forEach(x=>x.open=false));
document.querySelectorAll('.copy').forEach(button=>button.addEventListener('click',async()=>{const value=button.nextElementSibling.textContent;try{if(navigator.clipboard&&window.isSecureContext)await navigator.clipboard.writeText(value);else{const t=document.createElement('textarea');t.value=value;document.body.appendChild(t);t.select();const ok=document.execCommand('copy');t.remove();if(!ok)throw Error('copy failed')}button.textContent='已复制'}catch(e){button.textContent='请手动复制'}setTimeout(()=>button.textContent='复制',1800)}));
let previous=[];window.addEventListener('beforeprint',()=>{previous=cases.map(d=>d.open);cases.filter(d=>!d.hidden).forEach(d=>d.open=true)});window.addEventListener('afterprint',()=>cases.forEach((d,i)=>d.open=previous[i]||false));
</script></body></html>'''

if __name__=='__main__':
    OUTPUT.write_text(build(),encoding='utf-8')
    print(f'已生成：{OUTPUT}')
