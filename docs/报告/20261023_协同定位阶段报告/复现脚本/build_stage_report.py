#!/usr/bin/env python3
"""由RTE评估结果生成中文阶段报告、原始数据图和可编辑Word；PDF由LibreOffice导出。

依赖现有numpy/matplotlib/Pillow及python-docx；不使用生成式图片或外部模板。
"""
import csv
import hashlib
import json
import os
import platform
from pathlib import Path
import re
import shutil
import subprocess
import zipfile
import xml.etree.ElementTree as ET

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm
from matplotlib.patches import Rectangle, FancyArrowPatch
from matplotlib.transforms import Affine2D
import numpy as np
from PIL import Image

ROOT=Path(__file__).resolve().parents[1]
DATA=ROOT/'data/rte_accuracy_20261023'
DEST=ROOT/'docs/报告/20261023_协同定位阶段报告'
ASSETS=DEST/'figures'
for _font_path, _font_name in [
    ('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc', 'Noto Sans CJK JP'),
    ('/usr/share/fonts/truetype/wqy/wqy-microhei.ttc', 'WenQuanYi Micro Hei'),
    ('/usr/share/fonts/truetype/arphic/uming.ttc', 'AR PL UMing CN'),
]:
    if Path(_font_path).exists():
        fm.fontManager.addfont(_font_path)
        _plot_font = _font_name
        break
else:
    _plot_font = 'DejaVu Sans'
plt.rcParams.update({'font.family':_plot_font,'axes.unicode_minus':False,
                     'font.size':10,'axes.spines.top':False,'axes.spines.right':False,
                     'axes.titlesize':11,'figure.facecolor':'white','savefig.facecolor':'white'})
COLORS=['#24557b','#ad653a','#4f806b','#815c91','#7d7d7d']


def save(fig,name):
    fig.savefig(ASSETS/(name+'.png'),dpi=190,bbox_inches='tight')
    fig.savefig(ASSETS/(name+'.svg'),bbox_inches='tight')
    plt.close(fig)


def box(ax,x,y,w,h,text,color='#eef2f5'):
    ax.add_patch(Rectangle((x,y),w,h,facecolor=color,edgecolor='#59636c',lw=1.1))
    ax.text(x+w/2,y+h/2,text,ha='center',va='center',fontsize=10,linespacing=1.6)


def arrow(ax,a,b,label=None):
    ax.add_patch(FancyArrowPatch(a,b,arrowstyle='-|>',mutation_scale=12,color='#59636c',lw=1.15))
    if label: ax.text((a[0]+b[0])/2,(a[1]+b[1])/2+.12,label,ha='center',fontsize=9)


def architecture():
    """用本地HTML页面生成系统框架截图，确保中文字体由页面明确控制。"""
    html_path = ROOT / 'docs/visualization/system_framework.html'
    html_path.parent.mkdir(parents=True, exist_ok=True)
    html_path.write_text('''<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8"><style>
html,body{margin:0;background:#fff}body{font-family:"Times New Roman","SimHei","Noto Sans CJK SC","Microsoft YaHei",sans-serif}
.canvas{position:relative;width:1600px;height:900px;background:#fff}
.box{position:absolute;box-sizing:border-box;border:3px solid #59636c;background:#eef2f5;color:#111;text-align:center;display:flex;align-items:center;justify-content:center;line-height:1.6;font-size:26px;padding:12px}
.small{font-size:24px}.b1{left:36px;top:74px;width:386px;height:160px}.b2{left:534px;top:74px;width:372px;height:160px}.b3{left:1030px;top:74px;width:534px;height:160px}
.b4{left:36px;top:360px;width:386px;height:170px}.b5{left:534px;top:360px;width:372px;height:170px}.b6{left:1030px;top:627px;width:534px;height:146px}
svg{position:absolute;inset:0;pointer-events:none}.line{stroke:#59636c;stroke-width:3;fill:none;marker-end:url(#arrow)}
</style></head><body><div class="canvas">
<div class="box b1">单车传感器与计算盒 × 3<br>UWB／车载IMU<br>雷达／相机</div>
<div class="box b2">本车运动估计<br>IMU航位推算<br>本车位姿与速度</div>
<div class="box b3">自组网通信<br>传输车间测距和本车位姿</div>
<div class="box b4">仿真与实测数据<br>轨迹、测距、IMU和里程计<br>算法计算与误差分析</div>
<div class="box b5">协同定位计算<br>默认：UWB相对队形<br>扩展：RTE＋局部里程计</div>
<div class="box b6">决策／规划／控制／显示<br>三车位置、坐标系和结果时间</div>
<svg viewBox="0 0 1600 900"><defs><marker id="arrow" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="8" markerHeight="8" orient="auto-start-reverse"><path d="M 0 0 L 10 5 L 0 10 z" fill="#59636c"/></marker></defs>
<path class="line" d="M422 154 H534"/><path class="line" d="M906 154 H1030"/>
<path class="line" d="M1297 234 V360 H906"/><path class="line" d="M422 445 H534"/><path class="line" d="M906 445 L1110 627"/>
</svg></div></body></html>''', encoding='utf-8')
    output = ASSETS / '01_系统与联调边界.png'
    subprocess.run(['firefox', '--headless', '--screenshot', str(output),
                    '--window-size', '1600,900', html_path.as_uri()],
                   check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def scene_plan():
    world=ET.parse(ROOT/'ros2/zju_coop_gazebo/worlds/three_vehicle.sdf').getroot().find('world')
    fig,ax=plt.subplots(figsize=(9,5))
    # 按本仓库SDF的include实例位姿绘制，明确是布置图而非渲染截图。
    specs={'rack':(3.,1.2),'pallet':(1.5,1.2),'conveyor':(6.,1.3),'barrier':(3.,.4),'vehicle':(1.,.65),'shell':(30.,25.)}
    counts={}
    for inc in world.findall('include'):
        name=inc.findtext('name',''); uri=inc.findtext('uri',''); pose=[float(x) for x in inc.findtext('pose','0 0 0 0 0 0').split()]
        label=next((k for k in specs if k in uri),'other')
        if label in ('other','shell'): continue
        counts[label]=counts.get(label,0)+1
        w,h=specs[label]; x,y=pose[:2]
        patch=Rectangle((x-w/2,y-h/2),w,h,transform=Affine2D().rotate_around(x,y,pose[5])+ax.transData,
                        facecolor=COLORS[0] if label=='vehicle' else '#d1d7db',edgecolor='#63717a',alpha=.9)
        ax.add_patch(patch)
        if label=='vehicle': ax.text(x,y+.7,name.replace('vehicle_','车'),ha='center',fontsize=9)
    ax.set(xlim=(-4,28),ylim=(-5,23),xlabel='场景X（米）',ylabel='场景Y（米）',aspect='equal',
           title='已有Gazebo仓库场景布置（按本地SDF实例位姿绘制）')
    ax.grid(alpha=.18)
    ax.text(.99,.02,'示意矩形表示模型占位，尺寸简化；不是实地地图或仿真截图。',transform=ax.transAxes,ha='right',fontsize=8)
    save(fig,'02_已有物理仿真场景')


def hardware():
    fig,axes=plt.subplots(1,3,figsize=(12,3.3))
    for ax,num in zip(axes,[33,42,51]):
        ax.imshow(Image.open(ROOT/f'data/camera_samples_analysis/video{num}_preview.png'))
        ax.set_title(f'201相机通道 video{num}（实拍）'); ax.axis('off')
    fig.tight_layout(); save(fig,'03_三路相机实拍')
    shutil.copy2(ROOT/'data/three_boxes_20260811_012046/livox_visualization/livox_overview.png',ASSETS/'04_三盒实测点云.png')
    shutil.copy2(ROOT/'data/rte_coordinate_validation_20261028/坐标对照_桌面.png',ASSETS/'10_RTE坐标网页展示.png')


def real_plots():
    base=DATA/'uwb_baseline_20261017'
    with (base/'uwb_records.csv').open() as f: ranges=list(csv.DictReader(f))
    with (base/'pose_records.csv').open() as f: poses=list(csv.DictReader(f))
    start=min(int(r['header_ns']) for r in ranges)
    fig,axes=plt.subplots(2,1,figsize=(9,6),sharex=True)
    for e,color in zip([(1,2),(1,3),(2,3)],COLORS):
        rows=[r for r in ranges if (int(r['src_id']),int(r['target_id']))==e]
        axes[0].plot([(int(r['header_ns'])-start)/1e9 for r in rows],[float(r['distance_m']) for r in rows],color=color,lw=.7,label=f'{e[0]}→{e[1]}')
    for coord,color in zip(['x_m','y_m'],COLORS):
        rows=[r for r in poses if r['node_id']=='3' and r['position_valid']=='True']
        axes[1].plot([(int(r['header_ns'])-start)/1e9 for r in rows],[float(r[coord]) for r in rows],color=color,lw=.7,label='车3相对'+coord[0].upper())
    axes[0].set(title='三条测距边持续有数据',ylabel='UWB距离（米）')
    axes[1].set(title='实盒在线相对位置：静止输出仍有抖动',xlabel='消息Header相对起点（秒）',ylabel='相对坐标（米）')
    for ax in axes: ax.grid(alpha=.2); ax.legend(ncol=3,loc='upper right')
    fig.tight_layout(); save(fig,'05_实测在线输入输出')
    fig,ax=plt.subplots(figsize=(7.5,4.5))
    med={}
    for n,color in zip([1,2,3],COLORS):
        rows=[r for r in poses if r['node_id']==str(n) and r['position_valid']=='True']
        x=np.array([float(r['x_m']) for r in rows]); y=np.array([float(r['y_m']) for r in rows])
        ax.scatter(x[::4],y[::4],s=5,alpha=.2,color=color)
        med[n]=(np.median(x),np.median(y)); ax.plot(*med[n],'o',color=color,label=f'车{n}：有效输出中位数')
    for i,j in [(1,2),(1,3),(2,3)]: ax.plot([med[i][0],med[j][0]],[med[i][1],med[j][1]],color='#555',lw=1)
    ax.set(xlabel='规范相对X（米）',ylabel='规范相对Y（米）',title='实盒UWB三角形及静止散布（非外部真值）',aspect='equal')
    ax.legend(loc='upper right',fontsize=9); ax.grid(alpha=.2)
    fig.tight_layout(); save(fig,'06_实测三车相对三角形')


def rte_plots(summary):
    sims=summary['simulation']
    fig,axes=plt.subplots(1,2,figsize=(10,4.2))
    groups=[('odometry',.02),('odometry',.1),('imu_noisy',.02),('imu_noisy',.1)]
    for j,(mode,sigma) in enumerate(groups):
        rows=[r for r in sims if r['mode']==mode and r['range_sigma_m']==sigma]
        for k,r in enumerate(rows):
            x=j+(k-(len(rows)-1)/2)*.10
            axes[0].plot(x,r['position_rmse_m'],'o',color=COLORS[j],ms=6)
            yaw=np.sqrt(np.mean([v['yaw_rmse_deg']**2 for v in r['vehicles'].values()]))
            axes[1].plot(x,yaw,'o',color=COLORS[j],ms=6)
    labels=['仿真无噪声局部运动输入\n测距噪声2厘米','仿真无噪声局部运动输入\n测距噪声10厘米','原始IMU含噪\n测距噪声2厘米','原始IMU含噪\n测距噪声10厘米']
    for ax in axes: ax.set_xticks(range(4),labels,fontsize=8); ax.grid(axis='y',alpha=.25)
    axes[0].set(ylabel='有效输出位置均方根误差（米）',title='全部重复试验：位置误差')
    axes[1].set(ylabel='有效输出航向均方根误差（度）',title='全部重复试验：航向误差')
    axes[0].axhline(.3,color='#777',ls='--',lw=.8,label='拟定动态目标0.3米'); axes[0].legend(fontsize=8)
    fig.tight_layout(); save(fig,'07_RTE全部随机用例精度')
    # 预先固定展示seed0与seed4，不按结果挑选；完整18用例另有表。
    fig,axes=plt.subplots(2,1,figsize=(9,6),sharex=True)
    for seed,color in zip([0,4],COLORS):
        name=f'sim_odometry_0.02_seed{seed}'
        a=np.genfromtxt(DATA/(name+'_output.csv'),delimiter=',',names=True)
        truth=np.genfromtxt(DATA/(name+'.truth.csv'),delimiter=',',names=True)
        good=(a['status']==1)&(a['elapsed_s']<=40)
        t=a['elapsed_s']; stamp=a['state_timestamp_ns']
        error=np.hypot(a['x_3']-np.interp(stamp,truth['timestamp_ns'],truth['x_3']),a['y_3']-np.interp(stamp,truth['timestamp_ns'],truth['y_3']))
        axes[0].plot(t,np.where(good,error,np.nan),color=color,label=f'重复试验{seed + 1}，车3')
        axes[1].plot(t,np.where(good,a['rms_m'],np.nan),color=color,label=f'重复试验{seed + 1}')
    for ax in axes: ax.grid(alpha=.2); ax.legend(fontsize=9)
    axes[0].set(ylabel='位置误差（米）',title='同一轨迹、相同噪声量级：重复试验可能收敛到不同解')
    axes[1].set(ylabel='测距拟合残差（米）',xlabel='合成试验时间（秒）',title='拟合残差小，仍不能替代位置真值检验')
    fig.tight_layout(); save(fig,'08_位置误差与测距残差对照')
    real=next(r for r in summary['real'] if r['dataset']=='joint_time_20261017_151037')
    p=DATA/(real['dataset']+'_output.csv')
    if p.exists():
        a=np.genfromtxt(p,delimiter=',',names=True)
        fig,axes=plt.subplots(2,1,figsize=(9,5.8),sharex=True)
        for n,color in zip([1,2,3],COLORS): axes[0].plot(a['elapsed_s'],np.hypot(a[f'local_x_{n}'],a[f'local_y_{n}']),color=color,label=f'盒{200+n}')
        axes[0].set(ylabel='本地水平位移（米）',title='静止实测IMU：仅启动标定后的惯性积分仍会积累漂移')
        axes[1].step(a['elapsed_s'],a['status'],where='post',color=COLORS[0])
        axes[1].set(xlabel='原始消息Header相对起点（秒）',ylabel='RTE状态码',title='原始时间回放状态（状态码含义见正文）')
        for ax in axes: ax.grid(alpha=.2)
        axes[0].legend(ncol=3); fig.tight_layout(); save(fig,'09_静止实测惯导漂移与RTE状态')


def table(headers,rows):
    return '| '+' | '.join(headers)+' |\n|'+'|'.join(['---']*len(headers))+'|\n'+''.join('| '+' | '.join(map(str,row))+' |\n' for row in rows)


def num(value,digits=3):
    return '无有效解' if value is None else f'{value:.{digits}f}'


def render_word(markdown,path,*,title=None,header='浙江大学项目组  |  多平台协同定位阶段报告'):
    from docx import Document
    from docx.shared import Cm,Pt,RGBColor
    from docx.oxml import OxmlElement
    from docx.oxml.ns import qn
    from docx.enum.text import WD_ALIGN_PARAGRAPH
    from docx.enum.table import WD_CELL_VERTICAL_ALIGNMENT
    from docx.enum.section import WD_SECTION_START
    doc=Document(); sec=doc.sections[0]
    sec.page_width=Cm(21); sec.page_height=Cm(29.7)
    # 按正式中文报告的版心设置；中文使用宋体/黑体，英文和数字使用Times New Roman。
    sec.top_margin=Cm(3.7); sec.bottom_margin=Cm(3.5); sec.left_margin=Cm(2.8); sec.right_margin=Cm(2.6)
    sec.header_distance=Cm(1.5); sec.footer_distance=Cm(1.75)
    sec.different_first_page_header_footer=False
    def set_font(style,name,size=None,bold=None,color='000000'):
        style.font.name='Times New Roman'
        style._element.get_or_add_rPr().rFonts.set(qn('w:ascii'),'Times New Roman')
        style._element.get_or_add_rPr().rFonts.set(qn('w:hAnsi'),'Times New Roman')
        style._element.get_or_add_rPr().rFonts.set(qn('w:eastAsia'),name)
        style._element.get_or_add_rPr().rFonts.set(qn('w:cs'),'Times New Roman')
        if size is not None: style.font.size=Pt(size)
        if bold is not None: style.font.bold=bold
        style.font.color.rgb=RGBColor.from_string(color)
    for style in doc.styles:
        if hasattr(style,'font'): set_font(style,'宋体',color='000000')
    set_font(doc.styles['Normal'],'宋体',12)
    normal=doc.styles['Normal']; normal.paragraph_format.line_spacing=1.5; normal.paragraph_format.space_after=Pt(0)
    normal.paragraph_format.alignment=WD_ALIGN_PARAGRAPH.JUSTIFY
    set_font(doc.styles['Title'],'黑体',22,True)
    set_font(doc.styles['Subtitle'],'宋体',16,False)
    set_font(doc.styles['Heading 1'],'黑体',16,True)
    set_font(doc.styles['Heading 2'],'黑体',14,True)
    set_font(doc.styles['Heading 3'],'黑体',12,True)
    set_font(doc.styles['Caption'],'黑体',10.5,False)
    doc.styles['Caption'].paragraph_format.first_line_indent=Pt(0)
    doc.styles['Caption'].paragraph_format.space_before=Pt(6)
    doc.styles['Caption'].paragraph_format.space_after=Pt(12)
    set_font(doc.styles['Footer'],'宋体',10.5,False)
    for name,size in [('Heading 1',16),('Heading 2',14),('Heading 3',12)]:
        style=doc.styles[name]; style.paragraph_format.space_before=Pt(14); style.paragraph_format.space_after=Pt(6)
        style.paragraph_format.keep_with_next=True
    # 清除默认 Title 样式自带的蓝色下划线。
    for style_name in ['Title','Heading 1','Heading 2','Heading 3']:
        ppr=doc.styles[style_name]._element.get_or_add_pPr()
        for child in list(ppr):
            if child.tag==qn('w:pBdr'): ppr.remove(child)
    hp=sec.header.paragraphs[0]; hp.text=header; hp.style=doc.styles['Caption']; hp.alignment=WD_ALIGN_PARAGRAPH.CENTER
    sec.footer.paragraphs[0].text=''
    sec.first_page_header.paragraphs[0].text=''
    sec.first_page_footer.paragraphs[0].text=''
    lines=markdown.splitlines(); i=0; cover=True; pending_page_break=False; body_started=False
    def clean(s):
        return re.sub(r'\[([^\]]+)\]\([^)]+\)',r'\1',s).replace('**','').replace('`','')

    def add_hyperlink(paragraph, text, anchor):
        """在Word中写入指向本报告书签的内部超链接。"""
        hyperlink = OxmlElement('w:hyperlink')
        hyperlink.set(qn('w:anchor'), anchor.lstrip('#'))
        hyperlink.set(qn('w:history'), '1')
        run = OxmlElement('w:r')
        rpr = OxmlElement('w:rPr')
        rfonts = OxmlElement('w:rFonts')
        rfonts.set(qn('w:ascii'), 'Times New Roman')
        rfonts.set(qn('w:hAnsi'), 'Times New Roman')
        rfonts.set(qn('w:cs'), 'Times New Roman')
        rfonts.set(qn('w:eastAsia'), '宋体')
        rpr.append(rfonts)
        color = OxmlElement('w:color'); color.set(qn('w:val'), '0563C1'); rpr.append(color)
        underline = OxmlElement('w:u'); underline.set(qn('w:val'), 'single'); rpr.append(underline)
        size = OxmlElement('w:sz'); size.set(qn('w:val'), '21'); rpr.append(size)
        run.append(rpr)
        node = OxmlElement('w:t'); node.text = text; run.append(node)
        hyperlink.append(run)
        paragraph._p.append(hyperlink)

    def add_inline(paragraph, raw):
        """保留目录Markdown链接，并将其转换为Word内部超链接。"""
        pattern = re.compile(r'\[([^\]]+)\]\((#[^)]+)\)')
        pos = 0
        for match in pattern.finditer(raw):
            if match.start() > pos:
                paragraph.add_run(raw[pos:match.start()].replace('**', '').replace('`', ''))
            add_hyperlink(paragraph, match.group(1), match.group(2))
            pos = match.end()
        if pos < len(raw):
            paragraph.add_run(raw[pos:].replace('**', '').replace('`', ''))

    def add_bookmark(paragraph, anchor, number):
        start = OxmlElement('w:bookmarkStart')
        start.set(qn('w:id'), str(number)); start.set(qn('w:name'), anchor)
        end = OxmlElement('w:bookmarkEnd'); end.set(qn('w:id'), str(number))
        paragraph._p.insert(1 if paragraph._p.pPr is not None else 0, start)
        paragraph._p.append(end)

    def heading_anchor(text, level):
        text = clean(text)
        if level == 1:
            if text.startswith('附录'):
                return 'appendix'
            match = re.match(r'^(\d+)\s', text)
            if match:
                return f'chapter_{match.group(1)}'
            if text == '目录':
                return 'toc'
        if level == 2:
            match = re.match(r'^(\d+\.\d+)\s', text)
            if match:
                return 'section_' + match.group(1).replace('.', '_')
        return None

    bookmark_id = 1

    def set_cell_center(cell):
        cell.vertical_alignment = WD_CELL_VERTICAL_ALIGNMENT.CENTER
        for paragraph in cell.paragraphs:
            paragraph.alignment = WD_ALIGN_PARAGRAPH.CENTER
            paragraph.paragraph_format.first_line_indent = Cm(0)

    def set_table_borders(table):
        """给同一表格的外框、内框设置相同线宽，避免边框粗细不一。"""
        edges = ('top', 'left', 'bottom', 'right', 'insideH', 'insideV')
        tbl_pr = table._tbl.tblPr
        tbl_borders = tbl_pr.first_child_found_in('w:tblBorders')
        if tbl_borders is None:
            tbl_borders = OxmlElement('w:tblBorders')
            tbl_pr.append(tbl_borders)
        for child in list(tbl_borders):
            tbl_borders.remove(child)
        for edge in edges:
            element = OxmlElement(f'w:{edge}')
            element.set(qn('w:val'), 'single')
            element.set(qn('w:sz'), '6')
            element.set(qn('w:space'), '0')
            element.set(qn('w:color'), '808080')
            tbl_borders.append(element)
        for row in table.rows:
            for cell in row.cells:
                tc_pr = cell._tc.get_or_add_tcPr()
                tc_borders = tc_pr.first_child_found_in('w:tcBorders')
                if tc_borders is None:
                    tc_borders = OxmlElement('w:tcBorders')
                    tc_pr.append(tc_borders)
                for child in list(tc_borders):
                    tc_borders.remove(child)
                for edge in ('top', 'left', 'bottom', 'right'):
                    element = OxmlElement(f'w:{edge}')
                    element.set(qn('w:val'), 'single')
                    element.set(qn('w:sz'), '6')
                    element.set(qn('w:space'), '0')
                    element.set(qn('w:color'), '808080')
                    tc_borders.append(element)

    def single_line(value):
        value = clean(value).strip()
        return '\n' not in value and len(value) <= 18

    def page_boundary(paragraph):
        nonlocal pending_page_break
        if pending_page_break:
            paragraph.paragraph_format.page_break_before = True
            pending_page_break = False
        return paragraph

    while i<len(lines):
        line=lines[i].strip()
        if not line: i+=1; continue
        if line=='<!-- pagebreak -->': pending_page_break=True; cover=False; i+=1; continue
        if line.startswith('# '):
            cover_title = clean(line[2:])
            if cover_title == '多平台协同定位硬件仿真测试报告与联调方案':
                cover_title = '多平台协同定位\n硬件仿真测试报告与联调方案'
            p=page_boundary(doc.add_paragraph(cover_title,'Title')); p.alignment=WD_ALIGN_PARAGRAPH.CENTER
            p.paragraph_format.space_before=Pt(160); p.paragraph_format.space_after=Pt(0)
            p.paragraph_format.line_spacing=1.5
            i+=1; continue
        if line.startswith('## '):
            text = line[3:]
            if re.match(r'^1\s', text) and not body_started:
                body_section = doc.add_section(WD_SECTION_START.NEW_PAGE)
                body_section.different_first_page_header_footer = False
                body_section.header.is_linked_to_previous = False
                body_section.footer.is_linked_to_previous = False
                body_section.header.paragraphs[0].text = header
                page_number = OxmlElement('w:pgNumType')
                page_number.set(qn('w:start'), '1')
                body_section._sectPr.insert_element_before(page_number, 'w:cols')
                fp = body_section.footer.paragraphs[0]
                fp.text = ''
                fp.style = doc.styles['Footer']
                fp.alignment = WD_ALIGN_PARAGRAPH.CENTER
                fp.paragraph_format.first_line_indent = Pt(0)
                field = OxmlElement('w:fldSimple'); field.set(qn('w:instr'), 'PAGE')
                run = OxmlElement('w:r'); rpr = OxmlElement('w:rPr')
                font = OxmlElement('w:rFonts'); font.set(qn('w:ascii'), 'Times New Roman'); font.set(qn('w:hAnsi'), 'Times New Roman')
                rpr.append(font); size = OxmlElement('w:sz'); size.set(qn('w:val'), '21'); rpr.append(size)
                run.append(rpr); value = OxmlElement('w:t'); value.text = '1'; run.append(value); field.append(run)
                fp._p.append(field)
                body_started = True
                pending_page_break = False
            paragraph = page_boundary(doc.add_heading(clean(text), 1))
            anchor = heading_anchor(text, 1)
            if anchor:
                add_bookmark(paragraph, anchor, bookmark_id); bookmark_id += 1
            i += 1; continue
        if line.startswith('### '):
            text = line[4:]
            paragraph = page_boundary(doc.add_heading(clean(text), 2))
            anchor = heading_anchor(text, 2)
            if anchor:
                add_bookmark(paragraph, anchor, bookmark_id); bookmark_id += 1
            i += 1; continue
        if re.match(r'^表\d+[\s　]+', line):
            p = page_boundary(doc.add_paragraph(line, 'Caption'))
            p.alignment = WD_ALIGN_PARAGRAPH.CENTER
            p.paragraph_format.keep_with_next = True
            p.paragraph_format.space_before = Pt(4)
            p.paragraph_format.space_after = Pt(6)
            for run in p.runs:
                run.font.name = 'Times New Roman'
                run._element.get_or_add_rPr().rFonts.set(qn('w:eastAsia'), '黑体')
                run.font.bold = True
                run.font.size = Pt(10.5)
            i += 1; continue
        if line.startswith('!['):
            m=re.match(r'!\[(.*?)\]\((.*?)\)',line)
            picture=DEST/m.group(2)
            im=Image.open(picture); width=min(16.7,9.3*im.width/im.height)
            if picture.name in ('15_纯UWB逐车定位误差.png', '16_RTE逐车定位误差.png'):
                width=16.0
            if picture.name=='04_三盒实测点云.png':
                doc.add_page_break(); width=15.7
            p=page_boundary(doc.add_paragraph()); p.alignment=WD_ALIGN_PARAGRAPH.CENTER
            p.paragraph_format.keep_with_next=True
            p.add_run().add_picture(str(picture),width=Cm(width))
            p=doc.add_paragraph(m.group(1),'Caption'); p.alignment=WD_ALIGN_PARAGRAPH.CENTER
            i+=1; continue
        if line.startswith('|'):
            rows=[]
            while i<len(lines) and lines[i].strip().startswith('|'):
                cells=[c.strip() for c in lines[i].strip().strip('|').split('|')]
                if not all(re.fullmatch(r':?-+:?',c) for c in cells): rows.append(cells)
                i+=1
            t=doc.add_table(rows=0,cols=len(rows[0])); t.style='Table Grid'
            for n,row in enumerate(rows):
                cells=t.add_row().cells
                for cell,value in zip(cells,row):
                    cell.text=''
                    page_boundary(cell.paragraphs[0])
                    # 表格所有单元格统一垂直居中；水平方向在表格完成后
                    # 按整列或整行是否为短内容决定是否居中。
                    cell.vertical_alignment = WD_CELL_VERTICAL_ALIGNMENT.CENTER
                    add_inline(cell.paragraphs[0], value)
                    for p in cell.paragraphs:
                        p.paragraph_format.line_spacing=1.15
                        p.paragraph_format.space_after=Pt(0)
                        for run in p.runs:
                            run.font.name='Times New Roman'; run._element.get_or_add_rPr().rFonts.set(qn('w:ascii'),'Times New Roman'); run._element.get_or_add_rPr().rFonts.set(qn('w:hAnsi'),'Times New Roman'); run._element.get_or_add_rPr().rFonts.set(qn('w:cs'),'Times New Roman'); run._element.get_or_add_rPr().rFonts.set(qn('w:eastAsia'),'宋体'); run.font.size=Pt(10.5)
                            run.font.bold=(n==0)
                trpr=t.rows[-1]._tr.get_or_add_trPr(); trpr.append(OxmlElement('w:cantSplit'))
                if n==0:
                    trpr.append(OxmlElement('w:tblHeader'))
            # 表格中一整列或一整行的文字均可在一行显示时，统一水平居中。
            for col in range(len(t.columns)):
                values = [rows[row][col] for row in range(len(rows))]
                if all(single_line(value) for value in values):
                    for row in t.rows: set_cell_center(row.cells[col])
            for row_index, row in enumerate(t.rows):
                if all(single_line(value) for value in rows[row_index]):
                    for cell in row.cells: set_cell_center(cell)
            # 版本记录给修改说明保留主要列宽，避免五号字下日期、版本列挤占正文空间。
            if rows[0] == ['版本', '日期', '文件后缀说明', '本次改动']:
                t.autofit = False
                for column, width in zip(t.columns, (1.25, 2.25, 2.35, 9.75)):
                    column.width = Cm(width)
                    for cell in column.cells:
                        cell.width = Cm(width)
            set_table_borders(t)
            # 分页标记紧跟表格时不再插入空段落，避免排版器把空段落推到新页后
            # 再执行分页，形成只有页码的空白页。
            next_content = i
            while next_content < len(lines) and not lines[next_content].strip():
                next_content += 1
            if next_content >= len(lines) or lines[next_content].strip() != '<!-- pagebreak -->':
                doc.add_paragraph()
            continue
        if line.startswith('```'):
            code=[]; i+=1
            while i<len(lines) and not lines[i].startswith('```'): code.append(lines[i]); i+=1
            p=page_boundary(doc.add_paragraph('\n'.join(code))); p.paragraph_format.line_spacing=1.05; p.alignment=WD_ALIGN_PARAGRAPH.LEFT
            p.paragraph_format.first_line_indent=Pt(0)
            for r in p.runs:
                r.font.name='Times New Roman'; r._element.get_or_add_rPr().rFonts.set(qn('w:ascii'),'Times New Roman'); r._element.get_or_add_rPr().rFonts.set(qn('w:hAnsi'),'Times New Roman'); r._element.get_or_add_rPr().rFonts.set(qn('w:cs'),'Times New Roman'); r._element.get_or_add_rPr().rFonts.set(qn('w:eastAsia'),'宋体'); r.font.size=Pt(10.5)
            i+=1; continue
        p=page_boundary(doc.add_paragraph(style='List Bullet' if line.startswith('- ') else None))
        add_inline(p, line[2:] if line.startswith('- ') else line)
        if not line.startswith('- '):
            p.paragraph_format.first_line_indent=Pt(24)
            p._p.get_or_add_pPr().get_or_add_ind().set(qn('w:firstLineChars'), '200')
            p.alignment=WD_ALIGN_PARAGRAPH.JUSTIFY
        else:
            p.alignment=WD_ALIGN_PARAGRAPH.LEFT
        if cover and len(line)<60:
            p=doc.paragraphs[-1]; p.alignment=WD_ALIGN_PARAGRAPH.CENTER; p.paragraph_format.first_line_indent=Cm(0)
            p._p.pPr.ind.attrib.pop(qn('w:firstLineChars'), None)
            p.paragraph_format.space_before=Pt(371)
            p.paragraph_format.space_after=Pt(0)
            for run in p.runs: run.font.size=Pt(16)
        i+=1
    doc.core_properties.title=title or '多平台协同定位硬件接入与仿真测试阶段报告及联调验证方案'
    # 统一字体：中文正文宋体、中文标题黑体；英文和数字统一Times New Roman。
    all_paragraphs = list(doc.paragraphs)+[p for t in doc.tables for row in t.rows for c in row.cells for p in c.paragraphs]
    for section in doc.sections:
        all_paragraphs.extend(section.header.paragraphs)
        all_paragraphs.extend(section.footer.paragraphs)
    for p in all_paragraphs:
        heading = p.style.name in {'Title','Heading 1','Heading 2','Heading 3','Caption'}
        family = '黑体' if heading else '宋体'
        for run in p.runs:
            run.font.name='Times New Roman'
            rfonts = run._element.get_or_add_rPr().rFonts
            rfonts.set(qn('w:ascii'),'Times New Roman')
            rfonts.set(qn('w:hAnsi'),'Times New Roman')
            rfonts.set(qn('w:cs'),'Times New Roman')
            rfonts.set(qn('w:eastAsia'),family)
    doc.core_properties.author='浙江大学'
    doc.save(path)
    # python-docx模板含有未使用的Courier样式；清理后确保交付文件不残留该字体。
    cleaned=Path(str(path)+'.fontclean')
    with zipfile.ZipFile(path,'r') as source, zipfile.ZipFile(cleaned,'w',zipfile.ZIP_DEFLATED) as target:
        for item in source.infolist():
            data=source.read(item.filename)
            if item.filename.startswith('word/') and item.filename.endswith('.xml'):
                data=data.replace(b'Courier', '宋体'.encode('utf-8'))
            target.writestr(item,data)
    os.replace(cleaned,path)


def main():
    DEST.mkdir(parents=True,exist_ok=True); ASSETS.mkdir(exist_ok=True)
    summary=json.loads((DATA/'summary.json').read_text())
    assert len(summary['simulation'])==18 and len(summary['real'])==6,'请先完成全部评估'
    architecture(); scene_plan(); hardware(); real_plots(); rte_plots(summary)
    write_reports(summary)
    from build_formal_report import build_formal_report
    build_formal_report()


def write_reports(summary):
    sims=summary['simulation']; real=summary['real']
    baselines={d:json.loads((DATA/f'uwb_baseline_{d}/analysis.json').read_text()) for d in ['20261016','20261017']}
    online=[]
    for date,s in baselines.items():
        p=s['poses']; h=p['header_time']
        online.append([date,s['topics']['/uwb/range'],s['topics']['/cooperative_localization/poses_2d'],
                       f'{(h["count"]-1)/h["duration_s"]:.3f}',
                       f'{p["position_valid_frame_count"]}/{h["count"]}',f'{100*p["position_valid_ratio"]:.2f}%',
                       p['yaw_valid_frame_count']])
    grouped=[]
    for mode,sigma in [('odometry',.02),('odometry',.1),('imu_noisy',.02),('imu_noisy',.1)]:
        rows=[s for s in sims if s['mode']==mode and s['range_sigma_m']==sigma]
        errors=[r['position_rmse_m'] for r in rows]
        yaws=[np.sqrt(np.mean([v['yaw_rmse_deg']**2 for v in r['vehicles'].values()])) for r in rows]
        success=sum(r['position_rmse_m']<=.3 for r in rows)
        grouped.append(['仿真无噪声局部运动输入' if mode=='odometry' else '仿真含噪声IMU输入',f'{sigma:.2f}',len(rows),
                        f'{min(errors):.3f}～{max(errors):.3f}',f'{min(yaws):.2f}～{max(yaws):.2f}',f'{success}/{len(rows)}'])
    realrows=[]
    for r in real:
        ins=sum(h['count'] for k,h in r['streams'].items() if k.startswith('IMU_'))
        uwb=sum(h['count'] for k,h in r['streams'].items() if k.startswith('UWB_'))
        replay=r.get('replay')
        state='拒绝配对：'+ '；'.join(r['blockers']) if replay is None else f'可用状态{replay["ready_rows"]}/{replay["evaluated_rows"]}；修正{replay["final_corrections"]}次'
        realrows.append([r['dataset'],ins,uwb,f'{r["common_header_duration_s"]:.2f}',state])
    latest=next(r for r in real if r['dataset']=='joint_time_20261017_151037')
    latest_replay=latest.get('replay',{})
    drift='、'.join(f'盒{201+i}：{v:.2f}米' for i,v in enumerate(latest_replay.get('local_final_displacement_m',[])))
    position_std=baselines['20261017']['poses']['vehicles']['3']
    sigma_x=position_std['x_m']['std']; sigma_y=position_std['y_m']['std']
    counts=sum(sum(h['count'] for k,h in r['streams'].items() if k.startswith('IMU_')) for r in real)
    minimum=max(r['position_rmse_m'] for r in sims if r['mode']=='imu_ideal')
    zero=next(r for r in sims if r['mode']=='odometry' and r['range_sigma_m']==0)
    sim_table=table(['输入方式','UWB噪声标准差（米）','试验次数','位置均方根误差范围（米）','航向均方根误差范围（度）','位置均方根误差≤0.3米次数'],grouped)
    online_table=table(['批次编号','UWB消息数','位置输出帧数','Header输出频率（赫兹）','三车位置均有效帧','有效帧占比','航向有效帧'],online)
    real_table=table(['数据批次','车载IMU总帧数','201共享UWB帧数','Header共同区间（秒）','本次处理结果'],realrows)
    report=f'''# 多平台协同定位硬件接入与仿真测试阶段报告

联调验证方案与后续能力建设

浙江大学项目组

2026年10月23日　｜　阶段技术沟通版 V1.0

本报告以三节点协同定位为当前验证基线，面向近岸复杂地形、多平台协同机动等应用的系统联调需求，说明已形成的硬件与软件能力、可复现证据和后续验收路径。文中“我方”统一指浙江大学项目组；通信、决策、规划与控制等专业模块按合作接口衔接。

<!-- pagebreak -->

## 1. 阶段结论与本次报告的依据

当前已经形成可在实盒运行的UWB相对定位基线，以及从传感器采集、联合录包、离线分析到结果发布的完整工具链。201盒能够接收三盒间测距并持续输出三车相对位置。最新两组含在线输出的记录分别得到1660帧和1692帧位置，输出频率约9.24和9.42赫兹。这些成果说明项目已具备开展定位结果显示、消息接口与异常状态联调的基础。

为提高动态定位能力，项目已形成独立RTE算法库和三车ROS 2节点。它将各车本地里程计与车间UWB距离组合起来，估计各车局部坐标系之间的相对变换，并在滑动窗口内持续修正。本次增加了18组有真值的合成试验和6组静止实测数据审查。试验确认接口及计算链路可以运行，同时发现当前默认RTE参数和局部求解策略存在错误分支收敛风险，不能将其现阶段表述为动态精度已经达标。

现有实测均为静止采集，未提供RTK、动捕、全站仪或人工测量的相对位置真值。因此，报告把“实盒在线可运行”“静态稳定性”“有真值的仿真精度”分开陈述，不把测距残差、输出抖动或单元测试通过当成实测定位精度。定位部分可以先以已运行的基线接口与其他合作方对接；动态性能在同一接口约定下按阶段升级。

| 层次 | 当前结论 | 对系统联调的价值 |
|---|---|---|
| 硬件与数据 | 三计算盒、车载IMU、UWB测距、三盒雷达数据已获得；视觉已有201实拍及202/203安装反馈 | 可以开展真实输入、通信承载与记录回放联调 |
| 在线基础定位 | 201实盒UWB相对三角形持续输出，最新批次位置有效帧1691/1692 | 可以对接三车位置显示、队形几何与失效提示 |
| 本地惯导与RTE | 本地IMU传播、独立RTE库及三车节点具备；已开展精度评估 | 为动态航向、坐标对齐和连续定位提供升级路径 |
| 定量动态精度 | 现有实测不具备真值与运动激励；新增仿真暴露初始化可靠性问题 | 明确下一阶段需要解决和验收的具体问题 |
| 整体任务闭环 | 定位接口与仿真载体已形成，跨专业正式联调尚待开展 | 可按接口、时效、场景逐级验证，避免一次性集成风险 |

## 2. 为什么采用分层验证与渐进联调

多平台协同任务需要的是“带有明确时间、参考坐标和有效性的信息”，而不是一组孤立坐标。通信模块负责把必要信息可靠送达，定位模块给出车辆关系及可信状态，决策与规划模块据此形成可执行目标，控制模块完成运动并反馈执行情况。某一模块出错时，其余模块必须能识别数据失效，不能继续使用看似连续的旧结果。

面向室内外过渡、遮挡和近岸复杂环境，多源传感器各有作用：UWB提供车间距离但缺少方向信息，IMU提供姿态和短时运动连续性但位置会漂移，RTE利用运动过程中的里程计与距离序列对齐各车局部坐标系。雷达和视觉后续可提供额外运动约束，但必须经过时间、外参和里程计质量验收，不能仅凭“已有图像、点云”就推断融合精度已经提升。这也是先测量、再状态、再协同、最后闭环的技术依据。

本报告所述阶段工作包括硬件实测接入和软件仿真验证，两者已有独立证据；“真实计算盒接收仿真传感器流、经实际自组网交换状态、再反馈仿真运动”的完整硬件在环闭环尚未完成联合验收。下一阶段在接口冻结后组织该项试验，把算法、硬件算力和真实通信链路放进同一条可测量链路。

因此我方采用两条并行路线：用UWB相对三角形维持当前可运行基线，尽早验证接口与系统协作；用本地惯导和RTE推进动态能力，经过有真值试验后再进入实车控制链。两条路线使用独立启动入口和话题，新扩展默认关闭，可以单独验证、停止与回退，不改变现有已跑通程序。

![图1　当前定位链路与合作模块的边界。雷达、视觉是已接入或待验收的数据入口，尚未默认融合进主定位结果。](figures/01_系统与联调边界.png)

需要特别说明：UWB三条距离能恢复三角形，但不能确定地图北向或车辆车头朝向。当前UWB基线的参考车为车1，车2位于正X轴，车3选定规范半平面；该坐标用于表达相对几何。规划、控制若需要地图坐标或车体航向，必须另行完成坐标变换和航向验收，不能把三角形边方向当作车头方向。

## 3. 硬件接入与现场工作已经形成的基础

### 3.1 三盒网络和传感器职责

调试电脑通过网线连接201，201通过自组网连接202与203。201承担当前在线融合和三盒录包控制；每盒在本机采集车载IMU、Livox点云和Livox内置IMU。高带宽原始点云、图像不必持续汇聚到参考车，后续主要通过网络交换紧凑本地状态和必要测距，从而为通信、规划与控制保留带宽。

| 对象 | 已有证据与当前状态 | 下一项现场验收 |
|---|---|---|
| UWB | 三条无向边可形成相对三角形；最新批次六个有向流均有消息 | 已知间距、不同距离及遮挡条件下的误差和稳定性 |
| 车载IMU /imu/raw | 三盒约1000赫兹采集；已有轴系转换、启动静止标定和本地惯导传播 | 动态时间一致性、安装外参、漂移及必要辅助约束 |
| Livox /livox/lidar | 已得到三盒点云包和空间可视化 | 恢复PTP后分别检查点云Header与场景运动一致性 |
| Livox /livox/imu | 与点云独立统计、独立检查Header | 本机点云与内置IMU的时间关系及轴系 |
| 三路视觉 | 201已有三路真实图像；协作方反馈202/203已完成安装 | 三盒连续ROS 2 Image/CameraInfo、时间源、帧率及内外参 |
| GNSS | 已有驱动与接口基础；室外有效定位仍需专门验收 | 固定解状态、时间、杆臂及作为独立真值的适用性 |

![图2　201三路相机实拍预览，来自现有采集文件；不是合成效果图，也不作为三目同步或视觉里程计精度证据。](figures/03_三路相机实拍.png)

![图3　三盒Livox实测点云投影，来自three_boxes_20260811_012046。各盒分别处于自身雷达坐标系，未拼接为共同地图；零点和非有限点按原工具过滤。](figures/04_三盒实测点云.png)

### 3.2 时间与数据质量控制

传感器配对统一依据各消息自身的header.stamp。录包接收时间仅用于辅助诊断，本次RTE评估没有用它替换消息Header，也没有通过平移各流时间“做出同步”。时间范围重叠、静态曲线相近和服务显示active，都不能单独证明硬件同步精度。

现场已将UWB/GNSS授时建立后的PTP恢复步骤固化进启动检查：三盒分别确认本机ptp4l及phc2sys，再分别检查/livox/lidar和/livox/imu。视觉后续也沿用相同原则：先明确驱动的采样时间来源，再做动态共同事件测试。通过这些步骤，系统联调得到的是可追溯的数据，而不是只有话题名的接口。

## 4. 实盒在线结果：已经验证了什么

### 4.1 两组含在线输出的实测记录

以下两批记录均包含UWB输入和201实时位置输出，本次对原始bag重新读取统计。批次以文件编号索引；不把文件名中的日期自动等同于传感器Header日期。

{online_table}

频率按位置消息Header跨度计算；“有效帧占比”指已记录位置输出中三车position_valid同时为真的比例，不是全部UWB输入的利用率，更不是精度达标率。航向有效帧为0符合当前UWB基线的预期，不能将该路径用于未经验证的航向控制。

![图4　最新实盒在线输入与输出，横轴仅使用消息Header。静止条件下的坐标变化反映稳定性，尚不能计算绝对定位误差。](figures/05_实测在线输入输出.png)

![图5　实盒输出的规范相对三角形及静止散布。连线使用有效输出坐标的中位数，散点不是外部真值误差。](figures/06_实测三车相对三角形.png)

最新批次车3有效位置的X、Y样本标准差分别约{sigma_x:.3f}米和{sigma_y:.3f}米。这给出了现有静止条件下的输出抖动量级，支持下一步开展已知间距、多种布置及动态试验；它不等于绝对定位误差，也不覆盖中远距离、遮挡、多径或真实近岸环境。

### 4.2 静止数据对RTE能检验到哪一步

本次审查6组联合数据，车载IMU总计{counts:,}帧；原始数据库只读。存在Header回退或各流无足够共同时间区间的批次直接被门禁拦截，不通过事后改时标强行配对。具备基本时间条件的批次送入现有公开SDK，执行10秒静止初始化、本地IMU传播及RTE滑窗求解。

静止数据没有可用的相对位置真值，也缺少估计未知相对航向所需的运动激励。若算法拒绝初始化，应保留为退化识别结果；若出现可用状态，也不能因此证明姿态与位置正确，必须排除惯导漂移被误当成运动信息的情况。完整逐批结果另见配套《RTE精度验证报告》。

![图6　最新静止数据的离线本地惯导传播与RTE状态。状态0为尚无估计，1为内部质量检查通过，2为运动/信息不足，3为未收敛，4为歧义，6为输入过期，7为残差拒绝。](figures/09_静止实测惯导漂移与RTE状态.png)

最新批次在不增加持续零速约束、视觉或雷达里程计的情况下，本地积分末端水平位移为：{drift}。这是静止条件下的漂移诊断量，不是车辆真实运动。后续动态RTE需要质量可控的本地里程计，不能仅凭IMU频率高就推断长时间位置可靠。

## 5. 仿真评估：算法链路可运行，动态精度仍需收敛

### 5.1 已有物理仿真载体与本次定量试验的区别

工程已有Gazebo三车场景、IMU及测距消息桥接、真值话题、定位输出、数据录制，以及可选的雷达、图像与前端接口。场景中的路径运动和安全处理用于验证软件链路，尚不等同于合作方最终规划控制算法。当前场景为仓库环境，近岸地形、水陆交界、湿滑地面与无线传播模型仍属于后续扩展。

![图7　已有Gazebo场景的参数布置图，按仓库内SDF实例位姿绘制；用于说明已有仿真载体，不是近岸任务完成截图。](figures/02_已有物理仿真场景.png)

本轮另外复测了Gazebo资产与物理链路：资产检查通过；默认启动未在验收窗口内形成有效三车定位输出并超时。追加显式运行诊断后可见时钟及部分链路消息，但仍未满足有效输出断言，已有历史运行记录不能替代本次通过结论。对应日志已保留，后续应恢复仿真启动、时钟与结果有效性链路后重跑端到端验收。因此下文的定量误差来自独立的SDK合成试验，不宣称为Gazebo本轮精度或实车精度。

### 5.2 本次RTE定量试验方法

固定三车局部坐标原点与未知相对航向，生成40秒平面轨迹，前12秒静止，之后三车以不同轨迹运动；算法只接收局部里程计或原始IMU及三边距离，评估器另存真值，真值不送入估计器。默认滑窗5秒、每边最多128条测距、200毫秒解算间隔，测距序列连续生成。

试验分两层：理想局部里程计用来隔离RTE变换估计本身；原始IMU试验经过现有本地惯导库，用来检查从原始传感器到RTE的完整计算链。测距叠加独立零均值高斯噪声，标准差分别2厘米、10厘米；每组保留全部固定随机种子。含噪IMU另外叠加每样本陀螺0.0005弧度/秒、加速度0.02米/秒²白噪声，并在车2增加0.002米/秒²固定加速度零偏；该设定是算法敏感性试验，不是现有硬件噪声标定。

误差使用SDK返回的共同状态Header历元配对真值，位置直接在参考车局部坐标轴下比较，不事后旋转或平移拟合轨迹。位置均方根误差按车2、车3有效输出合并统计；航向误差作角度环绕处理。静止初始化、信息不足和输入过期不会被写成零误差，可用比例、初始化时刻和失效恢复单独保存。

### 5.3 全部随机用例的结果与含义

{sim_table}

表中0.3米仅用于统一比较本轮试验，不是已签订或已经达到的合同指标。另有两组无噪声对照：仿真无噪声局部运动输入对照位置均方根误差约{zero['position_rmse_m']:.3g}米，仅说明无噪声合成输入下模型与求解数值一致；仿真理想IMU链路对照仍出现约{minimum:.3f}米位置均方根误差，说明数值积分误差和早期初始化分支也可能触发错误收敛。

![图8　全部固定随机种子的RTE精度结果。每个点对应一整次试验，包含误差较大的用例，不按结果筛选。](figures/07_RTE全部随机用例精度.png)

![图9　固定展示2厘米测距噪声的种子0与4。上图为位置误差，下图为测距拟合残差。](figures/08_位置误差与测距残差对照.png)

18组用例均按完整测距序列运行，位置误差使用独立真值计算；状态码和首次有效时刻用于说明解算输出状态。

本轮发现的主要问题是当前实现采用局部数值求解，初次初始化后持续解算主要沿已有状态更新。短窗或早期运动不足时，错误分支可能通过现有残差和条件数门限，随后难以自动跳出。这里给出的是已观察到的失效现象和代码路径上的风险判断；尚未用全局最优证书确定每一个失败用例的唯一原因。

工程上已有清晰的改进方向：延长并量化初始化运动激励；对多个候选解进行跨窗口验证；加入错误初始化识别、重新初始化和独立运动一致性检查；用质量更稳定的本地里程计降低IMU漂移影响。改进后须重复相同种子和新增轨迹，证明性能提升不是只对某一组数据调参。上述改进尚未在本次测评中写成已完成成果。

## 6. 当前能力、目标能力与交付边界

| 能力项 | 当前可交付/可验证 | 下一阶段目标 |
|---|---|---|
| 三节点采集与回放 | 实盒UWB/IMU、三盒Livox，统一录包入口、中文分析及可视化 | 扩充动态、已知间距、遮挡与中远距离数据集 |
| 相对位置 | 实盒UWB规范三角形，位置有效位与持续消息 | 与显示、决策模块完成有效性和超时联调 |
| 本地航位推算 | 10秒静止初始化、本地惯性传播、紧凑NodeState | 零速/运动约束及可选视觉、雷达里程计接入 |
| 相对位姿初始化 | 独立RTE库、三车节点及仿真评估 | 提高正确初始化率，排除错误分支后验收动态精度 |
| 异常处理 | 时间、输入过期、残差、信息不足等状态 | 输出统一健康状态，建立失效降级和恢复联调闭环 |
| 多模态与近岸环境 | 实拍、实测点云与仿真前端基础 | 校准与时间验收后增量融合；逐步补充目标地形 |
| 整体系统协作 | 公共消息和独立算法库，低带宽状态设计 | 与合作方在同一时基和坐标契约下完成系统场景验证 |

RTE的本次扩展不会反向修改每车本地ESKF状态，主要修正相对变换和输出。现有UWB基线也不输出车辆航向。两项边界应在接口联调中明确，避免把“位置可以显示”理解为“所有控制所需状态均已验收”。

## 7. 与通信、决策、规划和控制的联调方式

### 7.1 先冻结可测试的接口契约

我方提供稳定的定位输出接口，合作方不需要介入算法内部。ROS 2使用CooperativePose2DArray，包含Header、参考节点、各车编号、相对位置、航向及有效位；原生算法同时提供C ABI接口，可供异构进程按需封装。参考坐标、单位、时间、有效性与更新频率必须写入同一版接口说明后再开展集成。

| 对接方 | 定位侧提供 | 对方配合事项 | 联调验收方法 |
|---|---|---|---|
| 通信 | 紧凑状态、测距、时间戳、车辆编号和必要质量信息 | 传输路径、链路状态、带宽/时延/丢包的测量 | 记录双端消息与序号，注入丢包/延迟，检查过期判定 |
| 决策 | 三车相对几何、有效性、时间与算法状态 | 明确何时允许使用状态、失效时采取何种策略 | 输入正常/退化/中断回放，检查状态转换 |
| 规划 | 通过验收的坐标与参考系、位置/航向有效性 | 提供地图系变换、规划周期与容许误差需求 | 同历元对比坐标，验证参考车运动和坐标变换 |
| 控制 | 通过质量门禁的定位状态 | 控制时钟、速度/姿态需求、超时和安全边界 | 先仿真闭环，再受控低速实车；记录目标与执行轨迹 |
| 显示与验收 | 可视化、录包、回放、统计和异常清单 | 联合确认显示语义与验收口径 | 同一数据包重复回放，核对结果一致性与失效提示 |

### 7.2 用三个可操作层次逐步联调

第一层是消息联调：合作方直接订阅已运行的UWB相对位置或回放数据，验证车辆编号、时间、位置有效位和超时显示。这一层可以立即开始，不必等待RTE精度完成收敛。

第二层是状态联调：在明确坐标契约的仿真环境中交换正常、定位退化、通信中断和恢复状态，检查决策与规划是否正确消费数据。对原UWB基线，航向应始终按无效处理；对尚未通过精度验收的RTE结果，应保留为测试通道，不进入正式控制依据。

第三层是闭环联调：在定位动态精度和时效指标通过后，由合作方规划与控制模块接管运动命令。我方按共同试验脚本提供状态与日志，在小范围、低速、具备独立真值条件下验证，再逐步扩展地形与通信条件。现有仿真运动脚本用于测试，不替代其他合作方的决策、规划或控制成果。

## 8. 后续计划：以验收门槛推进能力升级

各阶段按进入条件推进，不把尚未完成的环境准备写成固定日历承诺。T0指接口、设备、场地和人员共同确认后的项目联调起点。

| 阶段 | 主要工作与原因 | 输出物 | 转入下一阶段的条件 |
|---|---|---|---|
| A 基线接口联调 | 冻结编号、消息、时基、参考系、有效位；恢复Gazebo本轮时钟输入问题 | 接口说明、可回放包、联调记录 | 正常与异常输入均能被各合作模块正确解释 |
| B 测量与本地状态验收 | 校准杆臂/轴系；已知间距测试；动态共同事件对时；IMU约束与里程计质量评估 | 标定表、同步测试记录、带真值动态数据 | 原始时间连续、外参可追溯、本地状态质量满足RTE输入需求 |
| C RTE可靠性与精度 | 多候选初始化、运动激励门槛、错误解检测及恢复；重复本次全部种子并增加轨迹 | 改进版本、全量误差统计、失败用例清单 | 正确初始化率、位置/航向误差、异常恢复均达到共同确认指标 |
| D 系统闭环 | 先由仿真流驱动真实三盒和自组网形成硬件在环；再接入合作方规划控制，最后低速实车 | 硬件在环记录、联合场景录包、任务过程与验收报告 | 不仅跑完任务，还能解释每次失效与降级行为 |
| E 目标环境扩展 | 增加近岸地形、非平整路面、遮挡和通信波动等代表性条件 | 分场景数据集、性能边界与参数基线 | 在明确场地与工况范围内重复验证，不外推未经测试条件 |

### 8.1 建议共同确认的目标指标

以下是下一阶段试验设计建议，用来推动需求对齐，均不是当前实测达标声明，也不是替代合同的承诺。最终值需由平台尺度、速度、通信条件、合作方需求和现场测试共同确定。

| 指标 | 建议的首轮受控场景目标 | 统计与验收口径 |
|---|---|---|
| 场景基线 | 三车、平面/已知高度差、低速、视距、明确距离范围 | 先测10～30米范围内的可用性，再据实扩展；当前短距静态数据不覆盖此范围 |
| 动态相对位置 | 位置均方根误差≤0.30米，95%误差≤0.60米 | 独立RTK/动捕/全站仪真值，同Header历元，逐车及合并统计 |
| 动态航向 | 航向均方根误差≤5度 | 仅对声明航向有效的模式验收，不能从UWB三角形边方向推导 |
| 初始化 | 在规定激励轨迹下30秒内完成；正确率≥95% | 至少20次独立启动，失败、错误初始化和超时均计入分母 |
| 有效输出 | 完成初始化且正常输入时≥10赫兹，有效率≥95% | 同时报输出时效、输入缺失、有效/无效次数；不得只按话题发布频率判断 |
| 时效与失效 | 端到端状态年龄95%分位≤200毫秒；超时按接口门限置无效 | 状态年龄需要验证同域时钟；通信延迟用双端专门测量，不用bag接收减Header代替 |
| 恢复与回归 | 断流、乱序、时间跳变、遮挡后符合约定降级/恢复行为 | 故障注入脚本、原始日志与状态转移一并留档；恢复时限按工况另定 |

### 8.2 系统级目标怎样验收

最终系统演示以约定区域内的多平台协同机动流程为载体，由决策模块下达目标、规划模块生成可行运动方案、控制模块执行、通信模块维持状态交换，我方持续提供经过验收的定位状态及质量。系统级记录同时保存目标、规划、控制、通信状态、定位与真值，按任务完成率、轨迹偏差、有效定位覆盖率、状态年龄及异常降级行为共同评价。

由此，定位成果能直接进入整体任务链，也能在出现问题时定位到数据、通信、算法或执行环节。系统级指标由各合作方联合确认；不能以一次画面演示或单独定位测试替代整体任务验收，也不能把整体任务成功归结为定位单一模块的承诺。

## 9. 建议甲方确认的联调事项

项目已具备可运行的硬件输入、在线相对位置基线和可复现的软件链路；需要优先组织的是跨模块接口联调和动态真值条件，而非等待所有增强算法同时成熟。这样可以一边完成系统集成，一边用量化门槛推进定位性能。

建议首先共同确定三件事：一是合作方实际需要的坐标系、航向、更新周期和失效语义；二是带独立真值的受控动态试验场地与测量手段；三是按本报告A～E阶段组织联合验证，明确各阶段的进入条件与交付证据。对尚未通过的RTE精度项，采用独立测试通道持续改进；已运行的UWB基线继续承担早期接口与相对几何验证，保证系统联调有稳定起点。

## 附录A　证据来源与复现入口

| 证据编号 | 文件或工具 | 用途 |
|---|---|---|
| E01 | data/joint_time_20261016_150956 与 data/joint_time_20261017_151037 | 原始UWB、车载IMU、201在线位置输出 |
| E02 | data/three_boxes_20260811_054640 与 data/three_boxes_20260811_060745 | 三盒多传感器历史包及时间退化审查 |
| E03 | data/rte_accuracy_20261023/summary.json 与各_output.csv | 18组合成试验、6组实测审查、逐帧状态与误差计算 |
| E04 | data/rte_accuracy_20261023/uwb_baseline_*/analysis.json | 两组在线结果的重新统计 |
| E05 | data/camera_samples_analysis/video*_preview.png | 201三路原始采集图像的预览 |
| E06 | data/three_boxes_20260811_012046/livox_visualization | 三盒实测点云可视化及统计 |
| E07 | ros2/zju_coop_gazebo 与 gazebo_regression.log | 物理仿真资源、测试代码、本轮通过/失败原始记录 |
| E08 | docs/21、docs/参考/44、docs/参考/45 | 现场启动、RTE库边界、三车接入流程 |

报告全部图表来自程序绘制、既有真实图像或实测点云，没有使用生成式图片。仿真轨迹与噪声参数在tools/evaluate_rte.py中固定；原始bag保持只读。评估不改变默认UWB基线，不把静态数据伪造成动态轨迹，也不以拟合残差替代定位误差。
'''
    path=DEST/'多平台协同定位硬件仿真阶段报告与联调方案.md'
    path.write_text(report,encoding='utf-8')
    render_word(report,path.with_suffix('.docx'))
    allrows=[]
    for r in sims:
        yaw=np.sqrt(np.mean([v['yaw_rmse_deg']**2 for v in r['vehicles'].values()])) if r['vehicles'] else None
        allrows.append([r['name'],r['first_ready_s'],num(r['position_rmse_m']),num(yaw),
                        str(r['ready_rows']),str(r['evaluated_rows'])])
    detailed='# RTE精度验证报告（2026-10-23）\n\n'
    detailed+='现有实测全部静止，且无外部相对位置/航向真值。实测结论仅限数据与回放诊断；仿真数据有独立真值。仿真无噪声局部运动输入由仿真轨迹直接生成，用于隔离RTE相对变换求解，不是盒端实际传感器输出；工程中可选雷达里程计前端未用于本次结果。\n\n'
    detailed+='## 结论\n\n默认RTE实现尚不能据此宣称动态精度已达标。多个随机种子存在米级错误解，READY与小测距残差不保证正确位置。现有3项RTE自检检查功能与选定轨迹，不能覆盖本次更宽的精度试验。\n\n'
    detailed+='## 仿真全量结果\n\n表1　仿真全量结果\n\n'+table(['试验编号','首次可用时刻（秒）','位置均方根误差（米）','航向均方根误差（度）','有效输出帧数','评估帧数'],allrows)
    detailed+='\n位置均方根误差合并车2、3；航向均方根误差等权合并车2、3。真值在输出的共同状态历元配对；不做事后坐标拟合。有效输出帧数和评估帧数用于说明统计样本量，首车坐标固定不并入误差，避免用参考车零误差稀释指标。\n\n'
    detailed+='表2　仿真分组统计\n\n'+sim_table+'\n## 实测全量审查\n\n表3　实测全量审查\n\n'+real_table
    detailed+='\n实测输入：原始/imu/raw角速度/比力，经FRD→FLU变换，SI比例均为1；不使用AHRS四元数、不用Livox内置IMU代替车载IMU。每车采用10秒连续静止窗口、现有SDK静止初始化；不增加持续零速更新，不提供三车初始平移/航向真值。平面已知高度差和天线杆臂临时取零，仅作可运行性诊断，不构成已标定部署。\n\n'
    for r in real:
        detailed+=f'### {r["dataset"]}\n\n'
        if r['blockers']: detailed+='门禁：'+'；'.join(r['blockers'])+'。不改写原始Header强行配对。\n\n'
        else:
            rr=r['replay']
            detailed+=f'静止初始化通过：{rr["local_ready"]}；时间故障锁存：{rr["local_fault"]}；末端本地水平位移（米）：{rr["local_final_displacement_m"]}；状态码计数：{rr["status_counts"]}。\n\n'
            detailed+=f'SDK step调用耗时：95%分位{rr["solve_p95_ms"]:.2f}毫秒，最大{rr["solve_max_ms"]:.2f}毫秒。该值来自当前开发机离线运行，含解算/状态查询，且本机同时运行过其他验证进程；不能外推为RK3588实时性能或跨网络端到端时延。\n\n'
        rows=[]
        for name,h in r['streams'].items(): rows.append([name,h['count'],h['zero'],h['rollback'],h['duplicate'],f'{h["median_dt_ms"]:.3f}',f'{h["max_dt_ms"]:.3f}'])
        detailed+=table(['原始Header数据流','帧数','零值','回退','重复','间隔中位数（毫秒）','最大间隔（毫秒）'],rows)+'\n'
    detailed+='## 复现\n\n从项目根目录执行；现有build_rte_verify是本次使用的独立RTE构建目录。新环境先用cmake -S . -B build_rte_verify -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON -DZJU_COOP_BUILD_RTE=ON配置。\n\n```bash\n'
    detailed+='cmake --build build_rte_verify --target zju_coop zju_coop_rte zju_coop_rte_self_check zju_coop_three_rte_self_check c_header_smoke_rte -j2\n'
    detailed+='c++ -std=c++17 -O2 -I include tools/evaluate_rte.cpp -L build_rte_verify -Wl,-rpath,"$PWD/build_rte_verify" -lzju_coop_rte -lzju_coop -o build_rte_verify/evaluate_rte\n'
    detailed+='source /opt/ros/humble/setup.bash\nsource install/ros2-uwb-online/setup.bash\npython3 -u tools/evaluate_rte.py\nctest --test-dir build_rte_verify --output-on-failure -R rte\n```\n\n'
    detailed+='主要结果看summary.json、sim_*_output.csv、joint_time_*_output.csv和配套报告figures/07、08、09。输入CSV是机器接口格式，中文全量汇总另存为RTE仿真精度汇总.csv。离线适配将接收时刻置为当前Header时刻以隔离算法数值，不模拟真实网络延迟、ROS调度或硬件授时误差；不是实盒端到端时效验收。\n'
    (DATA/'RTE精度验证报告.md').write_text(detailed,encoding='utf-8')
    with (DATA/'RTE仿真精度汇总.csv').open('w',encoding='utf-8-sig',newline='') as f:
        w=csv.writer(f); w.writerow(['试验编号','首次可用时刻_秒','位置均方根误差_米','航向均方根误差_度','有效输出帧数','评估帧数']); w.writerows(allrows)
    shutil.copy2(DATA/'summary.json',DEST/'评估结果汇总.json')
    shutil.copy2(DATA/'RTE仿真精度汇总.csv',DEST/'RTE仿真精度汇总.csv')
    shutil.copy2(DATA/'RTE精度验证报告.md',DEST/'RTE精度验证报告.md')
    source_dir=DEST/'复现脚本'; source_dir.mkdir(exist_ok=True)
    for name in ['evaluate_rte.py','evaluate_rte.cpp','build_stage_report.py']:
        shutil.copy2(ROOT/'tools'/name,source_dir/name)
    for name in ['gazebo_regression.log','gazebo_run_diagnostic.log','gazebo_startup_diagnostic.log','rte_regression.log','原始输入文件校验清单.json']:
        if (DATA/name).exists(): shutil.copy2(DATA/name,DEST/name)
    manifest={'date':'2026-10-23','git_head':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
              'environment':{'platform':platform.platform(),'python':platform.python_version(),
                 'numpy':np.__version__,'matplotlib':matplotlib.__version__,
                 'compiler':subprocess.check_output(['c++','--version'],text=True).splitlines()[0]},
              'code_and_evidence_sha256':{},'notes':'原始bag只读；本报告生成于含未提交评估工具的工作区，代码哈希随附。'}
    inputs=[ROOT/'tools/evaluate_rte.py',ROOT/'tools/evaluate_rte.cpp',Path(__file__).resolve(),
            ROOT/'src/rte/relative_transform.cpp',ROOT/'src/rte/three_vehicle_rte.cpp',DATA/'summary.json',
            ROOT/'build_rte_verify/libzju_coop.so',ROOT/'build_rte_verify/libzju_coop_rte.so',ROOT/'build_rte_verify/evaluate_rte']
    for p in inputs: manifest['code_and_evidence_sha256'][str(p.relative_to(ROOT))]=hashlib.sha256(p.read_bytes()).hexdigest()
    (DEST/'复现版本清单.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
    (DEST/'阅读与复现说明.md').write_text('''# 阅读与复现说明

优先阅读同目录的《多平台协同定位硬件仿真阶段报告与联调方案.docx》或PDF；Markdown为正文源稿。Word可编辑，figures目录同时保留PNG及矢量SVG（原始点云图为PNG）。

《RTE精度验证报告.md》包含全部18组合成用例和6组实测数据审查。《RTE仿真精度汇总.csv》为中文表头的全量汇总。《评估结果汇总.json》为机器可读指标。完整逐帧文件位于工程data/rte_accuracy_20261023/。

复现脚本目录附带本次测评与成文代码，供审阅；这些脚本依赖本工程SDK和原始数据，应在工程tools目录运行，不是脱离工程独立安装包。原始输入数据库的SHA-256清单及代码、动态库版本清单随附。

复现评估的命令见《RTE精度验证报告.md》。生成文档需要python-docx（本次1.2.0）、现有numpy、matplotlib和Pillow；运行python3 tools/build_stage_report.py，然后使用LibreOffice导出PDF。临时依赖可用python3 -m pip install --target /tmp/cooperative_report_deps python-docx安装，生成时加PYTHONPATH=/tmp/cooperative_report_deps。

```bash
PYTHONPATH=/tmp/cooperative_report_deps python3 tools/build_stage_report.py
libreoffice -env:UserInstallation=file:///tmp/cooperative_report_lo --headless --convert-to pdf --outdir docs/报告/20261023_协同定位阶段报告 docs/报告/20261023_协同定位阶段报告/*.docx
```

本次只新增离线测评与报告，没有替换默认算法、改写原始bag、部署实盒或向任何合作方发送材料。建议目标均待双方确认；实测精度尚无真值支撑。
''',encoding='utf-8')
    print(f'报告与附件已生成：{DEST}')


if __name__=='__main__': main()
