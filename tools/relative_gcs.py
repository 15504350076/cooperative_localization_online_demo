#!/usr/bin/env python3
"""只读协同定位监视界面：订阅三车 ROS 2 相对位置，在浏览器显示队形和失效状态。

运行在装有本工程 ROS 2 消息包的机器上；网页只读取本机 HTTP 快照，不发控制指令。
不把 UWB 模拟时间当 UTC，也不把无效坐标或未知车头方向画成实时位姿。
"""

import argparse
import json
import math
import re
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from types import SimpleNamespace


POSE_TOPIC = "/cooperative_localization/poses_2d"
FRAME_PATTERN = re.compile(r"coop_ref_([1-9][0-9]*)_(relative|enu)\Z")
VALID_AGE_S = 0.3
HOLD_AGE_S = 1.0
OFFLINE_AGE_S = 2.0


class RelativeState:
    """原子保存最新整组位姿；新鲜度始终按本机单调接收时钟计算。"""

    def __init__(self, demo=False):
        self._lock = threading.Lock()
        self.demo = demo
        self.started_at = time.monotonic()
        self.latest = None
        self.last_good = {}
        self.messages = 0
        self.last_error = ""

    def ingest(self, message, received_at=None):
        """接收一条 CooperativePose2DArray；整组校验成功后才替换页面状态。"""
        now = time.monotonic() if received_at is None else received_at
        try:
            reference = int(message.reference_node_id)
            frame = str(message.header.frame_id)
            match = FRAME_PATTERN.fullmatch(frame)
            if reference < 1 or match is None or int(match.group(1)) != reference:
                raise ValueError("参考车编号与坐标系不匹配")
            stamp = message.header.stamp
            stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
            vehicles = {}
            for item in message.vehicles:
                node_id = int(item.node_id)
                x, y, yaw = float(item.x_m), float(item.y_m), float(item.yaw_rad)
                if node_id < 1 or node_id in vehicles:
                    raise ValueError("车辆编号无效或重复")
                valid = bool(item.position_valid)
                if valid and not (math.isfinite(x) and math.isfinite(y)):
                    raise ValueError("有效位姿含非有限坐标")
                yaw_valid = bool(item.yaw_valid) and match.group(2) == "enu"
                if yaw_valid and not math.isfinite(yaw):
                    raise ValueError("有效航向含非有限角度")
                vehicles[node_id] = {
                    "position_valid": valid,
                    "yaw_valid": yaw_valid,
                    "x": x if valid else None,
                    "y": y if valid else None,
                    "yaw": yaw if yaw_valid else None,
                }
            if not vehicles:
                raise ValueError("位姿数组为空")
        except (AttributeError, TypeError, ValueError, OverflowError) as error:
            with self._lock:
                self.last_error = str(error)
            return False

        with self._lock:
            # 不跨坐标系或参考车沿用旧点；否则会把两套坐标混画。
            if self.latest and self.latest["frame_id"] != frame:
                self.last_good.clear()
            self.latest = {
                "frame_id": frame,
                "reference_node_id": reference,
                "stamp_ns": stamp_ns,
                "received_at": now,
                "vehicles": vehicles,
            }
            for node_id, pose in vehicles.items():
                if pose["position_valid"]:
                    self.last_good[node_id] = {**pose, "received_at": now}
            self.messages += 1
            self.last_error = ""
        return True

    def snapshot(self, now=None):
        """供 HTTP 页面取最新值；失效与断流均不能刷新最后有效位置的年龄。"""
        now = time.monotonic() if now is None else now
        with self._lock:
            latest = self.latest
            receive_age = None if latest is None else max(0.0, now - latest["received_at"])
            nodes = []
            ids = set(self.last_good)
            if latest:
                ids.update(latest["vehicles"])
            for node_id in sorted(ids):
                current = latest["vehicles"].get(node_id) if latest else None
                good = self.last_good.get(node_id)
                good_age = None if good is None else max(0.0, now - good["received_at"])
                if receive_age is None or receive_age > OFFLINE_AGE_S:
                    status = "OFFLINE"
                elif current and current["position_valid"] and receive_age <= VALID_AGE_S:
                    status = "VALID"
                elif good_age is not None and good_age <= HOLD_AGE_S:
                    status = "HOLD"
                elif good is not None:
                    status = "LOST"
                else:
                    status = "WAITING"
                nodes.append({
                    "node_id": node_id,
                    "status": status,
                    "x": good["x"] if good and status in ("VALID", "HOLD") else None,
                    "y": good["y"] if good and status in ("VALID", "HOLD") else None,
                    "yaw": good["yaw"] if good and status == "VALID" else None,
                    "yaw_valid": bool(good and good["yaw_valid"] and status == "VALID"),
                    "last_valid_age_ms": None if good_age is None else round(good_age * 1000),
                })
            return {
                "demo": self.demo,
                "frame_id": latest["frame_id"] if latest else "--",
                "reference_node_id": latest["reference_node_id"] if latest else None,
                "stamp_ns": latest["stamp_ns"] if latest else None,
                "receive_age_ms": None if receive_age is None else round(receive_age * 1000),
                "messages": self.messages,
                "nodes": nodes,
                "last_error": self.last_error,
            }


PAGE = r"""<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>浙大协同定位监视界面</title><style>
:root{font-family:system-ui,"Microsoft YaHei",sans-serif;color:#e5eefb;background:#081321}
*{box-sizing:border-box}body{margin:0}header{padding:20px 26px;border-bottom:1px solid #20364d;background:#0d1c2c}
h1{font-size:22px;margin:0 0 6px}.muted{color:#9eb0c3;font-size:13px}.badge{display:inline-block;padding:5px 10px;border-radius:20px;background:#25384a;margin-right:8px}
.warning{color:#ffcb79}.layout{display:grid;grid-template-columns:minmax(440px,2fr) minmax(270px,1fr);gap:14px;padding:14px}
.panel{background:#102236;border:1px solid #25405a;border-radius:12px;padding:16px}.panel h2{font-size:16px;margin:0 0 12px}
canvas{width:100%;height:min(62vh,630px);display:block;background:#0b1a2b;border-radius:8px}
.grid{display:grid;gap:10px}.vehicle{border:1px solid #29435c;border-radius:8px;padding:10px;background:#0c1c2d}
.vehicle strong{font-size:16px}.state{float:right;font-weight:700}.VALID{color:#58e6a1}.HOLD{color:#ffc774}.LOST,.OFFLINE{color:#ff7f88}.WAITING{color:#9eb0c3}
.metric{color:#b9cad9;font-size:13px;margin-top:7px}footer{padding:0 14px 14px;color:#9eb0c3;font-size:13px}
@media(max-width:850px){.layout{grid-template-columns:1fr}canvas{height:50vh}}
</style></head><body>
<header><h1>浙大协同定位监视界面</h1><div class="muted">只读监视 · 局部米制坐标 · 不向车辆发送控制指令</div>
<p><span id="mode" class="badge">等待数据</span><span id="frame" class="badge">坐标系 --</span><span id="age" class="badge">接收年龄 --</span></p>
<div id="meaning" class="muted">UWB-only：1号车为原点，1→2为正X，3号车位于正Y侧；不是地图/北向坐标。</div></header>
<main class="layout"><section class="panel"><h2>三车相对队形</h2><canvas id="map"></canvas><p class="muted">绿点：当前有效；黄点：短时保持，仅供查看；丢失后不画旧点。无航向有效位时不画车头。</p></section>
<aside class="grid"><section class="panel"><h2>车辆状态</h2><div id="vehicles">等待位姿消息…</div></section>
<section class="panel"><h2>消息与限制</h2><div id="details" class="metric">等待消息…</div><p class="metric warning">GNSS真值、传感器健康和控制功能尚未接入本页；不能据此评价绝对精度。</p></section></aside></main>
<footer>新鲜度按本 GCS 接收时的单调时钟判断；Header 仅显示原值，不解释为当前 UTC。</footer>
<script>
const canvas=document.getElementById('map'),ctx=canvas.getContext('2d');
function plot(model){
  const rect=canvas.getBoundingClientRect(),dpr=devicePixelRatio||1;
  canvas.width=Math.max(1,Math.round(rect.width*dpr));canvas.height=Math.max(1,Math.round(rect.height*dpr));
  ctx.setTransform(dpr,0,0,dpr,0,0);const w=rect.width,h=rect.height;
  ctx.clearRect(0,0,w,h);ctx.fillStyle='#0b1a2b';ctx.fillRect(0,0,w,h);
  const nodes=model.nodes.filter(n=>n.x!==null&&n.y!==null);
  if(!nodes.length){ctx.fillStyle='#9eb0c3';ctx.textAlign='center';ctx.font='16px sans-serif';ctx.fillText('等待有效相对位置',w/2,h/2);return;}
  const xs=[0,...nodes.map(n=>n.x)],ys=[0,...nodes.map(n=>n.y)];
  const minX=Math.min(...xs),maxX=Math.max(...xs),minY=Math.min(...ys),maxY=Math.max(...ys);
  const span=Math.max(2,maxX-minX,maxY-minY),scale=Math.min((w-100)/span,(h-100)/span);
  const midX=(minX+maxX)/2,midY=(minY+maxY)/2;
  const p=(x,y)=>[w/2+(x-midX)*scale,h/2-(y-midY)*scale];
  ctx.strokeStyle='#233e55';ctx.lineWidth=1;
  for(let i=-20;i<=20;i++){let a=p(i,-20),b=p(i,20);ctx.beginPath();ctx.moveTo(...a);ctx.lineTo(...b);ctx.stroke();a=p(-20,i);b=p(20,i);ctx.beginPath();ctx.moveTo(...a);ctx.lineTo(...b);ctx.stroke();}
  const o=p(0,0);ctx.strokeStyle='#7590aa';ctx.lineWidth=2;ctx.beginPath();ctx.moveTo(o[0],0);ctx.lineTo(o[0],h);ctx.moveTo(0,o[1]);ctx.lineTo(w,o[1]);ctx.stroke();
  ctx.fillStyle='#9eb0c3';ctx.font='12px sans-serif';ctx.textAlign='right';ctx.fillText('+X →',w-12,Math.max(18,o[1]-8));ctx.textAlign='left';ctx.fillText('+Y ↑',Math.min(w-55,o[0]+8),16);
  if(nodes.length>=2){ctx.setLineDash([5,5]);ctx.strokeStyle='#426581';ctx.lineWidth=2;ctx.beginPath();nodes.forEach((n,i)=>{const q=p(n.x,n.y);i?ctx.lineTo(...q):ctx.moveTo(...q)});ctx.closePath();ctx.stroke();ctx.setLineDash([])}
  nodes.forEach(n=>{const q=p(n.x,n.y);ctx.fillStyle=n.status==='VALID'?'#58e6a1':'#ffc774';ctx.beginPath();ctx.arc(q[0],q[1],13,0,Math.PI*2);ctx.fill();ctx.fillStyle='#081321';ctx.textAlign='center';ctx.font='bold 13px sans-serif';ctx.fillText(String(n.node_id),q[0],q[1]+4);
    if(n.yaw_valid){ctx.strokeStyle='#e5eefb';ctx.lineWidth=3;ctx.beginPath();ctx.moveTo(...q);ctx.lineTo(q[0]+30*Math.cos(n.yaw),q[1]-30*Math.sin(n.yaw));ctx.stroke()}
    ctx.fillStyle='#dceaf5';ctx.font='12px sans-serif';ctx.fillText(`车${n.node_id} (${n.x.toFixed(2)}, ${n.y.toFixed(2)}) m`,q[0],q[1]+32)});
}
async function refresh(){try{const res=await fetch('/api/state',{cache:'no-store'});if(!res.ok)throw Error('HTTP '+res.status);const s=await res.json();
  document.getElementById('mode').textContent=s.demo?'演示数据，非实车':'ROS 2 实时订阅';
  document.getElementById('frame').textContent='坐标系 '+s.frame_id;
  document.getElementById('age').textContent='接收年龄 '+(s.receive_age_ms===null?'--':s.receive_age_ms+' ms');
  document.getElementById('meaning').textContent=s.frame_id.endsWith('_enu')?'公共 ENU 平行轴；仍是相对参考车的位置，不是绝对经纬度。':'UWB-only：1号车为原点，1→2为正X，3号车位于正Y侧；不是地图/北向坐标。';
  const box=document.getElementById('vehicles');box.replaceChildren();
  if(!s.nodes.length)box.textContent='尚无有效车辆位置';
  for(const n of s.nodes){const item=document.createElement('div');item.className='vehicle';const title=document.createElement('strong');title.textContent='车辆 '+n.node_id;const status=document.createElement('span');status.className='state '+n.status;status.textContent=n.status;const detail=document.createElement('div');detail.className='metric';detail.textContent=n.x===null?'位置不可用':`x=${n.x.toFixed(3)} m, y=${n.y.toFixed(3)} m · 最近有效 ${n.last_valid_age_ms} ms`;const yaw=document.createElement('div');yaw.className='metric';yaw.textContent=n.yaw_valid?`车头航向 ${(n.yaw*180/Math.PI).toFixed(1)}°`:'车头航向不可观';item.append(title,status,detail,yaw);box.append(item)}
  document.getElementById('details').textContent=`接收帧数 ${s.messages} · 参考车 ${s.reference_node_id??'--'} · 消息Header ${s.stamp_ns??'--'} ns`+(s.last_error?' · 拒收异常帧：'+s.last_error:'');plot(s);
}catch(e){document.getElementById('mode').textContent='页面连接中断';document.getElementById('age').textContent='接收年龄 --';document.getElementById('vehicles').textContent='无法读取 GCS 数据服务';canvas.getContext('2d').clearRect(0,0,canvas.width,canvas.height)}}
setInterval(refresh,300);addEventListener('resize',refresh);refresh();
</script></body></html>"""


class GcsHttpServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True


def create_server(host, port, state):
    """创建只读页面和 JSON API；默认只绑定本机，远程开放须显式指定。"""
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            path = self.path.partition("?")[0]
            if path == "/api/state":
                body = json.dumps(state.snapshot(), ensure_ascii=False).encode("utf-8")
                kind, code = "application/json; charset=utf-8", 200
            elif path in ("/", "/index.html"):
                body = PAGE.encode("utf-8")
                kind, code = "text/html; charset=utf-8", 200
            else:
                body, kind, code = b"not found", "text/plain; charset=utf-8", 404
            self.send_response(code)
            self.send_header("Content-Type", kind)
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, _format, *args):
            pass

    return GcsHttpServer((host, port), Handler)


def demo_message(tick):
    """只供页面自检的假三车位姿，绝不作为真实定位数据。"""
    vehicle = lambda node_id, x, y: SimpleNamespace(
        node_id=node_id, x_m=x, y_m=y, yaw_rad=0.0,
        position_valid=True, yaw_valid=False,
    )
    return SimpleNamespace(
        header=SimpleNamespace(frame_id="coop_ref_1_relative",
                               stamp=SimpleNamespace(sec=0, nanosec=tick)),
        reference_node_id=1,
        vehicles=[vehicle(1, 0, 0), vehicle(2, 2, 0),
                  vehicle(3, 0.8 + 0.2 * math.sin(tick / 10), 1.4)],
    )


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--demo", action="store_true", help="仅测试界面，不连接 ROS 2")
    parser.add_argument("--bind", default="127.0.0.1", help="HTTP 绑定地址；远程访问用 0.0.0.0")
    parser.add_argument("--port", type=int, default=8765, help="HTTP 端口")
    parser.add_argument("--topic", default=POSE_TOPIC, help="ROS 2 位姿话题")
    args = parser.parse_args(argv)
    if not 0 <= args.port <= 65535:
        parser.error("端口必须在 0..65535")
    state = RelativeState(demo=args.demo)
    if not args.demo:
        try:
            import rclpy
            from cooperative_localization_msgs.msg import CooperativePose2DArray
            from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        except ImportError as error:
            parser.error(f"请先加载 ROS 2 和浙大消息工作空间：{error}")
        rclpy.init()
        node = rclpy.create_node("zju_relative_gcs")
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        node.create_subscription(CooperativePose2DArray, args.topic, state.ingest, qos)
    else:
        node = None
    try:
        server = create_server(args.bind, args.port, state)
        worker = threading.Thread(target=server.serve_forever, daemon=True)
        worker.start()
        print(f"协同定位监视界面已启动：http://{args.bind}:{server.server_port}  模式={'演示' if args.demo else 'ROS 2'}", flush=True)
        try:
            if args.demo:
                tick = 0
                while True:
                    state.ingest(demo_message(tick))
                    tick += 1
                    time.sleep(0.1)
            else:
                rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            server.shutdown()
            server.server_close()
            worker.join(timeout=2)
    finally:
        if node is not None:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
