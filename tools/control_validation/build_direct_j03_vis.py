#!/usr/bin/env python3
"""Build a compact four-panel HTML fragment for a direct J03 sine run."""

from __future__ import annotations

import csv
import json
import sys
from pathlib import Path


def main() -> int:
    source = Path(sys.argv[1])
    destination = Path(sys.argv[2])
    rows = list(csv.DictReader(source.open("r", encoding="utf-8-sig", newline="")))
    points = []
    for row in rows:
        points.append(
            {
                "t": round(float(row["t_rel"]), 3),
                "phase": row["phase"],
                "target": [10.0, 10.0, 20.0, round(float(row["command_j03_target"]), 3)],
                "actual": [round(float(row[f"j{i}_actual"]), 3) for i in range(4)],
            }
        )
    payload = json.dumps(points, ensure_ascii=False, separators=(",", ":"))
    fragment = r'''<div id="direct-j03-sine" class="dj-wrap">
  <style>
    .dj-wrap{font-family:ui-sans-serif,system-ui,-apple-system,"Segoe UI",sans-serif;color:var(--foreground);background:var(--background);padding:18px;border:1px solid var(--border);border-radius:14px}
    .dj-title{display:flex;align-items:flex-end;justify-content:space-between;gap:18px;flex-wrap:wrap;margin-bottom:8px}
    .dj-title h2{font-size:20px;line-height:1.2;margin:0}.dj-title p{margin:5px 0 0;color:var(--muted-foreground);font-size:13px}
    .dj-metrics{font-size:12px;color:var(--muted-foreground);display:flex;gap:14px;flex-wrap:wrap}.dj-metrics b{color:var(--foreground);font-weight:650}
    .dj-legend{display:flex;gap:16px;align-items:center;font-size:12px;color:var(--muted-foreground);margin:10px 0 2px}
    .dj-key{display:inline-flex;gap:6px;align-items:center}.dj-line{width:23px;height:0;border-top:2px solid var(--viz-series-1)}.dj-line.target{border-color:var(--viz-series-2);border-top-style:dashed}
    .dj-panel{margin-top:8px}.dj-panel-label{font-size:12px;font-weight:650;margin:0 0 3px 2px}.dj-canvas{display:block;width:100%;height:178px;border:1px solid var(--border);border-radius:9px;background:var(--background)}
    .dj-foot{font-size:11px;color:var(--muted-foreground);text-align:center;margin-top:8px}
  </style>
  <div class="dj-title"><div><h2>J03 正弦响应｜直连 100 ms 下发</h2><p>初始姿态 10°, 10°, 20°, 10°；J03：20° ± 10°，0.1 Hz，3 周期</p></div><div class="dj-metrics"><span>发送 <b>570 帧</b></span><span>中位延迟 <b>3 ms</b></span><span>峰值张力 <b>28.9 N</b></span></div></div>
  <div class="dj-legend"><span class="dj-key"><i class="dj-line"></i>实测角度</span><span class="dj-key"><i class="dj-line target"></i>目标角度</span><span>灰色区域：基线/回零保持</span></div>
  <div id="dj-panels"></div>
  <div class="dj-foot">时间（s）｜基线 0–15 s · 正弦 15–45 s · 回零 45–57 s</div>
  <script>
  (()=>{
    const root=document.getElementById('direct-j03-sine');
    const data=__DATA__;
    const panels=root.querySelector('#dj-panels');
    for(let j=0;j<4;j++){
      const box=document.createElement('div');box.className='dj-panel';
      box.innerHTML=`<div class="dj-panel-label">J0${j} 角度（°）</div><canvas class="dj-canvas"></canvas>`;
      panels.appendChild(box);
      const canvas=box.querySelector('canvas');
      const draw=()=>{
        const dpr=Math.max(1,window.devicePixelRatio||1),w=canvas.clientWidth,h=canvas.clientHeight;
        canvas.width=Math.round(w*dpr);canvas.height=Math.round(h*dpr);const c=canvas.getContext('2d');c.setTransform(dpr,0,0,dpr,0,0);
        const css=getComputedStyle(root),fg=css.getPropertyValue('--foreground').trim()||'#111827',muted=css.getPropertyValue('--muted-foreground').trim()||'#64748b',border=css.getPropertyValue('--border').trim()||'#d1d5db';
        const resolvedColor=(name,fallback)=>{const s=document.createElement('span');s.style.color=`var(${name},${fallback})`;root.appendChild(s);const value=getComputedStyle(s).color;s.remove();return value||fallback};
        const actual=resolvedColor('--viz-series-1','#2563eb'),target=resolvedColor('--viz-series-2','#f97316');
        const m={l:50,r:14,t:12,b:28},x0=m.l,x1=w-m.r,y0=m.t,y1=h-m.b,tMin=data[0].t,tMax=data[data.length-1].t;
        const vals=data.flatMap(p=>[p.actual[j],p.target[j]]),lo=Math.min(...vals),hi=Math.max(...vals),pad=Math.max(.6,(hi-lo)*.14),yMin=lo-pad,yMax=hi+pad;
        const X=t=>x0+(t-tMin)/(tMax-tMin)*(x1-x0),Y=v=>y1-(v-yMin)/(yMax-yMin)*(y1-y0);
        c.fillStyle='rgba(127,127,127,.07)';c.fillRect(X(tMin),y0,X(15)-X(tMin),y1-y0);c.fillRect(X(45),y0,X(tMax)-X(45),y1-y0);
        c.strokeStyle=border;c.lineWidth=1;c.fillStyle=muted;c.font='11px ui-sans-serif,system-ui';
        for(let k=0;k<=4;k++){const yy=y0+(y1-y0)*k/4,v=yMax-(yMax-yMin)*k/4;c.beginPath();c.moveTo(x0,yy);c.lineTo(x1,yy);c.stroke();c.fillText(v.toFixed(1),4,yy+4)}
        for(let t=0;t<=50;t+=5){const xx=X(t);c.beginPath();c.moveTo(xx,y1);c.lineTo(xx,y1+4);c.stroke();c.textAlign='center';c.fillText(String(t),xx,y1+17)}c.textAlign='left';
        const line=(key,color,dash)=>{c.beginPath();data.forEach((p,i)=>{const xx=X(p.t),yy=Y(p[key][j]);i?c.lineTo(xx,yy):c.moveTo(xx,yy)});c.strokeStyle=color;c.lineWidth=2;c.setLineDash(dash);c.stroke();c.setLineDash([])};
        line('target',target,[7,5]);line('actual',actual,[]);
        c.fillStyle=fg;c.font='10px ui-sans-serif,system-ui';c.fillText('基线',X(4),y0+12);c.fillText('正弦',X(28),y0+12);c.fillText('回零',X(49),y0+12);
      };
      new ResizeObserver(draw).observe(canvas);draw();
    }
  })();
  </script>
</div>'''.replace("__DATA__", payload)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(fragment, encoding="utf-8")
    print(destination)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
