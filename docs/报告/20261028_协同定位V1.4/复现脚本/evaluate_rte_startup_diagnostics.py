#!/usr/bin/env python3
"""在独立副本增加只读诊断，比较窗口/位移条件；不覆盖现有SDK或参数。"""
import csv
import argparse
import hashlib
import json
import os
import subprocess
import time
from pathlib import Path
import numpy as np
from evaluate_rte import ROOT, summarize

OUT=ROOT/'data/rte_startup_diagnostics_20261030'


def build():
    folder=OUT/'build';folder.mkdir(parents=True,exist_ok=True)
    source=(ROOT/'src/rte/three_vehicle_rte.cpp').read_text();needle='void correct(zju_coop_three_handle_rte_t& h, uint64_t now) {'
    helper='''void diagnostic(const zju_coop_three_handle_rte_t& h, uint64_t now, const char* why, double value=0, double limit=0, int node=-1, int axis=-1) {
  if(h.initialized) return;
  static FILE* file=[](){ const char* p=std::getenv("ZJU_RTE_DIAGNOSTIC"); if(!p) return static_cast<FILE*>(nullptr); FILE* f=std::fopen(p,"w"); if(f) std::fprintf(f,"timestamp_ns,reason,value,limit,node,axis\\n"); return f; }();
  if(file) std::fprintf(file,"%llu,%s,%.17g,%.17g,%d,%d\\n",static_cast<unsigned long long>(now),why,value,limit,node,axis);
}
'''
    source=source.replace('#include <cstdio>','#include <cstdio>\n#include <cstdlib>');assert needle in source;source=source.replace(needle,helper+needle)
    replacements={
        'if (data.size() < 2 * h.config.minimum_samples) {':'if (data.size() < 2 * h.config.minimum_samples) { diagnostic(h,now,"samples",data.size(),2*h.config.minimum_samples);',
        'h.quality = ZJU_COOP_RTE_NO_ESTIMATE;\n    return;\n  }\n  std::vector<Fit> fits;':'diagnostic(h,now,"motion_span",0,h.config.minimum_initialization_motion_span_m); h.quality = ZJU_COOP_RTE_NO_ESTIMATE;\n    return;\n  }\n  std::vector<Fit> fits;',
        'if (!best->converged || !best->state.allFinite()) {':'if (!best->converged || !best->state.allFinite()) { diagnostic(h,now,"not_converged",best->cost);',
        'h.quality = ZJU_COOP_RTE_UNOBSERVABLE;\n    return;':'diagnostic(h,now,"geometry",eigen.eigenvalues().maxCoeff()/eigen.eigenvalues().minCoeff(),h.config.condition_limit); h.quality = ZJU_COOP_RTE_UNOBSERVABLE;\n    return;',
        'if (rms > h.config.max_rms_residual_m) {':'if (rms > h.config.max_rms_residual_m) { diagnostic(h,now,"range_residual",rms,h.config.max_rms_residual_m);',
        'h.quality = ZJU_COOP_RTE_UNOBSERVABLE;\n          return;':'diagnostic(h,now,axis==size-1?"yaw_uncertainty":"position_uncertainty",std::sqrt(covariance((n-1)*size+axis,(n-1)*size+axis)),limit,n,axis); h.quality = ZJU_COOP_RTE_UNOBSERVABLE;\n          return;',
        'h.quality = ZJU_COOP_RTE_AMBIGUOUS;':'diagnostic(h,now,"ambiguous"); h.quality = ZJU_COOP_RTE_AMBIGUOUS;',
        'h.state = best->state;':'diagnostic(h,now,"accepted",rms,h.config.max_rms_residual_m); h.state = best->state;'}
    for a,b in replacements.items():assert a in source,a;source=source.replace(a,b)
    path=folder/'three_vehicle_rte_diagnostic.cpp';path.write_text(source)
    runner=(ROOT/'tools/evaluate_rte.cpp').read_text().replace('if(argc!=3 && argc!=4)','if(argc!=5)').replace('if(argc==4) {','if(argc>=4) {').replace('config.minimum_initialization_motion_span_m=10.0;','config.minimum_initialization_motion_span_m=std::stod(argv[4]);')
    (folder/'evaluate_rte_diagnostic.cpp').write_text(runner)
    subprocess.run(['g++','-std=c++17','-O3','-DNDEBUG','-fPIC','-shared','-DZJU_COOP_BUILDING_RTE_LIBRARY','-I'+str(ROOT/'include'),'-I/usr/include/eigen3','-I'+str(ROOT/'src/rte'),str(path),str(ROOT/'src/rte/relative_transform.cpp'),'-o',str(folder/'libzju_coop_rte_diagnostic.so')],check=True)
    subprocess.run(['g++','-std=c++17','-O2','-I'+str(ROOT/'include'),str(folder/'evaluate_rte_diagnostic.cpp'),'-L'+str(folder),'-L'+str(ROOT/'build_rte_verify'),'-Wl,-rpath,'+str(folder)+':'+str(ROOT/'build_rte_verify'),'-lzju_coop_rte_diagnostic','-lzju_coop','-o',str(folder/'evaluate_rte_diagnostic')],check=True)
    return folder/'evaluate_rte_diagnostic'


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--reuse-outputs',action='store_true',help='仅从已完成的重放计算诊断统计');args=parser.parse_args()
    OUT.mkdir(exist_ok=True);runner=None if args.reuse_outputs else build();records=[];began=time.time()
    for window,span in ((5,10),(5,5),(5,0),(10,5)):
        for sigma in (.02,.05,.1):
            name=f'exp_odometry_{sigma:.2f}_seed0';source=ROOT/'data/rte_expanded_20261025'/f'{name}.csv';stem=f'w{window}_d{span}_s{sigma:.2f}';out=OUT/f'{stem}_output.csv';log=OUT/f'{stem}_stages.csv'
            if not args.reuse_outputs:
                env=dict(os.environ,ZJU_RTE_DIAGNOSTIC=str(log));subprocess.run([str(runner),str(source),str(out),str(window),str(span)],env=env,check=True)
            a=np.genfromtxt(out,delimiter=',',names=True);a=a[a['elapsed_s']<=300];good=a['status']==1;first=float(a['elapsed_s'][good][0]) if good.any() else None
            if window==5 and span==10:
                previous=np.genfromtxt(ROOT/'data/rte_expanded_20261025'/f'{name}_output.csv',delimiter=',',names=True);previous=previous[previous['elapsed_s']<=300]
                for field in a.dtype.names:
                    if field!='solve_ms':np.testing.assert_allclose(a[field],previous[field],equal_nan=True,atol=1e-8,rtol=1e-10,err_msg=field)
            with log.open() as stream:stages=list(csv.DictReader(stream))
            counts={why:sum(r['reason']==why for r in stages) for why in sorted({r['reason'] for r in stages})}
            metrics=summarize(out,source.with_suffix('.truth.csv'),duration_s=300,motion_start_s=30)
            truth=np.genfromtxt(source.with_suffix('.truth.csv'),delimiter=',',names=True)
            mask=good & (a['elapsed_s']>=216.24)
            errors=[np.hypot(a[f'x_{n}']-np.interp(a['state_timestamp_ns'],truth['timestamp_ns'],truth[f'x_{n}']),a[f'y_{n}']-np.interp(a['state_timestamp_ns'],truth['timestamp_ns'],truth[f'y_{n}']))[mask] for n in (2,3)]
            row=dict(name=stem,window_s=window,minimum_motion_span_m=span,range_sigma_m=sigma,first_valid_s=first,diagnostic_stage_counts=counts,metrics=metrics,
                common_interval_s=[216.24,300],common_interval_valid_rows=int(mask.sum()),common_interval_rmse_m=float(np.sqrt(np.mean(np.array(errors)**2))) if mask.any() else None,baseline_reproduces_original=window==5 and span==10)
            records.append(row);(OUT/'summary.json').write_text(json.dumps(dict(conditions=records,elapsed_seconds=time.time()-began,original_algorithms_unchanged=True),ensure_ascii=False,indent=2)+'\n')
            print(stem,'首次有效',first,'RMSE',metrics.get('position_rmse_m'),counts,flush=True)
    paths=[Path(__file__),ROOT/'src/rte/three_vehicle_rte.cpp',ROOT/'tools/evaluate_rte.cpp']+list((OUT/'build').glob('*.cpp'))
    (OUT/'运行清单.json').write_text(json.dumps(dict(conditions=12,elapsed_seconds=time.time()-began,source_sha256={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}),ensure_ascii=False,indent=2)+'\n')
    with (OUT/'建系参数对照.csv').open('w',newline='') as f:
        w=csv.writer(f);w.writerow(['窗口_秒','运动范围下限_米','测距标准差_厘米','首次有效_秒','有效段RMSE_米','共同216.24至300秒RMSE_米'])
        for r in records:w.writerow([r['window_s'],r['minimum_motion_span_m'],r['range_sigma_m']*100,r['first_valid_s'],r['metrics']['position_rmse_m'],r['common_interval_rmse_m']])


if __name__=='__main__':main()
