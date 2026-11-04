#!/usr/bin/env python3
"""汇总全部重复运行；每行对应真实完成的一次计算，不将不同坐标指标混合。"""
import csv
import json
from collections import Counter
from evaluate_sensor_anchor_v14 import OUT


def main():
    main=json.loads((OUT/'summary.json').read_text())
    rte=json.loads((OUT/'rte_evaluation/summary.json').read_text())
    sensitivity=json.loads((OUT/'sensitivity/summary.json').read_text())
    rows=[]
    for section,source in [('设备与锚点',main),('精度敏感性',sensitivity)]:
        for case in source['conditions']:
            for k,result in enumerate(case['trials'],1):
                rows.append(dict(category=section,condition=case['name'],trial=k,profile=case['profile'],
                    frame=case.get('frame','field'),anchors=case['anchors'],range_sigma_m=case.get('range_sigma_m'),
                    position_rmse_m=result['position_rmse_m'],field_position_rmse_m=result.get('field_position_rmse_m'),
                    position_p95_m=result.get('position_p95_m'),position_max_m=result.get('position_max_m')))
    for result in rte['rte']:
        rows.append(dict(category='未知初始坐标RTE',condition=f'rte_{result["profile"]}_{result["range_sigma_m"]:.2f}',
            trial=result['trial'],profile=result['profile'],frame='reference_local',anchors=0,
            range_sigma_m=result['range_sigma_m'],position_rmse_m=result['position_rmse_m'],field_position_rmse_m=None,
            position_p95_m=None,position_max_m=None))
    counts=Counter(r['condition'] for r in rows)
    assert len(rows)==1400 and len(counts)==70 and set(counts.values())=={20},counts
    with (OUT/'全部仿真结果.csv').open('w',newline='') as stream:
        w=csv.DictWriter(stream,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    (OUT/'评估总清单.json').write_text(json.dumps(dict(conditions=len(counts),trial_count=len(rows),
        trials_per_condition=20,condition_counts=counts,categories=dict(Counter(r['category'] for r in rows)),
        scene=main['scene'],coordinate_metrics='triangle=规范相对队形；field=固定场地轴向相对向量；reference_local=RTE参考车局部轴'),ensure_ascii=False,indent=2)+'\n')
    print('总清单完成：70种条件，每种20次，共1400次，逐次结果已保存。')


if __name__=='__main__':main()
