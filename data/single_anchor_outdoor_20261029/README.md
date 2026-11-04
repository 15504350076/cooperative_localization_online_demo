# 室外误差项审查后的单锚点仿真

最新输入和输出：一个已知位置UWB点，三条车间距离、三条车锚距离；84种条件、每种20次，共1680次。原始数据未覆盖，旧模型在data/single_anchor_20261029。

```bash
PYTHONPATH=tools OPENBLAS_NUM_THREADS=1 python3 -u tools/evaluate_single_anchor.py --trials 20
PYTHONPATH=tools python3 tools/audit_outdoor_assumptions.py
PYTHONPATH=tools python3 tools/check_single_anchor.py
PYTHONPATH=tools python3 tools/plot_single_anchor.py
```

运行日志.txt逐项输出条件及结果，运行清单.json保存入口、库和全部输入输出SHA。全部仿真结果.csv为1680次逐次结果，蒙特卡洛分组统计.csv为84行组均值、样本标准差、范围及小于1米次数。summary.json保存原始数值、模型设定及540次初始化的秩和收敛记录。

参数与可观性核对保存原始开源配置快照、15份本项目静止统计、参数用途分类及45度共同旋转验证。模型与单位参考不等于器件精度标定；VIO/LIO是显式局部位移输出模型，没有运行图像或点云前端。陀螺启动残余不是零偏稳定性，偏置游走按sqrt(dt)注入，重力泄漏按9.80665乘倾角弧度注入。

UWB逐时白噪声标准差20/10/5厘米，另含每条链路固定偏差标准差3厘米。不同分支配对使用同一批输入。初值仅由0～60秒运动和六条距离求解，60～300秒评价。真值不进入估计，不事后轨迹配准；完整20次位置在outputs，测量在inputs。局部轴为锚点原点、初始点至201车为正X，不代表观测到地图方向。单点位置CSV导出LIO增强10厘米重复1，航向有效位false。

独立结果核对.json复算1680次RMSE并重放三种输入；7项测试覆盖无噪声初始化、测量更新、相对误差、静止不可观、整体旋转不变及随机游走sqrt(dt)换算。报告正文按各类输入把条件、统计和图线紧接编排，详细依据在报告配套参数来源_可观性_初始化核对.md。
