# AM-DPBSURDF0422 Sim-only 数据报告

本报告包只使用 MuJoCo 仿真和 C/STM32 控制核心生成，不包含真机实验结论。

## 实验配置
- 模型: `AM-DPBSURDF0422.urdf`
- Monte Carlo 样本数: `500000`
- Monte Carlo 随机种子: `42`
- 闭环实验时长: `10.000 s`
- 固定步长: `0.001000 s`
- 闭环目标段数: `5`

## Monte Carlo 工作空间结果
| Arm | X span m | Y span m | Z span m | Safe box volume m^3 |
|---|---:|---:|---:|---:|
| left | 1.039280 | 0.891059 | 0.831387 | 0.152063810 |
| right | 1.042575 | 0.946452 | 0.941217 | 0.202168337 |

说明：`workspace_points.csv` 仅保留末端位置和四元数，不保存采样 qpos；关节采样边界和统计摘要保存在 `workspace_summary.json`。

## 闭环多目标阶跃结果
| Arm | Max error m | Mean error m | Final error m | Steady error m | Peak tau Nm | RMS tau Nm |
|---|---:|---:|---:|---:|---:|---:|
| left | 0.044565 | 0.013810 | 0.000092 | 0.000123 | 0.722 | 0.198 |
| right | 0.043113 | 0.011686 | 0.005499 | 0.006372 | 1.005 | 0.333 |

状态计数: `{"0": 10000}`

## 可复现性
- 数据包中的 CSV/JSON/SVG 均由同一次 `sim-report` 运行生成。
- 闭环实验使用固定 `Config.DT` 推进控制器时间，不依赖墙钟周期。
- 所有结论应表述为 sim-only 证据，不能替代真机验证。

## 文件索引
- `control_error_svg`: `results/sim_report_20260604_500k/control_error.svg`
- `control_loop_csv`: `results/sim_report_20260604_500k/control_loop.csv`
- `control_summary_json`: `results/sim_report_20260604_500k/control_summary.json`
- `control_torque_svg`: `results/sim_report_20260604_500k/control_torque.svg`
- `mc_terminal_summary_txt`: `results/sim_report_20260604_500k/mc_terminal_summary.txt`
- `report_md`: `results/sim_report_20260604_500k/report.md`
- `workspace_points_csv`: `results/sim_report_20260604_500k/workspace_points.csv`
- `workspace_span_svg`: `results/sim_report_20260604_500k/workspace_position_spans.svg`
- `workspace_summary_json`: `results/sim_report_20260604_500k/workspace_summary.json`
- `workspace_xy_svg`: `results/sim_report_20260604_500k/workspace_xy.svg`
- `workspace_xz_svg`: `results/sim_report_20260604_500k/workspace_xz.svg`
- `workspace_yz_svg`: `results/sim_report_20260604_500k/workspace_yz.svg`
