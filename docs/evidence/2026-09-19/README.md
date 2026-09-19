# 原始实测波形索引

图像来自随附 raw 快照，SHA256 见 manifest.json。未额外平滑、删异常点或补采样。
v114之前为C610原始速度字段；v114之后为固件实际闭环所用角度差分速度，二者不能当成同一测量方式直接比较噪声。
短时中断段不是完整稳态试验；统计仅辅助阅读，不自动判定验收通过。

|版本/段|目标rpm|记录非零目标时长s|左均值/标准差|右均值/标准差|
|---|---:|---:|---:|---:|
|[v107_trial_01](v107_trial_01.png)|10|2.60|9.09/4.95|9.21/5.78|
|[v108_trial_01](v108_trial_01.png)|10|2.80|8.62/2.41|6.90/3.85|
|[v109_trial_01](v109_trial_01.png)|10|5.90|9.78/1.43|10.64/1.58|
|[v110_trial_01](v110_trial_01.png)|10|5.80|10.25/1.75|10.05/1.37|
|[v112_trial_01](v112_trial_01.png)|10|5.90|10.29/1.61|9.95/1.43|
|[v114_trial_01](v114_trial_01.png)|10|5.70|9.98/0.34|10.07/0.53|
|[v114_trial_02](v114_trial_02.png)|10|5.70|10.01/0.34|9.99/0.48|
|[v114_trial_03](v114_trial_03.png)|5|5.80|4.93/0.43|1.62/4.36|
|[v114_trial_04](v114_trial_04.png)|5|5.90|4.88/0.42|5.02/0.38|
|[v114_trial_05](v114_trial_05.png)|5|5.80|4.67/1.86|5.04/0.42|
|[v115_trial_01](v115_trial_01.png)|5|5.80|4.99/0.46|5.02/0.35|
|[v116_trial_01](v116_trial_01.png)|5|2.10|2.41/2.56|0.53/1.46|
|[v116_trial_02](v116_trial_02.png)|5|5.80|4.96/0.41|4.98/0.44|
|[v116_trial_03](v116_trial_03.png)|5|5.80|5.16/0.41|5.19/0.44|
|[v116_trial_04](v116_trial_04.png)|5|5.80|4.86/0.36|4.64/0.56|
|[v116_trial_05](v116_trial_05.png)|10|5.80|10.01/0.39|10.02/0.31|
|[v117_trial_01](v117_trial_01.png)|10|5.70|9.99/0.40|10.10/0.56|
|[v117_trial_02](v117_trial_02.png)|10|5.80|9.97/0.30|10.02/0.31|
|[v117_trial_03](v117_trial_03.png)|10|5.80|10.04/0.45|9.81/0.35|
|[v117_trial_04](v117_trial_04.png)|5|5.60|5.09/0.31|4.89/0.35|
|[v117_trial_05](v117_trial_05.png)|5|5.80|4.97/0.45|4.95/0.45|

## comparison_5rpm_v114_v117

![comparison_5rpm_v114_v117](comparison_5rpm_v114_v117.png)
[SVG矢量图](comparison_5rpm_v114_v117.svg)

## v107_trial_01

![v107_trial_01](v107_trial_01.png)
[SVG矢量图](v107_trial_01.svg)

## v108_trial_01

![v108_trial_01](v108_trial_01.png)
[SVG矢量图](v108_trial_01.svg)

## v109_trial_01

![v109_trial_01](v109_trial_01.png)
[SVG矢量图](v109_trial_01.svg)

## v110_trial_01

![v110_trial_01](v110_trial_01.png)
[SVG矢量图](v110_trial_01.svg)

## v111_control_trace

![v111_control_trace](v111_control_trace.png)
[SVG矢量图](v111_control_trace.svg)

## v112_control_trace

![v112_control_trace](v112_control_trace.png)
[SVG矢量图](v112_control_trace.svg)

## v112_trial_01

![v112_trial_01](v112_trial_01.png)
[SVG矢量图](v112_trial_01.svg)

## v113_control_trace

![v113_control_trace](v113_control_trace.png)
[SVG矢量图](v113_control_trace.svg)

## v114_control_trace

![v114_control_trace](v114_control_trace.png)
[SVG矢量图](v114_control_trace.svg)

## v114_trial_01

![v114_trial_01](v114_trial_01.png)
[SVG矢量图](v114_trial_01.svg)

## v114_trial_02

![v114_trial_02](v114_trial_02.png)
[SVG矢量图](v114_trial_02.svg)

## v114_trial_03

![v114_trial_03](v114_trial_03.png)
[SVG矢量图](v114_trial_03.svg)

## v114_trial_04

![v114_trial_04](v114_trial_04.png)
[SVG矢量图](v114_trial_04.svg)

## v114_trial_05

![v114_trial_05](v114_trial_05.png)
[SVG矢量图](v114_trial_05.svg)

## v115_trial_01

![v115_trial_01](v115_trial_01.png)
[SVG矢量图](v115_trial_01.svg)

## v116_trial_01

![v116_trial_01](v116_trial_01.png)
[SVG矢量图](v116_trial_01.svg)

## v116_trial_02

![v116_trial_02](v116_trial_02.png)
[SVG矢量图](v116_trial_02.svg)

## v116_trial_03

![v116_trial_03](v116_trial_03.png)
[SVG矢量图](v116_trial_03.svg)

## v116_trial_04

![v116_trial_04](v116_trial_04.png)
[SVG矢量图](v116_trial_04.svg)

## v116_trial_05

![v116_trial_05](v116_trial_05.png)
[SVG矢量图](v116_trial_05.svg)

## v117_fault_1

![v117_fault_1](v117_fault_1.png)
[SVG矢量图](v117_fault_1.svg)

## v117_trial_01

![v117_trial_01](v117_trial_01.png)
[SVG矢量图](v117_trial_01.svg)

## v117_trial_02

![v117_trial_02](v117_trial_02.png)
[SVG矢量图](v117_trial_02.svg)

## v117_trial_03

![v117_trial_03](v117_trial_03.png)
[SVG矢量图](v117_trial_03.svg)

## v117_trial_04

![v117_trial_04](v117_trial_04.png)
[SVG矢量图](v117_trial_04.svg)

## v117_trial_05

![v117_trial_05](v117_trial_05.png)
[SVG矢量图](v117_trial_05.svg)

## v117_UART故障观察

![v117_UART故障观察](v117_UART故障观察.png)
[SVG矢量图](v117_UART故障观察.svg)

## v118_一分钟零速观察

![v118_一分钟零速观察](v118_一分钟零速观察.png)
[SVG矢量图](v118_一分钟零速观察.svg)
