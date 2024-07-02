修改时间：2024.06.14——创建本readme文件

1. 算法名称
不确定离散线性系统的多正弦干扰抑制

2. 算法简介
本算法针对确定的单输入单输出离散时间线性系统。本算法的扰动频率已知，且为单一频率正弦信号。
该方案消除了系统的类本质严格正实数条件，放宽了频率响应的前提知识。

3. 算法对应的文章
Jiangkun Xu, Yizhou Gong, Yang Wang，Multi-sinusoidal Disturbance Rejection for Uncertain Discrete-Time Linear Systems


4. 文件说明
controlFunction.cpp--C++源文件
controlFunction.h--C++头文件
multiDiscreteSeparate2.pdf--算法文章原文
multiDiscreteSeparate2xxx.yaml--参数配置文件
multiDiscreteSeparate2_hardware--可执行文件（设备）
multiDiscreteSeparate2_simulation可执行文件（仿真）

6. multiDiscreteSeparate2XXX.yaml说明
omega_star_param:
  - 137.0
  - 411.0  --频率参考值
vep_param: 0.05  --epsilon参数值
rho_param: 0.5 --rho参数值
beta_param: 1.0  --beta参数值
h_param: 0.01  --迟滞参数h值
swi_sig_param:
  - 1
  - 1 --swi_sig值
u_gain_param: 0.005 --设备增益参数值
sample_fs: 5000.0 --采样频率
dist_freq: 137.0 --干扰频率
dist_gain: 0.8 --干扰幅值
run_time: 30.0 --运行时间
warm_up: 5.0 --开环运行时间
start_wait: 2.0 --预热时间
analog_read_bias: -1.549 --偏置补偿项（固定值）
analog_read_gain: 8000 --数据读取固定值
log_folder: data_icra/multiDiscreteSeparate137 --数据存放地址
device_log_path: device.bin --设备数据存放地址
controller_log_path: cntrlr.bin --算法数据存放地址

7. 源码步骤
对应代码说明在controlFunction.cpp中说明

8. 对应文章公式在multiDiscreteSeparate.pdf中高亮表示