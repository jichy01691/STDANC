修改时间：2024.06.14——创建本readme文件

1. 算法名称
离散时间不确定稳定系统的多正弦扰动抑制

2. 算法简介
本算法针对确定的单输入单输出离散时间线性系统。本算法的扰动频率已知，且为多频率正弦信号。
同时该算法需要知道传递函数在扰动频率处的实部符号或者虚部符号。

3. 算法对应的文章
Tomei P. Multi-sinusoidal disturbance rejection for discrete-time uncertain stable systems[J]. Automatica, 2017, 79: 144-151.

4. 文件说明
controlFunction.cpp--C++源文件
controlFunction.h--C++头文件
Tomei2017Multi.pdf--算法文章原文
Tomei2017Multixxx.yaml--参数配置文件
Tomei2017Multi_hardware--可执行文件（设备）
Tomei2017Multi_simulation可执行文件（仿真）

6. Tomei2017MultiXXX.yaml说明
freq_param:
  - 100.0
  - 200.0
  - 300.0 --干扰频率参数值
SampleTime_param: 5000.0 --采样频率参数值
Iw_param: 
  - 1
  - 1
  - -1
  - -1 --传递函数在扰动频率处的虚部符号值
g_param: 0.0001 --g参数值
N_param: 3 --扰动频率数目
sample_fs: 5000.0 --采样频率
dist_freq: 
  - 100.0
  - 200.0
  - 300.0 --干扰频率
dist_gain:
  - 0.25
  - 0.25
  - 0.25 --干扰幅值
run_time: 10.0 --运行时间
warm_up: 5.0 --开环运行时间
start_wait: 2.0 --预热时间
analog_read_bias: -1.549 --偏置补偿项（固定值）
analog_read_gain: 8000 --数据读取固定值
log_folder: data/Tomei2017Multi --数据存放地址
device_log_path: device.bin --设备数据存放地址
controller_log_path: cntrlr.bin --算法数据存放地址

7. 源码步骤
对应代码说明在controlFunction.cpp中说明

8. 对应文章公式在Tomei2017Multi.pdf中高亮表示