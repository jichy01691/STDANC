*************修改时间：2024.06.14*******************************
1. 算法名称
离散时间自适应前馈控制单一频率（IFAC 2023）

2. 算法简介
本方法针对不确定的单输入
 单输出（SISO）离散时间线性系统。单一频率，只有频率信息是已知的，这是一种基于自适应的新策略
 开发了前馈控制（AFC）。与现有方法相比，不需要已知传递函数在激励频率处的频响符号。

3. 算法对应的文章
Xu J, Liu S, Jia J, et al. Robust Output Regulation for Uncertain Discrete-Time Linear Systems under the Effect of a Sinusoidal Disturbance[J]. IFAC-PapersOnLine, 2023, 56(2): 2895-2902.


4. 文件说明
controlFunction.cpp--C++源文件
controlFunction.h--C++头文件
singleDiscrete.pdf--算法文章原文
singleDiscretexxx.yaml--参数配置文件
singleDiscrete_hardware--可执行文件（设备）
singleDiscrete_simulation可执行文件（仿真）

6. singleDiscretexxx.yaml说明
vep_param: 0.1--epsilon参数值
rho_param: 0.5--rho参数值
a_param: 1--初始切换序号
beta_param: 1.0--beta参数值
h_param: 0.1--迟滞参数h值
gain_param: 0.1--设备增益参数值
sample_fs: 5000.0--采样频率
dist_freq: 137.0--干扰频率
dist_gain: 0.8--干扰幅值
run_time: 20.0--运行时间
warm_up: 5.0--开环运行时间
start_wait: 2.0--预热时间
analog_read_bias: -1.549--偏置补偿项（固定值）
analog_read_gain: 8000--数据读取固定值
log_folder: data/singleDiscrete137--数据存放地址
device_log_path: device.bin--设备数据存放地址
controller_log_path: cntrlr.bin--算法数据存放地址

7.源码解释
对应代码说明在controlFunction.cpp中说明


8. 对应文章公式亦在PDF中高亮表示
