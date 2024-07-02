修改时间：2024.06.14——创建本readme文件

1. 算法名称
基于开关的未知路径周期信号自适应有源噪声控制

2. 算法简介
本算法针对确定的单输入单输出离散时间线性系统。本算法的扰动频率已知，且为多频率正弦信号。
且本算法不需要先验的路径模型知识

3. 算法对应的文章
He G, Wang Y, Pin G, et al. Switching-based adaptive output regulation for uncertain systems affected by a periodic disturbance[C]//2022 American Control Conference (ACC). IEEE, 2022: 5030-5036.

4. 文件说明
controlFunction.cpp--C++源文件
controlFunction.h--C++头文件
switchingComposite.pdf--算法文章原文
switchingCompositexxx.yaml--参数配置文件
switchingComposite_hardware--可执行文件（设备）
switchingComposite_simulation可执行文件（仿真）

6. switchingCompositeXXX.yaml说明
epsilon_param: 0.05 --epsilon参数值
alpha_param: 0.05 --alpha参数值
gamma_param:
  - 0.5
  - 0.5
  - 0.5
  - 0.5
  - 0.5  --gamma参数值
h_param: 0.3  --h参数值
cntrlr_gain_param:
  - 0.1
  - 0.1
  - 0.1
  - 0.1
  - 0.1 --控制信号增益值
N_param: 5 --干扰信号数目
thetaHatSigma_param:
  - 1.0
  - 0.5
  - 0.9
  - 0.5
  - -0.4
  - 0.51
  - 1.3
  - 0.5
  - -0.4
  - 0.6  --sigma参数值
sample_fs: 5000.0  --采样频率
dist_freq:
  - 67.0
  - 167.0
  - 267.0
  - 367.0
  - 467.0  --干扰频率
dist_gain:
  - 1
  - 1
  - 1
  - 1
  - 1  --干扰幅值
run_time: 100.0  --运行时间
warm_up: 5.0  --开环运行时间
start_wait: 2.0  --预热时间
analog_read_bias: -1.549  --偏置补偿项（固定值）
analog_read_gain: 8000  --数据读取固定值
log_folder: data_composite/switchingComposite  --数据存放地址
device_log_path: device.bin  --设备数据存放地址
controller_log_path: cntrlr.bin  --算法数据存放地址

7. 源码步骤
对应代码说明在controlFunction.cpp中说明


8. 对应文章公式在switchingComposite.pdf中高亮表示