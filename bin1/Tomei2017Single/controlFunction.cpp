#include "controlFunction.h"
#include "logger/data_logging.hpp"

// 创建数据记录器实例
data_logger logger;

//定义常数pi
#define M_PI 3.1415926

// Tomei2017DiscreteJiChenyang 控制器构造函数，初始化控制器参数
Tomei2017DiscreteJiChenyang::Tomei2017DiscreteJiChenyang(double freq_param, double SampleTime_param, double Iw_param, double g_param)
{
    // 参数赋值
    freq = freq_param;
    SampleTime = SampleTime_param;
    omega = 2*M_PI*freq*SampleTime;
    Iw = Iw_param;
    g = g_param;

    // 初始化变量参数
    eta_1 = 0;
    eta_2 = 0;

    // 初始化变量参数
    omega_cos = cos(omega);
}

// 计算控制输出
double Tomei2017DiscreteJiChenyang::Outputs()
{
    double u;
    u = -eta_1;// 控制输出计算
    return u;
}

// 更新控制器状态
void Tomei2017DiscreteJiChenyang::Update()
{
    // 更新状态变量
    double eta_1_1 = eta_1;
    eta_1 = eta_2 +g*Iw*y;
    eta_2 = -eta_1_1 + 2*omega_cos*eta_2+2*omega_cos*g*Iw*y;

    k += 1;// 递增步数
}

// 设置控制器输入
void Tomei2017DiscreteJiChenyang::setControllerInput(double controller_input)
{
    y = controller_input;
}

// 控制器初始化函数
void *controllerInit(int argc, char *argv[])
{
    // 从命令行参数或默认路径加载配置文件
    std::string config_path = "config.yaml";
    if (argc >= 3)
    {
        config_path = std::string(argv[2]);
    }
    else if (argc == 2)
    {
        config_path = std::string(argv[1]);
    }

    YAML::Node config = YAML::LoadFile(config_path);

    // 从配置文件中读取参数
    const std::string log_folder = config["log_folder"].as<std::string>();
    const std::string controller_log_path = log_folder + '/' + config["controller_log_path"].as<std::string>();
    const double freq_param = config["freq_param"].as<double>();
    const double SampleTime_param = config["SampleTime_param"].as<double>();
    const double Iw_param = config["Iw_param"].as<double>();
    const double g_param = config["g_param"].as<double>();

    // 创建控制器实例
    Tomei2017DiscreteJiChenyang *ctrl_ptr = new Tomei2017DiscreteJiChenyang(freq_param,SampleTime_param,Iw_param,g_param);

    int sample_len = (config["run_time"].as<double>()) * config["sample_fs"].as<double>();

    // 初始化数据记录器
    logger.init({"k", "y", "u", "eta_1", "eta_2"}, sample_len, controller_log_path);       

    std::cout << "Controller log path: " << controller_log_path << std::endl;

    return (void *)(ctrl_ptr);
}

// 控制器计算函数
double controllerCompute(void *ctrl_ptr, double controller_input)
{
    Tomei2017DiscreteJiChenyang *ptr = (Tomei2017DiscreteJiChenyang *)(ctrl_ptr);
    double u;

    u = ptr->Outputs();// 计算控制输出
    ptr->setControllerInput(controller_input);// 设置控制器输入
    ptr->Update();// 更新控制器状态

    // 记录数据
    logger.log({double(ptr->k), ptr->y,  u, ptr->eta_1, ptr->eta_2});

    return u;// 返回控制输出
}

// 控制器结束函数
void controllerFinish(void *ctrl_ptr)
{
    logger.write();// 写入日志
    Tomei2017DiscreteJiChenyang *ptr = (Tomei2017DiscreteJiChenyang *)(ctrl_ptr);
    delete ptr;// 删除控制器实例
}
