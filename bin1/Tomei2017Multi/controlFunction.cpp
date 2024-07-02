#include "controlFunction.h"
#include "logger/data_logging.hpp"

// 创建数据记录器实例
data_logger logger;

// 设置常数pi值
#define M_PI 3.1415926

// Tomei2017Multi 控制器构造函数，初始化控制器参数
Tomei2017Multi::Tomei2017Multi(int N_param, std::vector<double> freq_param, double SampleTime_param, std::vector<double> Iw_param, double g_param)
{   
    // 参数赋值
    N = N_param;
    freq.setZero(N);
    freq << Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(freq_param.data(), freq_param.size());
    SampleTime = SampleTime_param;
    g = g_param;

    // 初始化变量参数
    omega.setZero(N);
    for (int i = 0 ; i < N ; i++){
        omega(i) = std::cos(2*M_PI*freq(i)/SampleTime);
    }
    Iw.setZero(N+1);
    Iw << Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(Iw_param.data(),Iw_param.size());
    eta.setZero(1+2*N);
    eta_temp.setZero(eta.size());
}

// 计算控制输出
double Tomei2017Multi::Outputs()
{
    double u;
    u = - eta(0) -eta(1) -eta(3) -eta(5); // 控制输出计算
    return u;
}

// 更新控制器状态
void Tomei2017Multi::Update()
{
    
    for (int i = 0 ; i < eta.size() ; i++){
        eta_temp(i) = eta(i);
    }

    // 更新状态变量
    eta(0) = eta(0) + g*Iw(0)*y;
    for(int i=0; i<N;i++){
        eta(2*i+1) = eta_temp(2*i+2) + g*Iw(i+1)*y;
        eta(2*i+2) = -eta_temp(2*i+1) + 2*omega(i)*eta_temp(2*i+2) + 2*omega(i)*g*Iw(i+1)*y;
    }

    // 递增步数
    k += 1;
}

// 设置控制器输入
void Tomei2017Multi::setControllerInput(double controller_input)
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
    const std::vector<double> freq_param = config["freq_param"].as<std::vector<double>>();
    const double SampleTime_param = config["SampleTime_param"].as<double>();
    const std::vector<double> Iw_param = config["Iw_param"].as<std::vector<double>>();
    const double g_param = config["g_param"].as<double>();
    const int N_param = config["N_param"].as<double>();
    
    // 创建控制器实例
    Tomei2017Multi *ctrl_ptr = new Tomei2017Multi(N_param,freq_param,SampleTime_param,Iw_param,g_param);

    int sample_len = (config["run_time"].as<double>()) * config["sample_fs"].as<double>();

    // 初始化数据记录器
    logger.init({"k", "y", "u"}, sample_len, controller_log_path);       

    std::cout << "Controller log path: " << controller_log_path << std::endl;

    return (void *)(ctrl_ptr);
}

// 控制器计算函数
double controllerCompute(void *ctrl_ptr, double controller_input)
{
    Tomei2017Multi *ptr = (Tomei2017Multi *)(ctrl_ptr);
    double u;

    u = ptr->Outputs();// 计算控制输出
    ptr->setControllerInput(controller_input);// 设置控制器输入
    ptr->Update();// 更新控制器状态

    // 记录数据
    logger.log({double(ptr->k), ptr->y, u});

    // 返回控制输出
    return u;
}

// 控制器结束函数
void controllerFinish(void *ctrl_ptr)
{
    logger.write();// 写入日志
    Tomei2017Multi *ptr = (Tomei2017Multi *)(ctrl_ptr);
    delete ptr;// 删除控制器实例
}

//    ctrl_ptr->omega1 = 2 * M_PI * config["dist_freq"].as<double>() / 5000;        