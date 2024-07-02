#include "controlFunction.h"
#include "logger/data_logging.hpp"

data_logger logger;

// switchingSingle 控制器构造函数，初始化控制器参数
switchingSingle::switchingSingle(double epsilon_param, double alpha_param, double gamma_param, double h_param, double omega_star_param, std::vector<double> thetaHatSigma_param, double cntrlr_gain_param)
{
    // 参数赋值
    epsilon = epsilon_param;
    alpha = alpha_param;
    gamma = gamma_param;
    h = h_param;
    cntrlr_gain = cntrlr_gain_param;

    // 初始化常量参数
    omega_star = omega_star_param / 5000;
    G << 1, 0;
    Gamma << 1, 0;
    S << cos(omega_star), sin(omega_star),
        -sin(omega_star), cos(omega_star);
    F << S - alpha * G * Gamma;

    // 初始化状态变量
    k = 0;
    y = 0.0;

    // 初始化矩阵
    thetaHatSigma << Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(thetaHatSigma_param.data(), thetaHatSigma_param.size());

    // 初始化估计变量
    x.setZero(8);
    x.segment(4, 2) << thetaHatSigma; // etaInit
}

// 计算控制输出
double switchingSingle::Outputs()
{
    double u;
    Eigen::Vector2d wHat;
    wHat << x.tail(2);
    u = cntrlr_gain * Gamma * wHat;  // 控制输出计算
    return u;
}

// 设置控制器输入
void switchingSingle::setControllerInput(double controller_input)
{
    y = controller_input;
}

// 更新控制器状态
void switchingSingle::Update()
{
    Eigen::Vector2d zetaHat, xi1Hat, eta, wHat, phi;
    double uaSigma;

    // 提取状态变量
    zetaHat << x.head(2);
    xi1Hat << x.segment(2, 2);
    eta << x.segment(4, 2);
    wHat << x.tail(2);

    // 模型更新规律
    uaSigma = -epsilon * thetaHatSigma.transpose() * zetaHat;
    zetaHat << S * zetaHat + thetaHatSigma * uaSigma - alpha * G * (Gamma * zetaHat - y);
    xi1Hat << F.transpose() * xi1Hat + G * uaSigma;

    phi << -gamma * xi1Hat * (Gamma * zetaHat - y - (thetaHatSigma - eta).transpose() * xi1Hat);
    eta << eta + phi;
    if ((eta - thetaHatSigma).norm() > h)
    {
        thetaHatSigma << eta; 
    }

    // 参数更新律
    wHat << S * wHat + G * uaSigma;

    // 更新状态变量
    x << zetaHat, xi1Hat, eta, wHat;

    k += 1;// 递增步数
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
    const double omega_star_param = config["dist_freq"].as<double>() * M_PI * 2;
    const double epsilon_param = config["epsilon_param"].as<double>();
    const double alpha_param = config["alpha_param"].as<double>();
    const double gamma_param = config["gamma_param"].as<double>();
    const double h_param = config["h_param"].as<double>();
    const double cntrlr_gain_param = config["cntrlr_gain_param"].as<double>();
    const std::vector<double> thetaHatSigma_param = config["thetaHatSigma_param"].as<std::vector<double>>();

    // 创建控制器实例
    switchingSingle *ctrl_ptr = new switchingSingle(epsilon_param, alpha_param, gamma_param, h_param, omega_star_param, thetaHatSigma_param, cntrlr_gain_param);

    int sample_len = (config["run_time"].as<double>()) * config["sample_fs"].as<double>();

    // 初始化数据记录器
    logger.init({"k", "y", "u", "eta1", "eta2", "zetaHat1", "zetaHat2"}, sample_len, controller_log_path);

    std::cout << "Controller log path: " << controller_log_path << std::endl;

    return (void *)(ctrl_ptr);
}

// 控制器计算函数
double controllerCompute(void *ctrl_ptr, double controller_input)
{
    switchingSingle *ptr = (switchingSingle *)(ctrl_ptr);
    double u;

    u = ptr->Outputs(); // 计算控制输出
    ptr->setControllerInput(controller_input); // 设置控制器输入
    ptr->Update(); // 更新控制器状态

    // 记录数据
    logger.log({double(ptr->k), ptr->y, u, ptr->x(4), ptr->x(5), ptr->x(0), ptr->x(1)});

    return u; // 返回控制输出
}

// 控制器结束函数
void controllerFinish(void *ctrl_ptr)
{
    logger.write(); // 写入日志
    switchingSingle *ptr = (switchingSingle *)(ctrl_ptr);
    delete ptr; // 删除控制器实例
}