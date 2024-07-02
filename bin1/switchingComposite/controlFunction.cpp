#include "controlFunction.h"
#include "logger/data_logging.hpp"

// 创建数据记录器实例
data_logger logger;

// switchingComposite 控制器构造函数，初始化控制器参数
switchingComposite::switchingComposite(double epsilon_param, double alpha_param, std::vector<double> gamma_param, double h_param, std::vector<double> omega_star_param, std::vector<double> thetaHatSigma_param, std::vector<double> cntrlr_gain_param, int N_param)
{
    // 参数赋值
    epsilon = epsilon_param;
    alpha = alpha_param;
    h = h_param;
    N = N_param;

    // 初始化矩阵
    gamma.setZero(N);
    gamma << Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(gamma_param.data(), gamma_param.size());
    cntrlr_gain.setZero(N);
    cntrlr_gain << Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(cntrlr_gain_param.data(), cntrlr_gain_param.size());
    omega_star.setZero(N);
    omega_star << Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(omega_star_param.data(), omega_star_param.size());
    for (int i = 0; i < N; i++)
    {
        omega_star(i) = omega_star(i) / 5000;
    }
    Eigen::Vector2d g;
    g << 1, 0;
    G.setZero(2 * N, N);
    G << blkdiag(g, N);
    Gamma.setZero(2 * N);
    Gamma << g.transpose().replicate(1, N);
    S.setZero(2 * N, 2 * N);
    for (int i = 0; i < N; i++)
    {
        S.block(i * 2, i * 2, 2, 2) << cos(omega_star(i)), sin(omega_star(i)), -sin(omega_star(i)), cos(omega_star(i));
    }
    F.setZero(2 * N, 2 * N);
    F << S - alpha * Gamma.transpose() * Gamma;

    // 初始化状态变量
    k = 0;
    y = 0.0;

   // 初始化矩阵
    thetaHatSigma.setZero(2 * N);
    thetaHatSigma << Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(thetaHatSigma_param.data(), thetaHatSigma_param.size());
    std::cout << "thetaHatSigma:\n"
              << thetaHatSigma << "\n";
    zetaHat.setZero(2 * N);
    xi1Hat.setZero(2 * N);
    eta.setZero(2 * N);
    eta << thetaHatSigma;
    wHat.setZero(2 * N);
}

// 计算控制输出
double switchingComposite::Outputs()
{
    double u;
    u = Gamma * wHat.cwiseProduct(repelem(cntrlr_gain, 2, 1)); // 控制输出计算
    return u;
}

// 更新控制器状态
void switchingComposite::Update()
{
    Eigen::VectorXd uaSigma;

    // 模型更新规律
    uaSigma.setZero(N);
    uaSigma << -epsilon * thetaHatSigma.cwiseProduct(zetaHat).reshaped(2, N).colwise().sum().transpose();
    zetaHat << S * zetaHat + thetaHatSigma.cwiseProduct(repelem(uaSigma, 2, 1)) - alpha * Gamma.transpose() * (Gamma * zetaHat - y);
    xi1Hat << F.transpose() * xi1Hat + G * uaSigma;
    eta << eta - xi1Hat.cwiseProduct(repelem(gamma, 2, 1)) * (Gamma * zetaHat - y - (thetaHatSigma - eta).transpose() * xi1Hat);

    for (int i = 0; i < N; i++)
    {
        if ((eta.segment(2 * i, 2) - thetaHatSigma.segment(2 * i, 2)).norm() >= h)
        {
            thetaHatSigma.segment(2 * i, 2) << eta.segment(2 * i, 2);
        }
    }

    // 参数更新律
    wHat << S * wHat + G * uaSigma;

    k += 1; // 递增步数
}

// 设置控制器输入
void switchingComposite::setControllerInput(double controller_input)
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
    std::vector<double> omega_star_param = config["dist_freq"].as<std::vector<double>>();
    for (auto omega_star_param_iter = omega_star_param.begin(); omega_star_param_iter != omega_star_param.end(); omega_star_param_iter++)
    {
        *omega_star_param_iter = *omega_star_param_iter * M_PI * 2;
    }
    const double epsilon_param = config["epsilon_param"].as<double>();
    const double alpha_param = config["alpha_param"].as<double>();
    const std::vector<double> gamma_param = config["gamma_param"].as<std::vector<double>>();
    const double h_param = config["h_param"].as<double>();
    const std::vector<double> cntrlr_gain_param = config["cntrlr_gain_param"].as<std::vector<double>>();
    const int N_param = config["N_param"].as<int>();
    const std::vector<double> thetaHatSigma_param = config["thetaHatSigma_param"].as<std::vector<double>>();

    // 创建控制器实例
    switchingComposite *ctrl_ptr = new switchingComposite(epsilon_param, alpha_param, gamma_param, h_param, omega_star_param, thetaHatSigma_param, cntrlr_gain_param, N_param);

    int sample_len = (config["run_time"].as<double>()) * config["sample_fs"].as<double>();

    // 初始化数据记录器
    logger.init({"k", "y", "u", "eta11", "eta12", "eta21", "eta22", "eta31", "eta32", "eta41", "eta42", "eta51", "eta52"}, sample_len, controller_log_path);

    std::cout << "Controller log path: " << controller_log_path << std::endl;

    return (void *)(ctrl_ptr);
}

// 控制器计算函数
double controllerCompute(void *ctrl_ptr, double controller_input)
{
    switchingComposite *ptr = (switchingComposite *)(ctrl_ptr);
    double u;

    u = ptr->Outputs(); // 计算控制输出
    ptr->setControllerInput(controller_input); // 设置控制器输入
    ptr->Update(); // 更新控制器状态

    // 记录数据
    logger.log({double(ptr->k), controller_input, u, ptr->eta(0), ptr->eta(1), ptr->eta(2), ptr->eta(3), ptr->eta(4), ptr->eta(5), ptr->eta(6), ptr->eta(7), ptr->eta(8), ptr->eta(9)});

    return u; // 返回控制输出
}

// 控制器结束函数
void controllerFinish(void *ctrl_ptr)
{
    logger.write();  // 写入日志
    switchingComposite *ptr = (switchingComposite *)(ctrl_ptr);
    delete ptr; // 删除控制器实例
}