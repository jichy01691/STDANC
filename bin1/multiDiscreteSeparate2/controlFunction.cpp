#include "controlFunction.h"
#include "logger/data_logging.hpp"

// 创建数据记录器实例
data_logger logger;

// multiDiscreteSeparate 控制器构造函数，初始化控制器参数
multiDiscreteSeparate::multiDiscreteSeparate(std::vector<double> omega_star_param, double vep_param, double rho_param, double beta_param, double h_param, std::vector<int> swi_sig_param, double u_gain_param)
{
    // 参数赋值
    vep = vep_param;
    rho = rho_param;
    beta = beta_param;
    h = h_param;
    u_gain = u_gain_param;

    // 初始化矩阵
    Eigen::VectorXd omega_star = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(omega_star_param.data(), omega_star_param.size());
    omega_star1 = omega_star(0) / 5000;
    omega_star2 = omega_star(1) / 5000;

    G << 1, 0;
    Gamma << 1, 0;

    blkdiag_Gamma.setZero(3, 6);
    blkdiag_Gamma << blkdiag(Gamma, 3);
    blkdiag_G.setZero(6, 3);
    blkdiag_G << blkdiag_Gamma.transpose();

    S1 << cos(omega_star1), sin(omega_star1),
        -sin(omega_star1), cos(omega_star1);
    S2 << cos(omega_star2), sin(omega_star2),
        -sin(omega_star2), cos(omega_star2);

    blkdiag_S1.setZero(6, 6);
    blkdiag_S1 << blkdiag(S1, 3);
    blkdiag_S2.setZero(6, 6);
    blkdiag_S2 << blkdiag(S2, 3);

    F1 << S1 - vep * G * Gamma;
    F2 << S2 - vep * G * Gamma;

    // 初始化状态变量
    k = 0;
    y = 0.0;

    // 初始化估计变量
    Eigen::VectorXi swi_sig = Eigen::Map<Eigen::VectorXi, Eigen::Unaligned>(swi_sig_param.data(), swi_sig_param.size());
    a1 = swi_sig(0);
    a2 = swi_sig(1);

    Eigen::VectorXd chi_hat_init(6);
    chi_hat_init << sqrt(3.0) / 2.0, 1.0 / 2.0, -sqrt(3.0) / 2.0, 1.0 / 2.0, 0.0, -1.0;

    x1.setZero(19);
    x1.segment(6, 6) << chi_hat_init;
    x2.setZero(19);
    x2.segment(6, 6) << chi_hat_init;
}

// 计算控制输出
double multiDiscreteSeparate::Outputs()
{
    double u;
    u = u_gain * Gamma * (x1.tail(2) + x2.tail(2)); // 控制输出计算
    return u;
}

// 更新控制器状态
void multiDiscreteSeparate::Update()
{
    Eigen::Vector2d xi1_hat1, v_hat1, xi1_hat2, v_hat2;

    Eigen::Vector3d J1, e1, y1_hat1, y_til1, m_sr1;
    Eigen::Vector3d J2, e2, y1_hat2, y_til2, m_sr2;

    Eigen::VectorXd zeta_hat1(6), chi_hat1(6), zeta_hat2(6), chi_hat2(6);

    double e_a1, e_a2;
    Eigen::Index min_ind1, min_ind2;

    // 提取状态变量
    zeta_hat1 << x1.segment(0, 6);
    chi_hat1 << x1.segment(6, 6);
    xi1_hat1 << x1.segment(12, 2);
    J1 << x1.segment(14, 3);
    v_hat1 << x1.segment(17, 2);

    zeta_hat2 << x2.segment(0, 6);
    chi_hat2 << x2.segment(6, 6);
    xi1_hat2 << x2.segment(12, 2);
    J2 << x2.segment(14, 3);
    v_hat2 << x2.segment(17, 2);

    // 模型估计器和切换控制器
    e1 << -vep * chi_hat1.cwiseProduct(zeta_hat1).reshaped(2, 3).colwise().sum().transpose();
    e_a1 = e1(a1);
    e2 << -vep * chi_hat2.cwiseProduct(zeta_hat2).reshaped(2, 3).colwise().sum().transpose();
    e_a2 = e2(a2);

    y1_hat1 << blkdiag_Gamma * zeta_hat1;
    y1_hat2 << blkdiag_Gamma * zeta_hat2;

    y_til1 << y1_hat1.array() - y;
    y_til2 << y1_hat2.array() - y;

    zeta_hat1 << blkdiag_S1 * zeta_hat1 + chi_hat1 * e_a1 - vep * blkdiag_G * y_til1;
    zeta_hat2 << blkdiag_S2 * zeta_hat2 + chi_hat2 * e_a2 - vep * blkdiag_G * y_til2;

    // 模型更新规律
    m_sr1 << 1 + xi1_hat1.squaredNorm() + y_til1.array().square();
    m_sr2 << 1 + xi1_hat2.squaredNorm() + y_til2.array().square();

    chi_hat1 << chi_hat1 - rho * pow(vep, 2) * xi1_hat1.replicate(3, 1).cwiseProduct(repelem(y_til1, 2, 1)).cwiseQuotient(repelem(m_sr1, 2, 1));
    chi_hat2 << chi_hat2 - rho * pow(vep, 2) * xi1_hat2.replicate(3, 1).cwiseProduct(repelem(y_til2, 2, 1)).cwiseQuotient(repelem(m_sr2, 2, 1));

    xi1_hat1 << F1.transpose() * xi1_hat1 + G * e_a1;
    xi1_hat2 << F2.transpose() * xi1_hat2 + G * e_a2;

    // 滞后开关逻辑
    J1 << J1 + beta * y_til1.array().square().matrix();
    J2 << J2 + beta * y_til2.array().square().matrix();

    // 确定性-等价估计器
    v_hat1 << S1 * v_hat1 + G * e_a1;
    v_hat2 << S2 * v_hat2 + G * e_a2;

    x1 << zeta_hat1, chi_hat1, xi1_hat1, J1, v_hat1;
    x2 << zeta_hat2, chi_hat2, xi1_hat2, J2, v_hat2;

    // 更新开关控制器
    J1(a1) = J1(a1) - h;
    J1.minCoeff(&min_ind1);
    a1 = min_ind1;
    // a1 = 1;

    J2(a2) = J2(a2) - h;
    J2.minCoeff(&min_ind2);
    a2 = min_ind2;
    // a2 = 0;

    k += 1; // 递增步数
}

// 设置控制器输入
void multiDiscreteSeparate::setControllerInput(double controller_input)
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
    std::vector<double> omega_star_param = config["omega_star_param"].as<std::vector<double>>();
    for (auto omega_star_param_iter = omega_star_param.begin(); omega_star_param_iter != omega_star_param.end(); omega_star_param_iter++)
    {
        *omega_star_param_iter = *omega_star_param_iter * M_PI * 2;
    }
    const double vep_param = config["vep_param"].as<double>();
    const double rho_param = config["rho_param"].as<double>();
    const double beta_param = config["beta_param"].as<double>();
    const double h_param = config["h_param"].as<double>();
    const std::vector<int> swi_sig_param = config["swi_sig_param"].as<std::vector<int>>();
    const double u_gain_param = config["u_gain_param"].as<double>();

    // 创建控制器实例
    multiDiscreteSeparate *ctrl_ptr = new multiDiscreteSeparate(omega_star_param, vep_param, rho_param, beta_param, h_param, swi_sig_param, u_gain_param);

    int sample_len = (config["run_time"].as<double>()) * config["sample_fs"].as<double>();

    // 初始化数据记录器
    logger.init({"k", "y", "u", "a1", "chi_hat11", "chi_hat12", "a2", "chi_hat21", "chi_hat22"}, sample_len, controller_log_path);

    std::cout << "Controller log path: " << controller_log_path << std::endl;

    return (void *)(ctrl_ptr);
}

// 控制器计算函数
double controllerCompute(void *ctrl_ptr, double controller_input)
{
    multiDiscreteSeparate *ptr = (multiDiscreteSeparate *)(ctrl_ptr);
    double u;

    u = ptr->Outputs(); // 计算控制输出
    ptr->setControllerInput(controller_input); // 设置控制器输入
    ptr->Update();  // 更新控制器状态

    // 记录数据
    logger.log({double(ptr->k), ptr->y, u, double(ptr->a1), ptr->x1(6 + 2 * ptr->a1), ptr->x1(7 + 2 * ptr->a1), double(ptr->a2), ptr->x2(6 + 2 * ptr->a2), ptr->x2(7 + 2 * ptr->a2)});

    // 返回控制输出
    return u;
}

// 控制器结束函数
void controllerFinish(void *ctrl_ptr)
{
    logger.write(); // 写入日志
    multiDiscreteSeparate *ptr = (multiDiscreteSeparate *)(ctrl_ptr);
    delete ptr; // 删除控制器实例
}