#include "controlFunction.h"
#include "logger/data_logging.hpp"

// 创建数据记录器实例
data_logger logger;

// Wang2016SingleDiscrete 控制器构造函数，初始化控制器参数
Wang2016SingleDiscrete::Wang2016SingleDiscrete(double omega_star_param, double vep_param, double rho_param, int a_param, double beta_param, double h_param, double gain_param)
{
    // 参数赋值
    vep = vep_param;
    rho = rho_param;
    beta = beta_param;
    h = h_param;
    gain = gain_param;

    // 初始化常量参数
    r1 = 0.1;
    r2 = 3;
    omega_star = omega_star_param / 5000;

    // 初始化矩阵
    G << 1, 0;
    Gamma << 1, 0;
    S << cos(omega_star), sin(omega_star),
        -sin(omega_star), cos(omega_star);
    F << S - vep * G * Gamma;

    // 初始化状态变量
    k = 0;
    y = 0.0;
    a = a_param;

    // 初始化估计变量
    Eigen::VectorXd chi_hat_init(6);
    chi_hat_init << sqrt(3.0) / 2.0, 1.0 / 2.0, -sqrt(3.0) / 2.0, 1.0 / 2.0, 0.0, -1.0;
    x.setZero(19);
    x.segment(6, 6) << chi_hat_init;

    // 初始化分块对角矩阵
    blkdiag_G.setZero(6, 3);
    blkdiag_G << blkdiag(G, 3);
    blkdiag_Gamma.setZero(3, 6);
    blkdiag_Gamma << blkdiag(Gamma, 3);
    blkdiag_S.setZero(6, 6);
    blkdiag_S << blkdiag(S, 3);
}

// 计算控制输出
double Wang2016SingleDiscrete::Outputs()
{
    double u;
    u = gain * Gamma * x.segment(17, 2); // 控制输出计算
    return u;
}

// 更新控制器状态
void Wang2016SingleDiscrete::Update()
{
    Eigen::Vector2d xi1_hat, v_hat;
    Eigen::Vector3d J, e, y1_hat, y_til, m_sr;
    Eigen::VectorXd zeta_hat(6), chi_hat(6), vph(6);
    double e_a;
    Eigen::Index min_ind;

    // 提取状态变量
    zeta_hat << x.segment(0, 6);
    chi_hat << x.segment(6, 6);
    xi1_hat << x.segment(12, 2);
    J << x.segment(14, 3);
    v_hat << x.segment(17, 2);

    // 多模型估计器和切换控制器
    e << -vep * chi_hat.cwiseProduct(zeta_hat).reshaped(2, 3).colwise().sum().transpose();
    e_a = e(a);

    y1_hat << blkdiag_Gamma * zeta_hat;
    y_til << y1_hat.array() - y;

    zeta_hat << blkdiag_S * zeta_hat + chi_hat * e_a - vep * blkdiag_G * y_til;

    // 多模型更新规律
    m_sr << 1 + xi1_hat.squaredNorm() + y_til.array().square();

    vph << -rho * pow(vep, 2) * xi1_hat.replicate(3, 1).cwiseProduct(repelem(y_til, 2, 1)).cwiseQuotient(repelem(m_sr, 2, 1));
    chi_hat << chi_hat + vph;

    xi1_hat << F.transpose() * xi1_hat + G * e_a;

    //代价函数计算
    J << J + beta * y_til.array().square().matrix();

    // 参数更新律
    v_hat << S * v_hat + G * e_a;

    // 更新状态变量
    x << zeta_hat, chi_hat, xi1_hat, J, v_hat;

    // 更新切换控制器
    J(a) = J(a) - h;
    J.minCoeff(&min_ind);
    a = min_ind;

    k += 1; // 递增步数
}

// 设置控制器输入
void Wang2016SingleDiscrete::setControllerInput(double controller_input)
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
    const double w_star_param = config["dist_freq"].as<double>() * M_PI * 2;
    const double vep_param = config["vep_param"].as<double>();
    const double rho_param = config["rho_param"].as<double>();
    const int a_param = config["a_param"].as<int>();
    const double beta_param = config["beta_param"].as<double>();
    const double h_param = config["h_param"].as<double>();
    const double gain_param = config["gain_param"].as<double>();

    // 创建控制器实例
    Wang2016SingleDiscrete *ctrl_ptr = new Wang2016SingleDiscrete(w_star_param, vep_param, rho_param, a_param, beta_param, h_param, gain_param);

    int sample_len = (config["run_time"].as<double>()) * config["sample_fs"].as<double>();

    // 初始化数据记录器
    logger.init({"k", "u", "y", "a", "chi_hat_1", "chi_hat_2"}, sample_len, controller_log_path);

    std::cout << "Controller log path: " << controller_log_path << std::endl;

    return (void *)(ctrl_ptr);
}

// 控制器计算函数
double controllerCompute(void *ctrl_ptr, double controller_input)
{
    Wang2016SingleDiscrete *ptr = (Wang2016SingleDiscrete *)(ctrl_ptr);
    double u;

    u = ptr->Outputs(); // 计算控制输出
    ptr->setControllerInput(controller_input); // 设置控制器输入
    ptr->Update(); // 更新控制器状态

    // 记录数据
    logger.log({double(ptr->k), u, ptr->y, double(ptr->a), ptr->x(6 + 2 * ptr->a), ptr->x(7 + 2 * ptr->a)});

    return u; // 返回控制输出
}

// 控制器结束函数
void controllerFinish(void *ctrl_ptr)
{
    logger.write(); // 写入日志
    Wang2016SingleDiscrete *ptr = (Wang2016SingleDiscrete *)(ctrl_ptr);
    delete ptr; // 删除控制器实例
}