#include "controlFunction.h"
#include "logger/data_logging.hpp"

data_logger logger;

priPath::priPath(double mu_w_param, int L_param, int M_param)
{
    mu_w = mu_w_param;
    L = L_param;
    M = M_param;

    k = 0;
    e = 0.0;
    y = 0.0;
    v = 0.0;
    Pe = 1.0;
    Pf = 1.0;

    w.setZero(L);
    x_L.setZero(L);

    s_hat.setZero(M);
    v_M.setZero(M);
    y_M.setZero(M);
    x_M.setZero(M);
    x_hat_prime_M.setZero(M);

    std::default_random_engine generator;
    std::normal_distribution<double> distribution(0.0, 0.1);

    I_M.setZero(M, M);
    I_M.diagonal<-1>().setOnes();
    I_L.setZero(L, L);
    I_L.diagonal<-1>().setOnes();
}

double priPath::Outputs()
{
    y = w.transpose() * x_L;
    v = distribution(generator);
    v = v / 20;
    // y_M << I_M * y_M;
    // y_M(0) = y;

    // v_M << I_M * v_M;
    // v_M(0) = v;

    return v;
}

void priPath::setControllerInput(double residual_noise)
{
    e = residual_noise;
}

void priPath::Update()
{
    k += 1;
}

void *controllerInit(int argc, char *argv[])
{
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

    const std::string log_folder = config["log_folder"].as<std::string>();
    const std::string controller_log_path = log_folder + '/' + config["controller_log_path"].as<std::string>();
    const double mu_w_param = config["mu_w_param"].as<double>();
    const int L_param = config["L_param"].as<int>();
    const int M_param = config["M_param"].as<int>();

    priPath *ctrl_ptr = new priPath(mu_w_param, L_param, M_param);

    int sample_len = (config["run_time"].as<double>()) * config["sample_fs"].as<double>();

    logger.init({"k", "sys_in", "res_noise", "cntrlr_out"}, sample_len, controller_log_path);

    std::cout << "Controller log path: " << controller_log_path << std::endl;

    return (void *)(ctrl_ptr);
}

double controllerCompute(void *ctrl_ptr, double system_input, double residual_noise)
{
    priPath *ptr = (priPath *)(ctrl_ptr);
    double u;

    // u = ptr->Outputs();
    u = 0.0;
    ptr->setControllerInput(residual_noise);
    ptr->Update();

    logger.log({double(ptr->k), system_input, residual_noise, ptr->y});

    return u;
}

void controllerFinish(void *ctrl_ptr)
{
    logger.write();
    priPath *ptr = (priPath *)(ctrl_ptr);
    delete ptr;
}