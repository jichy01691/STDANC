#pragma once

#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>
#include <chrono>
#include <thread>
#include <cmath>
#include <iterator>
#include <random>

#include "yaml.h"
#include <Eigen/Dense>
#include <Eigen/Core>
#include <boost/math/constants/constants.hpp>
#include <boost/numeric/odeint.hpp>
#include "msgpack.hpp"

#include "utils.h"

class priPath
{
public:
    // controller state
    Eigen::VectorXd w, x_L, x_M, s_hat, v_M, y_M, x_hat_prime_M;
    Eigen::MatrixXd I_M, I_L;
    // control gain
    double mu_w;
    int k, L, M;
    double e, y, v, Pe, Pf;

    std::default_random_engine generator;
    std::normal_distribution<double> distribution;

    priPath(double mu_w_param, int L_param, int M_param);
    double Outputs();
    void setControllerInput(double residual_noise);
    void Update();
};

void *controllerInit(int argc, char *argv[]);

double controllerCompute(void *ctrl_ptr, double system_input, double residual_noise);

void controllerFinish(void *ctrl_ptr);
