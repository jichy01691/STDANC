#pragma once

#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>
#include <chrono>
#include <thread>
#include <cmath>

#include "yaml.h"
#include <Eigen/Dense>
#include <Eigen/Core>
#include <boost/math/constants/constants.hpp>
#include <boost/numeric/odeint.hpp>
#include "msgpack.hpp"

#include "utils.h"

class switchingSingle
{
public:
    // controller state
    Eigen::VectorXd x;
    // control gain
    double epsilon, alpha, gamma, h, cntrlr_gain;
    Eigen::Vector2d G, thetaHatSigma;
    Eigen::RowVector2d Gamma;
    // omega_star
    double omega_star;
    Eigen::Matrix2d S, F;
    int k;
    double y;

    switchingSingle(double epsilon_param, double alpha_param, double gamma_param, double h_param, double omega_star_param, std::vector<double> thetaHatSigma_param, double cntrlr_gain_param);
    double Outputs();
    void Update();
    void setControllerInput(double controller_input);
};

void *controllerInit(int argc, char *argv[]);

double controllerCompute(void *ctrl_ptr, double u);

void controllerFinish(void *ctrl_ptr);
