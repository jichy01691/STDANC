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

class switchingComposite
{
public:
    // initial state
    Eigen::VectorXd zetaHat, xi1Hat, eta, wHat;
    // control gain
    double epsilon, alpha, h;
    Eigen::VectorXd cntrlr_gain, gamma, thetaHatSigma;
    Eigen::RowVectorXd Gamma;
    Eigen::MatrixXd G;
    // omega_star
    Eigen::VectorXd omega_star;
    Eigen::MatrixXd S, F;
    int N, k;
    double y;
    
    switchingComposite(double epsilon_param, double alpha_param, std::vector<double> gamma_param, double h_param, std::vector<double> omega_star_param, std::vector<double> thetaHatSigma_param, std::vector<double> cntrlr_gain_param, int N_param);
    double Outputs();
    void setControllerInput(double controller_input);
    void Update();
};

void *controllerInit(int argc, char *argv[]);

double controllerCompute(void *ctrl_ptr, double u);

void controllerFinish(void *ctrl_ptr);

template <typename XprType, typename RowFactorType, typename ColFactorType>
auto repelem(const XprType &xpr, RowFactorType row_factor, ColFactorType col_factor)
{
    using namespace Eigen;

    const int RowFactor = internal::get_fixed_value<RowFactorType>::value;
    const int ColFactor = internal::get_fixed_value<ColFactorType>::value;
    const int NRows = XprType::RowsAtCompileTime == Dynamic || RowFactor == Dynamic ? Dynamic : XprType::RowsAtCompileTime * RowFactor;
    const int NCols = XprType::ColsAtCompileTime == Dynamic || ColFactor == Dynamic ? Dynamic : XprType::ColsAtCompileTime * ColFactor;
    const int nrows = internal::get_runtime_value(row_factor) * xpr.rows();
    const int ncols = internal::get_runtime_value(col_factor) * xpr.cols();

    return xpr(
        Array<int, NRows, 1>::LinSpaced(nrows, 0, xpr.rows() - 1),
        Array<int, NCols, 1>::LinSpaced(ncols, 0, xpr.cols() - 1));
}

template <typename Derived>
Eigen::MatrixXd blkdiag(const Eigen::EigenBase<Derived>& A, int count)
{
    Eigen::MatrixXd bdm = Eigen::MatrixXd::Zero(A.rows() * count, A.cols() * count);
    for (int i = 0; i < count; ++i)
    {
        bdm.block(i * A.rows(), i * A.cols(), A.rows(), A.cols()) = A;
    }

    return bdm;
}