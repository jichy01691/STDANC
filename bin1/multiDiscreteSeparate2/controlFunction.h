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

class multiDiscreteSeparate
{
public:
    // initial state
    Eigen::VectorXd x1, x2;
    // control gain
    double vep, rho, beta, h, u_gain;
    Eigen::Vector2d G;
    Eigen::RowVector2d Gamma;
    // omega_star
    double omega_star1, omega_star2;
    Eigen::Matrix2d S1, S2, F1, F2;
    Eigen::MatrixXd blkdiag_Gamma, blkdiag_S1, blkdiag_S2, blkdiag_G;
    int a1, a2, k;
    double y;

    multiDiscreteSeparate(std::vector<double> omega_star_param, double vep_param, double rho_param, double beta_param, double h_param, std::vector<int> swi_sig_param, double u_gain_param);
    double Outputs();
    void Update();
    void setControllerInput(double controller_input);
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