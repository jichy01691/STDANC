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

class Tomei2017DiscreteJiChenyang
{
public:
/*
    double w,R_0,Iw,g,eta_1,eta_2,eta_3,w_cos;
*/    
    double freq,SampleTime,Iw,g;
    double eta_1,eta_2;
    double omega,omega_cos;
    // initial state
/*    Eigen::VectorXd x;
    // control gain
    double vep, rho, beta, h, r1, r2;
    // omega_star
    Eigen::VectorXd omega;
    Eigen::RowVectorXd Gamma;
    Eigen::MatrixXd G, S, F;
*/
    int k;
    double y;

    Tomei2017DiscreteJiChenyang(double freq_param, double SampleTime_param, double Iw_param, double g_param);
    double Outputs();
    void Update();
    void setControllerInput(double controller_input);
};

void *controllerInit(int argc, char *argv[]);

double controllerCompute(void *ctrl_ptr, double u);

void controllerFinish(void *ctrl_ptr);
/*
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
*/
