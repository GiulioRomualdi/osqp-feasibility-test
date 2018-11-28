/**
 * @file QPTest.cpp
 * @author Giulio Romualdi
 * @copyright Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

// osqp-eigen
#include "OsqpEigen/OsqpEigen.h"

// eigen
#include <Eigen/Dense>

#include <iostream>

#include <EigenMatio/EigenMatio.hpp>

bool solve()
{

    Eigen::MatioFile file("test.mat", MAT_ACC_RDONLY, false);

    Eigen::MatrixXd H;
    file.read_mat("hessian", H);

    Eigen::SparseMatrix<double> H_s;
    H_s = H.sparseView();

    Eigen::MatrixXd A;
    file.read_mat("constraint", A);

    Eigen::SparseMatrix<double> A_s;
    A_s = A.sparseView();

    Eigen::VectorXd gradient;
    file.read_mat("gradient", gradient);

    Eigen::VectorXd lowerBound;
    file.read_mat("lowerBound", lowerBound);

    Eigen::VectorXd upperBound;
    file.read_mat("upperBound", upperBound);

    OsqpEigen::Solver solver;
    solver.settings()->setVerbosity(false);

    std::cerr << "sizes hessian " << H_s.cols() << " " << H_s.rows() << "\n";
    std::cerr << "sizes constraint " << A_s.cols() << " " << A_s.rows() << "\n";
    std::cerr << "lower bound " << lowerBound.size() << "\n";
    std::cerr << "upper bound " << upperBound.size() << "\n";
    std::cerr << "gradient bound " << gradient.size() << "\n";

    solver.data()->setNumberOfVariables(H_s.cols());
    solver.data()->setNumberOfConstraints(A_s.rows());
    solver.data()->setHessianMatrix(H_s);
    solver.data()->setGradient(gradient);
    solver.data()->setLinearConstraintsMatrix(A_s);
    solver.data()->setLowerBound(lowerBound);
    solver.data()->setUpperBound(upperBound);


    solver.initSolver();

    if(!solver.solve())
        return false;

    std::cerr << "The solution of the QP problem is" << std::endl;
    std::cerr << "[ " << solver.getSolution() << " ]"
              << std::endl;

    return true;
};

int main(int argc, char **argv)
{
    if(!solve())
    {
        std::cerr << "Unable to find a solution";
    }

    return true;
}
