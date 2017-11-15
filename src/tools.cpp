#include <iostream>
#include "tools.h"
#include <cmath>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  VectorXd rmse(4);
  rmse <<  0, 0, 0, 0;

  if(estimations.size() != ground_truth.size() || estimations.size() == 0){
    std::cout << "Invalide estimation" << '\n';
    return rmse;
  }

  for(unsigned int i=0; i < estimations.size(); ++i){
    VectorXd residual = estimations[i] - ground_truth[i];
    residual = residual.array() * residual.array();
    rmse += residual;
  }
  rmse = rmse/estimations.size();
  rmse = rmse.array().sqrt();

  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
  MatrixXd Hj(3,4);
	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);
	float temp = 0;

	//TODO: YOUR CODE HERE
    Hj << 0,0,0,0,
          0,0,0,0,
          0,0,0,0;
	//check division by zero
    temp = pow(px, 2)+pow(py,2);
    if(temp == 0)
        return Hj;
	//compute the Jacobian matrix
    Hj(0,0) = px/sqrt(temp);
    Hj(0,1) = py/sqrt(temp);
    Hj(1,0) = -py/temp;
    Hj(1,1) = px/temp;
    Hj(2,0) = py*(vx*py-vy*px)/pow(temp, 3/2);
    Hj(2,1) = px*(vx*py-vy*px)/pow(temp, 3/2);
    Hj(2,2) = px/sqrt(temp);
    Hj(2,3) = py/sqrt(temp);
	return Hj;
}
