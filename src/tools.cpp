#include <iostream>
#include "tools.h"

#define EPS 0.0001 // A very small number
#define EPS2 0.0000001

using namespace std;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd rmse(4);
  rmse << 0,0,0,0;
  // Check the validity of the following inputs:
  // The estimation vector size should not be zero
  if(estimations.size() == 0){
    cout << "Input is empty" << endl;
    return rmse;
  }
  // The estimation vector size should equal ground truth vector size
  if(estimations.size() != ground_truth.size()){
    cout << "Invalid estimation or ground_truth. Data should have the same size" << endl;
    return rmse;
  }
  // Accumulate squared residuals
  for(unsigned int i=0; i < estimations.size(); ++i){
    VectorXd residual = estimations[i] - ground_truth[i];
    // Coefficient-wise multiplication
    residual = residual.array()*residual.array();
    rmse += residual;
  }

  // Calculate the mean
  rmse = rmse / estimations.size();
  rmse = rmse.array().sqrt();
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  // Code from lectures quizes
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);
  MatrixXd Hj(3,4);
  // Deal with the special case problems
  if (fabs(px) < EPS and fabs(py) < EPS){
	  px = EPS;
	  py = EPS;
  }
  // Pre-compute a set of terms to avoid repeated calculation
  float c1 = px*px+py*py;
  // Check division by zero
  if(fabs(c1) < EPS2){
	c1 = EPS2;
  }
  float c2 = sqrt(c1);
  float c3 = (c1*c2);
  // Compute the Jacobian matrix
  Hj << (px/c2), (py/c2), 0, 0,
       -(py/c1), (px/c1), 0, 0,
        py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;
  return Hj;
}
