#include <iostream>
#include "tools.h"

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
	rmse << 0,0,0,0;

	// checking the validity of the following inputs:
	//  * the estimation vector size should not be zero
	if(estimations.size() == 0){
	    std::cout << "Error: No Estimations Given";
	    return rmse;
	}
	//  * the estimation vector size should equal ground truth vector size
	if(estimations.size() != ground_truth.size()){
	    std::cout << "Error: Number of Estimations and Ground Truth values should be equal.";
	    return rmse;
	}
	
	//accumulating squared residuals
	for(int i=0; i < estimations.size(); i++){
        // ... your code here
        VectorXd diff = estimations[i] - ground_truth[i];
		VectorXd curr_square = diff.array()*diff.array();
		rmse += curr_square;
	}

	//calculate the mean
	// ... your code here
	rmse /= estimations.size();

	//calculate the squared root
	// ... your code here
    rmse = rmse.array().sqrt();
    
	//return the result
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

	//TODO: YOUR CODE HERE 

	float px2 = px*px;
	float py2 = py*py;
	float r2 = px2 + py2;
	float r = sqrt(r2);
	float r3 = r2 * r;
	float num1 = py*(vx*py - px*vy);
	float num2 = px*(vy*px - py*vx);
	
	//check division by zero
	if(fabs(r2) < 0.0001){
	    while(1){
	    	std::cout<< "Error: Division by Zero.";
	    }
	    Hj << 0,0,0,0,
	          0,0,0,0,
	          0,0,0,0;
	}else{
	    Hj << px/r, py/r, 0, 0,
	          -py/r2, px/r2, 0, 0,
	          num1/r3, num2/r3, px/r, py/r;
	}

	return Hj;
}
