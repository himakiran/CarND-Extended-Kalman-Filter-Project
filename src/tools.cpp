#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
  
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  if((estimations.size()==0) or (ground_truth.size()==0)){
      cout << "vector size is zero" << endl;
      return rmse;
  }
  else if( estimations.size() != ground_truth.size() ) { 
      cout << "vector sizes are  unequal" << endl;
      return rmse;
  }

   else {
  // accumulate squared residuals
  VectorXd temp(4);
  temp << 0,0,0,0;
  double val=0;
  for (int i=0; i < estimations.size(); ++i) {   
    temp = estimations[i]-ground_truth[i];
    temp = temp.array() * temp.array() ;
    rmse += temp;
    }
    
	//   calculate the mean
   	 rmse = rmse/estimations.size();
	// //   cout << rmse << endl;
	//  calculate the squared root
     rmse = rmse.cwiseSqrt();
	// return the result
  return rmse;
   }

}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
	MatrixXd Hj(3,4);
  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // TODO: YOUR CODE HERE 

  // check division by zero
  
  // compute the Jacobian matrix

  float r1c1,r1c2,r2c1,r2c2,r3c1,r3c2,r3c3,r3c4;
  Hj << 0,0,0,0,
        0,0,0,0,
        0,0,0,0;
  try
  {
    if ((px - py) < 0.0001 ) {
        r1c1 = px/sqrt(pow(px,2)+pow(py,2));
        r1c2 = py/sqrt(pow(px,2)+pow(py,2));
        r2c1 = -py/(pow(px,2)+pow(py,2));
        r2c2 = px/(pow(px,2)+pow(py,2));
        r3c1 = (py*(vx*py-vy*px))/pow((pow(px,2)+pow(py,2)),3/2);
        r3c2 = (px*(vy*px-vx*py))/pow((pow(px,2)+pow(py,2)),3/2);
        r3c3 = r1c1;
        r3c4 = r1c2;
        Hj << r1c1,r1c2,0,0,
              r2c1,r2c2,0,0,
              r3c1,r3c2,r3c3,r3c4;
        
        } else {
            throw (px - py);
        }
  }
  catch (int e)
  {
    cout << "An exception occurred. Integer division by  " << e << '\n';
    return Hj;
  }
  

  return Hj;
}
