#ifndef __HQP__MATH__SE3__
#define __HQP__MATH__SE3__

#include "fwd.h"
#include "utils/motion.h"

namespace HQP {
	Eigen::Matrix3d	alphaSkew(const double & s, const Vector3d & v);
	Eigen::VectorXd log3(const MatrixXd & R, double & theta);
	Eigen::VectorXd log3(const MatrixXd & R);
	MotionVector<double> log6(Transform3d & M);
	MotionVector<double> actinv(const Transform3d & M, const VectorXd & s);
	Matrix3d skew(const VectorXd s);
	double h_factor(const double & x, const double & upper, const double & lower);
	double h_factor_dis(double x, double upper, double lower);
	double cubic(double time,     ///< Current time
		double time_0,   ///< Start time
		double time_f,   ///< End time
		double x_0,      ///< Start state
		double x_f,      ///< End state
		double x_dot_0,  ///< Start state dot
		double x_dot_f   ///< End state dot
		
	);
	 Vector3d GetPhi(Matrix3d Rot, Matrix3d Rotd);


}
#endif __HQP__MATH__SE3__

