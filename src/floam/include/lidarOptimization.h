// Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro
#ifndef _LIDAR_OPTIMIZATION_ANALYTIC_H_
#define _LIDAR_OPTIMIZATION_ANALYTIC_H_

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

void getTransformFromSe3(const Eigen::Matrix<double,6,1>& se3, Eigen::Quaterniond& q, Eigen::Vector3d& t);
Eigen::Matrix3d skew(Eigen::Vector3d& mat_in);		// 向量 转 反对称矩阵

// 边特征计算
class EdgeAnalyticCostFunction : public ceres::SizedCostFunction<1, 7> {	
	public:

		EdgeAnalyticCostFunction(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_a_, Eigen::Vector3d last_point_b_);
		virtual ~EdgeAnalyticCostFunction() {}
		//! 通过paremeters来计算residual和jacobians
		virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;	// 必须重写函数Evaluate

		Eigen::Vector3d curr_point;
		Eigen::Vector3d last_point_a;
		Eigen::Vector3d last_point_b;
};
// 面特征计算
class SurfNormAnalyticCostFunction : public ceres::SizedCostFunction<1, 7> {
	public:
		SurfNormAnalyticCostFunction(Eigen::Vector3d curr_point_, Eigen::Vector3d plane_unit_norm_, double negative_OA_dot_norm_);
		virtual ~SurfNormAnalyticCostFunction() {}
		virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

		Eigen::Vector3d curr_point;
		Eigen::Vector3d plane_unit_norm;
		double negative_OA_dot_norm;
};		
// 自定义LocalParameterization
class PoseSE3Parameterization : public ceres::LocalParameterization {
public:
	
    PoseSE3Parameterization() {}
    virtual ~PoseSE3Parameterization() {}

	//! x:优化前参数, delta:用旋转矢量表示的增量, x_plus_delta:优化后的参数
    virtual bool Plus(const double* x, const double* delta, double* x_plus_delta) const;	// 对一些加法不封闭的优化变量, 重定义加法规则
    virtual bool ComputeJacobian(const double* x, double* jacobian) const;		 // 雅克比矩阵
    virtual int GlobalSize() const { return 7; }		// 实际上给的优化变量7自由度
    virtual int LocalSize() const { return 6; }		// 优化内部(正切空间上的参数维数)只有6自由度
};



#endif // _LIDAR_OPTIMIZATION_ANALYTIC_H_

