// Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

#include "lidarOptimization.h"

// 边特征
EdgeAnalyticCostFunction::EdgeAnalyticCostFunction(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_a_, Eigen::Vector3d last_point_b_)
        : curr_point(curr_point_), last_point_a(last_point_a_), last_point_b(last_point_b_){

}

// 通过paremeters来计算residual和jacobians
bool EdgeAnalyticCostFunction::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{       
    Eigen::Map<const Eigen::Quaterniond> q_last_curr(parameters[0]);
    Eigen::Map<const Eigen::Vector3d> t_last_curr(parameters[0] + 4);
    Eigen::Vector3d lp;
    lp = q_last_curr * curr_point + t_last_curr; 

    Eigen::Vector3d nu = (lp - last_point_a).cross(lp - last_point_b);      // 残差计算
    Eigen::Vector3d de = last_point_a - last_point_b;
    double de_norm = de.norm();
    residuals[0] = nu.norm()/de_norm;
    
    if(jacobians != NULL)
    {
        if(jacobians[0] != NULL)        // !计算雅克比,残差对parameters的导数   // Evaluate会检验这两个是否为空,若为空就用残差函数的导数值填充它
        {
            Eigen::Matrix3d skew_lp = skew(lp);
            Eigen::Matrix<double, 3, 6> dp_by_se3; 
            dp_by_se3.block<3,3>(0,0) = -skew_lp;
            (dp_by_se3.block<3,3>(0, 3)).setIdentity();

            Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor> > J_se3(jacobians[0]);
            J_se3.setZero();
            Eigen::Matrix3d skew_de = skew(de);
            J_se3.block<1,6>(0,0) = - nu.transpose() / nu.norm() * skew_de * dp_by_se3/de_norm; 

        }
    }  

    return true;
}   

// 面特征
SurfNormAnalyticCostFunction::SurfNormAnalyticCostFunction(Eigen::Vector3d curr_point_, Eigen::Vector3d plane_unit_norm_, double negative_OA_dot_norm_) 
                                                        : curr_point(curr_point_), plane_unit_norm(plane_unit_norm_), negative_OA_dot_norm(negative_OA_dot_norm_){

}

bool SurfNormAnalyticCostFunction::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    Eigen::Map<const Eigen::Quaterniond> q_w_curr(parameters[0]);           // 残差计算
    Eigen::Map<const Eigen::Vector3d> t_w_curr(parameters[0] + 4);
    Eigen::Vector3d point_w = q_w_curr * curr_point + t_w_curr;
    residuals[0] = plane_unit_norm.dot(point_w) + negative_OA_dot_norm;

    if(jacobians != NULL)
    {
        if(jacobians[0] != NULL)        // 计算雅克比
        {
            Eigen::Matrix3d skew_point_w = skew(point_w);
            Eigen::Matrix<double, 3, 6> dp_by_se3;
            dp_by_se3.block<3,3>(0,0) = -skew_point_w;
            (dp_by_se3.block<3,3>(0, 3)).setIdentity();

            Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor> > J_se3(jacobians[0]);
            J_se3.setZero();
            J_se3.block<1,6>(0,0) = plane_unit_norm.transpose() * dp_by_se3;
        }
    }           
    return true;

}   

// 自定义加法规则
// 这种结构就是x与delta维度相同可以直接加减
// x:优化前参数, delta:用旋转矢量表示的增量, x_plus_delta:优化后的参数
bool PoseSE3Parameterization::Plus(const double *x, const double *delta, double *x_plus_delta) const
{
    Eigen::Map<const Eigen::Vector3d> trans(x + 4);  // 优化前的平移
    Eigen::Map<const Eigen::Quaterniond> quater(x);     // 优化前的旋转

    Eigen::Quaterniond delta_q;
    Eigen::Vector3d delta_t;
    getTransformFromSe3( Eigen::Map<const Eigen::Matrix<double,6,1>>(delta), delta_q, delta_t );    // 旋转向量转四元素, 提取出q,t
    Eigen::Map<Eigen::Quaterniond> quater_plus(x_plus_delta);
    Eigen::Map<Eigen::Vector3d> trans_plus(x_plus_delta + 4);
    // 更新
    quater_plus = delta_q * quater;
    trans_plus = delta_q * trans + delta_t;

    return true;    
}

bool PoseSE3Parameterization::ComputeJacobian(const double *x, double *jacobian) const
{
    // 行主序
    Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> j(jacobian);       // ?! 四元素对旋转向量的雅克比
    (j.topRows(6)).setIdentity();
    (j.bottomRows(1)).setZero();

    return true;
}

// ! 旋转向量转四元素 p52页推倒
void getTransformFromSe3(const Eigen::Matrix<double,6,1>& se3, Eigen::Quaterniond& q, Eigen::Vector3d& t) {     
    Eigen::Vector3d omega(se3.data());      // 计算旋转
    Eigen::Vector3d upsilon(se3.data()+3);      // 计算平移
    Eigen::Matrix3d Omega = skew(omega);

    double theta = omega.norm();
    double half_theta = 0.5*theta;

    double real_factor = cos(half_theta);  //! w = cos(theta/2)
   
    double imag_factor; 
    if(theta<1e-10)     // 计算imag_factor, 进而计算q
    {
        double theta_sq = theta*theta;
        double theta_po4 = theta_sq*theta_sq;
        imag_factor = 0.5-0.0208333*theta_sq+0.000260417*theta_po4;     // ?加权啥子的吧?
    }
    else
    {
        double sin_half_theta = sin(half_theta);    //! x = nx * sin(theta/2)
        imag_factor = sin_half_theta/theta;
    }

    q = Eigen::Quaterniond(real_factor, imag_factor*omega.x(), imag_factor*omega.y(), imag_factor*omega.z());


    Eigen::Matrix3d J;
    if (theta<1e-10)        // 计算J, 进而计算t
    {
        J = q.matrix();
    }
    else
    {
        Eigen::Matrix3d Omega2 = Omega*Omega;
        J = (Eigen::Matrix3d::Identity() + (1-cos(theta))/(theta*theta)*Omega + (theta-sin(theta))/(pow(theta,3))*Omega2);
    }

    t = J*upsilon;
}

// 向量 转 反对称矩阵
Eigen::Matrix<double,3,3> skew(Eigen::Matrix<double,3,1>& mat_in){              
    Eigen::Matrix<double,3,3> skew_mat;
    skew_mat.setZero();
    skew_mat(0,1) = -mat_in(2);
    skew_mat(0,2) =  mat_in(1);
    skew_mat(1,2) = -mat_in(0);
    skew_mat(1,0) =  mat_in(2);
    skew_mat(2,0) = -mat_in(1);
    skew_mat(2,1) =  mat_in(0);
    return skew_mat;
}


