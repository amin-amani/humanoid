#ifndef RIGHT_HAND_H
#define RIGHT_HAND_H
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "Eigen/QuadProg.h"
#include "Eigen/testQPsolvers.hpp"
#include "Eigen/eiquadprog.hpp"
#include "Eigen/Core"
#include "Eigen/Cholesky"
#include "Eigen/LU"
#include<math.h>

using namespace Eigen;
using namespace std;

class right_hand
{
public:
    double T;

    double L_arm;
    double L_forearm;
    double L_palm;
    double angle_fix_shd;
    double angle_fix_elbow;

    double v_des;
    double d_des;
    double d_orient;
    double power;
    double Right_palm_position_power;
    double Right_palm_orientation_power;
    VectorXd qdot;
    //    double q1_ra;
    //    double q2_ra;
    //    double q3_ra;
    //    double q4_ra;
    //    double q5_ra;
    //    double q6_ra;
    //    double q7_ra;

    MatrixXd R1_fix_shd;
    MatrixXd R2_fix_shd;
    MatrixXd R1_ra ;
    MatrixXd R2_ra;
    MatrixXd R3_ra;
    MatrixXd R1_fix_elbow;
    MatrixXd R2_fix_elbow;
    MatrixXd R4_ra;
    MatrixXd R5_ra;
    MatrixXd R6_ra;
    MatrixXd R7_ra;

    MatrixXd P_arm_ra;
    MatrixXd P_forearm_ra;
    MatrixXd P_palm_ra;

    MatrixXd T_right_palm;
    MatrixXd R_right_palm;
    VectorXd r_right_palm;

    double sai;
    double theta;
    double phi;

    double sai_target;
    double theta_target;
    double phi_target;

    MatrixXd Jha;
    MatrixXd J_right_palm;
    MatrixXd J_w_right_palm;

    double Vx_right_palm;
    double Vy_right_palm;
    double Vz_right_palm;
    Vector3d w_right_palm;
    double sai_dot;
    double phi_dot;
    double theta_dot;

    VectorXd V;
    VectorXd q_next;
    right_hand();
    right_hand(VectorXd q_ra, VectorXd r_target, MatrixXd R_target);
    right_hand(VectorXd q_ra, VectorXd r_target, MatrixXd R_target, int i, double d0);
    double toRad(double d);
    double sai_calc(MatrixXd R);
    double phi_calc(MatrixXd R);
    MatrixXd trans(int axis, double d);
    MatrixXd trans(Vector3d d);
    MatrixXd rot(int axis, double q, int dim);
    double theta_calc(MatrixXd R);
    void HO_FK_right_palm(VectorXd q_ra);
    void euler2w();
    void jacob(VectorXd q_ra);
    double dist;
    VectorXd upbound;
    VectorXd lowbound;

    double qdot_max;
    MatrixXd G;
    VectorXd g;
    MatrixXd CI;
    VectorXd ci0;
    MatrixXd CE;
    VectorXd ce0;
    double distance(VectorXd V1, VectorXd V2);

    void doQP(VectorXd q_ra);


    double wrist_pos2mot(double pos);
    vector<int> data2qc_without_wrist(vector<double> cntrl);
    vector<int> data2qc(vector<double> cntrl);
};

#endif // RIGHT_HAND_H
