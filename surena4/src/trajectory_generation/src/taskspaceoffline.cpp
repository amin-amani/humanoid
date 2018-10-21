#include "taskspaceoffline.h"
TaskSpaceOffline::TaskSpaceOffline()
{

    RightFootXVelocity.append(0);
    RightFootYVelocity.append(0);

    RightFootXTrajectory.append(0);
    RightFootYTrajectory.append(0);

    RightFootXAcceleration.append(0);
    RightFootYAcceleration.append(0);

    LeftFootXVelocity.append(0);
    LeftFootYVelocity.append(0);

    LeftFootXTrajectory.append(0);
    LeftFootYTrajectory.append(0);

    LeftFootXAcceleration.append(0);
    LeftFootYAcceleration.append(0);

    LeftFootZTrajectory.append(_lenghtOfAnkle);
    LeftFootZVelocity.append(0);
    LeftFootZAcceleration.append(0);

    RightFootZTrajectory.append(_lenghtOfAnkle);
    RightFootZVelocity.append(0);
    RightFootZacceleration.append(0);

    RightFootAlphaTrajectory.append(0);
    RightFootBethaTrajectory.append(0);
    RightFootGamaTrajectory.append(0);

    LeftFootAlphaTrajectory.append(0);
    LeftFootBethaTrajectory.append(0);
    LeftFootGamaTrajectory.append(0);
    CoMXVelocityVector.append(0);
    CoMXVector.append(0);
    CoMYVector.append(0);
    CoMZVector.append(0);

    timeVector.append(0);
globalTime=0;
   time=0;
    _timeStep=0.01;
    SetParameters();
    CoeffArrayAnkle();
    CoeffArrayPelvis();
    bool _walkstate=true;


//    while (_walkstate==true) {


//    }


}

void TaskSpaceOffline::SetParameters(){
YOffsetOfAnkletrajectory=0.00;
    er=0.001;
    Rqa=0.95;
    Ra_i=0;
    Ra_f=0.93;
    Rla_i=1;
    Rla_f=1;


    Ra_st_i=0;
    Ra_st_f=1;


    Ra_end_i=0;
    Ra_end_f=1;


    Rla_st_i=1;
    Rla_st_f=1;

    Rla_end_i=1;
    Rla_end_f=1;


    Sc=0.3783;
    Rse=0.7364;
    Rd=0.5555;
    Rm=1.1095;
    Rzp=0.91;

    _lengthOfHip=0.10900;
    _lenghtOfAnkle=0.11200;
    _lengthOfShank=0.3700;
    _lengthOfThigh=0.3600;


    Delta=0.02;// Domain of pelvis movemevt in z direction
    NStride=1;
    DesiredVelocity=0.1;
    StepLength=0.15;

    Tc=StepLength*3.6/DesiredVelocity;
    //timeStepT=0.01;
    TStart=6;
    TEnd=6;
    TDs=.4*Tc; // Duration of double support phase
    T_beta=TDs/2;
    TSS=Tc-TDs; // Duration of single support phase
    TGait=TStart+NStride*2*Tc;
    XofAnkleMaximumHeight=(0.22/0.35)*StepLength; // Position of ankle in x direction when it reaches maximum heigth
    za_c=0.08+_lenghtOfAnkle;
    AnkleMaximumHeight=za_c; // maximum height of ankle
    za_st_m=AnkleMaximumHeight;


    _pelvisLength=0.23;
    YpMax=1.1*Rm*0.5*_pelvisLength;
    Yd=1.1*Rd*YpMax;
    YEndMax=1.0*YpMax;
    YStMax=1.0*YpMax;
    Xs=Rse*Xe;
    xa_st_m=(0.22/0.35)*StepLength/2;
    za_st_m=(AnkleMaximumHeight);
    Xe=Sc*StepLength/(Rse+1);
    xa_end_m=2*NStride*StepLength+(0.22/0.35)*StepLength/2;
    za_end_m=(AnkleMaximumHeight);
    L_2leg_Ds=sqrt(pow((_lengthOfShank+_lengthOfThigh),2)-pow((0.5*StepLength),2))+_lenghtOfAnkle+_lengthOfHip;
    MinHeightPelvis=Rzp*L_2leg_Ds; // minimum heigth of pelvis
    ReferencePelvisHeight=0.83;
    MaxHeightPelvis=MinHeightPelvis+Delta;
    if (true) {
        Xe=1*Sc*StepLength/(Rse+1);
        Xs=0.95*Rse*Xe;
        YpMax=0.85*Rm*0.5*_pelvisLength;
        Yd=1*Rd*YpMax;
        YStMax=1.0*YpMax;
        YEndMax=1.0*YpMax;

    }
    //    MinHeightPelvis=Rzp*L_2leg_Ds; //minimum heigth of pelvis
    //    MaxHeightPelvis=MinHeightPelvis+Delta; //maximum heigth of pelvis
    MotionTime=TStart+NStride*2*Tc+TDs+TEnd;


    TMinPelvisZ=0.5*TDs; // The time that pelvis reaches its minimum distance in z direction
    TMinPelvisY=0.5*TDs; // The time that pelvis reaches its minimum distance in y direction
    TMaxAnkle=TDs+0.35*TSS;//0.53 % The time that ankle reaches its maximum distance in z direction
    TMaxPelvisZ=TDs+0.5*TSS; // The time that pelvis reaches its maximum distance in z direction
    TMaxPelvisY=TDs+0.5*TSS; // The time that pelvis reaches its maximum distance in y direction

    T_end_p_sx=TGait+TDs+0.25*TEnd;
    T_end_p_sy=TGait+TDs+0.2*TEnd;
    T_end_p_dy=TGait+TDs+0.4*TEnd;
    T_end_p_ey=TGait+TDs+0.8*TEnd;

    T_end_p_sz=TGait+TDs+0.3*TEnd;
    T_end_p_dz=TGait+TDs+0.5*TEnd;
    T_end_p_ez=TGait+TDs+TEnd;

    T_end_p_sq=T_end_p_sx;
    T_end_a_s=TGait+TDs;
    T_end_a_e=TGait+TDs+0.5*TEnd;
    T_end_a_d=TGait+TDs+0.45*(T_end_a_e-T_end_a_s);

    T_st_p_sy=0.2*TStart;
    T_st_p_dy=0.65*TStart;
    T_st_p_ey=0.8*TStart;

    T_st_p_sx=0.7*TStart;

    T_st_p_sz=0;
    T_st_p_dz=0.5*TStart;
    T_st_p_ez=0.7*TStart;

    T_st_p_sq=T_st_p_sx;

    T_st_a_s=0.5*TStart;
    T_st_a_d=T_st_a_s+0.45*(TStart-T_st_a_s);

}

void TaskSpaceOffline::CoeffArrayAnkle(){
    double  v_i;
    double  v_f;
    double v_st_i;
    double v_st_f;

    double v_end_i;
    double v_end_f;

    if (Ra_i==0){
        v_i=0;
    }
    else{
        v_i=(Rla_i-1)*_lenghtOfAnkle/(Ra_i*Tc);
    }
    if (Ra_f==1){
        v_f=0;
    }
    else{
        v_f=(1-Rla_f)*_lenghtOfAnkle/((1-Ra_f)*Tc);
    }


    if (Ra_st_i==0){
        v_st_i=0;
    }
    else{
        v_st_i=(Rla_st_i-1)*_lenghtOfAnkle/(Ra_st_i*(TStart-T_st_a_s));
    }
    if (Ra_st_f==1){
        v_st_f=0;
    }
    else{
        v_st_f=(1-Rla_st_f)*_lenghtOfAnkle/((1-Ra_st_f)*(TStart-T_st_a_s));
    }

    if (Ra_end_i==0){
        v_end_i=0;
    }
    else{
        v_end_i=(Rla_end_i-1)*_lenghtOfAnkle/(Ra_end_i*(T_end_a_e-T_end_a_s));
    }
    if (Ra_end_f==1){
        v_end_f=0;
    }
    else{
        v_end_f=(1-Rla_end_f)*_lenghtOfAnkle/((1-Ra_end_f)*(T_end_a_e-T_end_a_s));
    }

    // X Coeff
    //    MatrixXd Cx_iTime(1,3);
    //    Cx_iTime<<(TDs+Ra_i*Tc), TMaxAnkle ,(Ra_f*Tc);
    //    MatrixXd Cx_iPos(1,3);
    //    Cx_iPos<<0, XofAnkleMaximumHeight, 2*StepLength;
    //    MatrixXd Cx_iVel(1,3);
    //    Cx_iVel<<0, INFINITY, 0;
    //    MatrixXd Cx_iAcc(1,3);
    //    Cx_iAcc<<0, INFINITY, 0;
    //    MatrixXd Cx_i=CoefOffline.Coefficient(Cx_iTime,Cx_iPos,Cx_iVel,Cx_iAcc);
    //    Cx.resize(1,12);
    //    Cx.block(0,0,1,6)=Cx_i.row(0);
    //    Cx.block(0,6,1,6)=Cx_i.row(1);


    // Z Coeff
    // Cz_i=Coefficient([(Td+Ra_i*Tc) Tm (Ra_f*Tc)],[5 4],[Rla_i*la za_m Rla_f*la+2*h*(Case==3);v_i 0 v_f;0 inf 0]);
    //    MatrixXd Cz_iTime(1,5);
    //    Cz_iTime<<TSS,(TDs+Ra_i*Tc)+er*(Ra_i==0), TMaxAnkle, (Ra_f*Tc)-er*(Ra_f==1), Tc;
    //    MatrixXd Cz_iPos(1,5);
    //    Cz_iPos<<_lenghtOfAnkle, Rla_i*_lenghtOfAnkle, AnkleMaximumHeight ,Rla_f*_lenghtOfAnkle, _lenghtOfAnkle;
    //    MatrixXd Cz_iVel(1,5);
    //    Cz_iVel<<0 ,v_i ,0, v_f ,0;
    //    MatrixXd Cz_iAcc(1,5);
    //    Cz_iAcc<<0 ,0 ,INFINITY, 0, 0;
    //    MatrixXd Cz_i;
    //    Cz_i=CoefOffline.Coefficient(Cz_iTime,Cz_iPos,Cz_iVel,Cz_iAcc);

    //    Cz.resize(12,1);
    //    Cz.block(0,0,6,1)=Cz_i.row(1).transpose();
    //    Cz.block(6,0,6,1)=Cz_i.row(2).transpose();


    // Angle about y axis Coeff
    // C_beta_i=Coefficient([Td Tm (Rqa*Tc)],[5 4],[-phi*(Case==2) beta_m -phi*(Case==2);0 0 0;0 inf 0]);
    //    //C_beta=[C_beta_i(1:6,1);C_beta_i(2:6,2)];
    //    MatrixXd Cx_st_iTime(1,3);
    //    Cx_st_iTime<<T_st_a_s+Ra_st_i*(TStart-T_st_a_s) ,T_st_a_d ,TStart+(Ra_st_f-1)*(TStart-T_st_a_s);
    //    MatrixXd Cx_st_iPos(1,3);
    //    Cx_st_iPos<<0, xa_st_m ,StepLength;
    //    MatrixXd Cx_st_iVel(1,3);
    //    Cx_st_iVel<<0, INFINITY, 0;
    //    MatrixXd Cx_st_iAcc(1,3);
    //    Cx_st_iAcc<<0, INFINITY, 0;
    //    MatrixXd Cx_st_i=CoefOffline.Coefficient(Cx_st_iTime,Cx_st_iPos,Cx_st_iVel,Cx_st_iAcc);
    //    Cx_st.resize(1,12);
    //    Cx_st.block(0,0,1,6)=Cx_st_i.row(0);
    //    Cx_st.block(0,6,1,6)=Cx_st_i.row(1);

    //    //start motion in z direction
    //    MatrixXd Cz_st_iTime(1,5);
    //    Cz_st_iTime<<T_st_a_s, T_st_a_s+Ra_st_i*(TStart-T_st_a_s)+er*(Ra_st_i==0), T_st_a_d, TStart+(Ra_st_f-1)*(TStart-T_st_a_s)-er*(Ra_st_f==1) ,TStart;
    //    MatrixXd Cz_st_iPos(1,5);
    //    Cz_st_iPos<<_lenghtOfAnkle, Rla_st_i*_lenghtOfAnkle ,za_st_m ,(Rla_st_f*_lenghtOfAnkle), (_lenghtOfAnkle);
    //    MatrixXd Cz_st_iVel(1,5);
    //    Cz_st_iVel<<0, v_st_i, 0 ,v_st_f ,0;
    //    MatrixXd Cz_st_iAcc(1,5);
    //    Cz_st_iAcc<<0 ,0 ,INFINITY, 0, 0;
    //    MatrixXd Cz_st_i=CoefOffline.Coefficient(Cz_st_iTime,Cz_st_iPos,Cz_st_iVel,Cz_st_iAcc);
    //    Cz_st.resize(12,1);
    //    Cz_st.block(0,0,6,1)=Cz_st_i.row(1).transpose();
    //    Cz_st.block(6,0,6,1)=Cz_st_i.row(2).transpose();




    // start motion angle about y axis Coeff
    //C_beta_st=Coefficient([T_st_a_s T_st],5,[0 -phi*(Case==2);0 0;0 0]);

    // End Motion
    // end motion in x direction

    //    MatrixXd Cx_end_iTime(1,3);
    //    Cx_end_iTime<<T_end_a_s+Ra_end_i*(T_end_a_e-T_end_a_s), T_end_a_d ,T_end_a_e+(Ra_end_f-1)*(T_end_a_e-T_end_a_s);
    //    MatrixXd Cx_end_iPos(1,3);
    //    Cx_end_iPos<<2*NStride*StepLength, xa_end_m ,(2*NStride+1)*StepLength;
    //    MatrixXd Cx_end_iVel(1,3);
    //    Cx_end_iVel<<0, INFINITY, 0;
    //    MatrixXd Cx_end_iAcc(1,3);
    //    Cx_end_iAcc<<0, INFINITY, 0;
    //    MatrixXd Cx_end_i=CoefOffline.Coefficient(Cx_end_iTime,Cx_end_iPos,Cx_end_iVel,Cx_end_iAcc);
    //    Cx_end.resize(12,1);
    //    Cx_end.block(0,0,6,1)=Cx_end_i.row(0).transpose();
    //    Cx_end.block(6,0,6,1)=Cx_end_i.row(1).transpose();

    //    MatrixXd Cz_end_iTime(1,5);
    //    Cz_end_iTime<<T_end_a_s ,T_end_a_s+Ra_end_i*(T_end_a_e-T_end_a_s)+er*(Ra_end_i==0), T_end_a_d, T_end_a_e+(Ra_end_f-1)*(T_end_a_e-T_end_a_s)-er*(Ra_end_f==1) ,T_end_a_e;
    //    MatrixXd Cz_end_iPos(1,5);
    //    Cz_end_iPos<<_lenghtOfAnkle, Rla_st_i*_lenghtOfAnkle ,za_st_m ,(Rla_st_f*_lenghtOfAnkle), (_lenghtOfAnkle);
    //    MatrixXd Cz_end_iVel(1,5);
    //    Cz_end_iVel<<0, v_st_i, 0 ,v_st_f ,0;
    //    MatrixXd Cz_end_iAcc(1,5);
    //    Cz_end_iAcc<<0 ,0 ,INFINITY, 0, 0;
    //    MatrixXd Cz_end_i=CoefOffline.Coefficient(Cz_end_iTime,Cz_end_iPos,Cz_end_iVel,Cz_end_iAcc);
    //    Cz_end.resize(12,1);
    //    Cz_end.block(0,0,6,1)=Cz_end_i.row(1).transpose();
    //    Cz_end.block(6,0,6,1)=Cz_end_i.row(2).transpose();

    //    Cz_end_i=Coefficient([T_end_a_s T_end_a_s+Ra_end_i*(T_end_a_e-T_end_a_s)+er*(Ra_end_i==0) T_end_a_d T_end_a_e+(Ra_end_f-1)*(T_end_a_e-T_end_a_s)-er*(Ra_end_f==1) T_end_a_e],[5 5 4 5],[la*(Case~=2)+r_D(3)*(Case==2) Rla_end_i*la*(Case~=2)+r_D_end(3)*(Case==2) za_end_m (Rla_end_f*la+h*(Case==3))*(Case~=2)+(Rla_end_f*la+z_Off_st_slope+((2*N_Stride+1)*Ds-x_Off_st_slope-x_Off_end_slope)*sin(phi)+z_Off_end_slope)*(Case==2) (la+h*(Case==3))*(Case~=2)+(la+z_Off_st_slope+((2*N_Stride+1)*Ds-x_Off_st_slope-x_Off_end_slope)*sin(phi)+z_Off_end_slope)*(Case==2);0 v_end_i 0 v_end_f 0;0 0 inf 0 0]);
    //    Cz_end=[Cz_end_i(1:6,2);Cz_end_i(2:6,3)];

    // end motion angle about y axis Coeff
    // C_beta_end=Coefficient([T_end_a_s T_end_a_e],5,[-phi*(Case==2) 0;0 0;0 0]);
    T_s_st=.5*TStart;

    MatrixXd Cx_st_iTime_al(1,2);
    Cx_st_iTime_al<<T_s_st ,TStart;
    MatrixXd Cx_st_iPos_al(1,2);
    Cx_st_iPos_al<<0, StepLength;
    MatrixXd Cx_st_iVel_al(1,2);
    Cx_st_iVel_al<<0, 0;
    MatrixXd Cx_st_iAcc_al(1,2);
    Cx_st_iAcc_al<<0, 0;
    C_st_x_al=CoefOffline.Coefficient(Cx_st_iTime_al,Cx_st_iPos_al,Cx_st_iVel_al,Cx_st_iAcc_al);

    MatrixXd C_st_iTime(1,3);
    C_st_iTime<<T_s_st, TStart-T_s_st/2 ,TStart;
    MatrixXd C_st_iPos(1,3);
    C_st_iPos<<_lenghtOfAnkle, AnkleMaximumHeight, _lenghtOfAnkle;
    MatrixXd C_st_iVel(1,3);
    C_st_iVel<<0 ,INFINITY, 0;;
    MatrixXd C_st_iAcc(1,3);
    C_st_iAcc<<0 ,INFINITY, 0;
    C_st_z_al=CoefOffline.Coefficient(C_st_iTime,C_st_iPos,C_st_iVel,C_st_iAcc);

    MatrixXd Cy_st_iTime(1,3);
    Cy_st_iTime<<T_s_st, TStart-T_s_st/2 ,TStart;
    MatrixXd Cy_st_iPos(1,3);
    Cy_st_iPos<<0.5*_pelvisLength,0.5*_pelvisLength+YOffsetOfAnkletrajectory, 0.5*_pelvisLength;
    MatrixXd Cy_st_iVel(1,3);
    Cy_st_iVel<<0 ,INFINITY, 0;
    MatrixXd Cy_st_iAcc(1,3);
    Cy_st_iAcc<<0 ,INFINITY, 0;
    C_st_y_al=CoefOffline.Coefficient(Cy_st_iTime,Cy_st_iPos,Cy_st_iVel,Cy_st_iAcc);

    //C_st_z_al=Coefficient([T_s_st T_st-T_s_st/2 T_st],[4 4],[la za_m la*cos(beta_min_st)-lh*sin(beta_min_st);0 inf 0;0 inf 0]);

    MatrixXd C_cy_iTime_al(1,2);
    C_cy_iTime_al<<0, TSS;
    MatrixXd C_cy_iPos_al(1,2);
    C_cy_iPos_al<<0, 2*StepLength;
    MatrixXd C_cy_iVel_al(1,2);
    C_cy_iVel_al<<0, 0;
    MatrixXd C_cy_iAcc_al(1,2);
    C_cy_iAcc_al<<0, 0;
    C_cy_x_al=CoefOffline.Coefficient(C_cy_iTime_al,C_cy_iPos_al,C_cy_iVel_al,C_cy_iAcc_al);


    MatrixXd C_cy_iTime_ar(1,2);
    C_cy_iTime_ar<<0, TSS;
    MatrixXd C_cy_iPos_ar(1,2);
    C_cy_iPos_ar<<0, 2*StepLength;
    MatrixXd C_cy_iVel_ar(1,2);
    C_cy_iVel_ar<<0, 0;
    MatrixXd C_cy_iAcc_ar(1,2);
    C_cy_iAcc_ar<<0, 0;
    C_cy_x_ar=CoefOffline.Coefficient(C_cy_iTime_ar,C_cy_iPos_ar,C_cy_iVel_ar,C_cy_iAcc_ar);


    MatrixXd C_cy_iTime(1,3);
    C_cy_iTime<<0 ,TSS/2, TSS;
    MatrixXd C_cy_iPos(1,3);
    C_cy_iPos<<_lenghtOfAnkle,AnkleMaximumHeight, _lenghtOfAnkle;
    MatrixXd C_cy_iVel(1,3);
    C_cy_iVel<<0 ,INFINITY, 0;;
    MatrixXd C_cy_iAcc(1,3);
    C_cy_iAcc<<0 ,INFINITY, 0;
    C_cy_z_ar=CoefOffline.Coefficient(C_st_iTime,C_st_iPos,C_st_iVel,C_st_iAcc);
    //C_cy_z_ar=Coefficient([0 Ts/2 Ts],[4 4],[la*cos(beta_max)+lf*sin(beta_max) za_m la*cos(beta_min)-lh*sin(beta_min);0 inf 0;0 inf 0]);

    MatrixXd C_end_z_iTime(1,3);
    C_end_z_iTime<<0 ,(T_end_a_e-T_end_a_s)/2 ,T_end_a_e-T_end_a_s;
    MatrixXd C_end_z_iPos(1,3);
    C_end_z_iPos<<_lenghtOfAnkle, AnkleMaximumHeight, _lenghtOfAnkle;
    MatrixXd C_end_z_iVel(1,3);
    C_end_z_iVel<<0 ,INFINITY, 0;;
    MatrixXd C_end_z_iAcc(1,3);
    C_end_z_iAcc<<0 ,INFINITY, 0;
    C_end_z_ar=CoefOffline.Coefficient(C_end_z_iTime,C_end_z_iPos,C_end_z_iVel,C_end_z_iAcc);
    //C_end_z_ar=Coefficient([0 (T_end_a_e-T_end_a_s)/2 T_end_a_e-T_end_a_s],[4 4],[la*cos(beta_max_end)+lf*sin(beta_max_end) za_m la;0 inf 0;0 inf 0]);

    MatrixXd C_end_iTime_ar(1,2);
    C_end_iTime_ar<<0, (T_end_a_e-T_end_a_s);
    MatrixXd C_end_iPos_ar(1,2);
    C_end_iPos_ar<<0, 1*StepLength;
    MatrixXd C_end_iVel_ar(1,2);
    C_end_iVel_ar<<0, 0;
    MatrixXd C_end_iAcc_ar(1,2);
    C_end_iAcc_ar<<0, 0;
    C_end_x_ar=CoefOffline.Coefficient(C_end_iTime_ar,C_end_iPos_ar,C_end_iVel_ar,C_end_iAcc_ar);


    MatrixXd C_end_y_iTime(1,3);
    C_end_y_iTime<<0 ,(T_end_a_e-T_end_a_s)/2 ,T_end_a_e-T_end_a_s;
    MatrixXd C_end_y_iPos(1,3);
    C_end_y_iPos<<-0.5*_pelvisLength,-0.5*_pelvisLength-YOffsetOfAnkletrajectory, -0.5*_pelvisLength;
    MatrixXd C_end_y_iVel(1,3);
    C_end_y_iVel<<0 ,INFINITY, 0;
    MatrixXd C_end_y_iAcc(1,3);
    C_end_y_iAcc<<0 ,INFINITY, 0;
    C_end_y_ar=CoefOffline.Coefficient(C_end_y_iTime,C_end_y_iPos,C_end_y_iVel,C_end_y_iAcc);



    MatrixXd Cy_cy_iTime(1,3);
    Cy_cy_iTime<<0 ,TSS/2, TSS;
    MatrixXd Cy_cy_iPos(1,3);
    Cy_cy_iPos<<-0.5*_pelvisLength,-0.5*_pelvisLength-YOffsetOfAnkletrajectory, -0.5*_pelvisLength;
    MatrixXd Cy_cy_iVel(1,3);
    Cy_cy_iVel<<0 ,INFINITY, 0;
    MatrixXd Cy_cy_iAcc(1,3);
    Cy_cy_iAcc<<0 ,INFINITY, 0;
    C_cy_y_ar=CoefOffline.Coefficient(Cy_cy_iTime,Cy_cy_iPos,Cy_cy_iVel,Cy_cy_iAcc);

    MatrixXd Cy_cy_lTime(1,3);
    Cy_cy_lTime<<0 ,TSS/2, TSS;
    MatrixXd Cy_cy_lPos(1,3);
    Cy_cy_lPos<<0.5*_pelvisLength,0.5*_pelvisLength+YOffsetOfAnkletrajectory, 0.5*_pelvisLength;
    MatrixXd Cy_cy_lVel(1,3);
    Cy_cy_lVel<<0 ,INFINITY, 0;
    MatrixXd Cy_cy_lAcc(1,3);
    Cy_cy_lAcc<<0 ,INFINITY, 0;
    C_cy_y_al=CoefOffline.Coefficient(Cy_cy_lTime,Cy_cy_lPos,Cy_cy_lVel,Cy_cy_lAcc);


}
void TaskSpaceOffline::CoeffArrayPelvis(){

    MatrixXd ConTime(1,3);
    ConTime<<0 ,TDs, Tc;
    MatrixXd ConPos(1,3);
    ConPos<<Xe, StepLength-Xs, StepLength+Xe;
    MatrixXd ConVel(1,3);
    ConVel<<INFINITY, INFINITY ,INFINITY;
    MatrixXd ConAccel(1,3);
    ConAccel<<INFINITY, INFINITY ,INFINITY;
    Cx_p_i=CoefOffline.Coefficient(ConTime,ConPos,ConVel,ConAccel);

    //Cx_p_i=Coefficient([0 Td Tc],[3 3],Con);
    MatrixXd ord(1,2);
    ord << 3,3;
    MatrixXd ttt(1,3);
    ttt <<0 ,TDs, Tc;

    MatrixXd con(3,3);
    con<<Xe, StepLength-Xs, StepLength+Xe,INFINITY, INFINITY ,INFINITY,INFINITY, INFINITY ,INFINITY;
    Cx_p_i.resize(2,6);
    Cx_p_i.fill(0);
    Cx_p_i.block(0,2,2,4)=CoefOffline.Coefficient1(ttt,ord,con,0.1).transpose();

    Cx_p.resize(1,12);
    Cx_p.fill(0);
    Cx_p.block(0,0,1,6)=Cx_p_i.row(0);
    Cx_p.block(0,6,1,6)=Cx_p_i.row(1);

    // Start motion in x direction
    //  Cx_st_p=Coefficient([T_st_p_sx T_st],5,[0 xe;0 Cx_p(3);0 2*Cx_p(2)]);

    MatrixXd Cx_st_pTime(1,2);
    Cx_st_pTime<<T_st_p_sx ,TStart;
    MatrixXd Cx_st_pPos(1,2);
    Cx_st_pPos<<0, Xe;
    MatrixXd Cx_st_pVel(1,2);
    Cx_st_pVel<<0 ,Cx_p(0,4);
    MatrixXd Cx_st_pAccel(1,2);
    Cx_st_pAccel<<0 ,2*Cx_p(0,3);
    Cx_st_p=CoefOffline.Coefficient(Cx_st_pTime,Cx_st_pPos,Cx_st_pVel,Cx_st_pAccel);

    //Cx_end_p=Coefficient([(T_Gait+Td) T_end_p_sx],5,[(2*N_Stride+1)*Ds-xs (2*N_Stride+1)*Ds;3*Cx_p(1)*Td^2+2*Cx_p(2)*Td+Cx_p(3) 0;6*Cx_p(0,0)*TDs+2*Cx_p(0,1) 0]);


    MatrixXd Cx_end_pTime(1,2);
    Cx_end_pTime<<(TGait+TDs),T_end_p_sx ;
    MatrixXd Cx_end_pPos(1,2);
    Cx_end_pPos<<(2*NStride+1)*StepLength-Xs, (2*NStride+1)*StepLength;
    MatrixXd Cx_end_pVel(1,2);
    Cx_end_pVel<<5*Cx_p(0,0)*pow(TDs,4)+4*Cx_p(0,1)*pow(TDs,3)+3*Cx_p(0,2)*pow(TDs,2)+2*Cx_p(0,3)*pow(TDs,1)+Cx_p(0,4), 0;
    MatrixXd Cx_end_pAccel(1,2);
    Cx_end_pAccel<<5*4*Cx_p(0,0)*pow(TDs,4)+4*3*Cx_p(0,1)*pow(TDs,3)+3*2*Cx_p(0,2)*pow(TDs,2)+2*Cx_p(0,3), 0;//like the last moment of first part of trajectory of cycle
    Cx_end_p=CoefOffline.Coefficient(Cx_end_pTime,Cx_end_pPos,Cx_end_pVel,Cx_end_pAccel);

    MatrixXd ordY(1,8);
    ordY << 3,3,4,4,3,3,4,4;
    MatrixXd tttY(1,9);
    tttY <<0,TMinPelvisY,TDs,TDs+TSS/2,Tc,Tc+TDs/2,Tc+TDs,Tc+TDs+TSS/2,2*Tc;

    MatrixXd conY(3,9);
    conY<<-1*Yd,0,Yd,YpMax,Yd,0,-1*Yd,-1*YpMax,-1*Yd,INFINITY, INFINITY,INFINITY, 0 ,INFINITY,INFINITY,INFINITY,0,INFINITY,INFINITY, INFINITY,INFINITY, INFINITY ,INFINITY,INFINITY,INFINITY,INFINITY,INFINITY;
    Cy_p_i.resize(8,6);
    Cy_p_i.fill(0);
    Cy_p_i.block(0,1,8,5)=CoefOffline.Coefficient1(tttY,ordY,conY,0.1).transpose();

    //    MatrixXd ConYTime(1,9);
    //    ConYTime<<0,TMinPelvisY,TDs,TDs+TSS/2,Tc,Tc+TDs/2,Tc+TDs,Tc+TDs+TSS/2,2*Tc;
    //    MatrixXd ConYPos(1,9);
    //    ConYPos<<-1*Yd,0,Yd,YpMax,Yd,0,-1*Yd,-1*YpMax,-1*Yd;
    //    MatrixXd ConYVel(1,9);
    //    ConYVel<<INFINITY, INFINITY,INFINITY, 0 ,INFINITY,INFINITY,INFINITY,0,INFINITY;
    //    MatrixXd ConYAccel(1,9);
    //    ConYAccel<<INFINITY, INFINITY,INFINITY, INFINITY ,INFINITY,INFINITY,INFINITY,INFINITY,INFINITY;
    //    Cy_p_i=CoefOffline.Coefficient(ConYTime,ConYPos,ConYVel,ConYAccel);
    Cy_p.resize(1,48);
    Cy_p.fill(0);


    Cy_p.block(0,0,1,6)=Cy_p_i.row(0);
    Cy_p.block(0,6,1,6)=Cy_p_i.row(1);
    Cy_p.block(0,12,1,6)=Cy_p_i.row(2);
    Cy_p.block(0,18,1,6)=Cy_p_i.row(3);
    Cy_p.block(0,24,1,6)=Cy_p_i.row(4);
    Cy_p.block(0,30,1,6)=Cy_p_i.row(5);
    Cy_p.block(0,36,1,6)=Cy_p_i.row(6);
    Cy_p.block(0,42,1,6)=Cy_p_i.row(7);



    //    time_p_ii =T_ac+[2*(i-1)*Tc          2*(i-1)*Tc+Td/2         2*(i-1)*Tc+Td        2*(i-1)*Tc+Td+Ts/2      (2*i-1)*Tc      (2*i-1)*Tc+Td/2     (2*i-1)*Tc+Td       (2*i-1)*Tc+Td+Ts/2];
    //       y_p=            [-yd                 0                       yd                   ye                      yd              0                   -yd                 -ye               ];
    //       dy_p=           [inf                 inf                     inf                  0                       inf             inf                 inf                 0                 ];
    //       ddy_p=          [inf                 inf                     inf                  inf                     inf             inf                 inf                 inf               ];



    // Start motion in y direction
    MatrixXd Cy_st_psTime(1,2);
    Cy_st_psTime<<T_st_p_sy ,T_st_p_dy;
    MatrixXd Cy_st_psPos(1,2);
    Cy_st_psPos<<0, -1*YStMax;
    MatrixXd Cy_st_psVel(1,2);
    Cy_st_psVel<<0 ,0;
    MatrixXd Cy_st_psAccel(1,2);
    Cy_st_psAccel<<0 ,0;
    Cy_st_pa=CoefOffline.Coefficient(Cy_st_psTime,Cy_st_psPos,Cy_st_psVel,Cy_st_psAccel);



    MatrixXd Cy_st_peTime(1,2);
    Cy_st_peTime<<T_st_p_ey, TStart;
    MatrixXd Cy_st_pePos(1,2);
    Cy_st_pePos<<-1*YStMax, -1*Yd;
    MatrixXd Cy_st_peVel(1,2);
    Cy_st_peVel<<0 ,5*Cy_p(0,0)*pow(0,4)+4*Cy_p(0,1)*pow(0,3)+3*Cy_p(0,2)*pow(0,2)+2*Cy_p(0,3)*pow(0,1)+Cy_p(0,4);
    MatrixXd Cy_st_peAccel(1,2);
    Cy_st_peAccel<<0 ,2*Cy_p(0,3);
    Cy_st_pb=CoefOffline.Coefficient(Cy_st_peTime,Cy_st_pePos,Cy_st_peVel,Cy_st_peAccel);

    //    Cy_st_b=Coefficient([T_st_p_ey TStart],5,[ -yd;0 -4*Cy_p(6)*Tc^3-3*Cy_p(7)*Tc^2-2*Cy_p(8)*Tc-Cy_p(9);0 -12*Cy_p(6)*Tc^2-6*Cy_p(7)*Tc-2*Cy_p(8)]);

    // end motion in y direction
    //    Cy_end_a=Coefficient([(T_Gait+Td) T_end_p_sy],5,[yd y_end_max;4*Cy_p(1)*Td^3+3*Cy_p(2)*Td^2+2*Cy_p(3)*Td+Cy_p(4) 0;12*Cy_p(1)*Td^2+6*Cy_p(2)*Td+2*Cy_p(3) 0]);
    //    Cy_end_b=Coefficient([T_end_p_dy T_end_p_ey],5,[y_end_max 0;0 0;0 0]);
    MatrixXd Cy_end_psTime(1,2);
    Cy_end_psTime<<TGait+TDs, T_end_p_sy;
    MatrixXd Cy_end_psPos(1,2);
    Cy_end_psPos<<Yd, YEndMax;
    MatrixXd Cy_end_psVel(1,2);
    Cy_end_psVel<<5*Cy_p_i(1,0)*pow(TDs,4)+4*Cy_p_i(1,1)*pow(TDs,3)+3*Cy_p_i(1,2)*pow(TDs,2)+2*Cy_p_i(1,3)*pow(TDs,1)+Cy_p_i(1,4),0;
    MatrixXd Cy_end_psAccel(1,2);
    Cy_end_psAccel<< 5*4*Cy_p_i(1,0)*pow(TDs,4)+4*3*Cy_p_i(1,1)*pow(TDs,3)+3*2*Cy_p_i(1,2)*pow(TDs,2)+2*Cy_p_i(1,3), 0;
    Cy_end_pa=CoefOffline.Coefficient(Cy_end_psTime,Cy_end_psPos,Cy_end_psVel,Cy_end_psAccel);


    MatrixXd Cy_end_peTime(1,2);
    Cy_end_peTime<<T_end_p_dy, T_end_p_ey;
    MatrixXd Cy_end_pePos(1,2);
    Cy_end_pePos<<YEndMax, 0;
    MatrixXd Cy_end_peVel(1,2);
    Cy_end_peVel<<0 ,0;
    MatrixXd Cy_end_peAccel(1,2);
    Cy_end_peAccel<<0 ,0;
    Cy_end_pb=CoefOffline.Coefficient(Cy_end_peTime,Cy_end_pePos,Cy_end_peVel,Cy_end_peAccel);

    //Cz_end_a=Coefficient([(T_Gait+Td) T_end_p_sz],5,[Cz_p(1)*Td^3+Cz_p(2)*Td^2+Cz_p(3)*Td+Cz_p(4) zp_max;3*Cz_p(1)*Td^2+2*Cz_p(2)*Td+Cz_p(3) 0;6*Cz_p(1)*Td+2*Cz_p(2) 0]);
    MatrixXd Cz_p_pTime(1,3);
    Cz_p_pTime<<TMinPelvisZ, TMaxPelvisZ,Tc+TMinPelvisZ ;
    MatrixXd Cz_p_pPos(1,3);
    Cz_p_pPos<<MinHeightPelvis, MaxHeightPelvis,MinHeightPelvis;
    MatrixXd Cz_p_pVel(1,3);
    Cz_p_pVel<<0, 0 ,0;
    MatrixXd Cz_p_pAccel(1,3);
    Cz_p_pAccel<<INFINITY,INFINITY,INFINITY;
    Cz_p=CoefOffline.Coefficient(Cz_p_pTime,Cz_p_pPos,Cz_p_pVel,Cz_p_pAccel);

    MatrixXd Cz_st_pTime(1,2);
    Cz_st_pTime<<T_st_p_sz, T_st_p_dz ;
    MatrixXd Cz_st_pPos(1,2);
    Cz_st_pPos<<ReferencePelvisHeight ,MaxHeightPelvis;
    MatrixXd Cz_st_pVel(1,2);
    Cz_st_pVel<<0, 0 ;
    MatrixXd Cz_st_pAccel(1,2);
    Cz_st_pAccel<<0,0;
    Cz_st_a=CoefOffline.Coefficient(Cz_st_pTime,Cz_st_pPos,Cz_st_pVel,Cz_st_pAccel);


    //Cz_st_a=Coefficient([T_st_p_sz T_st_p_dz],5,[z_p_ref zp_max;0 0;0 0]);
    MatrixXd Cz_st_pbTime(1,2);
    Cz_st_pbTime<<T_st_p_ez, TStart ;
    MatrixXd Cz_st_pbPos(1,2);
    Cz_st_pbPos<<MaxHeightPelvis ,Cz_p(1,0)*pow(Tc-TMaxPelvisZ,5)+Cz_p(1,1)*pow(Tc-TMaxPelvisZ,4)+Cz_p(1,2)*pow(Tc-TMaxPelvisZ,3)+Cz_p(1,3)*pow(Tc-TMaxPelvisZ,2)+Cz_p(1,4)*pow(Tc-TMaxPelvisZ,1)+Cz_p(1,5);
    MatrixXd Cz_st_pbVel(1,2);
    Cz_st_pbVel<<0, 5*Cz_p(1,0)*pow(Tc-TMaxPelvisZ,4)+4*Cz_p(1,1)*pow(Tc-TMaxPelvisZ,3)+3*Cz_p(1,2)*pow(Tc-TMaxPelvisZ,2)+2*Cz_p(1,3)*pow(Tc-TMaxPelvisZ,1)+Cz_p(1,4)*pow(Tc-TMaxPelvisZ,0) ;
    MatrixXd Cz_st_pbAccel(1,2);
    Cz_st_pbAccel<<0,5*4*Cz_p(1,0)*pow(Tc-TMaxPelvisZ,3)+4*3*Cz_p(1,1)*pow(Tc-TMaxPelvisZ,2)+3*2*Cz_p(1,2)*pow(Tc-TMaxPelvisZ,1)+2*Cz_p(1,3)*pow(Tc-TMaxPelvisZ,0);
    Cz_st_b=CoefOffline.Coefficient(Cz_st_pbTime,Cz_st_pbPos,Cz_st_pbVel,Cz_st_pbAccel);

    MatrixXd Cz_end_pTime(1,2);
    Cz_end_pTime<<TGait+TDs, T_end_p_sz ;
    MatrixXd Cz_end_pPos(1,2);
    Cz_end_pPos<<Cz_p(0,0)*pow(TDs-TMinPelvisZ,5)+Cz_p(0,1)*pow(TDs-TMinPelvisZ,4)+Cz_p(0,2)*pow(TDs-TMinPelvisZ,3)+Cz_p(0,3)*pow(TDs-TMinPelvisZ,2)+Cz_p(0,4)*pow(TDs-TMinPelvisZ,1)+Cz_p(0,5) ,MaxHeightPelvis;
    MatrixXd Cz_end_pVel(1,2);
    Cz_end_pVel<<5*Cz_p(0,0)*pow(TDs-TMinPelvisZ,4)+4*Cz_p(0,1)*pow(TDs-TMinPelvisZ,3)+3*Cz_p(0,2)*pow(TDs-TMinPelvisZ,2)+2*Cz_p(0,3)*pow(TDs-TMinPelvisZ,1)+Cz_p(0,4)*pow(TDs-TMinPelvisZ,0) , 0 ;
    MatrixXd Cz_end_pAccel(1,2);
    Cz_end_pAccel<<5*4*Cz_p(0,0)*pow(TDs-TMinPelvisZ,3)+4*3*Cz_p(0,1)*pow(TDs-TMinPelvisZ,2)+3*2*Cz_p(0,2)*pow(TDs-TMinPelvisZ,1)+2*Cz_p(0,3)*pow(TDs-TMinPelvisZ,0),0;
    Cz_end_a=CoefOffline.Coefficient(Cz_end_pTime,Cz_end_pPos,Cz_end_pVel,Cz_end_pAccel);
    //Cz_st_b=Coefficient([T_st_p_ez T_st],5,[zp_max Cz_p(5)*Tc^3+Cz_p(6)*Tc^2+Cz_p(7)*Tc+Cz_p(8);0 3*Cz_p(5)*Tc^2+2*Cz_p(6)*Tc+Cz_p(7);0 6*Cz_p(5)*Tc+2*Cz_p(6)]);


    MatrixXd Cz_end_pbTime(1,2);
    Cz_end_pbTime<<T_end_p_dz, T_end_p_ez;
    MatrixXd Cz_end_pbPos(1,2);
    Cz_end_pbPos<<MaxHeightPelvis,ReferencePelvisHeight;
    MatrixXd Cz_end_pbVel(1,2);
    Cz_end_pbVel<<0,0;
    MatrixXd Cz_end_pbAccel(1,2);
    Cz_end_pbAccel<<0,0;
    Cz_end_b=CoefOffline.Coefficient(Cz_end_pbTime,Cz_end_pbPos,Cz_end_pbVel,Cz_end_pbAccel);
    //Cz_p_i=Coefficient([Tmdz Tmsz Tc+Tmdz],[3 3],[zp_min zp_max zp_min;0 0 0;NaN NaN NaN]);
    //Cz_p=[Cz_p_i(1:4,1);Cz_p_i(1:4,2)];
    //Cz_end_b=Coefficient([T_end_p_dz T_end_p_ez],5,[zp_max z_p_ref;0 0;0 0]);




    //    Ay_p=[Tmdy^4 Tmdy^3 Tmdy^2 Tmdy 1 0 0 0 0 0;
    //          4*Tmdy^3 3*Tmdy^2 2*Tmdy 1 0 4*(Tc+Tmdy)^3 3*(Tc+Tmdy)^2 2*(Tc+Tmdy) 1 0;
    //          12*Tmdy^2 6*Tmdy 2 0 0 12*(Tc+Tmdy)^2 6*(Tc+Tmdy) 2 0 0;
    //          0 0 0 0 0 (Tc+Tmdy)^4 (Tc+Tmdy)^3 (Tc+Tmdy)^2 (Tc+Tmdy) 1;

    //          Td^4 Td^3 Td^2 Td 1 0 0 0 0 0;
    //          0 0 0 0 0 Tc^4 Tc^3 Tc^2 Tc 1;

    //          Tmsy^4 Tmsy^3 Tmsy^2 Tmsy 1 0 0 0 0 0;
    //          4*Tmsy^3 3*Tmsy^2 2*Tmsy 1 0 0 0 0 0 0;
    //          0 0 0 0 0 4*Tmsy^3 3*Tmsy^2 2*Tmsy 1 0;
    //          0 0 0 0 0 Tmsy^4 Tmsy^3 Tmsy^2 Tmsy 1];
    //    By_p=[0;0;0;0;
    //          yd;yd;
    //          YpMax;0;0;YpMax];
    //    Cy_p=Ay_p\By_p;


}


MatrixXd TaskSpaceOffline::PelvisTrajectory(double time){
    double N;
    double t;
    double xp;
    double yp;
    double zp;
    double dxp;
    double dyp;
    double dzp;
    double ddxp;
    double ddyp;
    double ddzp;
    if (time<=TStart||time>TGait){
        N=0;
        t=time;
    }
    else if (time>TStart && time<TGait){
        N=floor((time-TStart)/(2*Tc));
        t=fmod((time-TStart),2*Tc)+TStart;
    }
    else if (time==TGait){
        N=NStride;
        t=0;
    }


    if (t<=T_st_p_sx){
        xp=0;
        dxp=0;
        ddxp=0;

    }
    else if (t>T_st_p_sx  && t<=TStart){
        MatrixXd output=GetAccVelPos(Cx_st_p,t,T_st_p_sx,5);
        xp=output(0,0);
        dxp=output(0,1);
        ddxp=output(0,2);
    }
    else if (t>TStart &&  t<=(TDs+TStart)){
        MatrixXd output=GetAccVelPos(Cx_p_i.row(0),t-TStart,0,5);
        xp=output(0,0);
        dxp=output(0,1);
        ddxp=output(0,2);
    }
    else if (t>(TDs+TStart) && t<=(Tc+TStart)){
        MatrixXd output=GetAccVelPos(Cx_p_i.row(1),t-TStart,0,5);
        xp=output(0,0);
        dxp=output(0,1);
        ddxp=output(0,2);
    }
    else if (t>(Tc+TStart) && t<=(Tc+TDs+TStart)){
        MatrixXd output=GetAccVelPos(Cx_p_i.row(0),t-Tc-TStart,0,5);
        xp=output(0,0)+StepLength;
        dxp=output(0,1);
        ddxp=output(0,2);
    }
    else if (t>(Tc+TDs+TStart) && t<=(2*Tc+TStart)){
        MatrixXd output=GetAccVelPos(Cx_p_i.row(1),t-Tc-TStart,0,5);
        xp=output(0,0)+StepLength;
        dxp=output(0,1);
        ddxp=output(0,2);
    }
    else if (t>TGait && t<(TGait+TDs)){
        MatrixXd output=GetAccVelPos(Cx_p_i.row(0),t-TGait,0,5);
        xp=output(0,0)+2*NStride*StepLength;
        dxp=output(0,1);
        ddxp=output(0,2);
    }
    else if (t>=(TGait+TDs) && t<T_end_p_sx){
        MatrixXd output=GetAccVelPos(Cx_end_p,t,(TGait+TDs),5);
        xp=output(0,0);
        dxp=output(0,1);
        ddxp=output(0,2);
    }
    else if (t>=T_end_p_sx  && t<=(TGait+TDs+TEnd)){
        xp=(2*NStride+1)*StepLength;
        dxp=0;
        ddxp=0;
    }

    if (TStart==0 && t==0){
        xp=Cx_p(4);
        dxp=0;
        ddxp=0;
    }

    xp=xp+2*StepLength*N;

    //Y


    if(t<=T_st_p_sy){
        yp=0;
        dyp=0;
        ddyp=0;
    }
    else if (t>T_st_p_sy && t<=T_st_p_dy){
        MatrixXd output=GetAccVelPos(Cy_st_pa,t,T_st_p_sy,5);
        yp=output(0,0);
        dyp=output(0,1);
        ddyp=output(0,2);
        // polyval(Cy_st_a,t);
    }
    else if (t>T_st_p_dy && t<=T_st_p_ey){
        yp=-1*YStMax;
        dyp=0;
        ddyp=0;
    }
    else if (t>T_st_p_ey && t<=TStart){

        MatrixXd output=GetAccVelPos(Cy_st_pb,t,T_st_p_ey,5);
        yp=output(0,0);
        dyp=output(0,1);
        ddyp=output(0,2);
    }

    else if (t>TStart && t<=(TMinPelvisY+TStart)){
        MatrixXd output=GetAccVelPos(Cy_p_i.row(0),t-TStart,0,5);
        yp=output(0,0);
        dyp=output(0,1);
        ddyp=output(0,2);
        // yp=-polyval(Cy_p(6:10),t+Tc-TStart);
    }
    else if (t>(TMinPelvisY+TStart) && t<=(TDs+TStart)){
        MatrixXd output=GetAccVelPos(Cy_p_i.row(1),t-TStart,0,5);
        yp=output(0,0);
        dyp=output(0,1);
        ddyp=output(0,2);
        //yp=polyval(Cy_p(1:5),t-TStart);
    }
    else if (t>(TDs+TStart) && t<=(TMaxPelvisY+TStart)){
        MatrixXd output=GetAccVelPos(Cy_p_i.row(2),t-TStart,0,5);
        yp=output(0,0);
        dyp=output(0,1);
        ddyp=output(0,2);

        // yp=polyval(Cy_p(6:10),t-TStart);
    }
    else if (t>(TMaxPelvisY+TStart) && t<=(Tc+TStart)){
        MatrixXd output=GetAccVelPos(Cy_p_i.row(3),t-TStart,0,5);
        yp=output(0,0);
        dyp=output(0,1);
        ddyp=output(0,2);
        // yp=-polyval(Cy_p(1:5) ,t-Tc-TStart);
        //0,TMinPelvisY,TDs,Tmaxpelvis,Tc,Tc+Tminpelvis,Tc+TDs,Tc+Tmaxpelvis,2*Tc;
    }

    else if (t>(Tc+TStart) && t<=(Tc+TMinPelvisY+TStart)){
        MatrixXd output=GetAccVelPos(Cy_p_i.row(4),t-TStart,0,5);
        yp=output(0,0);
        dyp=output(0,1);
        ddyp=output(0,2);
        //yp=-polyval(Cy_p(6:10), t-Tc-TStart);
    }
    else if (t>(Tc+TMinPelvisY+TStart) && t<=(Tc+TDs+TStart)){
        MatrixXd output=GetAccVelPos(Cy_p_i.row(5),t-TStart,0,5);
        yp=output(0,0);
        dyp=output(0,1);
        ddyp=output(0,2);
        //yp=-polyval(Cy_p(6:10), t-Tc-TStart);
    }
    else if (t>(Tc+TDs+TStart) && t<=(Tc+TMaxPelvisY+TStart)){
        MatrixXd output=GetAccVelPos(Cy_p_i.row(6),t-TStart,0,5);
        yp=output(0,0);
        dyp=output(0,1);
        ddyp=output(0,2);
        //yp=-polyval(Cy_p(6:10), t-Tc-TStart);
    }
    else if (t>(Tc+TMaxPelvisY+TStart) && t<=(2*Tc+TStart)){
        MatrixXd output=GetAccVelPos(Cy_p_i.row(7),t-TStart,0,5);
        yp=output(0,0);
        dyp=output(0,1);
        ddyp=output(0,2);
        //yp=-polyval(Cy_p(6:10), t-Tc-TStart);
    }
    else if (t>TGait && t<=(TMinPelvisY+TGait)){
        MatrixXd output=GetAccVelPos(Cy_p_i.row(0),t-TGait,0,5);
        yp=output(0,0);
        dyp=output(0,1);
        ddyp=output(0,2);
        //yp=-polyval(Cy_p(6:10) , t+Tc-TGait);
    }
    else if (t>(TMinPelvisY+TGait) && t<=(TDs+TGait)){
        MatrixXd output=GetAccVelPos(Cy_p_i.row(1),t-TGait,0,5);
        yp=output(0,0);
        ddyp=output(0,2);
        dyp=output(0,1);
        //yp=polyval(Cy_p(1:5) , t-TGait);
    }
    else if (t>(TDs+TGait) && t<=T_end_p_sy){
        // yp=polyval(Cy_end_a,t);
        MatrixXd output=GetAccVelPos(Cy_end_pa,t,TGait+TDs,5);
        yp=output(0,0);
        dyp=output(0,1);
        ddyp=output(0,2);
    }
    else if (t>T_end_p_sy && t<=T_end_p_dy){
        yp=YEndMax;
        dyp=0;
        ddyp=0;
    }
    else if (t>T_end_p_dy && t<=T_end_p_ey){
        MatrixXd output=GetAccVelPos(Cy_end_pb,t,T_end_p_dy,5);
        yp=output(0,0);
        dyp=output(0,1);
        ddyp=output(0,2);

    }
    else if (t>T_end_p_ey && t<=(TGait+TDs+TEnd)){
        yp=0;
        dyp=0;
        ddyp=0;
    }
    if (TStart==0 && t==0){
        //  yp=-polyval(Cy_p(6:10),Tc);
    }


    if (t<=T_st_p_sz){
        zp=ReferencePelvisHeight;
        dzp=0;
        ddzp=0;
    }
    else if (t>T_st_p_sz && t<=T_st_p_dz){
        MatrixXd output=GetAccVelPos(Cz_st_a,t,T_st_p_sz,5);
        zp=output(0,0);
        dzp=output(0,1);
        ddzp=output(0,2);
        // zp=polyval(Cz_st_a,t);
    }
    else if (t>T_st_p_dz && t<=T_st_p_ez){

        zp=MaxHeightPelvis;
        dzp=0;
        ddzp=0;
    }
    else if (t>T_st_p_ez && t<=TStart){
        MatrixXd output=GetAccVelPos(Cz_st_b,t,T_st_p_ez,5);
        zp=output(0,0);
        dzp=output(0,1);
        ddzp=output(0,2);
        //zp=polyval(Cz_st_b,t);
    }

    else if (t>TStart && t<=(TMinPelvisZ+TStart)){
        MatrixXd output=GetAccVelPos(Cz_p.row(1),t+Tc-TStart,TMaxPelvisZ,5);
        zp=output(0,0);
        dzp=output(0,1);
        ddzp=output(0,2);
        //  zp=polyval(Cz_p(5:8) ,t+Tc-T_st);
        //TMinPelvisZ, TMaxPelvisZ,Tc+TMinPelvisZ
    }
    else if  (t>(TMinPelvisZ+TStart) && t<=(TMaxPelvisZ+TStart)){
        MatrixXd output=GetAccVelPos(Cz_p.row(0),t-TStart,TMinPelvisZ,5);
        zp=output(0,0);
        dzp=output(0,1);
        ddzp=output(0,2);
        //zp=polyval(Cz_p(1:4), t-T_st);
    }
    else if (t>(TMaxPelvisZ+TStart) && t<=(Tc+TMinPelvisZ+TStart)){
        MatrixXd output=GetAccVelPos(Cz_p.row(1),t-TStart,TMaxPelvisZ,5);
        zp=output(0,0);
        dzp=output(0,1);
        ddzp=output(0,2);
        // zp=polyval(Cz_p(5:8), t-T_st);
    }
    else if (t>(Tc+TMinPelvisZ+TStart) && t<=(Tc+TMaxPelvisZ+TStart)){
        MatrixXd output=GetAccVelPos(Cz_p.row(0),t-TStart-Tc,TMinPelvisZ,5);
        zp=output(0,0);
        dzp=output(0,1);
        ddzp=output(0,2);
        //zp=polyval(Cz_p(1:4),t-Tc-T_st);
    }
    else if (t>(Tc+TMaxPelvisZ+TStart) && t<=(2*Tc+TStart)){
        MatrixXd output=GetAccVelPos(Cz_p.row(1),t-TStart-Tc,TMaxPelvisZ,5);
        zp=output(0,0);
        dzp=output(0,1);
        ddzp=output(0,2);
        //zp=polyval(Cz_p(5:8),t-Tc-T_st);
    }

    else if (t>TGait && t<=(TMinPelvisZ+TGait)){
        MatrixXd output=GetAccVelPos(Cz_p.row(1),t+Tc-TGait,TMaxPelvisZ,5);
        zp=output(0,0);
        dzp=output(0,1);
        ddzp=output(0,2);
        //zp=polyval(Cz_p(5:8),t+Tc-T_Gait);
    }

    else if (t>(TMinPelvisZ+TGait) && t<=(TDs+TGait)){
        MatrixXd output=GetAccVelPos(Cz_p.row(0),t-TGait,TMinPelvisZ,5);
        zp=output(0,0);
        dzp=output(0,1);
        ddzp=output(0,2);
        // zp=polyval(Cz_p(1:4),t-T_Gait);
    }

    else if (t>(TDs+TGait) && t<=T_end_p_sz){
        MatrixXd output=GetAccVelPos(Cz_end_a,t,TGait+TDs,5);
        zp=output(0,0);
        dzp=output(0,1);
        ddzp=output(0,2);
        // zp=polyval(Cz_end_a,t);
    }
    else if (t>T_end_p_sz && t<=T_end_p_dz){
        zp=MaxHeightPelvis;
        dzp=0;
        ddzp=0;
    }
    else if (t>T_end_p_dz && t<=T_end_p_ez){
        //       zp=zp_max;T_end_p_dz
        MatrixXd output=GetAccVelPos(Cz_end_b,t,T_end_p_dz,5);
        zp=output(0,0);
        dzp=output(0,1);
        ddzp=output(0,2);
        //  zp=polyval(Cz_end_b,t);
    }
    else if (t>T_end_p_ez && t<=(TGait+TDs+TEnd)){
        zp=MaxHeightPelvis;
        dzp=0;
        ddzp=0;
        //zp=z_Off_st_slope+((2*N_Stride+1)*Ds-x_Off_st_slope-x_z_p_ref;
        //
    }


    MatrixXd pelvis(9,1);
    pelvis<<xp,yp,zp,dxp,dyp,dzp,ddxp,ddyp,ddzp;
    return pelvis;

}






MatrixXd TaskSpaceOffline::AnkleTrajectory(double time){

    double  y_ar=-0.5*_pelvisLength;
    double y_al=0.5*_pelvisLength;
    double alpha_ar=0;
    double  alpha_al=0;
    double beta_al=0;
    double beta_ar=0;
    double  gama_ar=0;
    double gama_al=0;
    double x_ar;
    double z_ar;
    double x_al;
    double z_al;
    if (time<=TStart){
        x_ar=0;
        z_ar=_lenghtOfAnkle;
        x_al=0;
        z_al=_lenghtOfAnkle;
        if (time>=T_s_st){
            //x_al=polyval(C_st_x_al,time);

            MatrixXd output=GetAccVelPos(C_st_x_al.topRows(1),time,T_s_st,5);
            x_al=output(0,0);
            if (time<=TStart-T_s_st/2){

                MatrixXd output=GetAccVelPos(C_st_z_al.row(0),time,T_s_st,5);
                z_al=output(0,0);

                MatrixXd output1=GetAccVelPos(C_st_y_al.row(0),time,T_s_st,5);
                y_al=output1(0,0);
            }
            else{
                MatrixXd output=GetAccVelPos(C_st_z_al.row(1),time,TStart-T_s_st/2,5);
                z_al=output(0,0);
                MatrixXd output2=GetAccVelPos(C_st_y_al.row(1),time,TStart-T_s_st/2,5);
                y_al=output2(0,0);

            }
        }
    }

    else if (time<=MotionTime-TEnd){
        double tt=time-TStart;
        double N=floor(tt/(2*Tc));
        tt=fmod(tt,(2*Tc));
        if (tt<TDs){
            x_ar=2*N*StepLength;
            z_ar=_lenghtOfAnkle;
            x_al=(2*N+1)*StepLength;
            z_al=_lenghtOfAnkle;
            if (tt<TDs-T_beta){
                x_al=(2*N+1)*StepLength;
                z_al=_lenghtOfAnkle;
            }
            else{
                x_ar=2*N*StepLength;
                z_ar=_lenghtOfAnkle;
            }
            if (time<=TStart+T_beta){
                x_al=(2*N+1)*StepLength;
                z_al=_lenghtOfAnkle;
            }
            else if (time>=TStart+2*NStride*Tc+T_beta){
                x_ar=2*N*StepLength;
                z_ar=_lenghtOfAnkle;
            }
        }
        else if (tt<Tc){

            MatrixXd output1=GetAccVelPos(C_cy_x_ar,tt-TDs,0,5);
            x_ar=(2*N)*StepLength+output1(0,0);

            MatrixXd output2=GetAccVelPos(C_cy_z_ar.row(0),tt-TDs,0,5);
            z_ar=output2(0,0);

            MatrixXd output3=GetAccVelPos(C_cy_y_ar.row(0),tt-TDs,0,5);
            y_ar=output3(0,0);
            x_al=(2*N+1)*StepLength;
            z_al=_lenghtOfAnkle;


            if (tt<TDs+TSS/2){

            }
            else{

                MatrixXd output2=GetAccVelPos(C_cy_z_ar.row(1),tt-TDs,TSS/2,5);
                z_ar=output2(0,0);

                MatrixXd output3=GetAccVelPos(C_cy_y_ar.row(1),tt-TDs,TSS/2,5);
                y_ar=output3(0,0);
            }
        }

        else if (tt<Tc+TDs){
            x_ar=(2*N+2)*StepLength;
            z_ar=_lenghtOfAnkle;
            x_al=(2*N+1)*StepLength;
            z_al=_lenghtOfAnkle;

            if (tt<Tc+TDs-T_beta){
                x_ar=(2*N+2)*StepLength;
                z_ar=_lenghtOfAnkle;
            }
            else{
                x_al=(2*N+1)*StepLength;
                z_al=_lenghtOfAnkle;
            }
        }
        else{
            x_ar=(2*N+2)*StepLength;
            z_ar=_lenghtOfAnkle;

            MatrixXd output2=GetAccVelPos(C_cy_x_ar.topRows(1),tt-(Tc+TDs),0,5);
            double temp=output2(0,0);
            x_al=(2*N+1)*StepLength+temp;

            MatrixXd output3=GetAccVelPos(C_cy_z_ar.row(0),tt-(Tc+TDs),0,5);
            z_al=output3(0,0);

            MatrixXd output4=GetAccVelPos(C_cy_y_al.row(0),tt-(Tc+TDs),0,5);
            y_al=output4(0,0);

            if (tt<2*Tc-TSS/2){
            }

            else{
                MatrixXd output2=GetAccVelPos(C_cy_z_ar.row(1),tt-(Tc+TDs),TSS/2,5);
                z_al=output2(0,0);

                MatrixXd output3=GetAccVelPos(C_cy_y_al.row(1),tt-(Tc+TDs),TSS/2,5);
                y_al=output3(0,0);

            }

        }
    }

    else{
        double tt=time-(MotionTime-TEnd);
        if (tt<=T_end_a_e-T_end_a_s){
            MatrixXd output1=GetAccVelPos(C_end_x_ar.topRows(1),tt,0,5);
            double temp=output1(0,0);
            x_ar=2*NStride*StepLength+temp;

            MatrixXd output2=GetAccVelPos(C_end_z_ar.row(0),tt,0,5);
            z_ar=output2(0,0);

            x_al=(2*NStride+1)*StepLength;
            z_al=_lenghtOfAnkle;

            MatrixXd output4=GetAccVelPos(C_end_y_ar.row(0),tt,0,5);
            y_ar=output4(0,0);
            if (tt<=(T_end_a_e-T_end_a_s)/2){
            }

            else if (tt>=(T_end_a_e-T_end_a_s)/2){

                MatrixXd output2=GetAccVelPos(C_end_z_ar.row(1),tt, (T_end_a_e-T_end_a_s)/2,5);
                z_ar=output2(0,0);

                MatrixXd output4=GetAccVelPos(C_end_y_ar.row(1),tt, (T_end_a_e-T_end_a_s)/2,5);
                y_ar=output4(0,0);
                // z_ar=polyval(C_end_z_ar(:,2),tt);

            }
        }
        else{

            x_ar=(2*NStride+1)*StepLength;
            z_ar=_lenghtOfAnkle;

            x_al=(2*NStride+1)*StepLength;
            z_al=_lenghtOfAnkle;
        }
    }

    //    r_ar=[x_ar;y_ar;z_ar];
    //    q_ar=[alpha_ar;beta_ar;gama_ar];
    //    r_al=[x_al;y_al;z_al];
    //    q_al=[alpha_al;beta_al;gama_al];
    MatrixXd footpos(6,1);
    footpos<<x_al,y_al,z_al,x_ar,y_ar,z_ar;
    return footpos;

}



MatrixXd TaskSpaceOffline::GetAccVelPos(MatrixXd Coef,double time,double ti,int PolynomialOrder)
{
    int PolyNomialDegree=PolynomialOrder;
    MatrixXd T(PolyNomialDegree+1,1);
    T.fill(0);
    MatrixXd Diag(PolyNomialDegree+1,PolyNomialDegree+1);
    Diag.fill(0);
    for (int var = 0; var < PolyNomialDegree+1; var++) {
        T(var,0)=pow((time-ti),PolyNomialDegree-var);
        if (var!=0) {
            Diag.diagonal(1)(var-1,0)=PolyNomialDegree-var+1;

        }
    }

    MatrixXd x=Coef*T;
    double X=x(0,0);

    MatrixXd v=Coef*Diag*T;
    double V=v(0,0);

    MatrixXd a=Coef*Diag*Diag*T;
    double A=a(0,0);

    MatrixXd Output(1,3);
    Output<<X,V,A;
    return Output;
}
