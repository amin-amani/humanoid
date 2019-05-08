#include "taskspaceonline3.h"

TaskSpaceOnline3::TaskSpaceOnline3()
{

//    XofAnkleMaximumHeight=StepLength;
//    qDebug()<<XofAnkleMaximumHeight;
    NStride=3;
    LeftHipRollModification= 3.1;2.7;+.5;
    RightHipRollModification=3.1;2.7;
    FirstHipRollModification=3.1;2.7;
    HipPitchModification=1;//2;

    //PelvisRollRange=10*M_PI/180;

    YpMax=.105;.08;.12;
    Yd=YpMax;.07;.12;
//    Xe=0.04;
//    Xs=0;
//    StepLength=0.084;//0.0840000;
//    DesiredVelocity=.05;//0.050;

    Xe=0.02*1;
    Xs=-.006;-.01;0;
    StepLength=.0840000;
    DesiredVelocity=0.050;
    Tc=StepLength*3.6/DesiredVelocity;
    TDs=.4*Tc;
    TSS=Tc-TDs;
    XofAnkleMaximumHeight=StepLength*1.7;





    ReferencePelvisHeight=0.86;
    AnkleMaximumHeight=0.07;
    YStMax=1.0*YpMax;///.9 Start motion parameter in y direction
    YEndMax=1.0*YpMax;///.9End motion parameter in y direction
    NStep=NStride*2;



    //test
    YpMax=.123;.13;.12;105;
    Yd=YpMax;YStMax=YpMax;YEndMax=1.0*YpMax;
    StepLength=.1;.15;.2;.12;.3;
    DesiredVelocity=0.1;Tc=StepLength*3.6/DesiredVelocity;
    TDs=3;
    TSS=Tc-TDs;
    TSS=3;
    Tc=TSS+TDs;

    //FastWalk17Ordibehesht
    YpMax=.123;.123;.13;.12;105;
    Yd=YpMax;YStMax=YpMax;YEndMax=1.0*YpMax;
    StepLength=.15;.2;.12;.3;
    DesiredVelocity=0.1;Tc=StepLength*3.6/DesiredVelocity;
    TDs=2.7;
    TSS=Tc-TDs;
    TSS=2;
    Tc=TSS+TDs;
    AnkleMaximumHeight=0.025;
    ReferencePelvisHeight=.91;.9;.89;0.86+.02-.055;



//    TDs=2;//2.5;
//    TSS=StepLength*3.6/DesiredVelocity-TDs;
//    Tc=TSS+TDs;

    Tm=0.35*TSS;
    TGait=TStart+NStride*2*Tc;
    MotionTime=TStart+NStride*2*Tc+TDs+TEnd;
    TMinPelvisY=0.5*TDs; // The time that pelvis reaches its minimum distance in y direction
    TMaxAnkle=TDs+0.35*TSS;//0.53 % The time that ankle reaches its maximum distance in z direction
    TMaxPelvisY=TDs+0.5*TSS; // The time that pelvis reaches its maximum distance in y direction
    // last step: timing parameter of pelvis motion
    T_end_of_first_SS=1;
    //    T_end_of_SS=.9;
    T_end_of_SS=.5;//TSS*.3;
    T_end_of_last_SS=1;

    h_end_of_SS=.015;

    T_end_p_sx_rel=TDs+0.25*TEnd;
    T_end_p_sx=TGait+TDs+0.25*TEnd;
    T_end_p_sy=TGait+TDs+0.2*TEnd;
    T_end_p_dy=TGait+TDs+0.4*TEnd;
    T_end_p_ey=TGait+TDs+0.8*TEnd;
    // last step: timing parameter of ankle motion
    T_end_a_s=TGait+TDs;
    T_end_a_e=TGait+TDs+0.5*TEnd;
    T_end_a_d=TGait+TDs+.6*(T_end_a_e-T_end_a_s);0.45;
    // first step: timing parameter of pelvis motion
    T_st_p_sy=0.2*TStart;
    T_st_p_dy=0.5*TStart;//0.65*TStart;
    T_st_p_ey=0.8*TStart;
    T_st_p_sx=0.7*TStart;
    // first step: timing parameter of ankle motion
    T_s_st=.5*TStart;
    T_st_a_d=T_s_st+0.45*(TStart-T_s_st);











    CoeffArrayAnkle();
    CoeffArrayPelvis();
}


double TaskSpaceOnline3::RollCharge(double t,double t_start,double t_end,double magnitude){
    double D_time_charge = t_end-t_start;
    MatrixXd Ct(1,2);    Ct<<0 ,D_time_charge;
    MatrixXd Cp(1,2);    Cp<<0, magnitude;
    MatrixXd Cv(1,2);    Cv<<0 ,0;
    MatrixXd Ca(1,2);    Ca<<0 ,0;
    MatrixXd C_roll=CoefOffline.Coefficient(Ct,Cp,Cv,Ca);
    if(t<=t_start){return 0;}
    else if((t-t_start)>=D_time_charge){return magnitude;}
    else{    MatrixXd output=CoefOffline.GetAccVelPos(C_roll,t-t_start,0,5);
        return output(0,0);}
}

double TaskSpaceOnline3::RollDecharge(double t,double t_start,double t_end,double magnitude){
    double D_time_decharge = t_end-t_start;
    MatrixXd Ct(1,2);    Ct<<0 ,D_time_decharge;
    MatrixXd Cp(1,2);    Cp<< 0, -magnitude;
    MatrixXd Cv(1,2);    Cv<<0 ,0;
    MatrixXd Ca(1,2);    Ca<<0 ,0;
    MatrixXd C_roll=CoefOffline.Coefficient(Ct,Cp,Cv,Ca);
    if(t<=t_start){return 0;}
    else if((t-t_start)>=D_time_decharge){return -magnitude;}
    else{    MatrixXd output=CoefOffline.GetAccVelPos(C_roll,t-t_start,0,5);
        return output(0,0);}
}

MatrixXd TaskSpaceOnline3::RollAngleModification(double time){
    double N;
    double t;
    double rollR=0;
    double rollL=0;
    double dt=0;
    double dt2=-0.20;
    double ChargeCoeffLeftStart =0.8;.4+dt;// 0.4;          //after DS start
    double ChargeCoeffLeftEnd =1.3;1;// 1.5;              //<1 before DS ends; >1 after SS start
    double DechargeCoeffLeftStart = 0.4+dt;//.4;        //after DS start
    double DechargeCoeffLeftEnd = 1+dt2;            //<1 before DS ends; >1 after SS start
    double ChargeCoeffLeftStartEndPhase =0.8; 0.4+dt;  //after DS start
    double ChargeCoeffLeftEndEndPhase =1.3; 1;      //<1 before DS ends; >1 after SS start
    double DechargeCoeffLeftStartEndPhase = 0.6;//must be bigger than 0.5; end foot pairing DS
    double DechargeCoeffLeftEndEndPhase = 0.8;  //must be greata

    double ChargeCoeffRightStartStartPhase = 0.3;   //while reducing height(0.5 means minimum height)
    double ChargeCoeffRightEndStartPhase = 0.7;     //almot maximum ankle height
    double DechargeCoeffRightStartStartPhase = 0.4+dt; //after DS start
    double DechargeCoeffRightEndStartPhase = 1+dt2;     //<1 before DS ends; >1 after SS start
    double ChargeCoeffRightStart =0.8;.4+dt;// 0.4;             //after DS start
    double ChargeCoeffRightEnd =1.3; 1;// 1.5;                 //<1 before DS ends; >1 after SS start
    double DechargeCoeffRightStart = 0.4+dt;//.4;           //after DS start
    double DechargeCoeffRightEnd = 1+dt2;               //<1 before DS ends; >1 after SS start

    if (time<=TStart){
        N=0;
        t=time;
    }
    else if (time>TStart && time<TGait){
        N=floor((time-TStart)/(2*Tc));
        t=fmod((time-TStart),2*Tc)+TStart;
    }
    else if (time>=TGait){
        t = TStart + 2*Tc + time - TGait;
    }


    double R_1=0.6;
    double  D_time=R_1*TDs;
    double D_teta_r;
    if(time<TStart+DechargeCoeffRightEndStartPhase*TDs){D_teta_r=-1*FirstHipRollModification*(M_PI/180);}
    else{D_teta_r=-1*RightHipRollModification*(M_PI/180);}
    double D_teta_l=LeftHipRollModification*(M_PI/180);

    double extra_D_theta_l=0*M_PI/180;

    //VectorXd t_left_charge;
    //VectorXd t_left_decharge;
    //VectorXd t_right_charge;
    //VectorXd t_right_decharge;


    //for (int i = 0; i <t_left_charge.rows() ; ++i) {rollL=rollL+RollCharge(t,t_left_charge(i),D_teta_l);}
    //for (int i = 0; i <t_left_decharge.rows() ; ++i) {rollL=rollL+RollCharge(t,t_left_decharge(i),D_teta_l);}
    //for (int i = 0; i <t_right_charge.rows() ; ++i) {rollR=rollR+RollCharge(t,t_right_charge(i),D_teta_r);}
    //for (int i = 0; i <t_right_decharge.rows() ; ++i) {rollR=rollR+RollCharge(t,t_right_decharge(i),D_teta_r);}
    rollL=rollL+RollCharge(t,TStart+ChargeCoeffLeftStart*TDs,TStart+ChargeCoeffLeftEnd*TDs,D_teta_l)+
            RollDecharge(t,TStart+Tc+DechargeCoeffLeftStart*TDs,TStart+Tc+DechargeCoeffLeftEnd*TDs,D_teta_l)+
            RollCharge(t,TStart+2*Tc+ChargeCoeffLeftStartEndPhase*TDs,TStart+2*Tc+ChargeCoeffLeftEndEndPhase*TDs,D_teta_l)+
            RollDecharge(t,TStart+2*Tc+TDs+DechargeCoeffLeftStartEndPhase*TEnd,TStart+2*Tc+TDs+DechargeCoeffLeftEndEndPhase*TEnd,D_teta_l);

    rollR=rollR+RollCharge(t,ChargeCoeffRightStartStartPhase*TStart,ChargeCoeffRightEndStartPhase*TStart,D_teta_r)+
            RollDecharge(t,TStart+DechargeCoeffRightStartStartPhase*TDs,TStart+DechargeCoeffRightEndStartPhase*TDs,D_teta_r)+
            RollCharge(t,TStart+Tc+ChargeCoeffRightStart*TDs,TStart+Tc+ChargeCoeffRightEnd*TDs,D_teta_r)+
            RollDecharge(t,TStart+2*Tc+DechargeCoeffRightStart*TDs,TStart+2*Tc+DechargeCoeffRightEnd*TDs,D_teta_r);






    MatrixXd RollMat(2,1);
    RollMat<<rollR,rollL;
    return RollMat;}

        double TaskSpaceOnline3::PelvisRoll(double time){
            double x=1;
            return x;
            }

void TaskSpaceOnline3::CoeffSideStartEnd(){
        //***** x start
        MatrixXd Cx_st_iTime_al(1,2);
        Cx_st_iTime_al<<T_s_st ,TStart-T_end_of_first_SS;
        MatrixXd Cx_st_iPos_al(1,2);
        Cx_st_iPos_al<<0, 2*StepLength;
        MatrixXd Cx_st_iVel_al(1,2);
        Cx_st_iVel_al<<0, 0;
        MatrixXd Cx_st_iAcc_al(1,2);
        Cx_st_iAcc_al<<0, 0;
        C_st_x_al=CoefOffline.Coefficient(Cx_st_iTime_al,Cx_st_iPos_al,Cx_st_iVel_al,Cx_st_iAcc_al);

        //***** x end
        MatrixXd C_end_iTime_ar(1,2);
        C_end_iTime_ar<<0, (T_end_a_e-T_end_a_s-T_end_of_last_SS);
        MatrixXd C_end_iPos_ar(1,2);
        C_end_iPos_ar<<0, 2*StepLength;
        MatrixXd C_end_iVel_ar(1,2);
        C_end_iVel_ar<<0, 0;
        MatrixXd C_end_iAcc_ar(1,2);
        C_end_iAcc_ar<<0, 0;
        C_end_x_ar=CoefOffline.Coefficient(C_end_iTime_ar,C_end_iPos_ar,C_end_iVel_ar,C_end_iAcc_ar);
        side_extra_step_length=true;
}

void TaskSpaceOnline3::CoeffArrayAnkle(){
double vz=-.04;
    //**************** start ***************************//
    //***** x start
    MatrixXd Cx_st_iTime_al(1,2);
    Cx_st_iTime_al<<T_s_st ,TStart-T_end_of_first_SS;
    MatrixXd Cx_st_iPos_al(1,2);
    Cx_st_iPos_al<<0, StepLength;
    MatrixXd Cx_st_iVel_al(1,2);
    Cx_st_iVel_al<<0, 0;
    MatrixXd Cx_st_iAcc_al(1,2);
    Cx_st_iAcc_al<<0, 0;
    C_st_x_al=CoefOffline.Coefficient(Cx_st_iTime_al,Cx_st_iPos_al,Cx_st_iVel_al,Cx_st_iAcc_al);

    //***** z start 1
    MatrixXd C_st_iTime(1,3);
    C_st_iTime<<T_s_st, TStart-T_s_st/2 ,TStart-T_end_of_first_SS;
    MatrixXd C_st_iPos(1,3);
    C_st_iPos<<0, AnkleMaximumHeight,h_end_of_SS;
    MatrixXd C_st_iVel(1,3);
    C_st_iVel<<0 ,INFINITY, vz;
    MatrixXd C_st_iAcc(1,3);
    C_st_iAcc<<0 ,INFINITY, 0;
    C_st_z_al=CoefOffline.Coefficient(C_st_iTime,C_st_iPos,C_st_iVel,C_st_iAcc);

    //***** z start 2
    MatrixXd C_st_iTime_e(1,2);
    C_st_iTime_e<<TStart-T_end_of_first_SS,TStart;
    MatrixXd C_st_iPos_e(1,2);
    C_st_iPos_e<<h_end_of_SS,0;
    MatrixXd C_st_iVel_e(1,2);
    C_st_iVel_e<<vz , 0;
    MatrixXd C_st_iAcc_e(1,2);
    C_st_iAcc_e<<0 , 0;
    C_st_z_al_end_of_SS=CoefOffline.Coefficient(C_st_iTime_e,C_st_iPos_e,C_st_iVel_e,C_st_iAcc_e);

    //***** y start
    MatrixXd Cy_st_iTime(1,3);
    Cy_st_iTime<<T_s_st, TStart-T_s_st/2 ,TStart;
    MatrixXd Cy_st_iPos(1,3);
    Cy_st_iPos<<0,YOffsetOfAnkletrajectory,0;
    MatrixXd Cy_st_iVel(1,3);
    Cy_st_iVel<<0 ,INFINITY, 0;
    MatrixXd Cy_st_iAcc(1,3);
    Cy_st_iAcc<<0 ,INFINITY, 0;
    C_st_y_al=CoefOffline.Coefficient(Cy_st_iTime,Cy_st_iPos,Cy_st_iVel,Cy_st_iAcc);

    //**************** cycle ***************************//

    //***** x cycle
    MatrixXd C_cy_iTime_ar(1,3);
    C_cy_iTime_ar<<0, Tm, TSS-T_end_of_SS;
    MatrixXd C_cy_iPos_ar(1,3);
    C_cy_iPos_ar<<0, StepLength , 2*StepLength;
    MatrixXd C_cy_iVel_ar(1,3);
    C_cy_iVel_ar<<0,INFINITY, 0;
    MatrixXd C_cy_iAcc_ar(1,3);
    C_cy_iAcc_ar<<0,INFINITY, 0;
    C_cy_x_ar=CoefOffline.Coefficient(C_cy_iTime_ar,C_cy_iPos_ar,C_cy_iVel_ar,C_cy_iAcc_ar);

    //***** z cycle1
    MatrixXd C_cy_iTime(1,3);
    C_cy_iTime<<0 ,Tm, TSS-T_end_of_SS;
    MatrixXd C_cy_iPos(1,3);
    C_cy_iPos<<0,AnkleMaximumHeight, h_end_of_SS;
    MatrixXd C_cy_iVel(1,3);
    C_cy_iVel<<0 ,INFINITY, vz;
    MatrixXd C_cy_iAcc(1,3);
    C_cy_iAcc<<0 ,INFINITY, 0;
    C_cy_z_ar=CoefOffline.Coefficient(C_cy_iTime,C_cy_iPos,C_cy_iVel,C_cy_iAcc);

    //***** z cycle2
    MatrixXd C_cy_iTime_e(1,2);
    C_cy_iTime_e<<TSS-T_end_of_SS, TSS;
    MatrixXd C_cy_iPos_e(1,2);
    C_cy_iPos_e<< h_end_of_SS, 0;
    MatrixXd C_cy_iVel_e(1,2);
    C_cy_iVel_e<<vz , 0;
    MatrixXd C_cy_iAcc_e(1,2);
    C_cy_iAcc_e<<0 , 0;
    C_cy_z_ar_end_of_SS=CoefOffline.Coefficient(C_cy_iTime_e,C_cy_iPos_e,C_cy_iVel_e,C_cy_iAcc_e);

    //***** y cycle
    MatrixXd Cy_cy_iTime(1,3);
    Cy_cy_iTime<<0 ,Tm, TSS;
    MatrixXd Cy_cy_iPos(1,3);
    Cy_cy_iPos<<0,YOffsetOfAnkletrajectory, 0;
    MatrixXd Cy_cy_iVel(1,3);
    Cy_cy_iVel<<0 ,INFINITY, 0;
    MatrixXd Cy_cy_iAcc(1,3);
    Cy_cy_iAcc<<0 ,INFINITY, 0;
    C_cy_y_ar=CoefOffline.Coefficient(Cy_cy_iTime,Cy_cy_iPos,Cy_cy_iVel,Cy_cy_iAcc);
    //*************************** end *************************************
    //***** x end
    MatrixXd C_end_iTime_ar(1,2);
    C_end_iTime_ar<<0, (T_end_a_e-T_end_a_s-T_end_of_last_SS);
    MatrixXd C_end_iPos_ar(1,2);
    C_end_iPos_ar<<0, 1*StepLength;
    MatrixXd C_end_iVel_ar(1,2);
    C_end_iVel_ar<<0, 0;
    MatrixXd C_end_iAcc_ar(1,2);
    C_end_iAcc_ar<<0, 0;
    C_end_x_ar=CoefOffline.Coefficient(C_end_iTime_ar,C_end_iPos_ar,C_end_iVel_ar,C_end_iAcc_ar);

    //***** z end 1
    MatrixXd C_end_z_iTime(1,3);
    C_end_z_iTime<<0 ,(T_end_a_e-T_end_a_s)/2 ,T_end_a_e-T_end_a_s-T_end_of_last_SS;
    MatrixXd C_end_z_iPos(1,3);
    C_end_z_iPos<<0, AnkleMaximumHeight, h_end_of_SS;
    MatrixXd C_end_z_iVel(1,3);
    C_end_z_iVel<<0 ,INFINITY, vz;
    MatrixXd C_end_z_iAcc(1,3);
    C_end_z_iAcc<<0 ,INFINITY, 0;
    C_end_z_ar=CoefOffline.Coefficient(C_end_z_iTime,C_end_z_iPos,C_end_z_iVel,C_end_z_iAcc);

    //***** z end 2
    MatrixXd C_end_z_iTime_e(1,2);
    C_end_z_iTime_e<<T_end_a_e-T_end_a_s-T_end_of_last_SS ,T_end_a_e-T_end_a_s;
    MatrixXd C_end_z_iPos_e(1,2);
    C_end_z_iPos_e<<h_end_of_SS, 0;
    MatrixXd C_end_z_iVel_e(1,2);
    C_end_z_iVel_e<<vz , 0;
    MatrixXd C_end_z_iAcc_e(1,2);
    C_end_z_iAcc_e<<0 , 0;
    C_end_z_ar_end_of_SS=CoefOffline.Coefficient(C_end_z_iTime_e,C_end_z_iPos_e,C_end_z_iVel_e,C_end_z_iAcc_e);

    //***** y end
    MatrixXd C_end_y_iTime(1,3);
    C_end_y_iTime<<0 ,(T_end_a_e-T_end_a_s)/2 ,T_end_a_e-T_end_a_s;
    MatrixXd C_end_y_iPos(1,3);
    C_end_y_iPos<<0,YOffsetOfAnkletrajectory, 0;
    MatrixXd C_end_y_iVel(1,3);
    C_end_y_iVel<<0 ,INFINITY, 0;
    MatrixXd C_end_y_iAcc(1,3);
    C_end_y_iAcc<<0 ,INFINITY, 0;
    C_end_y_ar=CoefOffline.Coefficient(C_end_y_iTime,C_end_y_iPos,C_end_y_iVel,C_end_y_iAcc);

}

void TaskSpaceOnline3::CoeffArrayPelvis(){
double vx=(!side_extra_step_length)*DesiredVelocity/3.6/2;
    //------------------Coefficient of cyclic motion in X direction--------------------
    MatrixXd ord(1,2);
    ord << 4,4;//3,3
    MatrixXd ttt(1,3);
    ttt <<0 ,TDs, Tc;
    MatrixXd con(3,3);
    con<<Xe, StepLength-Xs, StepLength+Xe,vx, INFINITY ,vx,0, INFINITY ,0;//con<<Xe, StepLength-Xs, StepLength+Xe,INFINITY, INFINITY ,INFINITY,INFINITY, INFINITY ,INFINITY;

    Cx_p_i.resize(2,6);
    Cx_p_i.fill(0);
    Cx_p_i.block(0,1,2,5)=CoefOffline.Coefficient1(ttt,ord,con,0.1).transpose();// .block(0,2,2,4)
    Cx_p.resize(1,12);
    Cx_p.fill(0);
    Cx_p.block(0,0,1,6)=Cx_p_i.row(0);
    Cx_p.block(0,6,1,6)=Cx_p_i.row(1);

    //------------------Coefficient of cyclic Pelvis motion in Y direction--------------------
    MatrixXd ordY(1,8);
    ordY << 4,4,5,5,4,4,5,5;//  ordY << 3,3,4,4,3,3,4,4;//old
    MatrixXd tttY(1,9);
    tttY <<0,TMinPelvisY,TDs,TDs+TSS/2,Tc,Tc+TDs/2,Tc+TDs,Tc+TDs+TSS/2,2*Tc;
    MatrixXd conY(3,9);
    conY<<-1*Yd,0,Yd,YpMax,Yd,0,-1*Yd,-1*YpMax,-1*Yd,0, INFINITY,INFINITY, INFINITY ,0,INFINITY,INFINITY,0,0   ,0, INFINITY,INFINITY, INFINITY ,0,INFINITY,INFINITY,0,0;

    Cy_p_i.resize(8,6);
    Cy_p_i.fill(0);
    Cy_p_i.block(0,0,8,6)=CoefOffline.Coefficient1(tttY,ordY,conY,0.1).transpose();//.block(0,1,8,5)
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



    //----------------Coefficient of Start of Pelvis motion in x direction---------------------
    MatrixXd Cx_st_pTime(1,2);
    Cx_st_pTime<<T_st_p_sx ,TStart;
    MatrixXd Cx_st_pPos(1,2);
    Cx_st_pPos<<0, Xe;
    MatrixXd Cx_st_pVel(1,2);
    Cx_st_pVel<<0, vx;//Cx_st_pVel<<0 ,Cx_p(0,4);
    MatrixXd Cx_st_pAccel(1,2);
    Cx_st_pAccel<<0 ,0;//Cx_st_pAccel<<0 ,2*Cx_p(0,3);
    Cx_st_p=CoefOffline.Coefficient(Cx_st_pTime,Cx_st_pPos,Cx_st_pVel,Cx_st_pAccel);

    // ----------------Coefficient of End of Pelvis motion in x direction----------------------

    MatrixXd Cx_end_pTime(1,2);
    Cx_end_pTime<<(TDs),T_end_p_sx_rel ;
    MatrixXd Cx_end_pPos(1,2);
    Cx_end_pPos<<(NStep+1)*StepLength-Xs, (NStep+1)*StepLength+(side_extra_step_length)*StepLength;
    MatrixXd Cx_end_pVel(1,2);
    Cx_end_pVel<<vx, 0;//Cx_end_pVel<<5*Cx_p(0,0)*pow(TDs,4)+4*Cx_p(0,1)*pow(TDs,3)+3*Cx_p(0,2)*pow(TDs,2)+2*Cx_p(0,3)*pow(TDs,1)+Cx_p(0,4), 0;
    MatrixXd Cx_end_pAccel(1,2);
    Cx_end_pAccel<<0, 0;//Cx_end_pAccel<<5*4*Cx_p(0,0)*pow(TDs,4)+4*3*Cx_p(0,1)*pow(TDs,3)+3*2*Cx_p(0,2)*pow(TDs,2)+2*Cx_p(0,3), 0;//like the last moment of first part of trajectory of cycle
    Cx_end_p=CoefOffline.Coefficient(Cx_end_pTime,Cx_end_pPos,Cx_end_pVel,Cx_end_pAccel);


    //--------------Coefficient of Start of Pelvis motion in y direction----------------
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
    MatrixXd Cy_st_peVel(1,2);   // Cy_st_peVel<<0 ,5*Cy_p(0,0)*pow(0,4)+4*Cy_p(0,1)*pow(0,3)+3*Cy_p(0,2)*pow(0,2)+2*Cy_p(0,3)*pow(0,1)+Cy_p(0,4);
    Cy_st_peVel<<0 ,0;
    MatrixXd Cy_st_peAccel(1,2);    //Cy_st_peAccel<<0 ,2*Cy_p(0,3);
    Cy_st_peAccel<<0 ,0;
    Cy_st_pb=CoefOffline.Coefficient(Cy_st_peTime,Cy_st_pePos,Cy_st_peVel,Cy_st_peAccel);
    // -------------------Coefficient of End of Pelvis motion in y direction--------------
    MatrixXd Cy_end_psTime(1,2);
    Cy_end_psTime<<TGait+TDs, T_end_p_sy;
    MatrixXd Cy_end_psPos(1,2);
    Cy_end_psPos<<Yd, YEndMax;
    MatrixXd Cy_end_psVel(1,2);//    Cy_end_psVel<<5*Cy_p_i(1,0)*pow(TDs,4)+4*Cy_p_i(1,1)*pow(TDs,3)+3*Cy_p_i(1,2)*pow(TDs,2)+2*Cy_p_i(1,3)*pow(TDs,1)+Cy_p_i(1,4),0;
    Cy_end_psVel<<0,0;
    MatrixXd Cy_end_psAccel(1,2);//    Cy_end_psAccel<< 5*4*Cy_p_i(1,0)*pow(TDs,4)+4*3*Cy_p_i(1,1)*pow(TDs,3)+3*2*Cy_p_i(1,2)*pow(TDs,2)+2*Cy_p_i(1,3), 0;
    Cy_end_psAccel<<0,0;
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

}

MatrixXd TaskSpaceOnline3::PelvisTrajectory(double time){


    double N,t,xp,yp,zp,dxp,dyp,dzp,ddxp,ddyp,ddzp,yawp;
//determining N
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
//*******************   x

    if (t<=T_st_p_sx){
        xp=0;
        dxp=0;
        ddxp=0;
        DoubleSupport=true;

    }
    else if (t>T_st_p_sx  && t<=TStart){
        MatrixXd output=CoefOffline.GetAccVelPos(Cx_st_p,t,T_st_p_sx,5);
        xp=output(0,0);
        dxp=output(0,1);
        ddxp=output(0,2);
        DoubleSupport=true;
    }
    else if (t>TStart &&  t<=(TDs+TStart)){
        MatrixXd output=CoefOffline.GetAccVelPos(Cx_p_i.row(0),t-TStart,0,5);
        xp=output(0,0);
        dxp=output(0,1);
        ddxp=output(0,2);
        DoubleSupport=true;
    }
    else if (t>(TDs+TStart) && t<=(Tc+TStart)){
        MatrixXd output=CoefOffline.GetAccVelPos(Cx_p_i.row(1),t-TStart,0,5);
        xp=output(0,0);
        dxp=output(0,1);
        ddxp=output(0,2);
        DoubleSupport=false;
    }
    else if (t>(Tc+TStart) && t<=(Tc+TDs+TStart)){
        MatrixXd output=CoefOffline.GetAccVelPos(Cx_p_i.row(0),t-Tc-TStart,0,5);
        xp=output(0,0)+StepLength;
        dxp=output(0,1);
        ddxp=output(0,2);
        DoubleSupport=true;
    }
    else if (t>(Tc+TDs+TStart) && t<=(2*Tc+TStart)){
        MatrixXd output=CoefOffline.GetAccVelPos(Cx_p_i.row(1),t-Tc-TStart,0,5);
        xp=output(0,0)+StepLength;
        dxp=output(0,1);
        ddxp=output(0,2);
        DoubleSupport=false;
    }
    else if (t>TGait && t<(TGait+TDs)){
        MatrixXd output=CoefOffline.GetAccVelPos(Cx_p_i.row(0),t-TGait,0,5);
        xp=output(0,0)+2*NStride*StepLength;
        dxp=output(0,1);
        ddxp=output(0,2);
        DoubleSupport=true;
    }
    else if (t>=(TGait+TDs) && t<T_end_p_sx){
        MatrixXd output=CoefOffline.GetAccVelPos(Cx_end_p,t,(TGait+TDs),5);
        xp=output(0,0);
        dxp=output(0,1);
        ddxp=output(0,2);
        DoubleSupport=true;
    }
    else if (t>=T_end_p_sx  && t<=(TGait+TDs+TEnd)){
        xp=(2*NStride+1)*StepLength+(side_extra_step_length)*StepLength;
        dxp=0;
        ddxp=0;
        DoubleSupport=true;
    }

    if (TStart==0 && t==0){
        xp=Cx_p(4);
        dxp=0;
        ddxp=0;
    }

    xp=xp+2*StepLength*N;


    //*******************   y
    if(t<=T_st_p_sy){
        yp=0;
        dyp=0;
        ddyp=0;
    }
    else if (t>T_st_p_sy && t<=T_st_p_dy){
        MatrixXd output=CoefOffline.GetAccVelPos(Cy_st_pa,t,T_st_p_sy,5);
        yp=output(0,0);
        dyp=output(0,1);
        ddyp=output(0,2);
    }
    else if (t>T_st_p_dy && t<=T_st_p_ey){
        yp=-1*YStMax;
        dyp=0;
        ddyp=0;
    }
    else if (t>T_st_p_ey && t<=TStart){

        MatrixXd output=CoefOffline.GetAccVelPos(Cy_st_pb,t,T_st_p_ey,5);
        yp=output(0,0);
        dyp=output(0,1);
        ddyp=output(0,2);
    }
    else if (t>TStart && t<=(TMinPelvisY+TStart)){
        MatrixXd output=CoefOffline.GetAccVelPos(Cy_p_i.row(0),t-TStart,0,5);
        yp=output(0,0);
        dyp=output(0,1);
        ddyp=output(0,2);
    }
    else if (t>(TMinPelvisY+TStart) && t<=(TDs+TStart)){
        MatrixXd output=CoefOffline.GetAccVelPos(Cy_p_i.row(1),t-TStart,0,5);
        yp=output(0,0);
        dyp=output(0,1);
        ddyp=output(0,2);
    }
    else if (t>(TDs+TStart) && t<=(TMaxPelvisY+TStart)){
        MatrixXd output=CoefOffline.GetAccVelPos(Cy_p_i.row(2),t-TStart,0,5);
        yp=output(0,0);
        dyp=output(0,1);
        ddyp=output(0,2);
    }
    else if (t>(TMaxPelvisY+TStart) && t<=(Tc+TStart)){
        MatrixXd output=CoefOffline.GetAccVelPos(Cy_p_i.row(3),t-TStart,0,5);
        yp=output(0,0);
        dyp=output(0,1);
        ddyp=output(0,2);
    }
    else if (t>(Tc+TStart) && t<=(Tc+TMinPelvisY+TStart)){
        MatrixXd output=CoefOffline.GetAccVelPos(Cy_p_i.row(4),t-TStart,0,5);
        yp=output(0,0);
        dyp=output(0,1);
        ddyp=output(0,2);
    }
    else if (t>(Tc+TMinPelvisY+TStart) && t<=(Tc+TDs+TStart)){
        MatrixXd output=CoefOffline.GetAccVelPos(Cy_p_i.row(5),t-TStart,0,5);
        yp=output(0,0);
        dyp=output(0,1);
        ddyp=output(0,2);
    }
    else if (t>(Tc+TDs+TStart) && t<=(Tc+TMaxPelvisY+TStart)){
        MatrixXd output=CoefOffline.GetAccVelPos(Cy_p_i.row(6),t-TStart,0,5);
        yp=output(0,0);
        dyp=output(0,1);
        ddyp=output(0,2);
    }
    else if (t>(Tc+TMaxPelvisY+TStart) && t<=(2*Tc+TStart)){
        MatrixXd output=CoefOffline.GetAccVelPos(Cy_p_i.row(7),t-TStart,0,5);
        yp=output(0,0);
        dyp=output(0,1);
        ddyp=output(0,2);
    }
    else if (t>TGait && t<=(TMinPelvisY+TGait)){
        MatrixXd output=CoefOffline.GetAccVelPos(Cy_p_i.row(0),t-TGait,0,5);
        yp=output(0,0);
        dyp=output(0,1);
        ddyp=output(0,2);
    }
    else if (t>(TMinPelvisY+TGait) && t<=(TDs+TGait)){
        MatrixXd output=CoefOffline.GetAccVelPos(Cy_p_i.row(1),t-TGait,0,5);
        yp=output(0,0);
        ddyp=output(0,2);
        dyp=output(0,1);
    }
    else if (t>(TDs+TGait) && t<=T_end_p_sy){
        MatrixXd output=CoefOffline.GetAccVelPos(Cy_end_pa,t,TGait+TDs,5);
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
        MatrixXd output=CoefOffline.GetAccVelPos(Cy_end_pb,t,T_end_p_dy,5);
        yp=output(0,0);
        dyp=output(0,1);
        ddyp=output(0,2);

    }
    else if (t>T_end_p_ey && t<=(TGait+TDs+TEnd)){
        yp=0;
        dyp=0;
        ddyp=0;
    }

    zp=ReferencePelvisHeight;
    dzp=0;
    ddzp=0;

   // qDebug()<<"t="<<time<<"\txp="<<xp<<"\typ="<<yp;

    MatrixXd pelvis(10,1);
    pelvis<<xp,yp,zp,dxp,dyp,dzp,ddxp,ddyp,ddzp,yawp;
//    qDebug()<<"xp="<<
    return pelvis;
}


MatrixXd TaskSpaceOnline3::AnkleTrajectory(double time,int n ,double localtiming){
    LeftFootOrientationAdaptator=false;
    RightFootOrientationAdaptator=false;

    double y_ar=-0.5*_pelvisLength;
    double y_al=0.5*_pelvisLength;
    double x_ar,x_al,z_ar,z_al;
    double yaw_al=0;
    double yaw_ar=0;


    if (n==1){//left foot moves in first step

        x_ar=0;
        z_ar=_lenghtOfAnkle;
        x_al=0;
        z_al=_lenghtOfAnkle;

        if (localtiming>=T_s_st && localtiming<=TStart-T_end_of_first_SS){
            MatrixXd output=CoefOffline.GetAccVelPos(C_st_x_al.row(0),localtiming,T_s_st,5);
            x_al=output(0,0);


            if (localtiming<=(TStart-T_s_st/2)){
                MatrixXd output=CoefOffline.GetAccVelPos(C_st_z_al.row(0),localtiming,T_s_st,5);
                z_al=_lenghtOfAnkle+output(0,0);

                MatrixXd output1=CoefOffline.GetAccVelPos(C_st_y_al.row(0),localtiming,T_s_st,5);
                y_al=0.5*_pelvisLength+output1(0,0);

            }
            else if (localtiming<=(TStart-T_end_of_first_SS)){
                MatrixXd output=CoefOffline.GetAccVelPos(C_st_z_al.row(1),localtiming,TStart-T_s_st/2,5);
                z_al=_lenghtOfAnkle+output(0,0);

                MatrixXd output2=CoefOffline.GetAccVelPos(C_st_y_al.row(1),time,TStart-T_s_st/2,5);
                y_al=0.5*_pelvisLength+output2(0,0);

            }
        }
        else if(localtiming>TStart-T_end_of_first_SS){
            LeftFootOrientationAdaptator=true;
            MatrixXd output1=CoefOffline.GetAccVelPos(C_st_x_al.row(0),TStart-T_end_of_first_SS,T_s_st,5);
            x_al=output1(0,0);
            //x_al=StepLength;
            y_al=0.5*_pelvisLength;
            MatrixXd output=CoefOffline.GetAccVelPos(C_st_z_al_end_of_SS.row(0),localtiming,TStart-T_end_of_first_SS,5);
            z_al=_lenghtOfAnkle+output(0,0);
        }
    }

    if (n!=1 && n!=(NStep+2)){//cyclic walking

        n=n-1;

        footIndex=fmod(n,2);// shows which foot is swing foot (in cyclic mode left foots is swinging in the even steps(N))
        //it means whenever the footIndex is 0 the left foots will go to the swing mode

        currentLeftFootX2=(n-fmod(n+1,2))*StepLength+side_extra_step_length*StepLength;
       // currentLeftFootY2;
        currentLeftFootZ=0.112;

        currentRightFootX2=(n-fmod(n,2))*StepLength;
        //currentRightFootY2;
        currentRightFootZ=0.112;


        if (localtiming<TDs){// double support of cyclic walking

            x_al=currentLeftFootX2;
            y_al=currentLeftFootY2;
            z_al=currentLeftFootZ;

            x_ar=currentRightFootX2;
            y_ar=currentRightFootY2;
            z_ar=currentRightFootZ;

        }

        else if (localtiming<Tc-T_end_of_SS){//single support of cyclic walking

            if (localtiming<TDs+Tm){

               MatrixXd output1=CoefOffline.GetAccVelPos(C_cy_x_ar.row(0),localtiming-TDs,0,5);
               x_al=(footIndex!=0)*currentLeftFootX2+(footIndex==0)*(footIndex==0)*(currentLeftFootX2+output1(0,0));
               x_ar=(footIndex!=0)*(currentRightFootX2+output1(0,0))+(footIndex==0)*currentRightFootX2;

           }
           else{
                MatrixXd output1=CoefOffline.GetAccVelPos(C_cy_x_ar.row(1),localtiming-TDs,Tm,5);
               x_al=(footIndex!=0)*currentLeftFootX2+(footIndex==0)*(footIndex==0)*(currentLeftFootX2+output1(0,0));
               x_ar=(footIndex!=0)*(currentRightFootX2+output1(0,0))+(footIndex==0)*currentRightFootX2;

           }



             if (localtiming<TDs+Tm){

                MatrixXd output2=CoefOffline.GetAccVelPos(C_cy_z_ar.row(0),localtiming-TDs,0,5);
                z_al=(footIndex!=0)*currentLeftFootZ+(footIndex==0)*(currentLeftFootZ+output2(0,0));
                z_ar=(footIndex!=0)*(currentRightFootZ+output2(0,0))+(footIndex==0)*currentRightFootZ;

                MatrixXd output3=CoefOffline.GetAccVelPos(C_cy_y_ar.row(0),localtiming-TDs,0,5);
                y_al=(footIndex!=0)*currentLeftFootY2+(footIndex==0)*(currentLeftFootY2+output3(0,0));
                y_ar=(footIndex!=0)*(currentRightFootY2-output3(0,0))+(footIndex==0)*currentRightFootY2;


            }
            else{

                MatrixXd output2=CoefOffline.GetAccVelPos(C_cy_z_ar.row(1),localtiming-TDs,Tm,5);

                z_al=(footIndex!=0)*currentLeftFootZ+(footIndex==0)*(currentLeftFootZ+output2(0,0));
                z_ar=(footIndex!=0)*(currentRightFootZ+output2(0,0))+(footIndex==0)*currentRightFootZ;

                MatrixXd output3=CoefOffline.GetAccVelPos(C_cy_y_ar.row(1),localtiming-TDs,Tm,5);
                y_al=(footIndex!=0)*currentLeftFootY2+(footIndex==0)*(currentLeftFootY2+output3(0,0));
                y_ar=(footIndex!=0)*(currentRightFootY2-output3(0,0))+(footIndex==0)*currentRightFootY2;

            }
        }

        else if (localtiming<Tc){
           // qDebug()<<"single support 2\t"<<Tc;
            LeftFootOrientationAdaptator=(footIndex==0);
            RightFootOrientationAdaptator=(footIndex!=0);

            x_al=(footIndex!=0)*currentLeftFootX2+(footIndex==0)*(currentLeftFootX2+2*StepLength);
            x_ar=(footIndex!=0)*(currentRightFootX2+2*StepLength)+(footIndex==0)*currentRightFootX2;

            MatrixXd output2=CoefOffline.GetAccVelPos(C_cy_z_ar_end_of_SS.row(0),localtiming-Tc+TSS,TSS-T_end_of_SS,5);


           z_al=(footIndex!=0)*currentLeftFootZ+(footIndex==0)*(currentLeftFootZ+output2(0,0));
           z_ar=(footIndex!=0)*(currentRightFootZ+output2(0,0))+(footIndex==0)*currentRightFootZ;

           MatrixXd output3=CoefOffline.GetAccVelPos(C_cy_y_ar.row(0),localtiming-TDs,0,5);
           y_al=(footIndex!=0)*currentLeftFootY2+(footIndex==0)*(currentLeftFootY2+output3(0,0));
           y_ar=(footIndex!=0)*(currentRightFootY2-output3(0,0))+(footIndex==0)*currentRightFootY2;

       }
    }

    if(n==NStep+2){//end step of walk right foot moves


            n=n-1;
            currentLeftFootX2=(n-fmod(n+1,2))*StepLength+side_extra_step_length*StepLength;
           // currentLeftFootY2;
            currentLeftFootZ=0.112;

            currentRightFootX2=(n-fmod(n,2))*StepLength;
            //currentRightFootY2;
            currentRightFootZ=0.112;

            if (localtiming<=(TDs+0.001)){// double support of end
                z_ar=currentRightFootZ;
                z_al=currentLeftFootZ;
                //
                x_ar=currentRightFootX2;
                x_al=currentLeftFootX2;

                y_ar=currentRightFootY2;
                y_al=currentLeftFootY2;
            }


        else {

            if (localtiming<=(T_end_a_e-T_end_a_s-T_end_of_last_SS)+TDs){

                MatrixXd output1=CoefOffline.GetAccVelPos(C_end_x_ar.row(0),localtiming-TDs,0,5);
                x_ar=currentRightFootX2+output1(0,0);

                x_al=currentLeftFootX2;//*/(2*NStride+1)*StepLength;/*
                y_al=currentLeftFootY2;
                z_al=currentLeftFootZ;

                if (localtiming<=(T_end_a_e-T_end_a_s)/2+TDs){
                    MatrixXd output2=CoefOffline.GetAccVelPos(C_end_z_ar.row(0),localtiming-TDs,0,5);
                    z_ar=currentRightFootZ+output2(0,0);//currentRightFootZ+output2(0,0);

                    MatrixXd output4=CoefOffline.GetAccVelPos(C_end_y_ar.row(0),localtiming-TDs,0,5);
                    y_ar=currentRightFootY2;//-output4(0,0);
                }

                else if (localtiming>=(T_end_a_e-T_end_a_s)/2+TDs){
                    MatrixXd output2=CoefOffline.GetAccVelPos(C_end_z_ar.row(1),localtiming-TDs, (T_end_a_e-T_end_a_s)/2,5);
                    z_ar=currentRightFootZ+output2(0,0);

                    MatrixXd output4=CoefOffline.GetAccVelPos(C_end_y_ar.row(1),localtiming, (T_end_a_e-T_end_a_s)/2,5);
                    y_ar=currentRightFootY2;//-output4(0,0);
                }

            }
            else if (localtiming<=(T_end_a_e-T_end_a_s)+TDs){
                RightFootOrientationAdaptator=true;
                MatrixXd output1=CoefOffline.GetAccVelPos(C_end_x_ar.row(0),(T_end_a_e-T_end_a_s-T_end_of_last_SS),0,5);
                x_ar=currentRightFootX2+output1(0,0);
               // x_ar=currentRightFootX2+StepLength;
                x_al=currentLeftFootX2;//*/(2*NStride+1)*StepLength;/*
                y_al=currentLeftFootY2;
                y_ar=currentRightFootY2;
                z_al=currentLeftFootZ;
                MatrixXd output2=CoefOffline.GetAccVelPos(C_end_z_ar_end_of_SS.row(0),localtiming,T_end_a_e-T_end_a_s-T_end_of_last_SS+TDs,5);
                z_ar=currentRightFootZ+output2(0,0);


            }

            else{

                MatrixXd output1=CoefOffline.GetAccVelPos(C_end_x_ar.row(0),(T_end_a_e-T_end_a_s-T_end_of_last_SS),0,5);
                x_ar=currentRightFootX2+output1(0,0);

               // x_ar=currentRightFootX2+StepLength;//(2*NStride+1)*StepLength;
                z_ar=currentRightFootZ;//_lenghtOfAnkle;

                x_al=currentLeftFootX2;//(2*NStride+1)*StepLength;
                z_al=currentLeftFootZ;//_lenghtOfAnkle;

                y_al=currentLeftFootY2;
                y_ar=currentRightFootY2;
            }


    }
}

    MatrixXd footpos(8,1);
    footpos<<x_al,y_al,z_al,yaw_al,x_ar,y_ar,z_ar,yaw_ar;
//qDebug()<<"time="<<time<<"\txal="<<x_al<<"\tzal="<<z_al-.112;
    return footpos;

}



