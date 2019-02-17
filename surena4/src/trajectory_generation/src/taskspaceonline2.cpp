#include "taskspaceonline2.h"
TaskSpaceOnline2::TaskSpaceOnline2()
{


    time=0;
    _timeStep=0.00500000000000;
    globalTime=0;
    localTime=0;
    localTime1=0;
    localTiming= 0;
    SetParameters();
    CoeffArrayAnkle();
    CoeffArrayPelvis();

    OldPelvisZ=ReferencePelvisHeight;
    NewPlevisZ=ReferencePelvisHeight;
    kbumpL=0;
    currentRightFootZ=_lenghtOfAnkle;
    currentLeftFootZ=_lenghtOfAnkle;
    kbumpR=0;

    localtimingInteger=0;
    bool _walkstate=true;

}

void TaskSpaceOnline2::SetParameters(){
    YOffsetOfAnkletrajectory=0.000;//for compensating the clearance of the hip roll in experiment
    RightHipRollModification=4;//3;//4;//5;//degree;
    LeftHipRollModification=6;//4;//4;//4;//degree
    HipPitchModification=3;

    toeOff=true;
    HipRollModification=true;


    Sc=0.3783;
    Rse=0.7364;
    Rd=0.5555;
    Rm=1.1095;
    Rzp=0.91;

    _lengthOfHip=0.10900;
    _lenghtOfAnkle=0.112;//0.112000;
    _lengthOfShank=0.3600;
    _lengthOfThigh=0.3700;
    _heelLength=0.1;
    _toeLength=0.15;
    _pelvisLength=0.23;
    ReferencePelvisHeight=0.86;
    InitialPelvisHeight=0.95100;
    NStride=2;
    NStep=NStride*2;
    DesiredVelocity=0.10000;
    StepLength=0.0840000;
    StepNumber=1;


    // localTime=0;

    /////////////////////////Note:the TC is inetgerrr!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    /// |||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||\\\\\\\\\

    Tc=StepLength*3.6/DesiredVelocity;
//    Tc=3;
//StepLength=.05;

    TStart=6;//time of first step
    TEnd=6;//time of last step
    TDs=.4*Tc; // Duration of double support phase
//TDs=1.2;
    //  T_beta=TDs/2;
    TSS=Tc-TDs; // Duration of single support phase
    TGait=TStart+NStride*2*Tc;
    Tm=0.5*TSS;  // Tm =TSS/2 (Shafiee)
    XofAnkleMaximumHeight=StepLength;//(0.22/0.35)*StepLength; // Position of ankle in x direction when it reaches maximum heigth
    za_c=0.07;
    AnkleMaximumHeight=za_c; // maximum height of ankle
    //    za_st_m=AnkleMaximumHeight;

    //    xa_st_m=(0.22/0.35)*StepLength/2;
    //    za_st_m=(AnkleMaximumHeight);
    //    xa_end_m=2*NStride*StepLength+(0.22/0.35)*StepLength/2;
    //    za_end_m=(AnkleMaximumHeight);


    ///these parameters should be tuned

    //sagital plane
    Xe=Sc*StepLength/(Rse+1)*1.2;//*.5;//pelvis and support at the end of sigle support phase
    Xs=Rse*Xe*.90;//*3.0;//pelvis and support at the begining of sigle support phase
//***********************ok in test
    Xe=.04;//
    Xs=0;//fixed


    //frontal plane
    YpMax=Rm*0.5*_pelvisLength*1.2;//*1.1;//distace between pelvis to global coord in the middle of single support
    Yd=1*Rd*YpMax;//distace between pelvis to global coord at the begining of single support
 //***********************ok in test
YpMax=0.08;
  Yd =0.08;
//    YStMax=.10;//1.0*YpMax/.9;///1.1;
//    YEndMax=.10;//1.0*YpMax/.9;///1.1;
  YStMax=1.0*YpMax/.9;///1.1;
  YEndMax=1.0*YpMax/.9;///1.1;


    cout<<" Xe="<<Xe<<" Xs="<< Xs <<" YpMax="<<YpMax<<" Yd="<<Yd<<" YStMax="<<YStMax<<" YEndMax="<<YEndMax<<endl;

    MotionTime=TStart+NStride*2*Tc+TDs+TEnd;

    TMinPelvisY=0.5*TDs; // The time that pelvis reaches its minimum distance in y direction
    TMaxAnkle=TDs+0.35*TSS;//0.53 % The time that ankle reaches its maximum distance in z direction
    TMaxPelvisY=TDs+0.5*TSS; // The time that pelvis reaches its maximum distance in y direction

    // last step: timing parameter of pelvis motion
    T_end_p_sx_rel=TDs+0.25*TEnd;
    T_end_p_sx=TGait+TDs+0.25*TEnd;
    T_end_p_sy=TGait+TDs+0.2*TEnd;
    T_end_p_dy=TGait+TDs+0.4*TEnd;
    T_end_p_ey=TGait+TDs+0.8*TEnd;
    // last step: timing parameter of ankle motion
    T_end_a_s=TGait+TDs;
    T_end_a_e=TGait+TDs+0.5*TEnd;
    T_end_a_d=TGait+TDs+0.45*(T_end_a_e-T_end_a_s);
    // first step: timing parameter of pelvis motion
    T_st_p_sy=0.2*TStart;
    T_st_p_dy=0.65*TStart;
    T_st_p_ey=0.8*TStart;
    T_st_p_sx=0.7*TStart;
    // first step: timing parameter of ankle motion
    T_st_a_s=0.5*TStart;
    T_st_a_d=T_st_a_s+0.45*(TStart-T_st_a_s);

}






MatrixXd TaskSpaceOnline2::RollAngleModification(double time, int n, double localtiming,bool LastDSIndex){
    double N;
    double t;
    double rollR=0;
    double rollL=0;

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


    double R_1=0.6;
    double  D_time=R_1*TDs;
    double D_teta_r=-1*RightHipRollModification*(M_PI/180);
    double D_teta_l=LeftHipRollModification*(M_PI/180);
double extra_D_theta_l=0*M_PI/180;
    if (t<=T_st_p_sx){
        D_time=T_st_p_sx-T_st_p_sy;
        MatrixXd Ct_roll_st(1,2);
        Ct_roll_st<<0 ,D_time;
        MatrixXd Cp_roll_st(1,2);
        Cp_roll_st<<0, D_teta_r;
        MatrixXd Cv_roll_st(1,2);
        Cv_roll_st<<0 ,0;
        MatrixXd Ca_roll_st(1,2);
        Ca_roll_st<<0 ,0;
        MatrixXd C_roll_st=CoefOffline.Coefficient(Ct_roll_st,Cp_roll_st,Cv_roll_st,Ca_roll_st);

        if (t>(T_st_p_sx-D_time) && t<=T_st_p_sx){
            MatrixXd output=GetAccVelPos(C_roll_st,t-(T_st_p_sx-D_time),0,5);
            rollR=output(0,0);
            rollL=0;
        }

    }
    else if (t>T_st_p_sx  && t<=TStart){
        rollR=D_teta_r;
        rollL=0;

    }
    else if (t>TStart &&  t<=(TDs+TStart)){


        rollR=D_teta_r;
        rollL=0;

        MatrixXd Ct_rollR_ds1(1,2);
        Ct_rollR_ds1<<0 ,D_time;
        MatrixXd Cp_rollR_ds1(1,2);
        Cp_rollR_ds1<<0, D_teta_r;
        MatrixXd Cv_rollR_ds1(1,2);
        Cv_rollR_ds1<<0 ,0;
        MatrixXd Ca_rollR_ds1(1,2);
        Ca_rollR_ds1<<0 ,0;
        MatrixXd C_rollR_ds1=CoefOffline.Coefficient(Ct_rollR_ds1,Cp_rollR_ds1,Cv_rollR_ds1,Ca_rollR_ds1);


        MatrixXd Ct_rollL_ds1(1,2);
        Ct_rollL_ds1<<0 ,D_time;
        MatrixXd Cp_rollL_ds1(1,2);
        Cp_rollL_ds1<<0, D_teta_l;
        MatrixXd Cv_rollL_ds1(1,2);
        Cv_rollL_ds1<<0 ,0;
        MatrixXd Ca_rollL_ds1(1,2);
        Ca_rollL_ds1<<0 ,0;
        MatrixXd C_rollL_ds1=CoefOffline.Coefficient(Ct_rollL_ds1,Cp_rollL_ds1,Cv_rollL_ds1,Ca_rollL_ds1);


        if (t>(TDs+TStart-D_time) && t<=(TDs+TStart)){
            MatrixXd outputl=GetAccVelPos(C_rollL_ds1,t-(TDs+TStart-D_time),0,5);
            rollL=rollL+outputl(0,0);

            MatrixXd outputR=GetAccVelPos(C_rollR_ds1,t-(TDs+TStart-D_time),0,5);
            rollR=rollR-outputR(0,0);
        }


    }
    else if (t>(TDs+TStart) && t<=(Tc+TStart)){
        rollR=0;
        rollL=D_teta_l;
    }
    else if (t>(Tc+TStart) && t<=(Tc+TDs+TStart)){

        rollR=0;
        rollL=D_teta_l;

        MatrixXd Ct_rollR_ds1(1,2);
        Ct_rollR_ds1<<0 ,D_time;
        MatrixXd Cp_rollR_ds1(1,2);
        Cp_rollR_ds1<<0, D_teta_r;
        MatrixXd Cv_rollR_ds1(1,2);
        Cv_rollR_ds1<<0 ,0;
        MatrixXd Ca_rollR_ds1(1,2);
        Ca_rollR_ds1<<0 ,0;
        MatrixXd C_rollR_ds1=CoefOffline.Coefficient(Ct_rollR_ds1,Cp_rollR_ds1,Cv_rollR_ds1,Ca_rollR_ds1);


        MatrixXd Ct_rollL_ds1(1,2);
        Ct_rollL_ds1<<0 ,D_time;
        MatrixXd Cp_rollL_ds1(1,2);
        Cp_rollL_ds1<<0, D_teta_l;
        MatrixXd Cv_rollL_ds1(1,2);
        Cv_rollL_ds1<<0 ,0;
        MatrixXd Ca_rollL_ds1(1,2);
        Ca_rollL_ds1<<0 ,0;
        MatrixXd C_rollL_ds1=CoefOffline.Coefficient(Ct_rollL_ds1,Cp_rollL_ds1,Cv_rollL_ds1,Ca_rollL_ds1);


        if (t>(Tc+TDs+TStart-D_time) && t<=(Tc+TDs+TStart)){
            MatrixXd outputl=GetAccVelPos(C_rollL_ds1,t-(Tc+TDs+TStart-D_time),0,5);
            rollL=rollL-outputl(0,0);

            MatrixXd outputR=GetAccVelPos(C_rollR_ds1,t-(Tc+TDs+TStart-D_time),0,5);
            rollR=rollR+outputR(0,0);
        }
    }
    else if (t>(Tc+TDs+TStart) && t<=(2*Tc+TStart)){
        rollR=D_teta_r;
        rollL=0;
    }
    else if (t>=TGait && t<(TGait+TDs)){
        rollR=D_teta_r;
        rollL=0;

        MatrixXd Ct_rollR_ds1(1,2);
        Ct_rollR_ds1<<0 ,D_time;
        MatrixXd Cp_rollR_ds1(1,2);
        Cp_rollR_ds1<<0, D_teta_r;
        MatrixXd Cv_rollR_ds1(1,2);
        Cv_rollR_ds1<<0 ,0;
        MatrixXd Ca_rollR_ds1(1,2);
        Ca_rollR_ds1<<0 ,0;
        MatrixXd C_rollR_ds1=CoefOffline.Coefficient(Ct_rollR_ds1,Cp_rollR_ds1,Cv_rollR_ds1,Ca_rollR_ds1);


        MatrixXd Ct_rollL_ds1(1,2);
        Ct_rollL_ds1<<0 ,D_time;
        MatrixXd Cp_rollL_ds1(1,2);
        Cp_rollL_ds1<<0, D_teta_l+extra_D_theta_l;
        MatrixXd Cv_rollL_ds1(1,2);
        Cv_rollL_ds1<<0 ,0;
        MatrixXd Ca_rollL_ds1(1,2);
        Ca_rollL_ds1<<0 ,0;
        MatrixXd C_rollL_ds1=CoefOffline.Coefficient(Ct_rollL_ds1,Cp_rollL_ds1,Cv_rollL_ds1,Ca_rollL_ds1);


        if (t>(TGait+TDs-D_time) && t<=(TGait+TDs)){
            MatrixXd outputl=GetAccVelPos(C_rollL_ds1,t-(TDs+TGait-D_time),0,5);
            rollL=rollL+outputl(0,0);

            MatrixXd outputR=GetAccVelPos(C_rollR_ds1,t-(TDs+TGait-D_time),0,5);
            rollR=rollR-outputR(0,0);
        }
    }
    else if (t>=(TGait+TDs) && t<T_end_p_sx){
        rollR=0;
        rollL=D_teta_l+extra_D_theta_l;
    }
    else if (t>=T_end_p_sx  && t<=(TGait+TDs+TEnd)){
        D_time=(T_end_p_ey-T_end_p_sx);
        MatrixXd Ct_roll_st(1,2);
        Ct_roll_st<<-1*D_time ,0;
        MatrixXd Cp_roll_st(1,2);
        Cp_roll_st<<D_teta_l+extra_D_theta_l, 0;
        MatrixXd Cv_roll_st(1,2);
        Cv_roll_st<<0 ,0;
        MatrixXd Ca_roll_st(1,2);
        Ca_roll_st<<0 ,0;
        MatrixXd C_roll_st=CoefOffline.Coefficient(Ct_roll_st,Cp_roll_st,Cv_roll_st,Ca_roll_st);

        if (t>(T_end_p_sx) && t<=T_end_p_sx+D_time){
            MatrixXd output=GetAccVelPos(C_roll_st,t-(T_end_p_sx+1*D_time),-1*D_time,5);
            rollL=output(0,0);
            rollR=0;
        }
//qDebug()<<"D_teta_l="<<D_teta_l<<"\t D_teta_r="<<D_teta_r;

    }
MatrixXd RollMat(2,1);
RollMat<<rollR,rollL;
return RollMat;
}


//double TaskSpaceOnline2::PitchAngleModification(double time, int n, double localtiming,bool LastDSIndex){
//    double N;
//    double t;
//    double pitch=0;

//    if (time<=TStart||time>TGait){
//        N=0;
//        t=time;
//    }
//    else if (time>TStart && time<TGait){
//        N=floor((time-TStart)/(2*Tc));
//        t=fmod((time-TStart),2*Tc)+TStart;
//    }
//    else if (time==TGait){
//        N=NStride;
//        t=0;
//    }

//    double D_pitch=-1*HipPitchModification*(M_PI/180);
//    double TstartofPitchModify=TStart/6;
//    double TendofPitchModify=TStart/2;
//    double D_time=TendofPitchModify-TstartofPitchModify;


//    if (t<=TendofPitchModify){

//        MatrixXd Ct_pitch_st(1,2);
//        Ct_pitch_st<<0 ,D_time;
//        MatrixXd Cp_pitch_st(1,2);
//        Cp_pitch_st<<0, D_pitch;
//        MatrixXd Cv_pitch_st(1,2);
//        Cv_pitch_st<<0 ,0;
//        MatrixXd Ca_pitch_st(1,2);
//        Ca_pitch_st<<0 ,0;
//        MatrixXd C_pitch_st=CoefOffline.Coefficient(Ct_pitch_st,Cp_pitch_st,Cv_pitch_st,Ca_pitch_st);


//        if (t>(TstartofPitchModify) && t<=TendofPitchModify){
//            MatrixXd output=GetAccVelPos(C_pitch_st,t-(TstartofPitchModify),0,5);
//            pitch=output(0,0);
//        }
//    }
//    else if (t>TendofPitchModify && t<=TGait+TDs+TEnd/2){
//        pitch=D_pitch;
//    }
//       else if (t>=TGait+TDs+TEnd/2  && t<=(TGait+TDs+TEnd)){
//        MatrixXd Ct_pitch_st(1,2);
//        Ct_pitch_st<<-TEnd/2 ,0 ;
//        MatrixXd Cp_pitch_st(1,2);
//        Cp_pitch_st<< D_pitch ,0;
//        MatrixXd Cv_pitch_st(1,2);
//        Cv_pitch_st<<0 ,0;
//        MatrixXd Ca_pitch_st(1,2);
//        Ca_pitch_st<<0 ,0;
//        MatrixXd C_pitch_st=CoefOffline.Coefficient(Ct_pitch_st,Cp_pitch_st,Cv_pitch_st,Ca_pitch_st);

//        MatrixXd output=GetAccVelPos(C_pitch_st,t-(TGait+TDs+TEnd),-TEnd/2,5);
//        pitch=output(0,0);
//        }

//return pitch;
//}



void TaskSpaceOnline2::CoeffArrayPelvis(){

    //------------------Coefficient of cyclic motion in X direction--------------------
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


    //------------------Coefficient of cyclic Pelvis motion in Y direction--------------------
    MatrixXd ordY(1,8);
    ordY << 3,3,4,4,3,3,4,4;
    MatrixXd tttY(1,9);
    tttY <<0,TMinPelvisY,TDs,TDs+TSS/2,Tc,Tc+TDs/2,Tc+TDs,Tc+TDs+TSS/2,2*Tc;
    MatrixXd conY(3,9);
    conY<<-1*Yd,0,Yd,YpMax,Yd,0,-1*Yd,-1*YpMax,-1*Yd,INFINITY, INFINITY,INFINITY, 0 ,INFINITY,INFINITY,INFINITY,0,INFINITY,INFINITY, INFINITY,INFINITY, INFINITY ,INFINITY,INFINITY,INFINITY,INFINITY,INFINITY;
    Cy_p_i.resize(8,6);
    Cy_p_i.fill(0);
    Cy_p_i.block(0,1,8,5)=CoefOffline.Coefficient1(tttY,ordY,conY,0.1).transpose();
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
    Cx_st_pVel<<0 ,Cx_p(0,4);
    MatrixXd Cx_st_pAccel(1,2);
    Cx_st_pAccel<<0 ,2*Cx_p(0,3);
    Cx_st_p=CoefOffline.Coefficient(Cx_st_pTime,Cx_st_pPos,Cx_st_pVel,Cx_st_pAccel);

    // ----------------Coefficient of End of Pelvis motion in x direction----------------------

    MatrixXd Cx_end_pTime(1,2);
    Cx_end_pTime<<(TDs),T_end_p_sx_rel ;
    MatrixXd Cx_end_pPos(1,2);
    Cx_end_pPos<<(NStep+1)*StepLength-Xs, (NStep+1)*StepLength;
    MatrixXd Cx_end_pVel(1,2);
    Cx_end_pVel<<5*Cx_p(0,0)*pow(TDs,4)+4*Cx_p(0,1)*pow(TDs,3)+3*Cx_p(0,2)*pow(TDs,2)+2*Cx_p(0,3)*pow(TDs,1)+Cx_p(0,4), 0;
    MatrixXd Cx_end_pAccel(1,2);
    Cx_end_pAccel<<5*4*Cx_p(0,0)*pow(TDs,4)+4*3*Cx_p(0,1)*pow(TDs,3)+3*2*Cx_p(0,2)*pow(TDs,2)+2*Cx_p(0,3), 0;//like the last moment of first part of trajectory of cycle
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
    MatrixXd Cy_st_peVel(1,2);
    Cy_st_peVel<<0 ,5*Cy_p(0,0)*pow(0,4)+4*Cy_p(0,1)*pow(0,3)+3*Cy_p(0,2)*pow(0,2)+2*Cy_p(0,3)*pow(0,1)+Cy_p(0,4);
    MatrixXd Cy_st_peAccel(1,2);
    Cy_st_peAccel<<0 ,2*Cy_p(0,3);
    Cy_st_pb=CoefOffline.Coefficient(Cy_st_peTime,Cy_st_pePos,Cy_st_peVel,Cy_st_peAccel);

    // -------------------Coefficient of End of Pelvis motion in y direction--------------
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


    //    //----------------Coefficient of modification of Pelvis motion in z direction---------------------
    //    MatrixXd Cz_mod_pTime(1,2);
    //    Cz_mod_pTime<<0 ,TSS;
    //    MatrixXd Cz_mod_pPos(1,2);
    //    Cz_mod_pPos<<OldPelvisZ, NewPlevisZ;
    //    MatrixXd Cz_mod_pVel(1,2);
    //    Cz_mod_pVel<<0 ,0;
    //    MatrixXd Cz_mod_pAccel(1,2);
    //    Cz_mod_pAccel<<0 ,0;
    //    Cz_mod_p=CoefOffline.Coefficient(Cz_mod_pTime,Cz_mod_pPos,Cz_mod_pVel,Cx_mod_pAccel);


}


MatrixXd TaskSpaceOnline2::PelvisTrajectory(double time, int n, double localtiming,bool LastDSIndex){


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
    //double localTime1=localTime;
    double Times;
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
        MatrixXd output=GetAccVelPos(Cx_st_p,t,T_st_p_sx,5);
        xp=output(0,0);
        dxp=output(0,1);
        ddxp=output(0,2);
        DoubleSupport=true;
    }
    else if (t>TStart &&  t<=(TDs+TStart)){
        MatrixXd output=GetAccVelPos(Cx_p_i.row(0),t-TStart,0,5);
        xp=output(0,0);
        dxp=output(0,1);
        ddxp=output(0,2);
        DoubleSupport=true;
    }
    else if (t>(TDs+TStart) && t<=(Tc+TStart)){
        MatrixXd output=GetAccVelPos(Cx_p_i.row(1),t-TStart,0,5);
        xp=output(0,0);
        dxp=output(0,1);
        ddxp=output(0,2);
        DoubleSupport=false;
    }
    else if (t>(Tc+TStart) && t<=(Tc+TDs+TStart)){
        MatrixXd output=GetAccVelPos(Cx_p_i.row(0),t-Tc-TStart,0,5);
        xp=output(0,0)+StepLength;
        dxp=output(0,1);
        ddxp=output(0,2);
        DoubleSupport=true;
    }
    else if (t>(Tc+TDs+TStart) && t<=(2*Tc+TStart)){
        MatrixXd output=GetAccVelPos(Cx_p_i.row(1),t-Tc-TStart,0,5);
        xp=output(0,0)+StepLength;
        dxp=output(0,1);
        ddxp=output(0,2);
        DoubleSupport=false;
    }
    else if (t>TGait && t<(TGait+TDs)){
        MatrixXd output=GetAccVelPos(Cx_p_i.row(0),t-TGait,0,5);
        xp=output(0,0)+2*NStride*StepLength;
        dxp=output(0,1);
        ddxp=output(0,2);
        DoubleSupport=true;
    }
    else if (t>=(TGait+TDs) && t<T_end_p_sx){
        MatrixXd output=GetAccVelPos(Cx_end_p,t,(TGait+TDs),5);
        xp=output(0,0);
        dxp=output(0,1);
        ddxp=output(0,2);
        DoubleSupport=true;
    }
    else if (t>=T_end_p_sx  && t<=(TGait+TDs+TEnd)){
        xp=(2*NStride+1)*StepLength;
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

    /// following was a try for online adapting of pelvis x motion,
    ///  however in last step we had some problems therefore we commented this and used previous offline version
    ///
    //    if (time<TStart){
    //        N=1;
    //        t=time;
    //        Times=time;
    //    }
    //    else if (time>=TStart && time<(TGait-0.001)){
    //        N=floor((time-TStart-0.00001)/(Tc))+2;//
    //        //
    //        //-------------------------------------------------note------------------------------------------//
    //        //above 0.00001 amount is decreased from time to handle the uncertainity of double numbers!!! we had a peak at the end of each step
    //        //-------------------------------------------------note-------------------------------------------//
    //        //cout<<N<<endl<<flush;
    //        t=fmod((time-TStart),2*Tc)+TStart;
    //        Times=fmod((time-TStart),Tc);
    //    }
    //    else if (time<=(TGait-0.001) && time>=(TGait+0.001) ) {
    //        N=NStep+2;
    //    }
    //    else if (time>(TGait)){
    //        N=NStep+2;
    //        t=time;
    //        Times=time-TGait;
    //    }





    ////    if ((abs(localtiming-Times))>0.000001) {
    ////        double mil=(abs(localtiming-Times));
    ////        cout<<"time="<<Times<<endl<<endl;
    ////        cout<<"localtime="<<(localtiming)<<endl;
    ////        cout<<"Tstep="<<Tc<<endl;
    ////        //localtiming=0.002;
    ////                     cout<<"number of step="<<StepNumber<<endl;
    ////                     cout<<"N="<<N<<endl;
    ////        cout<<"different="<<mil<<endl<<endl;
    ////    }



    //    //localtiming=Times;


    //    if (N==1){
    //        if (localtiming<=T_st_p_sx){
    //            xp=0;
    //            dxp=0;
    //            ddxp=0;
    //          //  DoubleSupport=true;
    //        }
    //        else {
    //            MatrixXd output=GetAccVelPos(Cx_st_p,localtiming,T_st_p_sx,5);
    //            xp=output(0,0);
    //            dxp=output(0,1);
    //            ddxp=output(0,2);
    //          //  DoubleSupport=true;
    //        }
    //    }


    //    if (N!=1 && N!=(NStep+2)) {

    //        if (localtiming>0 &&  localtiming<(TDs)){
    //            MatrixXd output=GetAccVelPos(Cx_p_i.row(0),localtiming,0,5);
    //            xp=output(0,0);
    //            dxp=output(0,1);
    //            ddxp=output(0,2);
    ////cout<<xp<<endl<<flush;
    //           // DoubleSupport=true;
    //        }
    //        else if (localtiming>=(TDs) && localtiming<=(Tc)){
    //            MatrixXd output=GetAccVelPos(Cx_p_i.row(1),localtiming,0,5);
    //            xp=output(0,0);
    //            dxp=output(0,1);
    //            ddxp=output(0,2);
    //            if (time>65.5   && time<66.1) {
    //                  cout<<xp<<" time= "<<time<<" timing= "<<localtiming<<" N= "<<N<<" Tds= "<<TDs<<endl<<flush;
    //            }
    //           // cout<<xp<<endl<<flush;
    //           // DoubleSupport=false;
    //        }
    //    }


    //    if ( N==(NStep+2) ) {

    ////        if (LastDSIndex==true) {
    ////            if (localtiming>=0 && localtiming<=(TDs+0.002)){
    ////                MatrixXd output=GetAccVelPos(Cx_p_i.row(0),localtiming,0,5);
    ////                xp=output(0,0);
    ////                dxp=output(0,1);
    ////                ddxp=output(0,2);

    ////            }
    ////        }
    ////        else {


    ////           if (localtiming>=0 && localtiming<=(T_end_p_sx_rel-TDs)){
    ////                MatrixXd output=GetAccVelPos(Cx_end_p,localtiming,0,5);
    ////                xp=output(0,0);
    ////                dxp=output(0,1);
    ////                ddxp=output(0,2);

    ////              //  DoubleSupport=true;
    ////            }
    ////            else if (localtiming>(T_end_p_sx_rel-TDs+0.002)  && localtiming<=(TEnd)){
    ////                xp=0;
    ////                dxp=0;
    ////                ddxp=0;
    ////              //  DoubleSupport=true;
    ////            }
    ////        }



    //       if (t>TGait && t<(TGait+TDs)){
    //            MatrixXd output=GetAccVelPos(Cx_p_i.row(0),t-TGait,0,5);
    //            xp=output(0,0)+2*NStride*StepLength;
    //            dxp=output(0,1);
    //            ddxp=output(0,2);

    //        }
    //        else if (t>=(TGait+TDs) && t<T_end_p_sx){
    //            MatrixXd output=GetAccVelPos(Cx_end_p,t,(TGait+TDs),5);
    //            xp=output(0,0);
    //            dxp=output(0,1);
    //            ddxp=output(0,2);

    //        }
    //        else if (t>=T_end_p_sx  && t<=(TGait+TDs+TEnd)){
    //            xp=(2*NStride+1)*StepLength;
    //            dxp=0;
    //            ddxp=0;

    //        }


    //    }

    ////    if (TStart==0 && t==0){
    ////        xp=Cx_p(4);
    ////        dxp=0;
    ////        ddxp=0;
    ////    }

    //    if (time<=TStart) {
    //        xp=xp+1*StepLength*(N-1);
    //    }
    //    else if (time<TGait) {
    //        xp=xp+1*StepLength*(N-2);
    //    }
    //    else if (time<=(TGait-0.001)  &&  time>=(TGait+0.001)) {
    //       xp=xp+1*StepLength*(N-2);
    //    }
    //    else if (time>=TGait && time<(TGait+TDs+0.002)){//sometimes 0.002 should be adde to second term
    //        xp=xp+1*StepLength*(N-2);

    //    }
    //    else if (time>=(TGait+TDs) && time<T_end_p_sx){
    //        xp=xp;

    //    }
    //    else if (time>T_end_p_sx) {
    //        xp=1*StepLength*(N-1);
    //    }













    //Y


    //    cout<<"time="<<localTime1<<endl;
    //    cout<<"timedsss="<<localtime<<endl;


    //*******************   y
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

    zp=ReferencePelvisHeight;
    dzp=0;
    ddzp=0;

    MatrixXd pelvis(9,1);
    pelvis<<xp,yp,zp,dxp,dyp,dzp,ddxp,ddyp,ddzp;
    return pelvis;
}


void TaskSpaceOnline2::CoeffArrayPelvisZMod(){

    //----------------Coefficient of modification of Pelvis motion in z direction---------------------
    MatrixXd Cz_mod_pTime(1,2);
    Cz_mod_pTime<<0 ,TSS;
    MatrixXd Cz_mod_pPos(1,2);
    Cz_mod_pPos<<OldPelvisZ, NewPlevisZ;
    MatrixXd Cz_mod_pVel(1,2);
    Cz_mod_pVel<<0 ,0;
    MatrixXd Cz_mod_pAccel(1,2);
    Cz_mod_pAccel<<0 ,0;
    Cz_mod_p=CoefOffline.Coefficient(Cz_mod_pTime,Cz_mod_pPos,Cz_mod_pVel,Cz_mod_pAccel);


    MatrixXd Cz_mod_st_pTime(1,2);
    Cz_mod_st_pTime<<0 ,TSS;
    MatrixXd Cz_mod_st_pPos(1,2);
    Cz_mod_st_pPos<<OldPelvisZ, OldPelvisZ;
    MatrixXd Cz_mod_st_pVel(1,2);
    Cz_mod_st_pVel<<0 ,0;
    MatrixXd Cz_mod_st_pAccel(1,2);
    Cz_mod_st_pAccel<<0 ,0;
    Cz_mod_st_p=CoefOffline.Coefficient(Cz_mod_st_pTime,Cz_mod_st_pPos,Cz_mod_st_pVel,Cz_mod_st_pAccel);
}

MatrixXd TaskSpaceOnline2::ModificationOfPelvisHeight(double time,int n, double localtiming,bool RFT_state,bool LFT_state,bool LastDSIndex){

    double dzp;
    double zp;
    double ddzp;

    if(n==1){
        zp=OldPelvisZ;
        dzp=0;
        ddzp=0;

    }
    else if (n!=1 && n!=(NStep+2)) {

        if (localtiming>=0.000 && localtiming<TDs){
            zp=OldPelvisZ;
            dzp=0;
            ddzp=0;
        }
        if (localtiming>=TDs && localtiming<=Tc){

            if (localtiming>=TDs && localtiming<=(TDs+0.006)) {
                OldPelvisZ=NewPlevisZ;
            }

            MatrixXd outputz=GetAccVelPos(Cz_mod_p.row(0),localtiming-TDs,0,5);
            zp=outputz(0,0);
            dzp=outputz(0,1);
            ddzp=outputz(0,2);
        }
    }
    else if( n==(NStep+2)){
        zp=OldPelvisZ;
        dzp=0;
        ddzp=0;
    }





    MatrixXd pelvisz(3,1);
    pelvisz<<zp,dzp,ddzp;
    return pelvisz;
}










void TaskSpaceOnline2::CoeffArrayAnkle(){


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
    C_st_iPos<<0, AnkleMaximumHeight,0;
    MatrixXd C_st_iVel(1,3);
    C_st_iVel<<0 ,INFINITY, 0;
    MatrixXd C_st_iAcc(1,3);
    C_st_iAcc<<0 ,INFINITY, 0;
    C_st_z_al=CoefOffline.Coefficient(C_st_iTime,C_st_iPos,C_st_iVel,C_st_iAcc);

    MatrixXd Cy_st_iTime(1,3);
    Cy_st_iTime<<T_s_st, TStart-T_s_st/2 ,TStart;
    MatrixXd Cy_st_iPos(1,3);
    Cy_st_iPos<<0,YOffsetOfAnkletrajectory,0;
    MatrixXd Cy_st_iVel(1,3);
    Cy_st_iVel<<0 ,INFINITY, 0;
    MatrixXd Cy_st_iAcc(1,3);
    Cy_st_iAcc<<0 ,INFINITY, 0;
    C_st_y_al=CoefOffline.Coefficient(Cy_st_iTime,Cy_st_iPos,Cy_st_iVel,Cy_st_iAcc);


//****************************************************//
//    MatrixXd C_cy_iTime_ar(1,2);
//    C_cy_iTime_ar<<0, TSS;
//    MatrixXd C_cy_iPos_ar(1,2);
//    C_cy_iPos_ar<<0, 2*StepLength;
//    MatrixXd C_cy_iVel_ar(1,2);
//    C_cy_iVel_ar<<0, 0;
//    MatrixXd C_cy_iAcc_ar(1,2);
//    C_cy_iAcc_ar<<0, 0;
//    C_cy_x_ar=CoefOffline.Coefficient(C_cy_iTime_ar,C_cy_iPos_ar,C_cy_iVel_ar,C_cy_iAcc_ar);

    MatrixXd C_cy_iTime_ar(1,3);
    C_cy_iTime_ar<<0, Tm, TSS;
    MatrixXd C_cy_iPos_ar(1,3);
    C_cy_iPos_ar<<0, XofAnkleMaximumHeight , 2*StepLength;
    MatrixXd C_cy_iVel_ar(1,3);
    C_cy_iVel_ar<<0,INFINITY, 0;
    MatrixXd C_cy_iAcc_ar(1,3);
    C_cy_iAcc_ar<<0,INFINITY, 0;
    C_cy_x_ar=CoefOffline.Coefficient(C_cy_iTime_ar,C_cy_iPos_ar,C_cy_iVel_ar,C_cy_iAcc_ar);
//**************************************************************


        MatrixXd Cy_cy_iTime(1,3);
        Cy_cy_iTime<<0 ,Tm, TSS;
        MatrixXd Cy_cy_iPos(1,3);
        Cy_cy_iPos<<0,YOffsetOfAnkletrajectory, 0;
        MatrixXd Cy_cy_iVel(1,3);
        Cy_cy_iVel<<0 ,INFINITY, 0;
        MatrixXd Cy_cy_iAcc(1,3);
        Cy_cy_iAcc<<0 ,INFINITY, 0;
        C_cy_y_ar=CoefOffline.Coefficient(Cy_cy_iTime,Cy_cy_iPos,Cy_cy_iVel,Cy_cy_iAcc);
 //*********************************************************************
    MatrixXd C_cy_iTime(1,3);
    C_cy_iTime<<0 ,Tm, TSS;
    MatrixXd C_cy_iPos(1,3);
    C_cy_iPos<<0,AnkleMaximumHeight, 0;
    MatrixXd C_cy_iVel(1,3);
    C_cy_iVel<<0 ,INFINITY, 0;
    MatrixXd C_cy_iAcc(1,3);
    C_cy_iAcc<<0 ,INFINITY, 0;
    C_cy_z_ar=CoefOffline.Coefficient(C_cy_iTime,C_cy_iPos,C_cy_iVel,C_cy_iAcc);
//***************************************************************************
    MatrixXd C_end_z_iTime(1,3);
    C_end_z_iTime<<0 ,(T_end_a_e-T_end_a_s)/2 ,T_end_a_e-T_end_a_s;
    MatrixXd C_end_z_iPos(1,3);
    C_end_z_iPos<<0, AnkleMaximumHeight, 0;
    MatrixXd C_end_z_iVel(1,3);
    C_end_z_iVel<<0 ,INFINITY, 0;
    MatrixXd C_end_z_iAcc(1,3);
    C_end_z_iAcc<<0 ,INFINITY, 0;
    C_end_z_ar=CoefOffline.Coefficient(C_end_z_iTime,C_end_z_iPos,C_end_z_iVel,C_end_z_iAcc);

    MatrixXd C_end_y_iTime(1,3);
    C_end_y_iTime<<0 ,(T_end_a_e-T_end_a_s)/2 ,T_end_a_e-T_end_a_s;
    MatrixXd C_end_y_iPos(1,3);
    C_end_y_iPos<<0,YOffsetOfAnkletrajectory, 0;
    MatrixXd C_end_y_iVel(1,3);
    C_end_y_iVel<<0 ,INFINITY, 0;
    MatrixXd C_end_y_iAcc(1,3);
    C_end_y_iAcc<<0 ,INFINITY, 0;
    C_end_y_ar=CoefOffline.Coefficient(C_end_y_iTime,C_end_y_iPos,C_end_y_iVel,C_end_y_iAcc);

    MatrixXd C_end_iTime_ar(1,2);
    C_end_iTime_ar<<0, (T_end_a_e-T_end_a_s);
    MatrixXd C_end_iPos_ar(1,2);
    C_end_iPos_ar<<0, 1*StepLength;
    MatrixXd C_end_iVel_ar(1,2);
    C_end_iVel_ar<<0, 0;
    MatrixXd C_end_iAcc_ar(1,2);
    C_end_iAcc_ar<<0, 0;
    C_end_x_ar=CoefOffline.Coefficient(C_end_iTime_ar,C_end_iPos_ar,C_end_iVel_ar,C_end_iAcc_ar);






}





MatrixXd TaskSpaceOnline2::AnkleTrajectory(double time,int n, double localtiming,bool RFT_state,bool LFT_state,bool LastDSIndex){

    double y_ar=-0.5*_pelvisLength;
    double y_al=0.5*_pelvisLength;
    double alpha_ar=0;
    double alpha_al=0;
    double beta_al=0;
    double beta_ar=0;
    double gama_ar=0;
    double gama_al=0;
    double x_ar;
    double z_ar;
    double x_al;
    double z_al;
    double pitch_al=0;
    double pitch_ar=0;
    // int Nn;



    if (n==1){//left foot moves in first step
        firstStep=true;
        LeftSS=false;
        RightSS=false;

        Leftmoves=true;
        Rightmoves=false;

        LeftSupport=true;
        RightSupport=true;

        x_ar=0;
        z_ar=_lenghtOfAnkle;

        x_al=0;
        z_al=_lenghtOfAnkle;

        if (localtiming>=T_s_st){
            MatrixXd output=GetAccVelPos(C_st_x_al.row(0),localtiming,T_s_st,5);
            x_al=(LFT_state==false)*output(0,0)+(LFT_state==true)*currentLeftFootX2;
            firstStep=true;


            if ((localtiming>=(TStart-T_s_st/2))  && localtiming<=((TStart-T_s_st/2)+0.01) ) {
                kbumpL=1;
            }


            if ( kbumpL==1){
                kbumpL=kbumpL+1;
                LeftSensorActive=false;
                LeftFootOrientationAdaptator=true;
            }




            if (localtiming<=(TStart-T_s_st/2)){
                MatrixXd output=GetAccVelPos(C_st_z_al.row(0),localtiming,T_s_st,5);
                z_al=_lenghtOfAnkle+output(0,0);

                MatrixXd output1=GetAccVelPos(C_st_y_al.row(0),time,T_s_st,5);
                y_al=0.5*_pelvisLength+output1(0,0);
                LeftSupport=false;
                LeftSS=true;
            }
            else{
                MatrixXd output=GetAccVelPos(C_st_z_al.row(1),localtiming,TStart-T_s_st/2,5);
                z_al=(LFT_state==false)*(_lenghtOfAnkle+output(0,0))+(LFT_state==true)*currentLeftFootZ;

                MatrixXd output2=GetAccVelPos(C_st_y_al.row(1),time,TStart-T_s_st/2,5);
                y_al=(LFT_state==false)*(0.5*_pelvisLength+output2(0,0))+(LFT_state==true)*currentLeftFootY2;
                LeftSupport=false;
                LeftSS=true;
            }
        }
    }

    if (n!=1 && n!=(NStep+2)){//cyclic walking

        n=n-1;
        //cout<<"localtime"<<localtiming<<endl;
        footIndex=fmod(n,2);// shows which foot is swing foot (in cyclic mode left foots is swinging in the even steps(N))
        //it means whenever the footIndex is 0 the left foots will go to the swing mode

        if (localtiming<TDs){// double support of cyclic walking

            if (footIndex==0) {
                Leftmoves=true;
                Rightmoves=false;
                LeftSupport=true;
                RightSupport=true;
                LeftSS=false;
                RightSS=false;
            }
            else {
                Leftmoves=false;
                Rightmoves=true;
                LeftSupport=true;
                RightSupport=true;
                LeftSS=false;
                RightSS=false;
            }


            x_al=currentLeftFootX2;
            y_al=currentLeftFootY2;
            z_al=currentLeftFootZ;

            x_ar=currentRightFootX2;
            y_ar=currentRightFootY2;
            z_ar=currentRightFootZ;

        }

        else if (localtiming<Tc){//single support of cyclic walking

            if (firstStep==true) {
                firstStep=false;
            }

            if (footIndex==0) {
                Leftmoves=true;
                Rightmoves=false;
                LeftSupport=false;
                RightSupport=true;
                LeftSS=true;
                RightSS=false;
            }
            else {
                Leftmoves=false;
                Rightmoves=true;
                LeftSupport=true;
                RightSupport=false;
                LeftSS=false;
                RightSS=true;
            }

            if (localtiming>(TDs+TSS/2)  && localtiming<(TDs+TSS/2+.01) && footIndex==0) {
                kbumpL=1;
            }

            if (localtiming>(TDs+TSS/2)  && localtiming<(TDs+TSS/2+.01) && footIndex!=0) {
                kbumpR=1;
            }

            if (footIndex==0 && kbumpL==1){
                kbumpL=kbumpL+1;
                LeftSensorActive=true;
                LeftFootOrientationAdaptator=true;
            }

            if (footIndex!=0 && kbumpR==1){
                kbumpR=kbumpR+1;
                RightSensorActive=true;
                RightFootOrientationAdaptator=true;
            }
//********************************************************************
//            MatrixXd output1=GetAccVelPos(C_cy_x_ar,localtiming-TDs,0,5);
//            x_al=(footIndex!=0)*currentLeftFootX2+(footIndex==0)*((LFT_state==true)*(currentLeftFootX2)+(footIndex==0)*(LFT_state!=true)*(currentLeftFootX2+output1(0,0)));
//            x_ar=(footIndex!=0)*((RFT_state==true)*(currentRightFootX2)+(footIndex!=0)*(RFT_state!=true)*(currentRightFootX2+output1(0,0)))+(footIndex==0)*currentRightFootX2;

            if (localtiming<TDs+Tm){

               MatrixXd output1=GetAccVelPos(C_cy_x_ar.row(0),localtiming-TDs,0,5);
               x_al=(footIndex!=0)*currentLeftFootX2+(footIndex==0)*((LFT_state==true)*(currentLeftFootX2)+(footIndex==0)*(LFT_state!=true)*(currentLeftFootX2+output1(0,0)));
               x_ar=(footIndex!=0)*((RFT_state==true)*(currentRightFootX2)+(footIndex!=0)*(RFT_state!=true)*(currentRightFootX2+output1(0,0)))+(footIndex==0)*currentRightFootX2;

           }
           else{
                MatrixXd output1=GetAccVelPos(C_cy_x_ar.row(1),localtiming-TDs,Tm,5);
               x_al=(footIndex!=0)*currentLeftFootX2+(footIndex==0)*((LFT_state==true)*(currentLeftFootX2)+(footIndex==0)*(LFT_state!=true)*(currentLeftFootX2+output1(0,0)));
               x_ar=(footIndex!=0)*((RFT_state==true)*(currentRightFootX2)+(footIndex!=0)*(RFT_state!=true)*(currentRightFootX2+output1(0,0)))+(footIndex==0)*currentRightFootX2;

           }



             if (localtiming<TDs+Tm){
                MatrixXd output2=GetAccVelPos(C_cy_z_ar.row(0),localtiming-TDs,0,5);
                z_al=(footIndex!=0)*currentLeftFootZ+(footIndex==0)*(currentLeftFootZ+output2(0,0));
                z_ar=(footIndex!=0)*(currentRightFootZ+output2(0,0))+(footIndex==0)*currentRightFootZ;

                MatrixXd output3=GetAccVelPos(C_cy_y_ar.row(0),localtiming-TDs,0,5);
                y_al=(footIndex!=0)*currentLeftFootY2+(footIndex==0)*(currentLeftFootY2+output3(0,0));
                y_ar=(footIndex!=0)*(currentRightFootY2-output3(0,0))+(footIndex==0)*currentRightFootY2;


            }
            else{

                 MatrixXd output2=GetAccVelPos(C_cy_z_ar.row(1),localtiming-TDs,Tm,5);
                z_al=(footIndex!=0)*currentLeftFootZ+(footIndex==0)*((LFT_state==true)*currentLeftFootZ+(footIndex==0)*(LFT_state!=true)*(currentLeftFootZ+output2(0,0)));
                z_ar=(footIndex!=0)*((RFT_state==true)*(currentRightFootZ)+(footIndex!=0)*(RFT_state!=true)*(currentRightFootZ+output2(0,0)))+(footIndex==0)*currentRightFootZ;

                MatrixXd output3=GetAccVelPos(C_cy_y_ar.row(1),localtiming-TDs,Tm,5);
                y_al=(footIndex!=0)*currentLeftFootY2+(footIndex==0)*((LFT_state==true)*currentLeftFootY2+(footIndex==0)*(LFT_state!=true)*(currentLeftFootY2+output3(0,0)));
                y_ar=(footIndex!=0)*((RFT_state==true)*(currentRightFootY2)+(footIndex!=0)*(RFT_state!=true)*(currentRightFootY2-output3(0,0)))+(footIndex==0)*currentRightFootY2;

            }
        }

    }

    if(n==NStep+2){//end step of walk right foot moves

        if (LastDSIndex==true) {
            n=n-1;

            if (localtiming<=(TDs+0.001)){// double support of end
                z_ar=currentRightFootZ;
                z_al=currentLeftFootZ;
                //
                x_ar=currentRightFootX2;
                x_al=currentLeftFootX2;

                y_ar=currentRightFootY2;
                y_al=currentLeftFootY2;
            }

        }
        else {
            firstStep=true;
            if (localtiming<=(T_end_a_e-T_end_a_s)){

                MatrixXd output1=GetAccVelPos(C_end_x_ar.row(0),localtiming,0,5);
                x_ar=currentRightFootX2+output1(0,0);

                x_al=currentLeftFootX2;//*/(2*NStride+1)*StepLength;/*
                y_al=currentLeftFootY2;
                z_al=currentLeftFootZ;

                if (localtiming<=(T_end_a_e-T_end_a_s)/2){
                    MatrixXd output2=GetAccVelPos(C_end_z_ar.row(0),localtiming,0,5);
                    z_ar=currentRightFootZ+output2(0,0);//currentRightFootZ+output2(0,0);

                    MatrixXd output4=GetAccVelPos(C_end_y_ar.row(0),localtiming,0,5);
                    y_ar=currentRightFootY2;//-output4(0,0);
                }

                else if (localtiming>=(T_end_a_e-T_end_a_s)/2){
                    MatrixXd output2=GetAccVelPos(C_end_z_ar.row(1),localtiming, (T_end_a_e-T_end_a_s)/2,5);
                    z_ar=currentRightFootZ+output2(0,0);

                    MatrixXd output4=GetAccVelPos(C_end_y_ar.row(1),localtiming, (T_end_a_e-T_end_a_s)/2,5);
                    y_ar=currentRightFootY2;//-output4(0,0);
                }

            }
            else{



                x_ar=currentRightFootX2;//(2*NStride+1)*StepLength;
                z_ar=currentRightFootZ;//_lenghtOfAnkle;

                x_al=currentLeftFootX2;//(2*NStride+1)*StepLength;
                z_al=currentLeftFootZ;//_lenghtOfAnkle;

                y_al=currentLeftFootY2;
                y_ar=currentRightFootY2;
            }
        }

    }


    MatrixXd footpos(8,1);
    footpos<<x_al,y_al,z_al,pitch_al,x_ar,y_ar,z_ar,pitch_ar;
    return footpos;

}



MatrixXd TaskSpaceOnline2::GetAccVelPos(MatrixXd Coef,double time,double ti,int PolynomialOrder)
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

