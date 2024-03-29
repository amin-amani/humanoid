#include "taskspaceonline3.h"

TaskSpaceOnline3::TaskSpaceOnline3(int N_s, double Ds)
{
_timeStep=.005;
    NStride=N_s;
    LeftHipRollModification= 2;3.2;3.1;2.7;2;
    RightHipRollModification=2;3.2;3.1;2.7;2;
    FirstHipRollModification=2;3.2;3.1;2.7;2;
    HipPitchModification=1;//2;
    beta_toe=7*M_PI/18*0;
    beta_heel=-1*M_PI/18*0;

    NStep=NStride*2;

    StepLength=Ds;
    XofAnkleMaximumHeight=StepLength*1.8;
        switch (int(StepLength*100)) {
        case 45://ff
            ReferencePelvisHeight=.8;
            Xe=0.092;
            Xs=0.092;
            break;
        case 40://ff
            ReferencePelvisHeight=.835;
            Xe=0.083;
            Xs=0.083;
            break;
        case 35://ff
            ReferencePelvisHeight=.86;
            Xe=0.073;
            Xs=0.073;
            break;
        case 30:
           // YStMax=.06;
            ReferencePelvisHeight=.885;
            Xe=0.065;
            Xs=0.065;
            break;
        case 25:
                ReferencePelvisHeight=.91; // 0.913 is gouth for xe=0.06 and xs=0.047
            Xe=0.06;// 0.06 is gouth for anklepitch=0 and zmp_min=-0.025 , zmp_max=0.025
            Xs=0.047;// 0.047 is gouth for anklepitch=0 and zmp_min=-0.025 , zmp_max=0.025
            XofAnkleMaximumHeight=StepLength*1.8; //1.9 would cause overshoot in x-direction of ankle trajectory
            Yd=.0562+0.008; //+0.008
            a_d=-.438;
            zmp_min=-0.025; // -0.025
            zmp_max=0.025; // 0.025
            YEndMax_Coef=1.1;
            break;
        case 20:
            ReferencePelvisHeight=.91;
            Xs=0.0201;
            Xe=0.0480;
            Yd=0.0701;
            a_d=-0.3745;
            zmp_min=0.0075;
            zmp_max=0.0049;
            YEndMax_Coef=1.05;
            break;
        case 15:
            ReferencePelvisHeight=.91; //0.92
            Xs=0.0123;
            Xe=0.0369;
            Yd=0.0708;
            a_d=-0.4527;
            zmp_min=0.0009; // -0.025
            zmp_max=0.0174; // 0.025
            YEndMax_Coef=1.05;
            break;
        default:
            break;
        }


        AnkleMaximumHeight=.045;


        //times
        TDs =0.7;
        TSS=0.9;
        Tm1=.4*TSS;//0.28
        Tm2=0.68*TSS;
        TStartofHeel=0.4*TSS; //Tm2 (ver43)
        TStartofAnkleAdaptation=Tm2;//0.75*TSS; // Tm2 (ver43)
        Tc=TSS+TDs;
        Tx=3;
        TE=3; //3
        TLastSS=1;
        TStart=Tx+TSS/2+Tc;
        TEnd=TLastSS+TE;
        T_st_p_sx=Tx+Tc; //0.7*TStart;
        T_s_st=Tx+Tc/2+TDs/2;//.5*TStart;
        TGait=TStart+NStride*2*Tc;
        MotionTime=TStart+NStride*2*Tc+TDs+TEnd;
        TMinPelvisY=0.5*TDs; // The time that pelvis reaches its minimum distance in y direction
        TMaxAnkle=TDs+0.35*TSS;//0.53 % The time that ankle reaches its maximum distance in z direction
        TMaxPelvisY=TDs+0.5*TSS; // The time that pelvis reaches its maximum distance in y direction
        T_end_of_first_SS=1e-4;0;
        T_end_of_SS=      1e-4;0;
        T_end_of_last_SS= 0;1e-4;
        h_end_of_SS=      1e-6;0;
        T_end_p_sx_rel=TDs+.5*TLastSS;
        T_end_p_sx=TGait+T_end_p_sx_rel;
        T_end_a_s=TGait+TDs;
        T_end_a_e=TGait+TDs+TLastSS;
        t_toe=0.2*TDs;
        t_heel=0.5*TDs;

    CoeffArrayAnkle();

    CoeffArrayPelvis();

}

TaskSpaceOnline3::TaskSpaceOnline3()
{
_timeStep=.005;
    NStride=3;
    LeftHipRollModification= 2;3.2;3.1;2.7;2;
    RightHipRollModification=2;3.2;3.1;2.7;2;
    FirstHipRollModification=2;3.2;3.1;2.7;2;
    HipPitchModification=1;//2;
    beta_toe=7*M_PI/18*0;
    beta_heel=-1*M_PI/18*0;

    NStep=NStride*2;

    StepLength=25;
    XofAnkleMaximumHeight=StepLength*1.8;
        switch (int(StepLength*100)) {
        case 45://ff
            ReferencePelvisHeight=.8;
            Xe=0.092;
            Xs=0.092;
            break;
        case 40://ff
            ReferencePelvisHeight=.835;
            Xe=0.083;
            Xs=0.083;
            break;
        case 35://ff
            ReferencePelvisHeight=.86;
            Xe=0.073;
            Xs=0.073;
            break;
        case 30:
           // YStMax=.06;
            ReferencePelvisHeight=.885;
            Xe=0.065;
            Xs=0.065;
            break;
        case 25:
                ReferencePelvisHeight=.91; // 0.913 is gouth for xe=0.06 and xs=0.047
            Xe=0.06;// 0.06 is gouth for anklepitch=0 and zmp_min=-0.025 , zmp_max=0.025
            Xs=0.047;// 0.047 is gouth for anklepitch=0 and zmp_min=-0.025 , zmp_max=0.025
            XofAnkleMaximumHeight=StepLength*1.8; //1.9 would cause overshoot in x-direction of ankle trajectory
            Yd=.0562+0.008; //+0.008
            a_d=-.438;
            zmp_min=-0.025; // -0.025
            zmp_max=0.025; // 0.025
            YEndMax_Coef=1.1;
            break;
        case 20:
            ReferencePelvisHeight=.91;
            Xs=0.0201;
            Xe=0.0480;
            Yd=0.0701;
            a_d=-0.3745;
            zmp_min=0.0075;
            zmp_max=0.0049;
            YEndMax_Coef=1.05;
            break;
        case 15:
            ReferencePelvisHeight=.91; //0.92
            Xs=0.0123;
            Xe=0.0369;
            Yd=0.0708;
            a_d=-0.4527;
            zmp_min=0.0009; // -0.025
            zmp_max=0.0174; // 0.025
            YEndMax_Coef=1.05;
            break;
        default:
            break;
        }


        AnkleMaximumHeight=.045;


        //times
        TDs =0.7;
        TSS=0.9;
        Tm1=.4*TSS;//0.28
        Tm2=0.68*TSS;
        TStartofHeel=0.4*TSS; //Tm2 (ver43)
        TStartofAnkleAdaptation=Tm2;//0.75*TSS; // Tm2 (ver43)
        Tc=TSS+TDs;
        Tx=3;
        TE=3; //3
        TLastSS=1;
        TStart=Tx+TSS/2+Tc;
        TEnd=TLastSS+TE;
        T_st_p_sx=Tx+Tc; //0.7*TStart;
        T_s_st=Tx+Tc/2+TDs/2;//.5*TStart;
        TGait=TStart+NStride*2*Tc;
        MotionTime=TStart+NStride*2*Tc+TDs+TEnd;
        TMinPelvisY=0.5*TDs; // The time that pelvis reaches its minimum distance in y direction
        TMaxAnkle=TDs+0.35*TSS;//0.53 % The time that ankle reaches its maximum distance in z direction
        TMaxPelvisY=TDs+0.5*TSS; // The time that pelvis reaches its maximum distance in y direction
        T_end_of_first_SS=1e-4;0;
        T_end_of_SS=      1e-4;0;
        T_end_of_last_SS= 0;1e-4;
        h_end_of_SS=      1e-6;0;
        T_end_p_sx_rel=TDs+.5*TLastSS;
        T_end_p_sx=TGait+T_end_p_sx_rel;
        T_end_a_s=TGait+TDs;
        T_end_a_e=TGait+TDs+TLastSS;
        t_toe=0.2*TDs;
        t_heel=0.5*TDs;

    CoeffArrayAnkle();

    CoeffArrayPelvis();

}



void TaskSpaceOnline3::numplot(double num,double min,double max){
  //

  QString str;
  int l=100;
  int n=int((num-min)/(max-min)*l);
  if (num<min){n=0;}
  if (num>max){n=100;}
  str+=QString::number(min);
  str+="|";
  if (n<=l/2){
      for (int i = 0; i < n; ++i) {
          str+=" ";
      }
      for (int i = 0; i < l/2-n; ++i) {
          str+="|";

      }
      str+="|";
      for (int i = 0; i < l/2; ++i) {
          str+=" ";
      }
  }
  else {
      for (int i = 0; i < l/2; ++i) {
          str+=" ";
      }
      for (int i = 0; i < n-l/2; ++i) {
          str+="|";

      }
      str+="|";
      for (int i = 0; i < l-n; ++i) {
          str+=" ";
      }

  }

  str+="|";
  str+=QString::number(max);
  str+="=>";str+=QString::number(num);
  qDebug()<<str;
qDebug()<<"";


}

void TaskSpaceOnline3::matrix_view(MatrixXd M){

    for (int i = 0; i <M.rows() ; ++i) {
        QString str;
        for (int j = 0; j <M.cols() ; ++j) {
            str+=QString::number(M(i,j));
            str+="   ";
        }
        qDebug()<<str;
    }
    qDebug()<<"";
}


void TaskSpaceOnline3::matrix_view(VectorXd M){
    QString str;
    for (int i = 0; i <M.rows() ; ++i) {str+=QString::number(M(i));str+="   ";}
    qDebug()<<str;
    qDebug()<<"";
}

double TaskSpaceOnline3::move2pose(double max,double t_local,double T_start ,double T_end){
    double T_move=T_end-T_start;
    double c3=(10*max)/pow(T_move,3);
    double c4=-(15*max)/pow(T_move,4);
    double c5=(6*max)/pow(T_move,5);
    double theta=0;
    if(t_local<T_start){theta=0;}
    else if (t_local<T_end){theta=c3*pow(t_local-T_start,3)+c4*pow(t_local-T_start,4)+c5*pow(t_local-T_start,5);}
    else{theta=max;}
    return theta;
}



double TaskSpaceOnline3::RollCharge(double t,double t_start,double t_end,double magnitude){
//    double D_time_charge = t_end-t_start;
//    MatrixXd Ct(1,2);    Ct<<0 ,D_time_charge;
//    MatrixXd Cp(1,2);    Cp<<0, magnitude;
//    MatrixXd Cv(1,2);    Cv<<0 ,0;
//    MatrixXd Ca(1,2);    Ca<<0 ,0;
//    MatrixXd C_roll=CoefOffline.Coefficient(Ct,Cp,Cv,Ca);
//    if(t<=t_start){return 0;}
//    else if((t-t_start)>=D_time_charge){return magnitude;}
//    else{    MatrixXd output=CoefOffline.GetAccVelPos(C_roll,t-t_start,0,5);
//        return output(0,0);}
   double roll=move2pose(magnitude,t,t_start,t_end) ;
    return roll;
}

double TaskSpaceOnline3::RollDecharge(double t,double t_start,double t_end,double magnitude){
//    double D_time_decharge = t_end-t_start;
//    MatrixXd Ct(1,2);    Ct<<0 ,D_time_decharge;
//    MatrixXd Cp(1,2);    Cp<< 0, -magnitude;
//    MatrixXd Cv(1,2);    Cv<<0 ,0;
//    MatrixXd Ca(1,2);    Ca<<0 ,0;
//    MatrixXd C_roll=CoefOffline.Coefficient(Ct,Cp,Cv,Ca);
//    if(t<=t_start){return 0;}
//    else if((t-t_start)>=D_time_decharge){return -magnitude;}
//    else{    MatrixXd output=CoefOffline.GetAccVelPos(C_roll,t-t_start,0,5);
//        return output(0,0);}

    double roll=move2pose(-magnitude,t,t_start,t_end) ;
     return roll;
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
    double DechargeCoeffLeftStartEndPhase =.4;// 0.6;//must be bigger than 0.5; end foot pairing DS
    double DechargeCoeffLeftEndEndPhase = 0.8;  //must be greata

double KStartStartPhase=0.1;
double KEndStartPhase=0.45;

double StartStartPhaseRatio=(Tx+Tc/2-KStartStartPhase*0.5*TDs)/(Tx+Tc+0.5*TSS);
double EndStartPhaseRatio=(Tx+TSS/2+TDs+KEndStartPhase*TSS)/(Tx+Tc+0.5*TSS);




    double ChargeCoeffRightStartStartPhase = StartStartPhaseRatio;   //0.3 while reducing height(0.5 means minimum height)
    double ChargeCoeffRightEndStartPhase = EndStartPhaseRatio;     //0.7 almot maximum ankle height
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
            RollDecharge(t,TStart+2*Tc+TDs+TLastSS+DechargeCoeffLeftStartEndPhase*TDs,TStart+2*Tc+TDs+TLastSS+DechargeCoeffLeftEndEndPhase*TDs,D_teta_l);

    rollR=rollR+RollCharge(t,ChargeCoeffRightStartStartPhase*TStart,ChargeCoeffRightEndStartPhase*TStart,D_teta_r)+
            RollDecharge(t,TStart+DechargeCoeffRightStartStartPhase*TDs,TStart+DechargeCoeffRightEndStartPhase*TDs,D_teta_r)+
            RollCharge(t,TStart+Tc+ChargeCoeffRightStart*TDs,TStart+Tc+ChargeCoeffRightEnd*TDs,D_teta_r)+
            RollDecharge(t,TStart+2*Tc+DechargeCoeffRightStart*TDs,TStart+2*Tc+DechargeCoeffRightEnd*TDs,D_teta_r);






    MatrixXd RollMat(2,1);
    RollMat<<rollR,rollL;
    return RollMat;}

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
    //different velocities for lowering foot in end of ss
    double vz_start=-(AnkleMaximumHeight-h_end_of_SS)*2/(T_s_st/2-T_end_of_first_SS)/4;
    double vz_cycle=-(AnkleMaximumHeight-h_end_of_SS)*2/(TSS-T_end_of_SS-Tm2)/4;
    double vz_end=-(AnkleMaximumHeight-h_end_of_SS)*2/((T_end_a_e-T_end_a_s)/2-T_end_of_last_SS)/4;
//    vz_start=0;
//    vz_cycle=0;
//    vz_end=0;
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
    C_st_iTime<<T_s_st, ((TStart+T_s_st)/2) ,TStart-T_end_of_first_SS;
    MatrixXd C_st_iPos(1,3);
    C_st_iPos<<0, AnkleMaximumHeight,h_end_of_SS;
    MatrixXd C_st_iVel(1,3);
    C_st_iVel<<0 ,INFINITY, vz_start;//0 ,INFINITY, vz_start
    MatrixXd C_st_iAcc(1,3);
    C_st_iAcc<<0 ,INFINITY, 0;
    C_st_z_al=CoefOffline.Coefficient(C_st_iTime,C_st_iPos,C_st_iVel,C_st_iAcc);
    //***** z start 2
    MatrixXd C_st_iTime_e(1,2);
    C_st_iTime_e<<TStart-T_end_of_first_SS,TStart;
    MatrixXd C_st_iPos_e(1,2);
    C_st_iPos_e<<h_end_of_SS,0;
    MatrixXd C_st_iVel_e(1,2);
    C_st_iVel_e<<vz_start , 0;
    MatrixXd C_st_iAcc_e(1,2);
    C_st_iAcc_e<<0 , 0;
    C_st_z_al_end_of_SS=CoefOffline.Coefficient(C_st_iTime_e,C_st_iPos_e,C_st_iVel_e,C_st_iAcc_e);

    //**************** cycle ***************************//

    //***** x cycle
    MatrixXd C_cy_iTime_ar(1,3);
    C_cy_iTime_ar<<0, Tm2, TSS-T_end_of_SS;
    MatrixXd C_cy_iPos_ar(1,3);
    C_cy_iPos_ar<<0, XofAnkleMaximumHeight , 2*StepLength;
    MatrixXd C_cy_iVel_ar(1,3);
    C_cy_iVel_ar<<0,INFINITY, 0;
    MatrixXd C_cy_iAcc_ar(1,3);
    C_cy_iAcc_ar<<0,INFINITY, 0;
    C_cy_x_ar=CoefOffline.Coefficient(C_cy_iTime_ar,C_cy_iPos_ar,C_cy_iVel_ar,C_cy_iAcc_ar);
//matrix_view(C_cy_x_ar);
    //***** z cycle1
    MatrixXd ordza(1,3);
    ordza << 4,3,5;
    MatrixXd tttza(1,4);
    tttza <<0 ,Tm1 ,Tm2, TSS-T_end_of_SS;
    MatrixXd conza(3,4);
    conza<<0,.6*AnkleMaximumHeight,AnkleMaximumHeight ,h_end_of_SS,
            0 ,INFINITY,INFINITY, vz_cycle,0 ,INFINITY,INFINITY, 0;

    C_cy_z_ar.resize(3,6);
    C_cy_z_ar.fill(0);
    C_cy_z_ar.block(0,0,3,6)=CoefOffline.Coefficient1(tttza,ordza,conza,0.1).transpose();//.block(0,1,2,5) .block(0,2,2,4) (comment)
    //***** z cycle2
    MatrixXd C_cy_iTime_e(1,2);
    C_cy_iTime_e<<TSS-T_end_of_SS, TSS;
    MatrixXd C_cy_iPos_e(1,2);
    C_cy_iPos_e<< h_end_of_SS, 0;
    MatrixXd C_cy_iVel_e(1,2);
    C_cy_iVel_e<<vz_cycle , 0;
    MatrixXd C_cy_iAcc_e(1,2);
    C_cy_iAcc_e<<0 , 0;
    C_cy_z_ar_end_of_SS=CoefOffline.Coefficient(C_cy_iTime_e,C_cy_iPos_e,C_cy_iVel_e,C_cy_iAcc_e);

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
    C_end_z_iVel<<0 ,INFINITY, vz_end;//0 ,INFINITY, vz_end
    MatrixXd C_end_z_iAcc(1,3);
    C_end_z_iAcc<<0 ,INFINITY, 0;
    C_end_z_ar=CoefOffline.Coefficient(C_end_z_iTime,C_end_z_iPos,C_end_z_iVel,C_end_z_iAcc);
    //***** z end 2
    MatrixXd C_end_z_iTime_e(1,2);
    C_end_z_iTime_e<<T_end_a_e-T_end_a_s-T_end_of_last_SS ,T_end_a_e-T_end_a_s;
    MatrixXd C_end_z_iPos_e(1,2);
    C_end_z_iPos_e<<h_end_of_SS, 0;
    MatrixXd C_end_z_iVel_e(1,2);
    C_end_z_iVel_e<<vz_end , 0;
    MatrixXd C_end_z_iAcc_e(1,2);
    C_end_z_iAcc_e<<0 , 0;
    C_end_z_ar_end_of_SS=CoefOffline.Coefficient(C_end_z_iTime_e,C_end_z_iPos_e,C_end_z_iVel_e,C_end_z_iAcc_e);

    //***** beta cycle
double v_beta_toe=2*beta_toe/t_toe;

        MatrixXd beta_toe_t_cycle(1,2);
        beta_toe_t_cycle<<0,t_toe;
        MatrixXd beta_toe_Pos_cycle(1,2);
        beta_toe_Pos_cycle<<0, beta_toe;
        MatrixXd beta_toe_Vel_cycle(1,2);
        beta_toe_Vel_cycle<<0 , v_beta_toe;
        MatrixXd beta_toe_Acc_cycle(1,2);
        beta_toe_Acc_cycle<<0 , 0;
        C_beta_toe_cycle=CoefOffline.Coefficient(beta_toe_t_cycle,beta_toe_Pos_cycle,beta_toe_Vel_cycle,beta_toe_Acc_cycle);


        MatrixXd beta_heel_t_cycle(1,2);
        beta_heel_t_cycle<<0,t_heel;
        MatrixXd beta_heel_Pos_cycle(1,2);
        beta_heel_Pos_cycle<< beta_heel,0;
        MatrixXd beta_heel_Vel_cycle(1,2);
        beta_heel_Vel_cycle<<0 , 0;
        MatrixXd beta_heel_Acc_cycle(1,2);
        beta_heel_Acc_cycle<<0 , 0;
        C_beta_heel_cycle=CoefOffline.Coefficient(beta_heel_t_cycle,beta_heel_Pos_cycle,beta_heel_Vel_cycle,beta_heel_Acc_cycle);



        MatrixXd beta_toe2heel_t_cycle(1,3);
        beta_toe2heel_t_cycle<<0,TStartofHeel,TSS;
        MatrixXd beta_toe2heel_Pos_cycle(1,3);
        beta_toe2heel_Pos_cycle<< beta_toe,INFINITY,beta_heel;
        MatrixXd beta_toe2heel_Vel_cycle(1,3);
        beta_toe2heel_Vel_cycle<<  v_beta_toe ,INFINITY, 0;//0 ,0, 0;
        MatrixXd beta_toe2heel_Acc_cycle(1,3);
        beta_toe2heel_Acc_cycle<< 0 ,INFINITY, 0; //0 ,0, 0;
        C_beta_toe2heel_cycle=CoefOffline.Coefficient(beta_toe2heel_t_cycle,beta_toe2heel_Pos_cycle,beta_toe2heel_Vel_cycle,beta_toe2heel_Acc_cycle);

}

void TaskSpaceOnline3::CoeffArrayPelvis(){
    //------------------Coefficient of cyclic motion in X direction--------------------
double a_0=(Xe-zmp_max)/0.1;
double a_Ds=(-Xs-zmp_min)/0.1;
double A_Ds=(a_Ds-a_0)/(6*TDs);
double C_Ds=(-Xs-Xe+StepLength-A_Ds*TDs*TDs*TDs-a_0*TDs*TDs/2)/TDs;
double v_0=C_Ds;
double v_Ds=3*A_Ds*TDs*TDs+a_0*TDs+C_Ds;

    //------------------Coefficient of cyclic motion in X direction--------------------
    MatrixXd ord(1,2);
    ord << 5,5;//3,3 (Gooth Test)
    MatrixXd ttt(1,3);
    ttt <<0 ,TDs, Tc;
    MatrixXd con(3,3);
    con<<Xe, StepLength-Xs, StepLength+Xe,v_0,v_Ds ,v_0,a_0, a_Ds ,a_0;
    Cx_p_i.resize(2,6);
    Cx_p_i.fill(0);
    Cx_p_i.block(0,0,2,6)=CoefOffline.Coefficient1(ttt,ord,con,0.1).transpose();//.block(0,1,2,5) .block(0,2,2,4) (comment)
  Cx_p.resize(1,12);
    Cx_p.fill(0);
    Cx_p.block(0,0,1,6)=Cx_p_i.row(0);
    Cx_p.block(0,6,1,6)=Cx_p_i.row(1);


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
    Cx_end_pPos<<(NStep+1)*StepLength-Xs, (NStep+1)*StepLength+(side_extra_step_length)*StepLength;
    MatrixXd Cx_end_pVel(1,2);
    Cx_end_pVel<<5*Cx_p(0,0)*pow(TDs,4)+4*Cx_p(0,1)*pow(TDs,3)+3*Cx_p(0,2)*pow(TDs,2)+2*Cx_p(0,3)*pow(TDs,1)+Cx_p(0,4), 0;
    MatrixXd Cx_end_pAccel(1,2);
    Cx_end_pAccel<<5*4*Cx_p(0,0)*pow(TDs,3)+4*3*Cx_p(0,1)*pow(TDs,2)+3*2*Cx_p(0,2)*pow(TDs,1)+2*Cx_p(0,3), 0;//like the last moment of first part of trajectory of cycle
    Cx_end_p=CoefOffline.Coefficient(Cx_end_pTime,Cx_end_pPos,Cx_end_pVel,Cx_end_pAccel);


    //------------------Coefficient of cyclic Pelvis motion in Y direction--------------------
 MatrixXd ordY(1,8);
    ordY << 5,5,5,5,5,5,5,5;
      MatrixXd tttY(1,9);
      tttY <<0,TMinPelvisY,TDs,TDs+TSS/2,Tc,Tc+TDs/2,Tc+TDs,Tc+TDs+TSS/2,2*Tc;
      MatrixXd conY(3,9);

      double a_h=a_d/3/TDs;
      double c_h=(Yd-a_h*TDs*TDs*TDs/8)*2/TDs;
      double v_d=3*a_h*TDs*TDs/4+c_h;
      double a_a_h=(a_d*TSS*TSS/2+v_d*TSS)*12/(TSS*TSS*TSS*TSS);
      double b_b_h=-a_a_h*TSS;
      YpMax=-.75*a_a_h*(TSS*TSS*TSS*TSS)/48+a_d*TSS*TSS/8+v_d*TSS/2+Yd;

      double a_p_max=a_a_h*TSS*TSS/4+b_b_h*TSS/2+a_d;
      conY<<-1*Yd,0,Yd,YpMax,Yd,0,-1*Yd,-1*YpMax,-1*Yd,
              v_d, c_h,v_d, 0 ,-v_d,-c_h, -v_d,0,v_d
              ,-a_d, 0,a_d, a_p_max ,a_d, 0 ,-a_d,-a_p_max,-a_d;//a= ,0, INFINITY,INFINITY, INFINITY ,0,INFINITY,INFINITY,INFINITY,0;
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

    //--------------Coefficient of Start of Pelvis motion in y direction----------------
    MatrixXd ordY_S(1,6);
       ordY_S << 5,5,5,5,5,5;
         MatrixXd tttY_S(1,7);
         tttY_S <<0,Tx,Tx+TSS/2,Tx+Tc/2,Tx+Tc/2+TDs/2,Tx+Tc,TStart;
         MatrixXd conY_S(3,7);
//qDebug()<<YpMax;
         conY_S<<0,YpMax,Yd,0,-1*Yd,-1.1*YpMax,-1*Yd,
                 0,0,-v_d,-c_h, -v_d,0,v_d,
                 0,a_p_max,a_d, 0 ,-a_d,-a_p_max,-a_d;
         Cy_p_i_S.resize(6,6);
         Cy_p_i_S.fill(0);
         Cy_p_i_S.block(0,0,6,6)=CoefOffline.Coefficient1(tttY_S,ordY_S,conY_S,0.1).transpose();
         Cy_p_S.resize(1,36);
         Cy_p_S.fill(0);

    // -------------------Coefficient of End of Pelvis motion in y direction--------------
//YEndMax=YEndMax_Coef*YpMax;
//YEndMax=0.1;
//         MatrixXd ordY_e(1,2);
//         ordY_e << 5,5;
//         MatrixXd tttY_e(1,3);
//         tttY_e <<TGait+TDs,TGait+TDs+TLastSS/2,TGait+TDs+TLastSS+TE;
//         MatrixXd conY_e(3,3);
//         conY_e<<Yd,YEndMax,0,
//                 v_d,0,0,
//                 a_d,0,0;

//              Cy_p_i_E.resize(2,6);
//              Cy_p_i_E.fill(0);
//              Cy_p_i_E.block(0,0,2,6)=CoefOffline.Coefficient1(tttY_e,ordY_e,conY_e,0.1).transpose();//.block(0,1,8,5)


         YEndMax=YEndMax_Coef*YpMax;
         YEndMax=0.1;
         MatrixXd ordY_e(1,3);
         ordY_e << 5,5,5;
         MatrixXd tttY_e(1,4);
         tttY_e <<TGait+TDs,TGait+TDs+TLastSS/2,TGait+TDs+TLastSS+0.5*TE,TGait+TDs+TLastSS+TE;
         MatrixXd conY_e(3,4);
         conY_e<<Yd,YEndMax,-0.05,0,
                 v_d,0,0,0,
                 a_d,0,-a_p_max,0;

         Cy_p_i_E.resize(3,6);
         Cy_p_i_E.fill(0);
         Cy_p_i_E.block(0,0,3,6)=CoefOffline.Coefficient1(tttY_e,ordY_e,conY_e,0.1).transpose();//.block(0,1,8,5)


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



    xp=xp+2*StepLength*N;

/// y
    if(t<Tx){
        MatrixXd output=CoefOffline.GetAccVelPos(Cy_p_i_S.row(0),t,0,5);
        yp=output(0,0);
        dyp=output(0,1);
        ddyp=output(0,2);
    }
   else if(t<Tx+TSS/2){
        MatrixXd output=CoefOffline.GetAccVelPos(Cy_p_i_S.row(1),t,0,5);
        yp=output(0,0);
        dyp=output(0,1);
        ddyp=output(0,2);
    }

   else if(t<Tx+Tc/2){
        MatrixXd output=CoefOffline.GetAccVelPos(Cy_p_i_S.row(2),t,0,5);
        yp=output(0,0);
        dyp=output(0,1);
        ddyp=output(0,2);
    }

    else if(t<Tx+Tc/2+TDs/2){
        MatrixXd output=CoefOffline.GetAccVelPos(Cy_p_i_S.row(3),t,0,5);
        yp=output(0,0);
        dyp=output(0,1);
        ddyp=output(0,2);
    }

    else if(t<Tx+Tc){
        MatrixXd output=CoefOffline.GetAccVelPos(Cy_p_i_S.row(4),t,0,5);
        yp=output(0,0);
        dyp=output(0,1);
        ddyp=output(0,2);
    }

    else if(t<TStart){
        MatrixXd output=CoefOffline.GetAccVelPos(Cy_p_i_S.row(5),t,0,5);
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


    else if (t>(TDs+TGait) && t<=TGait+TDs+TLastSS/2){
        MatrixXd output=CoefOffline.GetAccVelPos(Cy_p_i_E.row(0),t,0,5);
        yp=output(0,0);
        dyp=output(0,1);
        ddyp=output(0,2);
    }
    else if (t>(TDs+TGait+TLastSS/2) && t<=TGait+TDs+TLastSS+0.5*TE){
        MatrixXd output=CoefOffline.GetAccVelPos(Cy_p_i_E.row(1),t,0,5);
        yp=output(0,0);
        dyp=output(0,1);
        ddyp=output(0,2);
    }

    else if (t>(TDs+TGait+TLastSS+0.5*TE) && t<=TGait+TDs+TLastSS+TE){
        MatrixXd output=CoefOffline.GetAccVelPos(Cy_p_i_E.row(2),t,0,5);
        yp=output(0,0);
        dyp=output(0,1);
        ddyp=output(0,2);
    }

//    else if (t>(TGait+TDs+TLastSS) && t<=TGait+TDs+TLastSS+TE*0.5){
//        MatrixXd output=CoefOffline.GetAccVelPos(Cy_p_i_E.row(2),t,0,5);
//        yp=output(0,0);
//        dyp=output(0,1);
//        ddyp=output(0,2);
//    }
//    else if (t>(TGait+TDs+TLastSS+TE*0.5) && t<=TGait+TDs+TLastSS+TE){
//        MatrixXd output=CoefOffline.GetAccVelPos(Cy_p_i_E.row(3),t,0,5);
//        yp=output(0,0);
//        dyp=output(0,1);
//        ddyp=output(0,2);
//    }


    else{
              yp=0;
              dyp=0;
              ddyp=0;
    }

    zp=ReferencePelvisHeight;
    dzp=0;
    ddzp=0;
    MatrixXd pelvis(10,1);
    pelvis<<xp,yp,zp,dxp,dyp,dzp,ddxp,ddyp,ddzp,yawp;

    return pelvis;
}


MatrixXd TaskSpaceOnline3::AnkleTrajectory(double time,int n ,double localtiming){
    LeftFootOrientationAdaptator=false;
    RightFootOrientationAdaptator=false;

    double y_ar=-0.5*_pelvisLength;
    double y_al=0.5*_pelvisLength;
    double x_ar,x_al,z_ar,z_al;
    double pitch_al=0;
    double pitch_ar=0;



    if (n==1){//left foot moves in first step

        x_ar=0;
        z_ar=_lenghtOfAnkle;
        x_al=0;
        z_al=_lenghtOfAnkle;

        if (localtiming>=T_s_st && localtiming<=TStart-T_end_of_first_SS){
            MatrixXd output=CoefOffline.GetAccVelPos(C_st_x_al.row(0),localtiming,T_s_st,5);
            x_al=output(0,0);

            if (localtiming<=(((TStart+T_s_st)/2))){
                MatrixXd output=CoefOffline.GetAccVelPos(C_st_z_al.row(0),localtiming,T_s_st,5);
                z_al=_lenghtOfAnkle+output(0,0);

            }
            else if (localtiming<=(TStart-T_end_of_first_SS)){
                MatrixXd output=CoefOffline.GetAccVelPos(C_st_z_al.row(1),localtiming,((TStart+T_s_st)/2),5);
                z_al=_lenghtOfAnkle+output(0,0);
                         }
        }
        else if(localtiming>TStart-T_end_of_first_SS){
            LeftFootOrientationAdaptator=true;
            MatrixXd output1=CoefOffline.GetAccVelPos(C_st_x_al.row(0),TStart-T_end_of_first_SS,T_s_st,5);
            x_al=output1(0,0);
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
            z_al=currentLeftFootZ;

            x_ar=currentRightFootX2;
            z_ar=currentRightFootZ;

  if(beta_heel!=0||beta_toe!=0){
            if(localtiming>(TDs-t_toe)){
                MatrixXd output=CoefOffline.GetAccVelPos(C_beta_toe_cycle,localtiming-(TDs-t_toe),0,5);
                pitch_ar=output(0,0);

            }

            if(localtiming<t_heel&&n>1){
                pitch_al=beta_heel;
            }


            if(localtiming>t_heel&&n>1){
                MatrixXd output=CoefOffline.GetAccVelPos(C_beta_heel_cycle,localtiming-t_heel,0,5);
                pitch_al=output(0,0);

            }
}
        }

        else if (localtiming<Tc-T_end_of_SS){//single support of cyclic walking

            if (localtiming<TDs+Tm2){

               MatrixXd output1=CoefOffline.GetAccVelPos(C_cy_x_ar.row(0),localtiming-TDs,0,5);
               x_al=(footIndex!=0)*currentLeftFootX2+(footIndex==0)*(footIndex==0)*(currentLeftFootX2+output1(0,0));
               x_ar=(footIndex!=0)*(currentRightFootX2+output1(0,0))+(footIndex==0)*currentRightFootX2;


           }
           else{
                MatrixXd output1=CoefOffline.GetAccVelPos(C_cy_x_ar.row(1),localtiming-TDs,Tm2,5);
               x_al=(footIndex!=0)*currentLeftFootX2+(footIndex==0)*(footIndex==0)*(currentLeftFootX2+output1(0,0));
               x_ar=(footIndex!=0)*(currentRightFootX2+output1(0,0))+(footIndex==0)*currentRightFootX2;

           }
if(beta_heel!=0||beta_toe!=0){
            if (localtiming<TDs+TStartofHeel){
                MatrixXd output2=CoefOffline.GetAccVelPos(C_beta_toe2heel_cycle.row(0),localtiming-TDs,0,5);
                pitch_ar=output2(0,0);

            }
            else{MatrixXd output2=CoefOffline.GetAccVelPos(C_beta_toe2heel_cycle.row(1),localtiming-TDs,TStartofHeel,5);
                pitch_ar=output2(0,0);}

        }

            if (localtiming<TDs+Tm1){

               MatrixXd output2=CoefOffline.GetAccVelPos(C_cy_z_ar.row(0),localtiming-TDs,0,5);
               z_al=(footIndex!=0)*currentLeftFootZ+(footIndex==0)*(currentLeftFootZ+output2(0,0));
               z_ar=(footIndex!=0)*(currentRightFootZ+output2(0,0))+(footIndex==0)*currentRightFootZ;
           }

            else if (localtiming<TDs+Tm2){

                MatrixXd output2=CoefOffline.GetAccVelPos(C_cy_z_ar.row(1),localtiming-TDs,0,5);//-Tm1
                z_al=(footIndex!=0)*currentLeftFootZ+(footIndex==0)*(currentLeftFootZ+output2(0,0));
                z_ar=(footIndex!=0)*(currentRightFootZ+output2(0,0))+(footIndex==0)*currentRightFootZ;

            }
            else{

                MatrixXd output2=CoefOffline.GetAccVelPos(C_cy_z_ar.row(2),localtiming-TDs,0,5);//-Tm2
                z_al=(footIndex!=0)*currentLeftFootZ+(footIndex==0)*(currentLeftFootZ+output2(0,0));
                z_ar=(footIndex!=0)*(currentRightFootZ+output2(0,0))+(footIndex==0)*currentRightFootZ;
            }


        }

        else if (localtiming<Tc){
            if(beta_heel!=0||beta_toe!=0){
            MatrixXd output1=CoefOffline.GetAccVelPos(C_beta_toe2heel_cycle.row(1),localtiming-TDs,TStartofHeel,5); // Tm2
                        pitch_ar=output1(0,0);
                        }

            LeftFootOrientationAdaptator=(footIndex==0);
            RightFootOrientationAdaptator=(footIndex!=0);

            x_al=(footIndex!=0)*currentLeftFootX2+(footIndex==0)*(currentLeftFootX2+2*StepLength);
            x_ar=(footIndex!=0)*(currentRightFootX2+2*StepLength)+(footIndex==0)*currentRightFootX2;

            MatrixXd output2=CoefOffline.GetAccVelPos(C_cy_z_ar_end_of_SS.row(0),localtiming-Tc+TSS,TSS-T_end_of_SS,5);


           z_al=(footIndex!=0)*currentLeftFootZ+(footIndex==0)*(currentLeftFootZ+output2(0,0));
           z_ar=(footIndex!=0)*(currentRightFootZ+output2(0,0))+(footIndex==0)*currentRightFootZ;

       }
        if(footIndex==0){
                    double temp= pitch_ar;
                    pitch_ar=pitch_al;
                    pitch_al=temp;
                }
    }

    if(n==NStep+2){//end step of walk right foot moves


            n=n-1;
            currentLeftFootX2=(n-fmod(n+1,2))*StepLength+side_extra_step_length*StepLength;
            currentLeftFootZ=0.112;
            currentRightFootX2=(n-fmod(n,2))*StepLength;
            currentRightFootZ=0.112;

            if (localtiming<=(TDs+0.001)){// double support of end
                z_ar=currentRightFootZ;
                z_al=currentLeftFootZ;
                x_ar=currentRightFootX2;
                x_al=currentLeftFootX2;
            }


        else {

            if (localtiming<=(T_end_a_e-T_end_a_s-T_end_of_last_SS)+TDs){

                MatrixXd output1=CoefOffline.GetAccVelPos(C_end_x_ar.row(0),localtiming-TDs,0,5);
                x_ar=currentRightFootX2+output1(0,0);
                x_al=currentLeftFootX2;//*/(2*NStride+1)*StepLength;/*
                z_al=currentLeftFootZ;

                if (localtiming<=(T_end_a_e-T_end_a_s)/2+TDs){
                    MatrixXd output2=CoefOffline.GetAccVelPos(C_end_z_ar.row(0),localtiming-TDs,0,5);
                    z_ar=currentRightFootZ+output2(0,0);//currentRightFootZ+output2(0,0);

                }

                else if (localtiming>=(T_end_a_e-T_end_a_s)/2+TDs){
                    MatrixXd output2=CoefOffline.GetAccVelPos(C_end_z_ar.row(1),localtiming-TDs, (T_end_a_e-T_end_a_s)/2,5);
                    z_ar=currentRightFootZ+output2(0,0);
                }

            }
            else if (localtiming<=(T_end_a_e-T_end_a_s)+TDs){
                RightFootOrientationAdaptator=true;
                MatrixXd output1=CoefOffline.GetAccVelPos(C_end_x_ar.row(0),(T_end_a_e-T_end_a_s-T_end_of_last_SS),0,5);
                x_ar=currentRightFootX2+output1(0,0);
                x_al=currentLeftFootX2;//*/(2*NStride+1)*StepLength;/*
                z_al=currentLeftFootZ;
                MatrixXd output2=CoefOffline.GetAccVelPos(C_end_z_ar_end_of_SS.row(0),localtiming,T_end_a_e-T_end_a_s-T_end_of_last_SS+TDs,5);
                z_ar=currentRightFootZ+output2(0,0);
            }

            else{

                MatrixXd output1=CoefOffline.GetAccVelPos(C_end_x_ar.row(0),(T_end_a_e-T_end_a_s-T_end_of_last_SS),0,5);
                x_ar=currentRightFootX2+output1(0,0);
                z_ar=currentRightFootZ;//_lenghtOfAnkle;
                x_al=currentLeftFootX2;//(2*NStride+1)*StepLength;
                z_al=currentLeftFootZ;//_lenghtOfAnkle;
            }


    }
}

    MatrixXd footpos(8,1);
    footpos<<x_al,y_al,z_al,pitch_al,x_ar,y_ar,z_ar,pitch_ar;

    return footpos;

}



