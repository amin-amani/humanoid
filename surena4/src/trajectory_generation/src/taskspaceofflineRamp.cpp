#include "taskspaceofflineRamp.h"
TaskSpaceOfflineRamp::TaskSpaceOfflineRamp()
{

    //defines vectors for plotting
        RightFootXVelocity.append(0);
        RightFootYVelocity.append(0);
        RightFootXTrajectory.append(0);
        RightFootYTrajectory.append(0);
        RightFootXAcceleration.append(0);
        RightFootYAcceleration.append(0);
        RightFootZTrajectory.append(_lenghtOfAnkle);
        RightFootZVelocity.append(0);
        RightFootZacceleration.append(0);
        RightFootAlphaTrajectory.append(0);
        RightFootBethaTrajectory.append(0);
        RightFootGamaTrajectory.append(0);

        LeftFootXVelocity.append(0);
        LeftFootYVelocity.append(0);
        LeftFootXTrajectory.append(0);
        LeftFootYTrajectory.append(0);
        LeftFootXAcceleration.append(0);
        LeftFootYAcceleration.append(0);
        LeftFootZTrajectory.append(_lenghtOfAnkle);
        LeftFootZVelocity.append(0);
        LeftFootZAcceleration.append(0);
        LeftFootAlphaTrajectory.append(0);
        LeftFootBethaTrajectory.append(0);
        LeftFootGamaTrajectory.append(0);

        CoMXVelocityVector.append(0);
        CoMYVelocityVector.append(0);
        CoMZVelocityVector.append(0);
        CoMXVector.append(0);
        CoMYVector.append(0);
        CoMZVector.append(0);

        globalTime=0;
        timeVector.append(0);
         time=0;
       _timeStep=0.01;
        SetParameters();
        CoeffArrayAnkle();
        CoeffArrayPelvis();
        //CoeffArrayFootAngle();

        bool _walkstate=true;


}

void TaskSpaceOfflineRamp::SetParameters(){
    YOffsetOfAnkletrajectory=0.04;//for compensating the clearance of the hip roll in experiment
       er=0.000;
       Rqa=0.95;
       Ra_i=0;
       Ra_f=0.93;
       Rla_i=1;
       Rla_f=1;

       toeOff=false;
       HipRollModification=true;


       Ra_st_i=0;
       Ra_st_f=1;


       Ra_end_i=0;
       Ra_end_f=1;


       Rla_st_i=1;
       Rla_st_f=1;

       Rla_end_i=1;
       Rla_end_f=1;

       HeelLandingAnglePitch=8*0;
       ToeOffAnglePitch=-6*0;
       Sc=0.3661;//change
       Rse=1.7315;//change
       Rd=0.7418;//change
       Rm=1.1607;//change
       Rzp=0.9288;//change


       Sc=0.3887;
       Rse=0.5;
       Rd=0.7;
       Rm=1.3;
       Rzp=0.82;


       _lengthOfHip=0.10900;
       _lenghtOfAnkle=0.112000;
       _lengthOfShank=0.3600;//change
       _lengthOfThigh=0.3700;//change
       _heelLength=0.0805;//change
       _toeLength=0.1695;//change
       _pelvisLength=0.23;




       Delta=0.02;// Domain of pelvis movemevt in z direction
       NStride=4;
       DesiredVelocity=0.2;
       StepLength=0.285;

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
       //double motiontime;
       // motiontime=5;


//       YpMax=1.1*Rm*0.5*_pelvisLength;
//       Yd=1.1*Rd*YpMax;
//       YEndMax=1.0*YpMax;
//       YStMax=1.0*YpMax;
//       Xs=Rse*Xe;


       //xa_st_m=(0.22/0.35)*StepLength/2;
       // za_st_m=(AnkleMaximumHeight);
       Xe=Sc*StepLength/(Rse+1);
       xa_end_m_Flat=2*NStride*StepLength+(0.22/0.35)*StepLength/2;// Position of xa_end_m_Flatankle in x direction when it reaches maximum heigth in last step is used for slope
       za_end_m_Flat=(AnkleMaximumHeight);

       L_2leg_Ds=sqrt(pow((_lengthOfShank+_lengthOfThigh),2)-pow((0.5*StepLength),2))+_lenghtOfAnkle+_lengthOfHip;
       MinHeightPelvis=Rzp*L_2leg_Ds; // minimum heigth of pelvis

       ReferencePelvisHeight=0.83;

       MaxHeightPelvis=MinHeightPelvis+Delta;
       if (true) {
           Xe=1.1*Sc*StepLength/(Rse+1);
           Xs=0.9*Rse*Xe;
           YpMax=1*Rm*0.5*_pelvisLength;
           Yd=0.85*Rd*YpMax;
           YStMax=1.0*YpMax;
         YEndMax=1.0*YpMax;
//           Xe=1.3*Sc*StepLength/(Rse+1);
//           Xs=0.3*Rse*Xe;
//           YpMax=0.85*Rm*0.5*_pelvisLength;
//           Yd=0.85*Rd*YpMax;
//           YStMax=1*YpMax;
//           YEndMax=1*YpMax;
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



       //change
       //slope parameter
       slopeAngleX=0;
       slopeAngleY=1*8*M_PI/180;

       xAnkleToStartOfSlope=0.2;
       zAnkleToStartOfSlope=0.02;
       xAnkleToEndOfSlope=0.12;
       zAnkleToEndOfSlope=0.00;

       transformSlopetoFlat.resize(4,4);
       transformSlopetoFlat.block(0,0,3,3)=MatrixXd::Identity(3,3);
       MatrixXd temp1Mat(1,4);
       temp1Mat<<0,0,0,1;
       transformSlopetoFlat.block(3,0,1,4)=temp1Mat;
       MatrixXd temp2Mat(3,1);
       temp2Mat<<(xAnkleToStartOfSlope+(StepLength-xAnkleToStartOfSlope)*cos(slopeAngleY)),0,(zAnkleToStartOfSlope+(StepLength-xAnkleToStartOfSlope)*sin(slopeAngleY));
       transformSlopetoFlat.block(0,3,3,1)=temp2Mat;

       rotationSlopeX.resize(4,4);
       rotationSlopeX<<1,0,0,0,
               0,cos(-slopeAngleX),-sin(-slopeAngleX),0,
               0,sin(-slopeAngleX),cos(-slopeAngleX),0,
               0,0,0,1;

       rotationSlopeY.resize(4,4);
       rotationSlopeY<< cos(-slopeAngleY),0,sin(-slopeAngleY),0,
               0,1,0,0,
               -sin(-slopeAngleY),0,cos(-slopeAngleY),0,
               0,0,0,1;

       TransitionSlopeToFlat.resize(4,4);
       TransitionSlopeToFlat=transformSlopetoFlat*rotationSlopeY*rotationSlopeX;
       finalHeightOfAnkle=_lenghtOfAnkle+zAnkleToStartOfSlope+((NStride*2+1)*StepLength-xAnkleToStartOfSlope-xAnkleToEndOfSlope)*sin(slopeAngleY)+zAnkleToEndOfSlope;
       finalXPositionOfAnkle=xAnkleToStartOfSlope+((NStride*2+1)*StepLength-xAnkleToStartOfSlope-xAnkleToEndOfSlope)*cos(slopeAngleY)+xAnkleToEndOfSlope;

       MatrixXd temp3Mat(4,1);
       temp3Mat<<xa_end_m_Flat-StepLength,0,za_end_m_Flat,1;
       MatrixXd TempSlopeEndM(4,1);
       TempSlopeEndM=TransitionSlopeToFlat*temp3Mat;
       xa_end_m_Slope=TempSlopeEndM(0,0);
       za_end_m_Slope=TempSlopeEndM(2,0);


       MatrixXd temp6Mat(4,1);
       temp6Mat<<0,0,AnkleMaximumHeight,1;
       MatrixXd Temp666(4,1);
       Temp666=TransitionSlopeToFlat*temp6Mat;
       maxHightOfAnkleOnSlope=1*Temp666(2,0);//for start left foot




       finalPositionOfRightAnkleOnSlop.resize(4,1);
       finalPositionOfRightAnkleOnSlop<<(2*NStride-1)*StepLength,-_pelvisLength*0.5,_lenghtOfAnkle,1;//consider that it is relative to the slope frame
       MatrixXd finalPosOfRAnkleOnSlopRelativeToFlatGround= TransitionSlopeToFlat*finalPositionOfRightAnkleOnSlop;

       xa_end_st_slope=finalPosOfRAnkleOnSlopRelativeToFlatGround(0,0);
       za_end_st_slope=finalPosOfRAnkleOnSlopRelativeToFlatGround(2,0);

       xp_first_l=xAnkleToStartOfSlope+(StepLength-xAnkleToStartOfSlope)*cos(slopeAngleY);
       zp_first_l=zAnkleToStartOfSlope+(StepLength-xAnkleToStartOfSlope)*sin(slopeAngleY);

       xp_last_r=xAnkleToStartOfSlope+(2*NStride*StepLength-xAnkleToStartOfSlope)*cos(slopeAngleY);
       zp_last_r=zAnkleToStartOfSlope+(2*NStride*StepLength-xAnkleToStartOfSlope)*sin(slopeAngleY);

       xp_end_r=xAnkleToStartOfSlope+((2*NStride+1)*StepLength-xAnkleToStartOfSlope-xAnkleToEndOfSlope)*cos(slopeAngleY)+xAnkleToEndOfSlope;
       zp_end_r=zAnkleToStartOfSlope+((2*NStride+1)*StepLength-xAnkleToStartOfSlope-xAnkleToEndOfSlope)*sin(slopeAngleY)+zAnkleToEndOfSlope;

       relative_firstPositionofLeftANkleOnSlope.resize(4,1);
       relative_firstPositionofLeftANkleOnSlope<<0,0.5*_pelvisLength,_lenghtOfAnkle,1;
       firstPositionofLeftANkleOnSlope=TransitionSlopeToFlat*relative_firstPositionofLeftANkleOnSlope;

       relative_firstPositionofRightANkleOnSlope.resize(4,1);
       relative_firstPositionofRightANkleOnSlope<<StepLength,-1*0.5*_pelvisLength,_lenghtOfAnkle,1;
       firstPositionofRightANkleOnSlope=TransitionSlopeToFlat*relative_firstPositionofRightANkleOnSlope;

       MatrixXd tempMat4(4,1);
       tempMat4<<2*StepLength,-1*0.5*_pelvisLength,_lenghtOfAnkle,1;
       MappingstepLengthFromSlopeToFlat=TransitionSlopeToFlat*tempMat4;

       changeOfXPositionOfAnkleInOneStep=(MappingstepLengthFromSlopeToFlat(0,0)-firstPositionofLeftANkleOnSlope(0,0));
       changeOfZPositionOfAnkleInOneStep=(MappingstepLengthFromSlopeToFlat(2,0)-firstPositionofLeftANkleOnSlope(2,0));

       FinalZleftAnkleOnSlope_Abs=firstPositionofLeftANkleOnSlope(2,0)+(NStride-1)*changeOfZPositionOfAnkleInOneStep;
       FinalXleftAnkleOnSlope_Abs=firstPositionofLeftANkleOnSlope(0,0)+(NStride-1)*changeOfXPositionOfAnkleInOneStep;

       FinalXRightAnkleOnSlope_Abs=firstPositionofRightANkleOnSlope(0,0)+(NStride-1)*changeOfXPositionOfAnkleInOneStep;
       FinalZRightAnkleOnSlope_Abs=firstPositionofRightANkleOnSlope(2,0)+(NStride-1)*changeOfZPositionOfAnkleInOneStep;

       changeOfXPositionOfRightAnkleInLastStep=finalXPositionOfAnkle-FinalXRightAnkleOnSlope_Abs;
       changeOfZPositionOfRightAnkleInLastStep=finalHeightOfAnkle-FinalZRightAnkleOnSlope_Abs;

       changeOfXPositionOfLeftAnkleInLastStep=finalXPositionOfAnkle-FinalXleftAnkleOnSlope_Abs;
       changeOfZPositionOfLeftAnkleInLastStep=finalHeightOfAnkle-FinalZleftAnkleOnSlope_Abs;

}



void TaskSpaceOfflineRamp::CoeffArrayPelvis(){
    MatrixXd ord(1,2);
       ord << 3,3;
       MatrixXd ttt(1,3);
       ttt <<0 ,TDs, Tc;
       MatrixXd con(3,3);
       con<<Xe, cos(slopeAngleY)*StepLength-Xs, cos(slopeAngleY)*StepLength+Xe,INFINITY, INFINITY ,INFINITY,INFINITY, INFINITY ,INFINITY;
       Cx_p_i.resize(2,6);
       Cx_p_i.fill(0);
       Cx_p_i.block(0,2,2,4)=CoefOffline.Coefficient1(ttt,ord,con,0.1).transpose();
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

       // ----------------Coefficient of End of Pelvis motion in x direction edited for slope----------------------

       MatrixXd Cx_end_pTime(1,2);
       Cx_end_pTime<<(TGait+TDs),T_end_p_sx ;
       MatrixXd Cx_end_pPos(1,2);
       Cx_end_pPos<<Cx_p(0,0)*pow(TDs,5)+Cx_p(0,1)*pow(TDs,4)+Cx_p(0,2)*pow(TDs,3)+Cx_p(0,3)*pow(TDs,2)+Cx_p(0,4)*pow(TDs,1)+Cx_p(0,5)+(2*NStride)*StepLength*cos(slopeAngleY), xAnkleToStartOfSlope+((2*NStride+1)*StepLength-xAnkleToStartOfSlope-xAnkleToEndOfSlope)*cos(slopeAngleY)+xAnkleToEndOfSlope;
       MatrixXd Cx_end_pVel(1,2);
       Cx_end_pVel<<5*Cx_p(0,0)*pow(TDs,4)+4*Cx_p(0,1)*pow(TDs,3)+3*Cx_p(0,2)*pow(TDs,2)+2*Cx_p(0,3)*pow(TDs,1)+Cx_p(0,4), 0;
       MatrixXd Cx_end_pAccel(1,2);
       Cx_end_pAccel<<5*4*Cx_p(0,0)*pow(TDs,4)+4*3*Cx_p(0,1)*pow(TDs,3)+3*2*Cx_p(0,2)*pow(TDs,2)+2*Cx_p(0,3), 0;//like the last moment of first part of trajectory of cycle
       Cx_end_p=CoefOffline.Coefficient(Cx_end_pTime,Cx_end_pPos,Cx_end_pVel,Cx_end_pAccel);

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

       //------------------Coefficient of Cyclic motion in Z direction--------------------
       MatrixXd Cz_p_pTime(1,3);
       Cz_p_pTime<<TMinPelvisZ, TMaxPelvisZ,Tc+TMinPelvisZ ;
       MatrixXd Cz_p_pPos(1,3);
       Cz_p_pPos<<MinHeightPelvis, MaxHeightPelvis,MinHeightPelvis;
       MatrixXd Cz_p_pVel(1,3);
       Cz_p_pVel<<0, 0 ,0;
       MatrixXd Cz_p_pAccel(1,3);
       Cz_p_pAccel<<INFINITY,INFINITY,INFINITY;
       Cz_p=CoefOffline.Coefficient(Cz_p_pTime,Cz_p_pPos,Cz_p_pVel,Cz_p_pAccel);


       //-------------- Coefficient of Start of pelvis motion in Z direction----------------
       MatrixXd Cz_st_pTime(1,2);
       Cz_st_pTime<<T_st_p_sz, T_st_p_dz ;
       MatrixXd Cz_st_pPos(1,2);
       Cz_st_pPos<<ReferencePelvisHeight ,MaxHeightPelvis;
       MatrixXd Cz_st_pVel(1,2);
       Cz_st_pVel<<0, 0 ;
       MatrixXd Cz_st_pAccel(1,2);
       Cz_st_pAccel<<0,0;
       Cz_st_a=CoefOffline.Coefficient(Cz_st_pTime,Cz_st_pPos,Cz_st_pVel,Cz_st_pAccel);


       MatrixXd Cz_st_pbTime(1,2);
       Cz_st_pbTime<<T_st_p_ez, TStart ;
       MatrixXd Cz_st_pbPos(1,2);
       Cz_st_pbPos<<MaxHeightPelvis ,Cz_p(1,0)*pow(Tc-TMaxPelvisZ,5)+Cz_p(1,1)*pow(Tc-TMaxPelvisZ,4)+Cz_p(1,2)*pow(Tc-TMaxPelvisZ,3)+Cz_p(1,3)*pow(Tc-TMaxPelvisZ,2)+Cz_p(1,4)*pow(Tc-TMaxPelvisZ,1)+Cz_p(1,5);
       MatrixXd Cz_st_pbVel(1,2);
       Cz_st_pbVel<<0, 5*Cz_p(1,0)*pow(Tc-TMaxPelvisZ,4)+4*Cz_p(1,1)*pow(Tc-TMaxPelvisZ,3)+3*Cz_p(1,2)*pow(Tc-TMaxPelvisZ,2)+2*Cz_p(1,3)*pow(Tc-TMaxPelvisZ,1)+Cz_p(1,4)*pow(Tc-TMaxPelvisZ,0) ;
       MatrixXd Cz_st_pbAccel(1,2);
       Cz_st_pbAccel<<0,5*4*Cz_p(1,0)*pow(Tc-TMaxPelvisZ,3)+4*3*Cz_p(1,1)*pow(Tc-TMaxPelvisZ,2)+3*2*Cz_p(1,2)*pow(Tc-TMaxPelvisZ,1)+2*Cz_p(1,3)*pow(Tc-TMaxPelvisZ,0);
       Cz_st_b=CoefOffline.Coefficient(Cz_st_pbTime,Cz_st_pbPos,Cz_st_pbVel,Cz_st_pbAccel);


       //--------------Coefficient of End of Pelvis motion in Z direction----------------
       MatrixXd Cz_end_pTime(1,2);
       Cz_end_pTime<<TGait+TDs, T_end_p_sz ;
       MatrixXd Cz_end_pPos(1,2);
       Cz_end_pPos<<Cz_p(0,0)*pow(TDs-TMinPelvisZ,5)+Cz_p(0,1)*pow(TDs-TMinPelvisZ,4)+Cz_p(0,2)*pow(TDs-TMinPelvisZ,3)+Cz_p(0,3)*pow(TDs-TMinPelvisZ,2)+Cz_p(0,4)*pow(TDs-TMinPelvisZ,1)+Cz_p(0,5) ,MaxHeightPelvis;
       MatrixXd Cz_end_pVel(1,2);
       Cz_end_pVel<<5*Cz_p(0,0)*pow(TDs-TMinPelvisZ,4)+4*Cz_p(0,1)*pow(TDs-TMinPelvisZ,3)+3*Cz_p(0,2)*pow(TDs-TMinPelvisZ,2)+2*Cz_p(0,3)*pow(TDs-TMinPelvisZ,1)+Cz_p(0,4)*pow(TDs-TMinPelvisZ,0) , 0 ;
       MatrixXd Cz_end_pAccel(1,2);
       Cz_end_pAccel<<5*4*Cz_p(0,0)*pow(TDs-TMinPelvisZ,3)+4*3*Cz_p(0,1)*pow(TDs-TMinPelvisZ,2)+3*2*Cz_p(0,2)*pow(TDs-TMinPelvisZ,1)+2*Cz_p(0,3)*pow(TDs-TMinPelvisZ,0),0;
       Cz_end_a=CoefOffline.Coefficient(Cz_end_pTime,Cz_end_pPos,Cz_end_pVel,Cz_end_pAccel);


       MatrixXd Cz_end_pbTime(1,2);
       Cz_end_pbTime<<T_end_p_dz, T_end_p_ez;
       MatrixXd Cz_end_pbPos(1,2);
       Cz_end_pbPos<<MaxHeightPelvis,ReferencePelvisHeight;
       MatrixXd Cz_end_pbVel(1,2);
       Cz_end_pbVel<<0,0;
       MatrixXd Cz_end_pbAccel(1,2);
       Cz_end_pbAccel<<0,0;
       Cz_end_b=CoefOffline.Coefficient(Cz_end_pbTime,Cz_end_pbPos,Cz_end_pbVel,Cz_end_pbAccel);

       //Pelvis height relative to flat ground global frame for slope
       ///////////////////////////////////////////////////////////////////////////

       MatrixXd Cz_Off_st_X(1,2);
       Cz_Off_st_X<<0, xp_first_l;
       MatrixXd Cz_off_st_Pos(1,2);
       Cz_off_st_Pos<<0,zp_first_l;
       MatrixXd Cz_off_st_Vel(1,2);
       Cz_off_st_Vel<<0,tan(slopeAngleY);
       MatrixXd Cz_off_st_Accel(1,2);
       Cz_off_st_Accel<<0,0;
       Cz_Off_st_slope=CoefOffline.Coefficient(Cz_Off_st_X,Cz_off_st_Pos,Cz_off_st_Vel,Cz_off_st_Accel);


       MatrixXd Cz_Off_end_X(1,2);
       Cz_Off_end_X<<xp_last_r, xp_end_r;
       MatrixXd Cz_off_end_Pos(1,2);
       Cz_off_end_Pos<<zp_last_r,zp_end_r;
       MatrixXd Cz_off_end_Vel(1,2);
       Cz_off_end_Vel<<tan(slopeAngleY),0;
       MatrixXd Cz_off_end_Accel(1,2);
       Cz_off_end_Accel<<0,0;
       Cz_Off_end_slope=CoefOffline.Coefficient(Cz_Off_end_X,Cz_off_end_Pos,Cz_off_end_Vel,Cz_off_end_Accel);

       qDebug()<<2;

}


MatrixXd TaskSpaceOfflineRamp::PelvisTrajectory(double time){
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
           xp=output(0,0)+StepLength*cos(slopeAngleY);
           dxp=output(0,1);
           ddxp=output(0,2);
           DoubleSupport=true;
       }
       else if (t>(Tc+TDs+TStart) && t<=(2*Tc+TStart)){
           MatrixXd output=GetAccVelPos(Cx_p_i.row(1),t-Tc-TStart,0,5);
           xp=output(0,0)+StepLength*cos(slopeAngleY);
           dxp=output(0,1);
           ddxp=output(0,2);
           DoubleSupport=false;
       }
       else if (t>TGait && t<(TGait+TDs)){
           MatrixXd output=GetAccVelPos(Cx_p_i.row(0),t-TGait,0,5);
           xp=output(0,0)+2*NStride*StepLength*cos(slopeAngleY);
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
           xp=finalXPositionOfAnkle;
           dxp=0;
           ddxp=0;
           DoubleSupport=true;
       }

       if (TStart==0 && t==0){
           xp=Cx_p(4);
           dxp=0;
           ddxp=0;
       }

       xp=xp+2*StepLength*cos(slopeAngleY)*N;

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
           zp=ReferencePelvisHeight;
           dzp=0;
           ddzp=0;
           // zp=polyval(Cz_st_a,t);
       }
       else if (t>T_st_p_dz && t<=T_st_p_ez){

           zp=ReferencePelvisHeight;
           dzp=0;
           ddzp=0;
       }
       else if (t>T_st_p_ez && t<=TStart){
           MatrixXd output=GetAccVelPos(Cz_st_b,t,T_st_p_ez,5);
           zp=ReferencePelvisHeight;
           dzp=0;
           ddzp=0;
           //zp=polyval(Cz_st_b,t);
       }

       else if (t>TStart && t<=(TMinPelvisZ+TStart)){
           MatrixXd output=GetAccVelPos(Cz_p.row(1),t+Tc-TStart,TMaxPelvisZ,5);
           zp=ReferencePelvisHeight;
           dzp=0;
           ddzp=0;
           //  zp=polyval(Cz_p(5:8) ,t+Tc-T_st);
           //TMinPelvisZ, TMaxPelvisZ,Tc+TMinPelvisZ
       }
       else if  (t>(TMinPelvisZ+TStart) && t<=(TMaxPelvisZ+TStart)){
           MatrixXd output=GetAccVelPos(Cz_p.row(0),t-TStart,TMinPelvisZ,5);
           zp=ReferencePelvisHeight;
           dzp=0;
           ddzp=0;
           //zp=polyval(Cz_p(1:4), t-T_st);
       }
       else if (t>(TMaxPelvisZ+TStart) && t<=(Tc+TMinPelvisZ+TStart)){
           MatrixXd output=GetAccVelPos(Cz_p.row(1),t-TStart,TMaxPelvisZ,5);
           zp=ReferencePelvisHeight;
           dzp=0;
           ddzp=0;
           // zp=polyval(Cz_p(5:8), t-T_st);
       }
       else if (t>(Tc+TMinPelvisZ+TStart) && t<=(Tc+TMaxPelvisZ+TStart)){
           MatrixXd output=GetAccVelPos(Cz_p.row(0),t-TStart-Tc,TMinPelvisZ,5);
           zp=ReferencePelvisHeight;
           dzp=0;
           ddzp=0;
           //zp=polyval(Cz_p(1:4),t-Tc-T_st);
       }
       else if (t>(Tc+TMaxPelvisZ+TStart) && t<=(2*Tc+TStart)){
           MatrixXd output=GetAccVelPos(Cz_p.row(1),t-TStart-Tc,TMaxPelvisZ,5);
           zp=ReferencePelvisHeight;
           dzp=0;
           ddzp=0;
           //zp=polyval(Cz_p(5:8),t-Tc-T_st);
       }

       else if (t>TGait && t<=(TMinPelvisZ+TGait)){
           MatrixXd output=GetAccVelPos(Cz_p.row(1),t+Tc-TGait,TMaxPelvisZ,5);
           zp=ReferencePelvisHeight;
           dzp=0;
           ddzp=0;
           //zp=polyval(Cz_p(5:8),t+Tc-T_Gait);
       }

       else if (t>(TMinPelvisZ+TGait) && t<=(TDs+TGait)){
           MatrixXd output=GetAccVelPos(Cz_p.row(0),t-TGait,TMinPelvisZ,5);
           zp=ReferencePelvisHeight;
           dzp=0;
           ddzp=0;;
           // zp=polyval(Cz_p(1:4),t-T_Gait);
       }

       else if (t>(TDs+TGait) && t<=T_end_p_sz){
           MatrixXd output=GetAccVelPos(Cz_end_a,t,TGait+TDs,5);
           zp=ReferencePelvisHeight;
           dzp=0;
           ddzp=0;
           // zp=polyval(Cz_end_a,t);
       }
       else if (t>T_end_p_sz && t<=T_end_p_dz){
           zp=ReferencePelvisHeight;
           dzp=0;
           ddzp=0;
       }
       else if (t>T_end_p_dz && t<=T_end_p_ez){
           //       zp=zp_max;T_end_p_dz
           MatrixXd output=GetAccVelPos(Cz_end_b,t,T_end_p_dz,5);
           zp=ReferencePelvisHeight;
           dzp=0;
           ddzp=0;
           //  zp=polyval(Cz_end_b,t);
       }
       else if (t>T_end_p_ez && t<=(TGait+TDs+TEnd)){
           zp=finalHeightOfAnkle-_lenghtOfAnkle+ReferencePelvisHeight;//+;//MaxHeightPelvis
           dzp=0;
           ddzp=0;
           //zp=z_Off_st_slope+((2*N_Stride+1)*Ds-x_Off_st_slope-x_z_p_ref;
           //
       }

       // for height modification on slope
       if (xp<=xp_first_l) {
           MatrixXd output=GetAccVelPos(Cz_Off_st_slope,xp,0,5);
           zp=zp+output(0,0);
       }
       else if (xp>xp_first_l && xp<=xp_last_r) {
           zp=zp+zAnkleToStartOfSlope+(xp-xAnkleToStartOfSlope)*tan(slopeAngleY);

       }
       else if (xp>xp_last_r) {
           MatrixXd output=GetAccVelPos(Cz_Off_end_slope,xp,xp_last_r,5);
           zp=zp+output(0,0);
           qDebug()<<2;
       }





       MatrixXd pelvis(9,1);
       pelvis<<xp,yp,zp,dxp,dyp,dzp,ddxp,ddyp,ddzp;
       return pelvis;


}




void TaskSpaceOfflineRamp::CoeffArrayAnkle(){
    T_s_st=.5*TStart;
    // for first step of left foot on slope xxxx
    MatrixXd Cx_st_iTime_al(1,2);
    Cx_st_iTime_al<<T_s_st ,TStart;
    MatrixXd Cx_st_iPos_al(1,2);
    Cx_st_iPos_al<<0, firstPositionofLeftANkleOnSlope(0,0);
    MatrixXd Cx_st_iVel_al(1,2);
    Cx_st_iVel_al<<0, 0;
    MatrixXd Cx_st_iAcc_al(1,2);
    Cx_st_iAcc_al<<0, 0;
    C_st_x_al=CoefOffline.Coefficient(Cx_st_iTime_al,Cx_st_iPos_al,Cx_st_iVel_al,Cx_st_iAcc_al);

    // for first step of left foot on slope zzzz
    MatrixXd C_st_iTime(1,3);
    C_st_iTime<<T_s_st, TStart-T_s_st/2 ,TStart;
    MatrixXd C_st_iPos(1,3);
    C_st_iPos<<_lenghtOfAnkle, AnkleMaximumHeight,firstPositionofLeftANkleOnSlope(2,0);
    MatrixXd C_st_iVel(1,3);
    C_st_iVel<<0 ,INFINITY, 0;
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

    // for cyclic walk left foot on slope xxx
    MatrixXd C_cy_iTime_al(1,2);
    C_cy_iTime_al<<0, TSS;
    MatrixXd C_cy_iPos_al(1,2);
    C_cy_iPos_al<<0, changeOfXPositionOfAnkleInOneStep;
    MatrixXd C_cy_iVel_al(1,2);
    C_cy_iVel_al<<0, 0;
    MatrixXd C_cy_iAcc_al(1,2);
    C_cy_iAcc_al<<0, 0;
    C_cy_x_al=CoefOffline.Coefficient(C_cy_iTime_al,C_cy_iPos_al,C_cy_iVel_al,C_cy_iAcc_al);


    // for first step of right foot on slope xxxx
    MatrixXd Cx_st_Time_ar(1,2);
    Cx_st_Time_ar<<0, TSS;
    MatrixXd Cx_st_Pos_ar(1,2);
    Cx_st_Pos_ar<<0, firstPositionofRightANkleOnSlope(0,0);
    MatrixXd Cx_st_Vel_ar(1,2);
    Cx_st_Vel_ar<<0, 0;
    MatrixXd Cx_st_acc_ar(1,2);
    Cx_st_acc_ar<<0, 0;
    C_st_x_slope_ar=CoefOffline.Coefficient(Cx_st_Time_ar,Cx_st_Pos_ar,Cx_st_Vel_ar,Cx_st_acc_ar);


    // for first step of right foot on slope zzzz
    MatrixXd Cz_st_Time_ar(1,3);
    Cz_st_Time_ar<<0, TSS/2,TSS;
    MatrixXd Cz_st_Pos_ar(1,3);
    Cz_st_Pos_ar<<_lenghtOfAnkle,maxHightOfAnkleOnSlope, firstPositionofRightANkleOnSlope(2,0);
    MatrixXd Cz_st_Vel_ar(1,3);
    Cz_st_Vel_ar<<0,INFINITY, 0;
    MatrixXd Cz_st_acc_ar(1,3);
    Cz_st_acc_ar<<0,INFINITY, 0;
    C_st_z_slope_ar=CoefOffline.Coefficient(Cz_st_Time_ar,Cz_st_Pos_ar,Cz_st_Vel_ar,Cz_st_acc_ar);


    // pitch angle of  left  foot for first step of left foot on slope xxxx
    MatrixXd Cpitch_st_iTime(1,2);
    Cpitch_st_iTime<<T_s_st,TStart;
    MatrixXd Cpitch_st_iPos(1,2);
    Cpitch_st_iPos<<0,-1*slopeAngleY;
    MatrixXd Cpitch_st_iVel(1,2);
    Cpitch_st_iVel<<0 , 0;
    MatrixXd Cpitch_st_iAcc(1,2);
    Cpitch_st_iAcc<<0 , 0;
    Cpitch_st_slope_al=CoefOffline.Coefficient(Cpitch_st_iTime,Cpitch_st_iPos,Cpitch_st_iVel,Cpitch_st_iAcc);


    // pitch angle of  right foot for first step of right foot on slope zzzz
    MatrixXd Cpitch_st_Time_ar(1,2);
    Cpitch_st_Time_ar<<0,TSS;
    MatrixXd Cpitch_st_Pos_ar(1,2);
    Cpitch_st_Pos_ar<<0, -1*slopeAngleY;
    MatrixXd Cpitch_st_Vel_ar(1,2);
    Cpitch_st_Vel_ar<<0, 0;
    MatrixXd Cpitch_st_acc_ar(1,2);
    Cpitch_st_acc_ar<<0, 0;
    Cpitch_st_slope_ar=CoefOffline.Coefficient(Cpitch_st_Time_ar,Cpitch_st_Pos_ar,Cpitch_st_Vel_ar,Cpitch_st_acc_ar);


    // for cyclic walk left foot on slope zzz
    MatrixXd Cz_cy_iTime_al(1,3);
    Cz_cy_iTime_al<<0 ,TSS/2, TSS;
    MatrixXd Cz_cy_iPos_al(1,3);
    Cz_cy_iPos_al<<0, AnkleMaximumHeight,changeOfZPositionOfAnkleInOneStep;//peak of ankle can be changed
    MatrixXd Cz_cy_iVel_al(1,3);
    Cz_cy_iVel_al<<0,INFINITY, 0;
    MatrixXd Cz_cy_iAcc_al(1,3);
    Cz_cy_iAcc_al<<0,INFINITY, 0;
    C_cy_z_al=CoefOffline.Coefficient(Cz_cy_iTime_al,Cz_cy_iPos_al,Cz_cy_iVel_al,Cz_cy_iAcc_al);


    // for cyclic walk right foot on slope xxx
    MatrixXd C_cy_iTime_ar(1,2);
    C_cy_iTime_ar<<0, TSS;
    MatrixXd C_cy_iPos_ar(1,2);
    C_cy_iPos_ar<<0, changeOfXPositionOfAnkleInOneStep;
    MatrixXd C_cy_iVel_ar(1,2);
    C_cy_iVel_ar<<0, 0;
    MatrixXd C_cy_iAcc_ar(1,2);
    C_cy_iAcc_ar<<0, 0;
    C_cy_x_ar=CoefOffline.Coefficient(C_cy_iTime_ar,C_cy_iPos_ar,C_cy_iVel_ar,C_cy_iAcc_ar);


    // for cyclic walk right foot on slope zzz
    MatrixXd C_cy_iTime(1,3);
    C_cy_iTime<<0 ,TSS/2, TSS;
    MatrixXd C_cy_iPos(1,3);
    C_cy_iPos<<0,AnkleMaximumHeight,changeOfZPositionOfAnkleInOneStep;//peak of ankle can be changed
    MatrixXd C_cy_iVel(1,3);
    C_cy_iVel<<0 ,INFINITY, 0;
    MatrixXd C_cy_iAcc(1,3);
    C_cy_iAcc<<0 ,INFINITY, 0;
    C_cy_z_ar=CoefOffline.Coefficient(C_cy_iTime,C_cy_iPos,C_cy_iVel,C_cy_iAcc);

    MatrixXd C_end_z_iTime(1,3);
    C_end_z_iTime<<0 ,(T_end_a_e-T_end_a_s)/2 ,T_end_a_e-T_end_a_s;
    MatrixXd C_end_z_iPos(1,3);
    C_end_z_iPos<<_lenghtOfAnkle, AnkleMaximumHeight, _lenghtOfAnkle;
    MatrixXd C_end_z_iVel(1,3);
    C_end_z_iVel<<0 ,INFINITY, 0;
    MatrixXd C_end_z_iAcc(1,3);
    C_end_z_iAcc<<0 ,INFINITY, 0;
    C_end_z_ar=CoefOffline.Coefficient(C_end_z_iTime,C_end_z_iPos,C_end_z_iVel,C_end_z_iAcc);




    //        //end of left ankle on the slope zzzzz
    MatrixXd C_end_z_slope_lTime(1,3);
    C_end_z_slope_lTime<<0 ,TSS/2 ,TSS;
    MatrixXd C_end_z_slope_lPos(1,3);
    C_end_z_slope_lPos<<0, 1.25*changeOfZPositionOfLeftAnkleInLastStep,changeOfZPositionOfLeftAnkleInLastStep;
    MatrixXd C_end_z_slope_lVel(1,3);
    C_end_z_slope_lVel<<0 ,INFINITY, 0;
    MatrixXd C_end_z_slope_lAcc(1,3);
    C_end_z_slope_lAcc<<0 ,INFINITY, 0;
    C_end_z_slope_al=CoefOffline.Coefficient(C_end_z_slope_lTime,C_end_z_slope_lPos,C_end_z_slope_lVel,C_end_z_slope_lAcc);

    //        //end of left ankle on the slope xxxxx

    MatrixXd C_end_slope_iTime_al(1,3);
    C_end_slope_iTime_al<<0,TSS/2, TSS;
    MatrixXd C_end_slope_iPos_al(1,3);
    C_end_slope_iPos_al<<0, 0.5*changeOfXPositionOfLeftAnkleInLastStep,changeOfXPositionOfLeftAnkleInLastStep;
    MatrixXd C_end_slope_iVel_al(1,3);
    C_end_slope_iVel_al<<0, INFINITY,0;
    MatrixXd C_end_slope_iAcc_al(1,3);
    C_end_slope_iAcc_al<<0, INFINITY,0;
    C_end_x_slope_al=CoefOffline.Coefficient(C_end_slope_iTime_al,C_end_slope_iPos_al,C_end_slope_iVel_al,C_end_slope_iAcc_al);


    //pitch angle end of left ankle on the slope zzzzz
    MatrixXd Cpitch_end_slope_lTime(1,3);
    Cpitch_end_slope_lTime<<0 ,TSS/2 ,TSS;
    MatrixXd Cpitch_end_slope_lPos(1,3);
    Cpitch_end_slope_lPos<<-1*slopeAngleY, -0.5*slopeAngleY,0;
    MatrixXd Cpitch_end_slope_lVel(1,3);
    Cpitch_end_slope_lVel<<0 ,INFINITY, 0;
    MatrixXd Cpitch_end_slope_lAcc(1,3);
    Cpitch_end_slope_lAcc<<0 ,INFINITY, 0;
    Cpitch_end_slope_al=CoefOffline.Coefficient(Cpitch_end_slope_lTime,Cpitch_end_slope_lPos,Cpitch_end_slope_lVel,Cpitch_end_slope_lAcc);

    //pitch angle end of right ankle on the slope xxxxx

    MatrixXd Cpitch_end_slope_iTime(1,3);
    Cpitch_end_slope_iTime<<0 ,(T_end_a_e-T_end_a_s)/2 ,T_end_a_e-T_end_a_s;
    MatrixXd Cpitch_end_slope_iPos(1,3);
    Cpitch_end_slope_iPos<<-1*slopeAngleY,-0.5*slopeAngleY,0;
    MatrixXd Cpitch_end_slope_iVel(1,3);
    Cpitch_end_slope_iVel<<0 ,INFINITY, 0;
    MatrixXd Cpitch_end_slope_iAcc(1,3);
    Cpitch_end_slope_iAcc<<0 ,INFINITY, 0;
    Cpitch_end_slope_ar=CoefOffline.Coefficient(Cpitch_end_slope_iTime,Cpitch_end_slope_iPos,Cpitch_end_slope_iVel,Cpitch_end_slope_iAcc);


      //end of right ankle on the slope zzzzz
    MatrixXd C_end_z_slope_iTime(1,3);
    C_end_z_slope_iTime<<0 ,(T_end_a_e-T_end_a_s)/2 ,T_end_a_e-T_end_a_s;
    MatrixXd C_end_z_slope_iPos(1,3);
    C_end_z_slope_iPos<<0, 1.75*changeOfZPositionOfRightAnkleInLastStep,changeOfZPositionOfRightAnkleInLastStep;
    MatrixXd C_end_z_slope_iVel(1,3);
    C_end_z_slope_iVel<<0 ,INFINITY, 0;
    MatrixXd C_end_z_slope_iAcc(1,3);
    C_end_z_slope_iAcc<<0 ,INFINITY, 0;
    C_end_z_slope_ar=CoefOffline.Coefficient(C_end_z_slope_iTime,C_end_z_slope_iPos,C_end_z_slope_iVel,C_end_z_slope_iAcc);

      //end of Right ankle on the slope  xxxx

    MatrixXd C_end_slope_iTime_ar(1,3);
    C_end_slope_iTime_ar<<0,(T_end_a_e-T_end_a_s)/2, (T_end_a_e-T_end_a_s);
    MatrixXd C_end_slope_iPos_ar(1,3);
    C_end_slope_iPos_ar<<0,0.5*changeOfXPositionOfRightAnkleInLastStep ,changeOfXPositionOfRightAnkleInLastStep;
    MatrixXd C_end_slope_iVel_ar(1,3);
    C_end_slope_iVel_ar<<0, INFINITY,0;
    MatrixXd C_end_slope_iAcc_ar(1,3);
    C_end_slope_iAcc_ar<<0, INFINITY,0;
    C_end_x_slope_ar=CoefOffline.Coefficient(C_end_slope_iTime_ar,C_end_slope_iPos_ar,C_end_slope_iVel_ar,C_end_slope_iAcc_ar);


    //       //end of right ankle on the slope

    //                //


    MatrixXd C_end_y_iTime(1,3);
    C_end_y_iTime<<0 ,(T_end_a_e-T_end_a_s)/2 ,T_end_a_e-T_end_a_s;
    MatrixXd C_end_y_iPos(1,3);
    C_end_y_iPos<<-0.5*_pelvisLength,-0.5*_pelvisLength-YOffsetOfAnkletrajectory, -0.5*_pelvisLength;
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

    MatrixXd C_betha_st_Time(1,2);
    C_betha_st_Time<<T_s_st ,TStart;
    MatrixXd C_betha_st_Pos(1,2);
    C_betha_st_Pos<<0, -1*slopeAngleY;
    MatrixXd C_betha_st_vel(1,2);
    C_betha_st_vel<<0,0;
    MatrixXd C_betha_st_accel(1,2);
    C_betha_st_accel<<0,0;
    C_betha_st_al=CoefOffline.Coefficient(C_betha_st_Time,C_betha_st_Pos,C_betha_st_vel,C_betha_st_accel);






}


MatrixXd TaskSpaceOfflineRamp::AnkleTrajectory(double time){



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
    double pitch_al=0;
    double pitch_ar=0;
    if (time<=TStart){//left foot moves in first step
        x_ar=0;
        z_ar=_lenghtOfAnkle;
        x_al=0;
        z_al=_lenghtOfAnkle;
        pitch_al=0;
        pitch_ar=0;
        if (time>=T_s_st){//for first step of left foot
            MatrixXd output=GetAccVelPos(C_st_x_al.row(0),time,T_s_st,5);
            x_al=output(0,0);

            MatrixXd outputpitch=GetAccVelPos(Cpitch_st_slope_al.row(0),time,T_s_st,5);
            pitch_al=outputpitch(0,0);
            pitch_ar=0;

            if (time<=TStart-T_s_st/2){
                MatrixXd output=GetAccVelPos(C_st_z_al.row(0),time,T_s_st,5);
                z_al=output(0,0);
                MatrixXd output1=GetAccVelPos(C_st_y_al.row(0),time,T_s_st,5);
                y_al=output1(0,0);
                LeftSupport=false;
            }

            else{//for last step of left foot
                MatrixXd output=GetAccVelPos(C_st_z_al.row(1),time,TStart-T_s_st/2,5);
                z_al=output(0,0);
                MatrixXd output2=GetAccVelPos(C_st_y_al.row(1),time,TStart-T_s_st/2,5);
                y_al=output2(0,0);
                LeftSupport=false;
            }
        }
    }

    else if (time<=MotionTime-TEnd){
        double tt=time-TStart;
        double N=floor(tt/(2*Tc));
        tt=fmod(tt,(2*Tc));


        if (tt<TDs){//first double support of cyclic walking
            if (N!=NStride) {
                pitch_al=-1*slopeAngleY;
                pitch_ar=-1*(N!=0)*slopeAngleY;

                x_ar=(N!=0)*firstPositionofRightANkleOnSlope(0,0)+(N!=0)*((N-1))*changeOfXPositionOfAnkleInOneStep;
                z_ar=_lenghtOfAnkle*(N==0)+(N!=0)*firstPositionofRightANkleOnSlope(2,0)+(N!=0)*((N-1))*changeOfZPositionOfAnkleInOneStep;

                x_al= firstPositionofLeftANkleOnSlope(0,0)+N*changeOfXPositionOfAnkleInOneStep;
                z_al=1*firstPositionofLeftANkleOnSlope(2,0)+((N))*changeOfZPositionOfAnkleInOneStep;
            }
            else {//double support for last step of left foot
                pitch_al=0;
                pitch_ar=-1*slopeAngleY;

                x_ar=(N!=0)*firstPositionofRightANkleOnSlope(0,0)+(N!=0)*((N-1))*changeOfXPositionOfAnkleInOneStep;
                z_ar=_lenghtOfAnkle*(N==0)+(N!=0)*firstPositionofRightANkleOnSlope(2,0)+(N!=0)*((N-1))*changeOfZPositionOfAnkleInOneStep;

                x_al=finalXPositionOfAnkle;
                z_al=finalHeightOfAnkle;
            }

        }


        else if (tt<Tc){//first single support of cyclic walking

            if (N==0) {
                MatrixXd outputpitch=GetAccVelPos(Cpitch_st_slope_ar.row(0),tt-TDs,0,5);
                pitch_al=-1*slopeAngleY;
                pitch_ar=outputpitch(0,0);

                MatrixXd output1=GetAccVelPos(C_st_x_slope_ar,tt-TDs,0,5);
                x_ar=output1(0,0);
                if (tt<TDs+TSS/2){
                    MatrixXd output2=GetAccVelPos(C_st_z_slope_ar.row(0),tt-TDs,0,5);
                    z_ar=output2(0,0);
                    MatrixXd output3=GetAccVelPos(C_cy_y_ar.row(0),tt-TDs,0,5);
                    y_ar=output3(0,0);

                }
                else{

                    MatrixXd output2=GetAccVelPos(C_st_z_slope_ar.row(1),tt-TDs,TSS/2,5);
                    z_ar=output2(0,0);
                    MatrixXd output3=GetAccVelPos(C_cy_y_ar.row(1),tt-TDs,TSS/2,5);
                    y_ar=output3(0,0);
                }
            }



            else {
                pitch_al=-1*slopeAngleY;
                pitch_ar=-1*slopeAngleY;

                MatrixXd output1=GetAccVelPos(C_cy_x_ar.row(0),tt-TDs,0,5);
                x_ar=firstPositionofRightANkleOnSlope(0,0)+(N-1)*changeOfXPositionOfAnkleInOneStep+output1(0,0);
                if (tt<TDs+TSS/2){
                    MatrixXd output2=GetAccVelPos(C_cy_z_ar.row(0),tt-TDs,0,5);
                    z_ar=firstPositionofRightANkleOnSlope(2,0)+(N-1)*changeOfZPositionOfAnkleInOneStep+output2(0,0);
                    MatrixXd output3=GetAccVelPos(C_cy_y_ar.row(0),tt-TDs,0,5);
                    y_ar=output3(0,0);

                }
                else{

                    MatrixXd output2=GetAccVelPos(C_cy_z_ar.row(1),tt-TDs,TSS/2,5);
                    z_ar=firstPositionofRightANkleOnSlope(2,0)+(N-1)*changeOfZPositionOfAnkleInOneStep+output2(0,0);
                    MatrixXd output3=GetAccVelPos(C_cy_y_ar.row(1),tt-TDs,TSS/2,5);
                    y_ar=output3(0,0);
                }
            }



            x_al=firstPositionofLeftANkleOnSlope(0,0)+N*changeOfXPositionOfAnkleInOneStep;
            z_al=firstPositionofLeftANkleOnSlope(2,0)+N*changeOfZPositionOfAnkleInOneStep;

            LeftSupport=true;


        }


        else if (tt<Tc+TDs){//second double support of cyclic walking

            x_ar=firstPositionofRightANkleOnSlope(0,0)+(N)*changeOfXPositionOfAnkleInOneStep;
            z_ar=firstPositionofRightANkleOnSlope(2,0)+N*changeOfZPositionOfAnkleInOneStep;

            pitch_al=-1*slopeAngleY;
            pitch_ar=-1*slopeAngleY;
            LeftSupport=true;

            x_al=firstPositionofLeftANkleOnSlope(0,0)+N*changeOfXPositionOfAnkleInOneStep;
            z_al=firstPositionofLeftANkleOnSlope(2,0)+(N)*changeOfZPositionOfAnkleInOneStep;


        }



        else{//second single support of cyclic walking
            x_ar=firstPositionofRightANkleOnSlope(0,0)+(N)*changeOfXPositionOfAnkleInOneStep;
            z_ar=firstPositionofRightANkleOnSlope(2,0)+N*changeOfZPositionOfAnkleInOneStep;


            if (N!=(NStride-1)) {
                MatrixXd output2=GetAccVelPos(C_cy_x_al.row(0),tt-(Tc+TDs),0,5);
                double temp=output2(0,0);
                x_al=firstPositionofLeftANkleOnSlope(0,0)+N*changeOfXPositionOfAnkleInOneStep+temp;

                pitch_al=-1*slopeAngleY;

                pitch_ar=-1*slopeAngleY;
                LeftSupport=false;


                if (tt<2*Tc-TSS/2){
                    MatrixXd output3=GetAccVelPos(C_cy_z_al.row(0),tt-(Tc+TDs),0,5);
                    z_al=firstPositionofLeftANkleOnSlope(2,0)+N*changeOfZPositionOfAnkleInOneStep+output3(0,0);

                    MatrixXd output4=GetAccVelPos(C_cy_y_al.row(0),tt-(Tc+TDs),0,5);
                    y_al=output4(0,0);
                }

                else{
                    MatrixXd output2=GetAccVelPos(C_cy_z_al.row(1),tt-(Tc+TDs),TSS/2,5);
                    z_al=firstPositionofLeftANkleOnSlope(2,0)+N*changeOfZPositionOfAnkleInOneStep+output2(0,0);

                    MatrixXd output3=GetAccVelPos(C_cy_y_al.row(1),tt-(Tc+TDs),TSS/2,5);
                    y_al=output3(0,0);

                }
            }
            else {

                LeftSupport=false;


                if (tt<2*Tc-TSS/2){

                    MatrixXd outputpitch=GetAccVelPos(Cpitch_end_slope_al.row(0),tt-(Tc+TDs),0,5);
                    pitch_al=outputpitch(0,0);
                    pitch_ar=-1*slopeAngleY;

                    MatrixXd output222=GetAccVelPos(C_end_x_slope_al.row(0),tt-(Tc+TDs),0,5);
                    double temp=output222(0,0);
                    x_al=firstPositionofLeftANkleOnSlope(0,0)+N*changeOfXPositionOfAnkleInOneStep+temp;

                    MatrixXd output3=GetAccVelPos(C_end_z_slope_al.row(0),tt-(Tc+TDs),0,5);
                    z_al=firstPositionofLeftANkleOnSlope(2,0)+N*changeOfZPositionOfAnkleInOneStep+output3(0,0);

                    MatrixXd output4=GetAccVelPos(C_cy_y_al.row(0),tt-(Tc+TDs),0,5);
                    y_al=output4(0,0);
                }

                else{


                    MatrixXd outputpitch=GetAccVelPos(Cpitch_end_slope_al.row(1),tt-(Tc+TDs),TSS/2,5);
                    pitch_al=outputpitch(0,0);
                    pitch_ar=-1*slopeAngleY;

                    MatrixXd output222=GetAccVelPos(C_end_x_slope_al.row(1),tt-(Tc+TDs),TSS/2,5);
                    double temp=output222(0,0);
                    x_al=firstPositionofLeftANkleOnSlope(0,0)+N*changeOfXPositionOfAnkleInOneStep+temp;

                    MatrixXd output2=GetAccVelPos(C_end_z_slope_al.row(1),tt-(Tc+TDs),TSS/2,5);
                    z_al=firstPositionofLeftANkleOnSlope(2,0)+N*changeOfZPositionOfAnkleInOneStep+output2(0,0);

                    MatrixXd output3=GetAccVelPos(C_cy_y_al.row(1),tt-(Tc+TDs),TSS/2,5);
                    y_al=output3(0,0);

                }
            }

        }
    }

    else{
        double tt=time-(MotionTime-TEnd);

        if (tt<=T_end_a_e-T_end_a_s){



            x_al=finalXPositionOfAnkle;
            z_al=finalHeightOfAnkle;

            if (tt<=(T_end_a_e-T_end_a_s)/2){

                MatrixXd outputpitch=GetAccVelPos(Cpitch_end_slope_ar.row(0),tt,0,5);
                pitch_ar=outputpitch(0,0);
                pitch_al=0;


                MatrixXd output1=GetAccVelPos(C_end_x_slope_ar.row(0),tt,0,5);
                double temp=output1(0,0);
                x_ar=FinalXRightAnkleOnSlope_Abs+temp;

                MatrixXd output2=GetAccVelPos(C_end_z_slope_ar.row(0),tt,0,5);
                z_ar=FinalZRightAnkleOnSlope_Abs+output2(0,0);

                MatrixXd output4=GetAccVelPos(C_end_y_ar.row(0),tt,0,5);
                y_ar=output4(0,0);
            }

            else if (tt>=(T_end_a_e-T_end_a_s)/2){

                MatrixXd outputpitch=GetAccVelPos(Cpitch_end_slope_ar.row(1),tt,(T_end_a_e-T_end_a_s)/2,5);
                pitch_ar=outputpitch(0,0);
                pitch_al=0;

                MatrixXd output1=GetAccVelPos(C_end_x_slope_ar.row(1),tt,(T_end_a_e-T_end_a_s)/2,5);
                double temp=output1(0,0);
                x_ar=FinalXRightAnkleOnSlope_Abs+temp;

                MatrixXd output2=GetAccVelPos(C_end_z_slope_ar.row(1),tt, (T_end_a_e-T_end_a_s)/2,5);
                z_ar=FinalZRightAnkleOnSlope_Abs+output2(0,0);

                MatrixXd output4=GetAccVelPos(C_end_y_ar.row(1),tt, (T_end_a_e-T_end_a_s)/2,5);
                y_ar=output4(0,0);


            }
        }
        else{
            pitch_ar=0;
            pitch_al=0;
            x_ar=finalXPositionOfAnkle;
            z_ar=finalHeightOfAnkle;

            x_al=finalXPositionOfAnkle;
            z_al=finalHeightOfAnkle;
        }
    }


    MatrixXd footpos(8,1);
    footpos<<x_al,y_al,z_al,pitch_al,x_ar,y_ar,z_ar,pitch_ar;
    return footpos;
}



MatrixXd TaskSpaceOfflineRamp::GetAccVelPos(MatrixXd Coef,double time,double ti,int PolynomialOrder)
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
