﻿#include "qcgenerator.h"

QCgenerator::QCgenerator()
{

}

double ankle_pitch(double joint_angle, double offset){
    double motor_angle;

    double theta_0=atan(.025/(0.0822+offset));
    double N=sqrt((.025*.025)+(0.0822+offset)*(0.0822+offset));
    double X_A,Y_A,theta,psi_0;
    double X_B , Y_B;
    double X_M=-.00594;
    double Y_M=.22992;
    double L=0.23;
    double R=0.0875;
    double a,b,x,y,alfa;



    theta=M_PI-theta_0;
    X_A=N*cos(theta);
    Y_A=N*sin(theta);
    a=X_A-X_M; b=Y_A-Y_M;
    alfa= (L*L-R*R-a*a-b*b)/-2;
    y=(b*alfa+sqrt(b*b*alfa*alfa-(a*a+b*b)*(alfa*alfa-a*a*R*R)))/(a*a+b*b);
    x=-sqrt(R*R-y*y);
    X_B=x+X_M;
    Y_B=y+Y_M;
    psi_0=atan2(Y_B-Y_M,-(X_B-X_M));
//qDebug()<<psi_0;

    theta=M_PI-theta_0-joint_angle;
    X_A=N*cos(theta);
    Y_A=N*sin(theta);
    a=X_A-X_M; b=Y_A-Y_M;
    alfa= (L*L-R*R-a*a-b*b)/-2;
    y=(b*alfa+sqrt(b*b*alfa*alfa-(a*a+b*b)*(alfa*alfa-a*a*R*R)))/(a*a+b*b);
    x=-sqrt(R*R-y*y);
    X_B=x+X_M;
    Y_B=y+Y_M;
    motor_angle=atan2(Y_B-Y_M,-(X_B-X_M))-psi_0;

    return motor_angle;
}

double ankle_pitch_reverse(double motor_angle, double offset){
  double joint_angle;

  double theta_0=atan(.025/(0.0822+offset));
  double N=sqrt((.025*.025)+(0.0822+offset)*(0.0822+offset));
  double X_A,Y_A,theta,psi_0;
  double X_B , Y_B;
  double X_M=-.00594;
  double Y_M=.22992;
  double L=0.23;
  double R=0.0875;
  double a,b,x,y,alfa;

  theta=M_PI-theta_0;
  X_A=N*cos(theta);
  Y_A=N*sin(theta);
  a=X_A-X_M; b=Y_A-Y_M;
  alfa= (L*L-R*R-a*a-b*b)/-2;
  y=(b*alfa+sqrt(b*b*alfa*alfa-(a*a+b*b)*(alfa*alfa-a*a*R*R)))/(a*a+b*b);
  x=-sqrt(R*R-y*y);
  X_B=x+X_M;
  Y_B=y+Y_M;
  psi_0=atan2(Y_B-Y_M,X_B-X_M);
//qDebug()<<psi_0;

  Y_B=R*sin(motor_angle+psi_0)+Y_M;
  X_B=R*cos(motor_angle+psi_0)+X_M;
  a=X_B; b=Y_B;
  alfa= (L*L-N*N-a*a-b*b)/-2;
  y=(b*alfa-sqrt(b*b*alfa*alfa-(a*a+b*b)*(alfa*alfa-a*a*N*N)))/(a*a+b*b);
  x=-sqrt(N*N-y*y);
  X_A=x;
  Y_A=y;

  theta=atan2(Y_A,X_A);
  joint_angle=theta-M_PI+theta_0;
  return joint_angle;
}

vector<int> QCgenerator::data2qc(QList<LinkM> links,vector<double> cntrl){



vector<int> qref(12);
    qref[0]=int((links[6].JointAngle+cntrl[6])*(1/(2*M_PI))*(2304)*100);
    //qref[1]=int((links[5].JointAngle+cntrl[5])*(1)*(1/(2*M_PI))*(2304)*100);
    qref[1]=int(ankle_pitch((links[5].JointAngle+cntrl[5]),ankle_mechanism_offsetR)*(1)*(1/(2*M_PI))*(2304)*100);

    qref[2]=int((links[4].JointAngle+cntrl[4])*(1/(2*M_PI))*(2304)*50);
    qref[3]=int((links[3].JointAngle+cntrl[3])*1*(1/(2*M_PI))*(2304)*80);
    //qref[4]=int((links[11].JointAngle+cntrl[11])*(1/(2*M_PI))*(2304)*100);
    qref[4]=int(ankle_pitch((links[11].JointAngle+cntrl[11]),ankle_mechanism_offsetL)*(1)*(1/(2*M_PI))*(2304)*100);

    qref[5]=int((links[12].JointAngle+cntrl[12])*(1/(2*M_PI))*(2304)*100);
    qref[6]=int((links[10].JointAngle+cntrl[10])*1*(1/(2*M_PI))*(2304)*50);
    qref[7]=int((links[9].JointAngle+cntrl[9])*(1/(2*M_PI))*(2304)*80);
    qref[8]=int((links[1].JointAngle+cntrl[1])*(1)*(1/(2*M_PI))*(2304)*120);
    qref[9]=int((links[2].JointAngle+cntrl[2])*(1/(2*M_PI))*(2304)*120);
    qref[10]=int((links[8].JointAngle+cntrl[8])*(1/(2*M_PI))*(2304)*120);
//     qref[9]=int((links[2].JointAngle+cntrl[2])*(1/(2*M_PI))*8192);
//     qref[10]=int((links[2].JointAngle+cntrl[2])*(1/(2*M_PI))*8192);

    qref[11]=int((links[7].JointAngle+cntrl[7])*1*(1/(2*M_PI))*(2304)*120);




    vector<double> minimum(12);
    vector<double> maximum(12);

    vector<int> minimumQC(12);
    vector<int> maximumQC(12);
    minimum={-14.0,-55.0,0.0,-50.0,-55.0,-14.0,0.0,-50.0,-23.0,-15.0,-15.0,-23.0};
    maximum={14.0,26.0,75.0,35.0,26.0,14.0,75.0,35.0,23.0,10.0,10.0,23.0};

    minimumQC[0]=int(minimum[0]*(1/(2*M_PI))*(2304)*100);
    minimumQC[1]=int(minimum[1]*(1)*(1/(2*M_PI))*(2304)*100);
    minimumQC[2]=int(minimum[2]*(1/(2*M_PI))*(2304)*50);
    minimumQC[3]=int(minimum[3]*1*(1/(2*M_PI))*(2304)*80);
    minimumQC[4]=int(minimum[4]*(1/(2*M_PI))*(2304)*100);
    minimumQC[5]=int(minimum[5]*(1/(2*M_PI))*(2304)*100);
    minimumQC[6]=int(minimum[6]*1*(1/(2*M_PI))*(2304)*50);
    minimumQC[7]=int(minimum[7]*(1/(2*M_PI))*(2304)*80);
    minimumQC[8]=int(minimum[8]*(1)*(1/(2*M_PI))*(2304)*120);
    minimumQC[9]=int(minimum[9]*(1/(2*M_PI))*(2304)*120);
    minimumQC[10]=int(minimum[10]*(1/(2*M_PI))*(2304)*120);
//    minimumQC[9]=int(minimum[9]*(1/(2*M_PI))*8192);
//    minimumQC[10]=int(minimum[10]*(1/(2*M_PI))*8192);

    minimumQC[11]=int(minimum[11]*1*(1/(2*M_PI))*(2304)*120);

    maximumQC[0]=int(maximum[0]*(1/(2*M_PI))*(2304)*100);
    maximumQC[1]=int(maximum[1]*(1)*(1/(2*M_PI))*(2304)*100);
    maximumQC[2]=int(maximum[2]*(1/(2*M_PI))*(2304)*50);
    maximumQC[3]=int(maximum[3]*1*(1/(2*M_PI))*(2304)*80);
    maximumQC[4]=int(maximum[4]*(1/(2*M_PI))*(2304)*100);
    maximumQC[5]=int(maximum[5]*(1/(2*M_PI))*(2304)*100);
    maximumQC[6]=int(maximum[6]*1*(1/(2*M_PI))*(2304)*50);
    maximumQC[7]=int(maximum[7]*(1/(2*M_PI))*(2304)*80);
    maximumQC[8]=int(maximum[8]*(1)*(1/(2*M_PI))*(2304)*120);
    maximumQC[9]=int(maximum[9]*(1/(2*M_PI))*(2304)*120);
    maximumQC[10]=int(maximum[10]*(1/(2*M_PI))*(2304)*120);
//    maximumQC[9]=int(maximum[9]*(1/(2*M_PI))*8192);
//    maximumQC[10]=int(maximum[10]*(1/(2*M_PI))*8192);

    maximumQC[11]=int(maximum[11]*1*(1/(2*M_PI))*(2304)*120);

    for (int i = 0; i < 12; ++i) {
        if(qref[i]<minimumQC[i]){qref[i]=minimumQC[i];}
        if(qref[i]>maximumQC[i]){qref[i]=maximumQC[i];}
    }
    qref[1] = -qref[1];
    qref[3] = -qref[3];
    qref[6] = -qref[6];
    qref[8] = -qref[8];
    qref[11] = -qref[11];
    return qref;
}



vector<int> QCgenerator::trajdata2qc(QList<LinkM> links){



vector<int> qref(12);
    qref[0]=int((links[6].JointAngle)*(1/(2*M_PI))*(2304)*100);
    //qref[1]=int((links[5].JointAngle)*(1)*(1/(2*M_PI))*(2304)*100);
    qref[1]=int(ankle_pitch((links[5].JointAngle),ankle_mechanism_offsetR)*(1)*(1/(2*M_PI))*(2304)*100);

    qref[2]=int((links[4].JointAngle)*(1/(2*M_PI))*(2304)*50);
    qref[3]=int((links[3].JointAngle)*1*(1/(2*M_PI))*(2304)*80);
//    qref[4]=int((links[11].JointAngle)*(1/(2*M_PI))*(2304)*100);
    qref[4]=int(ankle_pitch((links[11].JointAngle),ankle_mechanism_offsetL)*(1)*(1/(2*M_PI))*(2304)*100);

    qref[5]=int((links[12].JointAngle)*(1/(2*M_PI))*(2304)*100);
    qref[6]=int((links[10].JointAngle)*1*(1/(2*M_PI))*(2304)*50);
    qref[7]=int((links[9].JointAngle)*(1/(2*M_PI))*(2304)*80);
    qref[8]=int((links[1].JointAngle)*(1)*(1/(2*M_PI))*(2304)*120);
    qref[9]=int((links[2].JointAngle)*(1/(2*M_PI))*(2304)*120);
    qref[10]=int((links[8].JointAngle)*(1/(2*M_PI))*(2304)*120);
//    qref[9]=int((links[2].JointAngle)*(1/(2*M_PI))*8192);
//    qref[10]=int((links[8].JointAngle)*(1/(2*M_PI))*8192);
    qref[11]=int((links[7].JointAngle)*1*(1/(2*M_PI))*(2304)*120);




    vector<double> minimum(12);
    vector<double> maximum(12);

    vector<int> minimumQC(12);
    vector<int> maximumQC(12);
    minimum={-14.0,-55.0,0.0,-50.0,-55.0,-14.0,0.0,-50.0,-23.0,-15.0,-15.0,-23.0};
    maximum={14.0,26.0,75.0,35.0,26.0,14.0,75.0,35.0,23.0,10.0,10.0,23.0};

    minimumQC[0]=int(minimum[0]*(1/(2*M_PI))*(2304)*100);
    minimumQC[1]=int(minimum[1]*(1)*(1/(2*M_PI))*(2304)*100);
    minimumQC[2]=int(minimum[2]*(1/(2*M_PI))*(2304)*50);
    minimumQC[3]=int(minimum[3]*1*(1/(2*M_PI))*(2304)*80);
    minimumQC[4]=int(minimum[4]*(1/(2*M_PI))*(2304)*100);
    minimumQC[5]=int(minimum[5]*(1/(2*M_PI))*(2304)*100);
    minimumQC[6]=int(minimum[6]*1*(1/(2*M_PI))*(2304)*50);
    minimumQC[7]=int(minimum[7]*(1/(2*M_PI))*(2304)*80);
    minimumQC[8]=int(minimum[8]*(1)*(1/(2*M_PI))*(2304)*120);
    minimumQC[9]=int(minimum[9]*(1/(2*M_PI))*(2304)*120);
    minimumQC[10]=int(minimum[10]*(1/(2*M_PI))*(2304)*120);
//    minimumQC[9]=int(minimum[9]*(1/(2*M_PI))*8192);
//    minimumQC[10]=int(minimum[10]*(1/(2*M_PI))*8192);

    minimumQC[11]=int(minimum[11]*1*(1/(2*M_PI))*(2304)*120);

    maximumQC[0]=int(maximum[0]*(1/(2*M_PI))*(2304)*100);
    maximumQC[1]=int(maximum[1]*(1)*(1/(2*M_PI))*(2304)*100);
    maximumQC[2]=int(maximum[2]*(1/(2*M_PI))*(2304)*50);
    maximumQC[3]=int(maximum[3]*1*(1/(2*M_PI))*(2304)*80);
    maximumQC[4]=int(maximum[4]*(1/(2*M_PI))*(2304)*100);
    maximumQC[5]=int(maximum[5]*(1/(2*M_PI))*(2304)*100);
    maximumQC[6]=int(maximum[6]*1*(1/(2*M_PI))*(2304)*50);
    maximumQC[7]=int(maximum[7]*(1/(2*M_PI))*(2304)*80);
    maximumQC[8]=int(maximum[8]*(1)*(1/(2*M_PI))*(2304)*120);
    maximumQC[9]=int(maximum[9]*(1/(2*M_PI))*(2304)*120);
    maximumQC[10]=int(maximum[10]*(1/(2*M_PI))*(2304)*120);
//    maximumQC[9]=int(maximum[9]*(1/(2*M_PI))*8192);
//    maximumQC[10]=int(maximum[10]*(1/(2*M_PI))*8192);

    maximumQC[11]=int(maximum[11]*1*(1/(2*M_PI))*(2304)*120);

    for (int i = 0; i < 12; ++i) {
        if(qref[i]<minimumQC[i]){qref[i]=minimumQC[i];}
        if(qref[i]>maximumQC[i]){qref[i]=maximumQC[i];}
    }
    qref[1] = -qref[1];
    qref[3] = -qref[3];
    qref[6] = -qref[6];
    qref[8] = -qref[8];
    qref[11] = -qref[11];
    return qref;
}





vector<int> QCgenerator::ctrldata2qc(vector<double> cntrl){

vector<int> qref(12);
    qref[0]=int((cntrl[6])*(1/(2*M_PI))*(2304)*100);
    //qref[1]=int((cntrl[5])*(1)*(1/(2*M_PI))*(2304)*100);
    qref[1]=int(ankle_pitch(cntrl[5],ankle_mechanism_offsetR)*(1)*(1/(2*M_PI))*(2304)*100);
    qref[2]=int((cntrl[4])*(1/(2*M_PI))*(2304)*50);
    qref[3]=int((cntrl[3])*1*(1/(2*M_PI))*(2304)*80);
    //qref[4]=int((cntrl[11])*(1/(2*M_PI))*(2304)*100);
    qref[4]=int(ankle_pitch(cntrl[11],ankle_mechanism_offsetL)*(1)*(1/(2*M_PI))*(2304)*100);
   // qDebug()<<"R: before:"<<qref[1]<<"now:"<<qref1_test<<"L: before:"<<qref[4]<<"now:"<<qref4_test;
    qref[5]=int((cntrl[12])*(1/(2*M_PI))*(2304)*100);
    qref[6]=int((cntrl[10])*1*(1/(2*M_PI))*(2304)*50);
    qref[7]=int((cntrl[9])*(1/(2*M_PI))*(2304)*80);
    qref[8]=int((cntrl[1])*(1)*(1/(2*M_PI))*(2304)*120);
    qref[9]=int((cntrl[2])*(1/(2*M_PI))*(2304)*120);
    qref[10]=int((cntrl[8])*(1/(2*M_PI))*(2304)*120);
//    qref[9]=int((cntrl[2])*(1/(2*M_PI))*8192);
//    qref[10]=int((cntrl[8])*(1/(2*M_PI))*8192);

    qref[11]=int((cntrl[7])*1*(1/(2*M_PI))*(2304)*120);


//qDebug()<<ankle_pitch_reverse(ankle_pitch(cntrl[5],ankle_mechanism_offsetR),ankle_mechanism_offsetR)-cntrl[5];
//qDebug()<<ankle_pitch_reverse(ankle_pitch(cntrl[11],ankle_mechanism_offsetL),ankle_mechanism_offsetL)-cntrl[11];
//    vector<double> minimum(12);
//    vector<double> maximum(12);

//    vector<int> minimumQC(12);
//    vector<int> maximumQC(12);
//    minimum={-14.0,-55.0,0.0,-50.0,-55.0,-14.0,0.0,-50.0,-23.0,-15.0,-15.0,-23.0};
//    maximum={14.0,26.0,75.0,35.0,26.0,14.0,75.0,35.0,23.0,10.0,10.0,23.0};

//    minimumQC[0]=int(minimum[0]*(1/(2*M_PI))*(2304)*100);
//    minimumQC[1]=int(minimum[1]*(1)*(1/(2*M_PI))*(2304)*100);
//    minimumQC[2]=int(minimum[2]*(1/(2*M_PI))*(2304)*50);
//    minimumQC[3]=int(minimum[3]*1*(1/(2*M_PI))*(2304)*80);
//    minimumQC[4]=int(minimum[4]*(1/(2*M_PI))*(2304)*100);
//    minimumQC[5]=int(minimum[5]*(1/(2*M_PI))*(2304)*100);
//    minimumQC[6]=int(minimum[6]*1*(1/(2*M_PI))*(2304)*50);
//    minimumQC[7]=int(minimum[7]*(1/(2*M_PI))*(2304)*80);
//    minimumQC[8]=int(minimum[8]*(1)*(1/(2*M_PI))*(2304)*120);
//    minimumQC[9]=int(minimum[9]*(1/(2*M_PI))*(2304)*120);
//    minimumQC[10]=int(minimum[10]*(1/(2*M_PI))*(2304)*120);
////    minimumQC[9]=int(minimum[9]*(1/(2*M_PI))*8192);
////    minimumQC[10]=int(minimum[10]*(1/(2*M_PI))*8192);
//    minimumQC[11]=int(minimum[11]*1*(1/(2*M_PI))*(2304)*120);

//    maximumQC[0]=int(maximum[0]*(1/(2*M_PI))*(2304)*100);
//    maximumQC[1]=int(maximum[1]*(1)*(1/(2*M_PI))*(2304)*100);
//    maximumQC[2]=int(maximum[2]*(1/(2*M_PI))*(2304)*50);
//    maximumQC[3]=int(maximum[3]*1*(1/(2*M_PI))*(2304)*80);
//    maximumQC[4]=int(maximum[4]*(1/(2*M_PI))*(2304)*100);
//    maximumQC[5]=int(maximum[5]*(1/(2*M_PI))*(2304)*100);
//    maximumQC[6]=int(maximum[6]*1*(1/(2*M_PI))*(2304)*50);
//    maximumQC[7]=int(maximum[7]*(1/(2*M_PI))*(2304)*80);
//    maximumQC[8]=int(maximum[8]*(1)*(1/(2*M_PI))*(2304)*120);
//    maximumQC[9]=int(maximum[9]*(1/(2*M_PI))*(2304)*120);
//    maximumQC[10]=int(maximum[10]*(1/(2*M_PI))*(2304)*120);
////    maximumQC[9]=int(maximum[9]*(1/(2*M_PI))*8192);
////    maximumQC[10]=int(maximum[10]*(1/(2*M_PI))*8192);
//    maximumQC[11]=int(maximum[11]*1*(1/(2*M_PI))*(2304)*120);


//    for (int i = 0; i < 12; ++i) {
//        if(qref[i]<minimumQC[i]){qref[i]=minimumQC[i];
//        }
//        if(qref[i]>maximumQC[i]){qref[i]=maximumQC[i];}
//    }
    qref[1] = -qref[1];
    qref[3] = -qref[3];
    qref[6] = -qref[6];
    qref[8] = -qref[8];
    qref[11] = -qref[11];

//qDebug()<<qref[9]<<"\t"<<minimumQC[9]<<"\t"<<maximumQC[9];
    return qref;
}




