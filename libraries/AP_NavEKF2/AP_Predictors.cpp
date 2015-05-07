
#include "AP_Predictors.h"
#include <AP_HAL.h>

#include <stdio.h>

//sean
#include <iostream>
using namespace std;
#include <stdlib.h>
//end sean

extern const AP_HAL::HAL& hal;
const Vector3f gravityNED(0, 0, GRAVITY_MSS);


AP_Predictors::AP_Predictors()
{
    //ctor
}

void AP_Predictors::AttitudeModel(Vector3f tilde_q)
{
    imuSampleTime_ms = hal.scheduler->millis();

    D_q = D_q_k1;

    //tilde_q = dAngIMU - gyro_bias;

    n_tilde_q = sqrt(tilde_q[0]*tilde_q[0]+tilde_q[1]*tilde_q[1]+tilde_q[2]*tilde_q[2]);
    if (n_tilde_q < 1e-12f)
    {
        delta_q[0] = 1;
        delta_q[1] = 0;
        delta_q[2] = 0;
        delta_q[3] = 0;
    }
    else
    {
// check: delta_q=single([cos(0.5*n_tilde_q);(sin(0.5*n_tilde_q)/n_tilde_q).*tilde_q]);
        delta_q[0] = cosf(0.5f * n_tilde_q);
        rotScaler = (sinf(0.5f * n_tilde_q)) / n_tilde_q;
        delta_q[1] = tilde_q.x * rotScaler;
        delta_q[2] = tilde_q.y * rotScaler;
        delta_q[3] = tilde_q.z * rotScaler;
    }

// update the quaternions by rotating from the previous attitude through
// the delta angle rotation quaternion
    D_q_k1[0] = D_q[0]*delta_q[0] - D_q[1]*delta_q[1] - D_q[2]*delta_q[2] - D_q[3]*delta_q[3];
    D_q_k1[1] = D_q[0]*delta_q[1] + D_q[1]*delta_q[0] + D_q[2]*delta_q[3] - D_q[3]*delta_q[2];
    D_q_k1[2] = D_q[0]*delta_q[2] + D_q[2]*delta_q[0] + D_q[3]*delta_q[1] - D_q[1]*delta_q[3];
    D_q_k1[3] = D_q[0]*delta_q[3] + D_q[3]*delta_q[0] + D_q[1]*delta_q[2] - D_q[2]*delta_q[1];

    n_D_q_k1 = D_q_k1[0]*D_q_k1[0]+D_q_k1[1]*D_q_k1[1]+D_q_k1[2]*D_q_k1[2]+D_q_k1[3]*D_q_k1[3];
    D_q_k1[0] = D_q_k1[0]/n_D_q_k1;
    D_q_k1[1] = D_q_k1[1]/n_D_q_k1;
    D_q_k1[2] = D_q_k1[2]/n_D_q_k1;
    D_q_k1[3] = D_q_k1[3]/n_D_q_k1;

    if (imuSampleTime_ms - lastDStoreTime_ms >= 10) {
        lastDStoreTime_ms = imuSampleTime_ms;
        if (storeIndexD > (BUFFER_SIZE-1)) {
            storeIndexD = 0;
        }
        storedD_s[storeIndexD]=D_q_k1[0];
        D_q_tmp[0]=D_q_k1[1];
        D_q_tmp[1]=D_q_k1[2];
        D_q_tmp[2]=D_q_k1[3];
        storedD_v[storeIndexD] = D_q_tmp;
        DTimeStamp[storeIndexD] = lastDStoreTime_ms;
        storeIndexD = storeIndexD + 1;
    }

}

void AP_Predictors::BestIndex(AP_Int16 _msecPosDelay)
{
    uint32_t timeD;
    uint32_t bestTimeD = 200;
    bestStoreIndex = 0;

    for (uint16_t i=0; i<=(BUFFER_SIZE-1); i++)
    {
        timeD = abs( (imuSampleTime_ms - DTimeStamp[i]) - constrain_int16(_msecPosDelay, 0, MAX_MSDELAY));
        //       printf("%u \n",_msecPosDelay);
        if (timeD < bestTimeD)
        {
            bestStoreIndex = i;
            bestTimeD = timeD;
        }
    }
}


void AP_Predictors::AttitudePredictor(Quaternion quat)
{
    D_Delay[0] = storedD_s[bestStoreIndex];
    D_q_tmp=storedD_v[bestStoreIndex];
    D_Delay[1] = D_q_tmp[0];
    D_Delay[2] = D_q_tmp[1];
    D_Delay[3] = D_q_tmp[2];

// D_q_delay^{-1}
    q_tmp[0]= D_Delay[0];
    q_tmp[1]= -D_Delay[1];
    q_tmp[2]= -D_Delay[2];
    q_tmp[3]= -D_Delay[3];

// D_q_delay^{-1} \times D_q
    q_hat[0] = q_tmp[0]*D_q[0] - q_tmp[1]*D_q[1] - q_tmp[2]*D_q[2] - q_tmp[3]*D_q[3];
    q_hat[1] = q_tmp[0]*D_q[1] + q_tmp[1]*D_q[0] + q_tmp[2]*D_q[3] - q_tmp[3]*D_q[2];
    q_hat[2] = q_tmp[0]*D_q[2] + q_tmp[2]*D_q[0] + q_tmp[3]*D_q[1] - q_tmp[1]*D_q[3];
    q_hat[3] = q_tmp[0]*D_q[3] + q_tmp[3]*D_q[0] + q_tmp[1]*D_q[2] - q_tmp[2]*D_q[1];

// q_hat_tau \times D_q_delay^{-1} \times D_q
    q_tmp[0] = quat[0]*q_hat[0] - quat[1]*q_hat[1] - quat[2]*q_hat[2] - quat[3]*q_hat[3];
    q_tmp[1] = quat[0]*q_hat[1] + quat[1]*q_hat[0] + quat[2]*q_hat[3] - quat[3]*q_hat[2];
    q_tmp[2] = quat[0]*q_hat[2] + quat[2]*q_hat[0] + quat[3]*q_hat[1] - quat[1]*q_hat[3];
    q_tmp[3] = quat[0]*q_hat[3] + quat[3]*q_hat[0] + quat[1]*q_hat[2] - quat[2]*q_hat[1];

    q_tmp.normalize();
    q_hat=q_tmp;
}

void AP_Predictors::VelocityModel(Vector3f tilde_Vel)
{
// velocity prediction
//    Matrix3f Tbn_temp;
//
//    q_h.rotation_matrix(Tbn_temp);
//    prevTnb_pred = Tbn_temp.transposed();

    d_v+=tilde_Vel ;

// buffering d_v
    storeIndexD = storeIndexD - 1;
    storedd_v[storeIndexD]=d_v;
    storeIndexD = storeIndexD + 1;
}

void AP_Predictors::VelocityPredictor(Vector3f velocity)
{
// picking up the delayed d_v
    d_v_Delay=storedd_v[bestStoreIndex];

// v_hat
    v_hat = velocity + d_v- d_v_Delay;
}

//void AP_Predictors::PositionModel(Vector3f d_p, AP_Int16 _msecPosDelay, Vector3f position)
//{
//// position prediction
//    d_p+= v_hat*dtIMU;
//// buffering d_p
//    storeIndexD = storeIndexD - 1;
//    storedd_p[storeIndexD]=d_p;
//    storeIndexD = storeIndexD + 1;
//}

void AP_Predictors::PositionModel(ftype dtIMU)
{
// position prediction
    d_p+= v_hat*dtIMU;
// buffering d_p
    storeIndexD = storeIndexD - 1;
    storedd_p[storeIndexD]=d_p;
    storeIndexD = storeIndexD + 1;
}

void AP_Predictors::PositionPredictor(Vector3f position)
{
// picking up the delayed d_p
    d_p_Delay=storedd_p[bestStoreIndex];
// p_hat
    p_hat = position + d_p- d_p_Delay;
}

void AP_Predictors::PositionModel2(ftype dtIMU)
{
// position prediction
    d_p_m+= v_hat_m*dtIMU;
// buffering d_p
    storeIndexD = storeIndexD - 1;
    storedd_p_m[storeIndexD]=d_p_m;
    storeIndexD = storeIndexD + 1;
}

void AP_Predictors::PositionPredictor2(Vector3f position)
{
// picking up the delayed d_p
    d_p_Delay=storedd_p_m[bestStoreIndex];
// p_hat
    p_hat_m = position + d_p_m- d_p_Delay;
}

//void AP_Predictors::PositionPredictor3(Vector3f p_pred, Vector3f p, VectorN<Vector3f,BUFFER_SIZE> stored_p, ftype dtIMU, Vector3f position)
//{
//// position prediction
//    p += v_hat_m*dtIMU;
//// buffering d_p
//    storeIndexD = storeIndexD - 1;
//    stored_p[storeIndexD]=p;
//    storeIndexD = storeIndexD + 1;
//// picking up the delayed d_p
//    Vector3f p_Delay;
//    p_Delay=stored_p[bestStoreIndex];
//// p_hat
//    p_pred = position + p - p_Delay;
//}

void AP_Predictors::VelocityModel2(Vector3f corrected_tilde_Vel12)
{
// Mixed-invariant
    D_q_k1.rotation_matrix(Tbn_temp);

    d_v_m= d_v_m+Tbn_temp*corrected_tilde_Vel12 ;

// buffering d_v_m
    storeIndexD = storeIndexD - 1;
    storedd_v_m[storeIndexD]=d_v_m;
    storeIndexD = storeIndexD + 1;

    if (ctr_rst==100) {   // reset every ctr_rst*20 (ms)
        ctr_rst=0;
        for (uint16_t i=0; i<=(BUFFER_SIZE-1); i++)
        {
            storedd_v_m[i]=storedd_v_m[i]-d_v_m;
        }
        d_v_m[0]=0;
        d_v_m[1]=0;
        d_v_m[2]=0;
    }
}

void AP_Predictors::VelocityPredictor2(Quaternion quat, Vector3f velocity, AP_Int16 _msecPosDelay)
{
// picking up the delayed d_v
    Vector3f d_v_m_Delay=storedd_v_m[bestStoreIndex];

// D_q_delay^{-1}
    q_tmp[0]= D_Delay[0];
    q_tmp[1]= -D_Delay[1];
    q_tmp[2]= -D_Delay[2];
    q_tmp[3]= -D_Delay[3];

    Quaternion q_tmp_m;

// q_hat_tau \times D_q_delay^{-1}
    q_tmp_m[0] = quat[0]*q_tmp[0] - quat[1]*q_tmp[1] - quat[2]*q_tmp[2] - quat[3]*q_tmp[3];
    q_tmp_m[1] = quat[0]*q_tmp[1] + quat[1]*q_tmp[0] + quat[2]*q_tmp[3] - quat[3]*q_tmp[2];
    q_tmp_m[2] = quat[0]*q_tmp[2] + quat[2]*q_tmp[0] + quat[3]*q_tmp[1] - quat[1]*q_tmp[3];
    q_tmp_m[3] = quat[0]*q_tmp[3] + quat[3]*q_tmp[0] + quat[1]*q_tmp[2] - quat[2]*q_tmp[1];

    q_tmp_m.rotation_matrix(Tbn_temp);

// v_hat_m mixed-invariant
    v_hat_m = velocity + gravityNED*(0.001f*constrain_int16(_msecPosDelay, 0, MAX_MSDELAY))+Tbn_temp*(d_v_m- d_v_m_Delay);
}

void AP_Predictors::CascadedPredictor(Vector3f tilde_q, Vector3f tilde_Vel, Vector3f corrected_tilde_Vel12, Quaternion quat, ftype dtIMU, AP_Int16 _msecPosDelay, Vector3f velocity, Vector3f position)
{
    AttitudeModel(tilde_q);
    VelocityModel(tilde_Vel);
    VelocityModel2(corrected_tilde_Vel12);
    PositionModel(dtIMU);
    PositionModel2(dtIMU);
    BestIndex(_msecPosDelay);
    AttitudePredictor(quat);
    VelocityPredictor(velocity);
    PositionPredictor(position);
    VelocityPredictor2(quat, velocity, _msecPosDelay);
    PositionPredictor2(p_hat);
    static FILE *mylog;
    if (mylog==NULL) {
        mylog = fopen("logtmp.txt", "w");
    }
    if (mylog!=NULL) {
        // fputs ("Ali",  mylog);
        fprintf (mylog, "{%lu;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f}\n",
                 (unsigned long)hal.scheduler->millis(),
                 q_hat[0], q_hat[1],q_hat[2],q_hat[3],v_hat[0],v_hat[1],v_hat[2],d_v[0],d_v[1],d_v[2],d_v_Delay[0],d_v_Delay[1],d_v_Delay[2],p_hat[0],p_hat[1],p_hat[2],v_hat_m[0],v_hat_m[1],v_hat_m[2],p_hat_m[0],p_hat_m[1],p_hat_m[2]);
    }

}
