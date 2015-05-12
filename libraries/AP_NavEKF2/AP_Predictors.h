#ifndef AP_PREDICTORS_H
#define AP_PREDICTORS_H

#include <AP_Math.h>
#include <AP_Param.h>
#include <vectorN.h>
//#include <matrix3.h>
//#include <quaternion.h>

#define BUFFER_SIZE  200   // sean buffer size for sensors
#define MAX_MSDELAY  2000   // maximum allowed delay

class AP_Predictors
{
public:
    AP_Predictors();
    typedef float ftype;
    void BestIndex(AP_Int16 _msecPosDelay);
    void AttitudeModel(Vector3f tilde_q);
    void VelocityModel(Vector3f tilde_Vel);
    void VelocityModel2(Vector3f corrected_tilde_Vel12);
    void PositionModel(ftype dtIMU);
    void PositionModel2(ftype dtIMU);
    void AttitudePredictor(Quaternion quat);
    void PositionPredictor(Vector3f position);
    void PositionPredictor2(Vector3f position);
    //void PositionPredictor3(Vector3f p_pred, Vector3f p, VectorN<Vector3f,BUFFER_SIZE> stored_p, ftype dtIMU, Vector3f position);
    void VelocityPredictor(Vector3f velocity);
    void VelocityPredictor2(Quaternion quat, Vector3f velocity, AP_Int16 _msecPosDelay);
    void CascadedPredictor(Vector3f tilde_q, Vector3f tilde_Vel, Vector3f corrected_tilde_Vel12, Quaternion quat, ftype dtIMU, AP_Int16 _msecPosDelay, Vector3f velocity, Vector3f position);
    void BestIndex2(uint32_t *closestTime, uint16_t *closestStoreIndex, uint32_t (*timeStamp)[BUFFER_SIZE], AP_Int16 _msecDelay);
    Quaternion q_hat;   //sean prediction of the current quaternion
    Vector3f v_hat; // prediction of current velocity
    Vector3f p_hat; // prediction of current position

private:
    uint32_t imuSampleTime_ms;
    Matrix3f D;
    Matrix3f D_T;
    Quaternion D_q;
    Quaternion D_q_k1;
    float n_D_q_k1;

    Quaternion q_hat_T_k1;
    // Vector3f tilde_q;
    float n_tilde_q;
    Quaternion delta_q;
    Matrix3f R_hat_T;
    Matrix3f R_hat;
    Vector3f d_v;

    Vector3f d_p;
    Vector3f v_hat_m; // prediction of current velocity mixed-invariant
    Vector3f d_p_m;
    Vector3f p_hat_m; // prediction of current position mixed-invariant
    Vector3f d_v_m;  // mixed-invariant
    Quaternion D_Delay;
    uint16_t bestStoreIndex;

    uint16_t storeIndexIMU;						// arash
    uint32_t lastAngRateStoreTime_ms;		 // sean
    VectorN<Vector3f,BUFFER_SIZE> storedAngRate;       //  sean
    VectorN<Vector3f,BUFFER_SIZE> storeddVelIMU1; //sean
    VectorN<Vector3f,BUFFER_SIZE>  storeddVelIMU2; //sean
    uint32_t angRateTimeStamp[BUFFER_SIZE];    		  // sean

    uint16_t storeIndexMag;
    uint32_t lastMagStoreTime_ms;
    VectorN<Vector3f,BUFFER_SIZE> storedMag;       //  sean
    uint32_t MagTimeStamp[BUFFER_SIZE];    		  // sean


    uint16_t storeIndexTas;              // sean
    uint32_t lastTasStoreTime_ms;           // sean
    float storedTas[BUFFER_SIZE];                 // sean
    uint32_t TasTimeStamp[BUFFER_SIZE];    		  // sean

    uint16_t storeIndexHgt;              // sean
    uint32_t lastHgtStoreTime_ms;           // sean
    float storedHgt[BUFFER_SIZE];                 // sean
    uint32_t HgtTimeStamp[BUFFER_SIZE];    		  // sean
    uint32_t lastHealthyHgtTime_ms; // Sean time the barometer was last declared healthy

    uint16_t storeIndexD;						// sean
    uint32_t lastDStoreTime_ms;		 // sean
    VectorN<Vector3f,BUFFER_SIZE> storedD_v;       //  sean vector part of quaternion Delta
    float storedD_s[BUFFER_SIZE];                 // sean  scalar part of quaternion Delta
    uint32_t DTimeStamp[BUFFER_SIZE];    		  // sean


    uint16_t storeIndexd_v;						// sean
    uint32_t lastd_vStoreTime_ms;
    uint32_t d_vTimeStamp[BUFFER_SIZE];
    VectorN<Vector3f,BUFFER_SIZE> storedd_v;       //  sean buffer for delta corrsponding to velocity prediction
    VectorN<Vector3f,BUFFER_SIZE> storedd_p;       //  sean buffer for delta corrsponding to position prediction
    VectorN<Vector3f,BUFFER_SIZE> storedd_p_m;       //  sean buffer for delta corrsponding to position prediction mixed-invariant
    VectorN<Vector3f,BUFFER_SIZE> storedd_v_m;       //  sean buffer for delta corrsponding to velocity prediction mixed-invariant
    uint32_t ctr_rst;  // reset predictor cntr

    float init_reset;   //sean reset initial quaternions

    float rotScaler;
    Vector3f D_q_tmp;
    Quaternion q_tmp;

//    Vector3f tilde_Vel;
//    Vector3f corrected_tilde_Vel1;
//    Vector3f corrected_tilde_Vel2;
//    Vector3f corrected_tilde_Vel12;

    Matrix3f prevTnb_pred;
    Matrix3f Tbn_temp;

    Vector3f d_p_Delay;
    Vector3f d_v_Delay;

    uint32_t bestTime;

};

#endif // AP_PREDICTORS_H
