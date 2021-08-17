#include <chrono>
#include <math.h>
#define Nsta 6 //the size of states
#define Mobs 6 //the size of measurements

#include "TinyEKF.h"

const double initial_theta = 0.0;
const double initial_w_wheel = 0.0;
const double initial_v_wheel = 0.0;

const double sigma_1_w = 0.01f; // 調整する変数
const double sigma_1_v = 0.01f; // 調整する変数
const double sigma_beta = 0.01f;// 調整する変数
const double sigma_2_w = sigma_1_w * sigma_1_w /2;
const double sigma_3_w = sigma_1_w * sigma_1_w * sigma_1_w /3;
const double sigma_2_v = sigma_1_v * sigma_1_v /2;
const double sigma_3_v = sigma_1_v * sigma_1_v * sigma_1_v /3;

double sigma_w_w;
double sigma_w_v;
const double delta_w_w = 0.01f;     // 調整する変数
const double delta_w_v = 0.01f;     // 調整する変数
const double sigma_g_w = 0.01f;     // 調整する変数
const double sigma_d_theta = 0.01f; // 調整する変数
const double sigma_d_x = 0.01f;     // 調整する変数
const double sigma_d_y = 0.01f;     // 調整する変数

class FusionOdometry : public TinyEKF{
    public:
        double dt;
        FusionOdometry(std::chrono::duration<double> period_seconds):
        dt(period_seconds.count())
        {
            //process noise
            this->setQ(0,0,sigma_3_w); //theta
            this->setQ(0,2,sigma_2_w);
            this->setQ(1,1,sigma_beta); //beta
            this->setQ(2,0,sigma_2_w); //omega_zk
            this->setQ(2,2,sigma_1_w);
            this->setQ(3,3,sigma_3_v); //xpk
            this->setQ(3,5,cos(initial_theta) * sigma_2_v);
            this->setQ(4,4,sigma_3_v); //ypk
            this->setQ(4,5,sin(initial_theta) * sigma_2_v);
            this->setQ(5,5,sigma_1_v); //vxk
            this->setQ(5,3,cos(initial_theta) * sigma_2_v);
            this->setQ(5,4,sin(initial_theta) * sigma_2_v);

            //the noise of measurements
            this->setR(0,0,sigma_w_w); //w_w Dynamic
            this->setR(1,1,sigma_g_w); //w_g 
            this->setR(2,2,sigma_w_v); //v_w Dynamic
            this->setR(3,3,sigma_d_theta); //theta_d
            this->setR(4,4,sigma_d_x); //xd
            this->setR(5,5,sigma_d_y); //yd
        }

        void updateQ(){
            this->setQ(3,5,cos(this->x[0]) * sigma_2_v);
            this->setQ(4,4,sigma_3_v); //ypk
            this->setQ(4,5,sin(this->x[0]) * sigma_2_v);
            this->setQ(5,5,sigma_1_v); //vxk
            this->setQ(5,3,cos(this->x[0]) * sigma_2_v);
            this->setQ(5,4,sin(this->x[0]) * sigma_2_v);
            this->setR(0,0,sigma_w_w); //w_w Dynamic
            this->setR(1,1,sigma_g_w); //w_g 
            this->setR(2,2,sigma_w_v); //v_w Dynamic
            /*
            //when using VOD
            this->setR(3,3,sigma_d_theta); //theta_d
            this->setR(4,4,sigma_d_x); //xd
            this->setR(5,5,sigma_d_y); //yd
            */
        }
        void updateR(double* z){
            sigma_w_w = delta_w_w * this->dt * z[0] + delta_w_w * this->dt * z[2];
            sigma_w_v = delta_w_v * this->dt * z[0] + delta_w_v * this->dt * z[2];
            this->setR(0,0,sigma_w_w); //w_w Dynamic
            this->setR(2,2,sigma_w_v); //v_w Dynamic
        }
        
        bool step_with_updateQR(double * z) 
        { 
            updateQ();
            updateR(z);
            return this->step(z);
        }
    protected:
        void model(double fx[Nsta], double F[Nsta][Nsta], double hx[Mobs], double H[Mobs][Nsta]){
            //process model
            //refer to p1295 column L
            fx[0] = this->x[0] + this->x[2] * this->dt;
            fx[1] = this->x[1];
            fx[2] = this->x[2];
            fx[3] = this->x[3] + cos(this->x[0]) * this->dt * this->x[5];
            fx[4] = this->x[4] + sin(this->x[0]) * this->dt * this->x[5];
            fx[5] = this->x[5];

            //Matrix F
            F[0][0] = 1;
            F[0][2] = this->dt;
            F[1][1] = 1;
            F[2][2] = 1;
            F[3][3] = 1;
            F[3][5] = cos(this->x[0]) * this->dt;
            F[4][4] = 1;
            F[4][5] = sin(this->x[0]) * this->dt;
            F[5][5] = 1;

            //measurements function
            hx[0] = this->x[2];
            hx[1] = this->x[1] + this->x[2];
            hx[2] = this->x[5];
            /*
            //when using VOD
            hx[3] = this->x[0];
            hx[4] = this->x[3];
            hx[5] = this->x[4];
            */
            //when not using VOD
            hx[3] = 0.0;
            hx[4] = 0.0;
            hx[5] = 0.0;

            //Matrix H
            H[0][2] = 1;
            H[1][1] = 1;
            H[1][2] = 1;
            H[2][5] = 1;
            /*
            //when using VOD
            H[3][0] = 1;
            H[4][3] = 1;
            H[5][4] = 1;
            */
        }
};