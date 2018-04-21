#include "PID.h"
#include <iostream>

#include <vector>
#include <numeric>
#include <limits>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd,bool twiddle,int nb_params) {
	this-> Kp=Kp;
	this-> Ki=Ki;
	this-> Kd=Kd;
	p_error=0;
	i_error=0;
	d_error=0;

	// Twiddle related params
	this->twiddle=twiddle;
	// Initialize dp
	this->nb_params=nb_params;
	dp={1.0,1.0,1.0};
	//dp={0.5,0.1,0.5};
	//dbg twiddle stop cond
	//dp={0.19,0,0};
	//p={Kp,Ki,Kd};
	p={Kp+1.0,Ki,Kd};

	step=0;
	param_index=0;
	sim_run=true;
	p_incr=true;
	p_decr=false;
	dp_incr=false;
	dp_decr=false;

	twiddle_loop_count=0;
	total_error=0.0;
	avg_error=0.0;
	//best_error=1000;
	best_error=std::numeric_limits<long int>::max();

	nb_settle=200;
	//nb_settle=100;
	//nb_settle=50;
	//nb_settle=400;
	//nb_settle=1200;

	distance_travelled=0;
	best_distance=nb_settle;
	best_distance_berror=step;


	brake_step=0;
	brake_applied=false;

	max_cte=0.0;
	best_error_loop=0;
	//setting init
	//sumdp=100.0;

	//if(this->twiddle) {
	//	this->Kp=p[0];
	//}

	for(int i=0;i<2*nb_settle;i++){
		angle_vector.push_back(0.0);
	}

	cout<<"avg angle size is:"<<angle_vector.size()<<endl;

	cout <<"PID Init Done!"<<endl;
	cout <<"Kp_Init: "<<this->Kp<<" Ki_Init: "<<this->Ki<<" Kd_Init: "<<this->Kd<<endl;
}

void PID::UpdateError(double cte) {
	//p_error at first step will be 0, so d_error=cte

	d_error=cte - p_error;
	p_error=cte;
	i_error +=cte;
	//cout<<"d_err: " <<d_error<<" p_error: "<<p_error<<" i_error: "<<i_error<<endl;
}

double PID::TotalError() {
	double t_error=-Kp*p_error + -Ki*i_error + -Kd*d_error;
	//cout<<"t_error"<<t_error<<endl;
	return t_error;
}

//////////////////////////
/*
 * Twiddle related functions
 */
double PID::TwSumDp() {
	double sd= accumulate(dp.begin(),dp.end(),0.0);
	//using int (0) as initial makes accumulate return int!
	//cout <<"Int sd: "<<sd<<endl;
	return sd;
}

void PID::TwBestError(){
	best_error = avg_error;
	if(step > best_distance_berror) {
		best_distance_berror = step;
	}
	dp[param_index] *= 1.1;
	//dp[param_index] *= 1.2;
	dp_incr=true;
	TwIncrIdx();
	TwIncreaseParam();
}

void PID::TwReduceParam(){
	p[param_index] -=2.0*dp[param_index];
	p_incr=false;
}

void PID::TwReduceDParam(){
	p[param_index] +=dp[param_index];
	dp[param_index] *=0.9;
	//dp[param_index] *=0.95;
	dp_incr=false;
	TwIncrIdx();
	TwIncreaseParam();
}


void PID::TwIncrIdx(){
	//param_index inceases cyclically
	param_index = (param_index + 1) % nb_params;
}
void PID::TwIncreaseParam(){
	p[param_index] +=dp[param_index];
	p_incr=true;
}


void PID::TwUpdateGains(){
	Kp=p[0];
	Ki=p[1];
	Kd=p[2];
	cout << "Updated P,I,D params: "<<Kp<<" "<<Ki<<" "<<Kd<<endl;
}

