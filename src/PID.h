#ifndef PID_H
#define PID_H

#include <vector>

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;


  /*
    * 	params for twiddle
     */
    std::vector<double> p;
    std::vector<double> dp;


    int step, param_index,nb_params;
    // nr of steps to settle
    int nb_settle;
    double total_error, avg_error,best_error;
    int distance_travelled,best_distance;
    int twiddle_loop_count;

    int twiddle;
    bool sim_run,p_incr,p_decr;
    bool dp_incr,dp_decr;

    //double sumdp;

    double max_cte;
    int best_error_loop;



  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  //void Init(double Kp, double Ki, double Kd);
  void Init(double Kp, double Ki, double Kd,bool twiddle,int nb_params);
  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();
  /////////////////////////////////////////////////
  /*
    * Twiddle Related method prototypes.
    */
  void TwBestError();
  void TwIncreaseParam();
  void TwReduceParam();
  void TwReduceDParam();
  double TwSumDp();
  void TwUpdateGains();
  void TwIncrIdx();


};

#endif /* PID_H */
