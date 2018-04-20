#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

#include <string>
#include <fstream>


using namespace std;
// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main(int argc, char *argv[])
{
  uWS::Hub h;

  PID pid;


  //////////////////////////////////
  // open a file handle to write cte for visualization
  string out_file_name="../outputs/cte.out";
  ofstream out_file (out_file_name, ofstream::out);
  if (!out_file.is_open())  {
	  cerr << "Cannot open output file: " << out_file_name << endl;
	  exit(EXIT_FAILURE);
  }
  ////////////////////

  //Grab pid values from command line, sequence is Kp,Ki,Kd
  double init_Kp=0.0;
  double init_Ki=0.0;
  double init_Kd=0.0;
  int twiddle=0; //0 is twiddle disable
  if (argc <= 1) {
	  cout << "Since No Arguements are given, Program will be run in non-Twiddle mode"<<endl;
	  cout << "Arguement list: twiddle_flag[0|1],Kp,Ki,Kd"<<endl;
	  //cout << "Quitting program"<<endl;
	  //exit(EXIT_FAILURE);

  }
  if (argc>1) {
	  twiddle=atoi(argv[1]);
	  if (argc>2) init_Kp=atof(argv[2]);
	  if (argc>3) init_Ki=atof(argv[3]);
	  if (argc>4) init_Kd=atof(argv[4]);
	  if (argc>5) {
		  cout << "Too Many Arguements"<<endl;
		  cout << "Arguement list: twiddle_flag[0|1],Kp,Ki,Kd"<<endl;
		  cout << "Quitting program"<<endl;
		  exit(EXIT_FAILURE);
	  }
  }


  //Initialize the pid variable.
  //if twiddle routine not to be used, set PID to previously found working value
  if (twiddle == 0) {
	  cout <<"PID init with twiddle disabled"<<endl;

	  //final PID gains
	  pid.Init(0.134587, 0.0, 3.05298,false,3);

	  //test with P only for report
	  //pid.Init(0.134587, 0.0, 0.0,false,3);
	  //test with PI only for report
	  //pid.Init(0.134587, 0.01, 0.0,false,3);

	  //pid.Init(0.109, 0.0, 1,false,3);
	  //pid.Init(0.654432, 0.0, 14.8397,false,3);

	  //values of p/dp found in twiddle loops ate different times (simulator is random!!)
	  //Twiddle stopped: 0,  Sum of Dp: 2.511
	  //p[0]: 0.109 p[1]: 0 p[2]: 1
	  //dp[0]: 0.891 dp[1]: 0.729 dp[2]: 0.891

	  //Twiddle stopped: 0 After: 58 Twiddle Loops --> slight issue at last turn
	  //Sum of Dp: 1.94714
	  //p[0]: 0.854432 p[1]: 0.01 p[2]: 14.8397
	  //dp[0]: 0.700274 dp[1]: 0.468779 dp[2]: 0.778083

	  //Twiddle stopped: 0 After: 85 Twiddle Loops --works well no crossing yellow
	  //Sum of Dp: 0.935124
	  //p[0]: 0.760926 p[1]: 0.01 p[2]: 15.9851
	  //dp[0]: 0.375914 dp[1]: 0.307566 dp[2]: 0.251645

	  //Twiddle stopped: 0 After: 106 Twiddle Loops --works well no crossing yellow, reduced Ki to 0
	  //Sum of Dp: 0.668929
	  //p[0]: 0.134587 p[1]: 0.01 p[2]: 3.05298
	  //dp[0]: 0.275914 dp[1]: 0.104657 dp[2]: 0.288358

  } else {
	  cout <<"PID init with twiddle enabled"<<endl;
	  pid.Init(init_Kp, init_Ki, init_Kd,true,3);
  }

  h.onMessage([&pid,&out_file](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
          /*
          *  Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */

          ////////////////////////////////////////////////////////////////////////
          //push the cte into output file
          //out_file << cte <<endl;

          //dbg keep track of max cte when simulator didn't reset for whole track (5000 steps)
          //abs means cte can have been +/-
          if((pid.step> 1500 && fabs(cte) > pid.max_cte) || !pid.twiddle ) {
        	  pid.max_cte=fabs(cte);
        	  //cout << "In Twiddle iteration: "<<pid.twiddle_loop_count<<" Max cte is :"<<pid.max_cte<<endl;
          }


          //Exit Twiddle mode when sumDp becomes smaller than tolerance(0.2)
          //if (pid.twiddle && pid.TwSumDp() < 0.2) {
          //or whole track has been traversed witch exceeding cte of 2.0
          //if (pid.twiddle && (pid.TwSumDp() < 0.2 || (pid.step > 5000 && pid.max_cte < 2.0))) {
          //if (pid.twiddle && (pid.TwSumDp() < 0.2 || pid.step > 5000 )) {
          if (pid.twiddle && (pid.TwSumDp() < 0.1 || pid.step > 6000 )) {
        	  pid.twiddle=false;
        	  cout <<"Twiddle stopped: "<<pid.twiddle<<" After: "<<pid.twiddle_loop_count<<" Twiddle Loops"<<endl;
        	  cout <<"Sum of Dp: "<< pid.TwSumDp() <<endl;
        	  cout <<"p[0]: "<<pid.p[0]<<" p[1]: "<<pid.p[1]<<" p[2]: "<<pid.p[2]<<endl;
        	  cout <<"dp[0]: "<<pid.dp[0]<<" dp[1]: "<<pid.dp[1]<<" dp[2]: "<<pid.dp[2]<<endl;
        	  cout << "When Twiddle iteration stopped, Max cte is :"<<pid.max_cte<<endl;
          }




          ////////////////////////////////////////////////////////////////////////
          //if twiddle is enabled, for each simulator step
          //increase step count --> step is distance travelled in simulator units
          //if param_index has changed increase p by +=dp
          //check if step > nb_settle/2 has passed then start accumultaing error (cte**2)
          // if step > nb_settle, then check if  cte > 4.0 or speed < 1.0, it's a good time to run twiddle checks and reset
          //simulator update Kp,Ki,Kd with value of p
          // NOTE: speed is zero at the beginning and takes about 100 steps to go past 1.0, so nb_settle > 100
          // when performing twiddle check, condition to track is if p has increased or decreased in previous step
          //if p has increased (which happens when avg_error < best_error) --> multiply dp by 1.1 and move to next param_index
          //if p has decreased, reduce p by 2*dp (since was previously increased by dp, so reducing double to compensate) and restart
          //simulator, if the avg_error is still < best_error, then restore p to +dp and reduce dp by factor 0.9 and chang param_index

          if (pid.twiddle==1) {

        	  //push the cte into output file
        	  out_file << pid.step<<","<<cte <<endl;
        	  pid.step++;
        	  pid.distance_travelled=pid.step;

        	  //first few steps after reset ctee doesn't reset
        	  if (pid.step > 5) {
        		  pid.total_error += cte*cte;
        		  pid.avg_error=pid.total_error/(pid.step-5);
        	  }
        	  //if (pid.step > pid.nb_settle/2) {
        		//  //then start aggregating error
        		//  pid.total_error += cte*cte;
        	  //}
        	  //if stabilization steps done and cte and speed outof target
        	  //if (pid.step > pid.nb_settle && (fabs(cte) > 3.0 || speed < 1.0)) {
        	  if (pid.step > pid.nb_settle && (fabs(cte) > 3.0 || speed < 1.0)) {
        		  out_file << "Step: "<<pid.step<< " Reset" <<endl;
        		  //calculate avg_error
        		  //pid.avg_error=pid.total_error/(pid.step-pid.nb_settle/2);
        		  //pid.avg_error=pid.total_error/pid.step;
        		  //to track as many twiddle loops run sofar
        		  pid.twiddle_loop_count++;

        		  //for debug
        		  cout<<"idx: "<<pid.param_index<<" Step: "<<pid.step<<" Total error: "<<pid.total_error<<" Avg: "<<pid.avg_error<<" Best error: "<<pid.best_error<<endl;
        		  cout<<"idx: "<<pid.param_index<<" pincr: "<<pid.p_incr<<" dpincr: "<<pid.dp_incr<<endl;

        		  //now check if avg_error < best_error
        		  if(pid.avg_error < pid.best_error) {
        			  //update dp (*=1.1) and increase index then add dp to next p
        			  pid.TwBestError();

        			  //crude way of breaking local minima
        			  //keep track of when last time avg_error was < best_error
        			  //if it has been more than 50 twiddle lop shake things up :)
        			  // change value of p[2] --> Kd
        			  pid.best_error_loop=pid.twiddle_loop_count;


        			  cout <<"avg_error < Best error"<<endl;
        			  cout<<"P: "<<pid.p[0]<<" "<<pid.p[1]<<" "<<pid.p[2]<<endl;
        			  cout<<"DP: "<<pid.dp[0]<<" "<<pid.dp[1]<<" "<<pid.dp[2]<<endl;
        			  cout<<"avg_err < best_err was found in twidle loop:"<<pid.best_error_loop<<endl;

        			  //if(pid.avg_error < pid.best_error)

        		  } //if avg_error > best error then reduce p by 2*dp reset and run simulator again if coming from p_incr
        		  //or reduce dp by 0.9 dp and move to next parameter
        		  else {
        			  //if avg_err>best_err and coming from top loop
        			  //if(pid.dp_incr && pid.p_incr) {
        			  if ( pid.p_incr) {
        				  pid.TwReduceParam();
        				  cout <<"avg_error > Best error, reduce p by -2*dp"<<endl;
        				  cout<<"P: "<<pid.p[0]<<" "<<pid.p[1]<<" "<<pid.p[2]<<endl;
        				  cout<<"DP: "<<pid.dp[0]<<" "<<pid.dp[1]<<" "<<pid.dp[2]<<endl;
        			  }
        			  //else if coming from a p increase and still avg_err > best_err reduce dp by 0.9*dp
        			  //and move to next param
        			  //else if (!pid.p_incr && pid.dp_incr){
        			  else {
        				  pid.TwReduceDParam();
        				  cout <<"avg_error > Best error dp decreased"<<endl;
        				  cout<<"P: "<<pid.p[0]<<" "<<pid.p[1]<<" "<<pid.p[2]<<endl;
        				  cout<<"DP: "<<pid.dp[0]<<" "<<pid.dp[1]<<" "<<pid.dp[2]<<endl;

        			  }

        			  //else
        		  }


        		  //check if last avg_error < best_error was greater than 50
        		  //then shake Kd --> p[2] +=1.0;
        		  if ((pid.twiddle_loop_count-pid.best_error_loop) > 20) {
        			  //increase Kd to 2*Kp if existing Kd is < 2*Kp
        			  double n_kd=pid.p[0]*5.0; //a bit too drastic?? damping factor increased
					  if (n_kd > pid.p[2]) {
						  pid.p[2]= n_kd;
					  }
        			  //reduce Kp by 2
        			  pid.p[0] /=2.0; //ratio of Kd/Kp is now 10, vehicle is very sensitive to Kp gain
        			  pid.best_error_loop=pid.twiddle_loop_count;
        			  cout<<"Twiddle loop shaken at step: "<<pid.twiddle_loop_count<<endl;
        		  }

        		  // update Kp,Ki,Kd before going into reset
        		  pid.TwUpdateGains();
        		  pid.d_error=0;
        		  pid.i_error=0;

        		  //we are here since cte or speed limit breached,so reset simulator
        		  pid.step=0;
        		  pid.total_error=0.0;
        		  pid.avg_error=0.0;
        		  cout<<"Resetting sim with cte: "<<cte<<" Speed: "<<speed<<" Best dist:"<<pid.best_distance<<endl;
        		  cout<<"Sumdp: "<<pid.TwSumDp()<<" twiddle loops sofar: "<<pid.twiddle_loop_count<<endl;
        		  cout<<"avg_err < best_err was last found in twiddle loop: "<<pid.best_error_loop<<" i.e. " << pid.twiddle_loop_count-pid.best_error_loop << " twiddle loop ago!"<<endl;
        		  //reset
        		  std::string msg = "42[\"reset\",{}]";
        		  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

        		  //if (pid.step > pid.nb_settle && (fabs(cte) > 4.0 || speed < 1.0))
        	  }

        	  //if (pid.twiddle==1)

          }



          pid.UpdateError(cte);
          steer_value = pid.TotalError();
          double throttle_value = 0.3;

          if (steer_value > 1.0) {
        	  steer_value = 1.0;
          } else if (steer_value < -1.0) steer_value = -1.0;

          // DEBUG
          if(!pid.twiddle) std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
  //close the data capture file
    out_file.close();
}
