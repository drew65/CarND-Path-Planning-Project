#ifndef FINITE_STATE_MACHINE_H_
#define FINITE_STATE_MACHINE_H_

#include <iostream>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <map>
//#include "helper_functions.h"

using namespace std;

// structure used to pass new trajectory x and y values back to the main function
struct x_y_vals {
  vector<double> x;
  vector<double> y;
};

// struct used to manage the lane change status between the finite state machine function that determises the requred action and the trajectory planner function that executes the action
struct lane_change {
  bool status;
  int start_lane;
  int target_lane;
};

/*
struct cost_function {
  string name;
  double (cost_function::*fctPtr)();
  double cost;
};
*/

// finte state machine class that stores variables and executes methods required to calculate an optimal trajectory for each cycle of the simulator
class FSM {
public:
  map<char, double> mspeed;
  map<char, double> mcollision;

  struct LessBySecond {
    template <typename Lhs, typename Rhs>
    bool operator()(const Lhs& lhs, const Rhs& rhs) const
    {
        return lhs.second < rhs.second;
    }
  };
  //std::max_element(maxmap.begin(), maxmap.end(), LessBySecond());

  typedef double (FSM::*cfuncType)();
  typedef void (FSM::*afuncType)();
  //unique_ptr<cfuncType> test;

  //struct that holds the name, cost function, action function and cost of an action
  struct cost_function {
    string name;
    cfuncType cfunc;
    afuncType afunc;
    double cost;
  };
  // object function to help sort a list of cost_function types by their cost variable.
  struct cf_less {
    bool operator ()(cost_function const& a, cost_function const& b) const {
      if (a.cost < b.cost) return true;
      if (a.cost > b.cost) return false;
      return false;
    }
  };

  // struct that stores required info for cars within the buffer zone of the sdc in any given lane
  struct near_car {
    int rear_id;
    int forward_id;
    double rear_gap;
    double forward_gap;
    double rear_speed;
    double forward_speed;
    double rear_collision;
    double forward_collision;
  };
  // list of near cars in each lane of the highway
  vector<near_car> near_cars;

  // sdc info given by the simulator
  vector<double> car;
  // sensor info of other cars given by the simulator
  vector<vector<double>> other_cars;
  // matrix of projected trajectories of the other cars from the sensor data
  vector<vector<vector<double>>> other_cars_traj;
  // velocity of the sdc used to calculate the new trajectory after speed modifications
  double ref_velocity;
  int forward_car_id;
  int rear_car_id;
  int lane;
  int next_lane;
  lane_change lc;
  int cycle;
  string previous_state;
  string next_state;
  //double (fp)();

  //vector<cfuncType> func_list;
  // list of lane change functions both cost and actions
  vector<cost_function> collision_cost_list;
  // list of speed functions both cost and actions
  vector<cost_function> speed_cost_list;


  /**
  	* Constructor
  	*/
  FSM();

	/*
 	* Destructor
 	*/
 	virtual ~FSM();

  // get cost and set action functions
  void formList2();
  void clear_near_cars();
  void get_near_cars();
  double lane_collision_cost(int lane1, int lane2);
  string finite_state_machine2(vector<double> car, vector<vector<double>> others);

  //cost functions
  double stay_in_lane_cost();
  double left_lane_change_cost();
  double right_lane_change_cost();
  double keep_speed_cost();
  double speed_up_cost();
  double slow_down_cost();

  //action functions
  void slow_down();
  void speed_up();
  void maintain_speed();
  void keep_lane();
  void left_lane_change();
  void right_lane_change();

  // trajectory planning functions
  x_y_vals plan_trajectory2(vector<double> previous_path_x, vector<double> previous_path_y, vector<double> map_waypoints_s, vector<double> map_waypoints_x, vector<double> map_waypoints_y);

  x_y_vals lane_travel_spline3(vector<double> previous_path_x, vector<double> previous_path_y, vector<double> car, vector<double> map_waypoints_s, vector<double> map_waypoints_x, vector<double> map_waypoints_y);

//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//___________________________________________________________________________NOT IN USE_________________________________________________________________________________________________
  string finite_state_machine(vector<double> car, vector<vector<double>> others);

  x_y_vals plan_trajectory(vector<double> previous_path_x, vector<double> previous_path_y, vector<double> map_waypoints_s, vector<double> map_waypoints_x, vector<double> map_waypoints_y);

  x_y_vals lane_travel1(vector<double> car, vector<double> map_waypoints_s, vector<double> map_waypoints_x, vector<double> map_waypoints_y);

  x_y_vals lane_travel_spline(vector<double> previous_path_x, vector<double> previous_path_y, vector<double> car, vector<double> map_waypoints_s, vector<double> map_waypoints_x, vector<double> map_waypoints_y);

  x_y_vals lane_travel_spline2(vector<double> previous_path_x, vector<double> previous_path_y, vector<double> car, vector<double> map_waypoints_s, vector<double> map_waypoints_x, vector<double> map_waypoints_y);

  string check_collisions();
  // checks for collisions and proposes the next best state

  string check_lane_available(double lc_limit);

  string check_speed();

  void change_lane_left();

  void change_lane_right();

  string emergency_change_lane();

  void accelerate();

  void break_hard();

  void other_cars_trajectory();

  void car_sensor(double end_path_s, int prev_size, vector<double> car, vector<vector<double>> others);

  void formList();

  double collision_cost(int lane);

  void execute_speed_cost();
  void execute_collision_cost();

};


#endif
