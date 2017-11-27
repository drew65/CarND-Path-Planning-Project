#include <iostream>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <map>
#include <algorithm>
#include "finite_state_machine.h"
#include "helper_functions.h"
#include "spline.h"

using namespace std;

// constants used throughout the program
int NBR_LANES = 3; //number of lanes on road
double MAX_SPEED = 49.0; //miles per hour
double SPEED_LIMIT = MAX_SPEED/2.24;
double TOLERANCE = 1.0/2.24; // delta to maintain below speed limit
double TRAJ_DUR = 2.0; // duration of trajectories in seconds
double TRAJ_INT = 0.02; // interval between points on trajectoriesin meters
double COL_GAP = 50.0; // minimum gap alowed between cars to avoid collision
double I_COL_GAP = 40.0; // imediate collision possible
int LANE_WIDTH = 4; // width of lane in meters
double LC_LIMIT = 60.0; // minimum gap alowed for a lane change
double MAX_ACC = 9.0; // meters per sec 2
double MAX_JERK = 10.0; // meters per sec 3
int PATH_LEN = 50; // length of projected trajectory in meters
int NBR_PATH_PTS = 50; // number of point on projected path
double VEL_INC = (0.224/2.24)*4.5; // increment to speed used by the "speed_up()" function
double SLOW_DOWN_INC = (0.224/2.24)*5; // increment to speed used by the "slow_down()" function
double CAR_SIZE = 11; // size of clear area around car, used if another viehcule is inside it, to prevent lane change

double COLLISION_COST = (100.0)*2;  // cost accosiated with a collision
double CL_COST = 60.0; // cost of changing lane
double SPEED_COST = 60.0;  // unite cost used in the speed change cost functions
double OUT_LANE_COST = (150.0)*2;  // cost of turnning out of the 3 main lanes on the highway
double MIN_SPEED_CL = SPEED_LIMIT/2; // the minimum speed required before a lane change can occur


// FSM "finite_state_machine" class methods
/**
  	* Constructor
  	*/
FSM::FSM() {

  this->previous_state = " ";
  this->ref_velocity = 0.0;
  this->cycle = 0;
  this->next_lane = -1;
  this->lc.status = false;

  //set up list of cost functions
  //formList();
  formList2();

  //set-up default values for vector containing information relating to the nearest cars behined and infront of the self driving car in each of the highway lanes
  for (int i=0;i!=NBR_LANES;i++) {
    near_car item;
    item.rear_id = -1;
    item.forward_id = -1;
    item.rear_gap = COL_GAP;
    item.forward_gap = COL_GAP;
    item.rear_speed = 0.0;
    item.forward_speed = 0.0;
    item.rear_collision = false;
    item.forward_collision = false;
    near_cars.push_back(item);
  }
}

	/**
 	* Destructor
 	*/
FSM::~FSM() {

}

/*
// legasy list of cost functions
void FSM::formList() {
    func_list.push_back(&FSM::right_lane_change_cost);
    func_list.push_back(&FSM::stay_in_lane_cost);
    func_list.push_back(&FSM::left_lane_change_cost);
}
*/

// inisialisation of the lane change cost function list and the speed cost function list
void FSM::formList2() {
    cost_function item;
    // collision cost functions list
    item.name = "RLC";
    item.cfunc = &FSM::right_lane_change_cost;
    item.afunc = &FSM::right_lane_change;
    item.cost = 0.0;
    collision_cost_list.push_back(item);
    cost_function item2;
    item2.name = "KL";
    item2.cfunc = &FSM::stay_in_lane_cost;
    item2.afunc = &FSM::keep_lane;
    item2.cost = 0.0;
    collision_cost_list.push_back(item2);
    cost_function item3;
    item3.name = "LLC";
    item3.cfunc = &FSM::left_lane_change_cost;
    item3.afunc = &FSM::left_lane_change;
    item3.cost = 0.0;
    collision_cost_list.push_back(item3);

    // speed cost functions list
    cost_function item4;
    item4.name = "KS";
    item4.cfunc = &FSM::keep_speed_cost;
    item4.afunc = &FSM::maintain_speed;
    item4.cost = 0.0;
    speed_cost_list.push_back(item4);
    cost_function item5;
    item5.name = "SU";
    item5.cfunc = &FSM::speed_up_cost;
    item5.afunc = &FSM::speed_up;
    item5.cost = 0.0;
    speed_cost_list.push_back(item5);
    cost_function item6;
    item6.name = "SD";
    item6.cfunc = &FSM::slow_down_cost;
    item6.afunc = &FSM::slow_down;
    item6.cost = 0.0;
    speed_cost_list.push_back(item6);
}

// each cycle of the simulator the near_car list needs to be reset to default values
void FSM::clear_near_cars() {
  for( int i=0; i<near_cars.size(); i++) {
    near_cars[i].rear_id = -1;
    near_cars[i].forward_id = -1;
    near_cars[i].rear_gap = COL_GAP;
    near_cars[i].forward_gap = COL_GAP;
    near_cars[i].rear_speed = 0.0;
    near_cars[i].forward_speed = 0.0;
    near_cars[i].rear_collision = false;
    near_cars[i].forward_collision = false;
  }

  /*
  // make maps for speed and collision costs
  mspeed['a'] = 0.0;
  mspeed['b'] = 0.0;
  mspeed['c'] = 0.0;
  mcollision['a'] = 0.0;
  mcollision['b'] = 0.0;
  mcollision['c'] = 0.0;
}
*/
}

void FSM::get_near_cars() {
  //function that populates the near_car list (nearest crs in each lane infront of and behind the self driving car)

  // reset the near_car list to default values
  clear_near_cars();


  int sdc_index = other_cars_traj[0].size() - 1;
  //for each highway lane
  for (int lane=0; lane!=NBR_LANES; lane++) {
    // for each car on the highway
    for (int i=0; i < sdc_index; i++) {
      double d = other_cars_traj[0][i][2];
      // if car is in lane
      if (d<(2+4*lane+2) && d>(2+4*lane-2)) {
        // for each point on the simulated trajectory of each car
        for (int j=0; j<other_cars_traj.size(); j++) {
          double s = other_cars_traj[j][i][0];
          double sdc_s = other_cars_traj[j][sdc_index][0];
          //find nearest car forward of self driving car
          if ((s > sdc_s) && (s - sdc_s <   near_cars[lane].forward_gap)) {
            near_cars[lane].forward_gap = s - sdc_s;
            near_cars[lane].forward_id = i;
            near_cars[lane].forward_speed = other_cars_traj[j][i][1];
            if (((s - sdc_s < I_COL_GAP) && (near_cars[lane].forward_speed < ref_velocity+1))||(s - sdc_s < CAR_SIZE)) {
              //if car is in potential collision zone then set collision flag
              near_cars[lane].forward_collision = true;
            }
          }
          //find nearest car behind self driving car
          else if ((s < sdc_s) && (sdc_s - s < near_cars[lane].rear_gap)) {
            near_cars[lane].rear_gap = sdc_s - s;
            near_cars[lane].rear_id = i;
            near_cars[lane].rear_speed = other_cars_traj[j][i][1];
            if (((sdc_s - s < I_COL_GAP) && (near_cars[lane].rear_speed > ref_velocity-1))||(sdc_s - s < CAR_SIZE)) {
              //if car is in potential collision zone then set collision flag
              near_cars[lane].rear_collision = true;
            }
          }
        }
      }
    }
  }

}

double FSM::lane_collision_cost(int lane1, int lane2) {
  double cost = 0.0;
  // calculate cost of staying in/ moving to "lane1 from lane2"
  // this function can be used by all of the possible lane state cost evaluation functions (ie left_lane_change_cost(), right_lane_change_cost() and stay_in_lane_cost())
  // it uses the data from the near_car list to determin each cost
  if (near_cars[lane1].forward_id != -1) {
    cost -=  COL_GAP - near_cars[lane].forward_gap;
    if (near_cars[lane1].forward_speed < ref_velocity) {
      cost -= CL_COST;
    }
    if (near_cars[lane1].forward_speed < near_cars[lane2].rear_speed) {
      cost -= CL_COST;
    }
  }
  else {
    cost += COL_GAP;
  }
  if (near_cars[lane1].rear_id != -1) {
    cost -=  COL_GAP - near_cars[lane].rear_gap;
    if (near_cars[lane1].rear_speed > ref_velocity) {
      cost -= CL_COST;
    }
    if (near_cars[lane1].rear_speed > near_cars[lane2].forward_speed) {
      cost -= CL_COST;
    }
  }
  if (near_cars[lane1].forward_collision) {
    cost -= COLLISION_COST;
  }
  if (near_cars[lane1].rear_collision) {
    cost -= COLLISION_COST;
  }
  return cost;
}


double FSM::right_lane_change_cost() {
  // Calculates the cost of a right lane change
  double cost = 0.0;
  int lane = (int) other_cars_traj[0][other_cars_traj[0].size() - 1][2] / LANE_WIDTH;
  if (lane +1 >= NBR_LANES) {
    cost -= OUT_LANE_COST;
  }
  else {
    cost += lane_collision_cost(lane+1, lane);
  }
  // defines a minimum speed required before a lane change and sets a cost if it is not reached
  if (ref_velocity < MIN_SPEED_CL) {
    cost -= OUT_LANE_COST;
  }
  return cost;
}

double FSM::left_lane_change_cost() {
  // Calculates the cost of a left lane change
  double cost = 0.0;
  int lane = (int) other_cars_traj[0][other_cars_traj[0].size() - 1][2] / LANE_WIDTH;
  if (lane -1 <0) {
    cost -= OUT_LANE_COST;
  }
  else {
    cost = lane_collision_cost(lane-1, lane);
  }
  // defines a minimum speed required before a lane change and sets a cost if it is not reached
  if (ref_velocity < MIN_SPEED_CL) {
    cost -= OUT_LANE_COST;
  }
  return cost;
}

double FSM::stay_in_lane_cost() {
  //calculates the cost of keeping the same lane
  int lane = (int) other_cars_traj[0][other_cars_traj[0].size() - 1][2] / LANE_WIDTH;
  double cost = lane_collision_cost(lane, lane);
  return cost;
}

double FSM::keep_speed_cost() {
  //calculates the cost of maintaing the same speed
  double cost = 0.0;
  int sdc_index = other_cars_traj[0].size() - 1;
  int lane = (int) other_cars_traj[0][sdc_index][2] / LANE_WIDTH;
  double sdc_speed = other_cars_traj[0][sdc_index][1];
  // only reward maintaining current speed if sdc speed is within the tolerance zone of the speed limit
  if ((SPEED_LIMIT - sdc_speed < TOLERANCE) && (SPEED_LIMIT - sdc_speed > 0)) {
    // reward maintaining speed if there is no car in front or if the front car is moving faster than the sdc
    if ((near_cars[lane].forward_id == -1) || (near_cars[lane].forward_speed > sdc_speed)) {
      // and if there is no car behind or the rear car is moving lslower than the sdc
      if ((near_cars[lane].rear_id == -1) || (near_cars[lane].rear_speed < sdc_speed)) {
        cost += SPEED_COST;
      }
    }
  }
  return cost;
}

double FSM::speed_up_cost() {
  //calculates the cost acceleration
  double cost = 0.0;
  int sdc_index = other_cars_traj[0].size() - 1;
  int lane = (int) other_cars_traj[0][sdc_index][2] / LANE_WIDTH;
  double sdc_speed = other_cars_traj[0][sdc_index][1];
  //rewards acceleration if sdc speed not not within a preset tolerance of the speed limit
  if (SPEED_LIMIT - sdc_speed > TOLERANCE) {
    // reward acceleration if there is no car in the preset buffer zone in front of the sdc or it is moving faster than the sdc
    if ((near_cars[lane].forward_id == -1) || (near_cars[lane].forward_speed > sdc_speed)) {
      cost += SPEED_COST;
    }
    // reward acceleration if there is a car in the preset buffer zone behind the sdc and it is moving faster than the sdc
    if ((near_cars[lane].rear_id != -1) && (near_cars[lane].rear_speed > sdc_speed)) {
      cost += SPEED_COST;
    }
  }
  // reward acceleration if sdc car is within the speed limit and there is a risk of rear collision
  if ((SPEED_LIMIT - sdc_speed > 0) && (near_cars[lane].rear_collision)) {
    cost += SPEED_COST;
  }
  return cost;
}

double FSM::slow_down_cost() {
  double cost = 0.0;
  int sdc_index = other_cars_traj[0].size() - 1;
  int lane = (int) other_cars_traj[0][sdc_index][2] / LANE_WIDTH;
  double sdc_speed = other_cars_traj[0][sdc_index][1];
  // reward breaking if sdc speed is greater than the speed limit
  if (SPEED_LIMIT - sdc_speed <= 0) {
    cost += SPEED_COST;
  }
  // reward breaking if if there is a car in a preset buffer zone in front of sdc and if that car is moving slower than sdc
  if ((near_cars[lane].forward_id != -1) && (near_cars[lane].forward_speed < sdc_speed)) {
    cost += SPEED_COST*2;
    //also reward breaking if there is a seriouse risk of forward collision
    if (near_cars[lane].forward_collision) {
      cost += SPEED_COST*2;
    }
  }
  return cost;
}

/*
// function has be replaced by a vector of struct containing pointers to cost and action functions as well as cost of actions this code is now in function finite_state_machine2()
void FSM::execute_collision_cost() {
  mcollision['a'] = left_lane_change_cost();
  mcollision['b'] = stay_in_lane_cost();
  mcollision['c'] = right_lane_change_cost();
  for (auto itm : mcollision) {
    cout << itm.first << "  " << itm.second << "  ";
  }
  cout << endl;
  map<char, double>::const_iterator itr;
  itr = max_element(mcollision.begin(), mcollision.end(), LessBySecond());
  switch((*itr).first) {
    case 'a':
      cout << "LLC ";
      left_lane_change();
      break;
    case 'b':
      cout << "KL  ";
      keep_lane();
      break;
    case 'c':
      cout << "RLC ";
      right_lane_change();
      break;
    default:
      break;
  }
  cout << " cost " << (*itr).second << endl;
}
// function has be replaced by a vector of struct containing pointers to cost and action functions as well as cost of actions this code is now in function finite_state_machine2()
void FSM::execute_speed_cost() {
  mspeed['a'] = slow_down_cost();
  mspeed['b'] = keep_speed_cost();
  mspeed['c'] = speed_up_cost();
  for (auto itm : mspeed) {
    cout << itm.first << "  " << itm.second << "  ";
  }
  cout << endl;
  map<char, double>::const_iterator itr;
  itr = max_element(mspeed.begin(), mspeed.end(), LessBySecond());
  switch((*itr).first) {
    case 'a':
      cout << "SD  ";
      slow_down();
      break;
    case 'b':
      cout << "KS  ";
      maintain_speed();
      break;
    case 'c':
      cout << "SU  ";
      speed_up();
      break;
    default:
      break;
  }
  cout << " cost " << (*itr).second << endl;
}
*/

string FSM::finite_state_machine2(vector<double> car, vector<vector<double>> others) {
  this->car = car;
  this->other_cars = others;
  ref_velocity = car[5]/2.24;
  lane = (int) car[3]/4;
  cycle += 1;
  //cout << "Cycle " << cycle << "  SPEED_LIMIT " << SPEED_LIMIT << "  car speed "  << ref_velocity << endl;

  // map possitions of all cars on road
  other_cars_trajectory();

  //get details of  cars in all lanes within forward and rear buffer zones of sdc as well as a collision indicator
  get_near_cars();

  //execute_speed_cost();

  // calculate the speed cost of each of the possible speed options used (speed_up, keep speed, slowdown) then perform the best action
  for (int i=0; i != speed_cost_list.size(); i++) {
    speed_cost_list[i].cost = (this->*(speed_cost_list[i].cfunc))();
  }
  //sort vector of cost function struct by cost
  sort(speed_cost_list.begin(), speed_cost_list.end(), cf_less());
  //Update car speed
  // do the action with the highest cost (negative cost are undesirable actions and positive cost are desirable)
  (this->*(speed_cost_list[speed_cost_list.size()-1].afunc))();

  //get cost of lane change
  //get_near_cars();
  //vector<funcType>::iterator it;
  //for (it = func_list.begin(); it != func_list.end(); ++it) {
        //(this->*(*it))(); // call each function in list
  //}
  //execute_collision_cost();

  // calculate the lane change cost of each of the possible lane change options used (left_lane_change, keep_lane, right_lane_change) then perform the best action
  // call each cost function and record cost
  for (int i=0; i != collision_cost_list.size(); i++) {
    collision_cost_list[i].cost = (this->*(collision_cost_list[i].cfunc))();
  }
  //sort vector of cost function struct by cost
  sort(collision_cost_list.begin(), collision_cost_list.end(), cf_less());
  // do the action with the highest cost (negative cost are undesirable actions and positive cost are desirable)
  (this->*(collision_cost_list[collision_cost_list.size()-1].afunc))();

  //cout << "Speed cost "  << speed_cost_list[speed_cost_list.size()-1].cost << "  action " << speed_cost_list[speed_cost_list.size()-1].name << endl;
  //cout << "Collision cost " << collision_cost_list[collision_cost_list.size()-1].cost << "  action " << collision_cost_list[collision_cost_list.size()-1].name << endl;
  return "test";
}

//speed action function
void FSM::slow_down() {
  //decrease the ref_velocity of the sdc, to be used in trajectory calculation, by a preset amount
  ref_velocity -= SLOW_DOWN_INC;
  // if the sdc speed tends towards zero then maintain slow possitive velocity. This stops the spline algorithem from crashing with a zero error.
  if (ref_velocity < 1.0) {
    ref_velocity = 1.0;
  }
  // if the new ref_velocity is above the speed limit then reduce to speed limit
  if (ref_velocity > SPEED_LIMIT) {
    ref_velocity = SPEED_LIMIT;
  }
}

//speed action function
void FSM::speed_up() {
  //increase the ref_velocity of the sdc, to be used in trajectory calculation, by a preset amount
  ref_velocity += VEL_INC;
  // if the new ref_velocity is above the speed limit then reduce to speed limit
  if (ref_velocity > SPEED_LIMIT) {
    ref_velocity = SPEED_LIMIT;
  }
}

//speed action function
void FSM::maintain_speed() {
  //keep current reference velocity
  ref_velocity = ref_velocity;
}

//lane change action function
// not action needed as no lane change requested
void FSM::keep_lane() {
  //blank
}

//lane change action function
void FSM::left_lane_change() {
  //set next lane in trajectory planner to be the lane to the left of the current lane
  double d = car[3];
  int car_lane = (int) d/4;
  //check the lane change status (is there currently a lane change in progress or not?)
  if (lc.status) {
    //if yes
    if (d<(2+4*lc.target_lane+1) && d>(2+4*lc.target_lane-1)) {
      // if the sdc has moved  all its body into the requred lane (its center possition is in the 2 meter center zone of the new lane) then stop lane change
      //next_state = "check_collisions";
      lc.status = false;
      lc.target_lane = car_lane;
      lc.start_lane = car_lane;
    }
    else {
      // else continue lane change
      next_state = "LLC";
    }
  }
  else {
    // if there is no lane change in progress
    if ((car_lane >= 1)&& (near_cars[car_lane-1].forward_collision==false)&&(near_cars[car_lane-1].rear_collision==false)&&(ref_velocity > MIN_SPEED_CL)) {
      /* check that the sdc:
      - is not trying to move out of the road
      - there is no car within the forward collision zone of the requested lane
      - there is no car within the rear collision zone of the requested lane
      - that the ref_velocity is greater that a min speed constant (this stops slow lane changes, which can cause collisions)
      */
      // set target lane to next lane on left and set lane change status to true
      lc.start_lane = car_lane;
      lc.target_lane = car_lane - 1;
      lc.status = true;
    }
    else {
      // if no lane change is possible then set lane change status to false and target lane to current lane
      lc.status = false;
      lc.start_lane = car_lane;
      lc.target_lane = car_lane;
    }
  }
}

//lane change action function
void FSM::right_lane_change() {
  //set next lane in trajectory planner to be the lane to the right of the current lane
  double d = car[3];
  int car_lane = (int) d/4;
  //check the lane change status (is there currently a lane change in progress or not?)
  if (lc.status) {
    //if yes
    if (d<(2+4*lc.target_lane+1) && d>(2+4*lc.target_lane-1)) {
      // if the sdc has moved  all its body into the requred lane (its center possition is in the 2 meter center zone of the new lane) then stop lane change
      //next_state = "check_collisions";
      lc.status = false;
      lc.target_lane = car_lane;
      lc.start_lane = car_lane;
    }
    else {
      // else continue lane change
      next_state = "RLC";
    }
  }
  else {
    // if there is no lane change in progress
    if ((car_lane <= 1)&&(near_cars[car_lane+1].forward_collision==false)&&(near_cars[car_lane+1].rear_collision==false)&&(ref_velocity > MIN_SPEED_CL)) {
      /* check that the sdc:
      - is not trying to move out of the road
      - there is no car within the forward collision zone of the requested lane
      - there is no car within the rear collision zone of the requested lane
      - that the ref_velocity is greater that a min speed constant (this stops slow lane changes, which can cause collisions)
      */
      // set target lane to next lane on right and set lane change status to true
      lc.start_lane = car_lane;
      lc.target_lane = car_lane + 1;
      lc.status = true;
    }
    else {
      // if no lane change is possible then set lane change status to false and target lane to current lane
      lc.status = false;
      lc.start_lane = car_lane;
      lc.target_lane = car_lane;
    }
  }
}

// based on the sensor data, calculate the futur trajectories of all ythe cars on the road and store in a matrix
void FSM::other_cars_trajectory() {
  int total_pts = (int) TRAJ_DUR / TRAJ_INT;
  int lane = (int) car[3]/4;
  //cout << "sdc s =  " << car[2] << endl;
  vector<vector<double>> all_cars = other_cars;
  vector<vector<double>> first_traj;
  for (int i=0; i<all_cars.size(); i++) {
    double first_s = all_cars[i][5];
    double first_d = all_cars[i][6];
    double vx = all_cars[i][3];
    double vy = all_cars[i][4];
    double speed = sqrt(vx*vx+vy*vy);
    first_traj.push_back({first_s, speed, first_d});
  }
  first_traj.push_back({car[2], car[5]/2.24, car[3]});
  vector<vector<vector<double>>> other_cars_traj1;
  other_cars_traj = other_cars_traj1;
  other_cars_traj.push_back(first_traj);
  for (int i=1; i<total_pts; i++){
    //increment all trajectories
    vector<vector<double>> next_traj;
    for (int j=0; j<first_traj.size(); j++) {
      double next_s = other_cars_traj[i-1][j][0] + other_cars_traj[i-1][j][1]*TRAJ_INT;
      double next_speed = other_cars_traj[i-1][j][1];
      double next_d = other_cars_traj[i-1][j][2];
      next_traj.push_back({next_s, next_speed, next_d});
    }
    other_cars_traj.push_back(next_traj);
  }
}

x_y_vals FSM::plan_trajectory2(vector<double> previous_path_x, vector<double> previous_path_y, vector<double> map_waypoints_s, vector<double> map_waypoints_x, vector<double> map_waypoints_y) {
// This function is there for historical reasons. It maintains compatability with the previouse hardcoded version of the finite state machine algorithm (FSM::plan_trajectory), prior to the inclusion of cost functions to determine the next state (action).

x_y_vals next_xy_vals = lane_travel_spline3(previous_path_x, previous_path_y, car, map_waypoints_s, map_waypoints_x, map_waypoints_y);
return next_xy_vals;
}


// Calculate the new sdc trajectory based on new ref_velcocity, new lane, and the remaining points from the previouse trajectory sent to the simulator
// The code is based on the trajectory planner detailed in the project walkthrough in the leason, it used a spline function to calculate the trajectory.
x_y_vals FSM::lane_travel_spline3(vector<double> previous_path_x, vector<double> previous_path_y, vector<double> car, vector<double> map_waypoints_s, vector<double> map_waypoints_x, vector<double> map_waypoints_y) {
  // checks to see if a lane change is underway and if so assigns the target lane to the lane variable used to calculate the trajectory. Else the sdc current lane is used
  int lane;
  if (lc.status) {
    lane = lc.target_lane;
  }
  else {
    lane = (int) car[3]/4;
  }

  int prev_size = previous_path_x.size();
  vector<double> ptsx;
  vector<double> ptsy;
  double ref_x = car[0];
  double ref_y = car[1];
  double ref_yaw = deg2rad(car[4]);

  if (prev_size < 2) {
    // use 2 pts that make te path tangent to the car
    double prev_car_x = car[0] - cos(ref_yaw); // what about deg2rad conversion??
    double prev_car_y = car[1] - sin(ref_yaw); // what about deg2rad conversion??
    ptsx.push_back(prev_car_x);
    ptsx.push_back(car[0]);
    ptsy.push_back(prev_car_y);
    ptsy.push_back(car[1]);
  }
  else {
    ref_x = previous_path_x[prev_size -1];
    ref_y = previous_path_y[prev_size -1];
    double ref_x_prev = previous_path_x[prev_size -2];
    double ref_y_prev = previous_path_y[prev_size -2];
    ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);
    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);
    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);
  }

  // In Frenet add evenly spaced points ahead of the starting ref (30, 60, and 90 meters)
  int add = 50;
  vector<double> next_wp0 = getXY(car[2] + add, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
  vector<double> next_wp1 = getXY(car[2] + add+add, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
  vector<double> next_wp2 = getXY(car[2] + add+add+add, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);
  ptsy.push_back(next_wp0[1]);
  ptsy.push_back(next_wp1[1]);
  ptsy.push_back(next_wp2[1]);

  // Shift car and rotate car reference angle to 0 degrees
  for (int i=0; i<ptsx.size(); i++) {
    double shift_x = ptsx[i] - ref_x;
    double shift_y = ptsy[i] - ref_y;
    ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw));
    ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw));
  }

  // Create a spline
  tk::spline s;

  // set (x,y) points to spline
  s.set_points(ptsx, ptsy);

  // define actual (x, y) points we will use for Planner
  vector<double> next_x_vals;
  vector<double> next_y_vals;

  // start with previous path pts from last run
  for(int i=0; i<previous_path_x.size(); i++) {
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }
  // calculate how to break up spline points to that we travel at desired velocity
  double target_x = PATH_LEN;
  double target_y = s(target_x);
  double target_dist = sqrt(target_x * target_x + target_y * target_y);
  double x_add_on = 0.0;
  // fill up the rest of our path planner so that we always output 50 points
  int N = (int) (target_dist/(0.02 * ref_velocity));
  //cout << "N  " << N << endl;
  for (int i=1; i<= NBR_PATH_PTS - previous_path_x.size(); i++) {
    N = (target_dist/(0.02 * ref_velocity));
    double x_point = x_add_on + (target_x/N);
    double y_point = s(x_point);
    x_add_on = x_point;
    //rotate and shift bak to global map coord system
    double x_ref = x_point;
    double y_ref = y_point;
    x_point = (x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw));
    y_point = (x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw));
    x_point += ref_x;
    y_point += ref_y;
    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
  }
  x_y_vals next_xy_vals;
  next_xy_vals.x = next_x_vals;
  next_xy_vals.y = next_y_vals;

  return next_xy_vals;
}

/*
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
//___________________________________CODE NOT IN USE ________________________________________________________//
// The following code was designed for the hard coded version of the finite_state_machine that doid not use cost functions it is not nessesary to compleat the project but can be activated by //changing finite_state_machine2() to finite_state_machine() and plan_trajectory2() to plan_trajectory() in main()
*/
void FSM::car_sensor(double end_path_s, int prev_size, vector<double> car, vector<vector<double>> others) {
  //ref_velocity = SPEED_LIMIT;
  int car_s = car[2];
  int lane = (int) car[3]/4;
  if (prev_size > 0) {
    car_s = end_path_s;
  }

  bool too_close = false;
  // find ref_velocity to use
  for (int i=0; i< others.size(); i++) {
    // Is car in my lane
    double d = others[i][6];
    if (d < (2+4*lane+2) && d> (2+4*lane-2)) {
      double vx = others[i][3];
      double vy = others[i][4];
      double check_speed = sqrt(vx*vx + vy*vy);
      double check_car_s = others[i][5];
      check_car_s += ((double) prev_size * 0.02 * check_speed); // if using previouse pts can project s value out
      // check s values are greater than mine and s gap
      if ((check_car_s > car_s) && (check_car_s - car_s < COL_GAP)) {
        // do some logic, lower ref_velocity so we dont crash
        //ref_velocity = 29.5/2.24;
        too_close = true;
      }
    }
  }
  if (too_close) {
    ref_velocity -= 0.224/2.24;
  }
  else if (ref_velocity < SPEED_LIMIT) {
    ref_velocity += 0.224/2.24;
  }
  if (ref_velocity <= 6.0) {
    ref_velocity = 6.0;
  }
  cout << "ref vel = " << ref_velocity << endl;
}

double FSM::collision_cost(int lane) {
  // cost function for remaining/moving to "lane"
  bool forward_collision = false;
  bool forward_danger = false;
  bool rear_collision = false;
  bool rear_danger = false;
  double forward_car_speed;
  double rear_car_speed;
  double forward_gap = COL_GAP;
  double rear_gap = COL_GAP;
  forward_car_id = -1;
  rear_car_id = -1;
  double cost;
  int sdc_index = other_cars_traj[0].size() - 1;
  //int lane = (int) other_cars_traj[0][sdc_index][2] / LANE_WIDTH;
  for (int i=0; i < sdc_index; i++) {
    double d = other_cars_traj[0][i][2];
    if (d<(2+4*lane+2) && d>(2+4*lane-2)) {
      for (int j=0; j<other_cars_traj.size(); j++) {
        double s = other_cars_traj[j][i][0];
        double sdc_s = other_cars_traj[j][sdc_index][0];
        if ((s > sdc_s) && (s - sdc_s < forward_gap)) {
          forward_danger = true;
          forward_gap = s - sdc_s;
          forward_car_id = i;
          forward_car_speed = other_cars_traj[j][i][1];
          if ((s - sdc_s < I_COL_GAP) && (forward_car_speed < other_cars_traj[j][sdc_index][1])) {
            forward_collision = true;
          }
        }
        else if ((s < sdc_s) && (sdc_s - s < rear_gap)) {
          rear_danger = true;
          rear_gap = sdc_s - s;
          rear_car_id = i;
          rear_car_speed = other_cars_traj[j][i][1];
          if ((sdc_s - s < I_COL_GAP) && (rear_car_speed > other_cars_traj[j][sdc_index][1])) {
            rear_collision = true;
          }
        }
      }
    }
  }
  // calculate cost of saying in/ moving to "lane"
  if (forward_danger) {
    cost -=  COL_GAP - forward_gap;
  }
  else {
    cost += COL_GAP;
  }
  if (rear_danger) {
    cost -=  COL_GAP - rear_gap;
  }
  if (forward_collision) {
    cost -= COLLISION_COST;
  }
  if (rear_collision) {
    cost -= COLLISION_COST;
  }
  return cost;
}

string FSM::check_collisions() {
  // checks for collisions and proposes the next best state
  string state;
  bool forward_collision = false;
  bool forward_danger = false;
  bool rear_collision = false;
  bool rear_danger = false;
  double forward_car_speed;
  double rear_car_speed;
  forward_car_id = -1;
  rear_car_id = -1;
  int sdc_index = other_cars_traj[0].size() - 1;
  int lane = (int) other_cars_traj[0][sdc_index][2] / LANE_WIDTH;
  for (int i=0; i < sdc_index; i++) {
    double d = other_cars_traj[0][i][2];
    if (d<(2+4*lane+2) && d>(2+4*lane-2)) {
      for (int j=0; j<other_cars_traj.size(); j++) {
        double s = other_cars_traj[j][i][0];
        double sdc_s = other_cars_traj[j][sdc_index][0];
        if ((s > sdc_s) && (s - sdc_s < COL_GAP)) {
          forward_danger = true;
          forward_car_id = i;
          forward_car_speed = other_cars_traj[j][i][1];
          if (s - sdc_s < I_COL_GAP) {
            forward_collision = true;
          }
        }
        else if ((s < sdc_s) && (sdc_s - s < COL_GAP)) {
          rear_danger = true;
          rear_car_id = i;
          rear_car_speed = other_cars_traj[j][i][1];
          if (sdc_s - s < I_COL_GAP) {
            rear_collision = true;
          }
        }
      }
    }
  }
  if ((rear_collision && forward_danger) || (forward_collision && rear_danger)) {
    state = "emergency_change_lane";
  }
  else if (rear_collision && !forward_danger) {
    state = "accelerate";
  }
  else if (forward_collision && !rear_danger) {
    state = "break_hard";
  }
  else if (forward_danger && rear_danger) {
    if (forward_car_speed > rear_car_speed) {
      state = "check_lane_available";
    }
    else {
      state = "speed_up";
    }
  }
  else if (forward_danger && !rear_danger) {
    state = "check_lane_available";
  }
  else if (rear_danger && !forward_danger) {
    state = "speed_up";
  }
  else {
    state = "check_speed";
  }
  return state;
}

string FSM::check_lane_available(double lc_limit) {
  // set up available lanes to change into
  string state;
  struct lane_change {
    int index;
    string direction;
    bool change_possible;
    double speed;
    double s;
  };
  lane_change new_lane;
  new_lane.change_possible = true;
  new_lane.speed = 0.0;
  new_lane.s = lc_limit;
  vector<lane_change> available_lanes;
  int sdc_lane = (int) car[3] / LANE_WIDTH;
  if (sdc_lane == 1) {
    //lane_change new_lane;
    new_lane.index = 0;
    new_lane.direction = "CLL";
    available_lanes.push_back(new_lane);
    //lane_change new_lane;
    new_lane.index = 2;
    new_lane.direction = "CLR";
    available_lanes.push_back(new_lane);
  }
  else {
    //lane_change new_lane;
    new_lane.index = 1;
    if (sdc_lane == 0) {
      new_lane.direction = "CLR";
    }
    else {
      new_lane.direction = "CLL";
    }
    available_lanes.push_back(new_lane);
  }
  // check availability in each lane for a change to that lane
  int sdc_index = other_cars_traj[0].size() - 1;
  for (int i=0; i<available_lanes.size(); i++) {
    for (int j=0; j<other_cars_traj.size(); j++) {
      double car_s = other_cars_traj[j][sdc_index][0];
      double car_speed = other_cars_traj[j][sdc_index][1];
      for (int k=0; k<other_cars_traj[0].size() - 1; k++) {
        double other_car_s = other_cars_traj[j][k][0];
        double other_car_speed = other_cars_traj[j][k][1];
        int other_car_lane = (int) other_cars_traj[j][k][2]/LANE_WIDTH;
        if (other_car_lane == available_lanes[i].index) {
          if ((other_car_s > car_s) && (other_car_s - car_s < lc_limit)) {
            if (other_car_speed <= car_speed) {
              available_lanes[i].change_possible = false;
            }
            else if (other_car_s - car_s < available_lanes[i].s) {
                available_lanes[i].s = other_car_s - car_s;
              }
          }
          else if ((other_car_s < car_s) && (car_s - other_car_s < lc_limit)) {
            if (other_car_speed >= car_speed) {
              available_lanes[i].change_possible = false;
            }
            else if (car_s - other_car_s < available_lanes[i].s) {
                available_lanes[i].s = car_s - other_car_s;
            }
          }
        }
      }
    }
  }
  // work out which lane to change to if any
  if (available_lanes.size() == 2) {
    if (available_lanes[0].change_possible && available_lanes[1].change_possible) {
      if (available_lanes[0].s < available_lanes[1].s) {
        state = available_lanes[1].direction;
      }
      else {
        state = available_lanes[0].direction;
      }
    }
    else if (available_lanes[0].change_possible) {
      state = available_lanes[0].direction;
    }
    else if (available_lanes[1].change_possible) {
      state = available_lanes[1].direction;
    }
    else {
      state = "slow_down";
    }
  }
  else if (available_lanes[0].change_possible) {
    state = available_lanes[0].direction;
  }
  else {
    state = "slow_down";
  }

  return state;
}

void FSM::accelerate() {
  //return "speed_up";
  double rear_speed = 0.0;
  double forward_speed = 0.0;
  if (rear_car_id != -1) {
    rear_speed = other_cars_traj[0][rear_car_id][1];
    //ref_velocity += sqrt(MAX_ACC);
    ref_velocity += VEL_INC*2;
  }
  if (forward_car_id != -1) {
    forward_speed = other_cars_traj[0][forward_car_id][1];
    if (forward_speed - ref_velocity < -MAX_ACC) {
      //ref_velocity -= sqrt(MAX_ACC);
      ref_velocity -= VEL_INC*2;
    }
    else if (forward_speed - ref_velocity < 0) {
      //ref_velocity -= sqrt(ref_velocity - forward_speed);
      ref_velocity -= VEL_INC*2;
    }
    else if (forward_speed - ref_velocity < MAX_ACC) {
      //ref_velocity += sqrt(forward_speed - ref_velocity);
      ref_velocity += VEL_INC*2;
    }
    else {
      //ref_velocity += sqrt(MAX_ACC);
      ref_velocity += VEL_INC*2;
    }
  }
  if (ref_velocity > SPEED_LIMIT) {
    ref_velocity = SPEED_LIMIT;
  }
}

void FSM::break_hard() {
  //return "slow_down";
  double rear_speed = 0.0;
  double forward_speed = 0.0;
  if (forward_car_id != -1) {
    forward_speed = other_cars_traj[0][forward_car_id][1];
    if (forward_speed - ref_velocity < -MAX_ACC) {
      //ref_velocity -= sqrt(MAX_ACC);
      ref_velocity -= VEL_INC*2;
    }
    else if (forward_speed - ref_velocity < 0) {
      //ref_velocity -= sqrt(forward_speed - ref_velocity);
      ref_velocity -= VEL_INC*2;
    }
  }
  if (ref_velocity > SPEED_LIMIT) {
    ref_velocity = SPEED_LIMIT;
  }
  /*
  if (rear_car_id != -1) {
    rear_speed = other_cars_traj[0][rear_car_id][1];
    if
    ref_velocity += MAX_ACC - 0.1;
  }
  */
}

void FSM::change_lane_left() {
  double d = car[3];
  int car_lane = (int) d/4;
  if (lc.status) {
    if (d<(2+4*lc.target_lane+1) && d>(2+4*lc.target_lane-1)) {
      next_state = "check_collisions";
      lc.status = false;
      lc.target_lane = car_lane;
      lc.start_lane = car_lane;
    }
    else {
      next_state = "CLL";
    }
  }
  else {
    if (car_lane >= 1) {
      lc.start_lane = car_lane;
      lc.target_lane = car_lane - 1;
      lc.status = true;
    }
    else {
      lc.status = false;
      lc.start_lane = car_lane;
      lc.target_lane = car_lane;
    }
  }
  cout << "CLL lane " << lc.target_lane << endl;
}

void FSM::change_lane_right() {
  double d = car[3];
  int car_lane = (int) d/4;
  if (lc.status) {
    if (d<(2+4*lc.target_lane+1) && d>(2+4*lc.target_lane-1)) {
      next_state = "check_collisions";
      lc.status = false;
      lc.target_lane = car_lane;
      lc.start_lane = car_lane;
    }
    else {
      next_state = "CLR";
    }
  }
  else {
    if (car_lane <= 1) {
      lc.start_lane = car_lane;
      lc.target_lane = car_lane + 1;
      lc.status = true;
    }
    else {
      lc.status = false;
      lc.start_lane = car_lane;
      lc.target_lane = car_lane;
    }
  }
  cout << "CLR lane " << lc.target_lane << endl;
}

string FSM::emergency_change_lane() {
  string state = check_lane_available(LC_LIMIT/3);
  //break_hard();
  return state;
}

string FSM::finite_state_machine(vector<double> car, vector<vector<double>> others) {
  this->car = car;
  this->other_cars = others;
  ref_velocity = car[5]/2.24;
  lane = (int) car[3]/4;
  cycle += 1;
  cout << "Cycle " << cycle << "  SPEED_LIMIT " << SPEED_LIMIT << "  car speed "  << ref_velocity << endl;
  cout << "1previous_state " << previous_state << "  next_state " << next_state << endl;
  other_cars_trajectory();
  get_near_cars();


  if (previous_state == " ") {

    next_state = check_speed();
  }
  else if (previous_state == "slow_down" || previous_state == "maintain_speed" || previous_state == "speed_up" || previous_state == "accelerate" || previous_state == "break_hard")  {
    next_state = check_collisions();
  }
  if (next_state == "check_collisions") {
    next_state = check_collisions();
  }
  if (next_state == "check_lane_available") {
    next_state = check_lane_available(LC_LIMIT);
  }
  if (next_state == "emergency_change_lane") {
    next_state = emergency_change_lane();
  }
  if (next_state == "check_speed") {
    next_state = check_speed();
  }

  cout << "2previous_state " << previous_state << "  next_state " << next_state << endl;
  return next_state;
}

x_y_vals FSM::plan_trajectory(vector<double> previous_path_x, vector<double> previous_path_y, vector<double> map_waypoints_s, vector<double> map_waypoints_x, vector<double> map_waypoints_y) {

  // Ajust lane and or speed as required
  cout << "PT ref vel = "<< ref_velocity << endl;
  cout << "PT1 next_state is " << next_state << endl;
  if (next_state == "hello1") {
    previous_state = "slow_down";
  }
  else {
    previous_state = next_state;
    if (next_state == "slow_down") {
      slow_down();
    }
    else if (next_state == "speed_up") {
      speed_up();
    }
    else if (next_state == "maintain_speed") {
      maintain_speed();
    }
    else if (next_state == "accelerate") {
      accelerate();
    }
    else if (next_state == "break_hard") {
      break_hard();
    }
    else if (next_state == "CLL") {
      change_lane_left();
    }
    else if (next_state == "CLR") {
      change_lane_right();
    }
  }
  cout << "PT2 ref vel = "<< ref_velocity << endl;

  x_y_vals next_xy_vals = lane_travel_spline(previous_path_x, previous_path_y, car, map_waypoints_s, map_waypoints_x, map_waypoints_y);
  return next_xy_vals;
}

x_y_vals FSM::lane_travel_spline(vector<double> previous_path_x, vector<double> previous_path_y, vector<double> car, vector<double> map_waypoints_s, vector<double> map_waypoints_x, vector<double> map_waypoints_y) {
  int lane;
  if (lc.status) {
    lane = lc.target_lane;
  }
  else {
    lane = (int) car[3]/4;
  }
  //double SPEED = 48.0;
  //double ref_velocity = SPEED/2.24;
  cout << "spline ref vel " << ref_velocity << endl;
  int prev_size = previous_path_x.size();
  vector<double> ptsx;
  vector<double> ptsy;
  double ref_x = car[0];
  double ref_y = car[1];
  double ref_yaw = deg2rad(car[4]);

  if (prev_size < 2) {
    // use 2 pts that make te path tangent to the car
    double prev_car_x = car[0] - cos(ref_yaw); // what about deg2rad conversion??
    double prev_car_y = car[1] - sin(ref_yaw); // what about deg2rad conversion??
    ptsx.push_back(prev_car_x);
    ptsx.push_back(car[0]);
    ptsy.push_back(prev_car_y);
    ptsy.push_back(car[1]);
    //cout << "car x = " << car[0] << "  y = " << car[1] << "  x-1 = " << prev_car_x << "  y-1 = " << prev_car_y << endl;
  }
  else {
    ref_x = previous_path_x[prev_size -1];
    ref_y = previous_path_y[prev_size -1];
    double ref_x_prev = previous_path_x[prev_size -2];
    double ref_y_prev = previous_path_y[prev_size -2];
    ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);
    //ref_yaw = deg2rad(ref_yaw);
    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);
    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);
    //cout << "ref_x = " << ref_x << "  ref_y = " << ref_y << "  x-1 = " << ref_x_prev << "  y-1 = " << ref_y_prev << endl;
  }

  // In Frenet add evenly spaced points ahead of the starting ref (30, 60, and 90 meters)
  int add = 30;
  cout << "lane = " << lane << endl;
  vector<double> next_wp0 = getXY(car[2] + add, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
  /*
  while (ref_x >= next_wp0[0]) {
    add += 5;
    next_wp0 = getXY(car[2] + add, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
  }
  */
  //vector<double> next_wp0 = getXY(car[2] + add, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
  vector<double> next_wp1 = getXY(car[2] + add+30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
  vector<double> next_wp2 = getXY(car[2] + add+60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);
  ptsy.push_back(next_wp0[1]);
  ptsy.push_back(next_wp1[1]);
  ptsy.push_back(next_wp2[1]);
  //cout << " next wps x" << next_wp0[0] << "  " << next_wp1[0] << "   " << next_wp2[0] << endl;
  //cout << " next wps x" << next_wp0[1] << "  " << next_wp1[1] << "   " << next_wp2[1] << endl;

  // Shift car and rotate car reference angle to 0 degrees
  for (int i=0; i<ptsx.size(); i++) {
    double shift_x = ptsx[i] - ref_x;
    double shift_y = ptsy[i] - ref_y;
    ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw));
    ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw));
    //cout << "t ptsx " << ptsx[i] << endl;
    //cout << "t ptsy" << ptsy[i] << endl;
  }

  // Create a spline
  tk::spline s;

  // set (x,y) points to spline
  s.set_points(ptsx, ptsy);

  // define actual (x, y) points we will use for Planner
  vector<double> next_x_vals;
  vector<double> next_y_vals;

  // start with previous path pts from last run
  //cout << "previous path size " << previous_path_x.size() << endl;
  for(int i=0; i<previous_path_x.size(); i++) {
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }
  // calculate how to break up spline points to that we travel at desired velocity
  double target_x = PATH_LEN;
  double target_y = s(target_x);
  double target_dist = sqrt(target_x * target_x + target_y * target_y);
  double x_add_on = 0.0;
  // fill up the rest of our path planner so that we always output 50 points
  //cout << "ref vel = "<< ref_velocity << endl;
  int N = (int) (target_dist/(0.02 * ref_velocity));
  cout << "N  " << N << endl;
  cout << "nbr path pts - prev path size " << NBR_PATH_PTS<< " - " <<  previous_path_x.size() << endl;
  for (int i=1; i<= NBR_PATH_PTS - previous_path_x.size(); i++) {
    N = (target_dist/(0.02 * ref_velocity));
    double x_point = x_add_on + (target_x/N);
    double y_point = s(x_point);
    x_add_on = x_point;
    //cout << " x_point = " << x_point << " y_point = " << y_point << endl;
    //rotate and shift bak to global map coord system
    double x_ref = x_point;
    double y_ref = y_point;
    x_point = (x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw));
    y_point = (x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw));
    x_point += ref_x;
    y_point += ref_y;
    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
    //cout << "car  x_point = " << car[0] << " y_point = " << car[1] << endl;
    //cout << "2 x_point = " << x_point << " y_point = " << y_point << endl;
  }
  x_y_vals next_xy_vals;
  next_xy_vals.x = next_x_vals;
  next_xy_vals.y = next_y_vals;

  return next_xy_vals;
}

x_y_vals FSM::lane_travel_spline2(vector<double> previous_path_x, vector<double> previous_path_y, vector<double> car, vector<double> map_waypoints_s, vector<double> map_waypoints_x, vector<double> map_waypoints_y) {
  int lane = 1;
  double SPEED = 48.0;
  //double ref_velocity = SPEED/2.24;
  int prev_size = previous_path_x.size();
  vector<double> ptsx;
  vector<double> ptsy;
  double ref_x = car[0];
  double ref_y = car[1];
  double ref_yaw = deg2rad(car[4]);

  if (prev_size < 2) {
    // use 2 pts that make te path tangent to the car
    double prev_car_x = car[0] - cos(ref_yaw); // what about deg2rad conversion??
    double prev_car_y = car[1] - sin(ref_yaw); // what about deg2rad conversion??
    ptsx.push_back(prev_car_x);
    ptsx.push_back(car[0]);
    ptsy.push_back(prev_car_y);
    ptsy.push_back(car[1]);
    //cout << "car x = " << car[0] << "  y = " << car[1] << "  x-1 = " << prev_car_x << "  y-1 = " << prev_car_y << endl;
  }
  else {  // use first two points of previous path
    ref_x = previous_path_x[0];
    ref_y = previous_path_y[0];
    double ref_x_next = previous_path_x[1];
    double ref_y_next = previous_path_y[1];
    ref_yaw = atan2(ref_y_next-ref_y, ref_x_next-ref_x);
    //ref_yaw = deg2rad(ref_yaw);
    ptsx.push_back(ref_x);
    ptsx.push_back(ref_x_next);
    ptsy.push_back(ref_y);
    ptsy.push_back(ref_y_next);
    //cout << "ref_x = " << ref_x << "  ref_y = " << ref_y << "  x-1 = " << ref_x_prev << "  y-1 = " << ref_y_prev << endl;
  }

  // In Frenet add evenly spaced points ahead of the starting ref (30, 60, and 90 meters)
  int add = 30;
  vector<double> next_wp0 = getXY(car[2] + add, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
  /*
  while (ref_x >= next_wp0[0]) {
    add += 5;
    next_wp0 = getXY(car[2] + add, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
  }
  */
  //vector<double> next_wp0 = getXY(car[2] + add, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
  vector<double> next_wp1 = getXY(car[2] + add+30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
  vector<double> next_wp2 = getXY(car[2] + add+60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);
  ptsy.push_back(next_wp0[1]);
  ptsy.push_back(next_wp1[1]);
  ptsy.push_back(next_wp2[1]);
  //cout << " next wps x" << next_wp0[0] << "  " << next_wp1[0] << "   " << next_wp2[0] << endl;
  //cout << " next wps x" << next_wp0[1] << "  " << next_wp1[1] << "   " << next_wp2[1] << endl;

  // Shift car and rotate car reference angle to 0 degrees
  for (int i=0; i<ptsx.size(); i++) {
    double shift_x = ptsx[i] - ref_x;
    double shift_y = ptsy[i] - ref_y;
    ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw));
    ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw));
    //cout << "t ptsx " << ptsx[i] << endl;
    //cout << "t ptsy" << ptsy[i] << endl;
  }

  // Create a spline
  tk::spline s;

  // set (x,y) points to spline
  s.set_points(ptsx, ptsy);

  // define actual (x, y) points we will use for Planner
  vector<double> next_x_vals;
  vector<double> next_y_vals;

  // start with previous path pts from last run
  //cout << "previous path size " << previous_path_x.size() << endl;
  int previous_pts = 0;
  if (previous_path_x.size() >= 2) {
    previous_pts = 2;
  }
  for(int i=0; i<previous_pts; i++) {
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }

  // calculate how to break up spline points to that we travel at desired velocity
  double target_x = 30.0;
  double target_y = s(target_x);
  double target_dist = sqrt(target_x * target_x + target_y * target_y);
  double x_add_on = 0.0;
  // fill up the rest of our path planner so that we always output 50 points
  //cout << "ref vel = "<< ref_velocity << endl;
  int N = (int) (target_dist/(0.02 * ref_velocity));
  cout << "N  " << N << endl;
  for (int i=1; i<= 50 - previous_pts; i++) {
    N = (target_dist/(0.02 * ref_velocity));
    double x_point = x_add_on + (target_x)/N;
    double y_point = s(x_point);
    x_add_on = x_point;
    //cout << " x_point = " << x_point << " y_point = " << y_point << endl;
    //rotate and shift bak to global map coord system
    double x_ref = x_point;
    double y_ref = y_point;
    x_point = (x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw));
    y_point = (x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw));
    x_point += ref_x;
    y_point += ref_y;
    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
    //cout << "car  x_point = " << car[0] << " y_point = " << car[1] << endl;
    //cout << "2 x_point = " << x_point << " y_point = " << y_point << endl;
  }
  x_y_vals next_xy_vals;
  next_xy_vals.x = next_x_vals;
  next_xy_vals.y = next_y_vals;

  return next_xy_vals;
}

x_y_vals FSM::lane_travel1(vector<double> car, vector<double> map_waypoints_s, vector<double> map_waypoints_x, vector<double> map_waypoints_y) {
  double dist_inc = 0.3;
  x_y_vals next_vals;
  for (int i=0; i < 50; i++) {
    double next_s = car[2] + (i+1) * dist_inc;
    double next_d = 6;
    vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    next_vals.x.push_back(xy[0]);
    next_vals.y.push_back(xy[1]);
  }
  return next_vals;
}

string FSM::check_speed() {
  if (car[5] > SPEED_LIMIT) {
    return "slow_down";
  }
  else if (car[5] == SPEED_LIMIT) {
    return "maintain_speed";
  }
  else {
    return "speed_up";
  }
}
