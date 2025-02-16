// Mars lander simulator
// Version 1.11
// Mechanical simulation functions
// Gabor Csanyi and Andrew Gee, August 2019

// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation, to make use of it
// for non-commercial purposes, provided that (a) its original authorship
// is acknowledged and (b) no modified versions of the source code are
// published. Restriction (b) is designed to protect the integrity of the
// exercise for future generations of students. The authors would be happy
// to receive any suggested modifications by private correspondence to
// ahg@eng.cam.ac.uk and gc121@eng.cam.ac.uk.

#include "lander.h"
vector3d acceleration, drag, current_pos, centrifugal_acceleration;
static vector3d previous_position;
double ro, total_lander_mass, error, pout, h, target, target_altitude, lander_mass, descent_rate;
double semi_major_axis, eccentricity, apoapsis;
vector<double> h_list, error_list, pout_list, vel_list, target_list;
AutopilotState autopilot_state = ORBITAL_FLIGHT;
ofstream fout;

bool genetic = false; // Set to true to run GA
// Global variables
double desired_periapsis = MARS_RADIUS + 50000.0; // Target periapsis altitude (50 km)

// THE AUTOPILOT WAS NOT ON! - 10km descent - 60 generations, and scaling for mutations was 0.15  to 1.5
// double kh = 0.150495;
// double kp = 1.73515;
// double delta = 0.0982715;
// double parachute_altitude = 1857.57;

//manually tuned
double kh = 0.014;
double kp = 0.5;
double delta = 0.2;
double parachute_altitude = 10000.0;

void ga_autopilot (const Autopilot& ap)
  // Autopilot to adjust the engine throttle, parachute and attitude control
{
    h = position.abs() - MARS_RADIUS;
    error = -(0.5 + ap.kh * h - velocity.abs()*1*cos(stabilized_attitude_angle*M_PI/180));
    pout = ap.kp*error;
    target = -(0.5 + ap.kh*h);

    if (pout <= -ap.delta){
        throttle = 0;
    } else if ((-ap.delta < pout) && (pout < 1 - ap.delta)){
        throttle = ap.delta + pout;
    } else if (pout >= 1 - ap.delta){
        throttle = 1;
    }

    
    // Parachute deployment conditions
    if (parachute_status == NOT_DEPLOYED &&
        h < ap.parachute_altitude && 
        safe_to_deploy_parachute())
    {
        parachute_status = DEPLOYED;
    }
   
    // // Write the trajectories to file
    // fout.open("info.txt", ofstream::app);
    // if (fout) { // file opened successfully
        
    //     fout << h << ' ' << -velocity.y << ' ' << target << endl;
        
    // } else { // file did not open successfully
    //     cout << "Could not open info file for writing" << endl;
    // }
    // fout.close();
    
}


void autopilot() {   

  if (ql_ap){
    q_learning_autopilot();
  }

  h = position.abs() - MARS_RADIUS;

  // Radial velocity (component of velocity along the radius vector)
  double radial_velocity = velocity * position.norm();

  switch (autopilot_state) {

    case ORBITAL_FLIGHT:
        if (autopilot_enabled) {
            if (eccentricity < 0.01) {
                // Orbit is nearly circular
                // Proceed to deorbit burn immediately
                autopilot_state = DEORBIT_BURN;
            } else {
                // Orbit is elliptical
                // Wait for apoapsis
                autopilot_state = WAIT_FOR_APOAPSIS;
            }
        }
        break;

    case WAIT_FOR_APOAPSIS:
        // Check if near apoapsis
        if (radial_velocity > -0.1 && radial_velocity < 0.1) {
            // Near apoapsis (radial_velocity ~ 0)
            autopilot_state = DEORBIT_BURN;
        }
        break;


  case DEORBIT_BURN:
      // Orient retrograde
      stabilized_attitude = true;
      stabilized_attitude_angle = 180.0; // Point opposite to velocity vector

      // Apply maximum thrust retrograde
      throttle = 1.0;

      // Continue burn until periapsis is at desired altitude
      {
          vector3d h_vec = position ^ velocity; // Specific angular momentum vector
          double mu = GRAVITY * MARS_MASS;
          double energy = velocity.abs2() / 2 - mu / position.abs();
          semi_major_axis = -mu / (2 * energy);
          eccentricity = sqrt(1 + (2 * energy * h_vec.abs2()) / (mu * mu));
          double periapsis = semi_major_axis * (1 - eccentricity);

          if (periapsis <= desired_periapsis) {
              // Stop burn
              throttle = 0.0;
              autopilot_state = ATMOSPHERIC_ENTRY;
          }
      }
      break;

  case ATMOSPHERIC_ENTRY:
      // Orient for maximum drag
      stabilized_attitude = true;
      stabilized_attitude_angle = 0.0; // Point forward (blunt end into airflow)

      // Use atmospheric drag to slow down
      // Deploy parachute when safe
      if (parachute_status == NOT_DEPLOYED &&
          h < 30000 &&
          safe_to_deploy_parachute())
      {
          parachute_status = DEPLOYED;
      }

      // Transition to descent phase when vertical speed is significant
      if (velocity.abs() < 100.0) {
          autopilot_state = DESCENT;
      }
      break;

  case DESCENT:
    error = -(0.5 + kh * h - velocity.abs()*1*cos(stabilized_attitude_angle*M_PI/180));
    pout = kp*error;
    target = -(0.5 + kh*h);

    if (pout <= -delta){
        throttle = 0;
    } else if ((-delta < pout) && (pout < 1 - delta)){
        throttle = delta + pout;
    } else if (pout >= 1 - delta){
        throttle = 1;
    }
        // Parachute deployment conditions
    if (parachute_status == NOT_DEPLOYED &&
        h < parachute_altitude &&
        safe_to_deploy_parachute())
    {
        parachute_status = DEPLOYED;
    }
    break;
  }

  // // Write the trajectories to file
  // fout.open("info.txt", ofstream::app);
  // if (fout) { // file opened successfully
      
  //     fout << h << ' ' << -velocity.y << ' ' << target << endl;
      
  // } else { // file did not open successfully
  //     cout << "Could not open info file for writing" << endl;
  // }
  // fout.close();
}



void euler (vector3d acceleration){
    position = position + velocity * delta_t;
    velocity = velocity + acceleration * delta_t;
}

void verlet(vector3d acceleration){
    if (simulation_time == 0) {
        previous_position = position;
        euler(acceleration);
    } else {
        current_pos = position;
        position = 2*position - previous_position + pow(delta_t, 2)*acceleration;
        velocity = (position - current_pos)/delta_t;
        previous_position = current_pos;
    }
}


void numerical_dynamics ()
  // This is the function that performs the numerical integration to update the
  // lander's pose. The time step is delta_t (global variable).
{
    ro = atmospheric_density(position);
    drag = -0.5*ro*DRAG_COEF_LANDER*M_PI*pow(LANDER_SIZE,2)*velocity.abs2()*velocity.norm();
    if (parachute_status == 1){
        drag = drag - 0.5*ro*DRAG_COEF_CHUTE*5*pow(2*LANDER_SIZE,2)*velocity.abs2()*velocity.norm();
    }
    
    total_lander_mass = UNLOADED_LANDER_MASS + fuel*FUEL_DENSITY*FUEL_CAPACITY;
    
    acceleration = -(GRAVITY*MARS_MASS/position.abs2())*position.norm() + (drag + thrust_wrt_world())/total_lander_mass;
    
    verlet(acceleration);

    if (autopilot_enabled) {
        if (q_learning) {
            q_learning_autopilot();
        } else {
            autopilot();
        }
    }

  // Here we can apply 3-axis stabilization to ensure the base is always pointing downwards
    if (stabilized_attitude) attitude_stabilization();
    
}



void ga_numerical_dynamics (Autopilot& ap)
{
    ro = atmospheric_density(position);
    drag = -0.5*ro*DRAG_COEF_LANDER*M_PI*pow(LANDER_SIZE,2)*velocity.abs2()*velocity.norm();
    if (parachute_status == 1){
        drag = drag - 0.5*ro*DRAG_COEF_CHUTE*5*pow(2*LANDER_SIZE,2)*velocity.abs2()*velocity.norm();
    }
    
    total_lander_mass = UNLOADED_LANDER_MASS + fuel*FUEL_DENSITY*FUEL_CAPACITY;
    
    acceleration = -(GRAVITY*MARS_MASS/position.abs2())*position.norm() + (drag + thrust_wrt_world())/total_lander_mass;
    
    verlet(acceleration);
  // Here we can apply an autopilot to adjust the thrust, parachute and attitude
    if (autopilot_enabled) ga_autopilot(ap);

  // Here we can apply 3-axis stabilization to ensure the base is always pointing downwards
    if (stabilized_attitude) attitude_stabilization();
    
}

void initialize_simulation (void)
  // Lander pose initialization - selects one of 10 possible scenarios
{
  ofstream fout("info.txt", ofstream::trunc);
  // The parameters to set are:
  // position - in Cartesian planetary coordinate system (m)
  // velocity - in Cartesian planetary coordinate system (m/s)
  // orientation - in lander coordinate system (xyz Euler angles, degrees)
  // delta_t - the simulation time step
  // boolean state variables - parachute_status, stabilized_attitude, autopilot_enabled
  // scenario_description - a descriptive string for the help screen

  scenario_description[0] = "circular orbit";
  scenario_description[1] = "descent from 10km";
  scenario_description[2] = "elliptical orbit, thrust changes orbital plane";
  scenario_description[3] = "polar launch at escape velocity (but drag prevents escape)";
  scenario_description[4] = "elliptical orbit that clips the atmosphere and decays";
  scenario_description[5] = "descent from 200km";
  scenario_description[6] = "areostationary orbit";
  scenario_description[7] = "altitude = 500";
  scenario_description[8] = "altitude = 510";
  scenario_description[9] = "altitude = 700";

  switch (scenario) {

  case 0:
    // a circular equatorial orbit
    position = vector3d(1.2*MARS_RADIUS, 0.0, 0.0);
    velocity = vector3d(0.0, -3247.087385863725, 0.0);
    orientation = vector3d(0.0, 90.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    autopilot_state = ORBITAL_FLIGHT;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 1:
    // a descent from rest at 10km altitude
    position = vector3d(0.0, -(MARS_RADIUS + 10000.0), 0.0);
    velocity = vector3d(0.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    autopilot_state = DESCENT;
    stabilized_attitude = true;
    autopilot_enabled = true;
    break;

  case 2:
    // an elliptical polar orbit
    position = vector3d(0.0, 0.0, 1.2*MARS_RADIUS);
    velocity = vector3d(3300.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    autopilot_state = ORBITAL_FLIGHT;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 3:
    // polar surface launch at escape velocity (but drag prevents escape)
    position = vector3d(0.0, 0.0, MARS_RADIUS + LANDER_SIZE/2.0);
    velocity = vector3d(0.0, 0.0, 5027.0);
    orientation = vector3d(0.0, 0.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 4:
    // an elliptical orbit that clips the atmosphere each time round, losing energy
    position = vector3d(0.0, 0.0, MARS_RADIUS + 100000.0);
    velocity = vector3d(4000.0, 0.0, 0.0);
    orientation = vector3d(0.0, 90.0, 0.0);
    delta_t = 0.1;
    autopilot_state = ORBITAL_FLIGHT;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 5:
    // a descent from rest at the edge of the exosphere
    position = vector3d(0.0, -(MARS_RADIUS + EXOSPHERE), 0.0);
    velocity = vector3d(0.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    autopilot_state = DESCENT;
    stabilized_attitude = true;
    autopilot_enabled = true;
    break;

  case 6:
    position = vector3d(cbrt(GRAVITY*MARS_MASS*MARS_DAY*MARS_DAY/(4*M_PI*M_PI)), 0.0, 0.0);
    velocity = vector3d(0.0, 2*M_PI*cbrt(GRAVITY*MARS_MASS*MARS_DAY*MARS_DAY/(4*M_PI*M_PI))/MARS_DAY, 0.0);
    orientation = vector3d(0.0, 90.0, 0.0);
    delta_t = 0.1;
    autopilot_state = ORBITAL_FLIGHT;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = true;
    break;

  case 7:
      position = vector3d(0.0, 500 + MARS_RADIUS, 0.0);
      velocity = vector3d(0.0, 0.0, 0.0);
      orientation = vector3d(0.0, 0.0, 90.0);
      delta_t = 0.01;
      parachute_status = NOT_DEPLOYED;
      stabilized_attitude = true;
      autopilot_enabled = true;
    break;

  case 8:
      position = vector3d(0.0, 510 + MARS_RADIUS, 0.0);
      velocity = vector3d(0.0, 0.0, 0.0);
      orientation = vector3d(0.0, 0.0, 90.0);
      delta_t = 0.01;
      parachute_status = NOT_DEPLOYED;
      stabilized_attitude = true;
      autopilot_enabled = true;
    break;

  case 9:
      position = vector3d(0.0, 700 + MARS_RADIUS, 0.0);
      velocity = vector3d(0.0, 0.0, 0.0);
      orientation = vector3d(0.0, 0.0, 90.0);
      delta_t = 0.01;
      parachute_status = NOT_DEPLOYED;
      stabilized_attitude = true;
      autopilot_enabled = true;
    break;

  }
}



//manually tuned
// double kh = 0.0158;
// double kp = 0.5;
// double delta = 0.2;
// double parachute_altitude = 10000.0;


// first test of ga
// double kp = 1.67536;
// double kh = 16.4698;
// double delta = 5.5632;

//second test of ga (changed the parachute altitudes min and max to 1000 and 10000)
// double kh = 0.873522;
// double kp = 0.657421;
// double delta = 6.96024;

//made mistake in ga_autopilot had h > parachute_altiude rather than <

//third test (still having issue that average fitness is constant)
// double kh = 3.04081;
// double kp = 0.686574;
// double delta = 1.78979;

//fourth test (removed crash subtract of fitness, and fixed fuel to fuel * fuel capacity. Also removed the multiplier of 1000 for that)
// double kh = 5.50007;
// double kp = 1.33305;
// double delta = 0.347755;
// double parachute_altitude = 5075.15;

//fifth test, the lander is actually reaching the ground now, but for some reason it is landing in all scenarios??
// double kh = 3.81512;
// double kp = 5.44889;
// double delta = 1.12245;
// double parachute_altitude = 10000;


    // //stabilisation stuff
    // double kp = 0.05; double kd = 0.04;
    // target_altitude = 0.5;
    // h = position.abs() - MARS_RADIUS;
    // lander_mass = UNLOADED_LANDER_MASS + fuel*FUEL_CAPACITY*FUEL_DENSITY;
    // delta = (GRAVITY*MARS_MASS*lander_mass/position.abs2())/ MAX_THRUST;
    // descent_rate = -velocity*position.norm();
    // throttle = delta - kp*(h - target_altitude) + kd*descent_rate;

    // if (throttle > 1.0) throttle = 1.0; if (throttle < 0.0) throttle = 0.0;

  
// 30 Generations, scaling for mutation was 0.5 to 2
// double kh = 0.15226;
// double kp = 4.68315;
// double delta = 1.33385;
// double parachute_altitude = 2852.31;
