// // q_learning.h

// #ifndef Q_LEARNING_H
// #define Q_LEARNING_H

// #include <vector>
// #include <unordered_map>
// #include <utility>
// #include <cmath>
// #include <cstdlib>
// #include <ctime>
// #include <fstream>

// // Include necessary headers
// #include "lander.h"

// // Constants for discretization
// #define ALTITUDE_BINS 20
// #define VELOCITY_BINS 18
// #define FUEL_BINS 20
// #define PARACHUTE_STATES 2

// // Constants for actions
// #define THROTTLE_LEVELS 5
// #define PARACHUTE_ACTIONS 2

// // Q-Learning parameters
// #define ALPHA 0.1          // Learning rate
// #define GAMMA 0.99         // Discount factor
// #define EPSILON_START 1.0  // Initial exploration rate
// #define EPSILON_MIN 0.01   // Minimum exploration rate
// #define EPSILON_DECAY 0.995 // Exploration decay rate
// #define NUM_EPISODES 5000  // Number of training episodes

// // Reward values
// #define REWARD_SUCCESS 1000
// #define REWARD_CRASH -1000
// #define REWARD_STEP -1

// // Data structures
// struct State {
//     int altitude_bin;
//     int velocity_bin;
//     int fuel_bin;
//     int parachute_status;

//     bool operator==(const State& other) const {
//         return (altitude_bin == other.altitude_bin &&
//                 velocity_bin == other.velocity_bin &&
//                 fuel_bin == other.fuel_bin &&
//                 parachute_status == other.parachute_status);
//     }
// };

// struct StateHash {
//     std::size_t operator()(const State& s) const {
//         return ((std::hash<int>()(s.altitude_bin) ^
//                 (std::hash<int>()(s.velocity_bin) << 1)) >> 1) ^
//                 (std::hash<int>()(s.fuel_bin) << 1) ^
//                 (std::hash<int>()(s.parachute_status) << 1);
//     }
// };

// // Function declarations
// void initialize_q_learning();
// void q_learning_episode();
// void q_learning_autopilot();
// int select_action(const State& state);
// void update_q_table(const State& state, int action, double reward, const State& next_state);
// int discretize_altitude(double altitude);
// int discretize_velocity(double velocity);
// int discretize_fuel(double fuel_percentage);
// void save_q_table(const std::string& filename);
// void load_q_table(const std::string& filename);

// extern bool q_learning; // Global flag to enable/disable Q-learning

// #endif // Q_LEARNING_H
