// q_learning.cpp
#include "lander.h"
extern bool crashed, landed;
extern vector3d last_position;
extern vector3d position;
extern vector3d velocity;
extern double altitude;
extern double fuel;
extern double climb_speed;
extern double delta_t;
extern double ground_speed;
extern double simulation_time;
extern double descent_velocity;
extern double surface_velocity;


// Q-table: maps State-Action pairs to Q-values
std::unordered_map<StateActionPair, double, StateActionHash> Q_table;

// Exploration rate
double epsilon = EPSILON_START;

bool q_learning = false;
bool ql_ap = false;

std::mt19937 rng(std::time(nullptr));

void initialize_q_learning() {
    epsilon = EPSILON_START;
    Q_table.clear();
}

void q_learning_episode() {

    reset_simulation();

    bool done = false;
    double total_reward = 0.0;

    while (!done) {
        // Discretise current state
        State current_state;
        current_state.altitude_bin = discretise_altitude(position.abs() - MARS_RADIUS);
        current_state.velocity_bin = discretise_velocity(velocity * position.norm());
        current_state.fuel_bin = discretise_fuel(fuel * 100.0 / FUEL_CAPACITY);
        current_state.parachute_status = parachute_status == DEPLOYED ? 1 : 0;

        // Select action using epsilon-greedy policy
        int action = select_action(current_state);

        // Apply action
        // Decode action into throttle setting and parachute deployment
        int throttle_action = action / PARACHUTE_ACTIONS;
        int parachute_action = action % PARACHUTE_ACTIONS;

        throttle = throttle_action / double(THROTTLE_LEVELS - 1); // Map to [0.0, 1.0]

        // Deploy parachute if action dictates and parachute not already deployed
        if (parachute_action == 1 && parachute_status == NOT_DEPLOYED && safe_to_deploy_parachute()) {
            parachute_status = DEPLOYED;
        }

        update_closeup_coords();
        last_position = position;
        numerical_dynamics();
        update_visualization();

        // Calculate reward
        double reward = REWARD_STEP - (FUEL_RATE_AT_MAX_THRUST * throttle * delta_t * 10);

        // Check for terminal conditions
        if (landed) {
            if (!crashed) {
                reward += REWARD_SUCCESS;
            } else {
                reward += REWARD_CRASH;
            }
            done = true;
        }

        // Discretize next state
        State next_state;
        next_state.altitude_bin = discretise_altitude(position.abs() - MARS_RADIUS);
        next_state.velocity_bin = discretise_velocity(velocity * position.norm());
        next_state.fuel_bin = discretise_fuel(fuel * 100.0 / FUEL_CAPACITY);
        next_state.parachute_status = parachute_status == DEPLOYED ? 1 : 0;

        update_q_table(current_state, action, reward, next_state);

        total_reward += reward;

        // Reduce epsilon
        if (done) {
            epsilon = std::max(epsilon * EPSILON_DECAY, EPSILON_MIN);
        }
    }

    std::cout << "Episode total reward: " << total_reward << std::endl;
}


int select_action(const State& state) {
    std::uniform_real_distribution<double> dist(0.0, 1.0);
    if (dist(rng) < epsilon) {
        // Exploration
        return std::uniform_int_distribution<int>(0, THROTTLE_LEVELS * PARACHUTE_ACTIONS - 1)(rng);
    } else {
        // Exploitation
        double max_q = -std::numeric_limits<double>::infinity();
        int best_action = 0;
        for (int a = 0; a < THROTTLE_LEVELS * PARACHUTE_ACTIONS; ++a) {
            StateActionPair sap = {state, a};
            auto it = Q_table.find(sap);
            double q_value = (it != Q_table.end()) ? it->second : 0.0;
            if (q_value > max_q) {
                max_q = q_value;
                best_action = a;
            }
        }
        return best_action;
    }
}


void update_q_table(const State& state, int action, double reward, const State& next_state) {
    StateActionPair sap = {state, action};
    double q_current = Q_table[sap];

    // Find maximum Q-value for next state
    double max_q_next = -std::numeric_limits<double>::infinity();
    for (int a = 0; a < THROTTLE_LEVELS * PARACHUTE_ACTIONS; ++a) {
        StateActionPair next_sap = {next_state, a};
        auto it = Q_table.find(next_sap);
        double q_value = (it != Q_table.end()) ? it->second : 0.0;
        if (q_value > max_q_next) {
            max_q_next = q_value;
        }
    }

    // Q-learning update rule
    q_current += ALPHA * (reward + GAMMA * max_q_next - q_current);

    Q_table[sap] = q_current;
}


int discretise_altitude(double altitude) {
    int bin = static_cast<int>(altitude / 500.0);
    return std::min(bin, ALTITUDE_BINS - 1);
}

int discretise_velocity(double velocity) {
    // Negative sign to make descending velocities positive
    double descending_velocity = -velocity;
    int bin = static_cast<int>((descending_velocity - 100.0) / 50.0);
    return std::clamp(bin, 0, VELOCITY_BINS - 1);
}

int discretise_fuel(double fuel_percentage) {
    int bin = static_cast<int>(fuel_percentage / 5.0);
    return std::clamp(bin, 0, FUEL_BINS - 1);
}



void save_q_table(const std::string& filename) {
    std::ofstream file(filename, std::ios::binary);
    for (const auto& entry : Q_table) {
        const StateActionPair& sap = entry.first;
        double q_value = entry.second;

        // Write State
        file.write(reinterpret_cast<const char*>(&sap.state), sizeof(State));
        // Write Action
        file.write(reinterpret_cast<const char*>(&sap.action), sizeof(int));
        // Write Q-value
        file.write(reinterpret_cast<const char*>(&q_value), sizeof(double));
    }
    file.close();
}


void load_q_table(const std::string& filename) {
    std::ifstream file(filename, std::ios::binary);
    if (!file) {
        std::cerr << "Failed to open Q-table file: " << filename << std::endl;
        return;
    }
    Q_table.clear();
    while (file.peek() != EOF) {
        StateActionPair sap;
        double q_value;

        // Read State
        file.read(reinterpret_cast<char*>(&sap.state), sizeof(State));
        // Read Action
        file.read(reinterpret_cast<char*>(&sap.action), sizeof(int));
        // Read Q-value
        file.read(reinterpret_cast<char*>(&q_value), sizeof(double));

        Q_table[sap] = q_value;
    }
    file.close();
}


// Autopilot function using the learned policy
void q_learning_autopilot() {
    // Discretize current state
    State current_state;
    current_state.altitude_bin = discretise_altitude(position.abs() - MARS_RADIUS);
    current_state.velocity_bin = discretise_velocity(velocity * position.norm());
    current_state.fuel_bin = discretise_fuel(fuel * 100.0 / FUEL_CAPACITY);
    current_state.parachute_status = parachute_status == DEPLOYED ? 1 : 0;

    // Select the best action
    double max_q = -std::numeric_limits<double>::infinity();
    int best_action = 0;
    for (int a = 0; a < THROTTLE_LEVELS * PARACHUTE_ACTIONS; ++a) {
        StateActionPair sap = {current_state, a};
        auto it = Q_table.find(sap);
        double q_value = (it != Q_table.end()) ? it->second : 0.0;
        if (q_value > max_q) {
            max_q = q_value;
            best_action = a;
        }
    }

    // Decode action into throttle setting and parachute deployment
    int throttle_action = best_action / PARACHUTE_ACTIONS;
    int parachute_action = best_action % PARACHUTE_ACTIONS;

    // Set throttle to between 0 and 1
    throttle = throttle_action / double(THROTTLE_LEVELS - 1);

    // Deploy parachute if action dictates and parachute not already deployed
    if (parachute_action == 1 && parachute_status == NOT_DEPLOYED && safe_to_deploy_parachute()) {
        parachute_status = DEPLOYED;
    }
}



// void q_learning_autopilot() {
//     // Discretize current state
//     State current_state;
//     current_state.altitude_bin = discretise_altitude(position.abs() - MARS_RADIUS);
//     current_state.velocity_bin = discretise_velocity(velocity * position.norm());
//     current_state.fuel_bin = discretise_fuel(fuel * 100.0 / FUEL_CAPACITY);
//     current_state.parachute_status = parachute_status == DEPLOYED ? 1 : 0;

//     // Select the best action
//     double max_q = -std::numeric_limits<double>::infinity();
//     int best_action = 0;
//     for (int a = 0; a < THROTTLE_LEVELS * PARACHUTE_ACTIONS; ++a) {
//         auto it = Q_table.find({current_state, a});
//         double q_value = (it != Q_table.end()) ? it->second : 0.0;
//         if (q_value > max_q) {
//             max_q = q_value;
//             best_action = a;
//         }
//     }

//     // Decode action into throttle setting and parachute deployment
//     int throttle_action = best_action / PARACHUTE_ACTIONS;
//     int parachute_action = best_action % PARACHUTE_ACTIONS;

//     // Set throttle
//     throttle = throttle_action / double(THROTTLE_LEVELS - 1); // Map to [0.0, 1.0]

//     // Deploy parachute if action dictates and parachute not already deployed
//     if (parachute_action == 1 && parachute_status == NOT_DEPLOYED && safe_to_deploy_parachute()) {
//         parachute_status = DEPLOYED;
//     }
// }


// Function to update the Q-table
// void update_q_table(const State& state, int action, double reward, const State& next_state) {
//     // Get current Q-value
//     double q_current = Q_table[{state, action}];

//     // Find maximum Q-value for next state
//     double max_q_next = -std::numeric_limits<double>::infinity();
//     for (int a = 0; a < THROTTLE_LEVELS * PARACHUTE_ACTIONS; ++a) {
//         auto it = Q_table.find({next_state, a});
//         double q_value = (it != Q_table.end()) ? it->second : 0.0;
//         if (q_value > max_q_next) {
//             max_q_next = q_value;
//         }
//     }

//     // Q-learning update rule
//     q_current += ALPHA * (reward + GAMMA * max_q_next - q_current);

//     // Update Q-table
//     Q_table[{state, action}] = q_current;
// }



// Function to load the Q-table from a file
// void load_q_table(const std::string& filename) {
//     std::ifstream file(filename, std::ios::binary);
//     if (!file) {
//         std::cerr << "Failed to open Q-table file: " << filename << std::endl;
//         return;
//     }
//     Q_table.clear();
//     while (file.peek() != EOF) {
//         State state;
//         int action;
//         double q_value;
//         file.read(reinterpret_cast<char*>(&state), sizeof(State));
//         file.read(reinterpret_cast<char*>(&action), sizeof(int));
//         file.read(reinterpret_cast<char*>(&q_value), sizeof(double));
//         Q_table[{state, action}] = q_value;
//     }
//     file.close();
// }


// Function to save the Q-table to a file
// void save_q_table(const std::string& filename) {
//     std::ofstream file(filename, std::ios::binary);
//     for (const auto& entry : Q_table) {
//         const State& state = entry.first.first;
//         int action = entry.first.second;
//         double q_value = entry.second;
//         file.write(reinterpret_cast<const char*>(&state), sizeof(State));
//         file.write(reinterpret_cast<const char*>(&action), sizeof(int));
//         file.write(reinterpret_cast<const char*>(&q_value), sizeof(double));
//     }
//     file.close();
// }


// Function to select an action using epsilon-greedy policy
// int select_action(const State& state) {
//     std::uniform_real_distribution<double> dist(0.0, 1.0);
//     if (dist(rng) < epsilon) {
//         // Exploration: choose a random action
//         return std::uniform_int_distribution<int>(0, THROTTLE_LEVELS * PARACHUTE_ACTIONS - 1)(rng);
//     } else {
//         // Exploitation: choose the best action
//         double max_q = -std::numeric_limits<double>::infinity();
//         int best_action = 0;
//         for (int a = 0; a < THROTTLE_LEVELS * PARACHUTE_ACTIONS; ++a) {
//             auto it = Q_table.find({state, a});
//             double q_value = (it != Q_table.end()) ? it->second : 0.0;
//             if (q_value > max_q) {
//                 max_q = q_value;
//                 best_action = a;
//             }
//         }
//         return best_action;
//     }
// }
