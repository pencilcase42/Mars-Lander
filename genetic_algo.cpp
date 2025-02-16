// genetic_algorithm.cpp

#include "lander.h"
#include <vector>
#include <random>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <numeric>
#include <string>

// External variables from lander.cpp
extern bool genetic;

// Function prototypes for functions from lander.cpp
void reset_simulation();
void initialize_simulation();
void ga_numerical_dynamics(const Autopilot& ap);
void ga_autopilot(const Autopilot& ap);

// Simulation state variables from lander.cpp
extern vector3d position;
extern vector3d velocity;
extern vector3d last_position;
extern double altitude;
extern double fuel;
extern bool crashed, landed;
extern double climb_speed;
extern double delta_t;
extern double ground_speed;
extern double simulation_time;
double descent_velocity;
double surface_velocity;


void genetic_algorithm() {
    // Random number generators
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> param_dis(0.0, PARAMETER_RANGE);
    std::uniform_real_distribution<> parachute_dis(MIN_PARACHUTE_ALTITUDE, MAX_PARACHUTE_ALTITUDE);

    // Initialize population
    std::vector<Autopilot> population(POPULATION_SIZE);
    for (int i = 0; i < POPULATION_SIZE; ++i) {
        population[i].kp = param_dis(gen);
        population[i].kh = param_dis(gen);
        population[i].delta = param_dis(gen);
        population[i].parachute_altitude = parachute_dis(gen);
        population[i].fitness = 0.0;
    }

    // File for average fitness
    std::ofstream fitness_log("fitness_log.txt");
    if (!fitness_log.is_open()) {
        std::cerr << "Unable to open fitness_log.txt for writing." << std::endl;
    }

    for (int generation = 0; generation < GENERATIONS; ++generation) {
        double total_fitness = 0.0;

        for (auto& ap : population) {
            run_simulation(ap);
            total_fitness += ap.fitness;
        }

        // Calculate average fitness
        double average_fitness = total_fitness / POPULATION_SIZE;

        // Find the best autopilot in the current generation
        auto best_ap = std::max_element(population.begin(), population.end(),
                                        [](const Autopilot& a, const Autopilot& b) {
                                            return a.fitness < b.fitness;
                                        });

        // Output progress
        std::cout << "Generation " << generation
                  << " - Best Fitness: " << best_ap->fitness
                  << " - Average Fitness: " << average_fitness
                  << " - kp: " << best_ap->kp
                  << ", kh: " << best_ap->kh
                  << ", delta: " << best_ap->delta
                  << ", parachute_altitude: " << best_ap->parachute_altitude << std::endl;

        // Add average fitness to file
        if (fitness_log.is_open()) {
            fitness_log << generation << " " << average_fitness << std::endl;
        }

        // Create next generation
        std::vector<Autopilot> new_population;
        while (new_population.size() < POPULATION_SIZE) {
            Autopilot parent1 = select_parent(population, gen);
            Autopilot parent2 = select_parent(population, gen);

            // Generate offspring
            Autopilot offspring = crossover(parent1, parent2, gen);

            mutate(offspring, gen);

            new_population.push_back(offspring);
        }

        population = new_population;
    }

    // After last generation, select the best autopilot
    auto best_ap = std::max_element(population.begin(), population.end(),
                                    [](const Autopilot& a, const Autopilot& b) {
                                        return a.fitness < b.fitness;
                                    });

    std::cout << "Best Autopilot Parameters:"
              << " kp: " << best_ap->kp
              << ", kh: " << best_ap->kh
              << ", delta: " << best_ap->delta
              << ", parachute_altitude: " << best_ap->parachute_altitude
              << ", Fitness: " << best_ap->fitness << std::endl;

    // Save the best parameters to a file
    std::ofstream outfile("best_autopilot.txt");
    if (outfile.is_open()) {
        outfile << "Best Fitness: " << best_ap->fitness << std::endl;
        outfile << "kp: " << best_ap->kp << std::endl;
        outfile << "kh: " << best_ap->kh << std::endl;
        outfile << "delta: " << best_ap->delta << std::endl;
        outfile << "parachute_altitude: " << best_ap->parachute_altitude << std::endl;
        outfile.close();
    } else {
        std::cerr << "Unable to open best_autopilot.txt for writing." << std::endl;
    }

    if (fitness_log.is_open()) {
        fitness_log.close();
    }
}

// Function to run the simulation for a given autopilot
void run_simulation(Autopilot& ap) {

    reset_simulation();

    bool simulation_done = false;
    const double SIMULATION_TIME_LIMIT = 20000.0; // Adjust as needed
    std::cout << "kp: " << ap.kp << " kh: " << ap.kh << " delta: " << ap.delta << std::endl;

    // Main simulation loop
    while (!simulation_done && simulation_time < SIMULATION_TIME_LIMIT) {
        update_closeup_coords();
        last_position = position;
        ga_numerical_dynamics(ap);
        update_visualization();

        if (landed || crashed) {
            simulation_done = true;
        }
    }
    // Compute fitness after simulation ends
    ap.fitness = compute_fitness(ap);

}

double compute_fitness(const Autopilot& ap) {

    std::cout << " DV: " << descent_velocity << " fuel: " << fuel << std::endl;
    double fitness = 0.0;
    fitness += (100.0 - std::min(100.0, descent_velocity));

    if (descent_velocity <= 1.0 && surface_velocity <= 1.0 && !crashed) {
        fitness += fuel*FUEL_CAPACITY; // Reward remaining fuel
    }

    return fitness;
}


// Tournament selection
Autopilot select_parent(const std::vector<Autopilot>& population, std::mt19937& gen) {
    std::uniform_int_distribution<> index_dis(0, population.size() - 1);

    Autopilot best_candidate;
    double best_fitness = -std::numeric_limits<double>::infinity();

    for (int i = 0; i < TOURNAMENT_SIZE; ++i) {
        int index = index_dis(gen);
        const Autopilot& candidate = population[index];
        if (candidate.fitness > best_fitness) {
            best_fitness = candidate.fitness;
            best_candidate = candidate;
        }
    }

    return best_candidate;
}


Autopilot crossover(const Autopilot& parent1, const Autopilot& parent2, std::mt19937& gen) {
    Autopilot offspring;
    std::uniform_real_distribution<> dis(0.0, 1.0);
    double alpha = dis(gen); // Random value between 0 and 1

    // Arithmetic crossover
    offspring.kp = alpha * parent1.kp + (1 - alpha) * parent2.kp;
    offspring.kh = alpha * parent1.kh + (1 - alpha) * parent2.kh;
    offspring.delta = alpha * parent1.delta + (1 - alpha) * parent2.delta;
    offspring.parachute_altitude = alpha * parent1.parachute_altitude + (1 - alpha) * parent2.parachute_altitude;
    offspring.fitness = 0.0;

    return offspring;
}

// Mutation function to introduce variability
void mutate(Autopilot& offspring, std::mt19937& gen) {
    std::uniform_real_distribution<> mutation_dis(0.5, 2.0);
    std::uniform_real_distribution<> chance_dis(0.0, 1.0);

    if (chance_dis(gen) < MUTATION_RATE) {
        offspring.kp *= mutation_dis(gen);
        offspring.kp = std::max(offspring.kp, 0.0);

    }
    if (chance_dis(gen) < MUTATION_RATE) {
        offspring.kh *= mutation_dis(gen);
        offspring.kh = std::max(offspring.kh, 0.0);
    }
    if (chance_dis(gen) < MUTATION_RATE) {
        offspring.delta *= mutation_dis(gen);
        offspring.delta = std::max(0.0, offspring.delta);

    }
    if (chance_dis(gen) < MUTATION_RATE) {
        offspring.parachute_altitude *= mutation_dis(gen);
        offspring.parachute_altitude = std::clamp(offspring.parachute_altitude, MIN_PARACHUTE_ALTITUDE, MAX_PARACHUTE_ALTITUDE);
    }
}


// // Parent selection function (roulette wheel selection)
// Autopilot select_parent(const std::vector<Autopilot>& population, std::mt19937& gen) {
//     // Calculate total fitness
//     double total_fitness = 0.0;
//     for (const auto& ap : population) {
//         total_fitness += ap.fitness;
//     }

//     // Generate a random value
//     std::uniform_real_distribution<> dis(0.0, total_fitness);
//     double random_value = dis(gen);

//     // Select parent based on fitness proportion
//     double cumulative_fitness = 0.0;
//     for (const auto& ap : population) {
//         cumulative_fitness += ap.fitness;
//         if (cumulative_fitness >= random_value) {
//             return ap;
//         }
//     }

//     // Fallback (should not reach here)
//     return population.back();
// }

// // Crossover function to create offspring from two parents
// Autopilot crossover(const Autopilot& parent1, const Autopilot& parent2, std::mt19937& gen) {
//     Autopilot offspring;
//     std::uniform_real_distribution<> dis(0.0, 1.0);

//     // Uniform crossover
//     offspring.kp = dis(gen) < 0.5 ? parent1.kp : parent2.kp;
//     offspring.kh = dis(gen) < 0.5 ? parent1.kh : parent2.kh;
//     offspring.delta = dis(gen) < 0.5 ? parent1.delta : parent2.delta;
//     offspring.parachute_altitude = dis(gen) < 0.5 ? parent1.parachute_altitude : parent2.parachute_altitude;
//     offspring.fitness = 0.0;

//     return offspring;
// }
