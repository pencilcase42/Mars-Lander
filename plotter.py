import matplotlib.pyplot as plt

def plot_average_fitness(file_path):
    # Initialize lists to store data
    generations = []
    fitness_values = []
    
    # Read the file
    with open(file_path, 'r') as file:
        for line in file:
            # Split each line into generation and fitness
            generation, fitness = map(float, line.split())
            generations.append(int(generation))
            fitness_values.append(fitness)
    
    # Plot the data
    plt.figure(figsize=(10, 6))
    plt.plot(generations, fitness_values, marker='o', linestyle='-', color='b')
    plt.title("Average Fitness")
    plt.xlabel("Generation")
    plt.ylabel("Average Fitness")
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.show()

plot_average_fitness("fitness_log.txt")