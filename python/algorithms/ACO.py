from enum import Enum
import numpy as np
import math
import time
import random
import matplotlib.pyplot as plt


num_ants = 6
iterations = 100
#Rate at which pheromone evaporates
pheromone_evaporation_rate = 0.1
#Rate at which pheromone is deposited
pheromone_deposition_rate = 0.7
#Initial pheromone level
pheromone_initial = 0.1

#Weapon information
bomber_info = ['bomber', 0.95, 0.85, 0.95, 0.99]
fighter_info = ['fighter', 0.65, 0.90, 0.75, 0.99]
slow_missile_info = ['slow missile', 0.75, 0.95, 0.90, 0.99]
fast_missile_info = ['fast missile', 0.25, 0.35, 0.15, 0.90]

# Defensive assets
class DefensiveAssets(Enum):
    Long_Range_Missile = 1
    Medium_Range_Missile = 2
    Short_Range_Missile = 3
    Directed_Energy = 4

# Function inputs
function_inputs = []
inputs_position = []
with open('dataFiles/threat_location_original.csv', 'r') as threats:
    inputs = threats.readlines()
    for line in inputs:
        as_list = line.split(',')
        function_inputs.append(as_list[0])
        inputs_position.append([as_list[1], as_list[2], as_list[3], as_list[4]])

# Initialize pheromone trails
pheromone_trails = np.full((len(function_inputs), len(DefensiveAssets)), pheromone_initial)

# Define fitness evaluation function
def evaluate_fitness(solution):
    success_count = 0
    weapons_quantity = [8, 20, 25, 10]  
    for threat in range(len(function_inputs)):
        if solution[threat] == 0:
            solution[threat] = 1
        if solution[threat] == 5:
            solution[threat] = 4
        curr_distance = math.sqrt(float(inputs_position[threat][0])**2 + float(inputs_position[threat][1])**2 + float(inputs_position[threat][2])**2)
        if function_inputs[threat] == 'bomber':
# Distance constraint implementation
            check = float(inputs_position[threat][3])
            if curr_distance < float(inputs_position[threat][3]):
                continue
# Weapon quantity constraint implementation
            if weapons_quantity[solution[threat]-1] <= 0:
                continue
# Reload time constraint implementation
            if threat >= 1:
                if solution[threat-1] == solution[threat]:
                    if random.choices([0, 1], [0.9, 0.1])[0] == 1:
                        continue
# Shoot-Look-Shoot implementation
            pk_management = random.choices([0, 1], [1 - bomber_info[int(solution[threat])], bomber_info[int(solution[threat])]])[0]
            if pk_management == 0:
                success_count += random.choices([0, 1], [1 - bomber_info[int(solution[threat])], bomber_info[int(solution[threat])]])[0]
                weapons_quantity[solution[threat] - 1] -= 2
            else:
                success_count += pk_management
                weapons_quantity[solution[threat] - 1] -= 1
            continue

        if function_inputs[threat] == 'fighter':
            if curr_distance < float(inputs_position[threat][3]):
                continue

            if weapons_quantity[solution[threat]-1] <= 0:
                continue

            if threat >= 1:
                if solution[threat-1] == solution[threat]:
                    if random.choices([0, 1], [0.9, 0.1])[0] == 1:
                        continue

            pk_management = random.choices([0, 1], [1-fighter_info[int(solution[threat])], fighter_info[int(solution[threat])]])[0]
            if pk_management == 0:
                success_count += random.choices([0, 1], [1 - fighter_info[int(solution[threat])], fighter_info[int(solution[threat])]])[0]
                weapons_quantity[solution[threat] - 1] -= 2
            else:
                success_count += pk_management
                weapons_quantity[solution[threat] - 1] -= 1
            continue

        if function_inputs[threat] == 'slow missile':
            if curr_distance < float(inputs_position[threat][3]):
                continue

            if weapons_quantity[solution[threat]-1] <= 0:
                continue

            if threat >= 1:
                if solution[threat-1] == solution[threat]:
                    if random.choices([0, 1], [0.9, 0.1])[0] == 1:
                        continue

            pk_management = random.choices([0, 1], [1 - slow_missile_info[int(solution[threat])], slow_missile_info[int(solution[threat])]])[0]
            if pk_management == 0:
                success_count += random.choices([0, 1], [1 - slow_missile_info[int(solution[threat])], slow_missile_info[int(solution[threat])]])[0]
                weapons_quantity[solution[threat] - 1] -= 2
            else:
                success_count += pk_management
                weapons_quantity[solution[threat] - 1] -= 1
            continue

        if function_inputs[threat] == 'fast missile':
            if curr_distance < float(inputs_position[threat][3]):
                continue

            if weapons_quantity[solution[threat]-1] <= 0:
                continue

            if threat >= 1:
                if solution[threat-1] == solution[threat]:
                    if random.choices([0, 1], [0.9, 0.1])[0] == 1:
                        continue

            pk_management = random.choices([0, 1], [1 - fast_missile_info[int(solution[threat])], fast_missile_info[int(solution[threat])]])[0]
            if pk_management == 0:
                success_count += random.choices([0, 1], [1 - fast_missile_info[int(solution[threat])], fast_missile_info[int(solution[threat])]])[0]
                weapons_quantity[solution[threat] - 1] -= 2
            else:
                success_count += pk_management
                weapons_quantity[solution[threat] - 1] -= 1
            continue

    fitness = success_count/len(function_inputs)
    return fitness

# Define ACO functions
def construct_ant_solutions(num_ants, pheromone_trails):
    ant_solutions = []
    for ant in range(num_ants):
        solution = []
        for threat_index in range(len(function_inputs)):
            pheromone_values = pheromone_trails[threat_index]
            probabilities = pheromone_values / np.sum(pheromone_values)
            chosen_defensive_asset = np.random.choice(list(DefensiveAssets), p=probabilities)
            solution.append(chosen_defensive_asset.value)
        ant_solutions.append(solution)
    return ant_solutions

# Update pheromone trails
def update_pheromones(pheromone_trails, ant_solutions, best_solution):
    #Evaporate pheromones
    for i in range(len(function_inputs)):
        for j in range(len(DefensiveAssets)):
            pheromone_trails[i][j] *= (1 - pheromone_evaporation_rate)
            for solution in ant_solutions:
                if solution[i] == j + 1:
                    pheromone_trails[i][j] += pheromone_deposition_rate / evaluate_fitness(solution)
    return pheromone_trails

# Main ACO loop
best_solution = None
best_fitness = 0
fitnesses = []  # to store the best fitness of each iteration
# In each iteration, the ants construct solutions, evaluate them, and update the pheromone trails
for iteration in range(iterations):
    ant_solutions = construct_ant_solutions(num_ants, pheromone_trails)
    for solution in ant_solutions:
        fitness = evaluate_fitness(solution)
        if fitness > best_fitness:
            best_solution = solution
            best_fitness = fitness
    pheromone_trails = update_pheromones(pheromone_trails, ant_solutions, best_solution)
    fitnesses.append(best_fitness)
#print(f"Iteration {iterations}\nBest solution: {best_solution},\nBest fitness: {best_fitness}")
print("Iteration:", iterations)
print("Final best solution found:", best_solution)
print("Final best fitness:", best_fitness)

# Plotting the best fitness of each iteration
# plt.plot(range(iterations), fitnesses)
# plt.title('Ant Colony Optimization')
# plt.xlabel('Iteration')
# plt.ylabel('Best Fitness')
# plt.grid(True)
# plt.show()

