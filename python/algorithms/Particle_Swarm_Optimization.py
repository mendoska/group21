import numpy as np
from enum import Enum
import random 
import math
from icecream import ic


def runPSO(threatFileLocation:str) -> list:
    # Number of particles
    num_particles = 50
    # Number of iterations
    iterations = 100
    c1 = c2 = 1.5
    w = 0.7

    # Weapon information
    bomber_info = ['bomber', 0.95, 0.85, 0.95, 0.99]
    fighter_info = ['fighter', 0.65, 0.90, 0.75, 0.99]
    slow_missile_info = ['slow missile', 0.75, 0.95, 0.90, 0.99]
    fast_missile_info = ['fast missile', 0.25, 0.35, 0.15, 0.90]

    # Initialize particle positions
    def initialize_particles():
        particles = []
        for i in range(num_particles):
            particle = []
            for j in range(len(function_inputs)):
                particle.append(np.random.randint(1, 5))
            particles.append(particle)
        return particles

    # Defensive assets
    class DefensiveAssets(Enum):
        Long_Range_Missile = 1
        Medium_Range_Missile = 2
        Short_Range_Missile = 3
        Directed_Energy = 4

    # Function inputs
    function_inputs = []
    inputs_position = []
    with open(file=threatFileLocation, mode='r') as threats:
        inputs = threats.readlines()
        for line in inputs:
            as_list = line.split(',')
            function_inputs.append(as_list[6].rstrip())
            inputs_position.append([as_list[1], as_list[2], as_list[3], as_list[4]])
            


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
            # ic(function_inputs, function_inputs[threat])
            solution[threat] = int(solution[threat])
            if function_inputs[threat] == 'bomber':
                # ic()
                # Distance constraint implementation
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
        # ic(success_count, len(function_inputs))
        fitness = success_count / len(function_inputs)
        return fitness

    def print_results(solution):
        res = []
        for index, weapon in enumerate(solution):
            if weapon == 1:
                res.append([index, ['Long Range Missile']])
                continue
            if weapon == 2:
                res.append([index, ['Medium Range Missile']])
                continue
            if weapon == 3:
                res.append([index, ['Short Range Missile']])
                continue
            if weapon == 4:
                res.append([index, ['Directed Energy']])
                continue
        return res


                




    # Initialize particles, personal bests, and global best
    swarm_particles = initialize_particles()
    personal_best_positions = swarm_particles.copy()
    global_best_position = None
    global_best_fitness = float('-inf')

    # Iterate over the specified number of iterations
    for _ in range(iterations):
        # Evaluate fitness for each particle and update personal best
        for i, particle in enumerate(swarm_particles):
            fitness = evaluate_fitness(particle)
            if fitness > evaluate_fitness(personal_best_positions[i]):
                personal_best_positions[i] = particle.copy()
            # Update global best if needed
            if fitness > global_best_fitness:
                global_best_position = particle.copy()
                global_best_fitness = fitness
        
        # Update velocities and positions for each particle
        for i, particle in enumerate(swarm_particles):
            for j in range(len(particle)):
                # Update velocity using PSO formula
                particle_velocity = w * particle[j] + c1 * random.random() * (personal_best_positions[i][j] - particle[j]) + c2 * random.random() * (global_best_position[j] - particle[j])
                # Update position
                particle[j] += particle_velocity
                # Ensure particle stays within bounds (1 to 4)
                particle[j] = min(max(1, particle[j]), 4)

    # Print best solution and fitness of the last iteration
    # print("Global Best Solution:", global_best_position)
    # print("Global Best Fitness:", global_best_fitness)
    # ic(print_results(global_best_position))
    return print_results(global_best_position), global_best_fitness

if __name__ == "__main__":
    response, algorithm_leaker_percentage = runPSO(threatFileLocation="dataFiles/simulationThreatLocations.csv")
    ic(response)