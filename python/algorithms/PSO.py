import numpy as np
from enum import Enum
import random 
import math

def runPSO(threatFileLocation):
    # Number of particles
    num_particles = 10
    # Number of iterations
    iterations = 100
    c1 = c2 = 1
    w = 0.5
    
    # Weapon information
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
    with open(threatFileLocation, 'r') as threats:
        inputs = threats.readlines()
        for line in inputs:
            as_list = line.split(',')
            function_inputs.append(as_list[1])
            inputs_position.append([as_list[1], as_list[2], as_list[3], as_list[4]])
            
    def initialize_particles():
        particles = []
        for i in range(num_particles):
            particle = []
            for j in range(len(function_inputs)):
                particle.append(np.random.randint(1, 5))
            particles.append(particle)
        return particles
    
    def update_velocity(particle, velocity, best_particle, best_swarm):
        for i in range(len(particle)):
            velocity[i] = w*velocity[i] + c1*np.random.rand()*(best_particle[i]-particle[i]) + c2*np.random.rand()*(best_swarm[i]-particle[i])
        return velocity
    
    def update_position(particle, velocity):
        for i in range(len(particle)):
            particle[i] = particle[i] + velocity[i]
            if particle[i] < 1:
                particle[i] = 1
            if particle[i] > 4:
                particle[i] = 4
        return particle
    
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
    
    # Main PSO loop
    particles = initialize_particles()
    velocities = [np.random.uniform(-1, 1, len(function_inputs)) for _ in range(num_particles)]
    best_particle = particles.copy()
    best_swarm = particles[np.argmax([evaluate_fitness(p) for p in particles])].copy()
    
    for i in range(iterations):
        for j in range(num_particles):
            fitness = evaluate_fitness(particles[j])
            if fitness > evaluate_fitness(best_particle[j]):
                best_particle[j] = particles[j].copy()
            if fitness > evaluate_fitness(best_swarm):
                best_swarm = particles[j].copy()
            velocities[j] = update_velocity(particles[j], velocities[j], best_particle[j], best_swarm)
            particles[j] = update_position(particles[j], velocities[j])
    
    best_solution = best_swarm
    best_fitness = evaluate_fitness(best_solution)
 
    print("Best Fitness:", best_fitness)
    print("Best Solution:", best_solution)

    return best_solution, best_fitness

best_solution, best_fitness = runPSO("threatFileLocation")
