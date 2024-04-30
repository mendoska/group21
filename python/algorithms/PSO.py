import numpy as np
from enum import Enum
import random 
import math
from icecream import ic

"""
    Use: Runs the Particle Swarm Optimization (PSO) algorithm to finds the best defensive strategy against threats.
    The Particle Swarm Optimization (PSO) algorithm is a population-based optimization algorithm that is inspired by the social behavior of birds flocking or fish schooling.
    Inputs: Reads data such as: [THREAT INFO FORMAT: THREAT NAME, X POS, Y POS, Z POS, MIN RANGE, SPEED, STATUS] 
    Returns: The best solution Ex. [1,2,1,4,3,2,2,4] where 1 represents a Long Range Missile(LRM), 2 represents Medium Range Missile(MRM), 3 represents Short Range Missile(SRM), and 4 represents Directed Energy(DE)
    and its fitness value Ex. 0.75.
    Each answer/solution will be ranked as such: 1 point awarded for each successful kill-> highest scoring solutions will be used as parents
    
"""

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
    """
    Use: Initializes the positions of all particles in the swarm.
    Inputs: None
    Returns: List of particle positions, each position is a list of integers representing weapon choices for each threat.
    """
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
with open('dataFiles/threat_location_original.csv', 'r') as threats:
    inputs = threats.readlines()
    for line in inputs:
        as_list = line.split(',')
        function_inputs.append(as_list[0])
        inputs_position.append([as_list[1], as_list[2], as_list[3], as_list[4]])


def evaluate_fitness(solution):
    """
    Use: Evaluates the fitness of a solution based on the success count of defending against threats.
    Inputs: A list of integers representing weapon choices for each threat.
    Returns: The fitness value of the solution.
    """
    success_count = 0
    weapons_quantity = [8, 20, 25, 10]  
    for threat in range(len(function_inputs)):
        if solution[threat] == 0:
            solution[threat] = 1
        if solution[threat] == 5:
            solution[threat] = 4
        curr_distance = math.sqrt(float(inputs_position[threat][0])**2 + float(inputs_position[threat][1])**2 + float(inputs_position[threat][2])**2)

        solution[threat] = int(solution[threat])
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
            # If threat is within the minimum range, threat will be consider a leaker
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
    """
    Use: Prints the weapons chosen for each threat in the solution.
    Inputs: A list of integers representing weapon choices for each threat.
    Returns: List of the threat index and the chosen weapon(s) for that threat.
    """
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
        # Update personal best
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
print("Global Best Solution:", global_best_position)
print("Global Best Fitness:", global_best_fitness)
