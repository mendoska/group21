from enum import Enum
import numpy as np
from random import choices
import math
import pygad
import time


def runGA(threatFileLocation):

    bomber_info = ['bomber', 0.95, 0.85, 0.95, 0.99]
    fighter_info = ['fighter', 0.65, 0.90, 0.75, 0.99]
    slow_missile_info = ['slow missile', 0.75, 0.95, 0.90, 0.99]
    fast_missile_info = ['fast missile', 0.25, 0.35, 0.15, 0.90]


    class DefensiveAssets(Enum):
        Long_Range_Missile = 1
        Medium_Range_Missile = 2
        Short_Range_Missile = 3
        Directed_Energy = 4


    """ example of one function input is as such:[THREAT INFO FORMAT: THREAT NAME, X POS, Y POS, Z POS, MIN RANGE, SPEED, STATUS] 
    and an example solution looks like [1,2,1,4,3,2,2,4] where 1 represents a Long Range Missile(LRM), 2 represents Medium Range Missile(MRM), 
    3 represents Short Range Missile(SRM), and 4 represents Directed Energy(DE)

    Each answer/solution will be ranked as such: 1 point awarded for each successful kill-> highest scoring solutions will be used as parents
    """
    function_inputs = []
    inputs_position = []
    with open(threatFileLocation, 'r') as threats:
        inputs = threats.readlines()
        for line in inputs:
            as_list = line.split(',')
            function_inputs.append(as_list[0])
            inputs_position.append([as_list[1], as_list[2], as_list[3], as_list[4]])


    def fitness_func(ga_instance, solution, solution_idx):
        success_count = 0
        weapons_quantity = [8, 20, 25, 10]
        #Initialize last fire times for each weapon
        last_fire_times = [0] * len(weapons_quantity)
        #Cooldown time in seconds
        cooldown_time = 5
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
    #Cooldown system implementation 
    #Get current time 
                current_time = time.time()
    #Chech if the cooldown time has passed 
                if current_time < last_fire_times[solution[threat]-1] + cooldown_time:
                    continue
                else:
    #Update the last fire time for the weapon used
                    last_fire_times[solution[threat]-1] = current_time
    # Weapon quantity constraint implementation
                if weapons_quantity[solution[threat]-1] <= 0:
                    continue
    # Reload time constraint implementation
                if threat >= 1:
                    if solution[threat-1] == solution[threat]:
                        if choices([0, 1], [0.9, 0.1])[0] == 1:
                            continue
    # Shoot-Look-Shoot implementation
                pk_management = choices([0, 1], [1 - bomber_info[int(solution[threat])], bomber_info[int(solution[threat])]])[0]
                if pk_management == 0:
                    success_count += choices([0, 1], [1 - bomber_info[int(solution[threat])], bomber_info[int(solution[threat])]])[0]
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
                        if choices([0, 1], [0.9, 0.1])[0] == 1:
                            continue

                pk_management = choices([0, 1], [1-fighter_info[int(solution[threat])], fighter_info[int(solution[threat])]])[0]
                if pk_management == 0:
                    success_count += choices([0, 1], [1 - fighter_info[int(solution[threat])], fighter_info[int(solution[threat])]])[0]
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
                        if choices([0, 1], [0.9, 0.1])[0] == 1:
                            continue

                pk_management = choices([0, 1], [1 - slow_missile_info[int(solution[threat])], slow_missile_info[int(solution[threat])]])[0]
                if pk_management == 0:
                    success_count += choices([0, 1], [1 - slow_missile_info[int(solution[threat])], slow_missile_info[int(solution[threat])]])[0]
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
                        if choices([0, 1], [0.9, 0.1])[0] == 1:
                            continue

                pk_management = choices([0, 1], [1 - fast_missile_info[int(solution[threat])], fast_missile_info[int(solution[threat])]])[0]
                if pk_management == 0:
                    success_count += choices([0, 1], [1 - fast_missile_info[int(solution[threat])], fast_missile_info[int(solution[threat])]])[0]
                    weapons_quantity[solution[threat] - 1] -= 2
                else:
                    success_count += pk_management
                    weapons_quantity[solution[threat] - 1] -= 1
                continue

        fitness = success_count/len(function_inputs)
        return fitness


    def on_gen(ga_instance):
        print("Generation : ", ga_instance.generations_completed)
        print("Current best solution:\n", ga_instance.best_solution()[0])
        print("Fitness of the best solution :", ga_instance.best_solution()[1], "\n")
        # Calculate Leaker Percentage
        leakers_percentage = 100 * (1 - ga_instance.best_solution()[1])
        
        if ga_instance.generations_completed == num_generations :
            print('{', end='')
            for solution in range(len(ga_instance.best_solution()[0])):
                if solution % 1 == 0 and solution != 0:
                    print()
                if ga_instance.best_solution()[0][solution] == 1:
                    print(f"[THREAT: {function_inputs[solution]}, Leaker Percentage: {leakers_percentage:.2f}%]", end=' ')
                    continue
                if ga_instance.best_solution()[0][solution] == 2:
                    print(f"[THREAT: {function_inputs[solution]}, Leaker Percentage: {leakers_percentage:.2f}%]", end=' ')
                    continue
                if ga_instance.best_solution()[0][solution] == 3:
                    print(f"[THREAT: {function_inputs[solution]}, Leaker Percentage: {leakers_percentage:.2f}%]", end=' ')
                    continue
                if ga_instance.best_solution()[0][solution] == 4:
                    print(f"[THREAT: {function_inputs[solution]}, Leaker Percentage: {leakers_percentage:.2f}%]", end=' ')
                    continue
            print('}')
            print()



    num_generations = 100
    num_parents_mating = 10

    fitness_function = fitness_func

    sol_per_pop = 20
    num_genes = len(function_inputs)

    init_range_low = 1
    init_range_high = 5
    gene_type = int

    parent_selection_type = "sss"
    keep_parents = 2

    crossover_type = "single_point"

    mutation_type = "random"
    mutation_percent_genes = 15

    ga_instance = pygad.GA(num_generations=num_generations,
                           num_parents_mating=num_parents_mating,
                           fitness_func=fitness_function,
                           sol_per_pop=sol_per_pop,
                           num_genes=num_genes,
                           init_range_low=init_range_low,
                           init_range_high=init_range_high,
                           gene_type=gene_type,
                           parent_selection_type=parent_selection_type,
                           keep_parents=keep_parents,
                           crossover_type=crossover_type,
                           mutation_type=mutation_type,
                           mutation_percent_genes=mutation_percent_genes,
                           on_generation=on_gen,
                           stop_criteria=f"saturate_{num_generations}")
    return ga_instance.run()
# ga_instance.plot_fitness()



