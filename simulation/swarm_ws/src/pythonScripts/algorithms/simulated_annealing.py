import copy
import random
import math

"""
simulated annealing is an algorithm modeled after the process of real annealing, like a metal. Particles move 
faster when they have a higher temperature and slow down as the metal cools. Similarly, our algorithm will search more
randomly to start with and hone in on one specific state as the temperature cools.
"""
bomber_info = ['bomber', 0.95, 0.85, 0.95, 0.99]
fighter_info = ['fighter', 0.65, 0.90, 0.75, 0.99]
slow_missile_info = ['slow missile', 0.75, 0.95, 0.90, 0.99]
fast_missile_info = ['fast missile', 0.25, 0.35, 0.15, 0.90]

 #populates function inputs and input position
function_inputs = []
inputs_position = []
with open('dataFiles/threat_location.csv', 'r') as threats:
    inputs = threats.readlines()
    for line in inputs:
        as_list = line.split(',')
        function_inputs.append(as_list[0])
        inputs_position.append([as_list[1], as_list[2], as_list[3], as_list[4]])

initial_temp = 40
final_temp = .1
alpha = 0.01
current_temp = 40

def simulated_annealing(initial_state):
    """Peforms simulated annealing to find a solution"""
    # Start by initializing the current state with the initial state
    global current_temp
    current_state = initial_state
    solution = current_state

    while current_temp > final_temp:
        neighbor = get_neighbors(current_state)

        # Check if neighbor is best so far
        leak_percent_diff = leak_percent(current_state)-leak_percent(neighbor)

        # if the new solution is better, accept it
        if leak_percent_diff > 0:
            solution = neighbor
        # if the new solution is not better, accept it with a probability of e^(-cost/temp)
        else:
            if random.choices([0, 1], [1-math.exp(leak_percent_diff/ current_temp), math.exp(leak_percent_diff/ current_temp)])[0] == 1:
                solution = neighbor
        # decrement the temperature
        current_temp -= alpha
    outputList =print_results(function_inputs, solution)
    leakPercent =(leak_percent(solution))
    return {"Weapon Selection":outputList, "Leaker Percentage":leakPercent}

def leak_percent(state):
    success_count = 0
    weapons_quantity = [30, 30, 30, 30]
    for threat in range(len(function_inputs)):
        curr_distance = math.sqrt(
            float(inputs_position[threat][0]) ** 2 + float(inputs_position[threat][1]) ** 2 + float(
                inputs_position[threat][2]) ** 2)
        if function_inputs[threat] == 'bomber':
            # Distance constraint implementation
            if curr_distance < float(inputs_position[threat][3]):
                continue
            # Weapon quantity constraint implementation
            if weapons_quantity[state[threat] - 1] <= 0:
                continue
            # Reload time constraint implementation
            if threat >= 1:
                if state[threat - 1] == state[threat]:
                    if random.choices([0, 1], [0.9, 0.1])[0] == 1:
                        continue
            # Shoot-Look-Shoot implementation
            pk_management = \
            random.choices([0, 1], [1 - bomber_info[int(state[threat])], bomber_info[int(state[threat])]])[0]
            if pk_management == 0:
                success_count += \
                random.choices([0, 1], [1 - bomber_info[int(state[threat])], bomber_info[int(state[threat])]])[0]
                weapons_quantity[state[threat] - 1] -= 2
            else:
                success_count += pk_management
                weapons_quantity[state[threat] - 1] -= 1
            continue

        if function_inputs[threat] == 'fighter':
            if curr_distance < float(inputs_position[threat][3]):
                continue

            if weapons_quantity[state[threat] - 1] <= 0:
                continue

            if threat >= 1:
                if state[threat - 1] == state[threat]:
                    if random.choices([0, 1], [0.9, 0.1])[0] == 1:
                        continue

            pk_management = \
            random.choices([0, 1], [1 - fighter_info[int(state[threat])], fighter_info[int(state[threat])]])[0]
            if pk_management == 0:
                success_count += \
                random.choices([0, 1], [1 - fighter_info[int(state[threat])], fighter_info[int(state[threat])]])[0]
                weapons_quantity[state[threat] - 1] -= 2
            else:
                success_count += pk_management
                weapons_quantity[state[threat] - 1] -= 1
            continue

        if function_inputs[threat] == 'slow missile':
            if curr_distance < float(inputs_position[threat][3]):
                continue

            if weapons_quantity[state[threat] - 1] <= 0:
                continue

            if threat >= 1:
                if state[threat - 1] == state[threat]:
                    if random.choices([0, 1], [0.9, 0.1])[0] == 1:
                        continue

            pk_management = \
            random.choices([0, 1], [1 - slow_missile_info[int(state[threat])], slow_missile_info[int(state[threat])]])[0]
            if pk_management == 0:
                success_count += random.choices([0, 1], [1 - slow_missile_info[int(state[threat])],
                                                  slow_missile_info[int(state[threat])]])[0]
                weapons_quantity[state[threat] - 1] -= 2
            else:
                success_count += pk_management
                weapons_quantity[state[threat] - 1] -= 1
            continue

        if function_inputs[threat] == 'fast missile':
            if curr_distance < float(inputs_position[threat][3]):
                continue

            if weapons_quantity[state[threat] - 1] <= 0:
                continue

            if threat >= 1:
                if state[threat - 1] == state[threat]:
                    if random.choices([0, 1], [0.9, 0.1])[0] == 1:
                        continue

            pk_management = \
            random.choices([0, 1], [1 - fast_missile_info[int(state[threat])], fast_missile_info[int(state[threat])]])[0]
            weapons_quantity[state[threat] - 1] -= 1
            if pk_management == 0:
                while pk_management != 1 and weapons_quantity[state[threat] - 1] >= 0:
                    pk_management = random.choices([0, 1], [1 - fast_missile_info[int(state[threat])],
                                                  fast_missile_info[int(state[threat])]])[0]
                    weapons_quantity[state[threat] - 1] -= 1
                success_count += pk_management
            else:
                success_count += pk_management
                weapons_quantity[state[threat] - 1] -= 1
            continue
    a = len(function_inputs)

    leaks = (len(function_inputs)-success_count) / len(function_inputs)

    return leaks


def get_neighbors(state):
    neighbor = copy.deepcopy(state)
    num_of_changes = int(current_temp)
    if len(function_inputs) < num_of_changes:
        print('please change your starting temp to be smaller than the number of inputs you have')
    positions_to_change = get_random(num_of_changes, 0, len(state))
    for position in range(len(neighbor)):
        neighbor[position] = (random.randint(1, 4))
    return neighbor


def get_random(how_many, lower_bound, upper_bound):
    random_numbers = set()
    for i in range(how_many):
        random_numbers.add(random.randint(lower_bound, upper_bound-1))
    return random_numbers


def print_results(function_input, solution):
    res = []
    for i in range(len(function_inputs)):
        if solution[i] == 1:
            res.append(f"[THREAT: {function_inputs[i]}, Weapon Chosen: 'Long Range Missile']")
            continue
        if solution[i] == 2:
            res.append(f"[THREAT: {function_inputs[i]}, Weapon Chosen: 'Medium Range Missile']")
            continue
        if solution[i] == 3:
            res.append(f"[THREAT: {function_inputs[i]}, Weapon Chosen: 'Short Range Missile']")
            continue
        if solution[i] == 4:
            res.append(f"[THREAT: {function_inputs[i]}, Weapon Chosen: 'Directed Energy']")
            continue

    return res

def runSimulatedAnnealing():
    initial_solution = []
    for i in range(50):
        initial_solution.append(random.randint(1, 4))
    return simulated_annealing(initial_solution)
