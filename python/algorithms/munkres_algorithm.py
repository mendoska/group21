from scipy.optimize import linear_sum_assignment
import numpy as np
import math
import random
import time
import csv


# reduce weapon quantity after usage
def reduce_quantity(ind_to_weapon, weapon_ind):
    ind_to_weapon[weapon_ind][5] -= 1


# track time when weapon fired
def fired(ind_to_weapon, weapon_ind):
    ind_to_weapon[weapon_ind][7] = time.time()


# print best solution and consider weapon strategy
def update_results(threat, ind_to_weapon, cost, row_ind, col_ind,):
    output = []
    threat_name = []
    # print results
    print("THREAT INFO FORMAT: THREAT NAME, X POS, Y POS, Z POS, MIN RANGE, SPEED, STATUS")
    print("WEAPON INFO FORMAT: WEAPON NAME, X POS, Y POS, Z POS, MAX RANGE, QUANTITY, "
          "FIRING RATE, LAST FIRED, SPEED, STRATEGY")
    print("=========================================================================================")
    print("Threat: {threat}\nThreat Info: {threat_info}\nWeapon: {weapon}\nWeapon Info: {weapon_info}\nCost: {cost}"
          .format(threat=threat[0],
                  threat_info=threat,
                  weapon=ind_to_weapon[row_ind[0]][0],
                  weapon_info=ind_to_weapon[row_ind[0]],
                  cost=cost[row_ind[0], col_ind[0]]))
    print("=========================================================================================")
    output += [ind_to_weapon[row_ind[0]][0]]
    threat_name.append(threat[0])
    # apply strategy
    if ind_to_weapon[row_ind[0]][9] == "SSL" and threat[6] == "A":
        reduce_quantity(ind_to_weapon, row_ind[0])
        fired(ind_to_weapon, row_ind[0])
        if ind_to_weapon[row_ind[0]][5] > 0:
            reduce_quantity(ind_to_weapon, row_ind[0])
            fired(ind_to_weapon, row_ind[0])
    elif ind_to_weapon[row_ind[0]][9] == "SLS" and threat[6] == "A":
        reduce_quantity(ind_to_weapon, row_ind[0])
        fired(ind_to_weapon, row_ind[0])
        look(threat, -cost[row_ind[0], col_ind[0]])
        if threat[6] == "A" and ind_to_weapon[row_ind[0]][5] > 0:
            reduce_quantity(ind_to_weapon, row_ind[0])
            fired(ind_to_weapon, row_ind[0])
    else:
        reduce_quantity(ind_to_weapon, row_ind[0])
        fired(ind_to_weapon, row_ind[0])
    return threat_name, output


# check if target killed
def look(threat, pr):
    if pr > 0.5:
        threat[6] = "D"


# check if weapon available given firing rate
def weapon_available(weapon):
    curr_time = time.time()
    fired_time = weapon[7]
    fire_rate = weapon[6]
    return curr_time - fired_time > fire_rate


# calculate meeting distance of weapon and threat
def dist_to_meet(weapon, threat):
    curr_dist = math.dist([threat[1], threat[2], threat[3]], [weapon[1], weapon[2], weapon[3]])
    weapon_speed, threat_speed = weapon[8], threat[5]
    meeting_time = curr_dist / (weapon_speed + threat_speed)
    return weapon_speed * meeting_time


# check if threat reachable by weapon given maximum range and meeting distance
def is_reachable(weapon, threat):
    return weapon[4] - dist_to_meet(weapon, threat) > 0


# check if threat is leaker
def is_leaker(cost, weapon_strategy):
    percent = -cost
    if weapon_strategy == "SSL" or weapon_strategy == "SLS":
        percent = -cost * 2
    if random.choices([0, 1], [percent, 1 - percent])[0] == 1:  # 1 = leaker, 0 = non-leaker
        return True
    return False


# calculate cost of assigning different weapons to incoming threat
def calc_cost(matrix, weapons, threat):
    for i, weapon in enumerate(weapons):
        quantity = weapon[5]
        if is_reachable(weapon, threat) and quantity > 0 and weapon_available(weapon):
            matrix[i][0] *= -1
        else:
            matrix[i][0] = 0
    return matrix


# main algorithm
def munkres(threat_file, weapon_file, pk):
    # ind_to_weapon format: {row ind: ["weapon name", x pos, y pos, z pos, max range, quantity, firing rate,
    # last_fired, speed, strategy]}
    threat_count = 0
    leaker_count = 0
    ind_to_weapon = {}
    output = []
    nameoutput = []

    # parse weapon data
    with open(weapon_file) as csv_file:
        weapon_reader = csv.reader(csv_file)
        for i, row in enumerate(weapon_reader):
            ind_to_weapon[i] = [row[0], float(row[1]), float(row[2]),
                                float(row[3]), float(row[4]), int(row[5]),
                                float(row[6]), float(row[7]), float(row[8]), row[9]]

    # parse threat data
    with open(threat_file) as csv_file:
        threat_reader = csv.reader(csv_file)
        for row in threat_reader:
            pk_matrix, threat_count = [], threat_count + 1
            threat = [row[0], float(row[1]), float(row[2]),
                      float(row[3]), float(row[4]), float(row[5]), row[6]]
            for weapon_ind, weapon in enumerate(ind_to_weapon):
                pk_matrix.append([pk["fighter"][weapon_ind]])
            cost = calc_cost(np.array(pk_matrix), ind_to_weapon.values(), threat)
            row_ind, col_ind = linear_sum_assignment(cost)
            if is_leaker(cost[row_ind[0], col_ind[0]], ind_to_weapon[row_ind[0]][9]):  # count number of leakers
                leaker_count += 1
            ##output += update_results(threat, ind_to_weapon, cost, row_ind, col_ind)
            nameoutput.append(update_results(threat, ind_to_weapon, cost, row_ind, col_ind))
            output.append([threat[0], [ind_to_weapon[row_ind[0]][0]]])
    # print(f"SELECTED WEAPONS: {output}")
    print(f"LEAKER PERCENTAGE {(leaker_count / threat_count) * 100}%")
    # write_output_to_csv(nameoutput, "output_results.csv")
    leaker_percentage = (leaker_count / threat_count) * 100
    return output, leaker_percentage

def write_output_to_csv(output, output_filename):
    with open(output_filename, 'w', newline='') as csv_file:
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(["Threat Name", "Assigned Weapon"])
        for item in output:
           csv_writer.writerow([item[0], item[1]])
           
# pk format: LRM MRM SRM DE
#           B [?, ?, ?, ?]
#           F [?, ?, ?, ?]
#          SM [?, ?, ?, ?]
#          FM [?, ?, ?, ?]
# run algorithm
def runMunkres(threatFileLocation, weaponFileLocation):
    leaker_percentage = 0
    pk_dict = {"bomber": [0.95, 0.85, 0.95, 0.99],
            "fighter": [0.65, 0.90, 0.75, 0.99],
            "slow missile": [0.75, 0.95, 0.90, 0.99],
            "fast missile": [0.25, 0.35, 0.15, 0.90]}
    output, leaker_percentage = munkres(threatFileLocation, weaponFileLocation, pk_dict)
    return output, leaker_percentage
    