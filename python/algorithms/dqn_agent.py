from gymnasium import Env, spaces
from stable_baselines3 import DQN
from stable_baselines3.dqn import MlpPolicy
import numpy as np
import math
import random
import csv

# Makes DQN env more consistent 
random.seed(42)
np.random.seed(42)

# DEFINE WEAPON ATTRIBUTES
class Weapon:
    def __init__(self, name, x, y, z, max_range, speed, pk):
        self.name = name
        self.x = x
        self.y = y
        self.z = z
        self.max_range = max_range
        self.speed = speed
        self.pk = pk

    # Access attributes
    def get_name(self):
        return self.name

    def get_x(self):
        return self.x

    def get_y(self):
        return self.y

    def get_z(self):
        return self.z

    def get_max_range(self):
        return self.max_range

    def get_speed(self):
        return self.speed

    def get_pk(self):
        return self.pk


# DEFINE THREAT ATTRIBUTES
class Threat:
    def __init__(self, name, x, y, z, min_range, speed):
        self.name = name
        self.x = x
        self.y = y
        self.z = z
        self.min_range = min_range
        self.speed = speed

    # Access attributes
    def get_name(self):
        return self.name

    def get_x(self):
        return self.x

    def get_y(self):
        return self.y

    def get_z(self):
        return self.z

    def get_speed(self):
        return self.speed


# DEFINE ENVIRONMENT FOR WEAPON-THREAT ASSIGNMENT
class BattleEnv(Env):
    def __init__(self, weapons, threats):
        self.weapons, self.num_weapons = weapons, len(weapons)
        self.threats, self.num_threats = threats, len(threats)
        # Simplify state space: Each element in the state array represents a threat, 
        # and its value is the index of the weapon assigned to it, -1 if no weapon is assigned
        self.state = np.full(self.num_threats, -1, dtype=int)
        # Action space is the product of the number of weapons and the number of threats
        self.action_space = spaces.Discrete(self.num_weapons * self.num_threats)
        # Observation space: Vector with length equal to number of threats,
        # where each element is -1 if the threat is unassigned, or the index of the assigned weapon
        self.observation_space = spaces.Box(low=-1, high=self.num_weapons-1, shape=(self.num_threats,), dtype=np.int32)
        # Keep track of current threat index
        self.current_threat = 0
        
    def update_observation_space(self):
        self.observation_space = spaces.Box(low=-1, high=self.num_weapons-1, shape=(self.num_threats,), dtype=np.int32)

    # Reset environment to initial state
    def reset(self):
        # Shuffle order of weapons
        random.shuffle(self.weapons)
        # Shuffle order of threats
        random.shuffle(self.threats)
        # Initially assign a random weapon to each threat
        self.state = np.array([random.randint(0, self.num_weapons - 1) for _ in range(self.num_threats)], dtype=int)
        self.current_threat = 0
        return self.get_observation()

    # Given an action, perform one agent-environment interaction
    def step(self, action):
        # Action is now an index representing which weapon is assigned to which threat
        weapon_index = action % self.num_weapons
        threat_index = action // self.num_weapons
        # Assign weapon to threat
        self.state[threat_index] = weapon_index
        observation = self.get_observation()
        # Calculate reward for current weapon-threat assignment
        reward = self.calculate_reward()
        # Episode done if all threats assigned resources
        done = (self.current_threat == self.num_threats - 1) and all(weapon_assigned != -1 for weapon_assigned in self.state)

        # Ensure agent will assign weapons to new threat in next step
        if self.current_threat < self.num_threats - 1:
            self.current_threat += 1
        else:
            done = True  # Ensure the episode is marked as done
        return observation, reward, done, {}

    def get_observation(self):
        return self.state

    # Calculate reward for current weapon-threat assignment
    def calculate_reward(self):
        total_reward = 0
        # Penalty for unassigned weapons
        unassigned_penalty = -10
        for threat_index, weapon_index in enumerate(self.state):
            if weapon_index != -1:  # Check if weapon is assigned
                weapon = self.weapons[weapon_index]
                threat = self.threats[threat_index]
                weapon_pk = weapon.get_pk()["bomber"]
                reward = weapon_pk * proximity(weapon, threat)
                total_reward += reward
            else:
                total_reward+= unassigned_penalty
        return total_reward

# These 2 functions help calcualte PK (probability of kill)
def dist_to_meet(weapon, threat):
    curr_dist = math.dist([threat.get_x(), threat.get_y(), threat.get_z()],
                          [weapon.get_x(), weapon.get_y(), weapon.get_z()])
    weapon_speed, threat_speed = weapon.get_speed(), threat.get_speed()
    meeting_time = curr_dist / (weapon_speed + threat_speed)
    return weapon_speed * meeting_time


def proximity(weapon, threat):
    return weapon.get_max_range() - dist_to_meet(weapon, threat)


# Train agent on random threats - This is fake data for training
def make_training_data(num_threats=15):
    threats = []
    threat_names = ["bomber", "fighter", "slow missile", "fast missile"]
    for i in range(num_threats):
        # Threat format: name, x, y, z, min_range, speed
        threats.append(Threat(random.choice(threat_names), r(0, 1000), r(0, 1000), r(0, 1000), r(0, 1000), r(0, 1000)))
    return threats

# Read the number of inputs
# This is vital as the agent will not work if the threat count differs from the training data
def count_threats(threat_file):
    with open(threat_file, 'r') as file:
        reader = csv.reader(file)
        threats = list(reader)
        return len(threats)

# Test agent on threats from file - This is the actual data
def use_testing_data(threat_file):
    threats = []
    
    with open(threat_file, 'r') as csv_file:
        threat_reader = csv.reader(csv_file)
        for row in threat_reader:
            threats.append(Threat(row[0], float(row[1]), float(row[2]), float(row[3]), float(row[4]), float(row[5])))
    return threats


# TRAIN DEEP Q-NETWORK TO SOLVE BATTLE ENV
""" 
    INPUT: weapons, threats, episode count
    OUTPUT: episode reward, leaker count, trained model
    Currently trains agent with random threat values for 100 iterations before using the agent against the real threats
"""
def train_dqn_agent(weapon_lst, threat_lst, num_episodes=1000, save_path=None, load_path=None, use_actual_data=False, test=False, threat_file_path=None):
    env = BattleEnv(weapon_lst, use_testing_data(threat_file_path) if use_actual_data else threat_lst)
    """"
        Define hyperparameters--These are the important values, 
        If agent isnt working right its probably because these values need to be changed in relation to the threat count
        Ideally given more time I would split these into seperate cases based on threat count
    """    
    policy_kwargs = dict(net_arch=[128, 128, 128])  # Specify NN architecture for agent: MLP with 3 hidden layers of 128 neurons
    learning_rate = 0.01  # Control how fast agent updates Q-table: low LR => doesn't learn, high LR => unstable
    learning_starts = 100 # Define number of initial steps to take in environment before training
    # initial_exploration_fraction = 1.0  # Start with full exploration
    # exploration_decay = 0.99  # Decay rate for exploration fraction per episode
    exploration_fraction = 0.5 # Manipulate this if DQN is not assigning weapons, Higher = more exploring, Lower = more decisions based on memory
    batch_size = 64
    buffer_size = 10000
    target_update_interval = 500

    # Load pre-trained model if load_path provided
    if load_path is not None:
        model = DQN.load(load_path, env=env, verbose=1)
        print("Model loaded:", model is not None)
    # Otherwise, create new DQN agent with predefined hyperparameters
    else:
        model = DQN(MlpPolicy, env, policy_kwargs=policy_kwargs, learning_rate=learning_rate,
                    buffer_size=buffer_size, learning_starts=learning_starts, batch_size=batch_size,
                    exploration_fraction=exploration_fraction, target_update_interval=target_update_interval,
                    verbose=1)

    # Keep track of rewards from each episode during training
    rewards_over_time = []
    # Keep track of leakers
    leaker_count = 0
    response = []

    for episode in range(num_episodes):
        # Reset environment
        obs = env.reset()
        episode_reward = 0
        done = False

        # Keep going until all threats have been assigned weapons
        while not done:
            # Choose action greedily (exploitation) or randomly (exploration) based on exploration_fraction
            action, _ = model.predict(obs, deterministic=np.random.rand() > exploration_fraction)

            # Print weapon assignments for current threat
            current_threat = env.threats[env.current_threat].get_name()
            assigned_weapons = [env.weapons[i] for i in range(env.num_weapons) if action & (1 << i)]
            print(f"Threat: {current_threat}, Assigned Weapons: {[weapon.get_name() for weapon in assigned_weapons]}")
            if test:
                response.append([current_threat, [weapon.get_name() for weapon in assigned_weapons]])
            # Identify leakers
            combined_pk = sum([weapon.get_pk()["bomber"] for weapon in assigned_weapons])
            # 1 = leaker, 0 = non-leaker
            leaker_id = random.choices([0, 1], [combined_pk, 1 - combined_pk])[0]
            if leaker_id == 1:
                leaker_count += 1

            # Take chosen action
            obs, reward, done, _ = env.step(action)

            # Add chosen action's reward to episode_reward
            episode_reward += reward
        
        # Decay exploration fraction after each episode
        # exploration_fraction *= exploration_decay
        # exploration_fraction = max(exploration_fraction, 0.1)  # Ensure exploration fraction does not go below 10%

        # Add episode_reward to rewards_over_time
        rewards_over_time.append(episode_reward)
        # Print episode number and episode reward
        print("=========================================================")
        print(f"EPISODE {episode + 1} - EPISODE REWARD: {episode_reward}")
        print("=========================================================")
        # Save model after each episode if save_path provided
        if save_path is not None:
            model.save(save_path)
    print(f"LEAKER PERCENTAGE {(leaker_count / (env.num_threats * num_episodes)) * 100}%")
    print("=========================================================")
    leaker_percentage = (leaker_count / (env.num_threats * num_episodes)) * 100
    if test: 
        return response, leaker_count
    return model, rewards_over_time, leaker_percentage


# START TRAINING PROCESS
r = random.uniform
lrm_pk = {"bomber": 0.95, "fighter": 0.65, "slow missile": 0.75, "fast missile": 0.25}
mrm_pk = {"bomber": 0.85, "fighter": 0.90, "slow missile": 0.95, "fast missile": 0.35}
srm_pk = {"bomber": 0.95, "fighter": 0.75, "slow missile": 0.90, "fast missile": 0.15}
de_pk = {"bomber": 0.99, "fighter": 0.99, "slow missile": 0.99, "fast missile": 0.90}
# Weapon format: name, x, y, z, max_range, speed, pk
test_weapons = [Weapon("long range missile", r(0, 1000), r(0, 1000), r(0, 1000), r(0, 1000), r(0, 1000), lrm_pk),
                Weapon("medium range missile", r(0, 1000), r(0, 1000), r(0, 1000), r(0, 1000), r(0, 1000), mrm_pk),
                Weapon("short range missile", r(0, 1000), r(0, 1000), r(0, 1000), r(0, 1000), r(0, 1000), srm_pk),
                Weapon("directed energy", r(0, 1000), r(0, 1000), r(0, 1000), r(0, 1000), r(0, 1000), de_pk)]

# threat_count = count_threats("dataFiles/threat_location.csv")
"""
    Train DQN agent and save it to "trained_model.zip" for future use
    This is essentially the 'main' and the function called in GUI and sim
    If train is true then fresh agent is trained, else it uses the model against actual threats
"""
def runDQN(loadPath=None, savePath="dataFiles/trained_model.zip", train=True, num_threats=None, threatFilePath=None):
    leaker_percentage = 0
    if train:
        if num_threats is None:
            raise ValueError("Number of threats must be provided before training")
        threat_data = make_training_data(num_threats)
        train_dqn_agent(test_weapons, threat_data, num_episodes=100, save_path=savePath, load_path=loadPath)
    else:
        threat_data = use_testing_data(threatFilePath)
        response, leaker_percentage = train_dqn_agent(test_weapons, threat_data, num_episodes=1, save_path=savePath, load_path=loadPath, use_actual_data=True, test=True, threat_file_path=threatFilePath)
        return response, leaker_percentage
    return savePath, leaker_percentage

# trained_model, rewards = train_dqn_agent(test_weapons, make_training_data(),
#                                          num_episodes=100, save_path="../dataFiles/trained_model.zip")

# runDQN(savePath="python/dataFiles/trained_model.zip", train=True)