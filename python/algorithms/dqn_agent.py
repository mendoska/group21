from gym import Env, spaces
from stable_baselines3 import DQN
from stable_baselines3.dqn import MlpPolicy
import numpy as np
import math
import random
import csv


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

        # Start with no weapons assigned to any threat
        self.state = "0" * self.num_weapons
        # Represent all possible weapon assignments in binary => ex: 2 weapons = 4 assignments (00, 01, 10, 11)
        self.action_space = spaces.Discrete(2 ** self.num_weapons)
        # Represent all possible observations in binary => observation = state (ex: 00) and current threat index
        self.observation_space = spaces.Discrete(2 ** (self.num_weapons + self.num_threats))
        # Keep track of current threat index
        self.current_threat = 0

    # Reset environment to initial state
    def reset(self):
        # Shuffle order of weapons
        random.shuffle(self.weapons)
        # Shuffle order of threats
        random.shuffle(self.threats)
        self.state = "0" * self.num_weapons
        self.current_threat = 0
        return self.get_observation()

    # Given an action, perform one agent-environment interaction
    def step(self, action):
        # Convert chosen action to binary representation and update state (weapon assignments)
        self.state = np.binary_repr(action, width=self.num_weapons)
        # Get current state (weapon assignments) and threat index
        observation = self.get_observation()
        # Calculate reward for current weapon-threat assignment
        reward = self.calculate_reward()
        # Episode done if all threats assigned resources
        done = (self.current_threat == self.num_threats - 1)

        # Ensure agent will assign weapons to new threat in next step
        self.current_threat += 1
        return observation, reward, done, {}

    # Convert binary representation of state (weapon assignments) and current threat index to integer (observation)
    def get_observation(self):
        return int(self.state + np.binary_repr(self.current_threat, width=self.num_threats), 2)

    # Calculate reward for current weapon-threat assignment
    def calculate_reward(self):
        total_reward = 0
        # Count number of weapons assigned to current threat (ex: state = 01, 1 = weapon assigned)
        num_assigned_weapons = sum(int(self.state[i]) for i in range(self.num_weapons))

        for i in range(self.num_weapons):
            # Check if weapon assigned to current threat (bit = 1)
            if self.state[i] == '1':
                weapon_pk = self.weapons[i].get_pk()[self.threats[self.current_threat].get_name()]
                # Better pk and proximity => higher reward
                reward = weapon_pk * proximity(self.weapons[i], self.threats[self.current_threat])
                # Scale down rewards for assigning multiple weapons to one threat
                reward *= 1.0 / (1 + 0.1 * num_assigned_weapons)
                total_reward += reward
        return total_reward


def dist_to_meet(weapon, threat):
    curr_dist = math.dist([threat.get_x(), threat.get_y(), threat.get_z()],
                          [weapon.get_x(), weapon.get_y(), weapon.get_z()])
    weapon_speed, threat_speed = weapon.get_speed(), threat.get_speed()
    meeting_time = curr_dist / (weapon_speed + threat_speed)
    return weapon_speed * meeting_time


def proximity(weapon, threat):
    return weapon.get_max_range() - dist_to_meet(weapon, threat)


# Train agent on random threats
def make_training_data(num_threats=10):
    threats = []
    threat_names = ["bomber", "fighter", "slow missile", "fast missile"]
    for i in range(num_threats):
        # Threat format: name, x, y, z, min_range, speed
        threats.append(Threat(random.choice(threat_names), r(0, 1000), r(0, 1000), r(0, 1000), r(0, 1000), r(0, 1000)))
    return threats


# Test agent on threats from file
def use_testing_data(threat_file):
    threats = []
    with open(threat_file, 'r') as csv_file:
        threat_reader = csv.reader(csv_file)
        for row in threat_reader:
            threats.append(Threat(row[0], float(row[1]), float(row[2]), float(row[3]), float(row[4]), float(row[5])))
    return threats


# TRAIN DEEP Q-NETWORK TO SOLVE BATTLE ENV
def train_dqn_agent(weapon_lst, threat_lst, num_episodes=1000, save_path=None, load_path=None, use_actual_data=False):
    env = BattleEnv(weapon_lst, use_testing_data("python/dataFiles/threat_location_dqn.csv") if use_actual_data else threat_lst)

    # Define hyperparameters
    policy_kwargs = dict(net_arch=[64, 64])  # Specify NN architecture for agent: MLP with 2 hidden layers of 64 neurons
    learning_rate = 0.0001  # Control how fast agent updates Q-table: low LR => doesn't learn, high LR => unstable
    learning_starts = 1000  # Define number of initial steps to take in environment before training
    exploration_fraction = 0.4  # Defines fraction of episodes agent will explore environment vs. using learned policy

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

            # Identify leakers
            combined_pk = sum([weapon.get_pk()[current_threat] for weapon in assigned_weapons])
            # 1 = leaker, 0 = non-leaker
            leaker_id = random.choices([0, 1], [combined_pk, 1 - combined_pk])[0]
            if leaker_id == 1:
                leaker_count += 1

            # Take chosen action
            obs, reward, done, _ = env.step(action)

            # Add chosen action's reward to episode_reward
            episode_reward += reward
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
    return model, rewards_over_time


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

# Train DQN agent and save it to "trained_model.zip" for future use
def runDQN(loadPath=None, savePath="python/dataFiles/trained_model.zip", train=True):
    if train:
        train_dqn_agent(test_weapons, make_training_data(), num_episodes=100, save_path=savePath, load_path=loadPath)
    else:
        train_dqn_agent(test_weapons, None, num_episodes=1, save_path=savePath, load_path=loadPath, use_actual_data=True)
    return savePath

# trained_model, rewards = train_dqn_agent(test_weapons, make_training_data(),
#                                          num_episodes=100, save_path="../dataFiles/trained_model.zip")
