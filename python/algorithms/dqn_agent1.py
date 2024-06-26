from gymnasium import Env, spaces
from stable_baselines3 import DQN
from stable_baselines3.dqn import MlpPolicy
import numpy as np
import math
import random
import csv
from collections import deque
import torch
import torch.nn.functional as F
from torch.utils.data import DataLoader

"""  
    This version of DQN contains all the experiremental changes like batch updates, Dual replay buffering and expoloration decay
    All of these have varying results in improving the performance based on threat count. 
    Otherwise it is identical to the other. Use this with these points in mind
"""

class CustomDQN(DQN):
    def compute_q_values(self, states):
           # Convert states to PyTorch tensor
        states = torch.tensor(states, dtype=torch.float)
        if torch.cuda.is_available():
            states = states.cuda()
    
        # Pass states through the model's policy to get Q-values
        q_values = self.policy(states)
    
        return q_values
    
    def compute_next_max_q(self, next_states):
        
        q_values = self.compute_q_values(next_states)
        return max(q_values)

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
        self.action_space = spaces.Discrete(self.num_weapons)
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
        self.state = np.full(self.num_threats, -1, dtype=int)
        self.current_threat = 0
        return self.get_observation()

    # Given an action, perform one agent-environment interaction
    def step(self, action):
        # Action is now an index representing which weapon is assigned to which threat
        weapon_index = action 
        threat_index = self.current_threat
        # Assign weapon to threat
        self.state[threat_index] = weapon_index
        observation = self.get_observation()
        # Calculate reward for current weapon-threat assignment
        reward = self.calculate_reward()
        # Episode done if all threats assigned resources
        done = (self.current_threat == self.num_threats - 1)

        # Ensure agent will assign weapons to new threat in next step
        self.current_threat += 1
        return observation, reward, done, {}

    def get_observation(self):
        return self.state

    # Calculate reward for current weapon-threat assignment
    def calculate_reward(self):
        total_reward = 0
        for threat_index, weapon_index in enumerate(self.state):
            if weapon_index != -1:  # Check if weapon is assigned
                weapon = self.weapons[weapon_index]
                threat = self.threats[threat_index]
                weapon_pk = weapon.get_pk()[threat.get_name()]
                reward = weapon_pk * proximity(weapon, threat)
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

# Read the number of inputs 
def count_threats(threat_file):
    with open(threat_file, 'r') as file:
        reader = csv.reader(file)
        threats = list(reader)
        return len(threats)

# Test agent on threats from file
def use_testing_data(threat_file):
    threats = []
    with open(threat_file, 'r') as csv_file:
        threat_reader = csv.reader(csv_file)
        for row in threat_reader:
            threats.append(Threat(row[0], float(row[1]), float(row[2]), float(row[3]), float(row[4]), float(row[5])))
    return threats


# TRAIN DEEP Q-NETWORK TO SOLVE BATTLE ENV
def train_dqn_agent(weapon_lst, threat_lst, num_episodes=1000, save_path=None, load_path=None, use_actual_data=False, test=False, threat_file_path=None):
    env = BattleEnv(weapon_lst, use_testing_data(threat_file_path) if use_actual_data else threat_lst)

    # Define hyperparameters
    policy_kwargs = dict(net_arch=[128, 128, 128])  # Specify NN architecture for agent: MLP with 3 hidden layers of 128 neurons
    learning_rate = 0.01  # Control how fast agent updates Q-table: low LR => doesn't learn, high LR => unstable
    learning_starts = 100  # Define number of initial steps to take in environment before training
    exploration_fraction = 0.6  # Defines fraction of episodes agent will explore environment vs. using learned policy

    batch_size = 64
    buffer_size = 10000
    target_update_interval = 500

    # Initialize the replay memory
    replay_memory = deque(maxlen=buffer_size)

    # Load pre-trained model if load_path provided
    if load_path is not None:
        model = DQN.load(load_path, env=env, verbose=1)
        print("Model loaded:", model is not None)
    # Otherwise, create new DQN agent with predefined hyperparameters
    else:
        model = CustomDQN(MlpPolicy, env, policy_kwargs=policy_kwargs, learning_rate=learning_rate,
                    buffer_size=buffer_size, learning_starts=learning_starts, batch_size=batch_size,
                    exploration_fraction=exploration_fraction, target_update_interval=target_update_interval,
                    verbose=1)

    if torch.cuda.is_available():
        model.policy = model.policy.cuda()

    optimizer = torch.optim.Adam(model.policy.parameters(), lr=learning_rate)

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
            assigned_weapon = env.weapons[action]
            print(f"Threat: {current_threat}, Assigned Weapon: {assigned_weapon.get_name()}")
            if test:
                response.append([current_threat, [assigned_weapon.get_name()]])
            # Identify leakers
            combined_pk = assigned_weapon.get_pk()[current_threat]
            # 1 = leaker, 0 = non-leaker
            leaker_id = random.choices([0, 1], [combined_pk, 1 - combined_pk])[0]
            if leaker_id == 1:
                leaker_count += 1

            # Take chosen action
            new_obs, reward, done, _ = env.step(action)

            # Store the experience in replay memory
            replay_memory.append((obs, action, reward, new_obs, done))

            # Update the current observation
            obs = new_obs

            # Add chosen action's reward to episode_reward
            episode_reward += reward

            # Once memory is big enough...
            if len(replay_memory) > batch_size:
                    # Sample a batch of experiences
                    experiences = random.sample(replay_memory, batch_size)
                    batch_state, batch_action, batch_reward, batch_new_state, batch_done = zip(*experiences)
                    # Flatten batch_action
                    batch_action = [a.item() for a in batch_action]

                    # Convert batches to tensors
                    batch_state = torch.tensor(batch_state, dtype=torch.float)
                    batch_action = torch.tensor(batch_action, dtype=torch.long)
                    batch_reward = torch.tensor(batch_reward, dtype=torch.float)
                    batch_new_state = torch.tensor(batch_new_state, dtype=torch.float)
                    batch_done = torch.tensor(batch_done, dtype=torch.uint8)  # Boolean

                    # Compute current Q value, q_func takes only state and output value for every state-action pair
                    # We choose Q based on action taken.
                    current_q_values = model.compute_q_values(batch_state)

                    # Compute next Q value based on which action gives max Q values
                    # Detach variable from the current graph since we don't want gradients for next Q to propagated
                    gamma = 0.99
                    next_max_q = model.compute_next_max_q(batch_new_state)
                    if torch.cuda.is_available():
                        batch_reward = batch_reward.cuda()
                        next_max_q = next_max_q.cuda()
                        batch_done = batch_done.cuda()
                    next_q_values = batch_reward + (gamma * next_max_q) * (1 - batch_done)

                    current_q_values = current_q_values.float()
                    next_q_values = next_q_values.float()

                    current_q_values.requires_grad = True
                    next_q_values.requires_grad = True

                    # Compute the Huber loss
                    loss = F.smooth_l1_loss(current_q_values, next_q_values)

                    # Optimize the model
                    for param in model.policy.parameters():
                        param.grad = None 
                    loss.backward()
                    optimizer.step()

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

# threat_count = count_threats("python/dataFiles/threat_location.csv")

# Train DQN agent and save it to "trained_model.zip" for future use
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

# runDQN(savePath="dataFiles/trained_model.zip", train=True)
# runDQN(loadPath="dataFiles/trained_model.zip", train=False)