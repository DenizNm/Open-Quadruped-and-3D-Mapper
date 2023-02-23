import gymnasium as gym
import tensorflow as tf
from keras import layers
import numpy as np
import matplotlib.pyplot as plt
import operator
from gymnasium import utils
from gymnasium.envs.mujoco import MujocoEnv
from gymnasium.spaces import Box
import mujoco as mj
import math

DEFAULT_CAMERA_CONFIG = {
    "trackbodyid": 0,
    "distance": 2,
    "lookat": np.array((0.0, 0.0, 0.1225)),
}

class QRPfRA(MujocoEnv, utils.EzPickle):
    metadata = {
        "render_modes": [
            "human",
            "rgb_array",
            "depth_array",
        ],
        "render_fps": 10,
    }

    def __init__(
        self,
        xml_file="/Users/deniz/PycharmProjects/mujoco_reinforcement_research/QRPfRA_deneme/qrpfra_2.xml",
        reset_noise_scale=0.1,
        exclude_current_positions_from_observation=True,
        **kwargs
    ):
        utils.EzPickle.__init__(
            self,
            xml_file,
            reset_noise_scale,
            exclude_current_positions_from_observation,
            **kwargs
        )
        self._reset_noise_scale = reset_noise_scale
        self._exclude_current_positions_from_observation = (
            exclude_current_positions_from_observation
        )

        obs_shape = 34 #observation space shape

        observation_space = Box(
            low=-np.inf, high=np.inf, shape=(obs_shape,), dtype=np.float64
        )

        MujocoEnv.__init__(
            self,
            xml_file,
            5,
            observation_space=observation_space,
            default_camera_config=DEFAULT_CAMERA_CONFIG,
            **kwargs
        )
        self.step_counter = 0
        self.episode_step_lenght = 400

        self.action_buffer_cnt = 0
        self.action_buffer_size = 5
        self.action_array_lenght = len(self.data.ctrl)
        self.action_buffer = np.zeros((self.action_buffer_size, self.action_array_lenght))


    def stopping_penalty(self, action_arr):
        penalty = 0
        for i in range(0, self.action_buffer_size):
            if i == 0:
                self.action_buffer[i] = action_arr
            else:
                self.action_buffer[self.action_buffer_size-i] = self.action_buffer[self.action_buffer_size-i-1]

        for j in range(0, self.action_array_lenght):
            if j <= 3: #Hindlimbs Angle
                if self.action_buffer[1][j] == self.action_buffer[-1][j]:
                    penalty -= 5

            elif j > 3: #Midlimb and Wrist Angles
                if self.action_buffer[1][j] == self.action_buffer[-1][j]:
                    penalty -= 10
            else:
                pass

        if self.step_counter == self.episode_step_lenght:
            self.action_buffer = np.zeros((self.action_buffer_size, self.action_array_lenght))
        else:
            pass
        return penalty

    def position_stopping_penalty(self, action_arr):
        penalty = 0
        for i in range(0, self.action_buffer_size):
            if i == 0:
                self.action_buffer[i] = action_arr
            else:
                self.action_buffer[self.action_buffer_size-i] = self.action_buffer[self.action_buffer_size-i-1]

        for j in range(0, self.action_array_lenght):
            if j <= 3: #Hindlimbs Angle
                if self.action_buffer[1][j] == self.action_buffer[-1][j]:
                    penalty -= 0 #previous was 5

            elif j > 3: #Midlimb and Wrist Angles
                if abs(self.action_buffer[1][j] == self.action_buffer[-1][j]) < 34:
                    penalty -= 10
            else:
                pass

        if self.step_counter == self.episode_step_lenght:
            self.action_buffer = np.zeros((self.action_buffer_size, self.action_array_lenght))
        else:
            pass
        return penalty

    @property
    def ultrasonic_range_reward(self):
        step_range_reward = 0
        gnd_front, gnd_rear, front, rear = [self.data.sensordata[6], self.data.sensordata[7], self.data.sensordata[8], self.data.sensordata[9]]
        if (0.05 <= gnd_front <= 0.4) and (0.05 <= gnd_rear <= 0.4):
            step_range_reward += 1
        else:
            step_range_reward -= 100

        if front == -1 or front >= 3.95:
            step_range_reward += 1
        elif front >= 0.20 and front <= 4:
            step_range_reward -= 10
        else:
            pass

        if rear == -1 or rear >= 3.95:
            step_range_reward += 1
        elif rear >= 0.20 and rear <= 4:
            step_range_reward -= 10
        else:
            pass
        return step_range_reward

    def stuck_turtle(self, obser):
        turtle_penalty = 0
        if 9.5 <= obser[2] <= 10:
            turtle_penalty -= 100
        else:
            pass
        if 9.81 >= abs(obser[0]) >= 5.0:
            turtle_penalty -= 10 * abs(obser[0])
        else:
            pass

        return turtle_penalty


    def step(self, action):
        self.step_counter += 1

        action = list(action)[0]
        actuator_before = self.data.ctrl
        base_link_acc_before = self.data.sensordata[1] #front-y

        #Simulation Step
        self.do_simulation(action, self.frame_skip)

        #Get Observation
        observation = self._get_obs()

        range_reward = self.ultrasonic_range_reward
        base_link_acc_after = self.data.sensordata[1] #front-y

        """obs_tot_acc = math.sqrt(observation[0]**2 + observation[1]**2 + observation[2]**2)
        print(f"norx = {observation[0]/-9.81}, nory = {observation[1]/-9.81}, norz = {observation[2]/-9.81}")
        print(f"tetha = {math.degrees(math.atan(math.sqrt(observation[0]**2 + observation[1]**2)/observation[2]))}")
        print(f"phi = {math.degrees(math.atan(observation[1]/observation[0]))}")"""

        reward = 10 + abs(1/2 * ((base_link_acc_after) - (base_link_acc_before)) * self.dt**2) * 100000
        reward = reward + range_reward + self.stuck_turtle(observation) + self.stopping_penalty(action)# + self.position_stopping_penalty(action)

        info = {}

        termination = False
        actuator_after = self.data.ctrl
        for i in range(0, len(actuator_before)):
            if np.isnan(actuator_after[i]) or np.isnan(actuator_before[i]):
                termination = True

        if 9.5 <= self.data.sensordata[2] <= 10:
            termination = True

        if self.render_mode == "human":
            self.render()

        if (self.step_counter == self.episode_step_lenght) or termination:
            terminated = True
        else:
            terminated = False

        return observation, reward, terminated, False, info

    def _get_obs(self):
        listed_obs = []
        for i in self.data.sensordata:
            listed_obs.append(i)
        for j in self.data.ctrl:
            listed_obs.append(j) #can be also divided j/100
        return listed_obs

    def reset_model(self):
        self.step_counter = 0
        noise_low = -self._reset_noise_scale
        noise_high = self._reset_noise_scale

        """qpos = self.init_qpos + self.np_random.uniform(
            low=noise_low, high=noise_high, size=self.model.nq
        )"""
        self.set_state(self.init_qpos, self.init_qvel)

        observation = self._get_obs()
        return observation

#problem = "InvertedPendulum-v4"#"Pendulum-v1"
env = QRPfRA()#(use_contact_forces=True, terminate_when_unhealthy=True)#gym.make(problem)
env.frame_skip = 1
env.render_mode = "human"

num_states = env.observation_space.shape[0]
print("Size of State Space ->  {}".format(num_states))
num_actions = env.action_space.shape[0]
print("Size of Action Space ->  {}".format(num_actions))

upper_bound = env.action_space.high[1]
lower_bound = env.action_space.low[1]

print("Max Value of Action ->  {}".format(upper_bound))
print("Min Value of Action ->  {}".format(lower_bound))


class OUActionNoise:
    def __init__(self, mean, std_deviation, theta=0.15, dt=1e-2, x_initial=None):
        self.theta = theta
        self.mean = mean
        self.std_dev = std_deviation
        self.dt = dt
        self.x_initial = x_initial
        self.reset()

    def __call__(self):
        # Formula taken from https://www.wikipedia.org/wiki/Ornstein-Uhlenbeck_process.
        x = (
            self.x_prev
            + self.theta * (self.mean - self.x_prev) * self.dt
            + self.std_dev * np.sqrt(self.dt) * np.random.normal(size=self.mean.shape)
        )
        # Store x into x_prev
        # Makes next noise dependent on current one
        self.x_prev = x
        return x

    def reset(self):
        if self.x_initial is not None:
            self.x_prev = self.x_initial
        else:
            self.x_prev = np.zeros_like(self.mean)

class Buffer:
    def __init__(self, buffer_capacity=100000, batch_size=64):
        # Number of "experiences" to store at max
        self.buffer_capacity = buffer_capacity
        # Num of tuples to train on.
        self.batch_size = batch_size

        # Its tells us num of times record() was called.
        self.buffer_counter = 0

        # Instead of list of tuples as the exp.replay concept go
        # We use different np.arrays for each tuple element
        self.state_buffer = np.zeros((self.buffer_capacity, num_states))
        self.action_buffer = np.zeros((self.buffer_capacity, num_actions))
        self.reward_buffer = np.zeros((self.buffer_capacity, 1))
        self.next_state_buffer = np.zeros((self.buffer_capacity, num_states))

    # Takes (s,a,r,s') obervation tuple as input
    def record(self, obs_tuple):
        # Set index to zero if buffer_capacity is exceeded,
        # replacing old records
        index = self.buffer_counter % self.buffer_capacity

        self.state_buffer[index] = obs_tuple[0]
        self.action_buffer[index] = obs_tuple[1]
        self.reward_buffer[index] = obs_tuple[2]
        self.next_state_buffer[index] = obs_tuple[3]

        self.buffer_counter += 1

    # Eager execution is turned on by default in TensorFlow 2. Decorating with tf.function allows
    # TensorFlow to build a static graph out of the logic and computations in our function.
    # This provides a large speed up for blocks of code that contain many small TensorFlow operations such as this one.
    @tf.function
    def update(
        self, state_batch, action_batch, reward_batch, next_state_batch,
    ):
        # Training and updating Actor & Critic networks.
        # See Pseudo Code.
        with tf.GradientTape() as tape:
            target_actions = target_actor(next_state_batch, training=True)
            y = reward_batch + gamma * target_critic(
                [next_state_batch, target_actions], training=True
            )
            critic_value = critic_model([state_batch, action_batch], training=True)
            critic_loss = tf.math.reduce_mean(tf.math.square(y - critic_value))

        critic_grad = tape.gradient(critic_loss, critic_model.trainable_variables)
        critic_optimizer.apply_gradients(
            zip(critic_grad, critic_model.trainable_variables)
        )

        with tf.GradientTape() as tape:
            actions = actor_model(state_batch, training=True)
            critic_value = critic_model([state_batch, actions], training=True)
            # Used `-value` as we want to maximize the value given
            # by the critic for our actions
            actor_loss = -tf.math.reduce_mean(critic_value)

        actor_grad = tape.gradient(actor_loss, actor_model.trainable_variables)
        actor_optimizer.apply_gradients(
            zip(actor_grad, actor_model.trainable_variables)
        )

    # We compute the loss and update parameters
    def learn(self):
        # Get sampling range
        record_range = min(self.buffer_counter, self.buffer_capacity)
        # Randomly sample indices
        batch_indices = np.random.choice(record_range, self.batch_size)

        # Convert to tensors
        state_batch = tf.convert_to_tensor(self.state_buffer[batch_indices])
        action_batch = tf.convert_to_tensor(self.action_buffer[batch_indices])
        reward_batch = tf.convert_to_tensor(self.reward_buffer[batch_indices])
        reward_batch = tf.cast(reward_batch, dtype=tf.float32)
        next_state_batch = tf.convert_to_tensor(self.next_state_buffer[batch_indices])

        self.update(state_batch, action_batch, reward_batch, next_state_batch)


# This update target parameters slowly
# Based on rate `tau`, which is much less than one.
@tf.function
def update_target(target_weights, weights, tau):
    for (a, b) in zip(target_weights, weights):
        a.assign(b * tau + a * (1 - tau))



def get_actor():
    # Initialize weights between -3e-3 and 3-e3
    last_init = tf.random_uniform_initializer(minval=-0.003, maxval=0.003)

    inputs = layers.Input(shape=(num_states,))
    out = layers.Dense(2*num_states, activation="relu")(inputs)
    drop1 = layers.Dropout(0.2)(out)
    out2 = layers.Dense(3*num_states, activation="tanh")(drop1)
    drop2 = layers.Dropout(0.2)(out2)
    out3 = layers.Dense(3*num_states, activation="sigmoid")(drop2)
    drop3 = layers.Dropout(0.2)(out3)
    out4 = layers.Dense(2*num_states, activation="tanh")(drop3)
    drop4 = layers.Dropout(0.2)(out4)
    out5 = layers.Dense(2 * num_actions, activation="relu")(drop4)
    drop5 = layers.Dropout(0.2)(out5)
    outputs = layers.Dense(num_actions, kernel_initializer=last_init, activation="sigmoid")(drop5) #add activation activation="sigmoid"

    outputs = outputs * upper_bound
    model = tf.keras.Model(inputs, outputs)
    return model


def get_critic():
    # State as input
    state_input = layers.Input(shape=(num_states))
    state_out1 = layers.Dense(num_states, activation="relu")(state_input)
    drop_state_1 = layers.Dropout(0.2)(state_out1)
    state_out2 = layers.Dense(num_states*2, activation="relu")(drop_state_1)
    drop_state_2 = layers.Dropout(0.2)(state_out2)
    state_out3 = layers.Dense(num_states * 2, activation="relu")(drop_state_2)

    # Action as input
    action_input = layers.Input(shape=(num_actions))
    action_out1 = layers.Dense(num_actions, activation="relu")(action_input)
    action_out2 = layers.Dense(num_actions*2, activation="relu")(action_out1)

    # Both are passed through seperate layer before concatenating
    concat = layers.Concatenate()([state_out3, action_out2])

    out = layers.Dense((num_states*2+num_actions*2), activation="relu")(concat)
    out2 = layers.Dropout(0.2)(out)
    out3 = layers.Dense((num_states+num_actions), activation="relu")(out2)
    out4 = layers.Dropout(0.2)(out3)
    out5 = layers.Dense((num_states+num_actions)/2, activation="relu")(out4)
    outputs = layers.Dense(1)(out5)

    # Outputs single value for give state-action
    model = tf.keras.Model([state_input, action_input], outputs)

    return model


def policy(state, noise_object):
    sampled_actions = tf.squeeze(actor_model(state))
    noise = noise_object()
    # Adding noise to action
    sampled_actions = sampled_actions.numpy() + noise

    # We make sure action is within bounds
    legal_action = np.clip(sampled_actions, lower_bound, upper_bound)

    return [np.squeeze(legal_action)]


std_dev = 0.1
ou_noise = OUActionNoise(mean=np.zeros(1), std_deviation=float(std_dev) * np.ones(1), dt=QRPfRA().dt)

actor_model = get_actor()
critic_model = get_critic()

target_actor = get_actor()
target_critic = get_critic()

# Making the weights equal initially
target_actor.set_weights(actor_model.get_weights())
target_critic.set_weights(critic_model.get_weights())

# Learning rate for actor-critic models
critic_lr = 0.002 #0.0002
actor_lr = 0.001 #0.0001

critic_optimizer = tf.keras.optimizers.Adam(critic_lr)
actor_optimizer = tf.keras.optimizers.Adam(actor_lr)

total_episodes = 1000
# Discount factor for future rewards
gamma = 0.99
# Used to update target networks
tau = 0.005 #0.0005

buffer = Buffer()


# To store reward history of each episode
ep_reward_list = []
# To store average reward history of last few episodes
avg_reward_list = []

# Takes about 4 min to train
for ep in range(total_episodes):

    prev_state = env.reset()
    episodic_reward = 0

    while True:
        # Uncomment this to see the Actor in action
        # But not in a python notebook.
        # env.render()
        #print(f"previous_step = {prev_state}")
        #prev_state, _ = prev_state
        if len(prev_state) == 2:
            prev_state, _ = prev_state
        else:
            prev_state = prev_state
        #print(f"previous_step object = {prev_state}")
        tf_prev_state = tf.expand_dims(tf.convert_to_tensor(prev_state), 0)

        action = policy(tf_prev_state, ou_noise)
        action = np.array(list(action))
        # Recieve state and reward from environment.
        state, reward, done, _, info = env.step(action)

        buffer.record((prev_state, action, reward, state))
        episodic_reward += reward

        buffer.learn()
        update_target(target_actor.variables, actor_model.variables, tau)
        update_target(target_critic.variables, critic_model.variables, tau)

        # End this episode when `done` is True
        if done:
            break

        prev_state = state


    ep_reward_list.append(episodic_reward)
    # Mean of last 10 episodes
    avg_reward = np.mean(ep_reward_list[-15:]) #avg reward calculation regarding past episodic rewards
    print("Episode = {}, Episodic_Reward={}, Avg Reward is ==> {}".format(ep, episodic_reward, avg_reward))
    avg_reward_list.append(avg_reward)

    if ep % 100 == 0:
        plt.clf()
        plt.plot(avg_reward_list)
        plt.xlabel("Episode")
        plt.ylabel("Avg. Epsiodic Reward")
        plt.pause(0.0001)




# Plotting graph
# Episodes versus Avg. Rewards
plt.plot(avg_reward_list)
plt.xlabel("Episode")
plt.ylabel("Avg. Epsiodic Reward")
plt.show()

# Save the weights
actor_model.save_weights("denemexml_actor.h5")
critic_model.save_weights("denemexml_critic.h5")

target_actor.save_weights("denemexml_target_actor.h5")
target_critic.save_weights("denemexml_target_critic.h5")



