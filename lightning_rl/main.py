import gym
from stable_baselines3 import PPO
from gym.envs.registration import register

def main():

    register(
        id='SimpleDriving-v0',
        entry_point='simple_driving.envs:SimpleDrivingEnv'
    )

    env = gym.make('SimpleDriving-v0')


    agent = PPO("MlpPolicy", env, verbose=1)
    agent.learn(total_timesteps=100)

    obs = env.reset()
    for i in range(100):
        action, _states = agent.predict(obs, deterministic=True)
        obs, reward, done, info = env.step(action)
        env.render()
        if done:
            obs = env.reset()

    # env.close()


if __name__ == '__main__':
    main()