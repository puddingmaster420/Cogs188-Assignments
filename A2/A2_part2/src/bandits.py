import numpy as np
import pandas as pd
from typing import List


def update(q: float, r: float, k: int) -> float:
    """
    Update the Q-value using the given reward and number of times the action has been taken.

    Parameters:
    q (float): The current Q-value.
    r (float): The reward received for the action.
    k (int): The number of times the action has been taken before.

    Returns:
    float: The updated Q-value.
    """
    # Note: since k is the number of times the action has been taken before this update, we need to add 1 to k before using it in the formula.
    k += 1
    alpha = 1.0 / k
    return q + alpha * (r - q)


def greedy(q_estimate: np.ndarray) -> int:
    """
    Selects the action with the highest Q-value.

    Parameters:
    q_estimate (numpy.ndarray): 1-D Array of Q-values for each action.

    Returns:
    int: The index of the action with the highest Q-value.
    """
    max_q = np.max(q_estimate)
    best_actions = np.flatnonzero(q_estimate == max_q)
    return int(np.random.choice(best_actions))


def egreedy(q_estimate: np.ndarray, epsilon: float) -> int:
    """
    Implements the epsilon-greedy exploration strategy for multi-armed bandits.

    Parameters:
    q_estimate (numpy.ndarray): 1-D Array of estimated action values.
    epsilon (float): Exploration rate, determines the probability of selecting a random action.
    n_arms (int): Number of arms in the bandit. default is 10.

    Returns:
    int: The index of the selected action.
    """
    n_arms = len(q_estimate)
    if np.random.rand() < epsilon:
        greedy_action = greedy(q_estimate)
        other_actions = [a for a in range(n_arms) if a != greedy_action]
        if other_actions:
            return int(np.random.choice(other_actions))
        return greedy_action
    else:
        return greedy(q_estimate)


def empirical_egreedy(epsilon: float, n_trials: int, n_arms: int, n_plays: int) -> List[List[float]]:
    """
    Run the epsilon-greedy algorithm on a multi-armed bandit problem. For each play,
    the algorithm selects an action based on the epsilon-greedy strategy and updates
    the Q-value of the selected action. For each trial, the algorithm returns
    the rewards for each play, a total of n_plays.

    Args:
        epsilon (float): epsilon value for the epsilon-greedy algorithm.
        n_trials (int): number of trials to run the algorithm
        n_arms (int): number of arms in the bandit
        n_plays (int): number of plays in each trial

    Returns:
        List[List[float]]: A list of rewards for each play in each trial.
    """
    rewards = []  # stores the rewards for each trial

    for _ in range(n_trials):
        true_rewards = np.random.normal(size=n_arms)
        q_estimate = np.zeros(n_arms)
        counts = np.zeros(n_arms, dtype=int)
        trial_rewards = []
        
        for _ in range(n_plays):
            action = egreedy(q_estimate, epsilon)
            reward = np.random.normal(loc=true_rewards[action], scale=1.0)
            trial_rewards.append(reward)
            
            # Update Q-value and increment action count
            q_estimate[action] = update(q_estimate[action], reward, counts[action])
            counts[action] += 1
            
        rewards.append(trial_rewards)

    return rewards
