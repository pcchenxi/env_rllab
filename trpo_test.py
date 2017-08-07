from sandbox.rocky.tf.algos.trpo import TRPO
# from rllab.algos.trpo import TRPO
from rllab.baselines.linear_feature_baseline import LinearFeatureBaseline
from rllab.envs.normalized_env import normalize
# from sandbox.rocky.tf.policies.categorical_mlp_policy import CategoricalMLPPolicy
# from rllab.policies.categorical_mlp_policy import CategoricalMLPPolicy
from model.conv_merge_policy import ConvMergePolicy
from rllab.misc.instrument import run_experiment_lite

import joblib
from environment import env_init
from environment import env_vrep

# def run_task(*_):
env = normalize(env_vrep.Simu_env(20000))
# env = env_vrep.Simu_env(20000)

policy = ConvMergePolicy(
    name = 'policy',
    env_spec=env.spec,
)

# policy = joblib.load('data/policy_params.pkl')

baseline = LinearFeatureBaseline(env_spec=env.spec)

algo = TRPO(
    env=env,
    policy=policy,
    baseline=baseline,
    batch_size=1000,
    max_path_length=200,
    n_itr=500,
    discount=0.99,
    # plot = True
    # step_size=0.05,
)
algo.train()

# run_experiment_lite(
#     run_task,
#     # Number of parallel workers for sampling
#     n_parallel=1,
#     # Only keep the snapshot parameters for the last iteration
#     snapshot_mode="last",
#     # Specifies the seed for the experiment. If this is not provided, a random seed
#     # will be used
#     seed=1,
#     # plot=True,
# )

