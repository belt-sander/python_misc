import numpy as np
import matplotlib.pyplot as plt

from scipy.signal import savgol_filter
from pykalman import KalmanFilter

rnd = np.random.RandomState(0)

# generate a noisy sine wave to act as our fake observations
n_timesteps = 101
x = np.linspace(0, 3 * np.pi, n_timesteps)
observations = 20 * (np.sin(x) + 0.5 * rnd.randn(n_timesteps))

# create a Kalman Filter by hinting at the size of the state and observation
# space.  If you already have good guesses for the initial parameters, put them
# in here.  The Kalman Filter will try to learn the values of all variables.
kf = KalmanFilter(transition_matrices=np.array([[1, 1], [0, 1]]),
                  transition_covariance=0.01 * np.eye(2))

# You can use the Kalman Filter immediately without fitting, but its estimates
# may not be as good as if you fit first.
states_pred = kf.em(observations).smooth(observations)[0]
print('fitted model: {0}'.format(kf))

# Plot lines for the observations without noise, the estimated position of the
# target before fitting, and the estimated position after fitting.

plt.style.use('dark_background')

plt.figure(figsize=(16, 6))
obs_scatter = plt.scatter(x, observations, marker='x', color='b',
                         label='observations')
position_line = plt.plot(x, states_pred[:, 0],
                        linestyle='-', marker='o', color='r',
                        label='position est.')
# velocity_line = plt.plot(x, states_pred[:, 1],
#                         linestyle='-', marker='o', color='g',
#                         label='velocity est.')
savgol_line_1 = plt.plot(x, (savgol_filter(observations,55,1)), 
						label='savgol filter 555')
savgol_line_2 = plt.plot(x, (savgol_filter(observations,41,1)), 
						label='savgol filter 441')
savgol_line_3 = plt.plot(x, (savgol_filter(observations,33,1)), 
						label='savgol filter 333')
savgol_line_4 = plt.plot(x, (savgol_filter(observations,21,1)), 
						label='savgol filter 221')
savgol_line_5 = plt.plot(x, (savgol_filter(observations,11,1)), 
						label='savgol filter 111')
plt.legend(loc='lower right')
plt.xlim(xmin=0, xmax=x.max())
plt.xlabel('time')

plt.show()
