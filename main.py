import fast_mpc
import transcription_mpc
import pylab as plt

fast_mpc.solve_trajectory(0, 10)
transcription_mpc.solve_trajectory(0,10)
plt.show()