import pylab as plt
from casadi import *
from constants import constants

def solve_trajectory(intial_velocity, reference):
    N = 100
    T = 5
    dt = T/N

    solver = Opti()

    # State variable - velocity for a flywheel
    X = solver.variable(N+1)

    # Control variable - voltage for flywheel
    U = solver.variable(N)

    # Dynamics (solution to differential equation V = kv * v + ka * a)
    x_next = lambda x, u: vertcat(
        ((x - u / constants.kv) * exp(-constants.kv / constants.ka * dt) + u / constants.kv)
    )

    for k in range(N):
        solver.subject_to(X[k+1]==x_next(X[k],U[k]))

    # Cost function - minimizing error, control effort is not included because hard voltage limits can be set later
    J = 0
    for k in range(N+1):
        J += (reference - X[k]) * (reference - X[k])
    solver.minimize(J)

    # Constraints
    solver.subject_to(X[0]==intial_velocity)
    solver.subject_to(solver.bounded(-12, U, 12))

    # Solve non-linear problem
    solver.solver("ipopt")
    sol = solver.solve()

    plt.plot([0,T],[reference,reference], label="Reference")
    plt.plot(np.linspace(0,N,(N+1)) * dt, sol.value(X), "--", label="Transcription MPC")
    plt.legend()