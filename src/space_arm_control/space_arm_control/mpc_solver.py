"""
CasADi-based Model Predictive Controller for the Space Arm.

Formulates and solves a nonlinear optimal control problem at each timestep.
The dynamics model is a simple double-integrator per joint (appropriate for
microgravity where gravitational torques are negligible).

State:  x = [q1..q6, dq1..dq6]  (12 states)
Control: u = [tau1..tau6]         (6 inputs)
"""
import casadi as ca
import numpy as np


class MPCSolver:
    """Nonlinear MPC solver using CasADi with IPOPT backend."""

    def __init__(
        self,
        num_joints: int = 6,
        prediction_horizon: int = 20,
        control_horizon: int = 10,
        dt: float = 0.05,
        torque_limits: np.ndarray = None,
        velocity_limits: np.ndarray = None,
        state_weight: np.ndarray = None,
        control_weight: np.ndarray = None,
        terminal_weight: np.ndarray = None,
    ):
        self.n = num_joints
        self.N = prediction_horizon
        self.M = min(control_horizon, prediction_horizon)
        self.dt = dt

        # Default limits
        if torque_limits is None:
            torque_limits = np.full(num_joints, 100.0)
        if velocity_limits is None:
            velocity_limits = np.full(num_joints, 2.0)
        if state_weight is None:
            state_weight = np.full(num_joints, 10.0)
        if control_weight is None:
            control_weight = np.full(num_joints, 0.01)
        if terminal_weight is None:
            terminal_weight = np.full(num_joints, 50.0)

        self.torque_limits = torque_limits
        self.velocity_limits = velocity_limits

        # Inertia estimates per joint (diagonal approximation, kg*m^2)
        # These can be refined from the URDF or system identification
        self.inertias = np.array([0.5, 0.8, 0.6, 0.2, 0.15, 0.1])

        # Build the optimization problem
        self._build_problem(state_weight, control_weight, terminal_weight)

    def _dynamics(self, x: ca.MX, u: ca.MX) -> ca.MX:
        """
        Continuous-time dynamics: double integrator per joint in microgravity.

        ddq_i = tau_i / I_i  (no gravity torque in space)

        Can be extended with:
        - Coriolis/centrifugal terms
        - Joint friction models
        - Flexible joint dynamics
        - Reaction wheel coupling (for free-floating base)
        """
        q = x[:self.n]       # positions
        dq = x[self.n:]      # velocities

        # Acceleration: torque / inertia (simplified)
        ddq = ca.MX.zeros(self.n)
        for i in range(self.n):
            ddq[i] = u[i] / self.inertias[i]

        # State derivative: [dq, ddq]
        return ca.vertcat(dq, ddq)

    def _rk4_step(self, x: ca.MX, u: ca.MX) -> ca.MX:
        """4th-order Runge-Kutta integration step."""
        k1 = self._dynamics(x, u)
        k2 = self._dynamics(x + self.dt / 2 * k1, u)
        k3 = self._dynamics(x + self.dt / 2 * k2, u)
        k4 = self._dynamics(x + self.dt * k3, u)
        return x + self.dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)

    def _build_problem(self, Q_diag, R_diag, Qf_diag):
        """Build the CasADi NLP for the MPC problem."""
        nx = 2 * self.n  # state dimension
        nu = self.n       # control dimension

        # Weight matrices
        Q = ca.diag(ca.vertcat(Q_diag, np.ones(self.n)))   # pos + vel weights
        R = ca.diag(R_diag)
        Qf = ca.diag(ca.vertcat(Qf_diag, np.ones(self.n)))

        # Decision variables
        X = ca.MX.sym('X', nx, self.N + 1)  # states
        U = ca.MX.sym('U', nu, self.N)      # controls

        # Parameters: [x0; x_ref]
        P = ca.MX.sym('P', 2 * nx)
        x0 = P[:nx]
        x_ref = P[nx:]

        # Objective and constraints
        obj = 0.0
        g = []       # constraint expressions
        lbg = []     # lower bounds
        ubg = []     # upper bounds

        # Initial state constraint
        g.append(X[:, 0] - x0)
        lbg += [0.0] * nx
        ubg += [0.0] * nx

        for k in range(self.N):
            # State error
            e = X[:, k] - x_ref
            obj += ca.mtimes(e.T, ca.mtimes(Q, e))

            # Control cost
            obj += ca.mtimes(U[:, k].T, ca.mtimes(R, U[:, k]))

            # Dynamics constraint (RK4)
            x_next = self._rk4_step(X[:, k], U[:, k])
            g.append(X[:, k + 1] - x_next)
            lbg += [0.0] * nx
            ubg += [0.0] * nx

        # Terminal cost
        e_f = X[:, self.N] - x_ref
        obj += ca.mtimes(e_f.T, ca.mtimes(Qf, e_f))

        # Flatten decision variables
        opt_vars = ca.vertcat(ca.reshape(X, -1, 1), ca.reshape(U, -1, 1))

        # Build NLP
        nlp = {'f': obj, 'x': opt_vars, 'g': ca.vertcat(*g), 'p': P}

        opts = {
            'ipopt.print_level': 0,
            'print_time': 0,
            'ipopt.max_iter': 50,
            'ipopt.tol': 1e-6,
            'ipopt.warm_start_init_point': 'yes',
        }

        self.solver_nlp = ca.nlpsol('mpc', 'ipopt', nlp, opts)

        # Store dimensions for unpacking
        self.nx = nx
        self.nu = nu
        self.n_opt = opt_vars.shape[0]

        # Variable bounds
        self.lbx = np.full(self.n_opt, -np.inf)
        self.ubx = np.full(self.n_opt, np.inf)

        n_x_vars = nx * (self.N + 1)

        # Velocity bounds within state variables
        for k in range(self.N + 1):
            for i in range(self.n):
                vel_idx = k * nx + self.n + i
                self.lbx[vel_idx] = -self.velocity_limits[i]
                self.ubx[vel_idx] = self.velocity_limits[i]

        # Torque bounds on control variables
        for k in range(self.N):
            for i in range(self.n):
                u_idx = n_x_vars + k * nu + i
                self.lbx[u_idx] = -self.torque_limits[i]
                self.ubx[u_idx] = self.torque_limits[i]

        self.lbg = np.array(lbg)
        self.ubg = np.array(ubg)

        # Warm start storage
        self._prev_sol = None

    def solve(self, x0: np.ndarray, x_ref: np.ndarray):
        """
        Solve the MPC problem for current state x0 and reference x_ref.

        Returns:
            torques: np.ndarray of shape (n,) -- first control action
            info: dict with solver diagnostics
        """
        p = np.concatenate([x0, x_ref])

        # Initial guess: warm start from previous solution or zeros
        if self._prev_sol is not None:
            x0_nlp = self._prev_sol
        else:
            x0_nlp = np.zeros(self.n_opt)
            # Initialize states with current state
            for k in range(self.N + 1):
                x0_nlp[k * self.nx:(k + 1) * self.nx] = x0

        try:
            sol = self.solver_nlp(
                x0=x0_nlp,
                lbx=self.lbx,
                ubx=self.ubx,
                lbg=self.lbg,
                ubg=self.ubg,
                p=p,
            )

            opt = np.array(sol['x']).flatten()
            self._prev_sol = opt  # store for warm start

            # Extract first control action
            n_x_vars = self.nx * (self.N + 1)
            torques = opt[n_x_vars:n_x_vars + self.nu]

            info = {
                'status': 'solved',
                'cost': float(sol['f']),
            }
            return torques, info

        except Exception as e:
            info = {'status': f'failed: {e}'}
            return None, info
