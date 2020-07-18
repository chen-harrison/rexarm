"""!
! Trajectory planner.

TODO: build a trajectory generator and waypoint planner so it allows your state machine to iterate through the plan at
the desired command update rate.
"""
import numpy as np
import time

class TrajectoryPlanner():
    """!
    @brief      This class describes a trajectory planner.
    """

    def __init__(self, rexarm):
        """!
        @brief      Constructs a new instance.

        @param      rexarm  The rexarm
        """
        self.idle = True
        self.rexarm = rexarm
        self.initial_wp = None
        self.final_wp = None
        self.dt = 0.005 # command rate

    def set_initial_wp(self):
        """!
        @brief      TODO: Sets the initial wp to the current position.
        """
        self.initial_wp = self.rexarm.position_fb

    def set_final_wp(self, waypoint):
        """!
        @brief      TODO: Sets the final wp.

        @param      waypoint  The waypoint
        """
        self.final_wp = waypoint

    def go(self, max_speed=2.5):
        """!
        @brief      TODO Plan and execute the trajectory.

        @param      max_speed  The maximum speed
        """
        T = self.calc_time_from_waypoints(self.initial_wp, self.final_wp, max_speed)
        plan = self.generate_cubic_spline(self.initial_wp, self.final_wp, T)
        self.execute_plan(plan)

    def stop(self):
        """!
        @brief      TODO Stop the trajectory planner
        """
        pass

    def calc_time_from_waypoints(self, initial_wp, final_wp, max_speed):
        """!
        @brief      TODO Calculate the time to get from initial to final waypoint.

        @param      initial_wp  The initial wp
        @param      final_wp    The final wp
        @param      max_speed   The maximum speed

        @return     The amount of time to get to the final waypoint.
        """
        disp = np.array(final_wp) - np.array(initial_wp)
        max_disp = max(abs(disp))
        return max_disp/max_speed * 2

    def generate_cubic_spline(self, initial_wp, final_wp, T):
        """!
        @brief      TODO generate a cubic spline

        @param      initial_wp  The initial wp
        @param      final_wp    The final wp
        @param      T           Amount of time to get from initial to final waypoint

        @return     The plan as num_steps x num_joints np.array
        """
        t0 = 0
        v0 = [0]*self.rexarm.num_joints
        vf = [0]*self.rexarm.num_joints

        M = np.array([[1, t0, t0**2, t0**3], [0, 1, 2*t0, 3*t0**2], [1, T, T**2, T**3], [0, 1, 2*T, 3*T**2]])
        b = np.array([initial_wp, v0, final_wp, vf])
        a = np.matmul(np.linalg.inv(M), b)

        num_steps = int(np.round(T / self.dt))

        plan = np.zeros([num_steps, self.rexarm.num_joints])
        for i in range(num_steps):
            t = i * self.dt
            time_vec = np.array([1, t, t**2, t**3])
            plan[i,:] = np.matmul(time_vec, a)

        return plan

    def execute_plan(self, plan, look_ahead=0):
        """!
        @brief      TODO: Execute the planed trajectory.

        @param      plan        The plan
        @param      look_ahead  The look ahead
        """
        num_rows = plan.shape[0]
        for i in range(num_rows):
            rev_i = np.minimum(i + look_ahead, num_rows - 1)
            self.rexarm.set_positions(plan[rev_i,:])
            time.sleep(self.dt)
