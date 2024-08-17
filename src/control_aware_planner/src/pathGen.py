from ruckig import InputParameter, Ruckig, Trajectory, Result

class PathGen:
    def __init__(self, dim, max_vel = None, max_acc = None, max_jerk = None):
        self.dim = dim

        max_vel = [1.0] * dim if max_vel is None else max_vel
        max_acc = [1.0] * dim if max_acc is None else max_acc
        max_jerk = [1.0] * dim if max_jerk is None else max_jerk

        self.inp = InputParameter(dim)

        self.inp.max_velocity = max_vel
        self.inp.max_acceleration = max_acc
        self.inp.max_jerk = max_jerk

        self.otg = Ruckig(self.dim)
        self.trj = Trajectory(self.dim)
        self.duration = None

    def generate_path(self, start_pos, start_vel, start_acc, goal_pos, goal_vel, goal_acc):
        """
        Generate a path from start_pos to goal_pos with start_vel, start_acc, goal_vel, goal_acc as initial conditions.
        """

        self.inp.current_position = start_pos
        self.inp.current_velocity = start_vel
        self.inp.current_acceleration = start_acc
        
        self.inp.target_position = goal_pos
        self.inp.target_velocity = goal_vel
        self.inp.target_acceleration = goal_acc
        

        result = self.otg.calculate(self.inp, self.trj)
        if result == Result.ErrorInvalidInput:
            raise Exception("Invalid input parameters")
        self.start = [start_pos, start_vel, start_acc]
        self.goal = [goal_pos, goal_vel, goal_acc]
        self.duration = self.trj.duration

    def set_bounds(self, max_vel, max_acc, max_jerk):
        self.inp.max_velocity = max_vel
        self.inp.max_acceleration = max_acc
        self.inp.max_jerk = max_jerk

    def get_duration(self):
        """
        Get the duration of the generated path.
        """
        return self.duration

    def get_waypoints_with_time(self,time):
        """
        Extract waypoints from the generated path with the given time.
        """
        return self.trj.at_time(time)

    def get_waypoints_with_scale(self, scale):
        """
        Extract waypoints from the generated path with the given scale (0 to 1).
        """
        return self.trj.at_time(scale*self.duration)
    
    def get_trj_extrema(self):
        return self.trj.extrema()

    



