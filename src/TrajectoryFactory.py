import pathfinder

class TrajectoryFactory:

    def __init__(self, filepath, wheelbase):
        self.traj_dict = {}
        self.filepath = filepath
        self.wheelbase = wheelbase
        
    
    def add_trajectory(self, name, traj):
        self.traj_dict.update({name + "_source.traj": traj})
        tank_modifier = pathfinder.modifiers.TankModifier(traj).modify(self.wheelbase)
        self.traj_dict.update({name + "_left.traj" : tank_modifier.getLeftTrajectory()})
        self.traj_dict.update({name + "_right.traj" : tank_modifier.getRightTrajectory()})

    def save_trajectories(self):
        for filename, traj in self.traj_dict.items():
            pathfinder.serialize(self.filepath + filename, traj)


    
