from creep.machine import CreepRobot

class BaseStrategy:
    def strategy_base(self, creep: CreepRobot):
        raise NotImplementedError("Subclasses should implement this method")
    
    def strategy_acid(self, creep: CreepRobot):
        raise NotImplementedError("Subclasses should implement this method")