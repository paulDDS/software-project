from utils.ssl.Navigation import Navigation
from utils.ssl.AgentNavigation import AgentNavigation
from utils.ssl.base_agent import BaseAgent
from Coordination import Coordination

class ExampleAgent(BaseAgent):
    
    Coordination_helper = Coordination()

    def __init__(self, id=0, yellow=False):
        super().__init__(id, yellow)
        self.helper = ExampleAgent.Coordination_helper

    def get_target(self):
        return

    def decision(self):
        if len(self.targets) == 0:
            return
        
        target = self.helper.know_target(self.id)
        target_velocity, target_angle_velocity = AgentNavigation.goToPoint(self.robot, target, self.opponents, self.teammates)
        self.set_vel(target_velocity)
        self.set_angle_vel(target_angle_velocity)

        return

    def post_decision(self):
        pass
