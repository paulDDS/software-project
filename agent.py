from utils.ssl.Navigation import Navigation
from utils.ssl.AgentNavigation import AgentNavigation
from utils.ssl.base_agent import BaseAgent

class ExampleAgent(BaseAgent):
    def __init__(self, id=0, yellow=False):
        super().__init__(id, yellow)

    def decision(self):
        if len(self.targets) == 0:
            return

        target_velocity, target_angle_velocity = AgentNavigation.goToPoint(self.robot, self.targets[0], self.opponents, self.teammates)
        self.set_vel(target_velocity)
        self.set_angle_vel(target_angle_velocity)

        return

    def post_decision(self):
        pass
