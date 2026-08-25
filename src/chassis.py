class ChassisController:

    def __init__(self, ep_robot):
        self.chassis = ep_robot.chassis

    def move_forward(self, distance, speed):

        print(f"[Chassis] Moving forward: distance={distance}m, speed={speed}m/s")
        self.chassis.move(x=distance, y=0, z=0, xy_speed=speed).wait_for_completed()

    def move_backward(self, distance, speed):

        print(f"[Chassis] Moving backward: distance={distance}m, speed={speed}m/s")
        self.chassis.move(x=-distance, y=0, z=0, xy_speed=speed).wait_for_completed()

    def turn_Right(self, angle, speed):

        print(f"[Chassis] Turning right: angle={angle}, speed={speed}rad/s")
        self.chassis.move(x=0, y=0, z=-angle, z_speed=45).wait_for_completed()

    def turn_Left(self, angle, speed):

        print(f"[Chassis] Turning left: angle={angle}, speed={speed}rad/s")
        self.chassis.move(x=0, y=0, z=angle, z_speed=45).wait_for_completed()
