import rospy

class PIDController:
    def __init__(self, kP : float = 0, kI : float = 0, kD : float = 0, kF : float = 0, filter_r : float = 0) -> None:
        self.kP = kP
        self.kI = kI
        self.kD = kD
        self.kF = kF
        self.__filter_r = filter_r
        self.__error = 0
        self.__error_sum = 0
        self.__error_last = 0
        self.__error_d = 0
        self.__actual = 0
        self.__setpoint = 0
        self.__last_time = 0

    def set_gains(self,  kP : float, kI : float, kD : float, kF : float):
        self.__kP = kP
        self.__kI = kI
        self.__kD = kD
        self.__kF = kF
    
    def set_filter(self, filter_r : float):
        self.__filter_r = filter_r

    def update(self, setpoint : float, actual : float) -> float:
        self.__setpoint = setpoint
        self.__actual = actual

        self.__error = self.__setpoint - self.__actual

        self.__error_sum = self.__error_sum + self.__error
        self.__error_d = self.__error_d + (1 - self.__filter_r) * (self.__error - self.__error_last)
        self.__error_last = self.__error

        time = rospy.get_time()
        dt = time - self.__last_time
        self.__last_time = time

        return self.__error * self.__kP + self.__error_sum * self.__kI + self.__error_d * self.__kD + self.__kF * self.__setpoint

    def update_by_error(self, error : float) -> float:
        self.__error = error
        self.__error_sum += self.__error_sum + self.__error
        self.__error_d = (1 - self.__filter_r) * (self.__error - self.__error_last)
        self.__error_last = self.__error

        time = rospy.get_time()
        dt = time - self.__last_time
        self.__last_time = time

        return self.__error * self.__kP + self.__error_sum * self.__kI + self.__error_d * self.__kD;