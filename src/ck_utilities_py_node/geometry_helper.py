import numpy
from ck_utilities_py_node.geometry import *
from ck_utilities_py_node.ckmath import *

class RectangularRange2D:
    def __init__(self, rectangle_inches : Rectangle) -> None:
        self.__A = [inches_to_meters(rectangle_inches.lower_right_x), inches_to_meters(rectangle_inches.lower_right_y)]
        self.__B = [inches_to_meters(rectangle_inches.lower_right_x), inches_to_meters(rectangle_inches.upper_left_y)]
        self.__C = [inches_to_meters(rectangle_inches.upper_left_x), inches_to_meters(rectangle_inches.upper_left_y)]

        self.__AB = numpy.subtract(self.__B, self.__A)
        self.__BC = numpy.subtract(self.__C, self.__B)
        self.__dotABAB = numpy.dot(self.__AB, self.__AB)
        self.__dotBCBC = numpy.dot(self.__BC, self.__BC)
        pass

    def is_within_range_2d(self, pose_meters : Pose) -> bool:
        M = [pose_meters.position.x, pose_meters.position.y]
        AM = numpy.subtract(M, self.__A)
        BM = numpy.subtract(M, self.__B)
        dotABAM = numpy.dot(self.__AB, AM)
        dotBCBM = numpy.dot(self.__BC, BM)
        return 0 <= dotABAM and dotABAM <= self.__dotABAB and 0 <= dotBCBM and dotBCBM <= self.__dotBCBC