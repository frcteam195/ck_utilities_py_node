import rospy
from threading import Lock
from ck_utilities_py_node.geometry import *
from visualization_msgs.msg import *
import tf
import rospy

class Color:
    def __init__(self, r : float, g : float, b, a : float):
        self.r = r
        self.g = g
        self.b = b
        self.a = a

class ShapeManager:
    __static_shape_publisher = None
    __dynamic_shape_publisher = None

    __static_shape_map = None

    def __init__(self):
        self.__class__.__static_shape_publisher = rospy.Publisher(
            name = "/static_shapes", data_class=MarkerArray, queue_size=1, tcp_nodelay=True, latch=True)
        self.__class__.__dynamic_shape_publisher = rospy.Publisher(
            name = "/shapes", data_class=Marker, queue_size=100, tcp_nodelay=True)
        self.__class__.__static_shape_map = {}

    def publish_static_shape(self, marker : Marker):
        self.__class__.__static_shape_map[str(marker.ns) + str(marker.id)] = marker

        transmit_array = MarkerArray()
        for transmit_marker_key, transmit_marker in self.__class__.__static_shape_map.items():
            transmit_array.markers.append(transmit_marker)

        # rospy.loginfo(self.__class__.__static_shape_map)
        # rospy.loginfo(transmit_array)

        self.__class__.__static_shape_publisher.publish(transmit_array)

    def publish_dynamic_shape(self, marker : Marker):
        self.__class__.__dynamic_shape_publisher.publish(marker)

class ShapeBase:
    __mutex = Lock()
    __manager = None

    def __init__(self, namespace : str, id : int, base_frame : str, type : int):
        self.__transform = Transform()
        self.__base_frame = base_frame
        self.__namespace = namespace
        self.__id = id
        self.__type = type
        self.__scale = Scale()
        self.__color = Color(1.0, 1.0, 1.0, 1.0)
        self.spawn_shape_manager()

    @classmethod
    def spawn_shape_manager(cls):
        with cls.__mutex:
            if cls.__manager is None:
                cls.__manager = ShapeManager()

    def set_transform(self, transform : Transform):
        self.__transform = transform

    def set_scale(self, scale : Scale):
        self.__scale = scale

    def set_color(self, color : Color):
        self.__color = color

    def convert_to_marker(self):
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = self.__base_frame
        marker.action = Marker.ADD
        marker.ns = self.__namespace
        marker.id = self.__id
        marker.type = self.__type
        marker.pose.position.x = self.__transform.linear.x
        marker.pose.position.y = self.__transform.linear.y
        marker.pose.position.z = self.__transform.linear.z
        quat = tf.transformations.quaternion_from_euler(
            float(self.__transform.angular.roll),
            float(self.__transform.angular.pitch),
            float(self.__transform.angular.yaw))
        marker.pose.orientation.x = quat[0]
        marker.pose.orientation.y = quat[1]
        marker.pose.orientation.z = quat[2]
        marker.pose.orientation.w = quat[3]
        marker.scale.x = self.__scale.x
        marker.scale.y = self.__scale.y
        marker.scale.z = self.__scale.z
        marker.color.r = self.__color.r
        marker.color.g = self.__color.g
        marker.color.b = self.__color.b
        marker.color.a = self.__color.a
        marker.frame_locked = True
        return marker

    @classmethod
    def manager(cls) -> ShapeManager:
        return cls.__manager

class Sphere(ShapeBase):
    def __init__(self, namespace : str, id : int, base_frame : str):
        super().__init__(namespace, id, base_frame, 2)

    def publish(self):
        self.manager().publish_dynamic_shape(self.convert_to_marker())

class StaticSphere(ShapeBase):
    def __init__(self, namespace : str, id : int, base_frame : str):
        super().__init__(namespace, id, base_frame, 2)

    def publish(self):
        self.manager().publish_static_shape(self.convert_to_marker())