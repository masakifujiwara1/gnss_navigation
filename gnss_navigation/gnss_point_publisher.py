import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped, Point
# import tf_transformations
from pyproj import Proj, transform
import pandas as pd
from scipy.interpolate import splprep, splev
import numpy as np

class GPSPointPublisher(Node):
    def __init__(self):
        super().__init__('gps_point_publisher')

        # WGS84座標系（GPSの座標系）
        self.wgs84 = Proj(init='epsg:4326')

        # UTM座標系への変換（ここではゾーン33Nを例とする）
        self.utm = Proj(init='epsg:32654')  # UTMゾーンに応じてEPSGコードを変更

        self.base_x = 408334.8343140974
        self.base_y = 3997004.1802519667

        self.latitude = 0
        self.longitude = 0

        self.gnss_point = Point()

        # 緯度経度をロボット座標系に変換
        # x, y = self.convert_gps_to_utm(data[1], data[0])

        self.subscriber = self.create_subscription(NavSatFix, 'gps/fix', self.listener_callback, 10)

        self.publisher_ = self.create_publisher(Point, 'gnss_point', 10)
        self.timer = self.create_timer(1, self.publish_path)
        # self.publish_path(smooth_path)

    def listener_callback(self, data):
        self.latitude = data.latitude
        self.longitude = data.longitude
        # print(self.latitude, self.longitude)
        
    def convert_gps_to_utm(self, lat, lon):
        x, y = transform(self.wgs84, self.utm, lat, lon)
        return x, y
               
    def publish_path(self):
        x, y = self.convert_gps_to_utm(self.longitude, self.latitude)
        self.Point = Point()
        self.Point.x = x - self.base_x
        self.Point.y = y - self.base_y
        self.publisher_.publish(self.Point)
        self.get_logger().info("pulish point")
        
def main():
    try:
        rclpy.init()
        generator = GPSPointPublisher()
        rclpy.spin(generator)
    except KeyboardInterrupt:
        print("ctrl-C")
    finally:
        generator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()