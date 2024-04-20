import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import tf_transformations
from pyproj import Proj, transform

class GPSPathGenerator(Node):
    def __init__(self):
        super().__init__('gps_path_generator')

        # WGS84座標系（GPSの座標系）
        self.wgs84 = Proj(init='epsg:4326')

        # UTM座標系への変換（ここではゾーン33Nを例とする）
        self.utm = Proj(init='epsg:32633')  # UTMゾーンに応じてEPSGコードを変更
        
        gps_data = [...] # 例: [(lat1, lon1), (lat2, lon2), ...]
        
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'map'
        
        for lat, lon in gps_data:
            # 緯度経度をロボット座標系に変換
            x, y = self.geodetic_to_robot(lat, lon)
            
            # PoseStampedを作成し、経路に追加
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'map'
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation = tf_transformations.quaternion_from_euler(0, 0, 0)
            path.poses.append(pose)
            
        # 経路の平滑化処理 (オプション)
        path = self.smooth_path(path)
        
        # 経路を出力
        self.publish_path(path)
        
    def convert_gps_to_utm(self, lat, lon):
        x, y = transform(self.wgs84, self.utm, lon, lat)
        return x, y
        
    def smooth_path(self, path):
        # 経路を平滑化するコードを記述
        return path
        
    def publish_path(self, path):
        # 生成した経路を出力するコードを記述
        pass
        
def main():
    rclpy.init()
    generator = GPSPathGenerator()
    rclpy.spin(generator)

if __name__ == '__main__':
    main()