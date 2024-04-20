import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
# import tf_transformations
from pyproj import Proj, transform
import pandas as pd
from scipy.interpolate import splprep, splev
import numpy as np

class GPSPathGenerator(Node):
    def __init__(self):
        super().__init__('gps_path_generator')

        # WGS84座標系（GPSの座標系）
        self.wgs84 = Proj(init='epsg:4326')

        # UTM座標系への変換（ここではゾーン33Nを例とする）
        self.utm = Proj(init='epsg:32654')  # UTMゾーンに応じてEPSGコードを変更
        
        # gps_data = [...] # 例: [(lat1, lon1), (lat2, lon2), ...]
        file_path = '/home/ros2_ws/src/gnss_navigation/config/path_compress.csv'
        df = pd.read_csv(file_path, header=None)     
        
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'map'

        cnt = 1
        
        for index, data in df.iterrows():
            # if cnt % 2 == 0:
            #     cnt += 1
            #     return

            print(index, data[0], data[1])

            # 緯度経度をロボット座標系に変換
            x, y = self.convert_gps_to_utm(data[1], data[0])
            # print(x, y)

            if index == 0:
                base_x = x
                base_y = y
            
            # PoseStampedを作成し、経路に追加
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'map'
            pose.pose.position.x = x - base_x
            pose.pose.position.y = y - base_y
            # pose.pose.orientation = tf_transformations.quaternion_from_euler(0, 0, 0)
            path.poses.append(pose)

            cnt += 1
            
        # # 経路の平滑化処理 (オプション)
        smooth_path = self.smooth_path(path)
        
        # 経路を出力
        # self.publish_path(path)

        self.publisher_ = self.create_publisher(Path, 'gnss_path', 10)
        # self.timer = self.create_timer(1, self.publish_path)
        self.publish_path(smooth_path)
        
    def convert_gps_to_utm(self, lat, lon):
        x, y = transform(self.wgs84, self.utm, lat, lon)
        return x, y
        
    def smooth_path(self, path):
        # 経路を平滑化するコードを記述
        # return path
        x = [pose.pose.position.x for pose in path.poses]
        y = [pose.pose.position.y for pose in path.poses]

        # スプライン補間
        tck, u = splprep([x, y], s=0, per=0)
        u_new = np.linspace(u.min(), u.max(), 500)
        x_new, y_new = splev(u_new, tck, der=0)

        # 補間した座標から新しいパスを生成
        smooth_path = Path()
        smooth_path.header = path.header
        for x, y in zip(x_new, y_new):
            pose = PoseStamped()
            pose.pose.position.x = x
            pose.pose.position.y = y
            smooth_path.poses.append(pose)
        return smooth_path
        
    def publish_path(self, path):
        # 生成した経路を出力するコードを記述
        print(path)
        self.publisher_.publish(path)
        # pass
        
def main():
    try:
        rclpy.init()
        generator = GPSPathGenerator()
        rclpy.spin(generator)
    except KeyboardInterrupt:
        print("ctrl-C")
    finally:
        generator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()