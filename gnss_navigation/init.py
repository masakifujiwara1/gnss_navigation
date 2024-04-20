import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from pyproj import Proj, transform

# WGS84座標系（GPSの座標系）
wgs84 = Proj(init='epsg:4326')

# UTM座標系への変換（ここではゾーン33Nを例とする）
utm = Proj(init='epsg:32633')  # UTMゾーンに応じてEPSGコードを変更

def convert_gps_to_utm(lat, lon):
    x, y = transform(wgs84, utm, lon, lat)
    return x, y

def publish_initial_pose(x, y):
    rospy.init_node('initial_pose_publisher')
    pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=1)
    initial_pose = PoseWithCovarianceStamped()

    initial_pose.header.stamp = rospy.Time.now()
    initial_pose.header.frame_id = "map"  # 地図座標系のフレーム
    initial_pose.pose.pose.position.x = x
    initial_pose.pose.pose.position.y = y
    initial_pose.pose.pose.orientation.w = 1.0  # 回転は無し（北を向いている）

    pub.publish(initial_pose)

if __name__ == '__main__':
    lat = 59.345237  # 緯度
    lon = 18.072648  # 経度
    x, y = convert_gps_to_utm(lat, lon)
    publish_initial_pose(x, y)
