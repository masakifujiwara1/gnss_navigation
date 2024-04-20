import pandas as pd
from common_python.setup_util import get_data_files

# CSVファイルを読み込む
file_path = '/home/host_files/gnss_output_2024_04_12-21_35_41.csv'
df = pd.read_csv(file_path)

# 必要なカラムを選択してタプルのリストを作成
coordinates = list(zip(df.iloc[:, 5], df.iloc[:, 6]))

# 結果を表示
# print(coordinates)
output_path = '/home/ros2_ws/src/gnss_navigation/config/path_compress.csv'

coordinates_df = pd.DataFrame(coordinates)
coordinates_df = coordinates_df.iloc[::40]
coordinates_df.to_csv(output_path, index=False, header=False)

print(f"Coordinates saved to {output_path}")