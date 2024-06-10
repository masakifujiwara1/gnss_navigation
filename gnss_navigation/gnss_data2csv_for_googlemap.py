import pandas as pd

# CSVファイルを読み込む
file_path = '~/Downloads/shihou_full_.csv'
df = pd.read_csv(file_path)

# 必要なカラムを選択してタプルのリストを作成
coordinates = list(zip(df.iloc[:, 1], df.iloc[:, 2]))

# 結果を表示
output_path = '~/Downloads/shihou_full_output_.csv'

coordinates_df = pd.DataFrame(coordinates)
# coordinates_df = coordinates_df.iloc[::10]
coordinates_df.to_csv(output_path, index=False, header=False)

print(f"Coordinates saved to {output_path}")