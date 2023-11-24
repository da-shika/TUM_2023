import numpy as np
from scipy.spatial.transform import Rotation

# 4x4の変換行列を定義します（例）
transform_matrix = np.array([
    [0.866, -0.5, 0.0, 2.0],
    [0.5, 0.866, 0.0, 1.0],
    [0.0, 0.0, 1.0, 3.0],
    [0.0, 0.0, 0.0, 1.0]
])
# 平行移動ベクトルを取得します
translation_vector = transform_matrix[:3, 3]
# 回転行列部分を取得して、それをクオータニオンに変換します
rotation_matrix = transform_matrix[:3, :3]
rotation = Rotation.from_matrix(rotation_matrix)
quaternion = rotation.as_quat()
# 結果を表示します
print("平行移動ベクトル:", translation_vector)
print("クオータニオン:", quaternion)