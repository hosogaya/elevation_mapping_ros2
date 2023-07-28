# Zed min

# Tf
1. map (yamlファイルから変更可能)
2. odom (yamlファイルから変更可能)
3. base_link (yamlファイルから変更可能)
4. zedm_base_link (urdfで定義されている)

# topic 
|topic name| type | description |
| ---| ---| --- |
|`zedm/zed_node/odom` | `nav_msgs/msg/Odometry` |odom->base_linkのposeとtwist|
|`/zedm/zed_node/rgb/image_rect_color`|`sensor_msgs/msgs/Image` | 左右のカメラの画像を統合した画像 |
|`/zedm/zed_node/point_cloud/cloud_registered`| `sensor_msgs/msg/PointCloud2`|ポインtクラウド|
|`/zedm/zed_node/pose_with_covariance`|`geometry_msgs/msg/PoseWithCovarianceStamped`|mapに対するbase_linkの位置姿勢|
