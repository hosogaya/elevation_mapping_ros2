# Elevation mapping
This is Ros2 version of [Robot centric elevation mapping](https://github.com/ANYbotics/elevation_mapping.git)
This consists of `elevation_mapping` and `post_processing` nodes. 


# elavation_mapping node
## Input
* input/point_clould `sensor_msgs::msg::PointCloud2`
  * raw point cloud data

## Output
* output/raw_map `grid_map_msgs::msg::GridMap`
  * raw grid map contains `elevation` and `variance` layers. 

# post processing node
## Input
* input/grid_map `grid_map_msgs::msg::GridMap`
  * raw grid map
## Output
* output/grid_map `grid_map_msgs::msg::GridMap`
  * grid map contains some layers calculated from `elevation` layer. 
## Processing type
See [grid_map_fiter](https://github.com/ANYbotics/grid_map/tree/master/grid_map_filters). 
The processings can be selected as `post_processing.param.yaml`