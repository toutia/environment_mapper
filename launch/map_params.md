The Grid/MaxObstacleHeight and Grid/MinGroundHeight parameters in RTAB-Map play a crucial role in filtering the 3D point cloud data used to generate the 2D occupancy grid. These parameters define height thresholds for classifying points as obstacles, ground, or free space.

1. Grid/MaxObstacleHeight
Purpose: Determines the maximum height above the ground where points are considered obstacles.
Default Value: Typically set to a value like 2.0 meters, depending on the use case.
Effect:
Points higher than this threshold are ignored when building the occupancy grid.
This helps to exclude tall structures like ceilings, overhanging objects, or points irrelevant to the navigation plane.
Lowering this value focuses the map on lower-height obstacles, suitable for environments where robots need to avoid small barriers but can ignore high objects.
Example:

If Grid/MaxObstacleHeight = 1.5, points above 1.5 meters will not be considered obstacles in the occupancy grid.
2. Grid/MinGroundHeight
Purpose: Specifies the minimum height below which points are considered as ground or free space.
Default Value: Often set to a small negative value, such as -0.1 meters.
Effect:
Points below this threshold are ignored in the grid calculation.
Useful for filtering out points below the robot's operational plane, such as those from uneven terrain, underpasses, or irrelevant floor clutter.
Raising this value can ensure that false obstacles caused by noisy low points (e.g., shadow artifacts from sensors) are ignored.
Example:

If Grid/MinGroundHeight = -0.2, points lower than -0.2 meters relative to the sensor's origin are excluded.
How They Work Together
These parameters set a "slice" of the 3D point cloud data to be projected onto the 2D occupancy grid:

Points between Grid/MinGroundHeight and Grid/MaxObstacleHeight are analyzed.
Points within this range are:
Classified as obstacles if they fall within the height range and align with occupied cells.
Classified as ground/free space if they fall closer to the ground or align with empty cells.
By carefully tuning these parameters, you can ensure that:

Obstacles like furniture, walls, and small barriers are included in the map.
Irrelevant or noisy points, such as high ceilings or very low ground clutter, are excluded.
Typical Use Cases
Indoor Environments:
Grid/MaxObstacleHeight is often set to exclude ceiling points (e.g., 2.0 meters).
Grid/MinGroundHeight is set slightly below the floor plane (e.g., -0.1 meters) to include all floor points while avoiding sensor noise.
Outdoor Environments:
Grid/MaxObstacleHeight might be higher (e.g., 5.0 meters) to include tall obstacles like trees.
Grid/MinGroundHeight might be set to exclude points far below the terrain, such as those from ditches or depressions.
Practical Tips
Use a visualization tool (e.g., RViz) to observe the filtered point cloud when adjusting these parameters.
Ensure the height thresholds are suitable for the sensor placement and the robot's operational height.
Test different settings in your environment to avoid missing relevant obstacles or including unnecessary data.
These parameters are especially important for robots operating in environments with diverse height variations and can greatly impact navigation and planning.