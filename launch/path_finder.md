Recommended Setup for a Blind Assistant
Simple Environments (Flat, Indoor):
    Global Planner: A* on /map.
    Local Planner: DWA using /local_grid_obstacle.
Complex Environments (Multi-level, Outdoor):
    Global Planner: Voxel-based A* or PRM on /octomap_full.
    Local Planner: TEB or RRT-based local planner with /local_grid_obstacle.
Dynamic Environments:
    Use a hybrid approach:
    Global Planning: A* on /map or /octomap_grid.
    Local Planning: TEB with dynamic obstacle updates from /cloud_obstacles or /local_grid_obstacle.
