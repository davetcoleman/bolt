# OMPL Experience Demos

## How to Build

Clone into your [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) and rebuild.

## Usage

Start Rviz using the included launch file:

```
roslaunch bolt_2d ompl_rviz.launch
```

### Experienced based-planning with Lightning Framework

The package ``bolt_2d`` can help debug experience-based planning like the lightning framework.

### Examples:

Image of an old path (red line) being repaired into feasible path (green line)

<img align="right" src="https://raw.githubusercontent.com/davetcoleman/ompl_visual_tools/hydro-devel/screenshots/similar_paths.png" />

Image of multiple paths in a experience database:

<img align="right" src="https://raw.githubusercontent.com/davetcoleman/ompl_visual_tools/hydro-devel/screenshots/repaired_path.png" />

To run:

```
wmctrl -a RViz && roslaunch bolt_2d ompl_demo.launch
```

## Documentation on Thunder

### Costs

 - All edges have weight
 - If an edge is found to be in collision, its weight is marked as infinity
