# Task 4 Part B Toolbox — launch-file mechanics reference

This is a reference for the *mechanics* of writing a launch file that
brings up a set of long-running, lifecycle-managed nodes. It is **not** a
list of which nodes Task 4 Part B needs — figuring that out from the capability
requirements in the handout, using the Nav2 architecture documentation, is
the actual assignment. Everything below is generic enough to apply to more
than one possible answer.

## Declaring a lifecycle-managed node in a launch file

Nothing special distinguishes a lifecycle node from a regular one at the
launch-file level — it's just a `Node` action:

```python
from launch_ros.actions import Node

my_node = Node(
    package="some_package",
    executable="some_executable",
    name="some_node",
    output="screen",
    parameters=[params_file_path, {"use_sim_time": True}],
)
```

A lifecycle node doesn't activate itself on startup, though — it comes up
in the `unconfigured` state and sits there until something drives it
through `configure` → `activate`. That's what a lifecycle manager is for.

## `lifecycle_manager`

One `lifecycle_manager` node takes a list of node *names* (not the launch
actions themselves — the ROS node names they'll register as) and, if
`autostart` is true, walks all of them through configure/activate in
order on startup:

```python
Node(
    package="nav2_lifecycle_manager",
    executable="lifecycle_manager",
    name="lifecycle_manager_example",
    output="screen",
    parameters=[{
        "autostart": True,
        "node_names": ["some_node", "some_other_node"],
    }],
)
```

You can have more than one `lifecycle_manager` in the same launch file,
each owning a different group of nodes — useful if two capabilities are
logically independent and you'd rather bring them up/down separately.

## Pointing a node at a YAML params file

```python
Node(
    ...,
    parameters=["/path/to/your_params.yaml"],
)
```

If the YAML has a top-level key matching the node's name (e.g. a
`some_node:` block with a `ros__parameters:` section under it), that
section is what gets loaded.

## Including another launch file wholesale

Sometimes the node(s) you need already ship their own launch file, and
"assembling components" means including that launch file with the right
arguments rather than re-declaring each `Node` yourself:

```python
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

included = IncludeLaunchDescription(
    PythonLaunchDescriptionSource("/path/to/some_launch_file.launch.py"),
    launch_arguments={"some_arg": "some_value"}.items(),
)
```

This is a completely legitimate way to assemble a system — real ROS 2
stacks are built this way constantly. The judgment call is *which*
existing launch file(s) actually provide the capability you need, and with
what arguments.

## Serving a live-built map vs. a premade one

One more mechanical option worth knowing about, since it's easy to reach
for by name-association: `slam_toolbox`'s
`online_async_launch.py` brings up a node that builds an occupancy grid
*live* from incoming lidar scans as the robot explores an *unknown*
environment — you'd point it at a params file and it starts publishing a
map as it goes, no premade map file involved at all.

```python
included = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        "/opt/ros/jazzy/share/slam_toolbox/launch/online_async_launch.py"),
    launch_arguments={"use_sim_time": "true"}.items(),
)
```

Whether that's the right tool for *this* task depends on whether the
environment's layout is actually unknown here, or whether you already have
a correct map of it sitting in a file.
