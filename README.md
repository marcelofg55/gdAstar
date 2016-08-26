# gdAstar
Godot A* module

A* implementation for godot, based off Justin Heyes-Jones A* C++ implementation ([https://github.com/justinhj/astar-algorithm-cpp](https://github.com/justinhj/astar-algorithm-cpp))

## Example usage in gdscript

```python
var astar = gdAstar.new()

# Add all your walkable map points
var x = 0
var y = 0
astar.AddPoint(x, y)

# Find a path
var src = Vector2(0, 0)
var dst = Vector2(1, 1)
astar.FindPath(src.x, src.y, dst.x, dst.y)

# You may clear the current points with ClearPoints
astar.ClearPoints()

# And you can instance multiple gdAstar in case you need to
var astarFlying = gdAstar.new()

```
