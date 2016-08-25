# gdAstar
Godot A* module

A* implementation for godot, based off Justin Heyes-Jones A* C++ implementation ([https://github.com/justinhj/astar-algorithm-cpp](https://github.com/justinhj/astar-algorithm-cpp))

## Example usage in gdscript

var astar = gdAstar.new()

# Add all your walkable map points
var x = 0
var y = 0
astar.AddPoint(x, y)

# Find a path
var src = Vector2(0, 0)
var dst = Vector2(0, 0)
astar.FindPath(src.x, src.y, dst.x, dst.y)
