# A* for Lua

A clean, simple implementation of the A* pathfinding algorithm for Lua.

This implementation has no dependencies and has a simple interface. It takes a table of nodes, a start and end point and a "valid neighbor" function which makes it easy to adapt the module's behavior, especially in circumstances where valid paths would frequently change.

## Usage example

```lua
local AStar = require("A-Star")

-- Determines which neighbors are valid (e.g. within range)
local function validNodeFunc(node, neighbor)
    local MAX_DIST = 300

    if distance(node.x, node.y, neighbor.x, neighbor.y) < MAX_DIST then
        return true
    end

    return false
end

local path = AStar.path (startNode, endNode, nodes, validNodeFunc)

if path then
 -- Do something with the path (table of nodes)
end
```

## Notes

This assumes that nodes are tables with (at least) members `x` and `y` that hold the node's coordinates.

```lua
Node = {}
Node.x = 123
Node.y = 456
Node.foo = "bar"
.
.
```
