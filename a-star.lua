-- ======================================================================
-- Copyright (c) 2012 RapidFire Studio Limited
-- All Rights Reserved.
-- http://www.rapidfirestudio.com

-- Permission is hereby granted, free of charge, to any person obtaining
-- a copy of this software and associated documentation files (the
-- "Software"), to deal in the Software without restriction, including
-- without limitation the rights to use, copy, modify, merge, publish,
-- distribute, sublicense, and/or sell copies of the Software, and to
-- permit persons to whom the Software is furnished to do so, subject to
-- the following conditions:

-- The above copyright notice and this permission notice shall be
-- included in all copies or substantial portions of the Software.

-- THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
-- EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
-- MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
-- IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
-- CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
-- TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
-- SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
-- ======================================================================

---@class AStar
local AStar = {}
local INF = 1 / 0
local isValidNode = function(node, neighbor) return true end

--[[ Local Functions ]]

local function dist(x1, y1, x2, y2)
	return math.sqrt((x2 - x1) ^ 2 + (y2 - y1) ^ 2)
end

local function distBetween(nodeA, nodeB)
	return dist(nodeA.x, nodeA.y, nodeB.x, nodeB.y)
end

local function heuristicCostEstimate(nodeA, nodeB)
	return dist(nodeA.x, nodeA.y, nodeB.x, nodeB.y)
end

local function lowestFScore(set, f_score)
	local lowest, bestNode = INF, nil
	for _, node in ipairs(set) do
		local score = f_score[node]
		if score < lowest then
			lowest, bestNode = score, node
		end
	end

	return bestNode
end

local function neighborNodes(theNode, nodes)
	local neighbors = {}
	for _, node in pairs(nodes) do
		if theNode ~= node and isValidNode(theNode, node) then
			table.insert(neighbors, node)
		end
	end

	return neighbors
end

local function notIn(set, theNode)
	for _, node in ipairs(set) do
		if node == theNode then return false end
	end

	return true
end

local function removeNode(set, theNode)
	for i, node in ipairs(set) do
		if node == theNode then
			set[i] = set[#set]
			set[#set] = nil
			break
		end
	end
end

local function unwindPath(flatPath, map, currentNode)
	if map[currentNode] then
		table.insert(flatPath, 1, map[currentNode])
		return unwindPath(flatPath, map, map[currentNode])
	else
		return flatPath
	end
end

--[[ Exposed Functions ]]

-- Finds the shortest path between two nodes using the A* algorithm
---@generic T
---@param start T
---@param goal T
---@param nodes table<T>
---@param validNodeFunc? fun(node : T, neighbor : T)
---@return T[]|nil
function AStar.path(start, goal, nodes, validNodeFunc)
	local closedSet = {}
	local openSet = { start }
	local cameFrom = {}

	if validNodeFunc then isValidNode = validNodeFunc end

	local gScore, fScore = {}, {}
	gScore[start] = 0
	fScore[start] = gScore[start] + heuristicCostEstimate(start, goal)

	while #openSet > 0 do

		local current = lowestFScore(openSet, fScore)
		if current == goal then
			local path = unwindPath({}, cameFrom, goal)
			table.insert(path, goal)
			return path
		end

		removeNode(openSet, current)
		table.insert(closedSet, current)

		local neighbors = neighborNodes(current, nodes)
		for _, neighbor in ipairs(neighbors) do
			if notIn(closedSet, neighbor) then

				local tentative_g_score = gScore[current] + distBetween(current, neighbor)

				if notIn(openSet, neighbor) or tentative_g_score < gScore[neighbor] then
					cameFrom[neighbor] = current
					gScore[neighbor] = tentative_g_score
					fScore[neighbor] = gScore[neighbor] + heuristicCostEstimate(neighbor, goal)
					if notIn(openSet, neighbor) then
						table.insert(openSet, neighbor)
					end
				end
			end
		end
	end

	-- No valid path
	return nil
end

return AStar
