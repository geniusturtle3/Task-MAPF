from heapq import heappush, heappop

def search(grid, start, goal, heuristic):
    path = []
    found = False
    rows, cols = len(grid), len(grid[0])
    visited = [[False for _ in range(cols)] for _ in range(rows)]
    costs = [[float('inf') for _ in range(cols)] for _ in range(rows)]
    parent = [[None for _ in range(cols)] for _ in range(rows)]
    parent[start[0]][start[1]] = (-1, -1)
    directions = [(0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1), (1, 0), (1, 1)]
    queue = [(0 + heuristic(start, goal), 0, start[0], start[1])]
    costs[start[0]][start[1]] = 0

    while queue:
        _, cost, curRow, curCol = heappop(queue)
        if (curRow, curCol) == goal:
            found = True
            break
        for direction in directions:
            newRow, newCol = curRow + direction[0], curCol + direction[1]
            if 0 <= newRow < rows and 0 <= newCol < cols and not visited[newRow][newCol] and not grid[newRow][newCol]:
                newCost = cost + 1
                if newCost < costs[newRow][newCol]:
                    costs[newRow][newCol] = newCost
                    prio = newCost + heuristic((newRow, newCol), goal)
                    parent[newRow][newCol] = (curRow, curCol)
                    visited[newRow][newCol] = True
                    heappush(queue, (prio, newCost, newRow, newCol))
    if found:
        curRow, curCol = goal
        while (curRow, curCol) != (-1, -1):
            path.insert(0, (curRow, curCol))
            curRow, curCol = parent[curRow][curCol]
    return found, path

def dijkstra(grid, start, goal):
    found, path = search(grid, start, goal, lambda x, y: 0)
    return found, path

def astar(grid, start, goal):
    found, path = search(grid, start, goal, lambda x, y: abs(x[0] - y[0]) + abs(x[1] - y[1]))
    return found, path