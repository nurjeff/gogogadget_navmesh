package gogogadget_navmesh

import (
	"container/heap"
	"errors"
	"fmt"
	"math"
)

type NavMesh struct {
	ID            int
	Vertices      []Vertex
	Triangles     [][3]int
	IsInitialized bool
	FileName      string
	ShortName     string
	Settings      NavMeshSettings
}

func (settings *NavMeshSettings) setDefaults() {
	settings.ToleranceXZ = .001
	settings.ToleranceY = 0.4
	settings.MaxAngleRadius = 150.0
}

func (navMesh *NavMesh) removeDuplicateVertices() {
	vertexMap := make(map[int]int)
	uniqueVertices := []Vertex{}

	for i, vertex := range navMesh.Vertices {
		found := false
		for j, uniqueVertex := range uniqueVertices {
			if vertex.equals(uniqueVertex) {
				vertexMap[i] = j
				found = true
				break
			}
		}
		if !found {
			uniqueVertices = append(uniqueVertices, vertex)
			vertexMap[i] = len(uniqueVertices) - 1
		}
	}

	for i, triangle := range navMesh.Triangles {
		for j, index := range triangle {
			navMesh.Triangles[i][j] = vertexMap[index]
		}
	}

	navMesh.Vertices = uniqueVertices
}

// IsPointInsideTriangleWithToleranceXY checks if a point is inside a triangle with separate tolerances for XZ and Y coordinates.
func (navMesh *NavMesh) isPointInsideTriangleWithToleranceXY(point, a, b, c Vertex, toleranceXZ, toleranceY float64) bool {
	// Calculate the normal vector of the triangle.
	normal := crossProduct(subtractVectors2(b, a), subtractVectors2(c, a))

	// Check if the point is in the same plane as the triangle (within toleranceY).
	if math.Abs(point.Y-a.Y) > toleranceY {
		return false
	}

	// Calculate barycentric coordinates of the point with respect to the triangle in the XZ plane.
	// If all coordinates are within [0, 1] and sum up to 1, the point is inside the triangle.
	alpha := dotProduct(crossProduct(subtractVectors2(point, b), subtractVectors2(point, c)), normal) / math.Pow(vectorLength(normal), 2)
	beta := dotProduct(crossProduct(subtractVectors2(point, c), subtractVectors2(point, a)), normal) / math.Pow(vectorLength(normal), 2)
	gamma := 1 - alpha - beta

	return alpha >= -toleranceXZ && beta >= -toleranceXZ && gamma >= -toleranceXZ
}

// Initializes the NavMesh. If no settings are passed, the defaults are used
func (n *NavMesh) initialize(settings *NavMeshSettings) error {
	if settings != nil {
		n.Settings = *settings
	} else {
		n.Settings.setDefaults()
	}

	if len(n.Triangles) <= 0 || len(n.Vertices) <= 0 {
		return errors.New("error initializing navmesh: triangles or vertices empty")
	}
	n.removeDuplicateVertices()
	n.orderTrianglesConsistently()
	n.IsInitialized = true
	return nil
}

type NavMeshSettings struct {
	// ToleranceXZ is the tolerance two vertices can move away on X and Z from the NavMesh to still be considered valid
	// Is useful for smoothing
	ToleranceXZ float64
	// ToleranceY is the tolerance two vertices can move away on Y from the NavMesh to still be considered valid
	// Is useful for smoothing
	ToleranceY float64
	// Max Angle the Agent can do when it's not strictly necessary, prevents zig-zagging
	// Default is 150
	MaxAngleRadius float64
}

func (navMesh NavMesh) findNearestVertexTowardDestination(start Vertex, destination Vertex) (Vertex, error) {
	// Step 1: Check if the destination is within the navmesh
	destination = navMesh.snapToNavMesh(destination)
	if !navMesh.isPointInNavMesh(destination) {
		return Vertex{}, fmt.Errorf("destination point is not reachable")
	}

	// Step 2: Find the nearest vertex towards the destination
	nearestVertex := navMesh.Vertices[0]
	minHeuristic := calculateHeuristic(start, nearestVertex, destination)

	for _, v := range navMesh.Vertices[1:] {
		currentHeuristic := calculateHeuristic(start, v, destination)
		if currentHeuristic < minHeuristic {
			minHeuristic = currentHeuristic
			nearestVertex = v
		}
	}

	return nearestVertex, nil
}

func (navMesh *NavMesh) isPointInNavMesh(point Vertex) bool {
	for _, triangle := range navMesh.Triangles {
		v0, v1, v2 := navMesh.Vertices[triangle[0]], navMesh.Vertices[triangle[1]], navMesh.Vertices[triangle[2]]
		if navMesh.isPointInTriangle(v0, v1, v2, point) {
			return true
		}
	}
	return false
}

func (navMesh *NavMesh) snapToNavMesh(point Vertex) Vertex {
	closestPoint := Vertex{}
	closestDist := math.MaxFloat64

	for _, triangle := range navMesh.Triangles {
		v0, v1, v2 := navMesh.Vertices[triangle[0]], navMesh.Vertices[triangle[1]], navMesh.Vertices[triangle[2]]

		// Check the vertices
		for _, vertex := range []Vertex{v0, v1, v2} {
			dist := distance(point, vertex)
			if dist < closestDist {
				closestDist = dist
				closestPoint = vertex
			}
		}

		// Check the edges
		edges := [][2]Vertex{{v0, v1}, {v1, v2}, {v2, v0}}
		for _, edge := range edges {
			closestEdgePoint := closestPointOnSegment(edge[0], edge[1], point)
			dist := distance(point, closestEdgePoint)
			if dist < closestDist {
				closestDist = dist
				closestPoint = closestEdgePoint
			}
		}

		// Check the interior of the triangle
		closestTrianglePoint, inside := closestPointInTriangle(v0, v1, v2, point)
		if inside {
			dist := distance(point, closestTrianglePoint)
			if dist < closestDist {
				closestDist = dist
				closestPoint = closestTrianglePoint
			}
		}
	}

	return closestPoint
}

func (navMesh *NavMesh) getYValueFromMesh(x, z float64) float64 {
	for _, triangle := range navMesh.Triangles {
		v0, v1, v2 := navMesh.Vertices[triangle[0]], navMesh.Vertices[triangle[1]], navMesh.Vertices[triangle[2]]

		// Calculate barycentric coordinates to check if the point is in the triangle
		denom := (v1.Z-v2.Z)*(v0.X-v2.X) + (v2.X-v1.X)*(v0.Z-v2.Z)
		a := ((v1.Z-v2.Z)*(x-v2.X) + (v2.X-v1.X)*(z-v2.Z)) / denom
		b := ((v2.Z-v0.Z)*(x-v2.X) + (v0.X-v2.X)*(z-v2.Z)) / denom
		c := 1 - a - b

		// If the point is in the triangle, find the y value based on the barycentric coordinates
		if 0 <= a && a <= 1 && 0 <= b && b <= 1 && 0 <= c && c <= 1 {
			y := a*v0.Y + b*v1.Y + c*v2.Y
			return y
		}
	}

	// If no triangle contains the point, return 0.0
	return 0.0
}

func (navMesh *NavMesh) optimizePath(path []Vertex) []Vertex {
	optimizedPath := []Vertex{}
	if len(path) == 0 {
		return optimizedPath
	}

	optimizedPath = append(optimizedPath, path[0])
	current := 0

	for current < len(path)-1 {
		lookAhead := len(path) - 1 // Start by assuming we can reach the end from current
		found := false
		for ; lookAhead > current; lookAhead-- {
			if navMesh.segmentInsideNavMesh(path[current], path[lookAhead]) {
				found = true
				break // Found the furthest direct reach from current
			}
		}

		if !found {
			lookAhead = current + 1 // Move to the next waypoint if no direct path was found
		}

		optimizedPath = append(optimizedPath, path[lookAhead])
		current = lookAhead
	}

	return optimizedPath
}

func (navMesh *NavMesh) segmentInsideNavMesh(p1, p2 Vertex) bool {
	// Sampling check
	for t := 0.0; t <= 1.0; t += 0.1 {
		interpolatedPoint := p1.multiply(1.0 - t).add(p2.multiply(t))
		if !navMesh.isPointInNavMesh(interpolatedPoint) {
			return false
		}
	}

	return true
}

func (navMesh *NavMesh) orderTrianglesConsistently() {
	// Define the desired direction for the normal (e.g., positive Y for upward normals)
	desiredNormalDirection := Vertex{X: 0, Y: 1, Z: 0} // Adjust if needed

	for i, triangleIndices := range navMesh.Triangles {
		v1 := navMesh.Vertices[triangleIndices[0]]
		v2 := navMesh.Vertices[triangleIndices[1]]
		v3 := navMesh.Vertices[triangleIndices[2]]

		normal := calculateNormal(v1, v2, v3)
		if normal.dot(desiredNormalDirection) < 0 {
			// If the normal's direction is not consistent with the desired direction,
			// swap two vertices to change the order
			navMesh.Triangles[i][1], navMesh.Triangles[i][2] = navMesh.Triangles[i][2], navMesh.Triangles[i][1]
		}
	}
}

func (navMesh *NavMesh) projectOntoNavMesh(point Vertex) *Vertex {
	closestIntersection := Vertex{}
	found := false
	smallestT := math.MaxFloat64

	rayOrigin := point
	rayDirection := Vertex{0, -1, 0} // pointing downwards

	for _, triangleIndices := range navMesh.Triangles {
		v0 := navMesh.Vertices[triangleIndices[0]]
		v1 := navMesh.Vertices[triangleIndices[1]]
		v2 := navMesh.Vertices[triangleIndices[2]]

		intersection, t := rayTriangleIntersection(rayOrigin, rayDirection, v0, v1, v2)
		if intersection != nil && t < smallestT {
			found = true
			smallestT = t
			closestIntersection = *intersection
		}
	}

	if found {
		return &closestIntersection
	}
	return nil
}

func (navMesh *NavMesh) reposition(A, C, P Vertex) Vertex {
	direction := C.subtract(A).normalize()
	newPosition := A.add(direction.multiply(A.distanceTo(P)))

	if !navMesh.isPointInNavMesh(newPosition) {
		adjustedPosition := navMesh.projectOntoNavMesh(newPosition)
		if adjustedPosition != nil && navMesh.isPointInNavMesh(*adjustedPosition) {
			return *adjustedPosition
		}
	}

	return P
}

func (navMesh *NavMesh) smoothPath(originalPath []Vertex) []Vertex {
	smoothedPath := []Vertex{}
	thresholdAngle := navMesh.Settings.MaxAngleRadius

	smoothedPath = append(smoothedPath, originalPath[0])

	for i := 1; i < len(originalPath)-1; i++ {
		A := originalPath[i-1]
		P := originalPath[i]
		C := originalPath[i+1]

		angle := computeAngle(A, P, C)

		if angle > thresholdAngle {
			// Try to reposition P on the line between A and C
			newPosition := navMesh.reposition(A, C, P)
			smoothedPath = append(smoothedPath, newPosition)
		} else {
			smoothedPath = append(smoothedPath, P)
		}
	}

	smoothedPath = append(smoothedPath, originalPath[len(originalPath)-1])

	return smoothedPath
}

func (navMesh *NavMesh) findBestNavMeshVertex(start Vertex, end Vertex) (Vertex, error) {
	start = navMesh.snapToNavMesh(start)
	var bestVertex Vertex
	minCost := math.MaxFloat64
	found := false

	// Loop through each vertex in the NavMesh
	for _, vertex := range navMesh.Vertices {
		// Check if the line segment between start and vertex is inside the NavMesh
		if navMesh.segmentInsideNavMesh(start, vertex) {
			distanceToStart := distanceBetweenVertices(start, vertex)
			distanceToEnd := distanceBetweenVertices(vertex, end)

			// The cost is a combination of the distance from the start point to the vertex
			// and the estimated distance from the vertex to the end point
			cost := distanceToStart + distanceToEnd
			if cost < minCost {
				minCost = cost
				bestVertex = vertex
				found = true
			}
		}
	}

	if !found {
		return Vertex{}, errors.New("no suitable vertex found within NavMesh constraints")
	}

	return bestVertex, nil
}

func distanceBetweenVertices(v1 Vertex, v2 Vertex) float64 {
	dx := v1.X - v2.X
	dy := v1.Y - v2.Y
	dz := v1.Z - v2.Z
	return math.Sqrt(dx*dx + dy*dy + dz*dz)
}

func (navMesh *NavMesh) PathFind(start Vertex, end Vertex) ([]Vertex, error) {
	path := []Vertex{}

	modifiedEnd, err := navMesh.findNearestVertexTowardDestination(end, end)
	if err != nil {
		return path, err
	}

	// Todo: change this so the start and end points do not snap to vertices
	modifiedStart, err := navMesh.findBestNavMeshVertex(start, modifiedEnd)
	fmt.Println(modifiedStart)
	if err != nil {
		return path, err
	}

	path, err = navMesh.findPath(modifiedStart, modifiedEnd)
	if err != nil {
		return path, err
	}

	if len(path) <= 0 {
		return path, errors.New("invalid path found")
	}

	path = navMesh.getFinalPath(path, start, end)
	path = navMesh.optimizePath(path)
	path = navMesh.smoothPath(path)
	for index, e := range path {
		path[index] = navMesh.snapToNavMesh(e)
		path[index].Y = navMesh.getYValueFromMesh(e.X, e.Z)
	}
	finishedPath := []Vertex{navMesh.snapToNavMesh(start)}
	for i := 1; i < len(path); i++ {
		if path[i].X != path[i-1].X || path[i].Y != path[i-1].Y || path[i].Z != path[i-1].Z {
			finishedPath = append(finishedPath, path[i])
		}
	}
	return finishedPath, nil
}

func (navMesh *NavMesh) findPath(start Vertex, end Vertex) ([]Vertex, error) {
	// Initialize open and closed list for A* algorithm
	openSet := make(map[Vertex]struct{})
	closedSet := make(map[Vertex]struct{})
	openList := &PriorityQueue{}
	heap.Init(openList)
	heap.Push(openList, &Item{value: start, priority: 0}) // Starting with 0 cost
	openSet[start] = struct{}{}

	parents := make(map[Vertex]Vertex)
	distances := make(map[Vertex]float64)
	for _, vertex := range navMesh.Vertices {
		distances[vertex] = vertex.distanceTo(end)
	}

	gScore := make(map[Vertex]float64)
	fScore := make(map[Vertex]float64)
	for _, vertex := range navMesh.Vertices {
		gScore[vertex] = math.MaxFloat64
		fScore[vertex] = gScore[vertex] + distances[vertex]
	}
	gScore[start] = 0
	fScore[start] = distances[start]

	for openList.Len() > 0 {
		// Find the node with the lowest score in the open list
		currentItem := heap.Pop(openList).(*Item)
		currentNode := currentItem.value
		delete(openSet, currentNode)
		closedSet[currentNode] = struct{}{}

		// Check if we reached the end point
		if currentNode.equals(end) {
			path := []Vertex{}
			for current := currentNode; !current.equals(start); current = parents[current] {
				path = append([]Vertex{current}, path...)
			}
			path = append([]Vertex{start}, path...)
			return path, nil
		}

		for _, neighbor := range navMesh.getNeighbors(currentNode) {
			if _, closed := closedSet[neighbor]; closed {
				continue
			}

			tentativeGScore := gScore[currentNode] + currentNode.distanceTo(neighbor)

			if tentativeGScore < gScore[neighbor] {
				parents[neighbor] = currentNode
				gScore[neighbor] = tentativeGScore
				fScore[neighbor] = gScore[neighbor] + distances[neighbor]

				if _, open := openSet[neighbor]; !open {
					heap.Push(openList, &Item{value: neighbor, priority: fScore[neighbor]})
					openSet[neighbor] = struct{}{}
				}
			}
		}
	}

	return nil, fmt.Errorf("no path found")
}

func (navMesh *NavMesh) getNeighbors(vertex Vertex) []Vertex {
	neighbors := []Vertex{}

	for _, triangle := range navMesh.Triangles {
		for i, index := range triangle {
			if navMesh.Vertices[index].equals(vertex) {
				if !vertexInVertices(navMesh.Vertices[triangle[(i+1)%3]], neighbors) {
					neighbors = append(neighbors, navMesh.Vertices[triangle[(i+1)%3]])
				}
				if !vertexInVertices(navMesh.Vertices[triangle[(i+2)%3]], neighbors) {
					neighbors = append(neighbors, navMesh.Vertices[triangle[(i+2)%3]])
				}
			}
		}
	}

	return neighbors
}

func (navMesh *NavMesh) isPointInTriangle(v0, v1, v2, point Vertex) bool {
	return navMesh.isPointInsideTriangleWithToleranceXY(point, v0, v1, v2, navMesh.Settings.ToleranceXZ, navMesh.Settings.ToleranceY)
}
