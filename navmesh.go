package gogogadget_navmesh

import (
	"container/heap"
	"errors"
	"fmt"
	"math"
)

type NavMesh struct {
	Vertices      []Vertex
	Triangles     [][3]int
	IsInitialized bool
	FileName      string
	Settings      NavMeshSettings
}

// Initializes the NavMesh. If no settings are passed, the defaults are used
func (n *NavMesh) initialize(settings *NavMeshSettings) error {
	if settings != nil {
		n.Settings = *settings
	} else {
		n.Settings.setDefaults()
	}

	n.IsInitialized = true
	if len(n.Triangles) <= 0 || len(n.Vertices) <= 0 {
		return errors.New("error initializing navmesh: triangles or vertices empty")
	}
	return nil
}

type NavMeshSettings struct {
	// This determines how aggressive the vertex picking should seek towards the destination point
	// Default Value 0.5
	DestinationHoming float64
	// Tolerance for a point to be still considered Valid
	// Default Value 0.01
	ValidPointTolerance float64
	// Tolerance for a point to still be considered on the navmesh on a vertical axis
	// Default Value 1e-9
	YCoordinateSameTolerance float64
	// How strong the path should be smoothed after optimizing
	// Default Value is 4
	SmoothingRadius int
	// How many iterations of smoothing should the path undergo
	// This might massively increase the amount of waypoints if set too high
	// Default Value is 2
	SmoothingIterations int
	// Determines the interpolation smoothing steps
	// Might massively increase the runtime if set too high
	// Default Value is .25
	SmoothingStep float64
}

func (navMesh *NavMesh) PathFind(start Vertex, end Vertex) ([]Vertex, error) {
	path := []Vertex{}
	if !navMesh.IsInitialized {
		return path, errors.New("navmesh not initialized")
	}

	// Find the nearest vertices to the start and end points
	sp := navMesh.findNearestVertex(start, end)
	ep, err := navMesh.findNearestVertexTowardDestination(end)
	if err != nil {
		return path, err
	}

	path, err = navMesh.AStar(sp, ep)
	if err != nil {
		fmt.Println(err)
		return path, err
	}

	path = appendStartAndEnd(path, start, end)
	path = navMesh.optimizePath(path)
	path = appendStartAndEnd(path, start, end)
	path = navMesh.addInterpolatedWaypoints(path)
	finishedPath := []Vertex{}
	// Remove duplicates
	for i := 1; i < len(path); i++ {
		if path[i].X != path[i-1].X || path[i].Y != path[i-1].Y || path[i].Z != path[i-1].Z {
			finishedPath = append(finishedPath, path[i])
		}
	}

	return finishedPath, nil
}

func (settings *NavMeshSettings) setDefaults() {
	settings.DestinationHoming = 0.5
	settings.ValidPointTolerance = 0.01
	settings.YCoordinateSameTolerance = 1e-9
	settings.SmoothingRadius = 4
	settings.SmoothingIterations = 2
	settings.SmoothingStep = 0.25
}

func removeDuplicateVertices(navMesh NavMesh) NavMesh {
	vertexMap := make(map[int]int)
	uniqueVertices := []Vertex{}

	for i, vertex := range navMesh.Vertices {
		found := false
		for j, uniqueVertex := range uniqueVertices {
			if vertex.Equals(uniqueVertex) {
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
	return navMesh
}

func (navMesh *NavMesh) findNearestVertex(vertex Vertex, endPoint Vertex) Vertex {
	nearestVertex := navMesh.Vertices[0]
	minDistance := vertex.distanceTo(nearestVertex)

	// Create a vector pointing from the specific start to the specific end
	dirToEnd := Vertex{X: endPoint.X - vertex.X, Y: endPoint.Y - vertex.Y, Z: endPoint.Z - vertex.Z}

	for _, v := range navMesh.Vertices[1:] {
		distance := vertex.distanceTo(v)

		// Create a vector pointing from the specific start to the current vertex
		dirToVertex := Vertex{X: v.X - vertex.X, Y: v.Y - vertex.Y, Z: v.Z - vertex.Z}

		// Calculate the cosine of the angle between the two vectors
		cosAngle := (dirToEnd.X*dirToVertex.X + dirToEnd.Y*dirToVertex.Y + dirToEnd.Z*dirToVertex.Z) /
			(dirToEnd.length() * dirToVertex.length())

		// Prefer vertices that are closer and more directly in the path to the end
		if distance < minDistance && cosAngle > navMesh.Settings.DestinationHoming { // You can tune this threshold as needed
			minDistance = distance
			nearestVertex = v
		}
	}

	return nearestVertex
}

func (navMesh *NavMesh) isPointInNavMesh(point Vertex) bool {
	for _, triangle := range navMesh.Triangles {
		v0, v1, v2 := navMesh.Vertices[triangle[0]], navMesh.Vertices[triangle[1]], navMesh.Vertices[triangle[2]]
		if isPointInTriangle(v0, v1, v2, point) {
			return true
		}
	}
	return false
}

func (navMesh NavMesh) addInterpolatedWaypoints(path []Vertex) []Vertex {
	for iter := 0; iter < navMesh.Settings.SmoothingIterations; iter++ {
		newPath := []Vertex{path[0]}
		for i := 0; i < len(path)-1; i++ {
			newPath = append(newPath, path[i])
			start, end := path[i], path[i+1]

			for t := navMesh.Settings.SmoothingStep; t < 1; t += navMesh.Settings.SmoothingStep {
				x := start.X + t*(end.X-start.X)
				z := start.Z + t*(end.Z-start.Z)
				y := navMesh.getYValueFromMesh(x, z)
				v := Vertex{X: x, Y: y, Z: z}

				if !navMesh.isValidPoint(v) {
					// Find the nearest valid point and add it to the path
					v = navMesh.snapToNavMesh(v)
				}

				newPath = append(newPath, v)
			}
		}
		newPath = append(newPath, path[len(path)-1])
		path = newPath
	}

	return path
}

// A* algorithm implementation using Min-Heap as a priority queue
func (navMesh *NavMesh) AStar(start Vertex, end Vertex) ([]Vertex, error) {
	// Initialize open and closed list for A* algorithm
	openSet := make(map[Vertex]struct{})
	closedSet := make(map[Vertex]struct{})
	openList := &PriorityQueue{}
	heap.Init(openList)
	heap.Push(openList, &Item{value: start, priority: start.distanceTo(end)})
	openSet[start] = struct{}{}

	parents := make(map[Vertex]Vertex)

	distances := make(map[Vertex]float64)
	for _, vertex := range navMesh.Vertices {
		distances[vertex] = vertex.distanceTo(end)
	}

	for openList.Len() > 0 {
		// Find the node with the lowest score in the open list
		currentItem := heap.Pop(openList).(*Item)
		currentNode := currentItem.value
		delete(openSet, currentNode)
		closedSet[currentNode] = struct{}{}

		// Check if we reached the end point
		if currentNode.Equals(end) {
			// Reconstruct the path here and return it
			path := []Vertex{}
			for current := currentNode; !current.Equals(start); current = parents[current] {
				path = append([]Vertex{current}, path...)
			}
			path = append([]Vertex{start}, path...)
			return path, nil
		}

		// For each neighbor of the current node
		for _, neighbor := range navMesh.getNeighbors(currentNode) {
			if _, closed := closedSet[neighbor]; closed {
				continue
			}

			if _, open := openSet[neighbor]; !open {
				heap.Push(openList, &Item{value: neighbor, priority: distances[neighbor]})
				openSet[neighbor] = struct{}{}
				parents[neighbor] = currentNode
			}
		}
	}

	return nil, fmt.Errorf("no path found")
}

func (navMesh *NavMesh) getNeighbors(vertex Vertex) []Vertex {
	neighbors := []Vertex{}

	for _, triangle := range navMesh.Triangles {
		for i, index := range triangle {
			if navMesh.Vertices[index].Equals(vertex) {
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

	// If no triangle contains the point, return a default value (you might want to return an error or a special value here)
	return 0.0
}

func (navMesh *NavMesh) isValidPoint(vertex Vertex) bool {
	for _, triangle := range navMesh.Triangles {
		v0, v1, v2 := navMesh.Vertices[triangle[0]], navMesh.Vertices[triangle[1]], navMesh.Vertices[triangle[2]]

		// Calculate barycentric coordinates to check if the point is in the triangle
		denom := (v1.Z-v2.Z)*(v0.X-v2.X) + (v2.X-v1.X)*(v0.Z-v2.Z)
		a := ((v1.Z-v2.Z)*(vertex.X-v2.X) + (v2.X-v1.X)*(vertex.Z-v2.Z)) / denom
		b := ((v2.Z-v0.Z)*(vertex.X-v2.X) + (v0.X-v2.X)*(vertex.Z-v2.Z)) / denom
		c := 1 - a - b

		// If the point is in the triangle, find the y value based on the barycentric coordinates
		if 0 <= a && a <= 1 && 0 <= b && b <= 1 && 0 <= c && c <= 1 {
			yMesh := a*v0.Y + b*v1.Y + c*v2.Y
			if math.Abs(vertex.Y-yMesh) < navMesh.Settings.ValidPointTolerance { // You may need to adjust this tolerance
				return true
			}
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
			dist := point.distanceTo(vertex)
			if dist < closestDist {
				closestDist = dist
				closestPoint = vertex
			}
		}

		// Check the edges
		edges := [][2]Vertex{{v0, v1}, {v1, v2}, {v2, v0}}
		for _, edge := range edges {
			closestEdgePoint := closestPointOnSegment(edge[0], edge[1], point)
			dist := point.distanceTo(closestEdgePoint)
			if dist < closestDist {
				closestDist = dist
				closestPoint = closestEdgePoint
			}
		}

		// Check the interior of the triangle
		closestTrianglePoint, inside := closestPointInTriangle(v0, v1, v2, point)
		if inside {
			dist := point.distanceTo(closestTrianglePoint)
			if dist < closestDist {
				closestDist = dist
				closestPoint = closestTrianglePoint
			}
		}
	}

	return closestPoint
}

func (navMesh *NavMesh) doesLineFollowTerrain(start, end Vertex) bool {
	for _, triangle := range navMesh.Triangles {
		for i := 0; i < 3; i++ {
			edgeStart := navMesh.Vertices[triangle[i]]
			edgeEnd := navMesh.Vertices[triangle[(i+1)%3]]

			intersectionPoint := lineIntersectionPoint(start, end, edgeStart, edgeEnd)
			if intersectionPoint != nil {
				// Calculate the Y coordinate of the line at the intersection point
				t := (intersectionPoint.X - start.X) / (end.X - start.X)
				if math.IsNaN(t) {
					t = (intersectionPoint.Z - start.Z) / (end.Z - start.Z)
				}
				yOnLine := start.Y + (end.Y-start.Y)*t

				// Calculate the Y coordinate of the navmesh edge at the intersection point
				u := (intersectionPoint.X - edgeStart.X) / (edgeEnd.X - edgeStart.X)
				if math.IsNaN(u) {
					u = (intersectionPoint.Z - edgeStart.Z) / (edgeEnd.Z - edgeStart.Z)
				}
				yOnNavMeshEdge := edgeStart.Y + (edgeEnd.Y-edgeStart.Y)*u

				// Compare the Y coordinates. If they are approximately equal (within the tolerance),
				// the line follows the terrain; otherwise, it doesn't
				if math.Abs(yOnLine-yOnNavMeshEdge) > navMesh.Settings.YCoordinateSameTolerance {
					return false
				}
			}
		}
	}
	return true
}

func (navMesh *NavMesh) isEdgeShared(v1, v2 Vertex) bool {
	count := 0
	for _, triangle := range navMesh.Triangles {
		containsV1 := false
		containsV2 := false
		for _, vertexIdx := range triangle {
			if navMesh.Vertices[vertexIdx] == v1 {
				containsV1 = true
			}
			if navMesh.Vertices[vertexIdx] == v2 {
				containsV2 = true
			}
		}
		if containsV1 && containsV2 {
			count++
		}
	}
	return count > 1
}

func (navMesh NavMesh) findNearestVertexTowardDestination(destination Vertex) (Vertex, error) {
	// Step 1: Check if the destination is within the navmesh
	if !navMesh.isPointInNavMesh(destination) {
		return Vertex{}, fmt.Errorf("destination point is not reachable")
	}

	// Step 2: Find the nearest vertex towards the destination
	nearestVertex := navMesh.Vertices[0]
	minHeuristic := calculateHeuristic(destination, nearestVertex, destination)

	for _, v := range navMesh.Vertices[1:] {
		currentHeuristic := calculateHeuristic(destination, v, destination)
		if currentHeuristic < minHeuristic {
			minHeuristic = currentHeuristic
			nearestVertex = v
		}
	}

	return nearestVertex, nil
}

func (navMesh *NavMesh) optimizePath(path []Vertex) []Vertex {
	path = navMesh.straightenPath(path)
	optimizedPath := []Vertex{}
	if len(path) < 3 {
		return path
	}
	optimizedPath = append(optimizedPath, path[0])

	for i := 0; i < len(path)-2; {
		j := i + 2
		valid := true
		for ; j < len(path); j++ {
			if !navMesh.canSkipWaypoint(path[i], path[j]) {
				valid = false
				break
			}
		}
		if valid {
			optimizedPath = append(optimizedPath, path[j-1])
			i = j - 1
		} else {
			optimizedPath = append(optimizedPath, path[i+1])
			i++
		}
	}
	optimizedPath = append(optimizedPath, path[len(path)-1])

	for i := 1; i < len(optimizedPath)-1; i++ {
		avgX, avgY, avgZ := optimizedPath[i].X, optimizedPath[i].Y, optimizedPath[i].Z
		count := 1.0

		// Calculate the average position with neighboring points within the smoothing radius
		for offset := 1; offset <= navMesh.Settings.SmoothingRadius; offset++ {
			if i-offset >= 0 {
				prev := optimizedPath[i-offset]
				avgX += prev.X
				avgY += prev.Y
				avgZ += prev.Z
				count++
			}
			if i+offset < len(optimizedPath) {
				next := optimizedPath[i+offset]
				avgX += next.X
				avgY += next.Y
				avgZ += next.Z
				count++
			}
		}

		// Get the averaged position
		avgX /= count
		avgY /= count
		avgZ /= count

		// Snap the averaged point to the navMesh
		averagedPoint := Vertex{avgX, avgY, avgZ}
		snappedPoint := navMesh.snapToNavMesh(averagedPoint) // assume snapToNavMesh is your function to snap points to the navMesh
		optimizedPath[i] = snappedPoint
	}

	finishedPath := []Vertex{}
	for i := 1; i < len(optimizedPath); i++ {
		if optimizedPath[i].X != optimizedPath[i-1].X || optimizedPath[i].Y != optimizedPath[i-1].Y || optimizedPath[i].Z != optimizedPath[i-1].Z {
			finishedPath = append(finishedPath, optimizedPath[i])
		}
	}

	return finishedPath[:len(finishedPath)-1]
}

func (navMesh NavMesh) canSkipWaypoint(start, end Vertex) bool {
	for _, triangle := range navMesh.Triangles {
		for i := 0; i < 3; i++ {
			edgeStart := navMesh.Vertices[triangle[i]]
			edgeEnd := navMesh.Vertices[triangle[(i+1)%3]]

			if lineIntersectsEdge(start, end, edgeStart, edgeEnd) {
				if !navMesh.isEdgeShared(edgeStart, edgeEnd) {
					return false
				}
			}

			// New Check: Verify if the line follows the terrain
			if !navMesh.doesLineFollowTerrain(start, end) {
				return false
			}
		}
	}
	return true
}

func (navMesh *NavMesh) straightenPath(path []Vertex) []Vertex {
	if len(path) < 3 {
		return path
	}

	straightenedPath := []Vertex{path[0]}

	for i := 0; i < len(path)-2; i++ {
		// Check if the current point, the next point, and the point after the next form a zigzag
		dir1 := path[i+1].subtract(path[i])
		dir2 := path[i+2].subtract(path[i+1])
		dir1.Y = 0
		dir2.Y = 0

		if math.Abs(dir1.angleWith(dir2)) > 150 { // You might want to tweak this angle
			// Attempt to replace the zigzag with a straight line, but only if it follows the terrain
			if navMesh.doesLineFollowTerrain(path[i], path[i+2]) {
				straightenedPath = append(straightenedPath, path[i+2])
				i++ // skip the next point
			} else {
				straightenedPath = append(straightenedPath, path[i+1])
			}
		} else {
			straightenedPath = append(straightenedPath, path[i+1])
		}
	}
	straightenedPath = append(straightenedPath, path[len(path)-1])

	return straightenedPath
}
