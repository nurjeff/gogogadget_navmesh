package gogogadget_navmesh

import "math"

func isPointInTriangle(v0, v1, v2, point Vertex) bool {
	// Calculate the normal of the plane
	edge1 := Vertex{v1.X - v0.X, v1.Y - v0.Y, v1.Z - v0.Z}
	edge2 := Vertex{v2.X - v0.X, v2.Y - v0.Y, v2.Z - v0.Z}
	normal := crossProduct(edge1, edge2)

	// Find the distance from the point to the plane
	toPoint := Vertex{point.X - v0.X, point.Y - v0.Y, point.Z - v0.Z}
	distToPlane := dotProduct(normal, toPoint) / math.Sqrt(dotProduct(normal, normal))

	// If the point is in the plane, check if it is in the triangle using barycentric coordinates
	if math.Abs(distToPlane) < 1e-1 {
		dot00 := dotProduct(edge1, edge1)
		dot01 := dotProduct(edge1, edge2)
		dot02 := dotProduct(edge1, toPoint)
		dot11 := dotProduct(edge2, edge2)
		dot12 := dotProduct(edge2, toPoint)

		invDenom := 1.0 / (dot00*dot11 - dot01*dot01)
		u := (dot11*dot02 - dot01*dot12) * invDenom
		v := (dot00*dot12 - dot01*dot02) * invDenom

		return (u >= 0) && (v >= 0) && (u+v < 1)
	}

	return false
}

func crossProduct(v1, v2 Vertex) Vertex {
	return Vertex{
		X: v1.Y*v2.Z - v1.Z*v2.Y,
		Y: v1.Z*v2.X - v1.X*v2.Z,
		Z: v1.X*v2.Y - v1.Y*v2.X,
	}
}

func dotProduct(v1, v2 Vertex) float64 {
	return v1.X*v2.X + v1.Y*v2.Y + v1.Z*v2.Z
}

func calculateHeuristic(start, current, destination Vertex) float64 {
	// The distance from start to current vertex
	distToCurrent := start.distanceTo(current)

	// The estimated distance from current vertex to destination (a straight line for simplicity)
	estDistToDestination := current.distanceTo(destination)

	// we're simply combining both instances to act as the heuristic
	// it might be useful to prioritzie one over the other here
	return distToCurrent + estDistToDestination
}

func appendStartAndEnd(rawPath []Vertex, exactStart, exactEnd Vertex) []Vertex {
	fp := append([]Vertex{}, exactStart)
	fp = append(fp, rawPath...)
	return append(fp, exactEnd)
}

func int32SliceToIntSlice(int32Slice [][3]int32) [][]int {
	intSlice := make([][]int, len(int32Slice))

	for i, int32Array := range int32Slice {
		intSlice[i] = make([]int, len(int32Array))
		for j, value := range int32Array {
			intSlice[i][j] = int(value)
		}
	}

	return intSlice
}

func closestPointOnSegment(a, b, p Vertex) Vertex {
	ab := Vertex{b.X - a.X, b.Y - a.Y, b.Z - a.Z}
	ap := Vertex{p.X - a.X, p.Y - a.Y, p.Z - a.Z}
	t := (ap.X*ab.X + ap.Y*ab.Y + ap.Z*ab.Z) / (ab.X*ab.X + ab.Y*ab.Y + ab.Z*ab.Z)
	t = math.Max(0, math.Min(1, t))
	return Vertex{a.X + ab.X*t, a.Y + ab.Y*t, a.Z + ab.Z*t}
}

func closestPointInTriangle(v0, v1, v2, p Vertex) (Vertex, bool) {
	// Compute the normal of the triangle
	normal := cross(v1.subtract(v0), v2.subtract(v0))

	// Find the closest point in the plane of the triangle
	d := -dot(normal, v0)
	t := -(dot(normal, p) + d) / dot(normal, normal)
	closestPoint := Vertex{
		p.X + t*normal.X,
		p.Y + t*normal.Y,
		p.Z + t*normal.Z,
	}

	// Check if the closest point is inside the triangle using barycentric coordinates
	u, v, w := barycentric(v0, v1, v2, closestPoint)
	if u >= 0 && v >= 0 && w >= 0 {
		return closestPoint, true
	}

	return Vertex{}, false
}

func dot(a, b Vertex) float64 {
	return a.X*b.X + a.Y*b.Y + a.Z*b.Z
}

func cross(a, b Vertex) Vertex {
	return Vertex{
		a.Y*b.Z - a.Z*b.Y,
		a.Z*b.X - a.X*b.Z,
		a.X*b.Y - a.Y*b.X,
	}
}

func barycentric(v0, v1, v2, p Vertex) (float64, float64, float64) {
	v0v2 := v2.subtract(v0)
	v0v1 := v1.subtract(v0)
	v0p := p.subtract(v0)
	d00 := dot(v0v2, v0v2)
	d01 := dot(v0v2, v0v1)
	d11 := dot(v0v1, v0v1)
	d20 := dot(v0p, v0v2)
	d21 := dot(v0p, v0v1)
	denom := d00*d11 - d01*d01
	v := (d11*d20 - d01*d21) / denom
	w := (d00*d21 - d01*d20) / denom
	u := 1.0 - v - w
	return u, v, w
}

func lineIntersectionPoint(a, b, c, d Vertex) *Vertex {
	// Calculate the differences in X and Z coordinates for the line segments
	dx1 := b.X - a.X
	dz1 := b.Z - a.Z
	dx2 := d.X - c.X
	dz2 := d.Z - c.Z

	// Calculate the determinants
	det := dx1*dz2 - dx2*dz1

	// If the determinants are zero, the lines are parallel and have no intersection point
	if det == 0 {
		return nil
	}

	// Calculate the parameters for the potential intersection point
	ua := ((c.X-a.X)*dz2 - (c.Z-a.Z)*dx2) / det
	ub := ((c.X-a.X)*dz1 - (c.Z-a.Z)*dx1) / det

	// Check if the potential intersection point lies on both line segments
	// If it does, calculate the intersection point's coordinates and return them
	if ua >= 0 && ua <= 1 && ub >= 0 && ub <= 1 {
		intersectionPoint := Vertex{
			X: a.X + ua*dx1,
			Y: a.Y + ua*(b.Y-a.Y), // Calculate the Y coordinate using the same parameter for the intersection point
			Z: a.Z + ua*dz1,
		}
		return &intersectionPoint
	}

	// If the potential intersection point doesn't lie on both line segments, return nil
	return nil
}

func (navMesh *NavMesh) lineIntersectsEdge(a, b, edge1, edge2 Vertex) bool {
	dir1 := b.subtract(a)
	dir2 := edge2.subtract(edge1)

	denom := dir2.X*dir1.Z - dir2.Z*dir1.X
	alphaNum := dir2.X*(edge1.Z-a.Z) - dir2.Z*(edge1.X-a.X)
	betaNum := dir1.X*(edge1.Z-a.Z) - dir1.Z*(edge1.X-a.X)

	if denom == 0 {
		return false // Lines are parallel
	}

	alpha := alphaNum / denom
	beta := betaNum / denom

	if (alpha > 0 && alpha < 1) && (beta > 0 && beta < 1) {
		// Now check if the Y coordinates also match at the intersection point
		y1 := a.Y + alpha*(b.Y-a.Y)
		y2 := edge1.Y + beta*(edge2.Y-edge1.Y)

		return math.Abs(y1-y2) < navMesh.Settings.LineIntersectsTolerance
	}

	return false //(alpha > 0 && alpha < 1) && (beta > 0 && beta < 1)
}
