package gogogadget_navmesh

import (
	"math"
)

func computeAngle(A, P, C Vertex) float64 {
	v1 := A.subtract(P).normalize()
	v2 := C.subtract(P).normalize()
	dotProduct := v1.dot(v2)
	return math.Acos(dotProduct) * (180.0 / math.Pi)
}

func rayTriangleIntersection(origin, direction, v0, v1, v2 Vertex) (*Vertex, float64) {
	// Implementation of the Moller–Trumbore intersection algorithm

	e1 := v1.subtract(v0)
	e2 := v2.subtract(v0)
	h := direction.cross(e2)
	a := e1.dot(h)

	if a > -math.SmallestNonzeroFloat64 && a < math.SmallestNonzeroFloat64 {
		return nil, 0
	}

	f := 1.0 / a
	s := origin.subtract(v0)
	u := f * s.dot(h)

	if u < 0.0 || u > 1.0 {
		return nil, 0
	}

	q := s.cross(e1)
	v := f * direction.dot(q)

	if v < 0.0 || u+v > 1.0 {
		return nil, 0
	}

	t := f * e2.dot(q)

	if t > math.SmallestNonzeroFloat64 {
		return &Vertex{
			X: origin.X + direction.X*t,
			Y: origin.Y + direction.Y*t,
			Z: origin.Z + direction.Z*t,
		}, t
	}

	return nil, 0
}

// Calculate the normal of a triangle defined by three vertices
func calculateNormal(v1, v2, v3 Vertex) Vertex {
	edge1 := v2.subtract(v1)
	edge2 := v3.subtract(v1)
	return edge1.cross(edge2).normalize()
}

func vertexInVertices(vertex Vertex, vertices []Vertex) bool {
	for _, v := range vertices {
		if v.equals(vertex) {
			return true
		}
	}
	return false
}

// Helper function to compute the cross product of two vectors.
func crossProduct(u, v Vertex) Vertex {
	return Vertex{
		X: u.Y*v.Z - u.Z*v.Y,
		Y: u.Z*v.X - u.X*v.Z,
		Z: u.X*v.Y - u.Y*v.X,
	}
}

// Helper function to subtract two vectors.
func subtractVectors2(u, v Vertex) Vertex {
	return Vertex{
		X: u.X - v.X,
		Y: u.Y - v.Y,
		Z: u.Z - v.Z,
	}
}

// Helper function to compute the length of a vector.
func vectorLength(v Vertex) float64 {
	return math.Sqrt(v.X*v.X + v.Y*v.Y + v.Z*v.Z)
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

func (navMesh *NavMesh) getFinalPath(rawPath []Vertex, exactStart, exactEnd Vertex) []Vertex {
	fp := append([]Vertex{}, navMesh.snapToNavMesh(exactStart))
	fp = append(fp, rawPath...)
	return append(fp, navMesh.snapToNavMesh(exactEnd))
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
	normal := cross(subtract(v1, v0), subtract(v2, v0))

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

func barycentric(v0, v1, v2, p Vertex) (float64, float64, float64) {
	v0v2 := subtract(v2, v0)
	v0v1 := subtract(v1, v0)
	v0p := subtract(p, v0)
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
