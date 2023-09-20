package gogogadget_navmesh

import "math"

type Vertex struct {
	X, Y, Z float64
}

func (v Vertex) length() float64 {
	return math.Sqrt(v.X*v.X + v.Y*v.Y + v.Z*v.Z)
}

func (v Vertex) angleWith(other Vertex) float64 {
	dotProduct := v.X*other.X + v.Y*other.Y + v.Z*other.Z
	magnitudeV := math.Sqrt(v.X*v.X + v.Y*v.Y + v.Z*v.Z)
	magnitudeOther := math.Sqrt(other.X*other.X + other.Y*other.Y + other.Z*other.Z)

	cosTheta := dotProduct / (magnitudeV * magnitudeOther)
	angleInRadians := math.Acos(math.Min(math.Max(cosTheta, -1.0), 1.0)) // Clamp cosTheta to [-1, 1] to avoid NaN errors

	return angleInRadians * (180.0 / math.Pi) // Convert angle to degrees
}

func (v Vertex) subtract(other Vertex) Vertex {
	return Vertex{
		X: v.X - other.X,
		Y: v.Y - other.Y,
		Z: v.Z - other.Z,
	}
}

func (v Vertex) Equals(other Vertex) bool {
	tolerance := 1e-9
	return math.Abs(v.X-other.X) < tolerance && math.Abs(v.Y-other.Y) < tolerance && math.Abs(v.Z-other.Z) < tolerance
}

func (v Vertex) distanceTo(other Vertex) float64 {
	return math.Sqrt(math.Pow(v.X-other.X, 2) + math.Pow(v.Y-other.Y, 2) + math.Pow(v.Z-other.Z, 2))
}

func vertexInVertices(vertex Vertex, vertices []Vertex) bool {
	for _, v := range vertices {
		if v.Equals(vertex) {
			return true
		}
	}
	return false
}
