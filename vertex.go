package gogogadget_navmesh

import "math"

type Vertex struct {
	X, Y, Z float64
}

func (v Vertex) equals(other Vertex) bool {
	tolerance := 1e-9
	return math.Abs(v.X-other.X) < tolerance && math.Abs(v.Y-other.Y) < tolerance && math.Abs(v.Z-other.Z) < tolerance
}

func (v Vertex) distanceTo(other Vertex) float64 {
	return math.Sqrt(math.Pow(v.X-other.X, 2) + math.Pow(v.Y-other.Y, 2) + math.Pow(v.Z-other.Z, 2))
}

func distance(a, b Vertex) float64 {
	return math.Sqrt(math.Pow(a.X-b.X, 2) + math.Pow(a.Y-b.Y, 2) + math.Pow(a.Z-b.Z, 2))
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

func subtract(a, b Vertex) Vertex {
	return Vertex{a.X - b.X, a.Y - b.Y, a.Z - b.Z}
}

// Dot product
func (v1 Vertex) dot(v2 Vertex) float64 {
	return v1.X*v2.X + v1.Y*v2.Y + v1.Z*v2.Z
}

// Multiply by a scalar
func (v Vertex) multiply(scalar float64) Vertex {
	return Vertex{v.X * scalar, v.Y * scalar, v.Z * scalar}
}

// Add two vertices
func (v1 Vertex) add(v2 Vertex) Vertex {
	return Vertex{v1.X + v2.X, v1.Y + v2.Y, v1.Z + v2.Z}
}

// Vector subtraction
func (v1 Vertex) subtract(v2 Vertex) Vertex {
	return Vertex{v1.X - v2.X, v1.Y - v2.Y, v1.Z - v2.Z}
}

// Cross product
func (v1 Vertex) cross(v2 Vertex) Vertex {
	return Vertex{
		v1.Y*v2.Z - v1.Z*v2.Y,
		v1.Z*v2.X - v1.X*v2.Z,
		v1.X*v2.Y - v1.Y*v2.X,
	}
}

// Normalize function for Vertex
func (v Vertex) normalize() Vertex {
	magnitude := math.Sqrt(v.X*v.X + v.Y*v.Y + v.Z*v.Z)
	return Vertex{v.X / magnitude, v.Y / magnitude, v.Z / magnitude}
}
