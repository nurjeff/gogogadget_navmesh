package gogogadget_navmesh

import (
	"errors"
	"io"
	"os"
	"regexp"
	"strconv"
	"strings"
	"sync"
)

type NavMeshLoader struct {
	id          int
	initialized bool
	navMeshes   sync.Map
}

// Loads a NavMesh from a Path. This should point to a Godot Scene file
// If no settings for the NavMesh are passed, default values are used.
func (nml *NavMeshLoader) LoadFromPath(path string, settings *NavMeshSettings) error {
	if !nml.initialized {
		return errors.New("load a navmesh first")
	}
	nm, err := nml.loadAndParse(path)
	if err != nil {
		return err
	}

	var findErr error = nil
	nml.navMeshes.Range(func(key any, value any) bool {
		n := value.(NavMesh)
		if n.FileName == nm.FileName {
			findErr = errors.New("navmesh with filename " + nm.FileName + " already exists")
			return false
		}
		return true
	})
	if findErr != nil {
		return findErr
	}

	if err := nm.initialize(settings); err != nil {
		return err
	}

	nml.navMeshes.Store(nm.FileName, nm)
	return nil
}

// Loads a NavMesh from a Path with a name. This should point to a Godot Scene file
// If no settings for the NavMesh are passed, default values are used.
func (nml *NavMeshLoader) LoadFromPathName(path string, settings *NavMeshSettings, name string) error {
	if !nml.initialized {
		return errors.New("load a navmesh first")
	}
	nm, err := nml.loadAndParse(path)
	if err != nil {
		return err
	}

	var findErr error = nil
	nml.navMeshes.Range(func(key any, value any) bool {
		n := value.(NavMesh)
		if n.FileName == nm.FileName {
			findErr = errors.New("navmesh with filename " + nm.FileName + " already exists")
			return false
		}
		return true
	})
	if findErr != nil {
		return findErr
	}

	if err := nm.initialize(settings); err != nil {
		return err
	}

	nm.ShortName = name
	nml.navMeshes.Store(nm.ShortName, nm)
	return nil
}

func (nml *NavMeshLoader) GetByName(name string) (*NavMesh, error) {
	if len(name) == 0 {
		return nil, errors.New("name can't be empty")
	}

	var nm *NavMesh
	nml.navMeshes.Range(func(key any, value any) bool {
		n := value.(NavMesh)
		if n.ShortName == name {
			nm = &n
			return false
		}
		return true
	})

	if nm == nil {
		return nil, errors.New("navmesh " + name + " not found")
	}
	return nm, nil
}

func (nml *NavMeshLoader) loadAndParse(path string) (NavMesh, error) {
	if !nml.initialized {
		nml.navMeshes = sync.Map{}
		nml.initialized = true
	}
	file, err := os.Open(path)
	if err != nil {
		return NavMesh{}, err
	}
	defer file.Close()
	data, err := io.ReadAll(file)
	if err != nil {
		return NavMesh{}, err
	}

	sceneData := string(data)
	// Define regular expressions to match the relevant portion
	navMeshRegex := regexp.MustCompile(`\[sub_resource type="NavigationMesh" id="NavigationMesh_unj38"\]\n(\s*)vertices = PackedVector3Array\(([\s\S]*?)\)\n(\s*)polygons = \[([\s\S]*?)\]`)
	match := navMeshRegex.FindStringSubmatch(sceneData)
	if len(match) != 5 {
		return NavMesh{}, errors.New("navigationMesh sub-resource not found")
	}

	// Parse vertices
	vertexStr := strings.TrimSpace(match[2])
	vertexValues := strings.Split(vertexStr, ", ")
	vertices := make([]Vertex, len(vertexValues)/3)
	for i := 0; i < len(vertexValues); i += 3 {
		x, _ := strconv.ParseFloat(vertexValues[i], 64)
		y, _ := strconv.ParseFloat(vertexValues[i+1], 64)
		z, _ := strconv.ParseFloat(vertexValues[i+2], 64)
		vertices[i/3] = Vertex{X: x, Y: y, Z: z}
	}

	// Parse triangles
	polygonStr := strings.TrimSpace(match[4])
	polygonValues := strings.Split(polygonStr, "), ")
	triangles := make([][3]int, 0)
	for index, ele := range polygonValues {
		cleaned := ele[17:]
		if index == len(polygonValues)-1 {
			cleaned = cleaned[:len(cleaned)-1]
		}
		vals := strings.Split(cleaned, ", ")
		x, _ := strconv.ParseInt(vals[0], 10, 64)
		y, _ := strconv.ParseInt(vals[1], 10, 64)
		z, _ := strconv.ParseInt(vals[2], 10, 64)
		triangle := [3]int{int(x), int(y), int(z)}
		triangles = append(triangles, triangle)
	}

	nml.id++
	return NavMesh{Vertices: vertices, Triangles: triangles, FileName: path, IsInitialized: false, ID: nml.id}, nil
}
