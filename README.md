## The definitely extremely inefficient but on the other hand also kind of inaccurate NavMesh-based 3D pathfinding Go package that loads Godot Scenes your mother probably warned you about

anyway use it like this

```go
gonav "github.com/nurjeff/gogogadget_navmesh"

func test() {
	nml := gonav.NavMeshLoader{}
	nml.LoadFromPath("../node_3d.tscn", nil)

	// We're just loading a single NavMesh
	nm := nml.NavMeshes[0]

	// Set Start- and Endpoint
	start := gonav.Vertex{X: -3.739, Y: 0.0, Z: 0.308}
	end := gonav.Vertex{X: 20.273, Y: 5.3, Z: 2.751}

	// Find a path. If no path can be found, this will return an error
	path, _ := nm.PathFind(start, end)

	for _, ele := range path {
		fmt.Println(ele)
	}

	/* Prints:
	----------
	{-3.739 0.23989900000000003 0.308}
	{-3.346343958333333 0.23989900000000003 0.5002916666666666}
	{-2.9536879166666665 0.239899 0.6925833333333333}
	{-2.561031875 0.239899 0.8848749999999999}
	{-2.168375833333333 0.239899 1.0771666666666666}
	{-2.0849733439163387 0.239899 1.5184843844740805}
	{-1.8462281122900088 0.239899 2.230657831564987}
	{-0.5793313577586205 0.239899 2.681732543103448}
	{-0.3071422413793101 0.239899 2.5728568965517242}
	{0.012861443965517472 0.239899 2.583517672413793}
	{0.33286512931034506 0.239899 2.594178448275862}
	{0.6528688146551727 0.239899 2.604839224137931}
	{0.9728725000000003 0.23989900000000003 2.6155}
	.
	.
	.
	----------
	*/
}
```

This uses an A* implementation to find connected Vertices on the NavMesh, and then does some hilarious, freestyled math to find a path. Might be useful for server-side world simulation, at least that is what i'm planning to use this for.

Supports path smoothing and a form of string pulling so the agent does not stricly follow the vertices and chooses a better path where possible, though this is definitely only done approximatively at the moment.

Automatically searches for a NavMeshRegion in the Godot scene file loaded, so bake a sane NavMesh first.

![](https://raw.githubusercontent.com/nurjeff/gogogadget_navmesh/main/example.gif)