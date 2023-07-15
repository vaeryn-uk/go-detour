[![go.dev reference](https://img.shields.io/badge/go.dev-reference-007d9c?logo=go&logoColor=white&style=flat-square)](https://pkg.go.dev/github.com/arl/go-detour)
[![Test Actions Status](https://github.com/arl/go-detour/workflows/Tests/badge.svg)](https://github.com/arl/go-detour/actions)
[![Go Report Card](https://goreportcard.com/badge/github.com/arl/go-detour)](https://goreportcard.com/report/github.com/arl/go-detour)
[![codecov](https://codecov.io/gh/arl/go-detour/branch/main/graph/badge.svg)](https://codecov.io/gh/arl/go-detour)

**A fork of the great work done in github.com/arl/go-detour**

This fork hacks at the library to support `.bin` meshes exported from Unreal Engine 
5.21 (via [this](https://github.com/vaeryn-uk/ue4-export-nav-data) UE plugin). This enables
nav meshes to be built in UE then exported to a server application. This is for 
prototyping only. These hacks may be made more sustainable if prototyping goes well.

**WARNING:** These hacks are not safe. The changes here undoubtedly break core
functionality. Do not use this.

# go-detour - port of [recast/detour](https://github.com/recastnavigation/recastnavigation) in Go

## Recast library

Recast is state of the art navigation mesh construction toolset for games.

* It is automatic, which means that you can throw any level geometry at it and you will get robust mesh out
* It is fast which means swift turnaround times for level designers
* It is open source so it comes with full source and you can customize it to your heart's content. 

The Recast process starts with constructing a voxel mold from a level geometry 
and then casting a navigation mesh over it. The process consists of three steps, 
building the voxel mold, partitioning the mold into simple regions, peeling off 
the regions as simple polygons.

1. The voxel mold is build from the input triangle mesh by rasterizing the triangles into a multi-layer heightfield. Some simple filters are  then applied to the mold to prune out locations where the character would not be able to move.
2. The walkable areas described by the mold are divided into simple overlayed 2D regions. The resulting regions have only one non-overlapping contour, which simplifies the final step of the process tremendously.
3. The navigation polygons are peeled off from the regions by first tracing the boundaries and then simplifying them. The resulting polygons are finally converted to convex polygons which makes them perfect for pathfinding and spatial reasoning about the level. 


## Recast command line interface

This package embeds a *command line interface* tool named `recast` to build 
navigation meshes:

```
$ recast -h
This is the command-line application accompanying go-detour:
        - build navigation meshes from any level geometry,
        - save them to binary files (usable in 'go-detour')
        - easily tweak build settings (YAML files),
        - check or show info about generated navmesh binaries.

Usage:
  recast [command]

Available Commands:
  build       build navigation mesh from input geometry
  config      generate a config file with default build settings
  infos       show infos about a navmesh

Use "recast [command] --help" for more information about a command.
```

Installation of the cli tool:

```
go get -u github.com/arl/go-detour/cmd/recast
```


## Detour library

Recast is accompanied with Detour, path-finding and spatial reasoning toolkit. You can use any navigation mesh with Detour, but of course the data generated with Recast fits perfectly.

Detour offers simple static navigation mesh which is suitable for many simple cases, as well as tiled navigation mesh which allows you to plug in and out pieces of the mesh. The tiled mesh allows you to create systems where you stream new navigation data in and out as the player progresses the level, or you may regenerate tiles as the world changes. 


## Compatibility with original **Detour & Recast**

The Go version and the original works both with the same binary format to save navmeshes, that means:
- you can use in the original **Detour** the navmeshes built in Go.
- you can use in Go the navmeshes built with the original **Recast**.

In other words, you can visualize and tweak your navmeshes with the handy GUI tool 
[**RecastDemo**](https://github.com/recastnavigation/recastnavigation). Once you 
are satisfied with the navmesh of your geometry, use the same build settings with 
either the `recast` Go package or the cli tool in order to have **identical results**.


## Samples

*todo*

... speak about the implemented sample from **RecastDemo** ...

## Package structure

*todo*:
... speak about the package structure or at least explain the big idea ...


## Minimum Go Version

Go 1.8+ is required

## Credits

**[go-detour][1]** has been ported to the Go language. As such, it's not an
original work.


The **[Detour & Recast][2]** libraries (which go-detour is port of) is an
original work from, and has been written by, Mikko Mononen, and released 
under the following:
> copyright (c) 2009 Mikko Mononen memon@inside.org.


## License

**[go-detour][1]** is licensed under ZLib license, see [LICENSE][5] for more
information.


[1]: https://github.com/arl/go-detour "go-detour"
[2]: https://github.com/recastnavigation/recastnavigation "Recast & Detour"
[3]: https://github.com/golang/go "The Go language"
[4]: https://github.com/arl
[5]: ./LICENSE "License"
