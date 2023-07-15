package detour

import (
	"encoding/binary"
	"io"
	"math"
)

type navMeshSetHeader struct {
	Magic    uint32
	Version  uint32
	NumTiles uint32
	_ uint32 // UE5: navmesh param alignment is 8, so need to pad out 4 bytes here.
	Params   NavMeshParams
}

func (s *navMeshSetHeader) size() int {
	return 12 + s.Params.size()
}

func (s *navMeshSetHeader) serialize(dst []byte) {
	if len(dst) < s.size() {
		panic("undersized buffer for navMeshSetHeader")
	}
	var (
		little = binary.LittleEndian
		off    int
	)

	// write each field as little endian
	little.PutUint32(dst[off:], s.Magic)
	little.PutUint32(dst[off+4:], s.Version)
	little.PutUint32(dst[off+8:], s.NumTiles)
	s.Params.serialize(dst[off+12:])
}

func (s *navMeshSetHeader) WriteTo(w io.Writer) (int64, error) {
	buf := make([]byte, s.size())
	s.serialize(buf)

	n, err := w.Write(buf)
	return int64(n), err
}

// NavMeshParams contains the configuration parameters used to define
// multi-tile navigation meshes.
//
// The values are used to allocate space during the initialization of a
// navigation mesh.
// see NavMesh.Init()
type NavMeshParams struct {

	// UE5 optimizes these fields which in recast exist in the tile data.
	WalkableHeight float64
	WalkableRadius float64
	WalkableClimb float64

	// UE5 stores BV quant values at different resolutions.
	BvQuants [3]float64

	Orig       [3]float64 // The world space origin of the navigation mesh's tile space. [(x, y, z)]
	TileWidth  float64    // The width of each tile. (Along the x-axis.)
	TileHeight float64    // The height of each tile. (Along the z-axis.)
	MaxTiles   uint32     // The maximum number of tiles the navigation mesh can contain.
	MaxPolys   uint32     // The maximum number of polygons each tile can contain.
}

// size returns the size of the serialized structure.
func (s *NavMeshParams) size() int {
	return 28
}

// serialize encodes the structure content into dst.
//
// The function panics is the destination slice is too small.
func (s *NavMeshParams) serialize(dst []byte) {
	if len(dst) < s.size() {
		panic("destination slice is too small")
	}
	var (
		little = binary.LittleEndian
		off    int
	)

	// write each field as little endian
	little.PutUint32(dst[off:], uint32(math.Float64bits(s.Orig[0])))
	little.PutUint32(dst[off+4:], uint32(math.Float64bits(s.Orig[1])))
	little.PutUint32(dst[off+8:], uint32(math.Float64bits(s.Orig[2])))
	little.PutUint32(dst[off+12:], uint32(math.Float64bits(s.TileWidth)))
	little.PutUint32(dst[off+16:], uint32(math.Float64bits(s.TileHeight)))
	little.PutUint32(dst[off+20:], uint32(s.MaxTiles))
	little.PutUint32(dst[off+24:], uint32(s.MaxPolys))
}

// MeshHeader provides high level information related to a MeshTile object.
type MeshHeader struct {
	Magic           int32      // Tile magic number. (Used to identify the data format.)
	Version         int32      // Tile data format version number.
	X               int32      // The x-position of the tile within the NavMesh tile grid. (x, y, layer)
	Y               int32      // The y-position of the tile within the NavMesh tile grid. (x, y, layer)
	Layer           int32      // The layer of the tile within the NavMesh tile grid. (x, y, layer)
	UserID          uint32     // The user defined id of the tile.
	PolyCount       int32      // The number of polygons in the tile.
	VertCount       int32      // The number of vertices in the tile.
	MaxLinkCount    int32      // The number of allocated links.
	DetailMeshCount int32      // The number of sub-meshes in the detail mesh.
	DetailVertCount int32      // The number of unique vertices in the detail mesh. (In addition to the polygon vertices.)
	DetailTriCount  int32      // The number of triangles in the detail mesh.
	BvNodeCount     int32      // The number of bounding volume nodes. (Zero if bounding volumes are disabled.)
	OffMeshConCount int32      // The number of off-mesh connections.
	OffMeshBase     int32      // The index of the first polygon which is an off-mesh connection.
	WalkableHeight  float32    // The height of the agents using the tile.
	WalkableRadius  float32    // The radius of the agents using the tile.
	WalkableClimb   float32    // The maximum climb height of the agents using the tile.
	BMin            [3]float64 // The minimum bounds of the tile's AABB. [(x, y, z)]
	BMax            [3]float64 // The maximum bounds of the tile's AABB. [(x, y, z)]
	BvQuantFactor   float32    // The bounding volume quantization factor.
}

// UE5: By default, UE stores additional data in each mesh header. These are:
// short offMeshSegConCount;
// short offMeshSegPolyBase;
// short offMeshSegVertBase;
// short clusterCount;
// short resolution;
// Each is 16b, so needs to ignore this many bytes when reading files.
const skipTileHeaderBytes = 5 * 2

func (s *MeshHeader) size() int {
	return 78 + skipTileHeaderBytes // UE5: The changed fields alter the size of our header.
}

func (s *MeshHeader) serialize(dst []byte) {
	if len(dst) < s.size() {
		panic("undersized buffer for MeshHeader")
	}
	var (
		little = binary.LittleEndian
		off    int
	)

	// write each field as little endian
	little.PutUint32(dst[off:], uint32(s.Magic))
	little.PutUint32(dst[off+4:], uint32(s.Version))
	little.PutUint32(dst[off+8:], uint32(s.X))
	little.PutUint32(dst[off+12:], uint32(s.Y))
	little.PutUint32(dst[off+16:], uint32(s.Layer))
	little.PutUint32(dst[off+20:], uint32(s.UserID))
	little.PutUint32(dst[off+24:], uint32(s.PolyCount))
	little.PutUint32(dst[off+28:], uint32(s.VertCount))
	little.PutUint32(dst[off+32:], uint32(s.MaxLinkCount))
	little.PutUint32(dst[off+36:], uint32(s.DetailMeshCount))
	little.PutUint32(dst[off+40:], uint32(s.DetailVertCount))
	little.PutUint32(dst[off+44:], uint32(s.DetailTriCount))
	little.PutUint32(dst[off+48:], uint32(s.BvNodeCount))
	little.PutUint32(dst[off+52:], uint32(s.OffMeshConCount))
	little.PutUint32(dst[off+56:], uint32(s.OffMeshBase))
	little.PutUint32(dst[off+60:], uint32(math.Float32bits(s.WalkableHeight)))
	little.PutUint32(dst[off+64:], uint32(math.Float32bits(s.WalkableRadius)))
	little.PutUint32(dst[off+68:], uint32(math.Float32bits(s.WalkableClimb)))
	little.PutUint32(dst[off+72:], uint32(math.Float64bits(s.BMin[0])))
	little.PutUint32(dst[off+76:], uint32(math.Float64bits(s.BMin[1])))
	little.PutUint32(dst[off+80:], uint32(math.Float64bits(s.BMin[2])))
	little.PutUint32(dst[off+84:], uint32(math.Float64bits(s.BMax[0])))
	little.PutUint32(dst[off+88:], uint32(math.Float64bits(s.BMax[1])))
	little.PutUint32(dst[off+92:], uint32(math.Float64bits(s.BMax[2])))
	little.PutUint32(dst[off+96:], uint32(math.Float32bits(s.BvQuantFactor)))
}

func (s *MeshHeader) unserialize(src []byte, params NavMeshParams) {
	if len(src) < s.size() {
		panic("undersized buffer for MeshHeader")
	}
	var (
		little = binary.LittleEndian
		off    int
	)

	// UE5: Rewritten this to match UE's output, which is significantly different.
	// https://github.com/EpicGames/UnrealEngine/commit/ac8a041d98f18c160569777144eba6e98228b8bf#diff-9045070659cb5abfbc28597fd7baa86f150362060571ecb25351275d9568f22a
	// Introduces a memory optimization that saves repeating shared data for every tile; instead
	// they only exist in the navmesh params, which we now accept in to this function
	// and apply to every tile.
	s.WalkableHeight = float32(params.WalkableHeight)
	s.WalkableClimb = float32(params.WalkableClimb)
	s.WalkableRadius = float32(params.WalkableHeight)
	s.BvQuantFactor = float32(params.BvQuants[0])
	s.Magic = navMeshMagic // TODO: Hardcoding this feels wrong, but it's not given to us by UE.

	s.Version = int32(little.Uint16(src[off:]))
	s.Layer = int32(little.Uint16(src[off+2:]))
	s.PolyCount = int32(little.Uint16(src[off+4:]))
	s.VertCount = int32(little.Uint16(src[off+6:]))
	s.X = int32(little.Uint32(src[off+8:]))
	s.Y = int32(little.Uint32(src[off+12:]))
	s.MaxLinkCount = int32(little.Uint16(src[off+16:]))
	s.DetailMeshCount = int32(little.Uint16(src[off+18:]))
	s.DetailVertCount = int32(little.Uint16(src[off+20:]))
	s.DetailTriCount = int32(little.Uint16(src[off+22:]))
	s.BvNodeCount = int32(little.Uint16(src[off+24:]))
	s.OffMeshConCount = int32(little.Uint16(src[off+26:]))
	s.OffMeshBase = int32(little.Uint16(src[off+28:]))

	// UE5: this is where the additional mesh headers appear. Start skipping now.

	s.BMin[0] = math.Float64frombits(little.Uint64(src[off+30+skipTileHeaderBytes:]))
	s.BMin[1] = math.Float64frombits(little.Uint64(src[off+38+skipTileHeaderBytes:]))
	s.BMin[2] = math.Float64frombits(little.Uint64(src[off+46+skipTileHeaderBytes:]))
	s.BMax[0] = math.Float64frombits(little.Uint64(src[off+54+skipTileHeaderBytes:]))
	s.BMax[1] = math.Float64frombits(little.Uint64(src[off+62+skipTileHeaderBytes:]))
	s.BMax[2] = math.Float64frombits(little.Uint64(src[off+70+skipTileHeaderBytes:]))
}
