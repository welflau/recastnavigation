//
// Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
//
// This software is provided 'as-is', without any express or implied
// warranty.  In no event will the authors be held liable for any damages
// arising from the use of this software.
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
// 1. The origin of this software must not be misrepresented; you must not
//    claim that you wrote the original software. If you use this software
//    in a product, an acknowledgment in the product documentation would be
//    appreciated but is not required.
// 2. Altered source versions must be plainly marked as such, and must not be
//    misrepresented as being the original software.
// 3. This notice may not be removed or altered from any source distribution.
//

#ifndef DETOURNAVMESH_H
#define DETOURNAVMESH_H

#include "DetourAlloc.h"
#include "DetourStatus.h"
#include "FreeList.h"
#include "DetourCommon.h"
#include "AABB.h"

// Undefine (or define in a build cofnig) the following line to use 64bit polyref.
// Generally not needed, useful for very large worlds.
// Note: tiles build using 32bit refs are not compatible with 64bit refs!
//#define DT_POLYREF64 1

#ifdef DT_POLYREF64
// TODO: figure out a multiplatform version of uint64_t
// - maybe: https://code.google.com/p/msinttypes/
// - or: http://www.azillionmonkeys.com/qed/pstdint.h
#include <stdint.h>
#endif

// Note: If you want to use 64-bit refs, change the types of both dtPolyRef & dtTileRef.
// It is also recommended that you change dtHashRef() to a proper 64-bit hash.

/// A handle to a polygon within a navigation mesh tile.
/// @ingroup detour
#ifdef DT_POLYREF64
static const unsigned int DT_SALT_BITS = 16;
static const unsigned int DT_TILE_BITS = 28;
static const unsigned int DT_POLY_BITS = 20;
typedef uint64_t dtPolyRef;
#else
typedef unsigned int dtPolyRef;
#endif

/// A handle to a tile within a navigation mesh.
/// @ingroup detour
#ifdef DT_POLYREF64
typedef uint64_t dtTileRef;
#else
typedef unsigned int dtTileRef;
#endif

/// The maximum number of vertices per navigation polygon.
/// @ingroup detour
static const int DT_VERTS_PER_POLYGON = 6;

/// @{
/// @name Tile Serialization Constants
/// These constants are used to detect whether a navigation tile's data
/// and state format is compatible with the current build.
///

/// A magic number used to detect compatibility of navigation tile data.
static const int DT_NAVMESH_MAGIC = 'D'<<24 | 'N'<<16 | 'A'<<8 | 'V';

/// A version number used to detect compatibility of navigation tile data.
static const int DT_NAVMESH_VERSION = 7;

/// A magic number used to detect the compatibility of navigation tile states.
static const int DT_NAVMESH_STATE_MAGIC = 'D'<<24 | 'N'<<16 | 'M'<<8 | 'S';

/// A version number used to detect compatibility of navigation tile states.
static const int DT_NAVMESH_STATE_VERSION = 1;

/// @}

/// A flag that indicates that an entity links to an external entity.
/// (E.g. A polygon edge is a portal that links to another polygon.)
static const unsigned short DT_EXT_LINK = 0x8000;

/// A value that indicates the entity does not link to anything.
static const unsigned int DT_NULL_LINK = 0xffffffff;

/// A flag that indicates that an off-mesh connection can be traversed in both directions. (Is bidirectional.)
static const unsigned int DT_OFFMESH_CON_BIDIR = 1;

/// The maximum number of user defined area ids.
/// @ingroup detour
static const int DT_MAX_AREAS = 64;

/// Tile flags used for various functions and fields.
/// For an example, see dtNavMesh::addTile().
enum dtTileFlags
{
	/// The navigation mesh owns the tile memory and is responsible for freeing it.
	DT_TILE_FREE_DATA = 0x01,
};

/// Vertex flags returned by dtNavMeshQuery::findStraightPath.
enum dtStraightPathFlags
{
	DT_STRAIGHTPATH_START = 0x01,				///< The vertex is the start position in the path.
	DT_STRAIGHTPATH_END = 0x02,					///< The vertex is the end position in the path.
	DT_STRAIGHTPATH_OFFMESH_CONNECTION = 0x04,	///< The vertex is the start of an off-mesh connection.
};

/// Options for dtNavMeshQuery::findStraightPath.
enum dtStraightPathOptions
{
	DT_STRAIGHTPATH_AREA_CROSSINGS = 0x01,	///< Add a vertex at every polygon edge crossing where area changes.
	DT_STRAIGHTPATH_ALL_CROSSINGS = 0x02,	///< Add a vertex at every polygon edge crossing.
};


/// Options for dtNavMeshQuery::initSlicedFindPath and updateSlicedFindPath
enum dtFindPathOptions
{
	DT_FINDPATH_ANY_ANGLE	= 0x02,		///< use raycasts during pathfind to "shortcut" (raycast still consider costs)
};

/// Options for dtNavMeshQuery::raycast
enum dtRaycastOptions
{
	DT_RAYCAST_USE_COSTS = 0x01,		///< Raycast should calculate movement cost along the ray and fill RaycastHit::cost
};


/// Limit raycasting during any angle pahfinding
/// The limit is given as a multiple of the character radius
static const float DT_RAY_CAST_LIMIT_PROPORTIONS = 50.0f;

/// Flags representing the type of a navigation mesh polygon.
enum dtPolyTypes
{
	/// The polygon is a standard convex polygon that is part of the surface of the mesh.
	DT_POLYTYPE_GROUND = 0,
	/// The polygon is an off-mesh connection consisting of two vertices.
	DT_POLYTYPE_OFFMESH_CONNECTION = 1,
};

enum NavMeshLinkDirectionFlags
{
	kLinkDirectionOneWay = 0,
	kLinkDirectionTwoWay = 1
};



/// Defines a polygon within a dtMeshTile object.
/// @ingroup detour
struct dtPoly
{
	/// The indices of the polygon's vertices.
	/// The actual vertices are located in dtMeshTile::verts.
	unsigned short verts[DT_VERTS_PER_POLYGON];

	/// Packed data representing neighbor polygons references and flags for each edge.
	unsigned short neis[DT_VERTS_PER_POLYGON];

	/// The user defined polygon flags.
	unsigned short flags;

	/// The number of vertices in the polygon.
	unsigned char vertCount;

	/// The bit packed area id and polygon type.
	/// @note Use the structure's set and get methods to acess this value.
	unsigned char area;

	/// Sets the user defined area id. [Limit: < #DT_MAX_AREAS]
	inline void setArea(unsigned char a) { area = a; }

	/// Gets the user defined area id.
	inline unsigned char getArea() const { return area; }
};

/// Defines the location of detail sub-mesh data within a dtMeshTile.
struct dtPolyDetail
{
	unsigned int vertBase;			///< The offset of the vertices in the dtMeshTile::detailVerts array.
	unsigned int triBase;			///< The offset of the triangles in the dtMeshTile::detailTris array.
	unsigned char vertCount;		///< The number of vertices in the sub-mesh.
	unsigned char triCount;			///< The number of triangles in the sub-mesh.
};

/// Defines a link between polygons.
/// @note This structure is rarely if ever used by the end user.
/// @see dtMeshTile
struct dtLink
{
	dtPolyRef ref;					///< Neighbour reference. (The neighbor that is linked to.)
	unsigned int next;				///< Index of the next link.
	unsigned char edge;				///< Index of the polygon edge that owns this link.
	unsigned char side;				///< If a boundary link, defines on which side the link is.
	unsigned char bmin;				///< If a boundary link, defines the minimum sub-edge area.
	unsigned char bmax;				///< If a boundary link, defines the maximum sub-edge area.
};

/// Bounding volume node.
/// @note This structure is rarely if ever used by the end user.
/// @see dtMeshTile
struct dtBVNode
{
	unsigned short bmin[3];			///< Minimum bounds of the node's AABB. [(x, y, z)]
	unsigned short bmax[3];			///< Maximum bounds of the node's AABB. [(x, y, z)]
	int i;							///< The node's index. (Negative for escape sequence.)
};

struct OffMeshLinkEndPoint
{
	float pos[3];                           // Position of the end point (based on input)
	Vector3f mapped[2];                     // Position mapped on navmesh
	dtTileRef tileRef;						// Reference to tile when the point is connected - otherwise zero
};

// 
enum OffMeshLinkType
{
	kLinkTypeManual = 0,
	kLinkTypeDropDown = 1,
	kLinkTypeJumpAcross = 2
};

/// Defines an navigation mesh off-mesh connection within a dtMeshTile object.
/// An off-mesh connection is a user defined traversable connection made up to two vertices.
struct dtOffMeshConnection
{
	/// The endpoints of the connection. [(ax, ay, az, bx, by, bz)]
	float pos[6];

	int agentTypeID;                        // Which agent type can use this link

	MinMaxAABB bounds;

	OffMeshLinkEndPoint endPoints[2];		// Connection End Points 末端点信息
	Vector3f axisX, axisY, axisZ;
	float width;                            // Width of the link.
	float costModifier;                     // Modify navmesh cost (multiplier applied to euclidean distance)
	/// The radius of the endpoints. [Limit: >= 0]
	float rad;		

	/// The polygon reference of the connection within the tile.
	unsigned short poly;

	unsigned char linkDirection;            // Link connection direction flags (NavMeshLinkDirectionFlags) 是否双向

	/// Link flags. 
	/// @note These are not the connection's user defined flags. Those are assigned via the 
	/// connection's dtPoly definition. These are link flags used for internal purposes.
	unsigned int flags;

	unsigned char area;                     // Area type
	unsigned short linkType;                // enum OffMeshLinkType
	/// End point side.
	unsigned char side;

	/// The id of the offmesh connection. (User assigned when the navigation mesh is built.)
	unsigned int userId;

	unsigned int firstLink;                 // Index to first link
	unsigned int salt;                      // Salt of the connection, increased on delete
	unsigned int next;                      // Next offmesh con in the active linked list, or freelist.

	inline dtOffMeshConnection() : salt(1) {}
	inline ~dtOffMeshConnection() {}
};

/// Provides high level information related to a dtMeshTile object.
/// @ingroup detour
struct dtMeshHeader
{
	int magic;				///< Tile magic number. (Used to identify the data format.)
	int version;			///< Tile data format version number.
	int x;					///< The x-position of the tile within the dtNavMesh tile grid. (x, y, layer)
	int y;					///< The y-position of the tile within the dtNavMesh tile grid. (x, y, layer)
	int layer;				///< The layer of the tile within the dtNavMesh tile grid. (x, y, layer)
	unsigned int userId;	///< The user defined id of the tile.
	int polyCount;			///< The number of polygons in the tile.
	int vertCount;			///< The number of vertices in the tile.
	int detailMeshCount;	///< The number of sub-meshes in the detail mesh.
	
	/// The number of unique vertices in the detail mesh. (In addition to the polygon vertices.)
	int detailVertCount;
	
	int detailTriCount;			///< The number of triangles in the detail mesh.
	int bvNodeCount;			///< The number of bounding volume nodes. (Zero if bounding volumes are disabled.)
	float walkableHeight;		///< The height of the agents using the tile.
	float walkableRadius;		///< The radius of the agents using the tile.
	float walkableClimb;		///< The maximum climb height of the agents using the tile.
	float bmin[3];				///< The minimum bounds of the tile's AABB. [(x, y, z)]
	float bmax[3];				///< The maximum bounds of the tile's AABB. [(x, y, z)]
	
	/// The bounding volume quantization factor. 
	float bvQuantFactor;
};

/// Defines a navigation mesh tile.
/// @ingroup detour
struct dtMeshTile
{
	unsigned int salt;					///< Counter describing modifications to the tile.

	dtMeshHeader* header;				///< The tile header.
	unsigned int* polyLinks;            ///< First link index for each polygon.
	dtPoly* polys;						///< The tile polygons. [Size: dtMeshHeader::polyCount]
	float* verts;						///< The tile vertices. [Size: dtMeshHeader::vertCount]

	dtPolyDetail* detailMeshes;			///< The tile's detail sub-meshes. [Size: dtMeshHeader::detailMeshCount]
	
	/// The detail mesh's unique vertices. [(x, y, z) * dtMeshHeader::detailVertCount]
	float* detailVerts;	

	/// The detail mesh's triangles. [(vertA, vertB, vertC) * dtMeshHeader::detailTriCount]
	unsigned char* detailTris;	

	/// The tile bounding volume nodes. [Size: dtMeshHeader::bvNodeCount]
	/// (Will be null if bounding volumes are disabled.)
	dtBVNode* bvTree;
	
	unsigned char* data;					///< The tile data. (Not directly accessed under normal situations.)
	int dataSize;							///< Size of the tile data.
	int flags;								///< Tile flags. (See: #dtTileFlags)
	dtMeshTile* next;						///< The next free tile, or the next tile in the spatial grid.
private:
	dtMeshTile(const dtMeshTile&);
	dtMeshTile& operator=(const dtMeshTile&);
};

/// Configuration parameters used to define multi-tile navigation meshes.
/// The values are used to allocate space during the initialization of a navigation mesh.
/// @see dtNavMesh::init()
/// @ingroup detour
struct dtNavMeshParams
{
	float orig[3];					///< The world space origin of the navigation mesh's tile space. [(x, y, z)]
	float tileWidth;				///< The width of each tile. (Along the x-axis.)
	float tileHeight;				///< The height of each tile. (Along the z-axis.)
	int maxTiles;					///< The maximum number of tiles the navigation mesh can contain.
	int maxPolys;					///< The maximum number of polygons each tile can contain.
	float cellSize;					///< Cell size in world unitss	
	float tileSize;					///< Size of the tiles in voxels
	float agentHeight;
	float agentRadius;
	float agentMaxClimb;
};

struct OffMeshConnectionParams
{
	float startPos[3];
	float endPos[3];
	Vector3f up;
	float width;                            // If width > 0.0f, link is segment-to-segment.
	float costModifier;                     // Modify navmesh cost (multiplier applied to euclidean distance)
	unsigned char linkDirection;            // Link connection direction flags (NavMeshLinkDirectionFlags)
	unsigned int flags;
	unsigned char area;
	unsigned short linkType;                // enum OffMeshLinkType
	int userID;                             // User ID to identify this connection.
	int agentTypeID;                        // Which agent type can use this link
};

#ifdef DT_POLYREF64
static const unsigned int kPolyRefSaltBits = 16; // Number of salt bits in the poly/tile ID.
static const unsigned int kPolyRefTileBits = 28; // Number of tile bits in the poly/tile ID.
static const unsigned int kPolyRefPolyBits = 16; // Number of poly bits in the poly/tile ID.
static const unsigned int kPolyRefTypeBits = 4;  // Number of type bits in the poly/tile ID.

static const dtPolyRef kPolyRefSaltMask = ((dtPolyRef)1 << kPolyRefSaltBits) - 1;
static const dtPolyRef kPolyRefTileMask = ((dtPolyRef)1 << kPolyRefTileBits) - 1;
static const dtPolyRef kPolyRefPolyMask = ((dtPolyRef)1 << kPolyRefPolyBits) - 1;
static const dtPolyRef kPolyRefTypeMask = ((dtPolyRef)1 << kPolyRefTypeBits) - 1;
#endif

struct NavMeshProcessCallback
{
	virtual void ProcessPolygons(const dtMeshTile* tile, const dtPolyRef* polyRefs, dtPoly** polys, const int itemCount) = 0;
};

/// A navigation mesh based on tiles of convex polygons.
/// @ingroup detour
class dtNavMesh
{
public:
	dtNavMesh();
	~dtNavMesh();

	/// @{
	/// @name Initialization and Tile Management

	/// Initializes the navigation mesh for tiled use.
	///  @param[in]	params		Initialization parameters.
	/// @return The status flags for the operation.
	dtStatus init(const dtNavMeshParams* params);

	/// Initializes the navigation mesh for single tile use.
	///  @param[in]	data		Data of the new tile. (See: #dtCreateNavMeshData)
	///  @param[in]	dataSize	The data size of the new tile.
	///  @param[in]	flags		The tile flags. (See: #dtTileFlags)
	/// @return The status flags for the operation.
	///  @see dtCreateNavMeshData
	dtStatus init(unsigned char* data, const int dataSize, const int flags);
	
	/// The navigation mesh initialization params.
	const dtNavMeshParams* getParams() const;

	/// Adds a tile to the navigation mesh.
	///  @param[in]		data		Data for the new tile mesh. (See: #dtCreateNavMeshData)
	///  @param[in]		dataSize	Data size of the new tile mesh.
	///  @param[in]		flags		Tile flags. (See: #dtTileFlags)
	///  @param[in]		lastRef		The desired reference for the tile. (When reloading a tile.) [opt] [Default: 0]
	///  @param[out]	result		The tile reference. (If the tile was succesfully added.) [opt]
	/// @return The status flags for the operation.
	dtStatus addTile(unsigned char* data, int dataSize, int flags, dtTileRef lastRef, dtTileRef* result);
	
	/// Removes the specified tile from the navigation mesh.
	///  @param[in]		ref			The reference of the tile to remove.
	///  @param[out]	data		Data associated with deleted tile.
	///  @param[out]	dataSize	Size of the data associated with deleted tile.
	/// @return The status flags for the operation.
	dtStatus removeTile(dtTileRef ref, unsigned char** data, int* dataSize);

	/// @}

	/// @{
	/// @name Query Functions

	/// Calculates the tile grid location for the specified world position.
	///  @param[in]	pos  The world position for the query. [(x, y, z)]
	///  @param[out]	tx		The tile's x-location. (x, y)
	///  @param[out]	ty		The tile's y-location. (x, y)
	void calcTileLoc(const float* pos, int* tx, int* ty) const;

	/// Gets the tile at the specified grid location.
	///  @param[in]	x		The tile's x-location. (x, y, layer)
	///  @param[in]	y		The tile's y-location. (x, y, layer)
	///  @param[in]	layer	The tile's layer. (x, y, layer)
	/// @return The tile, or null if the tile does not exist.
	const dtMeshTile* getTileAt(const int x, const int y, const int layer) const;

	/// Gets all tiles at the specified grid location. (All layers.)
	///  @param[in]		x			The tile's x-location. (x, y)
	///  @param[in]		y			The tile's y-location. (x, y)
	///  @param[out]	tiles		A pointer to an array of tiles that will hold the result.
	///  @param[in]		maxTiles	The maximum tiles the tiles parameter can hold.
	/// @return The number of tiles returned in the tiles array.
	int getTilesAt(const int x, const int y,
				   dtMeshTile const** tiles, const int maxTiles) const;
	
	/// Gets the tile reference for the tile at specified grid location.
	///  @param[in]	x		The tile's x-location. (x, y, layer)
	///  @param[in]	y		The tile's y-location. (x, y, layer)
	///  @param[in]	layer	The tile's layer. (x, y, layer)
	/// @return The tile reference of the tile, or 0 if there is none.
	dtTileRef getTileRefAt(int x, int y, int layer) const;

	/// Gets the tile reference for the specified tile.
	///  @param[in]	tile	The tile.
	/// @return The tile reference of the tile.
	dtTileRef getTileRef(const dtMeshTile* tile) const;

	/// Gets the tile for the specified tile reference.
	///  @param[in]	ref		The tile reference of the tile to retrieve.
	/// @return The tile for the specified reference, or null if the 
	///		reference is invalid.
	const dtMeshTile* getTileByRef(dtTileRef ref) const;
	
	/// The maximum number of tiles supported by the navigation mesh.
	/// @return The maximum number of tiles supported by the navigation mesh.
	int getMaxTiles() const;
	
	/// Gets the tile at the specified index.
	///  @param[in]	i		The tile index. [Limit: 0 >= index < #getMaxTiles()]
	/// @return The tile at the specified index.
	const dtMeshTile* getTile(int i) const;

	/// Gets the tile and polygon for the specified polygon reference.
	///  @param[in]		ref		The reference for the a polygon.
	///  @param[out]	tile	The tile containing the polygon.
	///  @param[out]	poly	The polygon.
	/// @return The status flags for the operation.
	dtStatus getTileAndPolyByRef(const dtPolyRef ref, const dtMeshTile** tile, const dtPoly** poly) const;

	/// Returns the tile and polygon for the specified polygon reference.
	///  @param[in]		ref		A known valid reference for a polygon.
	///  @param[out]	tile	The tile containing the polygon.
	///  @param[out]	poly	The polygon.
	void getTileAndPolyByRefUnsafe(const dtPolyRef ref, const dtMeshTile** tile, const dtPoly** poly) const;

	/// Checks the validity of a polygon reference.
	///  @param[in]	ref		The polygon reference to check.
	/// @return True if polygon reference is valid for the navigation mesh.
	bool isValidPolyRef(dtPolyRef ref) const;
	
	/// Gets the polygon reference for the tile's base polygon.
	///  @param[in]	tile		The tile.
	/// @return The polygon reference for the base polygon in the specified tile.
	dtPolyRef getPolyRefBase(const dtMeshTile* tile) const;
	
	/// Gets the endpoints for an off-mesh connection, ordered by "direction of travel".
	///  @param[in]		prevRef		The reference of the polygon before the connection.
	///  @param[in]		polyRef		The reference of the off-mesh connection polygon.
	///  @param[out]	startPos	The start position of the off-mesh connection. [(x, y, z)]
	///  @param[out]	endPos		The end position of the off-mesh connection. [(x, y, z)]
	/// @return The status flags for the operation.
	dtStatus getOffMeshConnectionPolyEndPoints(dtPolyRef prevRef, dtPolyRef polyRef, float* startPos, float* endPos) const;


	// polyRef - polyRef of the offmeshlink polygon
	// costOverride - use this as cost OffMeshLink cost modifier instead of what is specifed by areas.
	dtStatus SetOffMeshConnectionCostModifier(dtPolyRef polyRef, float costOverride);
	dtStatus SetOffMeshConnectionFlags(dtPolyRef polyRef, unsigned int flags);
	void SetOffMeshConnectionActive(dtPolyRef ref, bool active);

	/// Gets the specified off-mesh connection.
	///  @param[in]	ref		The polygon reference of the off-mesh connection.
	/// @return The specified off-mesh connection, or null if the polygon reference is not valid.
	const dtOffMeshConnection* GetOffMeshConnection(const dtPolyRef ref) const;
	dtOffMeshConnection* GetOffMeshConnectionUnsafe(const dtPolyRef ref);
	
	/// @}

	/// @{
	/// @name State Management
	/// These functions do not effect #dtTileRef or #dtPolyRef's. 

	/// Sets the user defined flags for the specified polygon.
	///  @param[in]	ref		The polygon reference.
	///  @param[in]	flags	The new flags for the polygon.
	/// @return The status flags for the operation.
	dtStatus setPolyFlags(dtPolyRef ref, unsigned short flags);

	/// Gets the user defined flags for the specified polygon.
	///  @param[in]		ref				The polygon reference.
	///  @param[out]	resultFlags		The polygon flags.
	/// @return The status flags for the operation.
	//dtStatus getPolyFlags(dtPolyRef ref, unsigned short* resultFlags) const;
	unsigned int GetPolyFlags(dtPolyRef ref) const;

	/// Sets the user defined area for the specified polygon.
	///  @param[in]	ref		The polygon reference.
	///  @param[in]	area	The new area id for the polygon. [Limit: < #DT_MAX_AREAS]
	/// @return The status flags for the operation.
	dtStatus setPolyArea(dtPolyRef ref, unsigned char area);

	/// Gets the user defined area for the specified polygon.
	///  @param[in]		ref			The polygon reference.
	///  @param[out]	resultArea	The area id for the polygon.
	/// @return The status flags for the operation.
	//dtStatus getPolyArea(dtPolyRef ref, unsigned char* resultArea) const;
	unsigned char GetPolyArea(dtPolyRef ref) const;

	void GetPolyFlagsAndArea(dtPolyRef ref, unsigned int* flags, unsigned char* area) const;

	// Returns vertices and neighbours of polygon pointed to by the polygon reference.
	// Note: this function returns 0 if ref is invalid
	// Params:
	//  ref - (in) reference to a polygon.
	//  verts - (out, optional) pointer to polygon vertices, must have a capacity of at least kNavMeshVertsPerPoly.
	//  neighbours - (out, optional) pointer to the polygon neighbours, must have a capacity of at least kNavMeshVertsPerPoly*maxNeisPerEdge.
	//  maxNeisPerEdge - (int) maximum number of neighbours stored in 'neighbours' per edge.
	int GetPolyGeometry(dtPolyRef ref, float* verts, dtPolyRef* neighbours, int maxNeisPerEdge) const;


	/// Gets the size of the buffer required by #storeTileState to store the specified tile's state.
	///  @param[in]	tile	The tile.
	/// @return The size of the buffer required to store the state.
	int getTileStateSize(const dtMeshTile* tile) const;
	
	/// Stores the non-structural state of the tile in the specified buffer. (Flags, area ids, etc.)
	///  @param[in]		tile			The tile.
	///  @param[out]	data			The buffer to store the tile's state in.
	///  @param[in]		maxDataSize		The size of the data buffer. [Limit: >= #getTileStateSize]
	/// @return The status flags for the operation.
	dtStatus storeTileState(const dtMeshTile* tile, unsigned char* data, const int maxDataSize) const;
	
	/// Restores the state of the tile.
	///  @param[in]	tile			The tile.
	///  @param[in]	data			The new state. (Obtained from #storeTileState.)
	///  @param[in]	maxDataSize		The size of the state within the data buffer.
	/// @return The status flags for the operation.
	dtStatus restoreTileState(dtMeshTile* tile, const unsigned char* data, const int maxDataSize);


	// Returns polygons which overlap the query box.
	// Params:
	//  typeID - (in) query surfaces with this id. -1 means query all surfaces.
	//  center[3] - (in) the center of the search box.
	//  extents[3] - (in) the extents of the search box.
	//  callback - (in/out) pointer to callback interface to batch-process the results, see NavMeshProcessCallback.
	void QueryPolygons(int typeID, const Vector3f& center, const Vector3f& extents,
		NavMeshProcessCallback* callback) const;

	// Queries polygons within a tile.
	void QueryPolygonsInTile(const dtMeshTile* tile, const Vector3f& center, const Vector3f& extents, NavMeshProcessCallback* callback) const;

	inline dtPolyRef EncodeLinkId(unsigned int salt, unsigned int ip) const
	{
#ifdef DT_POLYREF64
		const unsigned int tileMax = (1 << kPolyRefTileBits) - 1;
#else
		const unsigned int tileMax = (1 << m_tileBits) - 1;
#endif
		return EncodePolyId(salt, tileMax, DT_POLYTYPE_OFFMESH_CONNECTION, ip);
	}


	// Encodes a poly id.
	inline dtPolyRef EncodePolyId(unsigned int salt, unsigned int it, unsigned int type, unsigned int ip) const
	{
#ifdef DT_POLYREF64
		return ((dtPolyRef)salt << (kPolyRefPolyBits + kPolyRefTypeBits + kPolyRefTileBits)) |
			((dtPolyRef)it << (kPolyRefPolyBits + kPolyRefTypeBits)) |
			((dtPolyRef)type << (kPolyRefPolyBits)) |
			(dtPolyRef)ip;
#else
		return ((dtPolyRef)salt << (m_polyBits + m_polyTypeBits + m_tileBits)) |
			((dtPolyRef)it << (m_polyBits + m_polyTypeBits)) |
			((dtPolyRef)type << (m_polyBits)) |
			(dtPolyRef)ip;
#endif
	}

	inline dtPolyRef EncodeBasePolyId(unsigned int type, unsigned int ip) const
	{
#ifdef DT_POLYREF64
		return ((dtPolyRef)type << (kPolyRefPolyBits)) | (dtPolyRef)ip;
#else
		return ((dtPolyRef)type << (m_polyBits)) | (dtPolyRef)ip;
#endif
	}
	
	// Decodes a poly id.
	inline void DecodePolyId(unsigned int* salt, unsigned int* it, unsigned int* type, unsigned int* ip, dtPolyRef ref) const
	{
#ifdef DT_POLYREF64
		*salt = (unsigned int)((ref >> (kPolyRefPolyBits + kPolyRefTypeBits + kPolyRefTileBits)) & kPolyRefSaltMask);
		*it = (unsigned int)((ref >> (kPolyRefPolyBits + kPolyRefTypeBits)) & kPolyRefTileMask);
		*type = (unsigned int)((ref >> (kPolyRefPolyBits)) & kPolyRefTypeMask);
		*ip = (unsigned int)(ref & kPolyRefPolyMask);
#else
		const dtPolyRef saltMask = ((dtPolyRef)1 << m_saltBits) - 1;
		const dtPolyRef tileMask = ((dtPolyRef)1 << m_tileBits) - 1;
		const dtPolyRef polyMask = ((dtPolyRef)1 << m_polyBits) - 1;
		const dtPolyRef polyTypeMask = ((dtPolyRef)1 << m_polyTypeBits) - 1;

		*salt = (unsigned int)((ref >> (m_polyBits + m_polyTypeBits + m_tileBits)) & saltMask);
		*it = (unsigned int)((ref >> (m_polyBits + m_polyTypeBits) ) & tileMask);
		*type = (unsigned int)((ref >> (m_polyBits)) & polyTypeMask);
		*ip = (unsigned int)(ref & polyMask);
#endif
	}

	// Decodes a tile salt.
	inline unsigned int DecodePolyIdSalt(dtPolyRef ref) const
	{
#ifdef DT_POLYREF64
		return (unsigned int)((ref >> (kPolyRefPolyBits + kPolyRefTypeBits + kPolyRefTileBits)) & kPolyRefSaltMask);
#else
		const dtPolyRef saltMask = ((dtPolyRef)1 << m_saltBits) - 1;
		return (unsigned int)((ref >> (m_polyBits + m_polyTypeBits + m_tileBits)) & saltMask);
#endif
	}

	// Decodes a tile id.
	inline unsigned int DecodePolyIdTile(dtPolyRef ref) const
	{
#ifdef DT_POLYREF64
		return (unsigned int)((ref >> (kPolyRefPolyBits + kPolyRefTypeBits)) & kPolyRefTileMask);
#else
		const dtPolyRef tileMask = ((dtPolyRef)1 << m_tileBits) - 1;
		return (unsigned int)((ref >> (m_polyBits + m_polyTypeBits)) & tileMask);
#endif
	}

	// Decodes a type.
	inline unsigned int DecodePolyIdType(dtPolyRef ref) const
	{
#ifdef DT_POLYREF64
		return (unsigned int)((ref >> (kPolyRefPolyBits)) & kPolyRefTypeMask);
#else
		const dtPolyRef polyTypeMask = ((dtPolyRef)1 << m_polyTypeBits) - 1;
		return (unsigned int)((ref >> (m_polyBits)) & polyTypeMask);
#endif
	}

	// Decodes a poly id.
	inline unsigned int DecodePolyIdPoly(dtPolyRef ref) const
	{
#ifdef DT_POLYREF64
		return (unsigned int)(ref & kPolyRefPolyMask);
#else
		const dtPolyRef polyMask = ((dtPolyRef)1 << m_polyBits) - 1;
		return (unsigned int)(ref & polyMask);
#endif
	}


	/// @}

	/// @{
	/// @name Encoding and Decoding
	/// These functions are generally meant for internal use only.

	/// Derives a standard polygon reference.
	///  @note This function is generally meant for internal use only.
	///  @param[in]	salt	The tile's salt value.
	///  @param[in]	it		The index of the tile.
	///  @param[in]	ip		The index of the polygon within the tile.
	inline dtPolyRef encodePolyId(unsigned int salt, unsigned int it, unsigned int ip) const
	{
#ifdef DT_POLYREF64
		return ((dtPolyRef)salt << (DT_POLY_BITS+DT_TILE_BITS)) | ((dtPolyRef)it << DT_POLY_BITS) | (dtPolyRef)ip;
#else
		return ((dtPolyRef)salt << (m_polyBits+m_tileBits)) | ((dtPolyRef)it << m_polyBits) | (dtPolyRef)ip;
#endif
	}
	
	/// Decodes a standard polygon reference.
	///  @note This function is generally meant for internal use only.
	///  @param[in]	ref   The polygon reference to decode.
	///  @param[out]	salt	The tile's salt value.
	///  @param[out]	it		The index of the tile.
	///  @param[out]	ip		The index of the polygon within the tile.
	///  @see #encodePolyId
	inline void decodePolyId(dtPolyRef ref, unsigned int& salt, unsigned int& it, unsigned int& ip) const
	{
#ifdef DT_POLYREF64
		const dtPolyRef saltMask = ((dtPolyRef)1<<DT_SALT_BITS)-1;
		const dtPolyRef tileMask = ((dtPolyRef)1<<DT_TILE_BITS)-1;
		const dtPolyRef polyMask = ((dtPolyRef)1<<DT_POLY_BITS)-1;
		salt = (unsigned int)((ref >> (DT_POLY_BITS+DT_TILE_BITS)) & saltMask);
		it = (unsigned int)((ref >> DT_POLY_BITS) & tileMask);
		ip = (unsigned int)(ref & polyMask);
#else
		const dtPolyRef saltMask = ((dtPolyRef)1<<m_saltBits)-1;
		const dtPolyRef tileMask = ((dtPolyRef)1<<m_tileBits)-1;
		const dtPolyRef polyMask = ((dtPolyRef)1<<m_polyBits)-1;
		salt = (unsigned int)((ref >> (m_polyBits+m_tileBits)) & saltMask);
		it = (unsigned int)((ref >> m_polyBits) & tileMask);
		ip = (unsigned int)(ref & polyMask);
#endif
	}

	/// Extracts a tile's salt value from the specified polygon reference.
	///  @note This function is generally meant for internal use only.
	///  @param[in]	ref		The polygon reference.
	///  @see #encodePolyId
	inline unsigned int decodePolyIdSalt(dtPolyRef ref) const
	{
#ifdef DT_POLYREF64
		const dtPolyRef saltMask = ((dtPolyRef)1<<DT_SALT_BITS)-1;
		return (unsigned int)((ref >> (DT_POLY_BITS+DT_TILE_BITS)) & saltMask);
#else
		const dtPolyRef saltMask = ((dtPolyRef)1<<m_saltBits)-1;
		return (unsigned int)((ref >> (m_polyBits+m_tileBits)) & saltMask);
#endif
	}
	
	/// Extracts the tile's index from the specified polygon reference.
	///  @note This function is generally meant for internal use only.
	///  @param[in]	ref		The polygon reference.
	///  @see #encodePolyId
	inline unsigned int decodePolyIdTile(dtPolyRef ref) const
	{
#ifdef DT_POLYREF64
		const dtPolyRef tileMask = ((dtPolyRef)1<<DT_TILE_BITS)-1;
		return (unsigned int)((ref >> DT_POLY_BITS) & tileMask);
#else
		const dtPolyRef tileMask = ((dtPolyRef)1<<m_tileBits)-1;
		return (unsigned int)((ref >> m_polyBits) & tileMask);
#endif
	}
	
	/// Extracts the polygon's index (within its tile) from the specified polygon reference.
	///  @note This function is generally meant for internal use only.
	///  @param[in]	ref		The polygon reference.
	///  @see #encodePolyId
	inline unsigned int decodePolyIdPoly(dtPolyRef ref) const
	{
#ifdef DT_POLYREF64
		const dtPolyRef polyMask = ((dtPolyRef)1<<DT_POLY_BITS)-1;
		return (unsigned int)(ref & polyMask);
#else
		const dtPolyRef polyMask = ((dtPolyRef)1<<m_polyBits)-1;
		return (unsigned int)(ref & polyMask);
#endif
	}

	dtPolyRef AddOffMeshConnection(const float* spos, const float* epos, int instanceID, bool twoWay, unsigned char areaType, int agentTypeID);

	dtPolyRef AddOffMeshConnection(const struct OffMeshConnectionParams* params, float connectRadius, float connectHeight);

	dtStatus RemoveOffMeshConnection(const dtPolyRef ref);

	void UnconnectOffMeshConnection(unsigned int index);

	void ConnectOffMeshConnection(unsigned int index, float connectRadius, float connectHeight);

	const dtLink* GetFirstLink(dtPolyRef ref) const;

	inline const dtLink* GetNextLink(const dtLink* link) const
	{
		if (link == NULL) return NULL;
		if (link->next == DT_NULL_LINK) return NULL;
		dtAssert(link->next < m_links.Capacity());
		return &m_links[link->next];
	}

	inline const dtLink* GetLink(unsigned int i) const
	{
		if (i == DT_NULL_LINK) return NULL;
		dtAssert(i < m_links.Capacity());
		return &m_links[i];
	}


	/// @}
	FreeList<dtLink> m_links;
	FreeList<dtOffMeshConnection> m_offMeshConnections;
	unsigned int m_firstOffMeshConnection;

	dtPolyRef FindNearestPoly(int typeID, const Vector3f& center, const Vector3f& extents, Vector3f* nearestPt) const;

	void ClosestPointOnPolyInTileLocal(const dtMeshTile* tile, const dtPoly* poly, const float* pos, float* closest) const;

private:
	// Explicitly disabled copy constructor and copy assignment operator.
	dtNavMesh(const dtNavMesh&);
	dtNavMesh& operator=(const dtNavMesh&);

	/// Returns pointer to tile in the tile array.
	dtMeshTile* getTile(int i);

	/// Returns neighbour tile based on side.
	int getTilesAt(const int x, const int y,
				   dtMeshTile** tiles, const int maxTiles) const;

	/// Returns neighbour tile based on side.
	int getNeighbourTilesAt(const int x, const int y, const int side,
							dtMeshTile** tiles, const int maxTiles) const;
	
	/// Returns all polygons in neighbour tile based on portal defined by the segment.
	int findConnectingPolys(const float* va, const float* vb,
							const dtMeshTile* tile, int side,
							dtPolyRef* con, float* conarea, int maxcon) const;
	
	/// Builds internal polygons links for a tile.
	void connectIntLinks(dtMeshTile* tile);
	/// Builds internal polygons links for a tile.
	//void baseOffMeshLinks(dtMeshTile* tile);


	void ConnectOffMeshConnectionsToTile(dtMeshTile* tile);
	void UnconnectOffMeshConnectionsToTile(dtTileRef ref);

	void RemoveLinkBetween(dtPolyRef from, dtPolyRef to);

	/// Builds external polygon links for a tile.
	void connectExtLinks(dtMeshTile* tile, dtMeshTile* target, int side);

	/// Removes external links at specified side.
	void unconnectLinks(dtMeshTile* tile, dtMeshTile* target);
	
	// Removes all internal and external links from a tile.
	void UnconnectLinks(dtMeshTile* tile);

	// TODO: These methods are duplicates from dtNavMeshQuery, but are needed for off-mesh connection finding.
	
	/// Queries polygons within a tile.
	int queryPolygonsInTile(const dtMeshTile* tile, const float* qmin, const float* qmax,
							dtPolyRef* polyRefs, dtPoly** polys, const int maxPolys, NavMeshProcessCallback* callback) const;
	/// Find nearest polygon within a tile.
	dtPolyRef findNearestPolyInTile(const dtMeshTile* tile, const float* center,
									const float* halfExtents, float* nearestPt) const;

	/// Returns closest point on polygon.
	void closestPointOnPoly(dtPolyRef ref, const float* pos, float* closest, bool* posOverPoly) const;
	
	dtNavMeshParams m_params;			///< Current initialization params. TODO: do not store this info twice.
	float m_orig[3];					///< Origin of the tile (0,0)
	float m_tileWidth, m_tileHeight;	///< Dimensions of each tile.
	float m_cellSize, m_tileSize;
	float m_agentHeight;
	float m_agentRadius;
	float m_agentMaxClimb;

	int m_maxTiles;						///< Max number of tiles.
	int m_tileLutSize;					///< Tile hash lookup size (must be pot).
	int m_tileLutMask;					///< Tile hash lookup mask.

	dtMeshTile** m_posLookup;			///< Tile hash lookup.
	dtMeshTile* m_nextFree;				///< Freelist of tiles.
	dtMeshTile* m_tiles;				///< List of tiles.
	//FreeList<dtMeshTile> m_tiles;		///< List of tiles.

		
#ifndef DT_POLYREF64
	unsigned int m_saltBits;			///< Number of salt bits in the tile ID.
	unsigned int m_tileBits;			///< Number of tile bits in the tile ID.
	unsigned int m_polyBits;			///< Number of poly bits in the tile ID.
	unsigned int m_polyTypeBits;		///< Number of poly type bits in the tile ID.
#endif
};

// 查找Poly的索引下标
static inline unsigned int GetPolyIndex(const dtMeshTile* tile, const dtPoly* poly)
{
 	dtAssert(poly);
 	dtAssert(tile);
 	dtAssert(tile->header);
	const unsigned int ip = (unsigned int)(poly - tile->polys);
 	dtAssert(ip < (unsigned int)tile->header->polyCount);
	return ip;
}

inline Vector3f TileToWorld(const dtMeshTile& tile, const Vector3f& position)
{
#if ENABLE_RUNTIME_NAVMESH_BUILDING
	if (!tile.transformed)
		return position;
	Matrix4x4f mat;
	mat.SetTR(tile.position, tile.rotation);
	return mat.MultiplyPoint3(position);
#else
	//UNUSED(tile);
	return position;
#endif
}

inline void WorldToTile(const dtMeshTile& tile, const float* position, float* OutPosition)
{
#if ENABLE_RUNTIME_NAVMESH_BUILDING
	if (!tile.transformed)
		return position;
	Matrix4x4f mat;
	mat.SetTRInverse(tile.position, tile.rotation);
	return mat.MultiplyPoint3(position);
#else
	//UNUSED(tile);
	dtVcopy(OutPosition, position);

#endif
}

/// Allocates a navigation mesh object using the Detour allocator.
/// @return A navigation mesh that is ready for initialization, or null on failure.
///  @ingroup detour
dtNavMesh* dtAllocNavMesh();

/// Frees the specified navigation mesh object using the Detour allocator.
///  @param[in]	navmesh		A navigation mesh allocated using #dtAllocNavMesh
///  @ingroup detour
void dtFreeNavMesh(dtNavMesh* navmesh);

#endif // DETOURNAVMESH_H

///////////////////////////////////////////////////////////////////////////

// This section contains detailed documentation for members that don't have
// a source file. It reduces clutter in the main section of the header.

/**

@typedef dtPolyRef
@par

Polygon references are subject to the same invalidate/preserve/restore 
rules that apply to #dtTileRef's.  If the #dtTileRef for the polygon's
tile changes, the polygon reference becomes invalid.

Changing a polygon's flags, area id, etc. does not impact its polygon
reference.

@typedef dtTileRef
@par

The following changes will invalidate a tile reference:

- The referenced tile has been removed from the navigation mesh.
- The navigation mesh has been initialized using a different set
  of #dtNavMeshParams.

A tile reference is preserved/restored if the tile is added to a navigation 
mesh initialized with the original #dtNavMeshParams and is added at the
original reference location. (E.g. The lastRef parameter is used with
dtNavMesh::addTile.)

Basically, if the storage structure of a tile changes, its associated
tile reference changes.


@var unsigned short dtPoly::neis[DT_VERTS_PER_POLYGON]
@par

Each entry represents data for the edge starting at the vertex of the same index. 
E.g. The entry at index n represents the edge data for vertex[n] to vertex[n+1].

A value of zero indicates the edge has no polygon connection. (It makes up the 
border of the navigation mesh.)

The information can be extracted as follows: 
@code 
neighborRef = neis[n] & 0xff; // Get the neighbor polygon reference.

if (neis[n] & #DT_EX_LINK)
{
    // The edge is an external (portal) edge.
}
@endcode

@var float dtMeshHeader::bvQuantFactor
@par

This value is used for converting between world and bounding volume coordinates.
For example:
@code
const float cs = 1.0f / tile->header->bvQuantFactor;
const dtBVNode* n = &tile->bvTree[i];
if (n->i >= 0)
{
    // This is a leaf node.
    float worldMinX = tile->header->bmin[0] + n->bmin[0]*cs;
    float worldMinY = tile->header->bmin[0] + n->bmin[1]*cs;
    // Etc...
}
@endcode

@struct dtMeshTile
@par

Tiles generally only exist within the context of a dtNavMesh object.

Some tile content is optional.  For example, a tile may not contain any
off-mesh connections.  In this case the associated pointer will be null.

If a detail mesh exists it will share vertices with the base polygon mesh.  
Only the vertices unique to the detail mesh will be stored in #detailVerts.

@warning Tiles returned by a dtNavMesh object are not guarenteed to be populated.
For example: The tile at a location might not have been loaded yet, or may have been removed.
In this case, pointers will be null.  So if in doubt, check the polygon count in the 
tile's header to determine if a tile has polygons defined.

@var float dtOffMeshConnection::pos[6]
@par

For a properly built navigation mesh, vertex A will always be within the bounds of the mesh. 
Vertex B is not required to be within the bounds of the mesh.

*/
