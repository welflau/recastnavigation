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

#include <float.h>
#include <string.h>
#include <stdio.h>
#include "DetourNavMesh.h"
#include "DetourNode.h"
#include "DetourCommon.h"
#include "DetourMath.h"
#include "DetourAlloc.h"
#include "DetourAssert.h"
#include <new>
#include "AABB.h"

static MinMaxAABB GetWorldTileBounds(const dtMeshTile& tile)
{
//	Assert(tile.header);

	MinMaxAABB tileBounds(Vector3f(tile.header->bmin), Vector3f(tile.header->bmax));
// 	if (tile.transformed)
// 	{
// 		Matrix4x4f mat;
// 		mat.SetTR(tile.position, tile.rotation);
// 		TransformAABBSlow(tileBounds, mat, tileBounds);
// 	}
	return tileBounds;
}

bool IntersectAABBAABB(const MinMaxAABB& A, const MinMaxAABB& B)
{
	bool overlap = true;

	if (A.m_Min.x > B.m_Max.x || A.m_Max.x < B.m_Min.x) overlap = false;
	if (A.m_Min.y > B.m_Max.y || A.m_Max.y < B.m_Min.y) overlap = false;
	if (A.m_Min.z > B.m_Max.z || A.m_Max.x < B.m_Min.z) overlap = false;

	return overlap;
}

inline bool overlapSlabs(const float* amin, const float* amax,
						 const float* bmin, const float* bmax,
						 const float px, const float py)
{
	// Check for horizontal overlap.
	// The segment is shrunken a little so that slabs which touch
	// at end points are not connected.
	const float minx = dtMax(amin[0]+px,bmin[0]+px);
	const float maxx = dtMin(amax[0]-px,bmax[0]-px);
	if (minx > maxx)
		return false;
	
	// Check vertical overlap.
	const float ad = (amax[1]-amin[1]) / (amax[0]-amin[0]);
	const float ak = amin[1] - ad*amin[0];
	const float bd = (bmax[1]-bmin[1]) / (bmax[0]-bmin[0]);
	const float bk = bmin[1] - bd*bmin[0];
	const float aminy = ad*minx + ak;
	const float amaxy = ad*maxx + ak;
	const float bminy = bd*minx + bk;
	const float bmaxy = bd*maxx + bk;
	const float dmin = bminy - aminy;
	const float dmax = bmaxy - amaxy;
		
	// Crossing segments always overlap.
	if (dmin*dmax < 0)
		return true;
		
	// Check for overlap at endpoints.
	const float thr = dtSqr(py*2);
	if (dmin*dmin <= thr || dmax*dmax <= thr)
		return true;
		
	return false;
}

static float getSlabCoord(const float* va, const int side)
{
	if (side == 0 || side == 4)
		return va[0];
	else if (side == 2 || side == 6)
		return va[2];
	return 0;
}

static void calcSlabEndPoints(const float* va, const float* vb, float* bmin, float* bmax, const int side)
{
	if (side == 0 || side == 4)
	{
		if (va[2] < vb[2])
		{
			bmin[0] = va[2];
			bmin[1] = va[1];
			bmax[0] = vb[2];
			bmax[1] = vb[1];
		}
		else
		{
			bmin[0] = vb[2];
			bmin[1] = vb[1];
			bmax[0] = va[2];
			bmax[1] = va[1];
		}
	}
	else if (side == 2 || side == 6)
	{
		if (va[0] < vb[0])
		{
			bmin[0] = va[0];
			bmin[1] = va[1];
			bmax[0] = vb[0];
			bmax[1] = vb[1];
		}
		else
		{
			bmin[0] = vb[0];
			bmin[1] = vb[1];
			bmax[0] = va[0];
			bmax[1] = va[1];
		}
	}
}

inline int computeTileHash(int x, int y, const int mask)
{
	const unsigned int h1 = 0x8da6b343; // Large multiplicative constants;
	const unsigned int h2 = 0xd8163841; // here arbitrarily chosen primes
	unsigned int n = h1 * x + h2 * y;
	return (int)(n & mask);
}

dtNavMesh* dtAllocNavMesh()
{
	void* mem = dtAlloc(sizeof(dtNavMesh), DT_ALLOC_PERM);
	if (!mem) return 0;
	return new(mem) dtNavMesh;
}

/// @par
///
/// This function will only free the memory for tiles with the #DT_TILE_FREE_DATA
/// flag set.
void dtFreeNavMesh(dtNavMesh* navmesh)
{
	if (!navmesh) return;
	navmesh->~dtNavMesh();
	dtFree(navmesh);
}

//////////////////////////////////////////////////////////////////////////////////////////

/**
@class dtNavMesh

The navigation mesh consists of one or more tiles defining three primary types of structural data:

A polygon mesh which defines most of the navigation graph. (See rcPolyMesh for its structure.)
A detail mesh used for determining surface height on the polygon mesh. (See rcPolyMeshDetail for its structure.)
Off-mesh connections, which define custom point-to-point edges within the navigation graph.

The general build process is as follows:

-# Create rcPolyMesh and rcPolyMeshDetail data using the Recast build pipeline.
-# Optionally, create off-mesh connection data.
-# Combine the source data into a dtNavMeshCreateParams structure.
-# Create a tile data array using dtCreateNavMeshData().
-# Allocate at dtNavMesh object and initialize it. (For single tile navigation meshes,
   the tile data is loaded during this step.)
-# For multi-tile navigation meshes, load the tile data using dtNavMesh::addTile().

Notes:

- This class is usually used in conjunction with the dtNavMeshQuery class for pathfinding.
- Technically, all navigation meshes are tiled. A 'solo' mesh is simply a navigation mesh initialized 
  to have only a single tile.
- This class does not implement any asynchronous methods. So the ::dtStatus result of all methods will 
  always contain either a success or failure flag.

@see dtNavMeshQuery, dtCreateNavMeshData, dtNavMeshCreateParams, #dtAllocNavMesh, #dtFreeNavMesh
*/

dtNavMesh::dtNavMesh() :
	m_tileWidth(0),
	m_tileHeight(0),
	m_maxTiles(0),
	m_tileLutSize(0),
	m_tileLutMask(0),
	m_posLookup(0),
	m_nextFree(0),
	m_tiles(0),
	m_firstOffMeshConnection(DT_NULL_LINK)
{
#ifndef DT_POLYREF64
	m_saltBits = 0;
	m_tileBits = 0;
	m_polyBits = 0;
	m_polyTypeBits = 0;
#endif
	memset(&m_params, 0, sizeof(dtNavMeshParams));
	m_orig[0] = 0;
	m_orig[1] = 0;
	m_orig[2] = 0;
}

dtNavMesh::~dtNavMesh()
{
	dtFree(m_posLookup);

	//for (int i = 0; i < m_tiles.Capacity(); ++i)//LWF_TMP
	for (int i = 0; i < m_maxTiles; ++i)
	{
		dtMeshTile* tile = &m_tiles[i];
		if (!tile->header)
			continue;
		if (tile->flags &  DT_TILE_FREE_DATA)
		{
			dtFree(tile->data);
			tile->data = NULL;
			tile->dataSize = 0;
		}
		dtFree(tile->polyLinks);
		tile->polyLinks = NULL;
	}
}
		
dtStatus dtNavMesh::init(const dtNavMeshParams* params)
{
	memcpy(&m_params, params, sizeof(dtNavMeshParams));
	dtVcopy(m_orig, params->orig);
	m_tileWidth = params->tileWidth;
	m_tileHeight = params->tileHeight;
	m_cellSize = params->cellSize;
	m_tileSize = params->tileSize;
	m_agentHeight = params->agentHeight;
	m_agentRadius = params->agentRadius;
	m_agentMaxClimb = params->agentMaxClimb;

	// Init tiles
	m_maxTiles = params->maxTiles;
	m_tileLutSize = dtNextPow2(params->maxTiles/4);
	if (!m_tileLutSize) m_tileLutSize = 1;
	m_tileLutMask = m_tileLutSize-1;
	
	m_tiles = (dtMeshTile*)dtAlloc(sizeof(dtMeshTile)*m_maxTiles, DT_ALLOC_PERM);
	if (!m_tiles)
		return DT_FAILURE | DT_OUT_OF_MEMORY;
	m_posLookup = (dtMeshTile**)dtAlloc(sizeof(dtMeshTile*)*m_tileLutSize, DT_ALLOC_PERM);
	if (!m_posLookup)
		return DT_FAILURE | DT_OUT_OF_MEMORY;
	memset(m_tiles, 0, sizeof(dtMeshTile)*m_maxTiles);
	memset(m_posLookup, 0, sizeof(dtMeshTile*)*m_tileLutSize);
	m_nextFree = 0;
	for (int i = m_maxTiles-1; i >= 0; --i)
	{
		m_tiles[i].salt = 1;
		m_tiles[i].next = m_nextFree;
		m_nextFree = &m_tiles[i];
	}
	
	// Init ID generator values.
#ifndef DT_POLYREF64
	m_tileBits = dtIlog2(dtNextPow2((unsigned int)params->maxTiles));
	m_polyBits = dtIlog2(dtNextPow2((unsigned int)params->maxPolys));
	m_polyTypeBits = 2;
	// Only allow 31 salt bits, since the salt mask is calculated using 32bit uint and it will overflow.
	m_saltBits = dtMin((unsigned int)31, 32 - m_tileBits - m_polyBits);

	if (m_saltBits < 10)
		return DT_FAILURE | DT_INVALID_PARAM;
#endif
	
	return DT_SUCCESS;
}

dtStatus dtNavMesh::init(unsigned char* data, const int dataSize, const int flags)
{
	// Make sure the data is in right format.
	dtMeshHeader* header = (dtMeshHeader*)data;
	if (header->magic != DT_NAVMESH_MAGIC)
		return DT_FAILURE | DT_WRONG_MAGIC;
	if (header->version != DT_NAVMESH_VERSION)
		return DT_FAILURE | DT_WRONG_VERSION;

	dtNavMeshParams params;
	dtVcopy(params.orig, header->bmin);
	params.tileWidth = header->bmax[0] - header->bmin[0];
	params.tileHeight = header->bmax[2] - header->bmin[2];
	params.maxTiles = 1;
	params.maxPolys = header->polyCount;
	
	dtStatus status = init(&params);
	if (dtStatusFailed(status))
		return status;

	return addTile(data, dataSize, flags, 0, 0);
}

/// @par
///
/// @note The parameters are created automatically when the single tile
/// initialization is performed.
const dtNavMeshParams* dtNavMesh::getParams() const
{
	return &m_params;
}

//////////////////////////////////////////////////////////////////////////////////////////
int dtNavMesh::findConnectingPolys(const float* va, const float* vb,
								   const dtMeshTile* tile, int side,
								   dtPolyRef* con, float* conarea, int maxcon) const
{
	if (!tile) return 0;
	
	float amin[2], amax[2];
	calcSlabEndPoints(va, vb, amin, amax, side);
	const float apos = getSlabCoord(va, side);

	// Remove links pointing to 'side' and compact the links array. 
	float bmin[2], bmax[2];
	unsigned short m = DT_EXT_LINK | (unsigned short)side;
	int n = 0;
	
	dtPolyRef base = getPolyRefBase(tile);
	
	for (int i = 0; i < tile->header->polyCount; ++i)
	{
		dtPoly* poly = &tile->polys[i];
		const int nv = poly->vertCount;
		for (int j = 0; j < nv; ++j)
		{
			// Skip edges which do not point to the right side.
			if (poly->neis[j] != m) continue;
			
			const float* vc = &tile->verts[poly->verts[j]*3];
			const float* vd = &tile->verts[poly->verts[(j+1) % nv]*3];
			const float bpos = getSlabCoord(vc, side);
			
			// Segments are not close enough.
			if (dtAbs(apos-bpos) > 0.01f)
				continue;
			
			// Check if the segments touch.
			calcSlabEndPoints(vc,vd, bmin,bmax, side);
			
			if (!overlapSlabs(amin,amax, bmin,bmax, 0.01f, tile->header->walkableClimb)) continue;
			
			// Add return value.
			if (n < maxcon)
			{
				conarea[n*2+0] = dtMax(amin[0], bmin[0]);
				conarea[n*2+1] = dtMin(amax[0], bmax[0]);
				//con[n] = base | (dtPolyRef)i;
				con[n] = base | EncodeBasePolyId(DT_POLYTYPE_GROUND, i);
				n++;
			}
			break;
		}
	}
	return n;
}

// 去掉两个tile之间的Link
void dtNavMesh::unconnectLinks(dtMeshTile* tile, dtMeshTile* target)
{
	if (!tile || !target) return;

	const unsigned int targetNum = decodePolyIdTile(getTileRef(target)); // 目标Tile的ID
	dtPolyRef base = getPolyRefBase(tile);

	for (int i = 0; i < tile->header->polyCount; ++i)
	{
		dtPoly* poly = &tile->polys[i];
		//unsigned int j = poly->firstLink;
		unsigned int j = tile->polyLinks[i];
		unsigned int pj = DT_NULL_LINK;
		dtPolyRef polyRef = base | EncodeBasePolyId(DT_POLYTYPE_GROUND, i);

		while (j != DT_NULL_LINK)
		{
			const dtLink* link = GetLink(j);

			//if (decodePolyIdTile(tile->links[j].ref) == targetNum)
			if (decodePolyIdTile(link->ref) == targetNum)
			{
				// Remove link to this polygon
				// 将从Link连接的Poly开始，到目标Poly之间的所有Link都删掉
				RemoveLinkBetween(m_links[j].ref, polyRef);

				// Remove link.
 				unsigned int nj = link->next;

				const dtLink* Nlink = GetLink(nj);
				if (Nlink)
				{
					dtAssert(Nlink->next != j);
				}

				m_links.Release(j);
				j = nj;
			}
			else
			{
				// Advance
				pj = j;
				j = link->next;
			}
		}
		tile->polyLinks[i] = DT_NULL_LINK; // 需要清空标志
	}
}

// Removes all internal and external links from a tile.
void dtNavMesh::UnconnectLinks(dtMeshTile* tile)
{
	dtAssert(tile);
	dtAssert(tile->header);
	dtAssert(tile->polyLinks);

	dtPolyRef base = getPolyRefBase(tile);

	// 遍历Tile中的所有Poly
	for (int i = 0; i < tile->header->polyCount; ++i)
	{
		dtPolyRef polyRef = base | EncodeBasePolyId(DT_POLYTYPE_GROUND, i);

		unsigned int j = tile->polyLinks[i];
		while (j != DT_NULL_LINK)
		{
			unsigned int next = m_links[j].next;

			// Remove link to this polygon
			RemoveLinkBetween(m_links[j].ref, polyRef);

			m_links.Release(j);
			j = next;
		}
		tile->polyLinks[i] = DT_NULL_LINK;
	}
}

void dtNavMesh::connectExtLinks(dtMeshTile* tile, dtMeshTile* target, int side)
{
	if (!tile) return;
	
	// Connect border links.
	for (int i = 0; i < tile->header->polyCount; ++i)
	{
		dtPoly* poly = &tile->polys[i];

		// Create new links.
		const int nv = poly->vertCount;
		for (int j = 0; j < nv; ++j)
		{
			// Skip non-portal edges.
			if ((poly->neis[j] & DT_EXT_LINK) == 0)
				continue;
			
			const int dir = (int)(poly->neis[j] & 0xff);
			if (side != -1 && dir != side)
				continue;
			
			// Create new links
			const float* va = &tile->verts[poly->verts[j]*3];
			const float* vb = &tile->verts[poly->verts[(j+1) % nv]*3];
			dtPolyRef nei[4];
			float neia[4*2];
			int nnei = findConnectingPolys(va,vb, target, dtOppositeTile(dir), nei,neia,4);
			for (int k = 0; k < nnei; ++k)
			{
				//unsigned int idx = allocLink(tile);
				unsigned int idx = m_links.Alloc();
				if (idx != DT_NULL_LINK)
				{
					//dtLink* link = &tile->links[idx];
					dtLink* link = &m_links[idx];
					link->ref = nei[k];
					link->edge = (unsigned char)j;
					link->side = (unsigned char)dir;
					
					// 构建一个新的Link节点，插入到Link链表头
					dtAssert(idx != tile->polyLinks[i]);
					link->next = tile->polyLinks[i];
					tile->polyLinks[i] = idx;

					// Compress portal limits to a byte value.
					if (dir == 0 || dir == 4)
					{
						float tmin = (neia[k*2+0]-va[2]) / (vb[2]-va[2]);
						float tmax = (neia[k*2+1]-va[2]) / (vb[2]-va[2]);
						if (tmin > tmax)
							dtSwap(tmin,tmax);
						link->bmin = (unsigned char)(dtClamp(tmin, 0.0f, 1.0f)*255.0f);
						link->bmax = (unsigned char)(dtClamp(tmax, 0.0f, 1.0f)*255.0f);
					}
					else if (dir == 2 || dir == 6)
					{
						float tmin = (neia[k*2+0]-va[0]) / (vb[0]-va[0]);
						float tmax = (neia[k*2+1]-va[0]) / (vb[0]-va[0]);
						if (tmin > tmax)
							dtSwap(tmin,tmax);
						link->bmin = (unsigned char)(dtClamp(tmin, 0.0f, 1.0f)*255.0f);
						link->bmax = (unsigned char)(dtClamp(tmax, 0.0f, 1.0f)*255.0f);
					}
				}
			}
		}
	}
}

const dtLink* dtNavMesh::GetFirstLink(dtPolyRef ref) const
{
	if (DecodePolyIdType(ref) == DT_POLYTYPE_OFFMESH_CONNECTION) // OMC
	{
		const dtOffMeshConnection* con = GetOffMeshConnection(ref);
		if (con == NULL)
			return NULL;
		if (con->firstLink == DT_NULL_LINK)
			return NULL;
		return &m_links[con->firstLink];
	}
	else // 普通Poly
	{
		const dtMeshTile* tile = NULL;
		const dtPoly* poly = NULL;
		if (dtStatusFailed(getTileAndPolyByRef(ref, &tile, &poly)))
			return NULL;

		const unsigned int ip = GetPolyIndex(tile, poly);
		const unsigned int firstLink = tile->polyLinks[ip];
		if (firstLink == DT_NULL_LINK)
			return NULL;
		return &m_links[firstLink];
	}
}

void dtNavMesh::connectIntLinks(dtMeshTile* tile)
{
	if (!tile) return;

	const int polyCount = tile->header->polyCount;

	// Allocate and nullify per poly links
	if (tile->polyLinks) return;

	// 申请一块PolyLinks数据内存
	tile->polyLinks = (unsigned int*)dtAlloc(sizeof(unsigned int)*polyCount, DT_ALLOC_PERM);

	for (int i = 0; i < polyCount; ++i)
		tile->polyLinks[i] = DT_NULL_LINK; // 默认是没有Link的

	// 0号Poly的ID
	dtPolyRef base = getPolyRefBase(tile);

	// 遍历所有Poly
	for (int i = 0; i < polyCount; ++i)
	{
		dtPoly* poly = &tile->polys[i];

		// Build edge links backwards so that the links will be
		// in the linked list from lowest index to highest.
		// 每个相邻边都有Link
		for (int j = poly->vertCount - 1; j >= 0; --j)
		{
			// Skip hard and non-internal edges.
			if (poly->neis[j] == 0 || (poly->neis[j] & DT_EXT_LINK))
				continue;

			unsigned int idx = m_links.Alloc();
			if (idx != DT_NULL_LINK)
			{
				dtLink* link = &m_links[idx];
				link->ref = base | EncodeBasePolyId(DT_POLYTYPE_GROUND, poly->neis[j] - 1); // 计算一遍PolyID Tile中的所有Poly都是Ground类型的
				link->edge = (unsigned char)j;
				link->side = 0xff;
				link->bmin = link->bmax = 0;
				// Add to linked list.
				//Assert(idx != tile->polyLinks[i]);
				link->next = tile->polyLinks[i];
				tile->polyLinks[i] = idx;
			}
		}
	}
}


void dtNavMesh::ConnectOffMeshConnectionsToTile(dtMeshTile* tile)
{
	const MinMaxAABB tileBounds = GetWorldTileBounds(*tile);
	const int agentTypeID = tile->header->userId;

	for (unsigned int i = m_firstOffMeshConnection; i != DT_NULL_LINK; i = m_offMeshConnections[i].next)
	{
		dtOffMeshConnection& con = m_offMeshConnections[i];
		if (con.agentTypeID != agentTypeID)
			continue;
		// TODO: it might be possible that adding a new tile allows the end point to be mapped closer.
		//      if (con.endPoints[0].tileRef && con.endPoints[1].tileRef)
		//          continue;
		if (IntersectAABBAABB(con.bounds, tileBounds))
			ConnectOffMeshConnection(i, m_agentRadius, m_agentMaxClimb);
	}
}

// 两个Poly之间的Link都去掉
void dtNavMesh::RemoveLinkBetween(dtPolyRef from, dtPolyRef to)
{
	if (DecodePolyIdType(from) == DT_POLYTYPE_OFFMESH_CONNECTION) // 起点Poly是 OMC类型的Poly
	{
		dtOffMeshConnection* con = GetOffMeshConnectionUnsafe(from);
		if (con != NULL)
		{
			// Remove link from connect polygon.
			unsigned int k = con->firstLink; // OMC的第一个Link，
			unsigned int pk = DT_NULL_LINK;
			while (k != DT_NULL_LINK) // 遍历这个Link链条，找到目标Poly节点
			{
				if (m_links[k].ref == to)
				{
					unsigned int nk = m_links[k].next;
					if (pk == DT_NULL_LINK)
					{
						con->firstLink = nk;
					}
					else
					{
						dtAssert(pk != nk);
						m_links[pk].next = nk;
					}
					con->endPoints[m_links[k].edge].tileRef = 0;
					m_links.Release(k);
					break;
				}
				pk = k;
				k = m_links[k].next;
			}
		}
	}
	else // 起点Poly是 普通Poly类型
	{
		const dtMeshTile* neiTile = NULL;
		const dtPoly* neiPoly = NULL;
		if (dtStatusSucceed(getTileAndPolyByRef(from, &neiTile, &neiPoly)))
		{
			// 删除所有与这个Poly相连的Link
			// Remove link from connect polygon.
			const unsigned int ip = GetPolyIndex(neiTile, neiPoly);
			unsigned int k = neiTile->polyLinks[ip];
			unsigned int pk = DT_NULL_LINK;
			while (k != DT_NULL_LINK)
			{
				if (m_links[k].ref == to) // 直到终点Poly
				{
					unsigned int nk = m_links[k].next;
					if (pk == DT_NULL_LINK)
					{
						neiTile->polyLinks[ip] = nk;
					}
					else
					{
						dtAssert(pk != nk);
						m_links[pk].next = nk;
					}
					m_links.Release(k);
					break;
				}

				const dtLink* Nlink = GetLink(m_links[k].next);
				if (Nlink)
				{
					dtAssert(Nlink->next != k);
				}

				pk = k;
				k = m_links[k].next;
			}
			neiTile->polyLinks[ip] = DT_NULL_LINK;
		}
	}
}

// 解除相连
void dtNavMesh::UnconnectOffMeshConnectionsToTile(dtTileRef ref)
{
	// 由TileID获取Tile索引
 	const unsigned int tileIndex = DecodePolyIdTile((dtPolyRef)ref); // TileID 直接转Poly？

	// 遍历OffMesh列表
	for (unsigned int i = m_firstOffMeshConnection; i != DT_NULL_LINK; i = m_offMeshConnections[i].next)
	{
		dtOffMeshConnection& con = m_offMeshConnections[i];
		const dtPolyRef ref = EncodeLinkId(con.salt, i);// 编码出一个LinkID

		// OMC的两个连接点的Link
		for (int j = 0; j < 2; j++)
		{
			if (con.endPoints[j].tileRef == 0)
				continue;

			// Remove links associated with the tile.
			unsigned int k = con.firstLink;
			while (k != DT_NULL_LINK)
			{
				unsigned int next = m_links[k].next;
				unsigned int targetTileIndex = DecodePolyIdTile((dtPolyRef)m_links[k].ref);// 获取
				if (targetTileIndex == tileIndex) // 找出当前Tile中的连接点，都要去掉
				{
					RemoveLinkBetween(ref, m_links[k].ref);
					RemoveLinkBetween(m_links[k].ref, ref);
				}
				k = next;
			}
		}
	}

}

void dtNavMesh::ClosestPointOnPolyInTileLocal(const dtMeshTile* tile, const dtPoly* poly,
	const float* pos, float* closest) const
{
	// 获取Poly的在Tile中索引下标
	const unsigned int ip = GetPolyIndex(tile, poly);
	const dtPolyDetail* pd = &tile->detailMeshes[ip];

	float closestDistSqr = FLT_MAX;

	// 遍历detailMesh的所有三角面片，找出最近的面片中的点
	for (int j = 0; j < pd->triCount; ++j)
	{
		const unsigned char* t = &tile->detailTris[(pd->triBase + j) * 4];
		float* v[3]; // DetailMesh顶点列表 三角形的3个顶点 

		// 遍历3个分量 取出顶点数据
		for (int k = 0; k < 3; ++k)
		{
			if (t[k] < poly->vertCount) // 下标在低模的顶点范围内
				v[k] = &tile->verts[poly->verts[t[k]]];
			else
				v[k] = &tile->detailVerts[pd->vertBase + (t[k] - poly->vertCount)];
		}

		float pt[3];
		ClosestPtPointTriangle(pt,pos, v[0], v[1], v[2]); // 找到三角形中最接近的点

		const float d = dtVdist(pos, pt);
		if (d < closestDistSqr)
		{
			dtVcopy(closest, pt);
			closestDistSqr = d;
		}
	}
}

void dtNavMesh::closestPointOnPoly(dtPolyRef ref, const float* pos, float* closest, bool* posOverPoly) const
{
	const dtMeshTile* tile = 0;
	const dtPoly* poly = 0;
	getTileAndPolyByRefUnsafe(ref, &tile, &poly);
	
	// Off-mesh connections don't have detail polygons.
	if (DecodePolyIdType(ref) != DT_POLYTYPE_OFFMESH_CONNECTION)
	{
		const float* v0 = &tile->verts[poly->verts[0]*3];
		const float* v1 = &tile->verts[poly->verts[1]*3];
		const float d0 = dtVdist(pos, v0);
		const float d1 = dtVdist(pos, v1);
		const float u = d0 / (d0+d1);
		dtVlerp(closest, v0, v1, u);
		if (posOverPoly)
			*posOverPoly = false;
		return;
	}
	
	const unsigned int ip = (unsigned int)(poly - tile->polys);
	const dtPolyDetail* pd = &tile->detailMeshes[ip];
	
	// Clamp point to be inside the polygon.
	float verts[DT_VERTS_PER_POLYGON*3];	
	float edged[DT_VERTS_PER_POLYGON];
	float edget[DT_VERTS_PER_POLYGON];
	const int nv = poly->vertCount;
	for (int i = 0; i < nv; ++i)
		dtVcopy(&verts[i*3], &tile->verts[poly->verts[i]*3]);
	
	dtVcopy(closest, pos);
	if (!dtDistancePtPolyEdgesSqr(pos, verts, nv, edged, edget))
	{
		// Point is outside the polygon, dtClamp to nearest edge.
		float dmin = edged[0];
		int imin = 0;
		for (int i = 1; i < nv; ++i)
		{
			if (edged[i] < dmin)
			{
				dmin = edged[i];
				imin = i;
			}
		}
		const float* va = &verts[imin*3];
		const float* vb = &verts[((imin+1)%nv)*3];
		dtVlerp(closest, va, vb, edget[imin]);
		
		if (posOverPoly)
			*posOverPoly = false;
	}
	else
	{
		if (posOverPoly)
			*posOverPoly = true;
	}
	
	// Find height at the location.
	for (int j = 0; j < pd->triCount; ++j)
	{
		const unsigned char* t = &tile->detailTris[(pd->triBase+j)*4];
		const float* v[3];
		for (int k = 0; k < 3; ++k)
		{
			if (t[k] < poly->vertCount)
				v[k] = &tile->verts[poly->verts[t[k]]*3];
			else
				v[k] = &tile->detailVerts[(pd->vertBase+(t[k]-poly->vertCount))*3];
		}
		float h;
		if (dtClosestHeightPointTriangle(closest, v[0], v[1], v[2], h))
		{
			closest[1] = h;
			break;
		}
	}
}



void dtNavMesh::QueryPolygonsInTile(const dtMeshTile* tile, const Vector3f& center, const Vector3f& extents, NavMeshProcessCallback* callback) const
{
	Vector3f minPos = center - extents;
	Vector3f maxPos = center + extents;

	// Get nearby polygons from proximity grid.
	dtPolyRef polyRefs[128];
	dtPoly* polys[128];
	int n = queryPolygonsInTile(tile, minPos.GetPtr(), maxPos.GetPtr(), polyRefs, polys, 128, callback);
}

// 根据包围盒找到相应的Poly
void dtNavMesh::QueryPolygons(int typeID, const Vector3f& center, const Vector3f& extents,
	NavMeshProcessCallback* callback) const
{
	// 包围盒
	float bmin[3], bmax[3];
	dtVsub(bmin, center.GetPtr(), extents.GetPtr());
	dtVadd(bmax, center.GetPtr(), extents.GetPtr());

	// Find tiles the query touches.
	int minx, miny, maxx, maxy;
	calcTileLoc(bmin, &minx, &miny);
	calcTileLoc(bmax, &maxx, &maxy);

	// 获取当前坐标下的所有Layer的Tile
	static const int MAX_NEIS = 32;
	dtMeshTile* neis[MAX_NEIS];

	for (int y = miny; y <= maxy; ++y)
	{
		for (int x = minx; x <= maxx; ++x)
		{

			//int nneis;

			// Connect with layers in current tile.
			//nneis = getTilesAt(i, j, neis, MAX_NEIS);
			const int nneis = getTilesAt(x, y, neis, MAX_NEIS);
			for (int n = 0; n < nneis; ++n)
			{
				QueryPolygonsInTile(neis[n], center, extents, callback);
			}
		}
	}

}

// 找到
dtPolyRef dtNavMesh::FindNearestPoly(int typeID, const Vector3f& center, const Vector3f& extents, Vector3f* nearestPt) const
{
	struct NearestQuery : public NavMeshProcessCallback
	{
		const dtNavMesh* m_NavMesh;
		dtPolyRef m_PolyRef;
		float m_DistanceSqr;
		const Vector3f m_Center;
		Vector3f m_Point;

		inline NearestQuery(const dtNavMesh* navmesh, const Vector3f& center)
			: m_NavMesh(navmesh), m_PolyRef(0), m_DistanceSqr(FLT_MAX)
			, m_Center(center), m_Point(0.0f, 0.0f, 0.0f)
		{

		}

		virtual void ProcessPolygons(const dtMeshTile* tile, const dtPolyRef* polyRefs, dtPoly** polys, const int itemCount)
		{
			float localPosition[3];
			//const Vector3f localPosition = WorldToTile(*tile, m_Center);
			WorldToTile(*tile, m_Center.GetPtr(), localPosition);
			// Find nearest polygon amongst the nearby polygons.
			for (int i = 0; i < itemCount; ++i)
			{
				dtPolyRef ref = polyRefs[i];
				const dtPoly* poly = polys[i];
				//Vector3f closestPtPoly;
				float closestPtPoly[3];
				m_NavMesh->ClosestPointOnPolyInTileLocal(tile, poly, localPosition, closestPtPoly); // 找到Poly中最近的点
				const float d = dtVdist(localPosition, closestPtPoly);
				if (d < m_DistanceSqr)
				{
					m_Point = Vector3f(closestPtPoly);
					m_DistanceSqr = d;
					m_PolyRef = ref;
				}
			}
		}
	};

	// Get nearby polygons from proximity grid.
	NearestQuery nearest(this, center);
	QueryPolygons(typeID, center, extents, &nearest);

	if (nearest.m_PolyRef == 0)
		return 0;

	if (nearestPt)
	{
		// 根据Poly找到Tile
		const dtMeshTile* tile = getTileByRef(nearest.m_PolyRef);
		if (tile)
			*nearestPt = TileToWorld(*tile, nearest.m_Point);
	}

	return nearest.m_PolyRef;
}

dtPolyRef dtNavMesh::findNearestPolyInTile(const dtMeshTile* tile,
										   const float* center, const float* halfExtents,
										   float* nearestPt) const
{
	float bmin[3], bmax[3];
	dtVsub(bmin, center, halfExtents);
	dtVadd(bmax, center, halfExtents);
	
	// Get nearby polygons from proximity grid.
	dtPolyRef polyRefs[128];
	dtPoly* polys[128];
	int polyCount = queryPolygonsInTile(tile, bmin, bmax, polyRefs, polys, 128,NULL);
	
	// Find nearest polygon amongst the nearby polygons.
	dtPolyRef nearest = 0;
	float nearestDistanceSqr = FLT_MAX;
	for (int i = 0; i < polyCount; ++i)
	{
		dtPolyRef ref = polyRefs[i];
		float closestPtPoly[3];
		float diff[3];
		bool posOverPoly = false;
		float d;
		closestPointOnPoly(ref, center, closestPtPoly, &posOverPoly);

		// If a point is directly over a polygon and closer than
		// climb height, favor that instead of straight line nearest point.
		dtVsub(diff, center, closestPtPoly);
		if (posOverPoly)
		{
			d = dtAbs(diff[1]) - tile->header->walkableClimb;
			d = d > 0 ? d*d : 0;			
		}
		else
		{
			d = dtVlenSqr(diff);
		}
		
		if (d < nearestDistanceSqr)
		{
			dtVcopy(nearestPt, closestPtPoly);
			nearestDistanceSqr = d;
			nearest = ref;
		}
	}
	
	return nearest;
}

int dtNavMesh::queryPolygonsInTile(const dtMeshTile* tile, const float* qmin, const float* qmax,
	dtPolyRef* polyRefs, dtPoly** polys, const int maxPolys, NavMeshProcessCallback* callback) const
{
	if (tile->bvTree)
	{
		const dtBVNode* node = &tile->bvTree[0];
		const dtBVNode* end = &tile->bvTree[tile->header->bvNodeCount];
		const float* tbmin = tile->header->bmin;
		const float* tbmax = tile->header->bmax;
		const float qfac = tile->header->bvQuantFactor;
		
		// Calculate quantized box
		unsigned short bmin[3], bmax[3];
		// dtClamp query box to world box.
		float minx = dtClamp(qmin[0], tbmin[0], tbmax[0]) - tbmin[0];
		float miny = dtClamp(qmin[1], tbmin[1], tbmax[1]) - tbmin[1];
		float minz = dtClamp(qmin[2], tbmin[2], tbmax[2]) - tbmin[2];
		float maxx = dtClamp(qmax[0], tbmin[0], tbmax[0]) - tbmin[0];
		float maxy = dtClamp(qmax[1], tbmin[1], tbmax[1]) - tbmin[1];
		float maxz = dtClamp(qmax[2], tbmin[2], tbmax[2]) - tbmin[2];
		// Quantize
		bmin[0] = (unsigned short)(qfac * minx) & 0xfffe;
		bmin[1] = (unsigned short)(qfac * miny) & 0xfffe;
		bmin[2] = (unsigned short)(qfac * minz) & 0xfffe;
		bmax[0] = (unsigned short)(qfac * maxx + 1) | 1;
		bmax[1] = (unsigned short)(qfac * maxy + 1) | 1;
		bmax[2] = (unsigned short)(qfac * maxz + 1) | 1;
		
		// Traverse tree
		dtPolyRef base = getPolyRefBase(tile);
		int n = 0;
		while (node < end)
		{
			const bool overlap = dtOverlapQuantBounds(bmin, bmax, node->bmin, node->bmax);
			const bool isLeafNode = node->i >= 0;
			
			if (isLeafNode && overlap)
			{
				if (n < maxPolys)
				{ 
					polyRefs[n] = base | EncodeBasePolyId(DT_POLYTYPE_GROUND, node->i);
					polys[n] = &tile->polys[node->i];
					n++;
				}
			}
			
			if (overlap || isLeafNode)
				node++;
			else
			{
				const int escapeIndex = -node->i;
				node += escapeIndex;
			}
		}
		if (n > 0 && callback)
			callback->ProcessPolygons(tile, polyRefs, polys, n);

		return n;
	}
	else
	{
		float bmin[3], bmax[3];
		int n = 0;
		dtPolyRef base = getPolyRefBase(tile);
		for (int i = 0; i < tile->header->polyCount; ++i)
		{
			dtPoly* p = &tile->polys[i];
			// Calc polygon bounds.
			const float* v = &tile->verts[p->verts[0]*3];
			dtVcopy(bmin, v);
			dtVcopy(bmax, v);
			for (int j = 1; j < p->vertCount; ++j)
			{
				v = &tile->verts[p->verts[j]*3];
				dtVmin(bmin, v);
				dtVmax(bmax, v);
			}
			if (dtOverlapBounds(qmin,qmax, bmin,bmax))
			{
				if (n < maxPolys)
				{
					polyRefs[n] = base | EncodeBasePolyId(DT_POLYTYPE_GROUND, i);
					polys[n] = p;
					n++;
				}
			}
		}
		if (n > 0 && callback)
			callback->ProcessPolygons(tile, polyRefs, polys, n);
		return n;
	}
}

static bool TestPointInCylinder(const float* point, const float* center, const float halfHeight, const float radius)
{
	if (dtMathSqrtf(point[0] - center[0]) + dtMathSqrtf(point[2] - center[2]) > dtMathSqrtf(radius))
		return false;
	return dtMathFabsf(point[1] - center[1]) <= halfHeight;
}

// 查找OMC周围的Link点的Poly
void dtNavMesh::ConnectOffMeshConnection(unsigned int index, float connectRadius, float connectHeight)
{
	dtOffMeshConnection& con = m_offMeshConnections[index];
	const dtPolyRef conRef = EncodeLinkId(con.salt, index);

	if (con.width > 0.0f)
	{

	}
	else
	{
		// Point-to-point connection 点到点
		const float ext[3] = { connectRadius, connectHeight, connectRadius };

		// Link的 末端两点
		for (int i = 0; i < 2; i++)
		{
			if (con.endPoints[i].tileRef)
				continue;
			//const Vector3f searchPos = con.endPoints[i].pos;
			float searchPos[3];
			dtVcopy(searchPos, con.endPoints[i].pos);

			//Vector3f mappedPos;
			//const NavMeshPolyRef mappedRef = FindNearestPoly(con.agentTypeID, searchPos, ext, &mappedPos);
		
			// 找到OMC两个末端引脚所在的Poly，这里就没有Tile了，需要根据坐标找到Tile，找到最近的点
			Vector3f mappedPos;
			dtPolyRef mappedRef = FindNearestPoly(con.agentTypeID, Vector3f(searchPos), Vector3f(ext), &mappedPos);
			if (!mappedRef)
				continue;

			// 测试一下得到的点是否在 OMC的盒子范围
// 			if (!TestPointInCylinder(mappedPos.GetPtr(), searchPos, connectHeight, connectRadius))
// 				continue;

			// 获取Poly和Tile
			const dtMeshTile* mappedTile = NULL;
			const dtPoly* mappedPoly = NULL;

 			if (dtStatusFailed(getTileAndPolyByRef(mappedRef, &mappedTile, &mappedPoly)))
 				continue;

			con.endPoints[i].mapped[0] = con.endPoints[i].mapped[1] = mappedPos;
			con.endPoints[i].tileRef = mappedRef;

			// Link off-mesh connection to target poly.
			unsigned int idx = m_links.Alloc();
			if (idx != DT_NULL_LINK)
			{
				dtLink* link = &m_links[idx];
				link->ref = mappedRef;
				link->edge = (unsigned char)i;
				link->side = 0xff;
				link->bmin = link->bmax = 0;
				// Add to linked list.
	 			dtAssert(idx != con.firstLink);
				link->next = con.firstLink;
				con.firstLink = idx;
			}

			// Start end-point is always connect back to off-mesh connection,
			// Destination end-point only if it is bidirectional link.
			if (i == 0 || (i == 1 && (con.linkDirection & kLinkDirectionTwoWay))) //双向
			{
				// Link target poly to off-mesh connection.
				unsigned int idx = m_links.Alloc();
				if (idx != DT_NULL_LINK)
				{
					dtLink* link = &m_links[idx];
					link->ref = conRef;
					link->edge = (unsigned char)i;
					link->side = 0xff;
					link->bmin = link->bmax = 0;

					// Add to linked list.
					const unsigned int ip = GetPolyIndex(mappedTile, mappedPoly);
	 				dtAssert(idx != mappedTile->polyLinks[ip]);
					link->next = mappedTile->polyLinks[ip];
					mappedTile->polyLinks[ip] = idx;
				}
			}
		}
	}



}

dtPolyRef dtNavMesh::AddOffMeshConnection(const float* spos, const float* epos, int instanceID, bool twoWay, unsigned char areaType, int agentTypeID)
{
	
	OffMeshConnectionParams conn;
	dtVcopy(conn.startPos, spos);
	dtVcopy(conn.endPos, epos);
	conn.up = Vector3f(0, 1, 0);
	conn.width = 0.0f;
	conn.costModifier = -1.0f;
	conn.linkDirection = twoWay ? kLinkDirectionTwoWay : kLinkDirectionOneWay;
	conn.flags = 1 << areaType;
	conn.area = areaType;
	conn.linkType = static_cast<unsigned short>(kLinkTypeManual);
	conn.userID = instanceID;
	conn.agentTypeID = agentTypeID;

	const Vector3f query = Vector3f(m_agentRadius, m_agentMaxClimb, m_agentRadius);

	return AddOffMeshConnection(&conn, query.x, query.y);
}

dtPolyRef dtNavMesh::AddOffMeshConnection(const struct OffMeshConnectionParams* params, float connectRadius, float connectHeight)
{
	unsigned int index = m_offMeshConnections.Alloc();
	dtOffMeshConnection& con = m_offMeshConnections[index];

	// Retain salt.
	unsigned int salt = con.salt;
	memset(&con, 0, sizeof(con));
	con.salt = salt;


	// Add to active off-mesh connection list. 链表头
	con.next = m_firstOffMeshConnection;
	m_firstOffMeshConnection = index;

	// fill out connection struct
	dtVcopy(con.endPoints[0].pos, params->startPos);
	dtVcopy(con.endPoints[1].pos, params->endPos);

	float r[3];
	dtVsub(r, params->endPos, params->startPos);

	Vector3f dir = NormalizeSafe(Vector3f(r));
	if (Magnitude(dir) < 0.00001f)
		dir = Vector3f(0, 0, 1);

	con.axisY = params->up;
	con.axisX = Cross(con.axisY, dir);
	con.axisZ = Cross(con.axisX, con.axisY);
	con.width = params->width;
	con.costModifier = params->costModifier;
	con.linkDirection = params->linkDirection;
	con.flags = params->flags;
	con.area = params->area;
	con.linkType = params->linkType;
	con.userId = params->userID;
	con.agentTypeID = params->agentTypeID;

	// 包围盒计算
	con.bounds.Init();
	if (con.width > 0.0f) // 有宽度的OMC
	{
		const Vector3f extent = NormalizeSafe(con.axisX) * con.width * 0.5f;
		con.bounds.Encapsulate(con.endPoints[0].pos - extent);
		con.bounds.Encapsulate(con.endPoints[0].pos + extent);
		con.bounds.Encapsulate(con.endPoints[1].pos - extent);
		con.bounds.Encapsulate(con.endPoints[1].pos + extent);
	}
	else
	{
		con.bounds.Encapsulate(con.endPoints[0].pos);
		con.bounds.Encapsulate(con.endPoints[1].pos);
	}

	con.firstLink = DT_NULL_LINK;

	const dtPolyRef dynRef = EncodeLinkId(con.salt, index);// 编码OMC

	// Connect 建立Link列表
	ConnectOffMeshConnection(index, connectRadius, connectHeight);

	return dynRef;
}

void dtNavMesh::UnconnectOffMeshConnection(unsigned int index)
{
	// Remove links
	dtOffMeshConnection& con = m_offMeshConnections[index];
	const dtPolyRef ref = EncodeLinkId(con.salt, index);
	unsigned int i = con.firstLink;
	while (i != DT_NULL_LINK)
	{
		unsigned int next = m_links[i].next;
		RemoveLinkBetween(m_links[i].ref, ref);
		m_links.Release(i);
		i = next;
	}
	con.firstLink = DT_NULL_LINK;
}

dtStatus dtNavMesh::RemoveOffMeshConnection(const dtPolyRef ref)
{
	if (DecodePolyIdType(ref) != DT_POLYTYPE_OFFMESH_CONNECTION)
		return DT_FAILURE | DT_INVALID_PARAM;

	unsigned int index = DecodePolyIdPoly(ref);
	if (index >= m_offMeshConnections.Capacity())
		return DT_FAILURE | DT_INVALID_PARAM;

	unsigned int salt = DecodePolyIdSalt(ref);
	if (salt != m_offMeshConnections[index].salt)
		return DT_FAILURE | DT_INVALID_PARAM;

	// Remove links
	UnconnectOffMeshConnection(index);

	// Find previous offmesh link to be able to remove from the list.
	unsigned int i = m_firstOffMeshConnection;
	unsigned int prev = DT_NULL_LINK;
	while (i != DT_NULL_LINK)
	{
		if (i == index)
			break;
		prev = i;
		i = m_offMeshConnections[i].next;
	}
	// Remove from linked list
	const unsigned int next = m_offMeshConnections[index].next;
	if (prev == DT_NULL_LINK)
		m_firstOffMeshConnection = next;
	else
		m_offMeshConnections[prev].next = next;

	// Bump salt to distiguish deleted connections.
	m_offMeshConnections[index].salt++;
	if (m_offMeshConnections[index].salt == 0)
		m_offMeshConnections[index].salt = 1;

	m_offMeshConnections.Release(index);

	//BumpTimeStamp();

	return DT_SUCCESS;
}

/// @par
///
/// The add operation will fail if the data is in the wrong format, the allocated tile
/// space is full, or there is a tile already at the specified reference.
///
/// The lastRef parameter is used to restore a tile with the same tile
/// reference it had previously used.  In this case the #dtPolyRef's for the
/// tile will be restored to the same values they were before the tile was 
/// removed.
///
/// The nav mesh assumes exclusive access to the data passed and will make
/// changes to the dynamic portion of the data. For that reason the data
/// should not be reused in other nav meshes until the tile has been successfully
/// removed from this nav mesh.
///
/// @see dtCreateNavMeshData, #removeTile
dtStatus dtNavMesh::addTile(unsigned char* data, int dataSize, int flags,
							dtTileRef lastRef, dtTileRef* result)
{
	// 检查格式，通过Header数据检查
	// Make sure the data is in right format.
	dtMeshHeader* header = (dtMeshHeader*)data;
	if (header->magic != DT_NAVMESH_MAGIC)
		return DT_FAILURE | DT_WRONG_MAGIC;
	if (header->version != DT_NAVMESH_VERSION)
		return DT_FAILURE | DT_WRONG_VERSION;
		
	// Make sure the location is free.
	if (getTileAt(header->x, header->y, header->layer))
		return DT_FAILURE | DT_ALREADY_OCCUPIED;
		
	// Allocate a tile.
	dtMeshTile* tile = 0;
	if (!lastRef)
	{
		if (m_nextFree)
		{
			tile = m_nextFree;
			m_nextFree = tile->next;
			tile->next = 0;
		}
	}
	else
	{
		// Try to relocate the tile to specific index with same salt.
		int tileIndex = (int)DecodePolyIdTile((dtPolyRef)lastRef);
		if (tileIndex >= m_maxTiles)
			return DT_FAILURE | DT_OUT_OF_MEMORY;
		// Try to find the specific tile id from the free list.
		dtMeshTile* target = &m_tiles[tileIndex];
		dtMeshTile* prev = 0;
		tile = m_nextFree;
		while (tile && tile != target)
		{
			prev = tile;
			tile = tile->next;
		}
		// Could not find the correct location.
		if (tile != target)
			return DT_FAILURE | DT_OUT_OF_MEMORY;
		// Remove from freelist
		if (!prev)
			m_nextFree = tile->next;
		else
			prev->next = tile->next;

		// Restore salt.
		tile->salt = DecodePolyIdSalt((dtPolyRef)lastRef);
	}

	// Make sure we could allocate a tile.
	if (!tile)
		return DT_FAILURE | DT_OUT_OF_MEMORY;
	
	// Insert tile into the position lut.
	int h = computeTileHash(header->x, header->y, m_tileLutMask);
	tile->next = m_posLookup[h];
	m_posLookup[h] = tile;
	
	// Patch header pointers.
	const int headerSize = dtAlign4(sizeof(dtMeshHeader));
	const int vertsSize = dtAlign4(sizeof(float)*3*header->vertCount);
	const int polysSize = dtAlign4(sizeof(dtPoly)*header->polyCount);
	const int detailMeshesSize = dtAlign4(sizeof(dtPolyDetail)*header->detailMeshCount);
	const int detailVertsSize = dtAlign4(sizeof(float)*3*header->detailVertCount);
	const int detailTrisSize = dtAlign4(sizeof(unsigned char)*4*header->detailTriCount);
	const int bvtreeSize = dtAlign4(sizeof(dtBVNode)*header->bvNodeCount);
	
	unsigned char* d = data + headerSize;
	tile->verts = dtGetThenAdvanceBufferPointer<float>(d, vertsSize);
	tile->polys = dtGetThenAdvanceBufferPointer<dtPoly>(d, polysSize);
	tile->detailMeshes = dtGetThenAdvanceBufferPointer<dtPolyDetail>(d, detailMeshesSize);
	tile->detailVerts = dtGetThenAdvanceBufferPointer<float>(d, detailVertsSize);
	tile->detailTris = dtGetThenAdvanceBufferPointer<unsigned char>(d, detailTrisSize);
	tile->bvTree = dtGetThenAdvanceBufferPointer<dtBVNode>(d, bvtreeSize);

	// If there are no items in the bvtree, reset the tree pointer.
	if (!bvtreeSize)
		tile->bvTree = 0;

	// Init tile.
	tile->header = header;
	tile->data = data;
	tile->dataSize = dataSize;
	tile->flags = flags;

	connectIntLinks(tile);

	// Create connections with neighbour tiles.
	static const int MAX_NEIS = 32;
	dtMeshTile* neis[MAX_NEIS];
	int nneis;
	
	// Connect with layers in current tile.
	nneis = getTilesAt(header->x, header->y, neis, MAX_NEIS);
	for (int j = 0; j < nneis; ++j)
	{
		if (neis[j] == tile)
			continue;
	
		connectExtLinks(tile, neis[j], -1);
		connectExtLinks(neis[j], tile, -1);
	}
	
	// Connect with neighbour tiles.
	for (int i = 0; i < 8; ++i)
	{
		nneis = getNeighbourTilesAt(header->x, header->y, i, neis, MAX_NEIS);
		for (int j = 0; j < nneis; ++j)
		{
			connectExtLinks(tile, neis[j], i);
			connectExtLinks(neis[j], tile, dtOppositeTile(i));
		}
	}

	ConnectOffMeshConnectionsToTile(tile);
	
	if (result)
		*result = getTileRef(tile);
	
	return DT_SUCCESS;
}

const dtMeshTile* dtNavMesh::getTileAt(const int x, const int y, const int layer) const
{
	// Find tile based on hash.
	int h = computeTileHash(x,y,m_tileLutMask);
	dtMeshTile* tile = m_posLookup[h];
	while (tile)
	{
		if (tile->header &&
			tile->header->x == x &&
			tile->header->y == y &&
			tile->header->layer == layer)
		{
			return tile;
		}
		tile = tile->next;
	}
	return 0;
}

int dtNavMesh::getNeighbourTilesAt(const int x, const int y, const int side, dtMeshTile** tiles, const int maxTiles) const
{
	int nx = x, ny = y;
	switch (side)
	{
		case 0: nx++; break;
		case 1: nx++; ny++; break;
		case 2: ny++; break;
		case 3: nx--; ny++; break;
		case 4: nx--; break;
		case 5: nx--; ny--; break;
		case 6: ny--; break;
		case 7: nx++; ny--; break;
	};

	return getTilesAt(nx, ny, tiles, maxTiles);
}

int dtNavMesh::getTilesAt(const int x, const int y, dtMeshTile** tiles, const int maxTiles) const
{
	int n = 0;
	
	// Find tile based on hash.
	int h = computeTileHash(x,y,m_tileLutMask);
	dtMeshTile* tile = m_posLookup[h];
	while (tile)
	{
		if (tile->header &&
			tile->header->x == x &&
			tile->header->y == y)
		{
			if (n < maxTiles)
				tiles[n++] = tile;
		}
		tile = tile->next;
	}
	
	return n;
}

/// @par
///
/// This function will not fail if the tiles array is too small to hold the
/// entire result set.  It will simply fill the array to capacity.
int dtNavMesh::getTilesAt(const int x, const int y, dtMeshTile const** tiles, const int maxTiles) const
{
	int n = 0;
	
	// Find tile based on hash.
	int h = computeTileHash(x,y,m_tileLutMask);
	dtMeshTile* tile = m_posLookup[h];
	while (tile)
	{
		if (tile->header &&
			tile->header->x == x &&
			tile->header->y == y)
		{
			if (n < maxTiles)
				tiles[n++] = tile;
		}
		tile = tile->next;
	}
	
	return n;
}


dtTileRef dtNavMesh::getTileRefAt(const int x, const int y, const int layer) const
{
	// Find tile based on hash.
	int h = computeTileHash(x,y,m_tileLutMask);
	dtMeshTile* tile = m_posLookup[h];
	while (tile)
	{
		if (tile->header &&
			tile->header->x == x &&
			tile->header->y == y &&
			tile->header->layer == layer)
		{
			return getTileRef(tile);
		}
		tile = tile->next;
	}
	return 0;
}

const dtMeshTile* dtNavMesh::getTileByRef(dtTileRef ref) const
{
	if (!ref)
		return 0;
	unsigned int tileIndex = DecodePolyIdTile((dtPolyRef)ref);
	unsigned int tileSalt = DecodePolyIdSalt((dtPolyRef)ref);
	if ((int)tileIndex >= m_maxTiles)
		return 0;
	const dtMeshTile* tile = &m_tiles[tileIndex];
	if (tile->salt != tileSalt)
		return 0;
	return tile;
}

int dtNavMesh::getMaxTiles() const
{
	return m_maxTiles;
}

dtMeshTile* dtNavMesh::getTile(int i)
{
	return &m_tiles[i];
}

const dtMeshTile* dtNavMesh::getTile(int i) const
{
	return &m_tiles[i];
}

void dtNavMesh::calcTileLoc(const float* pos, int* tx, int* ty) const
{
	*tx = (int)floorf((pos[0]-m_orig[0]) / m_tileWidth);
	*ty = (int)floorf((pos[2]-m_orig[2]) / m_tileHeight);
}

dtStatus dtNavMesh::getTileAndPolyByRef(const dtPolyRef ref, const dtMeshTile** tile, const dtPoly** poly) const
{
	if (!ref)
		return DT_FAILURE;
	unsigned int salt, it, type, ip;
	DecodePolyId(&salt, &it, &type, &ip, ref); // 解码出来tile type poly

	if (it >= (unsigned int)m_maxTiles)
		return DT_FAILURE | DT_INVALID_PARAM;
	if (m_tiles[it].salt != salt /*|| m_tiles[it].header == 0*/)
		return DT_FAILURE | DT_INVALID_PARAM;
	if (type == DT_POLYTYPE_OFFMESH_CONNECTION)
	{
		*tile = 0;
		*poly = 0;
	}
	else
	{
		if ( m_tiles[it].header == 0 )
			return DT_FAILURE | DT_INVALID_PARAM;

		if (ip >= (unsigned int)m_tiles[it].header->polyCount)
			return DT_FAILURE | DT_INVALID_PARAM;
		*tile = &m_tiles[it];
		*poly = &m_tiles[it].polys[ip];
	}
	return DT_SUCCESS;
}


/// @par
///
/// @warning Only use this function if it is known that the provided polygon
/// reference is valid. This function is faster than #getTileAndPolyByRef, but
/// it does not validate the reference.
void dtNavMesh::getTileAndPolyByRefUnsafe(const dtPolyRef ref, const dtMeshTile** tile, const dtPoly** poly) const
{
	unsigned int salt, it, type, ip;
	DecodePolyId(&salt, &it, &type, &ip, ref);
	if (type == DT_POLYTYPE_OFFMESH_CONNECTION)
	{
		*tile = 0;
		*poly = 0;
	}
	else
	{
		*tile = &m_tiles[it];
		*poly = &m_tiles[it].polys[ip];
	}
}

bool dtNavMesh::isValidPolyRef(dtPolyRef ref) const
{
	if (!ref)
		return false;
	unsigned int salt, it, type, ip;
	DecodePolyId(&salt, &it, &type, &ip, ref);
	if (type == DT_POLYTYPE_OFFMESH_CONNECTION)
	{
		if (ip >= m_offMeshConnections.Capacity())
			return false;
		if (m_offMeshConnections[ip].salt != salt)
			return false;
	}
	else
	{
		//if (it >= (unsigned int)m_tiles.Capacity()) // LWF_TMP
		if (it >= (unsigned int)m_maxTiles)
			return false;
		if (m_tiles[it].salt != salt || m_tiles[it].header == 0)
			return false;
		if (ip >= (unsigned int)m_tiles[it].header->polyCount)
			return false;
	}
	return true;
}

/// @par
///
/// This function returns the data for the tile so that, if desired,
/// it can be added back to the navigation mesh at a later point.
///
/// @see #addTile
dtStatus dtNavMesh::removeTile(dtTileRef ref, unsigned char** data, int* dataSize)
{
	if (!ref)
		return DT_FAILURE | DT_INVALID_PARAM;
	unsigned int tileIndex = DecodePolyIdTile((dtPolyRef)ref);
	unsigned int tileSalt = DecodePolyIdSalt((dtPolyRef)ref);
	if ((int)tileIndex >= m_maxTiles)
		return DT_FAILURE | DT_INVALID_PARAM;
	dtMeshTile* tile = &m_tiles[tileIndex];
	if (tile->salt != tileSalt)
		return DT_FAILURE | DT_INVALID_PARAM;
	
	// Remove tile from hash lookup.
	int h = computeTileHash(tile->header->x,tile->header->y,m_tileLutMask);
	dtMeshTile* prev = 0;
	dtMeshTile* cur = m_posLookup[h];
	while (cur)
	{
		if (cur == tile)
		{
			if (prev)
				prev->next = cur->next;
			else
				m_posLookup[h] = cur->next;
			break;
		}
		prev = cur;
		cur = cur->next;
	}
	
	// Remove connections to neighbour tiles.
	static const int MAX_NEIS = 32;
	dtMeshTile* neis[MAX_NEIS];
	int nneis;
	
	// Disconnect from other layers in current tile.
	nneis = getTilesAt(tile->header->x, tile->header->y, neis, MAX_NEIS);
	for (int j = 0; j < nneis; ++j)
	{
		if (neis[j] == tile) continue;
		unconnectLinks(neis[j], tile);
	}
	
	// Disconnect from neighbour tiles.
	for (int i = 0; i < 8; ++i)
	{
		nneis = getNeighbourTilesAt(tile->header->x, tile->header->y, i, neis, MAX_NEIS);
		for (int j = 0; j < nneis; ++j)
			unconnectLinks(neis[j], tile);
	}

	// Remove links to connected tiles.
	UnconnectLinks(tile);
	// Remove off-mesh connections connected to the tile.
	// unconnectLinks() unconnects most of them, but one-directional
	// links may not be unconnected.
	UnconnectOffMeshConnectionsToTile(ref);
		
	// Reset tile.
	if (tile->flags & DT_TILE_FREE_DATA)
	{
		// Owns data
		dtFree(tile->data);
		tile->data = 0;
		tile->dataSize = 0;
		if (data) *data = 0;
		if (dataSize) *dataSize = 0;
	}
	else
	{
		if (data) *data = tile->data;
		if (dataSize) *dataSize = tile->dataSize;
	}

	tile->header = 0;
	tile->flags = 0;
	tile->polys = 0;
	tile->verts = 0;
	tile->detailMeshes = 0;
	tile->detailVerts = 0;
	tile->detailTris = 0;
	tile->bvTree = 0;

	// Update salt, salt should never be zero.
#ifdef DT_POLYREF64
	tile->salt = (tile->salt+1) & ((1<<DT_SALT_BITS)-1);
#else
	tile->salt = (tile->salt+1) & ((1<<m_saltBits)-1);
#endif
	if (tile->salt == 0)
		tile->salt++;

	// Add to free list.
	tile->next = m_nextFree;
	m_nextFree = tile;

	return DT_SUCCESS;
}

dtTileRef dtNavMesh::getTileRef(const dtMeshTile* tile) const
{
	if (!tile) return 0;
	const unsigned int it = (unsigned int)(tile - m_tiles);
	return (dtTileRef)EncodePolyId(tile->salt, it, 0, 0);
}

/// @par
///
/// Example use case:
/// @code
///
/// const dtPolyRef base = navmesh->getPolyRefBase(tile);
/// for (int i = 0; i < tile->header->polyCount; ++i)
/// {
///     const dtPoly* p = &tile->polys[i];
///     const dtPolyRef ref = base | (dtPolyRef)i;
///     
///     // Use the reference to access the polygon data.
/// }
/// @endcode
dtPolyRef dtNavMesh::getPolyRefBase(const dtMeshTile* tile) const
{
	if (!tile) return 0;
	const unsigned int it = (unsigned int)(tile - m_tiles);
	return EncodePolyId(tile->salt, it, 0, 0);// 第一个Poly的编码
}

struct dtTileState
{
	int magic;								// Magic number, used to identify the data.
	int version;							// Data version number.
	dtTileRef ref;							// Tile ref at the time of storing the data.
};

struct dtPolyState
{
	unsigned short flags;						// Flags (see dtPolyFlags).
	unsigned char area;							// Area ID of the polygon.
};

///  @see #storeTileState
int dtNavMesh::getTileStateSize(const dtMeshTile* tile) const
{
	if (!tile) return 0;
	const int headerSize = dtAlign4(sizeof(dtTileState));
	const int polyStateSize = dtAlign4(sizeof(dtPolyState) * tile->header->polyCount);
	return headerSize + polyStateSize;
}

/// @par
///
/// Tile state includes non-structural data such as polygon flags, area ids, etc.
/// @note The state data is only valid until the tile reference changes.
/// @see #getTileStateSize, #restoreTileState
dtStatus dtNavMesh::storeTileState(const dtMeshTile* tile, unsigned char* data, const int maxDataSize) const
{
	// Make sure there is enough space to store the state.
	const int sizeReq = getTileStateSize(tile);
	if (maxDataSize < sizeReq)
		return DT_FAILURE | DT_BUFFER_TOO_SMALL;
		
	dtTileState* tileState = dtGetThenAdvanceBufferPointer<dtTileState>(data, dtAlign4(sizeof(dtTileState)));
	dtPolyState* polyStates = dtGetThenAdvanceBufferPointer<dtPolyState>(data, dtAlign4(sizeof(dtPolyState) * tile->header->polyCount));
	
	// Store tile state.
	tileState->magic = DT_NAVMESH_STATE_MAGIC;
	tileState->version = DT_NAVMESH_STATE_VERSION;
	tileState->ref = getTileRef(tile);
	
	// Store per poly state.
	for (int i = 0; i < tile->header->polyCount; ++i)
	{
		const dtPoly* p = &tile->polys[i];
		dtPolyState* s = &polyStates[i];
		s->flags = p->flags;
		s->area = p->getArea();
	}
	
	return DT_SUCCESS;
}

/// @par
///
/// Tile state includes non-structural data such as polygon flags, area ids, etc.
/// @note This function does not impact the tile's #dtTileRef and #dtPolyRef's.
/// @see #storeTileState
dtStatus dtNavMesh::restoreTileState(dtMeshTile* tile, const unsigned char* data, const int maxDataSize)
{
	// Make sure there is enough space to store the state.
	const int sizeReq = getTileStateSize(tile);
	if (maxDataSize < sizeReq)
		return DT_FAILURE | DT_INVALID_PARAM;
	
	const dtTileState* tileState = dtGetThenAdvanceBufferPointer<const dtTileState>(data, dtAlign4(sizeof(dtTileState)));
	const dtPolyState* polyStates = dtGetThenAdvanceBufferPointer<const dtPolyState>(data, dtAlign4(sizeof(dtPolyState) * tile->header->polyCount));
	
	// Check that the restore is possible.
	if (tileState->magic != DT_NAVMESH_STATE_MAGIC)
		return DT_FAILURE | DT_WRONG_MAGIC;
	if (tileState->version != DT_NAVMESH_STATE_VERSION)
		return DT_FAILURE | DT_WRONG_VERSION;
	if (tileState->ref != getTileRef(tile))
		return DT_FAILURE | DT_INVALID_PARAM;
	
	// Restore per poly state.
	for (int i = 0; i < tile->header->polyCount; ++i)
	{
		dtPoly* p = &tile->polys[i];
		const dtPolyState* s = &polyStates[i];
		p->flags = s->flags;
		p->setArea(s->area);
	}
	
	return DT_SUCCESS;
}

/// @par
///
/// Off-mesh connections are stored in the navigation mesh as special 2-vertex 
/// polygons with a single edge. At least one of the vertices is expected to be 
/// inside a normal polygon. So an off-mesh connection is "entered" from a 
/// normal polygon at one of its endpoints. This is the polygon identified by 
/// the prevRef parameter.
/// 找到起终点
dtStatus dtNavMesh::getOffMeshConnectionPolyEndPoints(dtPolyRef prevRef, dtPolyRef polyRef, float* startPos, float* endPos) const
{
	// 检查Poly合法性
	if (!isValidPolyRef(polyRef))
		return DT_FAILURE;

	// 检查Type
	// Make sure that the current poly is indeed off-mesh link.
	if (DecodePolyIdType(polyRef) != DT_POLYTYPE_OFFMESH_CONNECTION)
		return DT_FAILURE;

	// Figure out which way to hand out the vertices.
	int idx0 = 0, idx1 = 1;
	
	const dtOffMeshConnection* con = GetOffMeshConnection(polyRef);
	if (con == NULL)
		return DT_FAILURE | DT_INVALID_PARAM;

	// 找到Poly的连接到目标Poly的Link
	// Find link which leads to previous polygon to figure out which way we are travering the off-mesh connection.
	const dtLink* foundLink = NULL;
	//for (const dtLink* link = GetLink(con->firstLink); link != NULL; link = GetNextLink(link))
	for (const dtLink* link = GetFirstLink(polyRef); link != NULL; link = GetNextLink(link))
	{
		if (link->ref == prevRef)
		{
			foundLink = link;
			break;
		}
	}
	if (!foundLink)
		return DT_FAILURE;
	
	// OffMesh connection can only have edge 0 or 1
	dtAssert(foundLink->edge == 0 || foundLink->edge == 1);

	// Set endpoints based on direction
	if (foundLink->edge == 0)
	{
		dtVcopy(startPos, con->endPoints[0].pos);
		dtVcopy(endPos, con->endPoints[1].pos);
		return DT_SUCCESS;
	}
	else if (foundLink->edge == 1) // 相反方向
	{
		dtVcopy(startPos, con->endPoints[1].pos);
		dtVcopy(endPos, con->endPoints[0].pos);
		return DT_SUCCESS;
	}

	return DT_FAILURE;
}

dtStatus dtNavMesh::SetOffMeshConnectionCostModifier(dtPolyRef ref, float costOverride)
{
	if (DecodePolyIdType(ref) != DT_POLYTYPE_OFFMESH_CONNECTION)
		return DT_FAILURE | DT_INVALID_PARAM;
	dtOffMeshConnection* con = GetOffMeshConnectionUnsafe(ref);
	if (con == NULL)
		return  DT_FAILURE | DT_INVALID_PARAM;

	if (costOverride >= 0.0f)
		con->costModifier = costOverride;
	else
		con->costModifier = -1.0f;

	//BumpTimeStamp();
	return DT_SUCCESS;
}

dtStatus dtNavMesh::SetOffMeshConnectionFlags(dtPolyRef ref, unsigned int flags)
{
	if (DecodePolyIdType(ref) != DT_POLYTYPE_OFFMESH_CONNECTION)
		return DT_FAILURE | DT_INVALID_PARAM;
	dtOffMeshConnection* con = GetOffMeshConnectionUnsafe(ref);
	if (con == NULL)
		return DT_FAILURE | DT_INVALID_PARAM;
	con->flags = flags;

	//BumpTimeStamp();
	return DT_SUCCESS;
}

void dtNavMesh::SetOffMeshConnectionActive(dtPolyRef ref, bool active)
{
	unsigned int flags = 0;
	unsigned char area = 0;
	GetPolyFlagsAndArea(ref, &flags, &area);
	if ((flags == 0 && !active) || (flags != 0 && active))
		return;
	if (active)
		SetOffMeshConnectionFlags(ref, 1 << (unsigned int)area);
	else
		SetOffMeshConnectionFlags(ref, 0);
}

// 获取OffMesh
const dtOffMeshConnection* dtNavMesh::GetOffMeshConnection(const dtPolyRef ref) const
{
	if (DecodePolyIdType(ref) != DT_POLYTYPE_OFFMESH_CONNECTION) // 检查类型
		return NULL;

	// 检查下标
	unsigned int index = DecodePolyIdPoly(ref);
	if (index >= m_offMeshConnections.Capacity())
		return NULL;

	// Salt 更新状态
	unsigned int salt = DecodePolyIdSalt(ref);
	if (salt != m_offMeshConnections[index].salt)
		return NULL;

	return &m_offMeshConnections[index];
}

// 非Constant版
dtOffMeshConnection* dtNavMesh::GetOffMeshConnectionUnsafe(const dtPolyRef ref)
{
	if (DecodePolyIdType(ref) != DT_POLYTYPE_OFFMESH_CONNECTION)// 检查类型
		return NULL;

	// 检查下标
	unsigned int index = DecodePolyIdPoly(ref);
	if (index >= m_offMeshConnections.Capacity())
		return NULL;

	// Salt 更新状态
	unsigned int salt = DecodePolyIdSalt(ref);
	if (salt != m_offMeshConnections[index].salt)
		return NULL;

	return &m_offMeshConnections[index];
}

dtStatus dtNavMesh::setPolyFlags(dtPolyRef ref, unsigned short flags)
{
	if (!ref) return DT_FAILURE;
	unsigned int salt, it,type, ip;
	//decodePolyId(ref, salt, it, ip);
	DecodePolyId(&salt, &it, &type, &ip, ref);// 解码出来

	// 检查一遍各个索引是否合理
	if (it >= (unsigned int)m_maxTiles) return DT_FAILURE | DT_INVALID_PARAM;
	if (m_tiles[it].salt != salt || m_tiles[it].header == 0) return DT_FAILURE | DT_INVALID_PARAM;
	dtMeshTile* tile = &m_tiles[it];
	if (ip >= (unsigned int)tile->header->polyCount) return DT_FAILURE | DT_INVALID_PARAM;
	dtPoly* poly = &tile->polys[ip];
	
	// Change flags.
	poly->flags = flags;
	
	return DT_SUCCESS;
}

unsigned int dtNavMesh::GetPolyFlags(dtPolyRef ref) const
{
	if (DecodePolyIdType(ref) == DT_POLYTYPE_OFFMESH_CONNECTION)
	{
		const dtOffMeshConnection* con = GetOffMeshConnection(ref);
		if (con == NULL)
			return 0;
		return con->flags;
	}
	else
	{
		const dtMeshTile* tile = NULL;
		const dtPoly* poly = NULL;
		if (dtStatusFailed(getTileAndPolyByRef(ref, &tile, &poly)))
			return 0;
		return poly->flags;
	}
	return 0;
}

dtStatus dtNavMesh::setPolyArea(dtPolyRef ref, unsigned char area)
{
	if (!ref) return DT_FAILURE;
	unsigned int salt, it,type, ip;
	//decodePolyId(ref, salt, it, ip);
	DecodePolyId(&salt, &it, &type, &ip, ref);// 解码出来

	// 检查一遍各个索引是否合理
	if (it >= (unsigned int)m_maxTiles) return DT_FAILURE | DT_INVALID_PARAM;
	if (m_tiles[it].salt != salt || m_tiles[it].header == 0) return DT_FAILURE | DT_INVALID_PARAM;
	dtMeshTile* tile = &m_tiles[it];
	if (ip >= (unsigned int)tile->header->polyCount) return DT_FAILURE | DT_INVALID_PARAM;
	dtPoly* poly = &tile->polys[ip];
	
	poly->setArea(area);
	
	return DT_SUCCESS;
}

unsigned char dtNavMesh::GetPolyArea(dtPolyRef ref) const
{
	if (DecodePolyIdType(ref) == DT_POLYTYPE_OFFMESH_CONNECTION)
	{
		const dtOffMeshConnection* con = GetOffMeshConnection(ref);
		if (con == NULL)
			return 0;
		return con->area;
	}
	else
	{
		const dtMeshTile* tile = NULL;
		const dtPoly* poly = NULL;
		if (dtStatusFailed(getTileAndPolyByRef(ref, &tile, &poly)))
			return 0;
		return poly->area;
	}
	return 0;
}

void dtNavMesh::GetPolyFlagsAndArea(dtPolyRef ref, unsigned int* flags, unsigned char* area) const
{
	if (flags)
		*flags = 0;
	if (area)
		*area = 0;

	if (DecodePolyIdType(ref) == DT_POLYTYPE_OFFMESH_CONNECTION)
	{
		const dtOffMeshConnection* con = GetOffMeshConnection(ref);
		if (con == NULL)
			return;
		if (flags)
			*flags = con->flags;
		if (area)
			*area = con->area;
	}
	else
	{
		const dtMeshTile* tile = NULL;
		const dtPoly* poly = NULL;
		if (dtStatusFailed(getTileAndPolyByRef(ref, &tile, &poly)))
			return;
		if (flags)
			*flags = poly->flags;
		if (area)
			*area = poly->area;
	}
}

int dtNavMesh::GetPolyGeometry(dtPolyRef ref, float* verts, dtPolyRef* neighbours, int maxNeisPerEdge) const
{
	if (DecodePolyIdType(ref) == DT_POLYTYPE_OFFMESH_CONNECTION)
	{
		// TODO: should we return off-mesh link geometry?
		return 0;
	}
	else
	{
		const dtMeshTile* tile = NULL;
		const dtPoly* poly = NULL;
		if (dtStatusFailed(getTileAndPolyByRef(ref, &tile, &poly)))
			return 0;
		const int nverts = poly->vertCount;

		// Copy vertices
		if (verts != NULL)
		{
			for (int i = 0; i < nverts; i++)
				dtVcopy(&verts[i * 3], &tile->verts[poly->verts[i] * 3]);
				//verts[i] = tile->verts[poly->verts[i]];
		}

		// Copy neighbours
		if (neighbours != NULL)
		{
			for (int i = 0; i < nverts * maxNeisPerEdge; i++)
				neighbours[i] = 0;

			const unsigned int ip = GetPolyIndex(tile, poly);
			const unsigned int firstLink = tile->polyLinks[ip];
			for (const dtLink* link = GetLink(firstLink); link != NULL; link = GetNextLink(link))
			{
				// Only accept polygon connections.
				if (DecodePolyIdType(link->ref) == DT_POLYTYPE_OFFMESH_CONNECTION)
					continue;
				int index = (int)link->edge;
				if (index >= 0 && index < nverts)
				{
					// find empty slot
					for (int j = 0; j < maxNeisPerEdge; j++)
					{
						if (neighbours[index * maxNeisPerEdge + j] == 0)
						{
							neighbours[index * maxNeisPerEdge + j] = link->ref;
							break;
						}
					}
				}
			}
		}
		return nverts;
	}
	return 0;
}

