import gg/vectors
import math
import algorithm
import sequtils

type
  Bin = tuple[ind, bin: int]
  Pair = tuple[st, en: int]
  Triple = array[3, int]
  MeshData = ref object
    bounds: BBox2f
    vert*: seq[Vec2f]
    scaledVert: seq[Vec2f]
    conEdge: seq[Pair]
    tri*: seq[Triple]
    adj*: seq[Triple]
    bin: seq[Bin]
    vertToTri*: seq[seq[int]]

const
  boundingL = 1000.0

proc binCmp(x, y: Bin): int = cmp(x.bin, y.bin)

proc isEdgeIntersecting(edgeA, edgeB: (Vec2f, Vec2f)): bool =
  let vecA0A1 = edgeA[1] - edgeA[0]
  let vecA0B0 = edgeB[0] - edgeA[0]
  let vecA0B1 = edgeB[1] - edgeA[0]
  let AxB0 = vecA0A1 ^ vecA0B0
  let AxB1 = vecA0A1 ^ vecA0B1

  # Check if the endpoints of edgeB are on the same side of edgeA
  if (AxB0 > 0 and AxB1 > 0) or (AxB0 < 0 and AxB1 < 0):
    return false

  let vecB0B1 = edgeB[1] - edgeB[0]
  let vecB0A0 = edgeA[0] - edgeB[0]
  let vecB0A1 = edgeA[1] - edgeB[0]
  let BxA0 = vecB0B1 ^ vecB0A0
  let BxA1 = vecB0B1 ^ vecB0A1

  # Check if the endpoints of edgeA are on the same side of edgeB
  if (BxA0 > 0 and BxA1 > 0) or (BxA0 < 0 and BxA1 < 0):
    return false

  # Special case of colinear edges
  if abs(AxB0) < 1e-14 and abs(AxB1) < 1e-14:
    # Separated in x
    if (max(edgeB[0].x, edgeB[1].x) < min(edgeA[0].x, edgeA[1].x)) or
      (min(edgeB[0].x, edgeB[1].x) > max(edgeA[0].x, edgeA[1].x)):
        return false
    
    # Separated in y
    if (max(edgeB[0].y, edgeB[1].y) < min(edgeA[0].y, edgeA[1].y)) or
      (min(edgeB[0].y, edgeB[1].y) > max(edgeA[0].y, edgeA[1].y)):
        return false
  return true

proc isEdgeIntersectingAtEndpoint(edgeA, edgeB: (Vec2f, Vec2f)): bool =
  const rsqTol = 1e-13
  (edgeA[0].distSq(edgeB[0]) < rsqTol) or
  (edgeA[0].distSq(edgeB[1]) < rsqTol) or
  (edgeA[1].distSq(edgeB[0]) < rsqTol) or
  (edgeA[1].distSq(edgeB[1]) < rsqTol)

proc isQuadConvex(p0, p1, p2, p3: Vec2f): bool =
  isEdgeIntersecting((p0, p2), (p1, p3))

proc isSameEdge(edge0, edge1: Pair): bool =
  (edge0.st == edge1.st and edge0.en == edge1.en) or
  (edge0.st == edge1.en and edge0.en == edge1.st)

proc isDelaunay(vtri: array[3, Vec2f], p: Vec2f): bool =
  let p0 = vtri[0] - p
  let p1 = vtri[1] - p
  let p2 = vtri[2] - p
  let p0sq = p0.lenSq
  let p1sq = p1.lenSq
  let p2sq = p2.lenSq
  let det = p0.x * (p1.y * p2sq - p2.y * p1sq) - p0.y * (p1.x * p2sq - p2.x * p1sq) + p0sq * (p1 ^ p2)
  return det <= 0

proc newMesh*(vert: seq[Vec2f], edges: seq[Pair] = @[]): MeshData =
  MeshData(
    bounds: vert.bbox,
    vert: vert,
    conEdge: edges,
  )

proc setupDelaunay(mesh: MeshData) =
  let nVertex = mesh.vert.len
  let nBinsX = round(pow(nVertex.float, 0.25)).int

  # Compute scaled vertex coordinates and assign each vertex to a bin
  let screenL = max(mesh.bounds.width, mesh.bounds.height)
  mesh.scaledVert = newSeq[Vec2f](nVertex)
  mesh.bin = newSeq[Bin](nVertex)
  for i, vert in mesh.vert:
    let scaled = (vert - mesh.bounds.min) / screenL
    mesh.scaledVert[i] = scaled

    let indI = round(scaled.x * (nBinsX.float - 1.0)).int
    let indJ = round(scaled.y * (nBinsX.float - 1.0)).int
    let binId = if indJ mod 2 == 0:
      indJ * nBinsX + indI
    else:
      (indJ + 1) * nBinsX - indI - 1
    mesh.bin[i] = (ind: i, bin: binId)

  # Add super-triangle vertices (far away)
  let D = boundingL
  mesh.scaledVert.add((-D + 0.5, -D / sqrt(3.0) + 0.5))
  mesh.scaledVert.add(( D + 0.5, -D / sqrt(3.0) + 0.5))
  mesh.scaledVert.add((     0.5, 2 * D / sqrt(3.0) + 0.5))
  for i in nVertex..<(nVertex + 3):
    mesh.vert.add(screenL * mesh.scaledVert[i] + mesh.bounds.min)

  # Sort the vertices in ascending bin order
  mesh.bin.sort(binCmp)

  # Super-triangle connectivity
  mesh.tri = @[[nVertex, nVertex + 1, nVertex + 2]]
  mesh.adj = @[[-1, -1, -1]]
  mesh.vertToTri = @[]

proc findEnclosingTriangle(mesh: MeshData, target: Vec2f, indTriCur: int): (int, int) =
  let vert = mesh.scaledVert
  var indTriCur = indTriCur
  for nhops in 0..<max(10, mesh.adj.len):
    if indTriCur == -1: # target is outside triangulation
      return (indTriCur, nhops)

    let triCur = mesh.tri[indTriCur]
    # Orientation of target wrt each edge of triangle (positive if on left of edge)
    let orients = [
      orient2d(vert[triCur[1]], vert[triCur[2]], target),
      orient2d(vert[triCur[2]], vert[triCur[0]], target),
      orient2d(vert[triCur[0]], vert[triCur[1]], target),
    ]
    
    if orients[0] <= orCollinear and orients[1] <= orCollinear and orients[2] <= orCollinear:
      # target is to left of all edges, so inside tri
      return (indTriCur, nhops)

    var baseInd = -1
    for iedge, orient in orients:
      if orient <= orCollinear:
        baseInd = iedge
        break
    let baseP1Ind = (baseInd + 1) mod 3
    let baseP2Ind = (baseInd + 2) mod 3

    if orients[baseP1Ind] <= orCollinear and orients[baseP2Ind] > orCollinear:
      # should move to the triangle opposite base_p2_ind
      indTriCur = mesh.adj[indTriCur][baseP2Ind]
    elif orients[baseP1Ind] > orCollinear and orients[baseP2Ind] <= orCollinear:
      # should move to the triangle opposite base_p1_ind
      indTriCur = mesh.adj[indTriCur][baseP1Ind]
    else:
      let vec0 = vert[triCur[baseP1Ind]] - vert[triCur[baseInd]]
      let vec1 = target - vert[triCur[baseInd]]
      if vec0 * vec1 > 0.0:
        indTriCur = mesh.adj[indTriCur][baseP2Ind]
      else:
        indTriCur = mesh.adj[indTriCur][baseP1Ind]
  raiseAssert("Failed to locate triangle containing vertex")

# Swaps the diagonal of adjacent triangles A and B
proc swapDiagonal(mesh: MeshData, indTriA, indTriB: int) =
  # Find the node index of the outer vertex in each triangle
  let outernodeTriA = mesh.adj[indTriA].find(indTriB)
  let outernodeTriB = mesh.adj[indTriB].find(indTriA)

  # Indices of nodes after the outernode (i.e. nodes of the common edge)
  let outernodeTriAp1 = (outernodeTriA + 1) mod 3
  let outernodeTriAp2 = (outernodeTriA + 2) mod 3
  let outernodeTriBp1 = (outernodeTriB + 1) mod 3
  let outernodeTriBp2 = (outernodeTriB + 2) mod 3

  # Update triangle nodes
  mesh.tri[indTriA][outernodeTriAp2] = mesh.tri[indTriB][outernodeTriB]
  mesh.tri[indTriB][outernodeTriBp2] = mesh.tri[indTriA][outernodeTriA]

  # Update adjacencies for triangle opposite outernode
  mesh.adj[indTriA][outernodeTriA] = mesh.adj[indTriB][outernodeTriBp1]
  mesh.adj[indTriB][outernodeTriB] = mesh.adj[indTriA][outernodeTriAp1]

  # Update adjacency of neighbor opposite triangle A's (outernode+1) node
  let indTriAneighOuterP1 = mesh.adj[indTriA][outernodeTriAp1]
  if indTriAneighOuterP1 >= 0:
    let neighNode = mesh.adj[indTriAneighOuterP1].find(indTriA)
    mesh.adj[indTriAneighOuterP1][neighNode] = indTriB
  
  # Update adjacency of neighbor opposite triangle B's (outernode+1) node
  let indTriBneighOuterP1 = mesh.adj[indTriB][outernodeTriBp1]
  if indTriBneighOuterP1 >= 0:
    let neighNode = mesh.adj[indTriBneighOuterP1].find(indTriB)
    mesh.adj[indTriBneighOuterP1][neighNode] = indTriA

  # Update adjacencies for triangles opposite the (outernode+1) node
  mesh.adj[indTriA][outernodeTriAp1] = indTriB
  mesh.adj[indTriB][outernodeTriBp1] = indTriA

  # Update vertex to triangle connectivity, if data structure exists
  if mesh.vertToTri.len > 0:
    # The original outernodes will now be part of both triangles
    mesh.vertToTri[mesh.tri[indTriA][outernodeTriA]].add(indTriB)
    mesh.vertToTri[mesh.tri[indTriB][outernodeTriB]].add(indTriA)

    # Remove triangle B from the triangle set of outernode_triA_p1
    var localInd = mesh.vertToTri[mesh.tri[indTriA][outernodeTriAp1]].find(indTriB)
    mesh.vertToTri[mesh.tri[indTriA][outernodeTriAp1]].del(localInd)

    # Remove triangle A from the triangle set of outernode_triB_p1
    localInd = mesh.vertToTri[mesh.tri[indTriB][outernodeTriBp1]].find(indTriA)
    mesh.vertToTri[mesh.tri[indTriB][outernodeTriBp1]].del(localInd)

proc restoreDelaunay(mesh: MeshData, indVert: int, stack: var seq[Pair]) =
  let vert = mesh.scaledVert
  var vNew = vert[indVert]
  while stack.len > 0:
    let (indTri, outernodeTri) = stack.pop()
    let indTriVert = mesh.tri[indTri]
    let vTri = [vert[indTriVert[0]], vert[indTriVert[1]], vert[indTriVert[2]]]
    if not isDelaunay(vTri, vNew):
      let indTriNeigh = mesh.adj[indTri][outernodeTri]
      assert indTriNeigh >= 0, "Negative index"

      # Swap the diagonal between the adjacent triangles
      mesh.swapDiagonal(indTri, indTriNeigh)

      # Add the triangles opposite the new vertex to the stack
      let newNodeIndTri = mesh.tri[indTri].find(indVert)
      let indTriOuterP2 = mesh.adj[indTri][newNodeIndTri]
      if indTriOuterP2 >= 0:
        let neighNode = mesh.adj[indTriOuterP2].find(indTri)
        stack.add((indTriOuterP2, neighNode))

      let newNodeIndTriNeigh = mesh.tri[indTriNeigh].find(indVert)
      let indTriNeighOuter = mesh.adj[indTriNeigh][newNodeIndTriNeigh]
      if indTriNeighOuter >= 0:
        let neighNode = mesh.adj[indTriNeighOuter].find(indTriNeigh)
        stack.add((indTriNeighOuter, neighNode))

proc removeBoundaryTriangles(mesh: MeshData) =
  let verts = mesh.scaledVert
  let n = verts.len - 3
  var delCount = 0
  var indMap = newSeq[int](mesh.tri.len)
  for i in 0..<mesh.tri.len:
    let prevDelCount = delCount
    for j in i..<mesh.tri.len:
      if mesh.tri[j][0] < n and mesh.tri[j][1] < n and mesh.tri[j][2] < n:
        indMap[i + delCount] = i
        break
      else:
        indMap[i + delCount] = -1
        delCount += 1
    let delLength = delCount - prevDelCount
    if delLength > 0:
      mesh.tri.delete(i, i + delLength - 1)
      mesh.adj.delete(i, i + delLength - 1)
  
  # Update adjacencies
  for i in 0..<mesh.adj.len:
    for j in 0..2:
      mesh.adj[i][j] = indmap[mesh.adj[i][j]]
  
  # Delete super-triangle nodes
  mesh.scaledVert.setLen(n)
  mesh.vert.setLen(n)

# Function for computing the unconstrained Delaunay triangulation
proc delaunay*(mesh: MeshData) =
  # Sort input vertices and setup super-triangle
  mesh.setupDelaunay()

  let verts = mesh.scaledVert
  let n = verts.len - 3 # vertices includes super-triangle nodes

  var indTri = 0 # points to the super-triangle
  var nhopsTotal = 0
  for i in 0..<n:
    let newI = mesh.bin[i].ind
    let res = mesh.findEnclosingTriangle(verts[newI], indTri)
    indTri = res[0]
    nhopsTotal += res[1]
    assert indTri != -1, "Could not find a triangle containing the new vertex!"

    var curTri = mesh.tri[indTri]
    var newTri0 = [curTri[0], curTri[1], newI]
    var newTri1 = [newI, curTri[1], curTri[2]]
    var newTri2 = [curTri[0], newI, curTri[2]]

    # Replace the triangle containing the point with new_tri0, and
    # fix its adjacency
    mesh.tri[indTri] = newTri0

    let nTri = mesh.tri.len
    let curTriAdj = mesh.adj[indTri]
    mesh.adj[indTri] = [nTri, nTri + 1, curTriAdj[2]]

    # Add the other two new triangles to the list
    mesh.tri.add(newTri1)
    mesh.tri.add(newTri2)

    mesh.adj.add([curTriAdj[0], nTri + 1, indTri])
    mesh.adj.add([nTri, curTriAdj[1], indTri])

    # stack of triangles which need to be checked for Delaunay condition
    # each element contains: [index of tri to check, adjncy index to goto triangle that contains new point]
    var stack = newSeq[Pair]()
    if curTriAdj[2] >= 0:
      let neighAdjInd = mesh.adj[curTriAdj[2]].find(indTri)
      stack.add((curTriAdj[2], neighAdjInd))
    if curTriAdj[0] >= 0:
      let neighAdjInd = mesh.adj[curTriAdj[0]].find(indTri)
      mesh.adj[curTriAdj[0]][neighAdjInd] = nTri
      stack.add((curTriAdj[0], neighAdjInd))
    if curTriAdj[1] >= 0:
      let neighAdjInd = mesh.adj[curTriAdj[1]].find(indTri)
      mesh.adj[curTriAdj[1]][neighAdjInd] = nTri + 1
      stack.add((curTriAdj[1], neighAdjInd))
    
    mesh.restoreDelaunay(newI, stack)
  mesh.removeBoundaryTriangles()

proc buildVertexConnectivity(mesh: MeshData) =
  mesh.vertToTri = newSeq[seq[int]](mesh.vert.len)
  for i in 0..<mesh.tri.len:
    for node in 0..2:
      mesh.vertToTri[mesh.tri[i][node]].add(i)

proc getEdgeIntersections(mesh: MeshData, iedge: int): seq[Pair] =
  let verts = mesh.scaledVert
  let edgeV0Ind = mesh.conEdge[iedge][0]
  let edgeV1Ind = mesh.conEdge[iedge][1]
  let edgeCoords = (verts[edgeV0Ind], verts[edgeV1Ind])

  let triAroundV0 = mesh.vertToTri[edgeV0Ind]

  var edgeInTriangulation = false

  # stores the index of tri that intersects current edge,
  # and the edge-index of intersecting edge in triangle
  var intersections = newSeq[Pair]()
  for tri in triAroundV0:
    let curTri = mesh.tri[tri]
    let v0node = curTri.find(edgeV0Ind)
    let v0p1node = (v0node + 1) mod 3
    let v0p2node = (v0node + 2) mod 3

    if edgeV1Ind == curTri[v0p1node] or edgeV1Ind == curTri[v0p2node]:
      # constrained edge is an edge of the current tri (node v0_node to v0_node+1)
      # constrained edge is an edge of the current tri (node v0_node to v0_node+2)
      edgeInTriangulation = true
      break

    let oppositeEdgeCoords = (verts[curTri[v0p1node]], verts[curTri[v0p2node]])
    if isEdgeIntersecting(edgeCoords, oppositeEdgeCoords):
      intersections.add((tri, v0node))
      break
  
  if not edgeInTriangulation:
    assert intersections.len > 0, "Cannot have no intersections!"

    while true:
      let prevIntersection = intersections[^1]
      let triInd = mesh.adj[prevIntersection[0]][prevIntersection[1]]

      if mesh.tri[triInd][0] == edgeV1Ind or mesh.tri[triInd][1] == edgeV1Ind or mesh.tri[triInd][2] == edgeV1Ind:
        break # found the end node of the edge

      # Find the index of the edge from which we came into this triangle
      let prevEdgeInd = mesh.adj[triInd].find(prevIntersection[0])
      assert prevEdgeInd > -1, "Could not find edge!"

      let curTri = mesh.tri[triInd]

      # Loop over the other two edges in this triangle,
      # and check if they intersect the constrained edge
      for offset in 1..2:
        let v0node = (prevEdgeInd + offset + 1) mod 3
        let v1node = (prevEdgeInd + offset + 2) mod 3
        let curEdgeCoords = (verts[curTri[v0node]], verts[curTri[v1node]])
        if isEdgeIntersecting(edgeCoords, curEdgeCoords):
          intersections.add((triInd, (prevEdgeInd + offset) mod 3))
          break
  return intersections

proc fixEdgeIntersections(mesh: MeshData, intersectionList: seq[Pair], conEdgeInd: int, newEdgeList: var seq[Pair]) =
  let verts = mesh.scaledVert

  # Node indices and endpoint coords of current constrained edge
  let conEdgeNodes = mesh.conEdge[conEdgeInd]
  let curConEdgeCoords = (verts[conEdgeNodes[0]], verts[conEdgeNodes[1]])

  for i in countdown(intersectionList.len - 1, 0):
    # Looping in reverse order is important since then the
    # indices in intersectionList remain unaffected by any diagonal swaps
    let tri0Ind = intersectionList[i][0]
    let tri0Node = intersectionList[i][1]
    let tri1Ind = mesh.adj[tri0Ind][tri0Node]
    let tri1Node = mesh.adj[tri1Ind].find(tri0Ind)
    let quadV0 = verts[mesh.tri[tri0Ind][tri0Node]]
    let quadV1 = verts[mesh.tri[tri0Ind][(tri0Node + 1) mod 3]]
    let quadV2 = verts[mesh.tri[tri1Ind][tri1Node]]
    let quadV3 = verts[mesh.tri[tri0Ind][(tri0Node + 2) mod 3]]
    let isConvex = isQuadConvex(quadV0, quadV1, quadV2, quadV3)
    if isConvex:
      mesh.swapDiagonal(tri0Ind, tri1Ind)

      let newDiagonalNodes = (mesh.tri[tri0Ind][tri0Node], mesh.tri[tri1Ind][tri1Node])
      let newDiagonalCoords = (quadV0, quadV2)
      let hasCommonNode = newDiagonalNodes[0] == conEdgeNodes[0] or newDiagonalNodes[0] == conEdgeNodes[1] or newDiagonalNodes[1] == conEdgeNodes[0] or newDiagonalNodes[1] == conEdgeNodes[1]
      if hasCommonNode or not isEdgeIntersecting(curConEdgeCoords, newDiagonalCoords):
        newEdgeList.add(newDiagonalNodes)

proc constrainEdges*(mesh: MeshData) =
  if mesh.conEdge.len == 0:
    return
  mesh.buildVertexConnectivity()

  let verts = mesh.scaledVert
  var newEdgeList = newSeq[Pair]()
  for edge in 0..<mesh.conEdge.len:
    var intersections = mesh.getEdgeIntersections(edge)
    let maxIter = max(1, intersections.len)
    for iter in 0..<maxIter:
      if intersections.len == 0:
        break
      mesh.fixEdgeIntersections(intersections, edge, newEdgeList)
      intersections = mesh.getEdgeIntersections(edge)
    assert intersections.len == 0, "Could not add edge " & $edge & " to triangulation after " & $maxIter & " iterations!"

  # Restore Delaunay
  while true:
    var numDiagonalSwaps = 0
    for i in 0..<newEdgeList.len:
      let newEdgeNodes = newEdgeList[i]

      # Check if the new edge is a constrained edge
      var isConEdge = false
      for edge in mesh.conEdge: # TODO: this is inefficient
        if isSameEdge(newEdgeNodes, edge):
          isConEdge = true
          break
      
      if isConEdge: # cannot change this edge if it's constrained
        continue

      let triAroundV0 = mesh.vertToTri[newEdgeNodes[0]]
      var triCount = 0
      var triIndPair = [-1, -1]
      for tri in triAroundV0:
        let curTri = mesh.tri[tri]
        if curTri[0] == newEdgeNodes[1] or curTri[1] == newEdgeNodes[1] or curTri[2] == newEdgeNodes[1]:
          triIndPair[triCount] = tri
          triCount += 1

          if triCount == 2:
            break # found both neighboring triangles
      
      if triIndPair[0] == -1:
        continue # this edge no longer exists, so nothing to do.

      let triAverts = [verts[mesh.tri[triIndPair[0]][0]], verts[mesh.tri[triIndPair[0]][1]], verts[mesh.tri[triIndPair[0]][2]]]
      let outerNodeBInd = mesh.adj[triIndPair[1]].find(triIndPair[0])
      let triBvert = verts[mesh.tri[triIndPair[1]][outerNodeBInd]]

      if not isDelaunay(triAverts, triBvert):
        let outerNodeAInd = mesh.adj[triIndPair[0]].find(triIndPair[1])

        # Swap the diagonal between the pair of triangles
        mesh.swapDiagonal(triIndPair[0], triIndPair[1])
        numDiagonalSwaps += 1

        # Replace current new edge with the new diagonal
        newEdgeList[i] = (mesh.tri[triIndPair[0]][outerNodeAInd], mesh.tri[triIndPair[1]][outerNodeBInd])
    if numDiagonalSwaps == 0:
      break # no further swaps, we're done.

proc triangulate*(mesh: MeshData) =
  mesh.delaunay()
  mesh.constrainEdges()