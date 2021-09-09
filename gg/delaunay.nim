import gg/[vectors, randomvec]
import gg/delaunator/[delaunator, constrainautor]
import random
import algorithm

export delaunator, constrainautor

type
  TriangleState* = enum
    tsInside, tsOutside, tsHole
  EdgeState = enum
    esNone, esContour, esHole

  # How node positions should be calculated
  NodePositionStrategy* = proc(a, b, c: Vec2f): Vec2f

proc triangleCnt*(dl: Delaunator): int {.inline.} = dl.triangles.len div 3

proc pointsOfTriangle*(dl: Delaunator, t: int): (int, int, int) =
  let i = 3 * t
  (dl.triangles[i], dl.triangles[i + 1], dl.triangles[i + 2])

proc pointsOfEdge*(dl: Delaunator, e: int): (int, int) =
  (dl.triangles[e], dl.triangles[nextEdge(e)])

proc triangleCenter*(dl: Delaunator, t: int): Vec2f =
  let (i0, i1, i2) = dl.pointsOfTriangle(t)
  circumcenter(dl.coords[i0], dl.coords[i1], dl.coords[i2])

# Returns closest point in triangle
proc closestInside*(p: Vec2f, a, b, c: Vec2f): Vec2f =
  var minDist = Inf
  for segm in segments(a, b, c):
    let p1 = p.proj(segm)
    let d = p.distSq(p1)
    if d < minDist:
      minDist = d
      result = p1
    
let npsCircumcenter*: NodePositionStrategy = proc(a, b, c: Vec2f): Vec2f = circumcenter(a, b, c)
let npsCentroid*: NodePositionStrategy = proc(a, b, c: Vec2f): Vec2f = centroid(a, b, c)
let npsIncenter*: NodePositionStrategy = proc(a, b, c: Vec2f): Vec2f = incenter(a, b, c)
let npsCircumcenterOrCentroid*: NodePositionStrategy = proc(a, b, c: Vec2f): Vec2f = 
  result = circumcenter(a, b, c)
  if not result.inside(a, b, c) and not result.inside(a, c, b):
    return centroid(a, b, c)
let npsCircumcenterOrIncenter*: NodePositionStrategy = proc(a, b, c: Vec2f): Vec2f =
  result = circumcenter(a, b, c)
  if not result.inside(a, b, c) and not result.inside(a, c, b):
    return incenter(a, b, c)
let npsCircumcenterOrClosest*: NodePositionStrategy = proc(a, b, c: Vec2f): Vec2f =
  result = circumcenter(a, b, c)
  if not result.inside(a, b, c) and not result.inside(a, c, b):
    return closestInside(result, a, b, c)
  
let npsAlwaysClosestEdge*: NodePositionStrategy = proc(a, b, c: Vec2f): Vec2f =
  closestInside(circumcenter(a, b, c), a, b, c)

let npsStrategies* = [npsCircumcenter, npsCentroid, npsIncenter, npsCircumcenterOrCentroid,
  npsCircumcenterOrIncenter, npsCircumcenterOrClosest, npsAlwaysClosestEdge]

proc npsCircumcenterOrMix*(t: fnum): NodePositionStrategy =
  return proc(a, b, c: Vec2f): Vec2f =
    result = circumcenter(a, b, c)
    if not result.inside(a, b, c) and not result.inside(a, c, b):
      return mix(closestInside(result, a, b, c), centroid(a, b, c), t)
proc npsRandom*(): NodePositionStrategy =
  var rand = initRand(111)
  return proc(a, b, c: Vec2f): Vec2f = rand.rand(a, b, c)
proc npsRandomEdge*(): NodePositionStrategy =
  var rand = initRand(111)
  return proc(a, b, c: Vec2f): Vec2f =
    case rand.rand(0..2):
    of 0: return a.mix(b, rand.rand(1.0))
    of 1: return b.mix(c, rand.rand(1.0))
    else: return c.mix(a, rand.rand(1.0))
proc npsRandomMidEdge*(): NodePositionStrategy =
  var rand = initRand(111)
  return proc(a, b, c: Vec2f): Vec2f =
    case rand.rand(0..2):
    of 0: return a.avg(b)
    of 1: return b.avg(c)
    else: return c.avg(a)

proc triangleCenterInside*(dl: Delaunator, t: int, strategy: NodePositionStrategy): Vec2f =
  let (i0, i1, i2) = dl.pointsOfTriangle(t)
  return strategy(dl.coords[i0], dl.coords[i1], dl.coords[i2])

proc edgeCenter*(dl: Delaunator, e: int): Vec2f =
  let (p0, p1) = dl.pointsOfEdge(e)
  (dl.coords[p0] + dl.coords[p1]) / 2.0

iterator triangles*(dl: Delaunator): (int, (int, int, int)) =
  for t in 0..<dl.triangleCnt:
    yield (t, dl.pointsOfTriangle(t))

# iterates around a point which halfedge start points to
# yield pairs of incoming and outgoing halfedges (either one can be a -1 if that edge is a part of the hull)
iterator edgesAroundPoint*(dl: Delaunator, start: int): (int, int) =
  var incoming = start
  yield (incoming, dl.halfedges[incoming])
  while incoming != -1:
    let outgoing = nextEdge(incoming)
    incoming = dl.halfedges[outgoing]
    if incoming == start:
      break
    else:
      yield (incoming, outgoing)

proc findEdge*(cons: Constrainautor, p0, p1: int): (int, int) =
  let del = cons.del
  for (incoming, outgoing) in del.edgesAroundPoint(cons.vertMap[p0]):
    if incoming != -1 and del.triangles[incoming] == p1:
      return (incoming, outgoing)
  for (incoming, outgoing) in cons.del.edgesAroundPoint(cons.vertMap[p1]):
    if incoming != -1 and del.triangles[incoming] == p0:
      return (outgoing, incoming)
  return (-1, -1)

proc markPolygon(cons: Constrainautor, polygon: seq[int], edges: var seq[EdgeState], state: EdgeState) =
  for i, p0 in polygon:
    let p1 = polygon[(i + 1) mod polygon.len]
    let (incoming, outgoing) = cons.findEdge(p0, p1)
    doAssert incoming != -1 or outgoing != -1, "Edge (" & $p0 & "-" & $p1 & ") is missing (not constrained?)"
    if incoming != -1:
      edges[incoming] = state
    if outgoing != -1:
      edges[outgoing] = state

# Detect triangles from concavities and holes (by default Delaunator produces convex hull)
proc classifyTriangles*(cons: Constrainautor, contour: seq[int], holes: seq[seq[int]] = @[]): seq[TriangleState] =
  let del = cons.del
  result = newSeq[TriangleState](del.triangles.len div 3)
  result.fill(tsHole)
  var edges = newSeq[EdgeState](del.triangles.len)
  cons.markPolygon(contour, edges, esContour)
  for hole in holes:
    cons.markPolygon(hole, edges, esHole)

  var queue = newSeq[int]()
  var iqueue = newSeq[int]()

  # Find outer edges, initialise queues
  for e0, e1 in del.halfedges:
    if e1 == -1:
      if edges[e0] == esContour:
        result[e0 div 3] = tsInside
        iqueue.add(e0)
      else:
        result[e0 div 3] = tsOutside
        queue.add(e0)

  # Fill outside
  while queue.len > 0:
    let e0 = queue.pop()
    for e in [e0.nextEdge(), e0.prevEdge()]:
      let other = del.halfedges[e]
      if other != -1:
        let t = other div 3
        if result[t] == tsHole:
          if edges[other] == esNone:
            result[t] = tsOutside
            queue.add(other)
          elif edges[other] == esContour:
            result[t] = tsInside
            iqueue.add(other)

  # Fill insides, leaving holes unfilled
  while iqueue.len > 0:
    let e0 = iqueue.pop()
    for e in [e0.nextEdge(), e0.prevEdge()]:
      let other = del.halfedges[e]
      if other != -1:
        let t = other div 3
        if result[t] == tsHole and edges[other] == esNone:
          result[t] = tsInside
          iqueue.add(other)

proc deleteExternalTriangles*(cons: Constrainautor, contour: seq[int], holes: seq[seq[int]] = @[]) =
  let del = cons.del
  let classes = cons.classifyTriangles(contour, holes)

  var newIndices = newSeq[int](classes.len)
  var j = 0
  for i, ts in classes:
    if ts == tsInside:
      newIndices[i] = j
      j += 1
    else:
      newIndices[i] = -1
  
  for i in 0..<cons.vertMap.len:
    let e0 = cons.vertMap[i]
    cons.vertMap[i] = -1

    var wasSet = false
    var hasGapes = false
    # Try to find first non-deleted edge (maybe the same one)
    for (e, outgoing) in del.edgesAroundPoint(e0):
      if e != -1:
        let t = newIndices[e div 3]
        if t == -1: # Found newly formed gape, iterate until we find first non-deleted edge
          hasGapes = true
          wasSet = false
        elif not wasSet:
          cons.vertMap[i] = t * 3 + (e mod 3)
          wasSet = true
          if hasGapes: # We can stop only if we already passed a gape
            break

  # FIXME: If contour visits same point twice, deleting external triangles can lead to multiple
  # gaps around single point. This would make iterating around them impossible.

  for i0 in 0..<del.triangles.len:
    let t0 = i0 div 3
    let t1 = newIndices[t0]
    if t1 != -1:
      let i1 = t1 * 3 + (i0 mod 3)
      assert i1 <= i0, "New index should be always less than the old one"
      del.triangles[i1] = del.triangles[i0]

      let he = del.halfedges[i0]
      if he != -1:
        let ht = newIndices[he div 3]
        if ht != -1:
          del.halfedges[i1] = ht * 3 + (he mod 3)
        else:
          del.halfedges[i1] = -1
      else:
        del.halfedges[i1] = -1

  del.triangles.setLen(j * 3)
  del.halfedges.setLen(j * 3)



#[
proc cells*(dl: Delaunator): seq[seq[int]] =
  var index = newSeq[int](dl.coords.len)
  index.fill(-1)
  for e in 0..<dl.halfedges.len:
    let endpoint = dl.triangles[nextEdge(e)]
    if index[endpoint] == -1 or dl.halfedges[e] == -1:
      index[endpoint] = e
  result = newSeq[seq[int]](dl.coords.len)
  for i, incoming in index:
    for e in dl.edgesAroundPoint(incoming):
      result[i].add(e)
]#
type
  Cell* = object
    points*: seq[Vec2f]
    edges*: seq[int]

proc cells*(cons: Constrainautor, strategy: NodePositionStrategy): seq[Cell] =
  let del = cons.del
  result = newSeq[Cell](del.coords.len)
  var centers = newSeq[Vec2f](del.triangleCnt)
  for i in 0..<del.triangleCnt:
    centers[i] = del.triangleCenterInside(i, strategy)
  for i in 0..<del.coords.len:
    let start = cons.vertMap[i]
    if start == -1:
      continue
    
    for (incoming, outgoing) in del.edgesAroundPoint(start):
      if outgoing == -1: # First point, add also the origin and then the first mid-edge
        result[i].edges.add(-1)
        result[i].points.add(del.coords[i])
        result[i].edges.add(-1)
        result[i].points.add(del.edgeCenter(incoming))
      
      result[i].edges.add(incoming)
      if incoming != -1:
        result[i].points.add(centers[incoming div 3])
      else:
        result[i].points.add(del.edgeCenter(outgoing))

#[
proc cells*(dl: Delaunator, strategy: NodePositionStrategy): seq[Cell] =
  result = newSeq[Cell](dl.coords.len)
  var visited = newSeq[bool](dl.coords.len)
  for e in 0..<dl.halfedges.len:
    let p = dl.triangles[nextEdge(e)]
    if visited[p]:
      continue
    visited[p] = true
    # Forward pass
    var incoming = e
    while incoming != -1:
      result[p].edges.add(incoming)
      result[p].points.add(dl.triangleCenterInside(incoming div 3, strategy))
      let outgoing = nextEdge(incoming)
      incoming = dl.halfedges[outgoing]
      if incoming == -1:
        result[p].edges.add(-1)
        result[p].points.add(dl.edgeCenter(outgoing))
      if incoming == e:
        break
    if incoming == -1: # We didn't loop, so we need to walk backwards too
      # Backward pass
      var edges = newSeq[int]()
      var points = newSeq[Vec2f]()
      incoming = e
      while true:
        let outgoing = dl.halfedges[incoming]
        if outgoing == -1:
          edges.add(-1)
          points.add(dl.edgeCenter(incoming))

          # Also add central point
          edges.add(-1)
          points.add(dl.coords[p])
          break
        incoming = prevEdge(outgoing)
        edges.add(incoming)
        points.add(dl.triangleCenterInside(incoming div 3, strategy))
      result[p].edges = edges.reversed & result[p].edges
      result[p].points = points.reversed & result[p].points

]#