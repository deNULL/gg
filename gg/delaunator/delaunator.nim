import gg/vectors
import algorithm
import math

type
  HullIndices = tuple[prev: int, next: int, tri: int]
  IdDist = tuple[id: int, dist: float]
  Delaunator* = ref object
    coords*: seq[Vec2f]
    triangles*: seq[int]
    halfedges*: seq[int]
    hashSize: int
    hull*: seq[int]
    hullStart: int
    hullIndices: seq[HullIndices]
    hullHash: seq[int]
    dists: seq[IdDist]
    center: Vec2f

proc link(dl: Delaunator, a, b: int) =
  if a >= dl.halfedges.len:
    dl.halfedges.setLen(a + 1)
  dl.halfedges[a] = b
  if b != -1: dl.halfedges[b] = a

proc cmpDist(a, b: IdDist): int = cmp(a.dist, b.dist)

proc addTriangle(dl: Delaunator, i0, i1, i2, a, b, c: int): int =
  result = dl.triangles.len
  dl.triangles.add([i0, i1, i2])
  dl.link(result, a)
  dl.link(result + 1, b)
  dl.link(result + 2, c)

proc hashKey(dl: Delaunator, p: Vec2f): int =
  int((p - dl.center).pseudoAngle * dl.hashSize.float) mod dl.hashSize

var EdgeStack: array[512, int]
proc legalize(dl: Delaunator, a: int): int =
  var a = a
  var i = 0
  var ar = 0

  # recursion eliminated with a fixed-size stack
  while true:
    let b = dl.halfedges[a]
    #[
    if the pair of triangles doesn't satisfy the Delaunay condition
    (p1 is inside the circumcircle of [p0, pl, pr]), flip them,
    then do the same check/flip recursively for the new pair of triangles
    
              pl                    pl
             /||\                  /  \
          al/ || \bl            al/    \a
           /  ||  \              /      \
          /  a||b  \    flip    /___ar___\
        p0\   ||   /p1   =>   p0\---bl---/p1
           \  ||  /              \      /
          ar\ || /br             b\    /br
             \||/                  \  /
              pr                    pr
    ]#
    let a0 = a - a mod 3
    ar = a0 + (a + 2) mod 3

    if b == -1: # convex hull edge
      if i == 0:
        break
      i -= 1
      a = EdgeStack[i]
      continue

    let b0 = b - b mod 3
    let al = a0 + (a + 1) mod 3
    let bl = b0 + (b + 2) mod 3

    let p0 = dl.triangles[ar]
    let pr = dl.triangles[a]
    let pl = dl.triangles[al]
    let p1 = dl.triangles[bl]

    let illegal = incircle(dl.coords[p0], dl.coords[pr], dl.coords[pl], dl.coords[p1])
    if illegal:
      dl.triangles[a] = p1
      dl.triangles[b] = p0

      let hbl = dl.halfedges[bl]

      # edge swapped on the other side of the hull (rare); fix the halfedge reference
      if hbl == -1:
        var e = dl.hullStart
        while true:
          if dl.hullIndices[e].tri == bl:
            dl.hullIndices[e].tri = a
            break
          e = dl.hullIndices[e].prev
          if e == dl.hullStart:
            break
      dl.link(a, hbl)
      dl.link(b, dl.halfedges[ar])
      dl.link(ar, bl)

      let br = b0 + (b + 1) mod 3

      # don't worry about hitting the cap: it can only happen on extremely degenerate input
      if i < EdgeStack.len:
        EdgeStack[i] = br
        i += 1
    else:
      if i == 0: break
      i -= 1
      a = EdgeStack[i]
  return ar

proc update(dl: Delaunator) =
  let bbox = dl.coords.bbox

  # pick a seed point close to the center
  var minDist = Inf
  var i0 = 0
  for i, p in dl.coords:
    let d = bbox.center.distSq(p)
    if d < minDist:
      i0 = i
      minDist = d
  let p0 = dl.coords[i0]
  
  # find the point closest to the seed
  minDist = Inf
  var i1 = 0
  for i, p in dl.coords:
    if i == i0: continue
    let d = p0.distSq(p)
    if d < minDist and d > 0:
      i1 = i
      minDist = d
  var p1 = dl.coords[i1]

  # find the third point which forms the smallest circumcircle with the first two
  var minRadius = Inf
  var i2 = 0
  for i, p in dl.coords:
    if i == i0 or i == i1: continue
    let r = circumradius(p0, p1, p)
    if r < minRadius:
      i2 = i
      minRadius = r
  var p2 = dl.coords[i2]

  if minRadius == Inf:
    # order collinear points by dx (or dy if all x are identical)
    # and return the list as a hull
    let first = dl.coords[0]
    for i, p in dl.coords:
      dl.dists[i] = (id: i, dist: if p.x == first.x: p.y - first.y else: p.x - first.x)
    dl.dists.sort(cmpDist)

    dl.hull = newSeqOfCap[int](dl.coords.len)
    var d0 = -Inf
    for (id, dist) in dl.dists:
      if dist > d0:
        dl.hull.add(id)
        d0 = dist
    return

  # swap the order of the seed points for counter-clockwise orientation
  if orient2d(p0, p1, p2) < orCollinear:
    swap(i1, i2)
    swap(p1, p2)

  dl.center = circumcenter(p0, p1, p2)
  for i, p in dl.coords:
    dl.dists[i] = (id: i, dist: dl.center.distSq(p))
  
  # sort the points by distance from the seed triangle circumcenter
  dl.dists.sort(cmpDist)

  # set up the seed triangle as the starting hull
  dl.hullStart = i0
  var hullSize = 3
  dl.hullIndices[i0] = (prev: i2, next: i1, tri: 0)
  dl.hullIndices[i1] = (prev: i0, next: i2, tri: 1)
  dl.hullIndices[i2] = (prev: i1, next: i0, tri: 2)

  for i in 0..<dl.hashSize:
    dl.hullHash[i] = -1
  dl.hullHash[dl.hashKey(p0)] = i0
  dl.hullHash[dl.hashKey(p1)] = i1
  dl.hullHash[dl.hashKey(p2)] = i2

  discard dl.addTriangle(i0, i1, i2, -1, -1, -1)

  var prev: Vec2f
  for k, (id, dist) in dl.dists:
    let p = dl.coords[id]

    # skip near-duplicate points
    if k > 0 and p ~= prev:
      continue
    prev = p

    # skip seed triangle points
    if id == i0 or id == i1 or id == i2: continue

    # find a visible edge on the convex hull using edge hash
    var start = 0
    let key = dl.hashKey(p)
    for j in 0..<dl.hashSize:
      start = dl.hullHash[(key + j) mod dl.hashSize]
      if start != -1 and start != dl.hullIndices[start].next:
        break

    start = dl.hullIndices[start].prev
    var e = start
    var q = dl.hullIndices[e].next
    while orient2d(p, dl.coords[e], dl.coords[q]) != orCCW:
      e = q
      if e == start:
        e = -1
        break
      q = dl.hullIndices[e].next
    if e == -1: # likely a near-duplicate point; skip it
      continue 

    # add the first triangle from the point
    var t = dl.addTriangle(e, id, dl.hullIndices[e].next, -1, -1, dl.hullIndices[e].tri)

    # recursively flip triangles from the point until they satisfy the Delaunay condition
    dl.hullIndices[id].tri = dl.legalize(t + 2)
    dl.hullIndices[e].tri = t # keep track of boundary triangles on the hull
    hullSize += 1

    # walk forward through the hull, adding more triangles and flipping recursively
    var n = dl.hullIndices[e].next
    q = dl.hullIndices[n].next
    while orient2d(p, dl.coords[n], dl.coords[q]) == orCCW:
      t = dl.addTriangle(n, id, q, dl.hullIndices[id].tri, -1, dl.hullIndices[n].tri)
      dl.hullIndices[id].tri = dl.legalize(t + 2)
      dl.hullIndices[n].next = n # mark as removed
      hullSize -= 1
      n = q
      q = dl.hullIndices[n].next

    # walk backward from the other side, adding more triangles and flipping
    if e == start:
      q = dl.hullIndices[e].prev
      while orient2d(p, dl.coords[q], dl.coords[e]) == orCCW:
        t = dl.addTriangle(q, id, e, -1, dl.hullIndices[e].tri, dl.hullIndices[q].tri)
        discard dl.legalize(t + 2)
        dl.hullIndices[q].tri = t
        dl.hullIndices[e].next = e # mark as removed
        hullSize -= 1
        e = q
        q = dl.hullIndices[e].prev
    
    # update the hull indices
    dl.hullStart = e
    dl.hullIndices[id].prev = e
    dl.hullIndices[id].next = n
    dl.hullIndices[e].next = id
    dl.hullIndices[n].prev = id

    # save the two new edges in the hash table
    dl.hullHash[dl.hashKey(p)] = id
    dl.hullHash[dl.hashKey(dl.coords[e])] = e

  dl.hull = newSeq[int](hullSize)
  var e = dl.hullStart
  for i in 0..<hullSize:
    dl.hull[i] = e
    e = dl.hullIndices[e].next

proc newDelaunator*(coords: seq[Vec2f]): Delaunator =
  let maxTriangles = max(2 * coords.len - 5, 0)
  let hashSize = int(ceil(sqrt(float(coords.len))))
  result = Delaunator(
    coords: coords,
    # arrays that will store the triangulation graph
    triangles: newSeqOfCap[int](maxTriangles * 3),
    halfedges: newSeqOfCap[int](maxTriangles * 3),

    # temporary arrays for tracking the edges of the advancing convex hull
    hashSize: hashSize,
    hullIndices: newSeq[HullIndices](coords.len),
    hullHash: newSeq[int](hashSize),   # angular edge hash

    # temporary arrays for sorting points
    dists: newSeq[IdDist](coords.len),
  )
  result.triangles.fill(-1)
  result.halfedges.fill(-1)
  result.update()