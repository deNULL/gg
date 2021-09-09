import gg/vectors
import gg/delaunator/delaunator
import algorithm

# Constrain a triangulation from Delaunator, using (parts of) the algorithm
# in "A fast algorithm for generating constrained Delaunay triangulations" by
# S. W. Sloan.

type
  Flip = enum
    fIgnd, fConsd, fFlipd
  Constrainautor* = ref object
    del*: Delaunator
    vertMap*: seq[int]
    flips: seq[Flip]

proc nextEdge*(e: int): int {.inline.} =
  if e mod 3 == 2: e - 2 else: e + 1
proc prevEdge*(e: int): int {.inline.} =
  if e mod 3 == 0: e + 2 else: e - 1

# Whether the segment between [p1, p2] intersects with [p3, p4]. When the
# segments share an end-point (e.g. p1 == p3 etc.), they are not considered
# intersecting.
proc intersectSegments(cons: Constrainautor, p1, p2, p3, p4: int): bool =
  # If the segments share one of the end-points, they cannot intersect
  # (provided the input is properly segmented, and the triangulation is
  # correct), but intersectSegments will say that they do. We can catch
  # it here already.
  if p1 == p3 or p1 == p4 or p2 == p3 or p2 == p4:
    return false
  let pts = cons.del.coords
  intersects(pts[p1], pts[p2], pts[p3], pts[p4])

# Whether point px is in the circumcircle of the triangle formed by p1, p2,
# and p3 (which are in counter-clockwise order).
proc incircle(cons: Constrainautor, p1, p2, p3, px: int): bool =
  let pts = cons.del.coords
  incircle(pts[p1], pts[p2], pts[p3], pts[px])

# Whether point p1, p2, and p are collinear.
proc isCollinear(cons: Constrainautor, p1, p2, p: int): bool =
  let pts = cons.del.coords
  orient2d(pts[p1], pts[p2], pts[p]) == orCollinear

# Whether an edge is a constrained edge.
proc isConstrained*(cons: Constrainautor, edg: int): bool = cons.flips[edg] == fConsd

# Whether the two triangles sharing edg conform to the Delaunay condition.
# As a shortcut, if the given edge has no adjacent (is on the hull), it is
# certainly Delaunay.
proc isDelaunay(cons: Constrainautor, edg: int): bool =
  let adj = cons.del.halfedges[edg]
  if adj == -1:
    return true

  let p1 = cons.del.triangles[prevEdge(edg)]
  let p2 = cons.del.triangles[edg]
  let p3 = cons.del.triangles[nextEdge(edg)]
  let px = cons.del.triangles[prevEdge(adj)]
  return not cons.inCircle(p1, p2, p3, px)

# Mark an edge as flipped, unless it is already marked as constrained.
proc markFlip(cons: Constrainautor, edg: int): bool {.discardable.} =
  if cons.flips[edg] == fConsd:
    return false
  let adj = cons.del.halfedges[edg]
  if adj != -1:
    cons.flips[adj] = fFlipd
    cons.flips[edg] = fFlipd
  return true

# Update the vertex -> incoming edge map.
proc updateVert(cons: Constrainautor, start: int): int {.discardable.} =
  let del = cons.del
  let v = del.triangles[start]

  # When iterating over incoming edges around a vertex, we do so in
  # clockwise order ('going left'). If the vertex lies on the hull, two
  # of the edges will have no opposite, leaving a gap. If the starting
  # incoming edge is not the right-most, we will miss edges between it
  # and the gap. So walk counter-clockwise until we find an edge on the
  # hull, or get back to where we started.

  var inc = prevEdge(start)
  var adj = del.halfedges[inc]
  while adj != -1 and adj != start:
    inc = prevEdge(adj)
    adj = del.halfedges[inc]

  cons.vertMap[v] = inc
  return inc

# Flip the edge shared by two triangles.
proc flipDiagonal(cons: Constrainautor, edg: int): int {.discardable.} =
  # Flip a diagonal
  #                top                     edg
  #          o  <----- o            o <------  o 
  #         | ^ \      ^           |       ^ / ^
  #      lft|  \ \     |        lft|      / /  |
  #         |   \ \adj |           |  bot/ /   |
  #         | edg\ \   |           |    / /top |
  #         |     \ \  |rgt        |   / /     |rgt
  #         v      \ v |           v  / v      |
  #         o ----->  o            o   ------> o 
  #           bot                     adj
  let adj = cons.del.halfedges[edg]
  let bot = prevEdge(edg)
  let lft = nextEdge(edg)
  let top = prevEdge(adj)
  let rgt = nextEdge(adj)
  let adjBot = cons.del.halfedges[bot]
  let adjTop = cons.del.halfedges[top]
  
  doAssert cons.flips[edg] != fConsd and cons.flips[adj] != fConsd, "Trying to flip a constrained edge"

  cons.del.triangles[edg] = cons.del.triangles[top]
  cons.del.halfedges[edg] = adjTop
  cons.flips[edg] = cons.flips[top]
  if adjTop != -1:
    cons.del.halfedges[adjTop] = edg
  cons.del.halfedges[bot] = top

  cons.del.triangles[adj] = cons.del.triangles[bot]
  cons.del.halfedges[adj] = adjBot
  cons.flips[adj] = cons.flips[bot]
  if adjBot != -1:
    cons.del.halfedges[adjBot] = adj
  cons.del.halfedges[top] = bot

  cons.markFlip(edg)
  cons.markFlip(lft)
  cons.markFlip(adj)
  cons.markFlip(rgt)
  cons.flips[bot] = fFlipd
  cons.flips[top] = fFlipd

  cons.updateVert(edg)
  cons.updateVert(lft)
  cons.updateVert(adj)
  cons.updateVert(rgt)

  return bot

# Fix the Delaunay condition after constraining edges.
proc delaunify*(cons: Constrainautor, deep: bool = false) =
  while true:
    var flipped = 0 # actual valid use of var: scoped outside the loop
    for edg in 0..<cons.flips.len:
      if cons.flips[edg] != fFlipd:
        continue
      cons.flips[edg] = fIgnd
      
      let adj = cons.del.halfedges[edg]
      if adj == -1:
        continue

      cons.flips[adj] = fIgnd
      if not cons.isDelaunay(edg):
        cons.flipDiagonal(edg)
        flipped += 1
    if not deep or flipped == 0:
      break

proc newConstrainautor*(del: Delaunator): Constrainautor =
  result = Constrainautor(
    del: del,
    # Map every vertex id to the left-most edge that points to that vertex.
    vertMap: newSeq[int](del.coords.len),
    # Keep track of edges flipped while constraining
    flips: newSeq[Flip](del.triangles.len),
  )
  result.vertMap.fill(int.high)
  result.flips.fill(fIgnd)
  for e in 0..<del.triangles.len:
    let v = del.triangles[e]
    if result.vertMap[v] == int.high:
      result.updateVert(e)

# Mark an edge as constrained, i.e. should not be touched by `delaunify`.
proc protect(cons: Constrainautor, edg: int): int {.discardable.} =
  let adj = cons.del.halfedges[edg]
  cons.flips[edg] = fConsd
  if adj != -1:
    cons.flips[adj] = fConsd
    return adj
  return -edg

# Constrain the triangulation such that there is an edge between p1 and p2.
proc constrainOne*(cons: Constrainautor, segP1, segP2: int): int {.discardable.} =
  let del = cons.del
  let start = cons.vertMap[segP1]

  # Loop over the edges touching segP1
  var edg = start
  while true:
    # edg points toward segP1, so its start-point is opposite it
    let p4 = del.triangles[edg]
    let nxt = nextEdge(edg)

    # already constrained, but in reverse order
    if p4 == segP2:
      return cons.protect(edg)

    # The edge opposite segP1
    let opp = prevEdge(edg)
    let p3 = del.triangles[opp]

    # already constrained
    if p3 == segP2:
      cons.protect(nxt)
      return nxt

    # edge opposite segP1 intersects constraint
    if cons.intersectSegments(segP1, segP2, p3, p4):
      edg = opp
      break

    let adj = del.halfedges[nxt]
    # The next edge pointing to segP1
    edg = adj
    if edg == -1 or edg == start:
      break
  
  var conEdge = edg
  # Walk through the triangulation looking for further intersecting
  # edges and flip them. If an intersecting edge cannot be flipped,
  # assign its id to `rescan` and restart from there, until there are
  # no more intersects.
  var rescan = -1
  while edg != -1:
    # edg is the intersecting half-edge in the triangle we came from
    # adj is now the opposite half-edge in the adjacent triangle, which
    # is away from segP1.
    let adj = del.halfedges[edg]
    # cross diagonal
    let bot = prevEdge(edg)
    let top = prevEdge(adj)
    let rgt = nextEdge(adj)

    doAssert adj != -1, "Constraining edge exited the hull"
    doAssert cons.flips[edg] != fConsd and cons.flips[adj] != fConsd, "Edge intersects already constrained edge"
    doAssert not cons.isCollinear(segP1, segP2, del.triangles[edg]) and not cons.isCollinear(segP1, segP2, del.triangles[adj]), "Constraining edge intersects point"

    let convex = cons.intersectSegments(
      del.triangles[edg],
      del.triangles[adj],
      del.triangles[bot],
      del.triangles[top],
    )

    # The quadrilateral formed by the two triangles adjoing edg is not
    # convex, so the edge can't be flipped. Continue looking for the
    # next intersecting edge and restart at this one later.
    if not convex:
      if rescan == -1:
        rescan = edg
      
      if del.triangles[top] == segP2:
        doAssert edg != rescan, "Infinite loop: non-convex quadrilateral"
        edg = rescan
        rescan = -1
        continue

      # Look for the next intersect
      if cons.intersectSegments(segP1, segP2, del.triangles[top], del.triangles[adj]):
        edg = top
      elif cons.intersectSegments(segP1, segP2, del.triangles[rgt], del.triangles[top]):
        edg = rgt
      else:
        doAssert edg != rescan, "Infinite loop: no further intersect after non-convex"
      
      continue

    cons.flipDiagonal(edg)

    # The new edge might still intersect, which will be fixed in the
    # next rescan.
    if cons.intersectSegments(segP1, segP2, del.triangles[bot], del.triangles[top]):
      if rescan == -1:
        rescan = bot
      
      doAssert bot != rescan, "Infinite loop: flipped diagonal still intersects"
    
    # Reached the other segment end-point? Start the rescan.
    if del.triangles[top] == segP2:
      conEdge = top
      edg = rescan
      rescan = -1
    # Otherwise, for the next edge that intersects. Because we just
    # flipped, it's either edg again, or rgt.
    elif cons.intersectSEgments(segP1, segP2, del.triangles[rgt], del.triangles[top]):
      edg = rgt
  
  return cons.protect(conEdge)

# Call constrainOne on each edge, and delaunify afterwards.
proc constrainAll*(cons: Constrainautor, edges: seq[(int, int)]) =
  for e in edges:
    cons.constrainOne(e[0], e[1])
  cons.delaunify(true)

