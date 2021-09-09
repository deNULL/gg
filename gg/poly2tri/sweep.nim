import gg/vectors
import algorithm

type
  Point = ref object
    id*: int
    pos*: Vec2f
    edges: seq[Edge]

  Edge = ref object
    p, q: Point

  PointError* = ref object of ValueError
    points*: seq[Point]

proc `<`(p1, p2: Point): bool =
  if p1.pos.y != p2.pos.y:
    return p1.pos.y < p2.pos.y
  else:
    return p1.pos.x < p2.pos.x

  #[
  PointProvider = concept p
    p.pos is Vec2f

  Point = concept p
    `==`(p, p) is bool
    p is Vec2f | PointProvider
  ]#
#[
template pos(point: Point): Vec2f =
  when type(point) is Vec2f:
    elem
  else:
    elem.pos

template x(point: Point): Vec2f =
  when type(point) is Vec2f:
    elem.x
  else:
    elem.pos.x

template y(point: Point): Vec2f =
  when type(point) is Vec2f:
    elem.y
  else:
    elem.pos.y
]#

#[
  <b>Requirement</b>:<br>
  1. a,b and c form a triangle.<br>
  2. a and d is know to be on opposite side of bc<br>
  <pre>
                 a
                 +
                / \
               /   \
             b/     \c
             +-------+
            /    d    \
           /           \
  </pre>
  <b>Fact</b>: d has to be in area B to have a chance to be inside the circle formed by
   a,b and c<br>
   d is outside B if orient2d(a,b,d) or orient2d(c,a,d) is CW<br>
   This preknowledge gives us a way to optimize the incircle test
  @param pa - triangle point, opposite d
  @param pb - triangle point
  @param pc - triangle point
  @param pd - point opposite a
  @return {boolean} true if d is inside circle, false if on circle edge
]#
# Note: this is different from the usual incircle test (in vectors.nim)
proc incircle2(pa, pb, pc, pd: Vec2f): bool =
  let ad = pa - pd
  let bd = pb - pd
  let oabd = ad ^ bd
  if oabd <= 0:
    return false

  let cd = pc - pd
  let ocad = cd ^ ad
  if ocad <= 0:
    return false

  let obcd = bd ^ cd
  return ad.lenSq * obcd + bd.lenSq * ocad + cd.lenSq * oabd > 0

include triangle
include advancingfront

include sweepcontext

proc isAngleObtuse(pa, pb, pc: Vec2f): bool = (pb - pa) * (pc - pa) < 0

proc inScanArea(pa, pb, pc, pd: Vec2f): bool =
  if (pa.x - pb.x) * (pd.y - pb.y) - (pd.x - pb.x) * (pa.y - pb.y) >= -eps:
    return false
  if (pa.x - pc.x) * (pd.y - pc.y) - (pd.x - pc.x) * (pa.y - pc.y) <= eps:
    return false
  return true

proc isEdgeSideOfTriangle(triangle: Triangle, ep, eq: Point): bool =
  let index = triangle.edgeIndex(ep, eq)
  if index != -1:
    triangle.markConstrainedEdgeByIndex(index)
    var t = triangle.neighbor(index)
    if t != nil:
      t.markConstrainedEdgeByPoints(ep, eq)
    return true

# The basin angle is decided against the horizontal line [1,0].
# true if angle < 3*π/4
proc isBasinAngleRight(node: Node): bool =
  let a = node.point.pos - node.next.next.point.pos
  assert a.y >= 0, "unordered y"
  return a.x >= 0 or abs(a.x) < a.y

#[
  Rotates a triangle pair one vertex CW
  <pre>
        n2                    n2
   P +-----+             P +-----+
     | t  /|               |\  t |
     |   / |               | \   |
   n1|  /  |n3           n1|  \  |n3
     | /   |    after CW   |   \ |
     |/ oT |               | oT \|
     +-----+ oP            +-----+
        n4                    n4
  </pre>
]#
proc rotateTrianglePair(t: Triangle, p: Point, ot: Triangle, op: Point) =
  let n1 = t.neighborCCW(p)
  let n2 = t.neighborCW(p)
  let n3 = ot.neighborCCW(op)
  let n4 = ot.neighborCW(op)

  let ce1 = t.constrainedEdgeCCW(p)
  let ce2 = t.constrainedEdgeCW(p)
  let ce3 = ot.constrainedEdgeCCW(op)
  let ce4 = ot.constrainedEdgeCW(op)

  let de1 = t.delaunayEdgeCCW(p)
  let de2 = t.delaunayEdgeCW(p)
  let de3 = ot.delaunayEdgeCCW(op)
  let de4 = ot.delaunayEdgeCW(op)

  t.legalize(p, op)
  ot.legalize(op, p)

  # Remap delaunayEdge
  ot.setDelaunayEdgeCCW(p, de1)
  t.setDelaunayEdgeCW(p, de2)
  t.setDelaunayEdgeCCW(op, de3)
  ot.setDelaunayEdgeCW(op, de4)

  # Remap constrainedEdge
  ot.setConstrainedEdgeCCW(p, ce1)
  t.setConstrainedEdgeCW(p, ce2)
  t.setConstrainedEdgeCCW(op, ce3)
  ot.setConstrainedEdgeCW(op, ce4)

  # Remap neighbors
  # XXX: might optimize the markNeighbor by keeping track of
  #      what side should be assigned to what neighbor after the
  #      rotation. Now mark neighbor does lots of testing to find
  #      the right side.
  t.clearNeighbors()
  ot.clearNeighbors()
  if n1 != nil:
    ot.markNeighbor(n1)
  if n2 != nil:
    t.markNeighbor(n2)
  if n3 != nil:
    t.markNeighbor(n3)
  if n4 != nil:
    ot.markNeighbor(n4)
  t.markNeighbor(ot)

# Returns true if triangle was legalized
proc legalize(tcx: var SweepContext, t: Triangle): bool =
  # To legalize a triangle we start by finding if any of the three edges
  # violate the Delaunay condition
  for i in 0..2:
    if t.delaunayEdge[i]:
      continue
    let ot = t.neighbor(i)
    if ot != nil:
      let p = t[i]
      let op = ot.oppositePoint(t, p)
      let oi = ot.index(op)

      # If this is a Constrained Edge or a Delaunay Edge(only during recursive legalization)
      # then we should not try to legalize
      if ot.constrainedEdge[oi] or ot.delaunayEdge[oi]:
        t.constrainedEdge[i] = ot.constrainedEdge[oi]
        continue

      let inside = incircle2(p.pos, t.pointCCW(p).pos, t.pointCW(p).pos, op.pos)
      if inside:
        # Lets mark this shared edge as Delaunay
        t.delaunayEdge[i] = true
        ot.delaunayEdge[oi] = true

        # Lets rotate shared edge one vertex CW to legalize it
        rotateTrianglePair(t, p, ot, op)

        # We now got one valid Delaunay Edge shared by two triangles
        # This gives us 4 new edges to check for Delaunay

        # Make sure that triangle to node mapping is done only one time for a specific triangle
        if not tcx.legalize(t):
          tcx.mapTriangleToNodes(t)
        if not tcx.legalize(ot):
          tcx.mapTriangleToNodes(ot)

        # Reset the Delaunay edges, since they only are valid Delaunay edges
        # until we add a new triangle or point.
        # XXX: need to think about this. Can these edges be tried after we
        #      return to previous recursive level?
        t.delaunayEdge[i] = false
        ot.delaunayEdge[oi] = false

        # If triangle have been legalized no need to check the other edges since
        # the recursive legalization will handles those so we can end here.
        return true

# Adds a triangle to the advancing front to fill a hole.
# node - middle node, that is the bottom of the hole
proc fill(tcx: var SweepContext, node: Node) =
  var triangle = newTriangle(node.prev.point, node.point, node.next.point)

  # TODO: should copy the constrained_edge value from neighbor triangles
  #       for now constrained_edge values are copied during the legalize
  triangle.markNeighbor(node.prev.triangle)
  triangle.markNeighbor(node.triangle)

  tcx.map.add(triangle)

  # Update the advancing front
  node.prev.next = node.next
  node.next.prev = node.prev

  # If it was legalized the triangle has already been mapped
  if not tcx.legalize(triangle):
    tcx.mapTriangleToNodes(triangle)

proc fillLeftConcaveEdgeEvent(tcx: var SweepContext, edge: Edge, node: Node) =
  tcx.fill(node.prev)
  if node.prev.point != edge.p:
    # Next above or below edge?
    if orient2d(edge.q.pos, node.prev.pos, edge.p.pos) == orCW:
      # Below
      if orient2d(node.pos, node.prev.pos, node.prev.prev.pos) == orCW:
        # Next is concave
        tcx.fillLeftConcaveEdgeEvent(edge, node)
      else:
        # Next is convex
        discard

proc fillLeftConvexEdgeEvent(tcx: var SweepContext, edge: Edge, node: Node) =
  # Next concave or convex?
  if orient2d(node.prev.pos, node.prev.prev.pos, node.prev.prev.prev.pos) == orCW:
    # Concave
    tcx.fillLeftConcaveEdgeEvent(edge, node.prev)
  else:
    # Convex
    # Next above or below edge?
    if orient2d(edge.q.pos, node.prev.prev.pos, edge.p.pos) == orCW:
      # Below
      tcx.fillLeftConvexEdgeEvent(edge, node.prev)
    else:
      # Above
      discard

proc fillLeftBelowEdgeEvent(tcx: var SweepContext, edge: Edge, node: Node) =
  if node.pos.x > edge.p.pos.x:
    if orient2d(node.pos, node.prev.pos, node.prev.prev.pos) == orCW:
      # Concave
      tcx.fillLeftConcaveEdgeEvent(edge, node)
    else:
      # Convex
      tcx.fillLeftConvexEdgeEvent(edge, node)
      # Retry this one
      tcx.fillLeftBelowEdgeEvent(edge, node)

proc fillLeftAboveEdgeEvent(tcx: var SweepContext, edge: Edge, node: Node) =
  var node = node
  while node.prev.pos.x > edge.p.pos.x:
    # Check if next node is below the edge
    if orient2d(edge.q.pos, node.prev.pos, edge.p.pos) == orCW:
      tcx.fillLeftBelowEdgeEvent(edge, node)
    else:
      node = node.prev

proc fillRightConcaveEdgeEvent(tcx: var SweepContext, edge: Edge, node: Node) =
  tcx.fill(node.next)
  if node.next.point != edge.p:
    # Next above or below edge?
    if orient2d(edge.q.pos, node.next.pos, edge.p.pos) == orCCW:
      # Below
      if orient2d(node.pos, node.next.pos, node.next.next.pos) == orCCW:
        # Next is concave
        tcx.fillRightConcaveEdgeEvent(edge, node)
      else:
        # Next is convex
        discard

proc fillRightConvexEdgeEvent(tcx: var SweepContext, edge: Edge, node: Node) =
  # Next concave or convex?
  if orient2d(node.next.pos, node.next.next.pos, node.next.next.next.pos) == orCCW:
    # Concave
    tcx.fillRightConcaveEdgeEvent(edge, node.next)
  else:
    # Convex
    # Next above or below edge?
    if orient2d(edge.q.pos, node.next.next.pos, edge.p.pos) == orCCW:
      # Below
      tcx.fillRightConvexEdgeEvent(edge, node.next)
    else:
      # Above
      discard

proc fillRightBelowEdgeEvent(tcx: var SweepContext, edge: Edge, node: Node) =
  if node.pos.x < edge.p.pos.x:
    if orient2d(node.pos, node.next.pos, node.next.next.pos) == orCCW:
      # Concave
      tcx.fillRightConcaveEdgeEvent(edge, node)
    else:
      # Convex
      tcx.fillRightConvexEdgeEvent(edge, node)
      # Retry this one
      tcx.fillRightBelowEdgeEvent(edge, node)

proc fillRightAboveEdgeEvent(tcx: var SweepContext, edge: Edge, node: Node) =
  var node = node
  while node.next.pos.x < edge.p.pos.x:
    # Check if next node is below the edge
    if orient2d(edge.q.pos, node.next.pos, edge.p.pos) == orCCW:
      tcx.fillRightBelowEdgeEvent(edge, node)
    else:
      node = node.next

proc fillEdgeEvent(tcx: var SweepContext, edge: Edge, node: Node) =
  if tcx.edgeEvent.right:
    tcx.fillRightAboveEdgeEvent(edge, node)
  else:
    tcx.fillLeftAboveEdgeEvent(edge, node)

proc isShallow(tcx: SweepContext, node: Node): bool =
  let height = if tcx.basin.leftHighest:
    tcx.basin.leftNode.pos.y - node.pos.y
  else:
    tcx.basin.rightNode.pos.y - node.pos.y

  # if shallow stop filling
  if tcx.basin.width > height:
    return true

# Recursive algorithm to fill a Basin with triangles
# node - bottom_node
proc fillBasinReq(tcx: var SweepContext, node: Node) =
  # if shallow stop filling
  if tcx.isShallow(node):
    return

  tcx.fill(node)

  var node = node
  if node.prev == tcx.basin.leftNode and node.next == tcx.basin.rightNode:
    return
  elif node.prev == tcx.basin.leftNode:
    if orient2d(node.pos, node.next.pos, node.next.next.pos) == orCW:
      return
    node = node.next
  elif node.next == tcx.basin.rightNode:
    if orient2d(node.pos, node.prev.pos, node.prev.prev.pos) == orCCW:
      return
    node = node.prev
  else:
    # Continue with the neighbor node with lowest Y value
    if node.prev.pos.y < node.next.pos.y:
      node = node.prev
    else:
      node = node.next

  tcx.fillBasinReq(node)

#[
  Fills a basin that has formed on the Advancing Front to the right
  of given node.<br>
  First we decide a left,bottom and right node that forms the
  boundaries of the basin. Then we do a reqursive fill.
  
  @param {!SweepContext} tcx - SweepContext object
  @param node - starting node, this or next node will be left node
]#
proc fillBasin(tcx: var SweepContext, node: Node) =
  if orient2d(node.pos, node.next.pos, node.next.next.pos) == orCCW:
    tcx.basin.leftNode = node.next.next
  else:
    tcx.basin.leftNode = node.next
  
  # Find the bottom and right node
  tcx.basin.bottomNode = tcx.basin.leftNode
  while tcx.basin.bottomNode.next != nil and tcx.basin.bottomNode.pos.y >= tcx.basin.bottomNode.next.pos.y:
    tcx.basin.bottomNode = tcx.basin.bottomNode.next
  if tcx.basin.bottomNode == tcx.basin.leftNode:
    # No valid basin
    return

  tcx.basin.rightNode = tcx.basin.bottomNode
  while tcx.basin.rightNode.next != nil and tcx.basin.rightNode.pos.y < tcx.basin.rightNode.next.pos.y:
    tcx.basin.rightNode = tcx.basin.rightNode.next
  if tcx.basin.rightNode == tcx.basin.bottomNode:
    # No valid basins
    return

  tcx.basin.width = tcx.basin.rightNode.pos.x - tcx.basin.leftNode.pos.x
  tcx.basin.leftHighest = tcx.basin.leftNode.pos.y > tcx.basin.rightNode.pos.y
  tcx.fillBasinReq(tcx.basin.bottomNode)



# Creates a new front triangle and legalize it
proc newFrontTriangle(tcx: var SweepContext, point: Point, node: Node): Node =
  var triangle = newTriangle(point, node.point, node.next.point)

  triangle.markNeighbor(node.triangle)
  tcx.map.add(triangle)

  result = newNode(point)
  result.next = node.next
  result.prev = node
  node.next.prev = result
  node.next = result

  if not tcx.legalize(triangle):
    tcx.mapTriangleToNodes(triangle)

# Fills holes in the Advancing Front
proc fillAdvancingFront(tcx: var SweepContext, n: Node) =
  var node = n.next
  while node.next != nil:
    # TODO integrate here changes from C++ version
    # (C++ repo revision acf81f1f1764 dated April 7, 2012)
    if isAngleObtuse(node.point.pos, node.next.point.pos, node.prev.point.pos):
      break
    tcx.fill(node)
    node = node.next
  
  # Fill left nodes
  node = n.prev
  while node.prev != nil:
    # TODO integrate here changes from C++ version
    # (C++ repo revision acf81f1f1764 dated April 7, 2012)
    if isAngleObtuse(node.point.pos, node.next.point.pos, node.prev.point.pos):
      break
    tcx.fill(node)
    node = node.prev
  
  # Fill right basins
  if n.next != nil and n.next.next != nil:
    if isBasinAngleRight(n):
      tcx.fillBasin(n)

#[
  When we need to traverse from one triangle to the next we need
  the point in current triangle that is the opposite point to the next
  triangle.
]#
proc nextFlipPoint(tcx: SweepContext, ep, eq: Point, ot: Triangle, op: Point): Point =
  case orient2d(eq.pos, op.pos, ep.pos):
  of orCW:
    # Right
    return ot.pointCCW(op)
  of orCCW:
    # Left
    return ot.pointCW(op)
  else:
    raise PointError(msg: "poly2tri [Unsupported] nextFlipPoint: opposing point on constrained edge!", points: @[eq, op, ep])

#[
  After a flip we have two triangles and know that only one will still be
  intersecting the edge. So decide which to contiune with and legalize the other

  @param {!SweepContext} tcx - SweepContext object
  @param o - should be the result of an orient2d( eq, op, ep )
  @param t - triangle 1
  @param ot - triangle 2
  @param p - a point shared by both triangles
  @param op - another point shared by both triangles
  @return returns the triangle still intersecting the edge
]#
proc nextFlipTriangle(tcx: var SweepContext, o: Orient, t, ot: Triangle, p, op: Point): Triangle =
  if o == orCCW:
    # ot is not crossing edge after flip
    ot.delaunayEdge[ot.edgeIndex(p, op)] = true
    discard tcx.legalize(ot)
    ot.clearDelaunayEdges()
    return t

  # t is not crossing edge after flip
  t.delaunayEdge[t.edgeIndex(p, op)] = true
  discard tcx.legalize(t)
  t.clearDelaunayEdges()
  return ot

proc flipEdgeEvent(tcx: var SweepContext, ep, eq: Point, t: Triangle, p: Point)
proc edgeEventByPoints(tcx: var SweepContext, ep, eq: Point, triangle: Triangle, point: Point)

#[
  Scan part of the FlipScan algorithm<br>
  When a triangle pair isn't flippable we will scan for the next
  point that is inside the flip triangle scan area. When found
  we generate a new flipEdgeEvent

  @param {!SweepContext} tcx - SweepContext object
  @param ep - last point on the edge we are traversing
  @param eq - first point on the edge we are traversing
  @param {!Triangle} flip_triangle - the current triangle sharing the point eq with edge
  @param t
  @param p
]#
proc flipScanEdgeEvent(tcx: var SweepContext, ep, eq: Point, flipTriangle: Triangle, t: Triangle, p: Point) =
  let ot = t.neighborAcross(p)
  assert ot != nil, "FLIP failed due to missing triangle"

  let op = ot.oppositePoint(t, p)
  if inScanArea(eq.pos, flipTriangle.pointCCW(eq).pos, flipTriangle.pointCW(eq).pos, op.pos):
    # flip with new edge op.eq
    tcx.flipEdgeEvent(eq, op, ot, op)
  else:
    let newP = tcx.nextFlipPoint(ep, eq, ot, op)
    tcx.flipScanEdgeEvent(ep, eq, flipTriangle, ot, newP)

proc flipEdgeEvent(tcx: var SweepContext, ep, eq: Point, t: Triangle, p: Point) =
  var t = t
  let ot = t.neighborAcross(p)
  assert ot != nil, "FLIP failed due to missing triangle!"

  let op = ot.oppositePoint(t, p)

  # Additional check from Java version (see issue #88)
  if t.constrainedEdgeAcross(p):
    let index = t.index(p)
    raise PointError(msg: "poly2tri Intersecting Constraints", points: @[p, op, t[(index + 1) mod 3], t[(index + 2) mod 3]])

  if inScanArea(p.pos, t.pointCCW(p).pos, t.pointCW(p).pos, op.pos):
    # Lets rotate shared edge one vertex CW
    rotateTrianglePair(t, p, ot, op)
    tcx.mapTriangleToNodes(t)
    tcx.mapTriangleToNodes(ot)

    # XXX: in the original C++ code for the next 2 lines, we are
    # comparing point values (and not pointers). In this JavaScript
    # code, we are comparing point references (pointers). This works
    # because we can't have 2 different points with the same values.
    # But to be really equivalent, we should use "Point.equals" here.
    if p == eq and op == ep:
      if eq == tcx.edgeEvent.constrainedEdge.q and ep == tcx.edgeEvent.constrainedEdge.p:
        t.markConstrainedEdgeByPoints(ep, eq)
        ot.markConstrainedEdgeByPoints(ep, eq)
        discard tcx.legalize(t)
        discard tcx.legalize(ot)
      else:
        # XXX: I think one of the triangles should be legalized here?
        discard
    else:
      let o = orient2d(eq.pos, op.pos, ep.pos)
      t = tcx.nextFlipTriangle(o, t, ot, p, op)
      tcx.flipEdgeEvent(ep, eq, t, p)
  else:
    let newP = tcx.nextFlipPoint(ep, eq, ot, op)
    tcx.flipScanEdgeEvent(ep, eq, t, ot, newP)
    tcx.edgeEventByPoints(ep, eq, t, p)

proc edgeEventByPoints(tcx: var SweepContext, ep, eq: Point, triangle: Triangle, point: Point) =
  var triangle = triangle
  if isEdgeSideOfTriangle(triangle, ep, eq):
    return

  let p1 = triangle.pointCCW(point)
  let o1 = orient2d(eq.pos, p1.pos, ep.pos)
  if o1 == orCollinear:
    # TODO integrate here changes from C++ version
    # (C++ repo revision 09880a869095 dated March 8, 2011)
    raise PointError(msg: "poly2tri EdgeEvent: Collinear not supported!", points: @[eq, p1, ep])

  let p2 = triangle.pointCW(point)
  let o2 = orient2d(eq.pos, p2.pos, ep.pos)
  if o2 == orCollinear:
    # TODO integrate here changes from C++ version
    # (C++ repo revision 09880a869095 dated March 8, 2011)
    raise PointError(msg: "poly2tri EdgeEvent: Collinear not supported!", points: @[eq, p2, ep])

  if o1 == o2:
    # Need to decide if we are rotating CW or CCW to get to a triangle
    # that will cross edge
    if o1 == orCW:
      triangle = triangle.neighborCCW(point)
    else:
      triangle = triangle.neighborCW(point)
    tcx.edgeEventByPoints(ep, eq, triangle, point)
  else:
    # This triangle crosses constraint so lets flippin start!
    tcx.flipEdgeEvent(ep, eq, triangle, point)
    

proc edgeEventByEdge(tcx: var SweepContext, edge: Edge, node: Node) =
  tcx.edgeEvent.constrainedEdge = edge
  tcx.edgeEvent.right = edge.p.pos.x > edge.q.pos.x

  if isEdgeSideOfTriangle(node.triangle, edge.p, edge.q):
    return

  # For now we will do all needed filling
  # TODO: integrate with flip process might give some better performance
  #       but for now this avoid the issue with cases that needs both flips and fills
  tcx.fillEdgeEvent(edge, node)
  tcx.edgeEventByPoints(edge.p, edge.q, node.triangle, edge.q)

# Find closes node to the left of the new point and
# create a new triangle. If needed new holes and basins
# will be filled to.
proc pointEvent(tcx: var SweepContext, point: Point): Node =
  let node = tcx.locateNode(point)
  let newNode = tcx.newFrontTriangle(point, node)

  # Only need to check +epsilon since point never have smaller
  # x value than node due to how we fetch nodes from the front
  if point.pos.x <= node.point.pos.x + eps:
    tcx.fill(node)
  
  tcx.fillAdvancingFront(newNode)
  return newNode

proc finalizationPolygon(tcx: var SweepContext) =
  # Get an Internal triangle to start with
  var t = tcx.front.head.next.triangle
  var p = tcx.front.head.next.point
  while not t.constrainedEdgeCW(p):
    t = t.neighborCCW(p)

  # Collect interior triangles constrained by edges
  tcx.meshClean(t)

# Start sweeping the Y-sorted point set from bottom to top
proc sweepPoints(tcx: var SweepContext) =
  for i in 1..<tcx.points.len:
    let point = tcx[i]
    let node = tcx.pointEvent(point)
    for edge in point.edges:
      tcx.edgeEventByEdge(edge, node)

# Triangulate the polygon with holes and Steiner points.
# Do this AFTER you've added the polyline, holes, and Steiner points
proc triangulate*(tcx: var SweepContext) =
  tcx.initTriangulation()
  tcx.createAdvancingFront()
  # Sweep points; build mesh
  tcx.sweepPoints()
  # Clean up
  tcx.finalizationPolygon()
