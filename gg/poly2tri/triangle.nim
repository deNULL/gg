type
  Triangle = ref object
    # Triangle points
    points: array[3, Point]
    # Neighbor list
    neighbors: array[3, Triangle]
    # Has this triangle been marked as an interior triangle?
    interior: bool
    # Flags to determine if an edge is a Constrained edge
    constrainedEdge: array[3, bool]
    # Flags to determine if an edge is a Delauney edge
    delaunayEdge: array[3, bool]

#[
  Triangle class.<br>
  Triangle-based data structures are known to have better performance than
  quad-edge structures.
  See: J. Shewchuk, "Triangle: Engineering a 2D Quality Mesh Generator and
  Delaunay Triangulator", "Triangulations in CGAL"
]#

proc newTriangle*(p0, p1, p2: Point): Triangle =
  Triangle(points: [p0, p1, p2])

proc `[]`*(t: Triangle, index: int): Point {.inline.} = t.points[index]
proc points*(t: Triangle): array[3, Point] {.inline.} = t.points

proc `$`*(t: Triangle): string =
  "[" & $t[0].pos & $t[1].pos & $t[2].pos & "]"

proc neighbor(t: Triangle, index: int): Triangle {.inline.} = t.neighbors[index]
proc containsPoint(t: Triangle, point: Point): bool {.inline.} =
  t[0] == point or t[1] == point or t[2] == point
proc containsEdge(t: Triangle, edge: Edge): bool {.inline.} =
  t.containsPoint(edge.p) or t.containsPoint(edge.q)
proc containsPoints(t: Triangle, p1, p2: Point): bool {.inline.} =
  t.containsPoint(p1) and t.containsPoint(p2)

proc markNeighborPointers(t: Triangle, p1, p2: Point, neighbor: Triangle) =
  if (p1 == t[2] and p2 == t[1]) or (p1 == t[1] and p2 == t[2]):
    t.neighbors[0] = neighbor
  elif (p1 == t[0] and p2 == t[2]) or (p1 == t[2] and p2 == t[0]):
    t.neighbors[1] = neighbor
  elif (p1 == t[0] and p2 == t[1]) or (p1 == t[1] and p2 == t[0]):
    t.neighbors[2] = neighbor
  else:
    raise newException(AssertionDefect, "poly2tri Invalid Triangle.markNeighborPointers() call")
proc markNeighbor*(t, neighbor: Triangle) =
  if neighbor.containsPoints(t[1], t[2]):
    t.neighbors[0] = neighbor
    neighbor.markNeighborPointers(t[1], t[2], t)
  elif neighbor.containsPoints(t[0], t[2]):
    t.neighbors[1] = neighbor
    neighbor.markNeighborPointers(t[0], t[2], t)
  elif neighbor.containsPoints(t[0], t[1]):
    t.neighbors[2] = neighbor
    neighbor.markNeighborPointers(t[0], t[1], t)

proc clearNeighbors(t: Triangle) =
  t.neighbors[0] = nil
  t.neighbors[1] = nil
  t.neighbors[2] = nil
proc clearDelaunayEdges(t: Triangle) =
  t.delaunayEdge[0] = false
  t.delaunayEdge[1] = false
  t.delaunayEdge[2] = false

proc pointCW(t: Triangle, p: Point): Point =
  if p == t[0]:
    t[2]
  elif p == t[1]:
    t[0]
  elif p == t[2]:
    t[1]
  else:
    nil

proc pointCCW(t: Triangle, p: Point): Point =
  if p == t[0]:
    t[1]
  elif p == t[1]:
    t[2]
  elif p == t[2]:
    t[0]
  else:
    nil

proc neighborCW(t: Triangle, p: Point): Triangle =
  if p == t[0]:
    t.neighbors[1]
  elif p == t[1]:
    t.neighbors[2]
  else:
    t.neighbors[0]

proc neighborCCW(t: Triangle, p: Point): Triangle =
  if p == t[0]:
    t.neighbors[2]
  elif p == t[1]:
    t.neighbors[0]
  else:
    t.neighbors[1]

proc constrainedEdgeCW(t: Triangle, p: Point): bool =
  if p == t[0]:
    t.constrainedEdge[1]
  elif p == t[1]:
    t.constrainedEdge[2]
  else:
    t.constrainedEdge[0]

proc constrainedEdgeCCW(t: Triangle, p: Point): bool =
  if p == t[0]:
    t.constrainedEdge[2]
  elif p == t[1]:
    t.constrainedEdge[0]
  else:
    t.constrainedEdge[1]

proc constrainedEdgeAcross(t: Triangle, p: Point): bool =
  if p == t[0]:
    t.constrainedEdge[0]
  elif p == t[1]:
    t.constrainedEdge[1]
  else:
    t.constrainedEdge[2]

proc setConstrainedEdgeCW(t: Triangle, p: Point, ce: bool) =
  if p == t[0]:
    t.constrainedEdge[1] = ce
  elif p == t[1]:
    t.constrainedEdge[2] = ce
  else:
    t.constrainedEdge[0] = ce

proc setConstrainedEdgeCCW(t: Triangle, p: Point, ce: bool) =
  if p == t[0]:
    t.constrainedEdge[2] = ce
  elif p == t[1]:
    t.constrainedEdge[0] = ce
  else:
    t.constrainedEdge[1] = ce

proc delaunayEdgeCW(t: Triangle, p: Point): bool =
  if p == t[0]:
    t.delaunayEdge[1]
  elif p == t[1]:
    t.delaunayEdge[2]
  else:
    t.delaunayEdge[0]

proc delaunayEdgeCCW(t: Triangle, p: Point): bool =
  if p == t[0]:
    t.delaunayEdge[2]
  elif p == t[1]:
    t.delaunayEdge[0]
  else:
    t.delaunayEdge[1]

proc setDelaunayEdgeCW(t: Triangle, p: Point, e: bool) =
  if p == t[0]:
    t.delaunayEdge[1] = e
  elif p == t[1]:
    t.delaunayEdge[2] = e
  else:
    t.delaunayEdge[0] = e

proc setDelaunayEdgeCCW(t: Triangle, p: Point, e: bool) =
  if p == t[0]:
    t.delaunayEdge[2] = e
  elif p == t[1]:
    t.delaunayEdge[0] = e
  else:
    t.delaunayEdge[1] = e

proc neighborAcross(t: Triangle, p: Point): Triangle =
  if p == t[0]:
    t.neighbors[0]
  elif p == t[1]:
    t.neighbors[1]
  else:
    t.neighbors[2]

proc oppositePoint(t: Triangle, t1: Triangle, p: Point): Point =
  t.pointCW(t1.pointCW(p))

proc legalize(t: Triangle, opoint, npoint: Point) =
  if opoint == t[0]:
    t.points[1] = t[0]
    t.points[0] = t[2]
    t.points[2] = npoint
  elif opoint == t[1]:
    t.points[2] = t[1]
    t.points[1] = t[0]
    t.points[0] = npoint
  elif opoint == t[2]:
    t.points[0] = t[2]
    t.points[2] = t[1]
    t.points[1] = npoint
  else:
    raise newException(AssertionDefect, "poly2tri Invalid Triangle.legalize() call")

proc index(t: Triangle, p: Point): int =
  if p == t[0]:
    0
  elif p == t[1]:
    1
  elif p == t[2]:
    2
  else:
    raise newException(AssertionDefect, "poly2tri Invalid Triangle.index() call")

proc edgeIndex(t: Triangle, p1, p2: Point): int =
  if p1 == t[0]:
    if p2 == t[1]: 
      return 2
    elif p2 == t[2]: 
      return 1
  elif p1 == t[1]:
    if p2 == t[2]: 
      return 0 
    elif p2 == t[0]: 
      return 2
  elif p1 == t[2]:
    if p2 == t[0]: 
      return 1 
    elif p2 == t[1]:
      return 0
  return -1

proc markConstrainedEdgeByIndex(t: Triangle, index: int) =
  t.constrainedEdge[index] = true

proc markConstrainedEdgeByPoints(t: Triangle, p, q: Point) =
  if (q == t[0] and p == t[1]) or (q == t[1] and p == t[0]):
    t.constrainedEdge[2] = true
  elif (q == t[0] and p == t[2]) or (q == t[2] and p == t[0]):
    t.constrainedEdge[1] = true
  elif (q == t[1] and p == t[2]) or (q == t[2] and p == t[1]):
    t.constrainedEdge[0] = true

proc markConstrainedEdgeByEdge(t: Triangle, edge: Edge) =
  t.markConstrainedEdgeByPoints(edge.p, edge.q)