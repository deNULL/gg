import gg/[vectors, priorityqueue, cellgraph]
import algorithm

const MAX = 3.402823466e+38
const INVALID = -3.402823466e+38
const ZERO = (INVALID, INVALID)

type
  Voronoi = ref object
    graph: Graph
    min*: Vec2f
    max*: Vec2f

    pq: PriorityQueue[HalfEdge]
    beachlineStart: HalfEdge
    beachlineEnd: HalfEdge
    lastInserted: HalfEdge
    bottomCell: CellIndex
    padding: int
    bounds: BBox2f

  Direction = enum left, right
  HalfEdge = ref object
    edge: EdgeIndex
    left: HalfEdge
    right: HalfEdge
    pos: Vec2f
    y: float
    direction: Direction
    pqpos: int

proc `<`(he1, he2: HalfEdge): bool =
  if he1.y == he2.y:
    return he1.pos.x < he2.pos.x
  else:
    return he1.y < he2.y

proc isNone*(node: NodeIndex): bool = node == NodeIndex.high

proc intersect(graph: Graph, he1, he2: HalfEdge, p: var Vec2f): bool =
  let e1 = graph.edges[he1.edge]
  let e2 = graph.edges[he2.edge]
  let d = e1.a * e2.b - e1.b * e2.a
  if d ~= 0.0:
    return false
  p.x = (e1.c * e2.b - e1.b * e2.c) / d
  p.y = (e1.a * e2.c - e1.c * e2.a) / d
  var e: Edge
  var he: HalfEdge
  if cmpY(graph.cellPos[e1.cells[1]], graph.cellPos[e2.cells[1]]) < 0:
    he = he1
    e = e1
  else:
    he = he2
    e = e2

  let rightOfCell = p.x >= graph.cellPos[e.cells[1]].x
  if (rightOfCell and he.direction == Direction.left) or
     (not rightOfCell and he.direction == Direction.right):
    return false
  return true

proc link(he: HalfEdge, ne: HalfEdge) =
  ne.left = he
  ne.right = he.right
  he.right.left = ne
  he.right = ne

proc unlink(he: HalfEdge) =
  he.left.right = he.right
  he.right.left = he.left
  he.left = nil
  he.right = nil

proc rightOf(graph: Graph, he: HalfEdge, p: Vec2f): bool =
  var e = graph.edges[he.edge]
  var topCell = graph.cellPos[e.cells[1]]

  var rightOfCell = p.x > topCell.x
  if rightOfCell and he.direction == Direction.left:
    return true
  if not rightOfCell and he.direction == Direction.right:
    return false

  var dxp, dyp, dxs, t1, t2, t3, yl: float
  var above: bool
  if e.a == 1.0:
    dyp = p.y - topCell.y
    dxp = p.x - topCell.x
    var fast = false
    if (not rightOfCell and e.b < 0.0) or
       (rightOfCell and e.b >= 0.0):
      above = dyp >= e.b * dxp
      fast = above
    else:
      above = p.x + p.y * e.b > e.c
      if e.b < 0.0:
        above = not above
      if not above:
        fast = true
    if not fast:
      dxs = topCell.x - graph.cellPos[e.cells[0]].x
      above = e.b * (dxp * dxp - dyp * dyp) <
        dxs * dyp * (1.0 + 2.0 * dxp / dxs + e.b * e.b)
      if e.b < 0.0:
        above = not above
  else:
    yl = e.c - e.a * p.x
    t1 = p.y - yl
    t2 = p.x - topCell.x
    t3 = yl - topCell.y
    above = t1 * t1 > t2 * t2 + t3 * t3

  if he.direction == Direction.left: return above else: return not above

proc leftCell(graph: Graph, he: HalfEdge): CellIndex =
  graph.edges[he.edge].cells[ord(he.direction)]

proc rightCell(graph: Graph, he: HalfEdge): CellIndex =
  if he.edge != EdgeIndex.high: graph.edges[he.edge].cells[1 - ord(he.direction)] else: CellIndex.high

proc getEdgeAbove(v: Voronoi, p: Vec2f): HalfEdge =
  # Gets the arc on the beach line at the x coordinate (i.e. right above the new cell event)
  # A good guess it's close by (Can be optimized)
  result = v.lastInserted
  if result == nil:
    if p.x < (v.bounds.max.x - v.bounds.min.x) / 2.0:
      result = v.beachlineStart
    else:
      result = v.beachlineEnd

  if result == v.beachlineStart or (result != v.beachlineEnd and v.graph.rightOf(result, p)):
    while true:
      result = result.right
      if result == v.beachlineEnd or not v.graph.rightOf(result, p): break
    result = result.left
  else:
    while true:
      result = result.left
      if result == v.beachlineStart or v.graph.rightOf(result, p): break

proc checkCircleEvent(graph: Graph, he1, he2: HalfEdge, p: var Vec2f): bool =
  if he1.edge == EdgeIndex.high or he2.edge == EdgeIndex.high or graph.edges[he1.edge].cells[1] == graph.edges[he2.edge].cells[1]:
    return false
  return graph.intersect(he1, he2, p)

proc clipLine(v: Voronoi, e: Edge): bool =
  var x1, y1, x2, y2: float
  let i = if e.a == 1.0 and e.b >= 0.0: 1 else: 0
  let s1 = if e.nodes[i] != NodeIndex.high: v.graph.nodes[e.nodes[i]].pos else: ZERO
  let s2 = if e.nodes[1 - i] != NodeIndex.high: v.graph.nodes[e.nodes[1 - i]].pos else: ZERO
  if e.a == 1.0:
    y1 = v.min.y
    if s1 != ZERO and s1.y > v.min.y: y1 = s1.y
    y1 = min(y1, v.max.y)
    x1 = e.c - e.b * y1
    if x1 > v.max.x:
      x1 = v.max.x
      y1 = (e.c - x1) / e.b
    elif x1 < v.min.x:
      x1 = v.min.x
      y1 = (e.c - x1) / e.b

    y2 = v.max.y
    if s2 != ZERO and s2.y < v.max.y: y2 = s2.y
    y2 = max(y2, v.min.y)
    x2 = e.c - e.b * y2
    if x2 > v.max.x:
      x2 = v.max.x
      y2 = (e.c - x2) / e.b
    elif x2 < v.min.x:
      x2 = v.min.x
      y2 = (e.c - x2) / e.b
  else:
    x1 = v.min.x
    if s1 != ZERO and s1.x > v.min.x: x1 = s1.x
    x1 = min(x1, v.max.x)
    y1 = e.c - e.a * x1
    if y1 > v.max.y:
      y1 = v.max.y
      x1 = (e.c - y1) / e.a
    elif y1 < v.min.y:
      y1 = v.min.y
      x1 = (e.c - y1) / e.a

    x2 = v.max.x
    if s2 != ZERO and s2.x < v.max.x: x2 = s2.x
    x2 = max(x2, v.min.x)
    y2 = e.c - e.a * x2
    if y2 > v.max.y:
      y2 = v.max.y
      x2 = (e.c - y2) / e.a
    elif y2 < v.min.y:
      y2 = v.min.y
      x2 = (e.c - y2) / e.a
  if e.nodes[i] != NodeIndex.high:
    v.graph.nodes[e.nodes[i]].pos = (x1, y1)
  else:
    #return false
    e.nodes[i] = v.graph.newNode((x1, y1))
    #echo "added node #", e.nodes[i].index, " from clipLine"
  if e.nodes[1 - i] != NodeIndex.high:
    v.graph.nodes[e.nodes[1 - i]].pos = (x2, y2)
  else:
    #return false
    e.nodes[1 - i] = v.graph.newNode((x2, y2))
    #echo "added node #", e.nodes[1 - i].index, " from clipLine"
  return x1 != x2 or y1 != y2

# Turn one Edge into two Sides and store them in Cells
proc finishLine(v: Voronoi, ei: EdgeIndex) =
  let e = v.graph.edges[ei]
  if not clipLine(v, e):
    return
  let flip = area(v.graph.cellPos[e.cells[0]], v.graph.nodes[e.nodes[0]].pos, v.graph.nodes[e.nodes[1]].pos) > 0.0
  for i in 0..1:
    # alloc
    let side = Side(edge: ei, neighbor: e.cells[1 - i])
    side.nodes[ord(flip)] = e.nodes[i]
    side.nodes[1 - ord(flip)] = e.nodes[1 - i]
    v.graph.sortEdgesInsert(e.cells[i], side)
    # check that we didn't accidentally add a duplicate (rare), then remove it
    if side.next != nil and side.angle == side.next.angle:
      if side.nodes[0] == side.next.nodes[0] and side.nodes[1] == side.next.nodes[1]:
        side.next = side.next.next # Throw it away, they're so few anyways

proc endPos(v: Voronoi, e: EdgeIndex, n: NodeIndex, direction: Direction) =
  let edge = v.graph.edges[e]
  edge.nodes[ord(direction)] = n
  v.graph.nodes[n].add(e, edge.cells[ord(direction)]) # TODO: can be wrong
  if edge.nodes[1 - ord(direction)] != NodeIndex.high:
    v.finishLine(e)

proc cellEvent(v: Voronoi, cell: CellIndex) =
  let pos = v.graph.cellPos[cell]
  let left = v.getEdgeAbove(pos)
  let right = left.right
  let bottom = if v.graph.rightCell(left) == CellIndex.high: v.bottomCell else: v.graph.rightCell(left)
  let edge = v.graph.newEdge(bottom, cell)
  # alloc
  let edge1 = HalfEdge(edge: edge, direction: Direction.left)
  let edge2 = HalfEdge(edge: edge, direction: Direction.right)
  left.link(edge1)
  edge1.link(edge2)
  v.lastInserted = right
  var p: Vec2f
  if v.graph.checkCircleEvent(left, edge1, p):
    v.pq.remove(left)
    left.pos = p
    left.y = p.y + pos.dist(p)
    v.pq.push(left)
  if v.graph.checkCircleEvent(edge2, right, p):
    edge2.pos = p
    edge2.y = p.y + pos.dist(p)
    v.pq.push(edge2)

proc circleEvent(v: Voronoi) =
  let left = v.pq.pop()
  let leftLeft = left.left
  let right = left.right
  let rightRight = right.right
  var bottom = v.graph.leftCell(left)
  var top = v.graph.rightCell(right)
  # alloc
  let node = v.graph.newNode(left.pos)
  #echo "added node #", node.index, " from circleEvent"
  v.endPos(left.edge, node, left.direction)
  v.endPos(right.edge, node, right.direction)
  v.lastInserted = rightRight
  v.pq.remove(right)
  left.unlink()
  right.unlink()
  var direction = Direction.left
  if v.graph.cellPos[bottom].y > v.graph.cellPos[top].y:
    swap(top, bottom)
    direction = Direction.right
  let edge = v.graph.newEdge(bottom, top)
  # alloc
  let he = HalfEdge(edge: edge, direction: direction)
  leftLeft.link(he)
  v.endPos(edge, node, Direction(1 - ord(direction)))
  var p: Vec2f
  if v.graph.checkCircleEvent(leftLeft, he, p):
    v.pq.remove(leftLeft)
    leftLeft.pos = p
    leftLeft.y = p.y + v.graph.cellPos[bottom].dist(p)
    v.pq.push(leftLeft)
  if v.graph.checkCircleEvent(he, rightRight, p):
    he.pos = p
    he.y = p.y + v.graph.cellPos[bottom].dist(p)
    v.pq.push(he)

proc fillGap(v: Voronoi, cell: CellIndex) =
  # TODO
  return

proc fillGaps(v: Voronoi) =
  for cell in 0..<v.graph.cellPos.len:
    v.fillGap(cell.CellIndex)

proc voronoi*(graph: Graph, cells: var seq[CellIndex], sort: bool = true, bounds: BBox2f = bbox((-MAX, -MAX), (MAX, MAX))) =
  var v = Voronoi(
    graph: graph,
    beachlineStart: HalfEdge(edge: EdgeIndex.high),
    beachlineEnd: HalfEdge(edge: EdgeIndex.high),
    pq: initPriorityQueue[HalfEdge](2 * cells.len),
  )
  v.beachlineStart.right = v.beachlineEnd
  v.beachlineEnd.left = v.beachlineStart
  v.bounds = bounds
  v.min = bounds.min
  v.max = bounds.max
  if sort:
    cells.sort do (s1, s2: CellIndex) -> int:
      cmpY(graph.cellPos[s1], graph.cellPos[s2])
  v.bottomCell = if cells.len > 0: cells[0] else: CellIndex.high
  var cell = if cells.len > 1: cells[1] else: CellIndex.high
  var index = 2
  while true:
    var lowestPqVec2: Vec2f
    if not v.pq.empty:
      let he = v.pq.top()
      lowestPqVec2.x = he.pos.x
      lowestPqVec2.y = he.y
    if cell != CellIndex.high and (v.pq.empty or cmpY(graph.cellPos[cell], lowestPqVec2) < 0):
      v.cellEvent(cell)
      cell = if cells.len > index: cells[index] else: CellIndex.high
      index += 1
    elif not v.pq.empty:
      v.circleEvent()
    else:
      break
  var he = v.beachlineStart.right
  while he != v.beachlineEnd:
    v.finishLine(he.edge)
    he = he.right
  v.fillGaps()

proc newVoronoi*(
  points: seq[Vec2f],
  bounds: BBox2f = bbox((-MAX, -MAX), (MAX, MAX)),
  sort: bool = true,
  pruneDuplicates: bool = false
): Graph =
  result = newGraph(points, pruneDuplicates)
  var indices = newSeq[CellIndex](points.len)
  for i in 0..<points.len:
    indices[i] = i.CellIndex
  voronoi(result, indices, sort, bounds)