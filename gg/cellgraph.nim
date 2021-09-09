import gg/vectors
import algorithm
import math

type
  CellIndex* = uint32
  NodeIndex* = uint32
  EdgeIndex* = uint32

  # Nodes of Voronoi cells, triangles for Delanay.
  Node* = ref object
    pos*: Vec2f
    cells*: array[3, CellIndex]
    edges*: array[3, EdgeIndex]
    len: byte # Usually len=3, except for border nodes

  # The coefficients a, b and c are from the general line equation: ax + by + c = 0
  Edge* = ref object of RootObj
    nodes*: array[2, NodeIndex]
    cells*: array[2, CellIndex]
    a*, b*, c*: float

  # Oriented sides of Voronoi polygons
  Side* = ref object of RootObj
    next*: Side
    edge*: EdgeIndex
    neighbor*: CellIndex
    # pos: array[2, Vec2f]
    nodes*: array[2, NodeIndex]
    angle*: float

  Graph* = ref object
    cellPos*: seq[Vec2f]
    cellSides*: seq[Side] # n, initial side for each cell
    edges*: seq[Edge] # 3n - 6 max
    nodes*: seq[Node] # 2n - 5 max

proc other*(edge: Edge, node: NodeIndex): NodeIndex =
  if edge.nodes[0] == node: edge.nodes[1] else: edge.nodes[0]


iterator cellNeighbors*(graph: Graph, cell: CellIndex): CellIndex =
  var side = graph.cellSides[cell]
  while side != nil:
    yield side.neighbor
    side = side.next

iterator sides*(graph: Graph, cell: CellIndex): Side =
  var side = graph.cellSides[cell]
  while side != nil:
    yield side
    side = side.next

proc sideCount*(graph: Graph, cell: CellIndex): int =
  var side = graph.cellSides[cell]
  while side != nil:
    result += 1
    side = side.next

iterator nodeCells*(graph: Graph, node: NodeIndex): CellIndex =
  let n = graph.nodes[node]
  for i in 0.byte..<n.len:
    yield n.cells[i]

iterator nodeEdges*(graph: Graph, node: NodeIndex): EdgeIndex =
  let n = graph.nodes[node]
  for i in 0.byte..<n.len:
    yield n.edges[i]

proc sides*(graph: Graph, cell: CellIndex): seq[Side] =
  result = newSeq[Side](graph.sideCount(cell))
  var side = graph.cellSides[cell]
  var i = 0
  while side != nil:
    result[i] = side
    i += 1
    side = side.next

proc nodes*(graph: Graph, cell: CellIndex): seq[NodeIndex] =
  result = newSeq[NodeIndex](graph.sideCount(cell))
  var side = graph.cellSides[cell]
  var last: NodeIndex = NodeIndex.high
  var i = 0
  while side != nil:
    if last == NodeIndex.high:
      if side.nodes[0] == side.next.nodes[0] or side.nodes[0] == side.next.nodes[1]:
        result[i] = side.nodes[0]
      else:
        result[i] = side.nodes[1]
    else:
      if side.nodes[0] == last:
        result[i] = side.nodes[1]
      else:
        result[i] = side.nodes[0]
    last = result[i]
    i += 1
    side = side.next

proc add*(node: Node, edge: EdgeIndex, cell: CellIndex) =
  node.edges[node.len] = edge
  node.cells[node.len] = cell
  node.len += 1

proc newEdge*(graph: Graph, s1, s2: CellIndex): EdgeIndex =
  # alloc
  var edge = Edge(nodes: [NodeIndex.high, NodeIndex.high], cells: [s1, s2])
  let s1p = graph.cellPos[s1]
  let s2p = graph.cellPos[s2]
  let dx = s2p.x - s1p.x
  let dy = s2p.y - s1p.y
  let dxLarger = dx * dx > dy * dy
  edge.c = dx * (s1p.x + dx * 0.5) + dy * (s1p.y + dy * 0.5)
  if dxLarger:
    edge.a = 1
    edge.b = dy / dx
    edge.c /= dx
  else:
    edge.a = dx / dy
    edge.b = 1
    edge.c /= dy
  graph.edges.add(edge)
  return EdgeIndex(graph.edges.len) - 1

proc newNode*(graph: Graph, p: Vec2f): NodeIndex =
  graph.nodes.add(Node(pos: p, cells: [CellIndex.high, CellIndex.high, CellIndex.high], edges: [EdgeIndex.high, EdgeIndex.high, EdgeIndex.high], len: 0))
  return NodeIndex(graph.nodes.len) - 1

proc delNode*(graph: Graph, node: NodeIndex) =
  for cell in graph.nodeCells(node):
    var prevSide: Side = nil
    for side in graph.sides(cell): # No way to find the appropriate sides quickly :(
      if side.nodes[0] == node or side.nodes[1] == node:
        if prevSide == nil:
          graph.cellSides[cell] = side.next
        else:
          prevSide.next = side.next
      else:
        prevSide = side

  for edge in graph.nodeEdges(node):
    let otherNode = graph.nodes[graph.edges[edge].other(node)]
    for i in 0.byte..<otherNode.len:
      if otherNode.edges[i] == edge:
        otherNode.len -= 1
        for j in i..<otherNode.len:
          otherNode.edges[j] = otherNode.edges[j + 1]
          otherNode.cells[j] = otherNode.cells[j + 1]
        break

proc newGraph*(points: seq[Vec2f], pruneDuplicates: bool = false): Graph =
  result = Graph(
    cellPos: points,
    cellSides: newSeq[Side](points.len),
    edges: newSeqOfCap[Edge](3 * points.len), # Should be 3 * n - 6 max
    nodes: newSeqOfCap[Node](2 * points.len), # Should be 2 * n - 5 max
  )

  if pruneDuplicates:
    result.cellPos.sort do (s1, s2: Vec2f) -> int:
      if s1 < s2: result = -1 else: result = 1

    var i, k = 0
    while i < result.cellPos.len:
      var j = i + 1
      while j < result.cellPos.len and result.cellPos[i] ~= result.cellPos[j]:
        j += 1
      result.cellPos[k] = result.cellPos[i]
      i = j
      k += 1
    result.cellPos.setLen(k)
    result.cellSides.setLen(k)


proc calcSortMetric*(graph: Graph, cell: CellIndex, side: Side): float =
  let n0p = graph.nodes[side.nodes[0]].pos
  let n1p = graph.nodes[side.nodes[1]].pos
  let x = (n0p.x + n1p.x) * 0.5
  let y = (n0p.y + n1p.y) * 0.5
  let diffy = y - graph.cellPos[cell].y
  # TODO: remove arctan2? see: https://github.com/mapbox/delaunator/blob/705d6df3e753bd3212c0a295d21e6c40248c29bd/index.js#L372
  result = arctan2(diffy, x - graph.cellPos[cell].x)
  if diffy < 0:
    result += 2 * PI

proc sortEdgesInsert*(graph: Graph, cell: CellIndex, side: Side) =
  side.angle = graph.calcSortMetric(cell, side)
  if graph.cellSides[cell] == nil or graph.cellSides[cell].angle >= side.angle:
    side.next = graph.cellSides[cell]
    graph.cellSides[cell] = side
  else:
    var current = graph.cellSides[cell]
    while current.next != nil and current.next.angle < side.angle:
      current = current.next
    side.next = current.next
    current.next = side

#[
proc merge*(d1: var Graph, d2: Graph, indices: seq[int], cells: seq[Cell]) =
  # d1.cells &= d2.cells
  for i, j in indices:
    let cell = cells[j]
    var lastSide: Side = nil
    # Drop removed sides at start
    while cell.firstSide != nil and cell.firstSide.edge.index == -1:
      cell.firstSide = cell.firstSide.next
    var merged: bool
    for side in cell.sides:
      if side.edge.index != -1:
        lastSide = side
      else:
        # Add sides from new diagram
        lastSide.next = d2.cells[i].firstSide
        # TODO: fix missing neighbors
        merged = true
        break
    if not merged:
      if lastSide != nil:
        lastSide.next = d2.cells[i].firstSide
      else:
        cell.firstSide = d2.cells[i].firstSide

  for edge in d2.edges:
    #var pair = newSeq[int]()
    for i, cell in edge.cells:
      edge.cells[i] = cells[indices[cell.index]]
      #pair.add(edge.cells[i].index)
    #echo "new edge ", pair
  d1.edges &= d2.edges
  for node in d2.nodes:
    for i, cell in node.cells:
      if cell != nil:
        node.cells[i] = cells[indices[cell.index]]
  d1.nodes &= d2.nodes
  # TODO: prune duplicates

]#