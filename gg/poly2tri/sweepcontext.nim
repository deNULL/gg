type
  Basin = ref object
    leftNode: Node
    bottomNode: Node
    rightNode: Node
    width: float
    leftHighest: bool

  EdgeEvent = ref object
    constrainedEdge: Edge
    right: bool

  SweepContext* = object
    points: seq[Point]
    triangles: seq[Triangle]
    map: seq[Triangle]
    edges: seq[Edge]

    # Bounding box of all points. Computed at the start of the triangulation, 
    # it is stored in case it is needed by the caller.
    bbox: BBox2f

    # Advancing front
    front: AdvancingFront

    # head point used with advancing front
    head: Point

    # tail point used with advancing front
    tail: Point

    afHead: Node
    afMiddle: Node
    afTail: Node

    basin: Basin
    edgeEvent: EdgeEvent


proc `[]`(tcx: SweepContext, index: int): Point {.inline.} = tcx.points[index]

let kAlpha = 0.3

proc clear(basin: Basin) =
  basin.leftNode = nil
  basin.bottomNode = nil
  basin.rightNode = nil
  basin.width = 0.0
  basin.leftHighest = false

proc newEdge(p1, p2: Point): Edge =
  result = Edge(p: p1, q: p2)
  if p1.pos.y > p2.pos.y:
    result.q = p1
    result.p = p2
  elif p1.pos.y == p2.pos.y:
    if p1.pos.x > p2.pos.x:
      result.q = p1
      result.p = p2
    elif p1.pos.x == p2.pos.x:
      raise PointError(msg: "poly2tri Invalid Edge constructor: repeated points!", points: @[p1, p2])
  
  result.q.edges.add(result)

proc initEdges(tcx: var SweepContext, points: openArray[Point]) =
  for i in 0..<points.len:
    tcx.edges.add(newEdge(points[i], points[(i + 1) mod points.len]))

proc newSweepContext*(contour: Path2f): SweepContext =
  result = SweepContext(
    points: newSeq[Point](contour.len),
    bbox: BBox2f.empty, 
    basin: Basin(), 
    edgeEvent: EdgeEvent()
  )
  for i in 0..<contour.len:
    result.points[i] = Point(id: i, pos: contour[i])
  result.initEdges(result.points)

proc addHole*(tcx: var SweepContext, path: Path2f) =
  let st = tcx.points.len
  tcx.points.setLen(st + path.len)
  for i in st..<tcx.points.len:
    tcx.points[i] = Point(id: i, pos: path[i - st])
  tcx.initEdges(tcx.points[st..<tcx.points.len])

proc addHoles*(tcx: var SweepContext, holes: varargs[Path2f]) =
  for hole in holes:
    tcx.addHole(hole)

proc addPoint*(tcx: var SweepContext, point: Vec2f) =
  tcx.points.add(Point(id: tcx.points.len, pos: point))

proc addPoints*(tcx: var SweepContext, points: varargs[Vec2f]) =
  let st = tcx.points.len
  tcx.points.setLen(st + points.len)
  for i in st..<tcx.points.len:
    tcx.points[i] = Point(id: i, pos: points[i - st])

proc bbox*(tcx: SweepContext): BBox2f = tcx.bbox
proc triangles*(tcx: SweepContext): seq[Triangle] = tcx.triangles

proc initTriangulation(tcx: var SweepContext) =
  tcx.bbox = BBox2f.empty
  for p in tcx.points:
    tcx.bbox = tcx.bbox.union(p.pos.bbox)
  let d = kAlpha * (tcx.bbox.max - tcx.bbox.min)
  tcx.head = Point(pos: (tcx.bbox.max.x + d.x, tcx.bbox.min.y - d.y))
  tcx.tail = Point(pos: (tcx.bbox.min.x - d.x, tcx.bbox.min.y - d.y))

  # Sort points along y-axis
  tcx.points.sort()

proc createAdvancingFront(tcx: var SweepContext) = 
  # Initial triangle
  let triangle = newTriangle(tcx.points[0], tcx.tail, tcx.head)
  
  tcx.map.add(triangle)
  let head = newNode(triangle[1], triangle)
  let middle = newNode(triangle[0], triangle)
  let tail = newNode(triangle[2])

  tcx.front = newAdvancingFront(head, tail)
  head.next = middle
  middle.next = tail
  middle.prev = head
  tail.prev = middle

proc mapTriangleToNodes(tcx: var SweepContext, t: Triangle) =
  for i in 0..2:
    if t.neighbor(i) == nil:
      let n = tcx.front.locatePoint(t.pointCW(t[i]))
      if n != nil:
        n.triangle = t

proc removeFromMap(tcx: var SweepContext, t: Triangle) =
  for i in 0..<tcx.map.len:
    if tcx.map[i] == t:
      tcx.map.delete(i) # is it safe to use del?
      return

proc meshClean(tcx: var SweepContext, t: Triangle) =
  # New implementation avoids recursive calls and use a loop instead.
  # Cf. issues # 57, 65 and 69.
  var triangles = @[t]
  var t = t
  while triangles.len > 0:
    t = triangles.pop()
    if not t.interior:
      t.interior = true
      tcx.triangles.add(t)
      for i in 0..2:
        if not t.constrainedEdge[i]:
          triangles.add(t.neighbor(i))

proc locateNode(tcx: var SweepContext, point: Point): Node = tcx.front.locateNode(point.pos.x)