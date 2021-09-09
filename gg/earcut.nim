# Based on https://github.com/mapbox/earcut
#
# Denis Olshin, 2021

import gg/vectors
import algorithm
import math

type
  Node = ref object
    # vertex index in coordinates array
    i: int

    # vertex coordinates
    pos: Vec2f

    # previous and next vertex nodes in a polygon ring
    prev: Node
    next: Node

    # z-order curve value
    z: int

    # previous and next node in z-order
    prevZ: Node
    nextZ: Node

    # indicates whether this is a steiner point
    steiner: bool

  Triangles = seq[(int, int, int)]

iterator loop(n0: Node): Node =
  yield n0
  var n = n0.next
  while n != n0:
    yield n
    n = n.next

proc hash(n0: Node): int =
  var h = 7
  for n in n0.loop:
    var v = n.i
    if not n.nextZ.isNil:
      v = v * 17 + n.nextZ.i + 1
    if not n.prevZ.isNil:
      v = v * 19 + n.prevZ.i + 1
    h = (h * 13 + v + 1) and 0x7FFFFFFF
  return h

# find the leftmost node of a polygon ring
proc leftmost(start: Node): Node =
  result = start
  for node in start.loop:
    if node.pos < result.pos:
      result = node

proc signedArea(data: openArray[Vec2f], st, en: int): float =
  var sum = 0.0
  var j = en - 1
  for i in st..<en:
    sum += (data[j].x - data[i].x) * (data[j].y + data[i].y)
    j = i
  return sum

# create a node and optionally link it with previous one (in a circular doubly linked list)
proc insertNode(i: int, pos: Vec2f, last: Node): Node =
  result = Node(i: i, pos: pos)
  if last.isNil:
    result.prev = result
    result.next = result
  else:
    result.next = last.next
    result.prev = last
    last.next.prev = result
    last.next = result

proc removeNode(n: Node) =
  n.next.prev = n.prev
  n.prev.next = n.next
  if not n.prevZ.isNil:
    n.prevZ.nextZ = n.nextZ
  if not n.nextZ.isNil:
    n.nextZ.prevZ = n.prevZ

# create a circular doubly linked list from polygon points in the specified winding order
proc linkedList(data: openArray[Vec2f], st, en: int, clockwise: bool): Node =
  var last: Node
  if clockwise == (signedArea(data, st, en) > 0):
    for i in countup(st, en - 1):
      last = insertNode(i, data[i], last)
  else:
    for i in countdown(en - 1, st):
      last = insertNode(i, data[i], last)

  if not last.isNil and last.pos == last.next.pos:
    removeNode(last)
    last = last.next
  
  return last

# check if a polygon diagonal is locally inside the polygon
proc locallyInside(a, b: Node): bool {.inline.} =
  if area(a.prev.pos, a.pos, a.next.pos) > 0:
    area(a.pos, b.pos, a.next.pos)  <= 0 and area(a.pos, a.prev.pos, b.pos) <= 0
  else:
    area(a.pos, b.pos, a.prev.pos) > 0 or area(a.pos, a.next.pos, b.pos) > 0

# whether sector in vertex m contains sector in vertex p in the same coordinates
proc sectorContainsSector(m, p: Node): bool {.inline.} =
  area(m.prev.pos, m.pos, p.prev.pos) > 0 and area(p.next.pos, m.pos, m.next.pos) > 0
  
# David Eberly's algorithm for finding a bridge between hole and outer polygon
proc findHoleBridge(outerNode: Node, hole: Node): Node =
  # find a segment intersected by a ray from the hole's leftmost point to the left;
  # segment's endpoint with lesser x will be potential connection point
  var hp = hole.pos
  var qx = -Inf
  for n in outerNode.loop:
    if hp.y <= n.pos.y and hp.y >= n.next.pos.y and n.next.pos.y != n.pos.y:
      let x = n.pos.x + (hp.y - n.pos.y) * (n.next.pos.x - n.pos.x) / (n.next.pos.y - n.pos.y)
      if x <= hp.x and x > qx:
        qx = x
        if x == hp.x:
          if hp.y == n.pos.y:
            return n
          if hp.y == n.next.pos.y:
            return n.next
        result = if n.pos.x < n.next.pos.x: n else: n.next
  
  # hole touches outer segment; pick leftmost endpoint
  if result.isNil or hp.x == qx:
    return

  # look for points inside the triangle of hole point, segment intersection and endpoint;
  # if there are no points found, we have a valid connection;
  # otherwise choose the point of the minimum angle with the ray as connection point
  let mp = result.pos
  var tnMin = Inf
  for n in result.loop:
    if hp.x >= n.pos.x and n.pos.x >= mp.x and hp.x >= n.pos.x and
      n.pos.inside(if hp.y < mp.y: hp else: (qx, hp.y), mp, if hp.y < mp.y: (qx, hp.y) else: hp):
        let tn = abs(hp.y - n.pos.y) / (hp.x - n.pos.x) # tangential
        if n.locallyInside(hole) and
          (tn < tnMin or (tn == tnMin and (n.pos.x > result.pos.x or
            (n.pos.x == result.pos.x and sectorContainsSector(result, n))))):
              result = n
              tnMin = tn

# link two polygon vertices with a bridge; if the vertices belong to the same ring, it splits polygon into two;
# if one belongs to the outer ring and another to a hole, it merges it into a single ring
proc splitPolygon(a, b: Node): Node =
  let a2 = Node(i: a.i, pos: a.pos)
  let b2 = Node(i: b.i, pos: b.pos)
  let an = a.next
  let bp = b.prev
  a.next = b
  b.prev = a
  a2.next = an
  an.prev = a2
  b2.next = a2
  a2.prev = b2
  bp.next = b2
  b2.prev = bp
  return b2

# eliminate colinear or duplicate points
proc filterPoints(st: Node, en: Node = nil): Node {.discardable.} =
  if st.isNil:
    return st
  result = if en.isNil: st else: en
  var n = st
  var again = true
  while again or n != result:
    again = false
    if not n.steiner and (n.pos == n.next.pos or area(n.prev.pos, n.pos, n.next.pos) == 0.0):
      removeNode(n)
      result = n.prev
      n = result
      if n == n.next:
        break
      again = true
    else:
      n = n.next

# find a bridge between vertices that connects hole with an outer ring and and link it
proc eliminateHole(outerNode: var Node, hole: Node) =
  let bridge = findHoleBridge(outerNode, hole)
  if bridge.isNil:
    return
  let bridgeReverse = splitPolygon(bridge, hole)

  # filter collinear points around the cuts
  let filteredBridge = filterPoints(bridge, bridge.next)
  filterPoints(bridgeReverse, bridgeReverse.next)

  # check if input node was removed by the filtering
  if outerNode == bridge:
    outerNode = filteredBridge

# link every hole into the outer loop, producing a single-ring polygon without holes
proc eliminateHoles(data: openArray[Vec2f], holeIndices: openArray[int], outerNode: var Node) =
  var queue = newSeqOfCap[Node](data.len)
  for i, st in holeIndices:
    let en = if i < holeIndices.len - 1: holeIndices[i + 1] else: data.len
    let list = linkedList(data, st, en, false)
    list.steiner = list == list.next
    queue.add(list.leftmost)

  queue.sort() do (a, b: Node) -> int:
    sgn(a.pos.x - b.pos.x)

  # process holes from left to right
  for hole in queue:
    eliminateHole(outerNode, hole)
    outerNode = filterPoints(outerNode, outerNode.next)

# Simon Tatham's linked list merge sort algorithm
# http://www.chiark.greenend.org.uk/~sgtatham/algorithms/listsort.html
proc sortLinked(list: Node): Node {.discardable.} =
  var inSize = 1
  result = list
  while true:
    var p = result
    result = nil
    var tail: Node
    var numMerges = 0
    while not p.isNil:
      numMerges += 1
      var q = p
      var pSize = 0
      for i in 0..<inSize:
        pSize += 1
        q = q.nextZ
        if q.isNil:
          break
      var qSize = inSize
      while pSize > 0 or (qSize > 0 and not q.isNil):
        var e: Node
        if pSize != 0 and (qSize == 0 or q.isNil or p.z <= q.z):
          e = p
          p = p.nextZ
          pSize -= 1
        else:
          e = q
          q = q.nextZ
          qSize -= 1
        if not tail.isNil:
          tail.nextZ = e
        else:
          result = e
        e.prevZ = tail
        tail = e
      p = q
    tail.nextZ = nil
    inSize *= 2
    if numMerges <= 1:
      break

# z-order of a point given coords and inverse of the longer side of data bbox
proc zOrder(p: Vec2f): int {.inline.} =
  # coords are transformed into non-negative 15-bit integer range
  var x = int(32767 * p.x)
  var y = int(32767 * p.y)
  x = (x or (x shl 8)) and 0x00FF00FF
  x = (x or (x shl 4)) and 0x0F0F0F0F
  x = (x or (x shl 2)) and 0x33333333
  x = (x or (x shl 1)) and 0x55555555
  y = (y or (y shl 8)) and 0x00FF00FF
  y = (y or (y shl 4)) and 0x0F0F0F0F
  y = (y or (y shl 2)) and 0x33333333
  y = (y or (y shl 1)) and 0x55555555
  return x or (y shl 1)

# interlink polygon nodes in z-order
proc indexCurve(st: Node, min: Vec2f, invSize: float) =
  for n in st.loop:
    if n.z == 0:
      n.z = zOrder((n.pos - min) * invSize)
    n.prevZ = n.prev
    n.nextZ = n.next
  st.prevZ.nextZ = nil
  st.prevZ = nil
  sortLinked(st)

# check whether a polygon node forms a valid ear with adjacent nodes
proc isEar(ear: Node): bool =
  let a = ear.prev.pos
  let b = ear.pos
  let c = ear.next.pos

  if area(a, b, c) <= 0: # reflex, can't be an ear
    return false

  # now make sure we don't have other points inside the potential ear
  var p = ear.next.next
  while p != ear.prev:
    if p.pos.inside(a, b, c) and area(p.prev.pos, p.pos, p.next.pos) <= 0:
      return false
    p = p.next
  return true

proc isEarHashed(ear: Node, min: Vec2f, invSize: float): bool =
  let a = ear.prev.pos
  let b = ear.pos
  let c = ear.next.pos

  if area(a, b, c) <= 0: # reflex, can't be an ear
    return false

  # triangle bbox; min & max are calculated like this for speed
  let minT = min(a, b, c)
  let maxT = max(a, b, c)
  # z-order range for the current triangle bbox;
  let minZ = zOrder((minT - min) * invSize)
  let maxZ = zOrder((maxT - min) * invSize)

  var p = ear.prevZ
  var n = ear.nextZ

  # look for points inside the triangle in both directions
  while not p.isNil and p.z >= minZ and not n.isNil and n.z <= maxZ:
    if p != ear.prev and p != ear.next and
      p.pos.inside(a, b, c) and area(p.prev.pos, p.pos, p.next.pos) <= 0:
        return false
    p = p.prevZ

    if n != ear.prev and n != ear.next and
      n.pos.inside(a, b, c) and area(n.prev.pos, n.pos, n.next.pos) <= 0:
        return false
    n = n.nextZ

  # look for remaining points in decreasing z-order
  while not p.isNil and p.z >= minZ:
    if p != ear.prev and p != ear.next and
      p.pos.inside(a, b, c) and area(p.prev.pos, p.pos, p.next.pos) <= 0:
        return false
    p = p.prevZ
  
  # look for remaining points in increasing z-order
  while not n.isNil and n.z <= maxZ:
    if n != ear.prev and n != ear.next and
      n.pos.inside(a, b, c) and area(n.prev.pos, n.pos, n.next.pos) <= 0:
        return false
    n = n.nextZ

  return true

# go through all polygon nodes and cure small local self-intersections
proc cureLocalIntersections(start: Node, triangles: var Triangles): Node =
  var start = start
  var p = start
  while true:
    let a = p.prev
    let b = p.next.next

    if a.pos != b.pos and intersects(a.pos, p.pos, p.next.pos, b.pos) and
      locallyInside(a, b) and locallyInside(b, a):
        triangles.add((a.i, p.i, b.i))
      
        # remove two nodes involved
        removeNode(p)
        removeNode(p.next)

        start = b
        p = start
    p = p.next
    if p == start:
      break
  return filterPoints(p)

# check if a polygon diagonal intersects any polygon segments
proc intersectsPolygon(a, b: Node): bool =
  for p in a.loop:
    if p.i != a.i and p.next.i != a.i and p.i != b.i and p.next.i != b.i and
      intersects(p.pos, p.next.pos, a.pos, b.pos):
        return true

# check if the middle point of a polygon diagonal is inside the polygon
proc middleInside(a, b: Node): bool =
  let px = (a.pos.x + b.pos.x) / 2.0
  let py = (a.pos.y + b.pos.y) / 2.0
  for p in a.loop:
    if ((p.pos.y > py) != (p.next.pos.y > py)) and p.next.pos.y != p.pos.y and
      (px < (p.next.pos.x - p.pos.x) * (py - p.pos.y) / (p.next.pos.y - p.pos.y) + p.pos.x):
        result = not result

# check if a diagonal between two polygon nodes is valid (lies in polygon interior)
proc isValidDiagonal(a, b: Node): bool =
  # doesn't intersect other edges
  a.next.i != b.i and a.prev.i != b.i and not intersectsPolygon(a, b) and
    # locally visible
    (locallyInside(a, b) and locallyInside(b, a) and middleInside(a, b) and
      # does not create opposite-facing sectors
      # TODO: check this conditions
      (area(a.prev.pos, a.pos, b.prev.pos) != 0.0 or area(a.pos, b.prev.pos, b.pos) != 0.0) or
      a.pos == b.pos and 
        area(a.prev.pos, a.pos, a.next.pos) < 0.0 and area(b.prev.pos, b.pos, b.next.pos) < 0.0)

proc earcutLinked(ear: Node, triangles: var Triangles, min: Vec2f, invSize: float, pass: int = 0)

# try splitting polygon into two and triangulate them independently
proc splitEarcut(start: Node, triangles: var Triangles, min: Vec2f, invSize: float) =
  # look for a valid diagonal that divides the polygon into two
  var a = start
  while true:
    var b = a.next.next
    while b != a.prev:
      if a.i != b.i and isValidDiagonal(a, b):
        # split the polygon in two by the diagonal
        var c = splitPolygon(a, b)

        # filter colinear points around the cuts
        a = filterPoints(a, a.next)
        c = filterPoints(c, c.next)

        # run earcut on each half
        earcutLinked(a, triangles, min, invSize)
        earcutLinked(c, triangles, min, invSize)
        return
      b = b.next
    a = a.next
    if a == start:
      break

# main ear slicing loop which triangulates a polygon (given as a linked list)
proc earcutLinked(ear: Node, triangles: var Triangles, min: Vec2f, invSize: float, pass: int = 0) =
  var ear = ear
  if ear.isNil:
    return

  # interlink polygon nodes in z-order
  if pass == 0 and invSize > 0.0:
    indexCurve(ear, min, invSize)

  # iterate through ears, slicing them one by one
  var stop = ear
  while ear.prev != ear.next:
    let prev = ear.prev
    let next = ear.next

    if (if invSize > 0.0: isEarHashed(ear, min, invSize) else: isEar(ear)):
      # cut off the triangle
      triangles.add((prev.i, ear.i, next.i))
      removeNode(ear)
    
      # skipping the next vertex leads to less sliver triangles
      ear = next.next
      stop = next.next
      continue

    ear = next

    # if we looped through the whole remaining polygon and can't find any more ears
    if ear == stop:
      # try filtering points and slicing again
      if pass == 0:
        earcutLinked(filterPoints(ear), triangles, min, invSize, 1)
      # if this didn't work, try curing all small self-intersections locally
      elif pass == 1:
        ear = cureLocalIntersections(filterPoints(ear), triangles)
        earcutLinked(ear, triangles, min, invSize, 2)
      # as a last resort, try splitting the remaining polygon into two
      elif pass == 2:
        splitEarcut(ear, triangles, min, invSize)
      break

proc earcut*(data: openArray[Vec2f], holeIndices: openArray[int] = []): Triangles =
  let hasHoles = holeIndices.len > 0
  let outerLen = if hasHoles: holeIndices[0] else: data.len
  var outerNode = linkedList(data, 0, outerLen, true)
  result = newSeqOfCap[(int, int, int)](max(0, data.len - 2))
  if outerNode.isNil or outerNode.next == outerNode.prev:
    return

  if hasHoles:
    eliminateHoles(data, holeIndices, outerNode)
  
  var bbox: BBox2f
  var invSize: float
  if data.len > 80:
    bbox = data.bbox
    invSize = max(bbox.width, bbox.height)
    invSize = if invSize > 0: 1 / invSize else: 0.0
  
  earcutLinked(outerNode, result, bbox.min, invSize)