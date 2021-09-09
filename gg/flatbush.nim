# Based on https://github.com/mourner/flatbush

import gg/vectors
import algorithm
import heapqueue
import math

type
  QueueEntry = tuple[index: int, leaf: bool, dist: float]
  BoxEntry = tuple[index: int, bbox: BBox2f]
  Flatbush* = object
    numItems: int
    nodeSize: int
    levelBounds: seq[int]
    boxes: seq[BoxEntry]
    bounds: BBox2f
    pos: int

  Intersection* = tuple[p: Vec2f, s1, s2: int]

proc `<`(a, b: QueueEntry): bool = a.dist < b.dist

proc initFlatbush*(numItems: int, nodeSize: int = 16): Flatbush =
  result.numItems = numItems
  result.nodeSize = min(max(nodeSize, 2), 65535)

  # calculate the total number of nodes in the R-tree to allocate space for
  # and the index of each tree level (used in search later)
  var n = numItems
  var numNodes = n
  result.levelBounds = @[n]
  while true:
    n = int(ceil(float(n) / float(result.nodeSize)))
    numNodes += n
    result.levelBounds.add(numNodes)
    if n == 1:
      break
  
  result.boxes = newSeq[BoxEntry](numNodes)
  result.bounds = BBox2f.empty

proc add*(bush: var Flatbush, bbox: BBox2f): int {.discardable.} =
  let index = bush.pos
  bush.boxes[bush.pos] = (index, bbox)
  bush.pos += 1
  bush.bounds = bush.bounds.union(bbox)
  return index

# Fast Hilbert curve algorithm by http://threadlocalmutex.com/
# Ported from C++ https://github.com/rawrunprotected/hilbert_curves (public domain)
proc hilbert(x, y: int): int {.inline.} =
  var a = x xor y
  var b = 0xFFFF xor a
  var c = 0xFFFF xor (x or y)
  var d = x and (y xor 0xFFFF)

  var A = a or (b shr 1)
  var B = (a shr 1) xor a
  var C = ((c shr 1) xor (b and (d shr 1))) xor c
  var D = ((a and (c shr 1)) xor (d shr 1)) xor d

  (a, b, c, d) = (A, B, C, D)
  A = ((a and (a shr 2)) xor (b and (b shr 2)))
  B = ((a and (b shr 2)) xor (b and ((a xor b) shr 2)))
  C = C xor ((a and (c shr 2)) xor (b and (d shr 2)))
  D = D xor ((b and (c shr 2)) xor ((a xor b) and (d shr 2)))
  
  (a, b, c, d) = (A, B, C, D)
  A = ((a and (a shr 4)) xor (b and (b shr 4)))
  B = ((a and (b shr 4)) xor (b and ((a xor b) shr 4)))
  C = C xor ((a and (c shr 4)) xor (b and (d shr 4)))
  D = D xor ((b and (c shr 4)) xor ((a xor b) and (d shr 4)))

  (a, b, c, d) = (A, B, C, D)
  C = C xor ((a and (c shr 8)) xor (b and (d shr 8)))
  D = D xor ((b and (c shr 8)) xor ((a xor b) and (d shr 8)))

  a = C xor (C shr 1)
  b = D xor (D shr 1)

  var i0 = x xor y
  var i1 = b or (0xFFFF xor (i0 or a))

  i0 = (i0 or (i0 shl 8)) and 0x00FF00FF
  i0 = (i0 or (i0 shl 4)) and 0x0F0F0F0F
  i0 = (i0 or (i0 shl 2)) and 0x33333333
  i0 = (i0 or (i0 shl 1)) and 0x55555555

  i1 = (i1 or (i1 shl 8)) and 0x00FF00FF
  i1 = (i1 or (i1 shl 4)) and 0x0F0F0F0F
  i1 = (i1 or (i1 shl 2)) and 0x33333333
  i1 = (i1 or (i1 shl 1)) and 0x55555555

  return (i1 shl 1) or i0

proc finish*(bush: var Flatbush) =
  doAssert bush.pos == bush.numItems, "Invalid number of items"    

  if bush.numItems <= bush.nodeSize:
    bush.boxes[bush.pos] = (bush.pos, bush.bounds)
    bush.pos += 1
    return

  let (w, h) = bush.bounds.size
  const hilbertMax = float((1 shl 16) - 1)
  var hilbertValues = newSeq[int](bush.numItems)

  # map item centers into Hilbert coordinate space and calculate Hilbert values
  for i in 0..<bush.numItems:
    let center = bush.boxes[i].bbox.center - bush.bounds.min
    let x = int(hilbertMax * center.x / w)
    let y = int(hilbertMax * center.y / h)
    hilbertValues[i] = hilbert(x, y)

  # sort items by their Hilbert value (for packing later)
  bush.boxes.sort do (b1, b2: BoxEntry) -> int:
    cmp(hilbertValues[b1.index], hilbertValues[b2.index])

  var pos = 0
  for en in bush.levelBounds[0..^2]:
    # generate a parent node for each block of consecutive <nodeSize> nodes
    while pos < en:
      let nodeIndex = pos
      # calculate bbox for the new node
      var nodeBounds = BBox2f.empty
      for i in 0..<bush.nodeSize:
        nodeBounds = nodeBounds.union(bush.boxes[pos].bbox)
        pos += 1
        if pos == en:
          break

      # add the new node to the tree data
      bush.boxes[bush.pos] = (nodeIndex, nodeBounds)
      bush.pos += 1

iterator search*[T](bush: Flatbush, obj: T): BoxEntry =
  doAssert bush.pos == bush.boxes.len, "Data not yet indexed - call index.finish()"

  let bbox = obj.bbox

  var nodeIndex = bush.boxes.len - 1
  var queue = newSeq[int]()
  while true:
    # find the end index of the node
    let en = min(nodeIndex + bush.nodeSize, bush.levelBounds[bush.levelBounds.upperBound(nodeIndex)])

    #echo "nodeIndex ", nodeIndex, ", en ", en
    # search through child nodes
    for pos in nodeIndex..<en:
      let entry = bush.boxes[pos]
      
      if bbox.intersects(entry.bbox):
        if nodeIndex < bush.numItems:
          yield entry # leaf item
        else:
          queue.add(entry.index) # node; add it to the search queue
    
    if queue.len > 0:
      nodeIndex = queue.pop()
    else:
      break
  
iterator neighbors*(bush: Flatbush, p: Vec2f, maxDistance: float = Inf): BoxEntry =
  doAssert bush.pos == bush.boxes.len, "Data not yet indexed - call index.finish()"

  var nodeIndex = bush.boxes.len - 1
  var queue = initHeapQueue[QueueEntry]()
  let maxDistanceSq = maxDistance * maxDistance
  block outer:
    while true:
      # find the end index of the node
      let en = min(nodeIndex + bush.nodeSize, bush.levelBounds.upperBound(nodeIndex))

      # add child nodes to the queue
      for pos in nodeIndex..<en:
        let entry = bush.boxes[pos]
        let dist = p.distSq(entry.bbox)

        if nodeIndex < bush.numItems:
          queue.push((pos, true, dist))
        else:
          queue.push((entry.index, false, dist))

      # pop items from the queue
      while queue.len > 0 and queue[0].leaf:
        if queue[0].dist > maxDistanceSq:
          break outer
        yield bush.boxes[queue.pop().index]
      
      if queue.len > 0:
        nodeIndex = queue.pop().index
      else:
        break
      
iterator bush*(lines: openArray[Segment2f]): Intersection =
  var index = initFlatbush(lines.len)
  for line in lines:
    index.add(line.bbox)
  index.finish()
  for i, line in lines:
    for entry in index.search(line):
      if entry.index > i:
        var pos: Vec2f
        if line.intersects(lines[entry.index], pos):
          yield (pos, i, entry.index)