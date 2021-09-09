# Based on ideas from
# https://stackoverflow.com/questions/41946007/efficient-and-well-explained-implementation-of-a-quadtree-for-2d-collision-det
#
# Denis Olshin, 2021

import gg/[vectors, smallseq, freeseq]
import math

import random
import ropes
import strutils
import strformat
import times

type
  ## An element that can provide a vector
  Vec2Provider* = concept q
    q.pos is Vec2

  ## An element that can provide a bounding box
  BBoxProvider* = concept q
    q.bbox is BBox2

  ## An element that can be stored in a quadtree. It can take multiple
  ## forms, depending on the level of control desired
  Quadable* = concept q
    `==`(q, q) is bool
    q is BBox2 | Vec2 | BBoxProvider | Vec2Provider
  
  Index* = int32
  QuadEltNode = tuple[elem, next: Index]
  QuadNode = tuple[first, count: Index]
  Quadtree*[E] = object
    data: fseq[E] # Using freeseq to reuse deleted items
    bounds: BBox2i
    eltNodes: fseq[QuadEltNode]
    nodes: seq[QuadNode]
    freeNode: Index
    scale: float # Coords multiplied by this number before converting to integers
    maxDepth: int
    maxChildren: int

template bbox(elem: Quadable): BBox2 =
  when type(elem) is BBox2:
    elem
  elif type(elem) is Vec2Provider:
    elem.pos.bbox
  else:
    elem.bbox

proc setBounds(tree: var Quadtree, bounds: BBox2) =
  var bounds = (bounds * tree.scale).ibox
  bounds.max = max(bounds.max, bounds.min + (1, 1)) # Make bounds at least 1x1
  let size = max(bounds.width, bounds.height).nextPowerOfTwo
  tree.bounds = bbox(bounds.min, bounds.min + (size, size)) # Ensure bounds are always square and power of 2

## Initializes Quadtree
## bounds is the initial bounds (if known)
## maxDepth is the depth limit
## maxChildren is the count of leaf nodes when the split happens
## minDist is the smallest unit size (internally floating point coords are converted to int units for speed)
proc initQuadtree*[E: Quadable](bounds: BBox2f = bbox((-100.0, -100.0), (100.0, 100.0)), maxChildren = 10, maxDepth = int.high, minDist = 1e-8): Quadtree[E] =
  assert minDist > 0.0, "Minimal distance should be larger than zero"
  result.data = newFSeq[E]()
  result.eltNodes = newFSeq[QuadEltNode]()
  result.nodes = @[(first: -1.Index, count: 0.Index)]
  result.maxDepth = maxDepth
  result.maxChildren = maxChildren
  result.freeNode = -1
  result.scale = 1.0 / minDist
  result.setBounds(bounds)

proc isLeaf(tree: Quadtree, i: Index): bool = tree.nodes[i].count != -1

iterator leaves(tree: Quadtree, within: BBox2i): (Index, int, BBox2i) =
  var queue = @@[(i: 0.Index, depth: 0, bounds: tree.bounds)] # Using smallseq to prevent unnecessary allocations
  while queue.len > 0:
    let node = queue.pop()
    if tree.isLeaf(node.i):
      yield node
    else:
      let first = tree.nodes[node.i].first
      let mid = node.bounds.center
      if within.min.y <= mid.y:
        if within.min.x <= mid.x: # TL
          queue.push((i: first, depth: node.depth + 1, bounds: bbox(node.bounds.min, mid)))
        if within.max.x > mid.x:  # TR
          queue.push((i: first + 1, depth: node.depth + 1, bounds: bbox((mid.x, node.bounds.min.y), (node.bounds.max.y, mid.y))))
      if within.max.y > mid.y:
        if within.min.x <= mid.x: # BL
          queue.push((i: first + 2, depth: node.depth + 1, bounds: bbox((node.bounds.min.x, mid.y), (mid.x, node.bounds.max.y))))
        if within.max.x > mid.x:  # BR
          queue.push((i: first + 3, depth: node.depth + 1, bounds: bbox(mid, node.bounds.max)))

iterator elements[E](tree: Quadtree[E], leaf: Index): Index =
  var i = tree.nodes[leaf].first
  while i != -1:
    let node = tree.eltNodes[i]
    let next = node.next
    yield node.elem
    i = next

proc addNode(tree: var Quadtree): Index =
  if tree.freeNode == -1:
    result = tree.nodes.len.Index
    tree.nodes.setLen(result + 4)
    tree.nodes[result].first = -1
    tree.nodes[result + 1].first = -1
    tree.nodes[result + 2].first = -1
    tree.nodes[result + 3].first = -1
  else:
    result = tree.freeNode
    tree.freeNode = tree.nodes[result].first
    tree.nodes[result].first = -1
    # all other childs should be reset already    

## Find all elements within specified bounding box
iterator find*[E](tree: Quadtree[E], bbox: BBox2): (Index, E) =
  var ibox = (bbox * tree.scale).ibox.intersect(tree.bounds)
  if ibox.width >= 0 and ibox.height >= 0:
    for leaf, _, _ in tree.leaves(ibox):
      for elem in tree.elements(leaf):
        yield (elem, tree.data[elem])

## Find all elements within radius r around point p
iterator find*[E](tree: Quadtree[E], p: Vec2, r: float = 0.0): (Index, E) =
  let rSq = r * r
  for id, elem in tree.find(p.bbox(r)):
    if p.distSq(elem.bbox) <= rSq:
      yield (id, elem)

## Find all elements within radius r around point p (without actual filtering by distances, so it can return false positives)
iterator findApprox*[E](tree: Quadtree[E], p: Vec2, r: float = 0.0): (Index, E) =
  for id, elem in tree.find(p.bbox(r)):
    yield (id, elem)

## Find element's index (or -1 if not found)
proc find*[E](tree: Quadtree[E], elem: E): Index =
  let mid = elem.bbox.center
  for id, e in tree.find((mid, mid)):
    if e == elem:
      return id
  return -1

## Expand tree so it fully covers specified bounding box
proc cover*(tree: var Quadtree, bounds: BBox2) =
  if tree.bounds == BBox2i.empty:
    # Uninitialized bounds: just set
    tree.setBounds(bounds)
  else:
    var bounds = (bounds * tree.scale).ibox # Convert to integer coords
    # Quadruple in size until we fit specified area
    while not bounds.inside(tree.bounds):
      var idx = 0
      var node = tree.addNode()
      let size = tree.bounds.size
      if bounds.max.y > tree.bounds.max.y: # -> bottom
        if bounds.max.x > tree.bounds.max.x: # -> bottom-right (root now at top-left)
          idx = 0
          tree.bounds = bbox(tree.bounds.min, tree.bounds.max + size)
        else: # -> bottom-left (root now at top-right)
          idx = 1
          tree.bounds = bbox(tree.bounds.min - (size.x, 0), tree.bounds.max + (0, size.y))
      else: # -> top
        if bounds.max.x > tree.bounds.max.x: # -> top-right (root now at bottom-left)
          idx = 2
          tree.bounds = bbox(tree.bounds.min - (0, size.y), tree.bounds.max + (size.x, 0))
        else: # -> top-left (root now at bottom-right)
          idx = 3
          tree.bounds = bbox(tree.bounds.min - size, tree.bounds.max)
      tree.nodes[node + idx] = tree.nodes[0]
      tree.nodes[0] = (first: node, count: -1.Index)
      

## Expand tree so it fully covers specified point
proc cover*(tree: var Quadtree, p: Vec2) = tree.cover(p.bbox)

# Forward declaration of split
proc split(tree: var Quadtree, leaf: Index, depth: int, bounds: BBox2i)

proc addElement(tree: var Quadtree, leaf: Index, depth: int, bounds: BBox2i, elem: Index) =
  let node = tree.eltNodes.add((elem: elem, next: tree.nodes[leaf].first))
  let count = tree.nodes[leaf].count + 1
  tree.nodes[leaf] = (first: node.Index, count: count.Index)
  if count >= tree.maxChildren and depth < tree.maxDepth and bounds.width > 1 and bounds.height > 1:
    tree.split(leaf, depth, bounds)

# Turns leaf node into 4
proc split(tree: var Quadtree, leaf: Index, depth: int, bounds: BBox2i) =
  let branch = tree.addNode()
  let mid = bounds.center
  for elem in tree.elements(leaf):
    tree.eltNodes.del(elem)
    let bbox = (tree.data[elem].bbox * tree.scale).ibox
    if bbox.min.y <= mid.y:
      if bbox.min.x <= mid.x: # TL
        tree.addElement(branch, depth + 1, bbox(bounds.min, mid), elem)
      if bbox.max.x > mid.x:  # TR
        tree.addElement(branch + 1, depth + 1, bbox((mid.x, bounds.min.y), (bounds.max.y, mid.y)), elem)
    if bbox.max.y > mid.y:
      if bbox.min.x <= mid.x: # BL
        tree.addElement(branch + 2, depth + 1, bbox((bounds.min.x, mid.y), (mid.x, bounds.max.y)), elem)
      if bbox.max.x > mid.x:  # BR
        tree.addElement(branch + 3, depth + 1, bbox(mid, bounds.max), elem)
  tree.nodes[leaf] = (first: branch, count: -1.Index) # Turn leaf into branch

## Add single element (and return its index)
proc add*[E: Quadable](tree: var Quadtree[E], elem: E): Index {.discardable.} =
  let bbox = elem.bbox
  tree.cover(bbox)
  result = tree.data.add(elem).Index
  for leaf, depth, bounds in tree.leaves((bbox * tree.scale).ibox):
    tree.addElement(leaf, depth, bounds, result)

## Add multiple elements at once
proc add*[E: Quadable](tree: var Quadtree[E], els: varargs[E]) =
  if els.len == 0:
    return
  var bounds = els[0].bbox
  for elem in els:
    bounds = bounds.union(elem.bbox)
  tree.cover(bounds)  
  for elem in els:
    let id = tree.data.add(elem)
    let bbox = elem.bbox
    for leaf, depth, bounds in tree.leaves((bbox * tree.scale).ibox):
      tree.addElement(leaf, depth, bounds, id)

#[
  var i = tree.nodes[leaf].first
  while i != -1:
    let node = tree.eltNodes[i]
    let next = node.next
    yield node.elem
    i = next
]#
## Delete element by its index (use find if index is unknown)
proc del*[E](tree: var Quadtree[E], i: Index) =
  let bbox = (tree.data[i].bbox * tree.scale).ibox
  for leaf, depth, bounds in tree.leaves(bbox):
    var cur = tree.nodes[leaf].first
    var prev = -1
    while cur != -1:
      let node = tree.eltNodes[cur]
      if node.elem == i:
        if prev == -1: # first one
          tree.nodes[leaf].first = node.next
        else:
          tree.eltNodes[prev] = (elem: tree.eltNodes[prev].elem, next: node.next)
        tree.eltNodes.del(cur)
        tree.nodes[leaf].count -= 1
        break
      prev = cur
      cur = node.next
  tree.data.del(i)

proc cleanup*(tree: var Quadtree) =
  if tree.isLeaf(0):
    return
  var queue = @@[0]
  while queue.len > 0:
    let index = queue.pop()
    let first = tree.nodes[index].first
    var emptyLeavesCnt = 0
    for i in 0..3:
      let child = first + i
      let childCount = tree.nodes[child].count
      if childCount == 0: # empty leaf
        emptyLeavesCnt += 1
      elif childCount == -1: # branch
        queue.push(child)
    if emptyLeavesCnt == 4:
      tree.nodes[first].first = tree.freeNode
      tree.freeNode = first
      tree.nodes[index] = (first: -1.Index, count: 0.Index)

iterator items*[E](tree: Quadtree[E]): E =
  for elem in tree.data.items:
    yield elem

proc items*[E](tree: Quadtree[E]): seq[E] = tree.data.items

proc len*(tree: Quadtree): int = tree.data.len

proc subtreeToStr(tree: Quadtree, i: Index, depth: int, bounds: BBox2i, s: var Rope) =
  s.add("  ".repeat(depth))
  s.add(&"#{i} (first: {tree.nodes[i].first}, count: {tree.nodes[i].count}), bounds: {bounds.bbox / tree.scale}, isLeaf: {tree.nodes[i].count != -1}\n")
  if tree.isLeaf(i):
    # Print elements
    for elem in tree.elements(i):
      s.add("  ".repeat(depth + 1))
      let pos = tree.data[elem].bbox
      s.add(&"={elem} ({pos})\n")
  else:
    # Print child nodes
    let mid = bounds.center
    tree.subtreeToStr(tree.nodes[i].first, depth + 1, bbox(bounds.min, mid), s)
    tree.subtreeToStr(tree.nodes[i].first + 1, depth + 1, bbox((mid.x, bounds.min.y), (bounds.max.y, mid.y)), s)
    tree.subtreeToStr(tree.nodes[i].first + 2, depth + 1, bbox((bounds.min.x, mid.y), (mid.x, bounds.max.y)), s)
    tree.subtreeToStr(tree.nodes[i].first + 3, depth + 1, bbox(mid, bounds.max), s)

proc `$`*[E: Quadable](tree: Quadtree[E]): string =
  var s = rope("Quadtree(\n")
  #s.add(&"  data: {tree.data}")
  s.add(&"  maxChildren: {tree.maxChildren},\n")
  s.add(&"  maxDepth: {tree.maxDepth},\n")
  s.add(&"  scale: {tree.scale},\n")
  s.add(&"  freeNode: {tree.maxChildren},\n")
  s.add(&"  root:\n")
  tree.subtreeToStr(0.Index, 2, tree.bounds, s)
  s.add(")")
  return $s