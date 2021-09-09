# TODO: incomplete

import gg/vectors
import gg/delaunator/delaunator
import math
import sequtils

type
  Voronator = ref object
    del*: Delaunator
    bounds: BBox2f
    circumcenters: seq[Vec2f]
    vectors: seq[Vec2f]

proc voronoi*(del: Delaunator, bounds: BBox2f = bbox((-1e9, -1e9), (1e9, 1e9))): Voronator =
  result = Voronator(
    del: del,
    bounds: bounds,
    circumcenters: newSeq[Vec2f](del.triangleCnt),
    vectors: newSeq[Vec2f](del.coords.len),
  )

  for i in 0..<del.triangleCnt:
    let t1 = del.triangles[i * 3]
    let t2 = del.triangles[i * 3 + 1]
    let t3 = del.triangles[i * 3 + 2]
    let p1 = del.coords[t1]
    let p2 = del.coords[t2]
    let p3 = del.coords[t3]

    let d = p2 - p1
    let e = p3 - p1
    let ab = 2.0 * (d ^ e)
    if ab ~= 0.0:
      # degenerate case (collinear diagram)
      # almost equal points (degenerate triangle)
      # the circumcenter is at the infinity, in a
      # direction that is:
      # 1. orthogonal to the halfedge.
      var a = 1e9
      # 2. points away from the center; since the list of triangles starts
      # in the center, the first point of the first triangle
      # will be our reference
      let r = del.triangles[0]
      a *= sgn((del.coords[r] - p1) ^ e).float
      result.circumcenters[i] = (p1 + p3) / 2.0 + (-a * e.x, a * e.y)
    else:
      let bl = d.lenSq
      let cl = e.lenSq
      result.circumcenters[i] = p1 + (e.y * bl - d.y * cl, d.x * cl - e.x * bl) / ab

  # Compute exterior cell rays.
  let h = del.hull[^1]
  var (i0, i1) = (0, h * 2)
  var (p0, p1) = (Vec2f.zero, del.coords[h])
  for i, h in del.hull:
    (i0, p0) = (i1, p1)
    (i1, p1) = (h * 2, del.coords[h])
    result.vectors[i1] = (p0.y - p1.y, p1.x - p0.x)
    result.vectors[i0 + 1] = (p0.y - p1.y, p1.x - p0.x)

proc edgecode(v: Voronator, p: Vec2f): int =
  (if p.x == v.bounds.min.x: 0b0001 elif p.x == v.bounds.max.x: 0b0010 else: 0b0000) or
  (if p.y == v.bounds.min.y: 0b0100 elif p.y == v.bounds.max.y: 0b1000 else: 0b0000)

proc regioncode(v: Voronator, p: Vec2f): int =
  (if p.x < v.bounds.min.x: 0b0001 elif p.x > v.bounds.max.x: 0b0010 else: 0b0000) or
  (if p.y < v.bounds.min.y: 0b0100 elif p.y > v.bounds.max.y: 0b1000 else: 0b0000)

proc project(vr: Voronator, p0, v: Vec2f, p: var Vec2f): bool =
  var t = Inf
  let bb = vr.bounds
  if v.y < 0.0: # top
    if p0.y <= bb.min.y: return false
    let c = (bb.min.y - p0.y) / v.y
    if c < t:
      t = c
      p = (p0.x + t * v.x, bb.min.y)
  elif v.y > 0.0: # bottom
    if p0.y >= bb.max.y: return false
    let c = (bb.max.y - p0.y) / v.y
    if c < t:
      t = c
      p = (p0.x + t * v.x, bb.max.y)
  if v.x > 0.0: # right
    if p0.x >= bb.max.x: return false
    let c = (bb.max.x - p0.x) / v.x
    if c < t:
      t = c
      p = (bb.max.x, p0.y + t * v.y)
  elif v.x < 0.0: # left
    if p0.x <= bb.min.x: return false
    let c = (bb.min.x - p0.x) / v.x
    if c < t:
      t = c
      p = (bb.min.x, p0.y + t * v.y)
  return true

proc contains(vr: Voronator, i: int, p: Vec2f): bool =
  

proc edge(vr: Voronator, i, e0, e1: int, P: var seq[Vec2f], j: int): int =
  let bb = vr.bounds
  var j = j
  var e0 = e0
  while e0 != e1:
    var p: Vec2f
    case e0:
    of 0b0101: e0 = 0b0100; continue # top-left
    of 0b0100: e0 = 0b0110; p = (bb.max.x, bb.min.y); break # top
    of 0b0110: e0 = 0b0010; continue # top-right
    of 0b0010: e0 = 0b1010; p = bb.max; break # right
    of 0b1010: e0 = 0b1000; continue # bottom-right
    of 0b1000: e0 = 0b1001; p = (bb.min.x, bb.max.y); break # bottom
    of 0b1001: e0 = 0b0001; continue # bottom-left
    of 0b0001: e0 = 0b0101; p = bb.min; break # left
    else: discard
    if (j >= P.len or P[j] != p) and vr.contains(i, p):
      P.insert(p, j)
      j += 1
  if P.len > 4:
    var i = 0
    while i < P.len:
      let j = (i + 1) mod P.len
      let k = (i + 2) mod P.len
      if (P[i].x == P[j].x and P[j].x == P[k].x) or
        (P[i].y == P[j].y and P[j].y == P[k].y):
          P.delete(j)
      else:
        i += 1
  return j