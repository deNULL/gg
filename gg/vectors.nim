import math
import algorithm
import sequtils

# TODO: add a way to configure eps/precision?
type
  # 64 bit precision
  fnum* = float
  inum* = int

const eps* = 1e-10

type
  # Should include: BBox2f, Triangle2f, Rect2f, Path2f (can be problematic), Polygon2f
  Region2f* = concept e
    bbox(e) is BBox2f
    inside(Vec2f, e) is bool
    dist(Vec2f, e) is fnum
    distSq(Vec2f, e) is fnum

  # Vectors 

  Vec2*[T: SomeNumber] = tuple[x, y: T]
  Vec2f* = Vec2[fnum]
  Vec2i* = Vec2[inum]

  Vec3*[T: SomeNumber] = tuple[x, y, z: T]
  Vec3f* = Vec3[fnum]
  Vec3i* = Vec3[inum]

  Vec4*[T: SomeNumber] = tuple[x, y, z, w: T]
  Vec4f* = Vec4[fnum]
  Vec4i* = Vec4[inum]

  Polar* = tuple[r, theta: fnum]
  
  # Shapes

  Segment*[T] = object
    st*, en*: T
  Segment2f* = Segment[Vec2f]
  Segment2i* = Segment[Vec2i]

  Triangle*[T] = array[3, T]
  Triangle2f* = Triangle[Vec2f]
  Triangle3f* = Triangle[Vec3f]
  
  Rect2f* = object
    origin*, size*: Vec2f

  Path2f* = seq[Vec2f] # Can sometimes mean "polygon without holes"
  Polygon2f* = object
    contour: Path2f
    holes: seq[Path2f]

  Circle2f* = object
    center*: Vec2f
    r: fnum

  # Bounding boxes

  BBox*[T] = object
    min*, max*: T

  BBox2*[T: SomeNumber] = BBox[Vec2[T]]
  BBox2f* = BBox[Vec2f]
  BBox2i* = BBox[Vec2i]

  BBox3*[T: SomeNumber] = BBox[Vec3[T]]
  BBox3f* = BBox[Vec3f]
  BBox3i* = BBox[Vec3i]

  # Matrices

  Mat3x3*[T] = array[3, array[3, T]]
  Mat3x3f*[T] = array[3, array[3, fnum]]

  Orient* = enum
    orCCW = -1, orCollinear = 0, orCW = 1

{.push inline.}

proc rect*(origin, size: Vec2f): Rect2f = Rect2f(origin: origin, size: size)
proc bbox*[T](min, max: T): BBox[T] = BBox[T](min: min, max: max)
proc `..`*[T](min, max: Vec2[T]): BBox2[T] = BBox2[T](min: min, max: max)
proc `..`*[T](min, max: Vec3[T]): BBox3[T] = BBox3[T](min: min, max: max)
proc segment*[T](st, en: T): Segment[T] = Segment[T](st: st, en: en)
proc `->`*[T](st, en: T): Segment[T] = Segment[T](st: st, en: en)

proc `$`*(p: Vec2): string = "(" & $p.x & ", " & $p.y & ")"
proc `$`*(p: Vec3): string = "(" & $p.x & ", " & $p.y & ", " & $p.z & ")"
proc `$`*(bbox: BBox): string = $bbox.min & ".." & $bbox.max
proc `$`*(segm: Segment): string = $segm.st & "->" & $segm.en

# Scalar utilities

proc mix*(p1: fnum, p2: fnum, t: fnum): fnum = (1.0 - t) * p1 + t * p2
proc `~=`*(a, b: fnum): bool = abs(a - b) <= eps
template `!~=`*(a, b: typed): bool = not a ~= b

# 2D Vectors

template zero*(T: typedesc[Vec2f]): Vec2f = (0.0, 0.0)
template one*(T: typedesc[Vec2f]): Vec2f = (1.0, 1.0)
template down*(T: typedesc[Vec2f]): Vec2f = (0.0, -1.0)
template up*(T: typedesc[Vec2f]): Vec2f = (0.0, 1.0)
template left*(T: typedesc[Vec2f]): Vec2f = (-1.0, 0.0)
template right*(T: typedesc[Vec2f]): Vec2f = (1.0, 0.0)
template inf*(T: typedesc[Vec2f]): Vec2f = (Inf, Inf)

proc lenSq*[T](p: Vec2[T]): T = p.x * p.x + p.y * p.y
proc lenManh*[T](p: Vec2[T]): T = abs(p.x) + abs(p.y)

# NOTE: it's called `length`, not `len` - for consistency with path.length
# (path.len means the number of points, not the length of the polyline)
# It's also longer to type than lenSq, which should be preferred when possible :)
proc length*(p: Vec2f): fnum = hypot(p.x, p.y)

proc `^`*[T](p1: Vec2[T], p2: Vec2[T]): T = p1.x * p2.y - p1.y * p2.x
proc `*`*[T](p1: Vec2[T], p2: Vec2[T]): T = p1.x * p2.x + p1.y * p2.y
proc `-`*(p1: Vec2, p2: Vec2): Vec2 = (p1.x - p2.x, p1.y - p2.y)
proc `-`*(p: Vec2): Vec2 = (-p.x, -p.y)
proc `-=`*[T](p1: var Vec2, p2: Vec2) = p1.x -= p2.x; p1.y -= p2.y
proc `+`*[T](p1: Vec2[T], p2: Vec2[T]): Vec2[T] = (p1.x + p2.x, p1.y + p2.y)
proc `+=`*[T](p1: var Vec2[T], p2: Vec2[T]) = p1.x += p2.x; p1.y += p2.y
proc `*`*[T](p: Vec2[T], s: T): Vec2[T] = (p.x * s, p.y * s)
proc `*`*[T](s: T, p: Vec2[T]): Vec2[T] = (p.x * s, p.y * s)
proc `*=`*[T](p: var Vec2[T], s: T) = p.x *= s; p.y *= s
proc `/`*[T](p: Vec2[T], s: T): Vec2[T] = (p.x / s, p.y / s)
proc `/=`*[T](p: var Vec2[T], s: T) = p.x /= s; p.y /= s
proc norm*(p: Vec2f): Vec2f = p / p.length
proc perp*[T](p: Vec2[T]): Vec2[T] = (-p.y,  p.x)
proc min*[T](p1: Vec2[T], p2: Vec2[T]): Vec2[T] = (min(p1.x, p2.x), min(p1.y, p2.y))
proc min*[T: Vec2](ps: varargs[T]): T =
  var res: T = T.inf
  for p in ps:
    res = (min(res.x, p.x), min(res.y, p.y))
  return res
proc max*[T](p1: Vec2[T], p2: Vec2[T]): Vec2[T] = (max(p1.x, p2.x), max(p1.y, p2.y))
proc max*[T: Vec2](ps: varargs[T]): T =
  var res: T = -T.inf
  for p in ps:
    res = (max(res.x, p.x), max(res.y, p.y))
  return res
proc avg*(p1: Vec2f, p2: Vec2f): Vec2f = ((p1.x + p2.x) * 0.5, (p1.y + p2.y) * 0.5)
proc avg*(p1: Vec2i, p2: Vec2i): Vec2i = ((p1.x + p2.x) shr 1, (p1.y + p2.y) shr 1)
proc avg*(ps: varargs[Vec2f]): Vec2f =
  var res: Vec2f = Vec2f.zero
  for p in ps:
    res = (res.x + p.x, res.y + p.y)
  return res / fnum(ps.len)
proc round*(p: Vec2f): Vec2f = (round(p.x), round(p.y))
proc floor*(p: Vec2f): Vec2f = (floor(p.x), floor(p.y))
proc ceil*(p: Vec2f): Vec2f = (ceil(p.x), ceil(p.y))
proc ivec*(p: Vec2f): Vec2i = (int(p.x), int(p.y))
proc fvec*(p: Vec2i): Vec2f = (fnum(p.x), fnum(p.y))
proc aspect*(p: Vec2f): fnum = p.x / p.y

proc distSq*[T](p1: Vec2[T], p2: Vec2[T]): T = (p1 - p2).lenSq
proc dist*(p1: Vec2f, p2: Vec2f): fnum = (p1 - p2).length
proc distManh*[T](p1: Vec2[T], p2: Vec2[T]): T = abs(p1.x - p2.x) + abs(p1.y - p2.y)

proc angle*(p: Vec2f): fnum = arctan2(p.y, p.x)
proc angle*(p1: Vec2f, p2: Vec2f): fnum = (p1 - p2).angle

# monotonically increases with real angle, but doesn't need expensive trigonometry
proc pseudoAngle*(p: Vec2f): fnum =
  if p.x == 0.0 and p.y == 0.0: return 0.0
  let v = p.x / (abs(p.x) + abs(p.y))
  (if p.y > 0.0: 3.0 - v else: 1.0 + v) / 4.0 # [0..1]

proc `~=`*(p1: Vec2f, p2: Vec2f): bool = abs(p1.x - p2.x) <= eps and abs(p1.y - p2.y) <= eps

# Returns _unoriented_ (minimal) angle between two vectors
proc angleBetween*(p1: Vec2f, p2: Vec2f): fnum = arccos((p1 * p2) / (p1.length * p2.length))

# Returns _oriented_ angle from first to second vector
proc angleTo*(p1: Vec2f, p2: Vec2f): fnum = arctan2(p1 ^ p2, p1 * p2)

# TODO: compare those formulas
proc angleTo2*(p1: Vec2f, p2: Vec2f): fnum =
  result = p1.angle - p2.angle
  if result > PI:
    result -= 2.0 * PI
  elif result <= -PI:
    result += 2.0 * PI

proc rot*(p: Vec2f, angle: fnum): Vec2f =
  var cs = cos(angle)
  var sn = sin(angle)
  return (cs * p.x - sn * p.y, sn * p.x + cs * p.y)

proc rot*(p1: Vec2f, p2: Vec2f, angle: fnum): Vec2f = (p1 - p2).rot(angle) + p2

proc bisect*(p1: Vec2f, p2: Vec2f): Vec2f =
  var p = p1.norm + p2.norm # TODO: not looking good
  if p.lenSq < eps:
    return (p1.y, -p1.x).norm
  else:
    return p.norm

proc polar*(p: Vec2f): Polar = (p.length, p.angle)
proc cartesian*(p: Polar): Vec2f = (p.r * cos(p.theta), p.r * sin(p.theta))
proc cartesian*(r, theta: fnum): Vec2f = (r * cos(theta), r * sin(theta))

# Allows easy conversion to 3D
proc xz*(p: Vec2f, y: fnum): Vec3 = (p.x, y, p.y)
proc xy*(p: Vec2f, z: fnum): Vec3 = (p.x, p.y, z)

proc mix*(p1: Vec2f, p2: Vec2f, t: fnum): Vec2f = (1.0 - t) * p1 + t * p2


proc area*(a, b, c: Vec2f): fnum =
  (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)
# TODO: also see robust
#[
  Formula to calculate signed area<br>
  Positive if CCW<br>
  Negative if CW<br>
  0 if collinear<br>
  <pre>
  A[P1,P2,P3]  =  (x1*y2 - y1*x2) + (x2*y3 - y2*x3) + (x3*y1 - y3*x1)
               =  (x1-x3)*(y2-y3) - (y1-y3)*(x2-x3)
  </pre>

  @private
  @param {!XY} pa  point object with {x,y}
  @param {!XY} pb  point object with {x,y}
  @param {!XY} pc  point object with {x,y}
  @return {Orientation}
]#
proc orient2d*(pa, pb, pc: Vec2f): Orient =
  let val = area(pa, pb, pc)
  if val ~= 0: orCollinear elif val > 0: orCCW else: orCW

# Segment

proc `/`*[T](segm: Segment[T], s: fnum): Segment[T] = segment(segm.st / s, segm.en / s)
proc vec*[T](s: Segment[T]): T = s.en - s.st
proc dir*[T](s: Segment[T]): T = s.vec.norm
proc len*[T](s: Segment[T]): fnum = (s.en - s.st).len
proc lenSq*[T](s: Segment[T]): fnum = (s.en - s.st).lenSq
proc lenManh*[T](s: Segment[T]): fnum = (s.en - s.st).lenManh
proc center*[T](s: Segment[T]): T = (s.st + s.en) / 2.0

proc proj*(p: Vec2f, s, d: Vec2f): fnum =
  if d ~= Vec2f.zero: 0.0 else: (p - s) * d / d.lenSq

# project point to segment
proc proj*(p: Vec2f, s: Segment2f, bound: bool = true): Vec2f = 
  result = s.st
  let d = s.en - s.st
  if d.x != 0 or d.y != 0:
    let t = (p - s.st) * d / d.lenSq
    if t in 0.0..1.0 or not bound:
      return s.st + d * t
    elif t > 1.0:
      result = s.en

# square distance from a point to a segment
proc distSq*(p: Vec2f, s: Segment2f): fnum = p.proj(s).distSq(p)
proc dist*(p: Vec2f, s: Segment2f): fnum = sqrt(p.distSq(s))

proc intersect1(a, b, c, d: fnum): bool =
  max(min(a, b), min(c, d)) <= min(max(a, b), max(c, d)) + eps
  
# returns true if segment (a, b) intersects (c, d)
proc intersects*(a, b, c, d: Vec2f): bool =
  intersect1(a.x, b.x, c.x, d.x) and intersect1(a.y, b.y, c.y, d.y) and
    orient2d(a, b, c).ord * orient2d(a, b, d).ord <= 0 and
    orient2d(c, d, a).ord * orient2d(c, d, b).ord <= 0

proc bbox*(segm: Segment2f): BBox2f = bbox(min(segm.st, segm.en), max(segm.st, segm.en))
# https://stackoverflow.com/a/1968345/125351
proc intersects*(a, b: Segment2f): bool =
  let s1 = a.st - a.en
  let s2 = b.st - b.en
  var d = s1 ^ s2
  if abs(d) < eps: # Collinear
    return false

  var v = a.st - b.st
  var s = (s1 ^ v) / d
  if s < 0.0 or s > 1.0:
    return false
  var t = (s2 ^ v) / d
  return t >= 0.0 and t <= 1.0
# also returns the point of intersection
proc intersects*(a, b: Segment2f, p: var Vec2f): bool =
  let s1 = a.en - a.st
  let s2 = b.en - b.st
  var d = s1 ^ s2
  if abs(d) < eps: # Collinear
    return false

  var v = a.st - b.st
  var s = (s1 ^ v) / d
  if s < 0.0 or s > 1.0:
    return false
  var t = (s2 ^ v) / d
  if t < 0.0 or t > 1.0:
    return false

  p = a.st + t * s1
  return true

# Triangle

proc segments*(a, b, c: Vec2f): seq[Segment2f] = @[a->b, b->c, c->a]
iterator segments*(a, b, c: Vec2f): Segment2f =
  yield a->b
  yield b->c
  yield c->a

proc perimeter*(a, b, c: Vec2f): fnum = a.dist(b) + b.dist(c) + c.dist(a)
proc normal*(a, b, c: Vec2f): fnum = (c - b) ^ (c - a)

# check if a point lies within a convex triangle
proc inside*(p: Vec2f, a, b, c: Vec2f): bool =
  (c.x - p.x) * (a.y - p.y) - (a.x - p.x) * (c.y - p.y) >= 0 and
  (a.x - p.x) * (b.y - p.y) - (b.x - p.x) * (a.y - p.y) >= 0 and
  (b.x - p.x) * (c.y - p.y) - (c.x - p.x) * (b.y - p.y) >= 0

proc circumcenter*(p1, p2, p3: Vec2f): Vec2f =
  let ab = p2 - p1
  let ac = p3 - p1
  let abXac = ab ^ ac
  let bl = ab.lenSq
  let cl = ac.lenSq
  return p1 + 0.5 * (ac.y * bl - ab.y * cl, ab.x * cl - ac.x * bl) / abXac

proc circumradius*(p1, p2, p3: Vec2f): fnum =
  let ab = p2 - p1
  let ac = p3 - p1
  let abXac = ab ^ ac
  let bl = ab.lenSq
  let cl = ac.lenSq
  return (0.5 * (ac.y * bl - ab.y * cl, ab.x * cl - ac.x * bl) / abXac).lenSq

# = barycenter
proc centroid*(p1, p2, p3: Vec2f): Vec2f = (p1 + p2 + p3) / 3.0

proc incenter*(p1, p2, p3: Vec2f): Vec2f =
  let a = (p3 - p2).length
  let b = (p3 - p1).length
  let c = (p2 - p1).length
  return (p1 * a + p2 * b + p3 * c) / (a + b + c)

# TODO: proc orthocenter*(p1, p2, p3: Vec2f): Vec2f = 

proc incircle*(pa, pb, pc, pd: Vec2f): bool =
  let d = pa - pd
  let e = pb - pd
  let f = pc - pd
  let ap = d.lenSq
  let bp = e.lenSq
  let cp = f.lenSq
  return d.x * (e.y * cp - bp * f.y) - d.y * (e.x * cp - bp * f.x) + ap * (e.x * f.y - e.y * f.x) < 0

proc distSq*(p, a, b, c: Vec2f): fnum =
  min(min(p.distSq(a->b), p.distSq(b->c)), p.distSq(c->a))
proc dist*(p, a, b, c: Vec2f): fnum = sqrt(p.distSq(a, b, c))

template segments*(t: Triangle2f): seq[Segment2f] = segments(t[0], t[1], t[2])
iterator segments*(t: Triangle2f): Segment2f =
  yield t[0]->t[1]
  yield t[1]->t[2]
  yield t[2]->t[0]
template orient2d*(t: Triangle2f): fnum = orient2d(t[0], t[1], t[2])
template perimeter*(t: Triangle2f): fnum = perimeter(t[0], t[1], t[2])
template normal*(t: Triangle2f): fnum = normal(t[0], t[1], t[2])
template circumcenter*(t: Triangle2f): fnum = circumcenter(t[0], t[1], t[2])
template centroid*(t: Triangle2f): fnum = centroid(t[0], t[1], t[2])
template incenter*(t: Triangle2f): fnum = incenter(t[0], t[1], t[2])
template inside*(p: Vec2f, t: Triangle2f): fnum = inside(p, t[0], t[1], t[2])
template incircle*(t: Triangle2f, p: Vec2f): bool = incircle(t[0], t[1], t[2], p)
template distSq*(p: Vec2f, t: Triangle2f): fnum = distSq(p, t[0], t[1], t[2])
template dist*(p: Vec2f, t: Triangle2f): fnum = dist(p, t[0], t[1], t[2])

proc bbox*[T](t: Triangle[T]): BBox[T] = (min(min(t[0], t[1]), t[2]), max(max(t[0], t[1]), t[2]))

proc bbox*(p: Vec2f): BBox2f = p..p
proc bbox*(p: Vec2f, r: fnum): BBox2f = bbox(p - (r, r), p + (r, r))
proc bbox*(ps: varargs[Vec2f]): BBox2f = bbox(min(ps), max(ps))
proc ibox*(p: Vec2f): BBox2i = bbox(floor(p).ivec, ceil(p).ivec)

proc bboxAt*(p: Vec2f, size: Vec2f): BBox2f = bbox(p - size * 0.5, p + size * 0.5)

# Polyline/polygon

proc segments*(path: Path2f, closed: bool = false): seq[Segment2f] =
  result = newSeq[Segment2f](path.len - (if closed: 0 else: 1))
  for i in 0..<result.len:
    result[i] = path[i] -> path[(i + 1) mod path.len]

iterator segments*(path: Path2f, closed: bool = false): Segment2f =
  for i in 0..<(path.len - (if closed: 0 else: 1)):
    yield path[i] -> path[(i + 1) mod path.len]

# Uses crossing number, so it can be used only for simple (non-self-intersecting) paths
proc inside*(p: Vec2f, path: Path2f): bool =
  var cn = 0
  for i, p0 in path:
    let p1 = path[(i + 1) mod path.len]
    if (p0.y <= p.y and p1.y > p.y) or (p0.y > p.y and p1.y <= p.y):
      let vt = (p.y - p0.y) / (p1.y - p0.y)
      if p.x < p0.x + vt * (p1.x - p0.x):
        cn += 1
  return (cn and 1) != 0

proc windingNumber(p: Vec2f, path: Path2f): int =
  for i, p0 in path:
    let p1 = path[(i + 1) mod path.len]
    if p0.y <= p.y:
      if p1.y > p.y: # an upward crossing
        if orient2d(p0, p1, p) > orCollinear:
          result += 1
    else:
      if p1.y <= p.y: # a downward crossing
        if orient2d(p0, p1, p) < orCollinear:
          result -= 1

# For nonsimple (self-intersecting) paths
proc insideNonZero*(p: Vec2f, path: Path2f): bool =
  windingNumber(p, path) != 0
proc insideEvenOdd*(p: Vec2f, path: Path2f): bool =
  (windingNumber(p, path) mod 1) != 0

# Signed area of a (simple) path
proc area*(path: Path2f): fnum =
  for i in 2..<path.len:
    result += area(path[0], path[i - 1], path[i])
    
proc length*(path: Path2f, closed: bool = false): fnum =
  for i in 0..<(path.len - (if closed: 0 else: 1)):
    result += path[(i + 1) mod path.len].dist(path[i])

proc lenSq*(path: Path2f, closed: bool = false): fnum = 
  let len = path.length(closed)
  return len * len

template perimeter*(path: Path2f): fnum = path.length(true)

proc distSq*(p: Vec2f, path: Path2f, closed: bool = false): fnum =
  result = Inf
  for segm in path.segments(closed):
    result = min(result, p.distSq(segm))

proc dist*(p: Vec2f, path: Path2f, closed: bool = false): fnum =
  sqrt(p.distSq(path, closed))

proc segments*(path: Polygon2f): seq[Segment2f] =
  var j = path.contour.len
  for hole in path.holes:
    j += hole.len
  result = newSeq[Segment2f](j)
  for i in 0..<path.contour.len:
    result[i] = path.contour[i] -> path.contour[(i + 1) mod path.contour.len]
  j = path.contour.len
  for hole in path.holes:
    for i in 0..<hole.len:
      result[j + i] = hole[i] -> hole[(i + 1) mod hole.len]

iterator segments*(path: Polygon2f): (Segment2f, bool) =
  for segm in path.contour.segments(true):
    yield (segm, false)
  for hole in path.holes:
    for segm in hole.segments(true):
      yield (segm, true)

proc bbox*(path: Polygon2f): BBox2f = bbox(path.contour)
proc inside*(p: Vec2f, contour: Path2f, holes: seq[Path2f]): bool =
  if not p.inside(contour):
    return false
  for hole in holes:
    if p.inside(hole):
      return false
  return true
proc inside*(p: Vec2f, path: Polygon2f): bool = p.inside(path.contour, path.holes)
proc area*(path: Polygon2f): fnum =
  result = path.contour.area
  for hole in path.holes:
    result -= hole.area
proc perimeter*(path: Polygon2f): fnum =
  result = path.contour.perimeter
  for hole in path.holes:
    result += hole.perimeter

proc distSq*(p: Vec2f, contour: Path2f, holes: seq[Path2f]): fnum =
  result = Inf
  for segm in contour.segments(true):
    result = min(result, p.distSq(segm))
  for hole in holes:
    for segm in hole.segments(true):
      result = min(result, p.distSq(segm))
proc dist*(p: Vec2f, contour: Path2f, holes: seq[Path2f]): fnum = 
  sqrt(p.distSq(contour, holes))
proc distSq*(p: Vec2f, path: Polygon2f): fnum =
  p.distSq(path.contour, path.holes)
proc dist*(p: Vec2f, path: Polygon2f): fnum =
  sqrt(p.distSq(path.contour, path.holes))

template within*(p: Vec2f, inset, insetSq: fnum, area: varargs[untyped]): bool =
  if p.inside(area):
    inset <= 0.0 or p.distSq(area) >= insetSq
  else:
    inset < 0.0 and p.distSq(area) < insetSq

proc ccw*(path: Path2f): Path2f =
  if path.area < 0.0: path.reversed else: path
proc cw*(path: Path2f): Path2f =
  if path.area > 0.0: path.reversed else: path

proc close*(contour: Path2f): Polygon2f = Polygon2f(contour: contour.ccw)
proc poke*(contour: Path2f, holes: seq[Path2f]): Polygon2f =
  Polygon2f(contour: contour.ccw, holes: holes.mapIt(it.ccw))

# Rect

proc min*(rect: Rect2f): Vec2f = rect.origin + min(rect.size, (0.0, 0.0))
proc max*(rect: Rect2f): Vec2f = rect.origin + max(rect.size, (0.0, 0.0))
proc x*(rect: Rect2f): fnum = rect.origin.x
proc y*(rect: Rect2f): fnum = rect.origin.y
proc width*(rect: Rect2f): fnum = rect.size.x
proc height*(rect: Rect2f): fnum = rect.size.y
proc area*(rect: Rect2f): fnum = rect.size.x * rect.size.y
proc union*(r1, r2: Rect2f): Rect2f = 
  let origin = min(r1.min, r2.min)
  rect(origin, max(r1.max, r2.max) - origin)
proc inside*(p: Vec2f, rect: Rect2f): bool = 
  p.x >= rect.x and p.y >= rect.y and p.x < rect.x + rect.width and p.y < rect.y + rect.height
proc bbox*(rect: Rect2f): BBox[Vec2f] = bbox(rect.min, rect.max)

proc corners*(rect: Rect2f): array[4, Vec2f] =
  [rect.min, (rect.min.x, rect.max.y), rect.max, (rect.max.x, rect.min.y)]

# BBox

template zero*(T: typedesc[BBox2f]): BBox2f = bbox((0.0, 0.0), (0.0, 0.0))
template one*(T: typedesc[BBox2f]): BBox2f = bbox((0.0, 0.0), (1.0, 1.0))
template empty*(T: typedesc[BBox2f]): BBox2f = bbox((Inf, Inf), (-Inf, -Inf))
template zero*(T: typedesc[BBox2i]): BBox2i = bbox((0, 0), (0, 0))
template one*(T: typedesc[BBox2i]): BBox2i = bbox((0, 0), (1, 1))
template empty*(T: typedesc[BBox2i]): BBox2i = bbox((int.high, int.high), (int.low, int.low))

proc `*`*[T](bbox: BBox[T], s: fnum): BBox[T] = bbox(bbox.min * s, bbox.max * s)
proc `*`*[T](s: fnum, bbox: BBox[T]): BBox[T] = bbox(bbox.min * s, bbox.max * s)
proc `/`*[T](bbox: BBox[T], s: fnum): BBox[T] = bbox(bbox.min / s, bbox.max / s)
proc size*[T](bbox: BBox[T]): T = bbox.max - bbox.min
proc x*[T](bbox: BBox2[T]): T = bbox.min.x
proc y*[T](bbox: BBox2[T]): T = bbox.min.y
proc width*[T](bbox: BBox2[T]): T = bbox.max.x - bbox.min.x
proc height*[T](bbox: BBox2[T]): T = bbox.max.y - bbox.min.y
proc aspect*[T](bbox: BBox2[T]): T = bbox.width / bbox.height
proc area*[T](bbox: BBox2[T]): T = bbox.size.x * bbox.size.y
proc center*[T](bbox: BBox2[T]): Vec2[T] = avg(bbox.min, bbox.max)
proc union*[T](b1, b2: BBox[T]): BBox[T] = bbox(min(b1.min, b2.min), max(b1.max, b2.max))
proc intersect*[T](b1, b2: BBox2[T]): BBox2[T] = bbox(max(b1.min, b2.min), min(b1.max, b2.max))
proc `+`*[T](b1, b2: BBox2[T]): BBox2[T] = union(b1, b2)
proc `*`*[T](b1, b2: BBox2[T]): BBox2[T] = intersect(b1, b2)
proc inside*[T](p: Vec2[T], bbox: BBox2[T]): bool = 
  p.x >= bbox.min.x and p.y >= bbox.min.y and
  p.x <= bbox.max.x and p.y <= bbox.max.y
proc `in`*[T](p: Vec2[T], bbox: BBox2[T]): bool = inside(p, bbox)
proc inside*[T](p: Vec3[T], bbox: BBox3[T]): bool = 
  p.x >= bbox.min.x and p.y >= bbox.min.y and p.z >= bbox.min.z and 
  p.x < bbox.max.x and p.y < bbox.max.y and p.z < bbox.max.z
proc `in`*[T](p: Vec3[T], bbox: BBox3[T]): bool = inside(p, bbox)
proc intersects*[T](b1, b2: BBox2[T]): bool =
  b1.max.x >= b2.min.x and b1.max.y >= b2.min.y and
  b1.min.x <= b2.max.x and b1.min.y <= b2.max.y
proc contains*[T](b1, b2: BBox2[T]): bool =
  b1.min.x <= b2.min.x and b1.max.x >= b2.max.x and
  b1.min.y <= b2.min.y and b1.max.y >= b2.max.y
proc inside*[T](b1, b2: BBox2[T]): bool = b2.contains(b1)
proc distSq*[T](p: Vec2[T], bbox: BBox2[T]): fnum =
  let dx = if p.x < bbox.min.x: bbox.min.x - p.x elif p.x > bbox.max.x: p.x - bbox.max.x else: 0.0
  let dy = if p.y < bbox.min.y: bbox.min.y - p.y elif p.y > bbox.max.y: p.y - bbox.max.y else: 0.0
  return dx * dx + dy * dy
proc dist*[T](p: T, bbox: BBox[T]): fnum = sqrt(p.dist(bbox))
template ibox*(bbox: BBox2i): BBox2i = bbox
template bbox*(bbox: BBox2f): BBox2f = bbox
proc bbox*(bbox: BBox2i): BBox2f = bbox(bbox.min.fvec, bbox.max.fvec)
proc ibox*(bbox: BBox2f): BBox2i = bbox(floor(bbox.min).ivec, ceil(bbox.max.ceil).ivec)

proc inset*[T](bbox: BBox2[T], dist: T): BBox2[T] = bbox(bbox.min + (dist, dist), bbox.max - (dist, dist))

proc fit*(bbox: BBox2f, ratio: fnum = 1.0): BBox2f = 
  var size: Vec2f = (bbox.height * ratio, bbox.height)
  if bbox.width > size.x:
    size = (bbox.width, bbox.width / ratio)
  bboxAt(bbox.center, size)
proc cover*(bbox: BBox2f, ratio: fnum = 1.0): BBox2f = 
  var size: Vec2f = (bbox.height * ratio, bbox.height)
  if bbox.width < size.x:
    size = (bbox.width, bbox.width / ratio)
  bboxAt(bbox.center, size)

proc map*(p: Vec2f, bbox1: BBox2f, bbox2: BBox2f = BBox2f.one): Vec2f = (
  bbox2.min.x + (p.x - bbox1.min.x) * (bbox2.width / bbox1.width),
  bbox2.min.y + (p.y - bbox1.min.y) * (bbox2.height / bbox1.height),
)

proc corners*[T](bbox: BBox2[T]): array[4, Vec2[T]] =
  [bbox.min, (bbox.min.x, bbox.max.y), bbox.max, (bbox.max.x, bbox.min.y)]

#proc intersects*(s: Segment2, bbox: BBox[Vec2f]): bool =
  # s.st.inside(bbox) or s.en.inside(bbox) wrong
  # s.bbox.intersects(bbox)


# 3D Vectors

template zero*(T: typedesc[Vec3f]): Vec3f = (0.0, 0.0, 0.0)
template one*(T: typedesc[Vec3f]): Vec3f = (1.0, 1.0, 1.0)
template up*(T: typedesc[Vec3f]): Vec3f = (0.0, 1.0, 0.0)
template down*(T: typedesc[Vec3f]): Vec3f = (0.0, -1.0, 0.0)
template left*(T: typedesc[Vec3f]): Vec3f = (-1.0, 0.0, 0.0)
template right*(T: typedesc[Vec3f]): Vec3f = (1.0, 0.0, 0.0)
template forward*(T: typedesc[Vec3f]): Vec3f = (0.0, 0.0, 1.0)
template backward*(T: typedesc[Vec3f]): Vec3f = (0.0, 0.0, -1.0)
template inf*(T: typedesc[Vec3f]): Vec3f = (Inf, Inf, Inf)

proc length*(p: Vec3f): fnum = sqrt(p.x * p.x + p.y * p.y + p.z * p.z)
proc lenSq*(p: Vec3f): fnum = p.x * p.x + p.y * p.y + p.z * p.z
proc lenManh*(p: Vec3f): fnum = abs(p.x) + abs(p.y) + abs(p.z)
proc dist*(p1: Vec3f, p2: Vec3f): fnum = sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z))
proc distSq*(p1: Vec3f, p2: Vec3f): fnum = (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z)
proc distManh*(p1: Vec3f, p2: Vec3f): fnum = abs(p1.x - p2.x) + abs(p1.y - p2.y) + abs(p1.z - p2.z)
proc `+`*(p1: Vec3, p2: Vec3): Vec3 = (p1.x + p2.x, p1.y + p2.y, p1.z + p2.z)
proc `+=`*(p1: var Vec3f, p2: Vec3f) =
  p1.x += p2.x
  p1.y += p2.y
  p1.z += p2.z
proc `-`*(p1: Vec3f, p2: Vec3f): Vec3f = (p1.x - p2.x, p1.y - p2.y, p1.z - p2.z)
proc `-=`*(p1: var Vec3f, p2: Vec3f) =
  p1.x -= p2.x
  p1.y -= p2.y
  p1.z -= p2.z
proc `*`*(p: Vec3f, s: fnum): Vec3f = (p.x * s, p.y * s, p.z * s)
proc `*`*(s: fnum, p: Vec3f): Vec3f = (p.x * s, p.y * s, p.z * s)
proc `/`*(p: Vec3f, s: fnum): Vec3f = (p.x / s, p.y / s, p.z / s)
proc `/=`*(p: var Vec3f, s: fnum) =
  p.x /= s
  p.y /= s
  p.z /= s
proc `*`*(p1: Vec3f, p2: Vec3f): fnum = p1.x * p2.x + p1.y * p2.y + p1.z * p2.z
proc `^`*(p1: Vec3f, p2: Vec3f): Vec3f = (p1.y * p2.z - p1.z * p2.y, p1.z * p2.x - p1.x * p2.z, p1.x * p2.y - p1.y * p2.x)
proc norm*(p: Vec3f): Vec3f = p / p.length
proc proj*(p1: Vec3f, p2: Vec3f): Vec3f = ((p1 * p2) / p2.lenSq) * p2

proc `*`*(mat: Mat3x3, p: Vec3f): Vec3f = (
  mat.a00 * p.x + mat.a01 * p.y + mat.a02 * p.z,
  mat.a10 * p.x + mat.a11 * p.y + mat.a12 * p.z,
  mat.a20 * p.x + mat.a21 * p.y + mat.a22 * p.z
)

proc round*(p: Vec3f): Vec3f = (round(p.x), round(p.y), round(p.z))
proc floor*(p: Vec3f): Vec3f = (floor(p.x), floor(p.y), floor(p.z))
proc ceil*(p: Vec3f): Vec3f = (ceil(p.x), ceil(p.y), ceil(p.z))
proc ivec*(p: Vec3f): Vec3i = (int(p.x), int(p.y), int(p.z))
proc fvec*(p: Vec3i): Vec3f = (fnum(p.x), fnum(p.y), fnum(p.z))

proc bbox*(p: Vec3f): BBox[Vec3f] = p..p

proc min*(ps: varargs[Vec3f]): Vec3f =
  var res: Vec3f = (Inf, Inf, Inf)
  for p in ps:
    res = (min(res.x, p.x), min(res.y, p.y), min(res.z, p.z))
  return res
proc max*(ps: varargs[Vec3f]): Vec3f =
  var res: Vec3f = (-Inf, -Inf, -Inf)
  for p in ps:
    res = (max(res.x, p.x), max(res.y, p.y), max(res.z, p.z))
  return res

proc rotAlign*(p1: Vec3f, p2: Vec3f): Mat3x3 =
  var axis = p1 ^ p2
  var cosA = p1 * p2
  var k = 1.0 / (1.0 + cosA)
  return (
    axis.x * axis.x * k + cosA, axis.y * axis.x * k - axis.z, axis.z * axis.x * k + axis.y,
    axis.x * axis.y * k + axis.z, axis.y * axis.y * k + cosA, axis.z * axis.y * k - axis.x,
    axis.x * axis.z * k - axis.y, axis.y * axis.z * k + axis.x, axis.z * axis.z * k + cosA,
  )

proc mix*(p1: Vec3f, p2: Vec3f, t: fnum): Vec3f = (1.0 - t) * p1 + t * p2

proc centroid*(p1, p2, p3: Vec3f): Vec3f = (p1 + p2 + p3).norm

proc circumcenter*(p1, p2, p3: Vec3f): Vec3f =
  let ac = p3 - p1
  let ab = p2 - p1
  let abXac = ab ^ ac
  return p1 + 0.5 * ((abXac ^ ab) * ac.lenSq + (ac ^ abXac) * ab.lenSq) / abXac.lenSq


# Allows easy conversion to 2D
proc xz*(p: Vec3f): Vec2f = (p.x, p.z)
proc xy*(p: Vec3f): Vec2f = (p.x, p.y)

proc stereographic*(p: Vec3f): Vec2f = (p.x / (1 - p.y), p.z / (1 - p.y))
proc stereographicInv*(p: Vec2f): Vec3f =
  var d = 1 + p.lenSq
  return (2.0 * p.x / d, (d - 2.0) / d, 2.0 * p.y / d)

proc winkelTripel*(p: Vec3f): Vec3f =
  let p = p.norm
  let lng = arctan2(p.z, p.x)
  let lat = PI / 2.0 - arccos(p.y)
  let cosPhi = cos(lat)
  # let sinPhi = sqrt(1.0 - cosPhi * cosPhi)
  let alpha = arccos(cosPhi * cos(lng / 2.0))
  let sincAlpha = sin(alpha) / alpha
  let x = 2.0 * lng / PI + 2.0 * cosPhi * sin(lng / 2.0) / sincAlpha
  let y = lat + sin(lat) / sincAlpha
  return (x, 0.001 * abs(x), y)

proc triangleArea*(vs: varargs[Vec3f]): fnum = 0.5 * ((vs[1] - vs[0]) ^ (vs[2] - vs[0])).length
proc triangleNormal*(vs: varargs[Vec3f]): Vec3f = ((vs[2] - vs[1]) ^ (vs[2] - vs[0])).norm


{.pop.}

proc cmpX*[T](p1, p2: Vec2[T]): int =
  if p1.x != p2.x:
    return cmp(p1.x, p2.x)
  else:
    return cmp(p1.y, p2.y)

proc cmpY*[T](p1, p2: Vec2[T]): int =
  if p1.y != p2.y:
    return cmp(p1.y, p2.y)
  else:
    return cmp(p1.x, p2.x)