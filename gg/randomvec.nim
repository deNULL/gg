import gg/vectors
import random

proc rand*(r: var Rand, max: Vec2f): Vec2f = (r.rand(max.x), r.rand(max.y))
proc rand*(r: var Rand, max: Vec2i): Vec2i = (r.rand(max.x), r.rand(max.y))
proc rand*(r: var Rand, max: Vec3f): Vec3f = (r.rand(max.x), r.rand(max.y), r.rand(max.z))
proc rand*(r: var Rand, max: Vec3i): Vec3i = (r.rand(max.x), r.rand(max.y), r.rand(max.z))
# proc rand*(r: var Rand, bbox: BBox2f): Vec2f = (r.rand(bbox.min.x..bbox.max.x), r.rand(bbox.min.y..bbox.max.y))
proc rand*(r: var Rand, bbox: BBox2i): Vec2i = (r.rand(bbox.min.x..bbox.max.x), r.rand(bbox.min.y..bbox.max.y))
proc rand*(r: var Rand, bbox: BBox3f): Vec3f = (r.rand(bbox.min.x..bbox.max.x), r.rand(bbox.min.y..bbox.max.y), r.rand(bbox.min.z..bbox.max.z))
proc rand*(r: var Rand, bbox: BBox3i): Vec3i = (r.rand(bbox.min.x..bbox.max.x), r.rand(bbox.min.y..bbox.max.y), r.rand(bbox.min.z..bbox.max.z))

proc rand*(r: var Rand, region: Region2f, inset: fnum = 0.0, maxit: int = 1000000): Vec2f =
  ## Random point inside any region with bounding box and a inside function
  runnableExamples:
    import gg/vectors
    import random
    var rnd = initRand(42)
    let a = rnd.rand((300.0, 100.0)..(333.0, 200.0))
  
  when region is BBox2f:
    (
      r.rand((region.min.x + inset)..(region.max.x - inset)),
      r.rand((region.min.y + inset)..(region.max.y - inset)),
    )
  else:
    let bbox = region.bbox.inset(inset)
    let insetSq = inset * inset
    for it in 1..maxit:
      result = (r.rand(bbox.min.x..bbox.max.x), r.rand(bbox.min.y..bbox.max.y))
      if result.within(inset, insetSq, region):
        return
    return Vec2f.inf

proc rand*(r: var Rand, contour: Path2f, holes: seq[Path2f] = @[], inset: fnum = 0.0, maxit: int = 1000000): Vec2f =
  ## Random point inside a polygon
  let bbox = contour.bbox.inset(inset)
  let insetSq = inset * inset
  for it in 1..maxit:
    result = r.rand(bbox)
    if result.within(inset, insetSq, contour, holes):
      return
  return Vec2f.inf

proc rand*(r: var Rand, a, b, c: Vec2f, inset: fnum = 0.0, maxit: int = 1000000): Vec2f =
  ## Random point inside a triangle (defined by three points)
  var b = b
  var c = c
  if orient2d(a, b, c) == orCW:
    swap(b, c)
  let bbox = bbox(a, b, c).inset(inset)
  let insetSq = inset * inset
  for it in 1..maxit:
    result = r.rand(bbox)
    if result.within(inset, insetSq, a, b, c):
      return
  return Vec2f.inf