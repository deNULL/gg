import gg/vectors
import algorithm
import ropes
import math

type
  Grid[T, U, V] = object
    size: U
    bounds: BBox[V]
    cells: seq[T]

  Grid2*[T] = Grid[T, Vec2i, Vec2f]
  Grid3*[T] = Grid[T, Vec3i, Vec3f]

proc size*[T, U, V](g: Grid[T, U, V]): U = g.size
proc width*(g: Grid): int = g.size.x
proc height*(g: Grid): int = g.size.y
proc depth*(g: Grid3): int = g.size.z
proc bounds*[T, U, V](g: Grid[T, U, V]): BBox[V] = g.bounds

proc fill*[T, U, V](g: var Grid[T, U, V], value: T) = g.cells.fill(value)

proc contains*[T](g: Grid2[T], idx: Vec2i): bool =
  idx.x >= 0 and idx.x < g.size.x and idx.y >= 0 and idx.y < g.size.y

proc `in`*[T](idx: Vec2i, g: Grid2[T]): bool =
  idx.x >= 0 and idx.x < g.size.x and idx.y >= 0 and idx.y < g.size.y

proc contains*[T](g: Grid3[T], idx: Vec3i): bool =
  idx.x >= 0 and idx.x < g.size.x and idx.y >= 0 and idx.y < g.size.y and idx.z >= 0 and idx.z < g.size.z

proc contains*[T, U, V](g: Grid[T, U, V], p: V): bool = p.inside(g.bounds)

proc `in`*[T, U, V](p: V, g: Grid[T, U, V]): bool = p.inside(g.bounds)

proc initGrid2*[T](size: Vec2i, bounds: BBox2f): Grid2[T] = Grid2[T](size: size, bounds: bounds, cells: newSeq[T](size.x * size.y))
proc initGrid2*[T](size: Vec2i, cell: fnum = 1.0): Grid2[T] = initGrid2[T](size, Vec2f.zero..(size.fvec * cell))
proc initGrid2*[T](width, height: int, cell: fnum = 1.0): Grid2[T] = initGrid2[T]((width, height), cell)
proc initGrid2*[T](size: Vec2f, cell: fnum = 1.0): Grid2[T] = initGrid2[T](ceil(size / cell).ivec, Vec2f.zero..size)
proc initGrid2*[T](width, height: fnum, cell: fnum = 1.0): Grid2[T] = initGrid2[T]((width, height), cell)
proc initGrid2*[T](bounds: BBox2f, cell: fnum = 1.0): Grid2[T] = initGrid2[T](ceil(bounds.size / cell).ivec, bounds)

proc initGrid3*[T](size: Vec3i, bounds: BBox3f): Grid3[T] = Grid3[T](size: size, bounds: bounds, cells: newSeq[T](size.x * size.y * size.z))
proc initGrid3*[T](size: Vec3i, cell: fnum = 1.0): Grid3[T] = initGrid3[T](size, Vec3f.zero..(size.fvec * cell))
proc initGrid3*[T](width, height, depth: int, cell: fnum = 1.0): Grid3[T] = initGrid3[T]((width, height, depth), cell)
proc initGrid3*[T](size: Vec3f, cell: fnum = 1.0): Grid3[T] = initGrid3[T](ceil(size / cell).ivec, Vec3f.zero..size)
proc initGrid3*[T](width, height, depth: fnum, cell: fnum = 1.0): Grid3[T] = initGrid3[T]((width, height, depth), cell)
proc initGrid3*[T](bounds: BBox3f, cell: fnum = 1.0): Grid3[T] = initGrid3[T](ceil(bounds.size / cell).ivec, bounds)

template ix(g: Grid, px: fnum): int = floor((px - g.bounds.min.x) * g.size.x.float / (g.bounds.max.x - g.bounds.min.x)).int
template iy(g: Grid, py: fnum): int = floor((py - g.bounds.min.y) * g.size.y.float / (g.bounds.max.y - g.bounds.min.y)).int
template iz(g: Grid, pz: fnum): int = floor((pz - g.bounds.min.z) * g.size.z.float / (g.bounds.max.z - g.bounds.min.z)).int

proc cellOf*[T](g: Grid2[T], p: Vec2f): Vec2i = (ix(g, p.x), iy(g, p.y))
proc cellOf*[T](g: Grid3[T], p: Vec3f): Vec3i = (ix(g, p.x), iy(g, p.y), iz(g, p.z))

proc cell*[T](g: Grid2[T], idx: Vec2i): BBox2f =
  let c: Vec2f = (g.bounds.width / g.width.float, g.bounds.height / g.height.float)
  let p: Vec2f = g.bounds.min + (idx.x.float * c.x, idx.y.float * c.y)
  p..(p + c)

# Grid2
proc `[]`*[T](g: Grid2[T], idx: Vec2i): T =
  assert g.contains(idx), "Index outside bounds"
  g.cells[idx.y * g.size.x + idx.x]

proc `[]`*[T](g: Grid2[T], i, j: int): T =
  assert (i, j) in g, "Index outside bounds"
  g.cells[j * g.size.x + i]

proc `[]`*[T](g: Grid2[T], p: Vec2f): T =
  assert p in g, "Index outside bounds"
  g.cells[iy(g, p.y) * g.size.x + ix(g, p.x)]

proc `[]`*[T](g: Grid2[T], x, y: fnum): T =
  assert (x, y) in g, "Index outside bounds"
  g.cells[iy(g, y) * g.size.x + ix(g, x)]

proc `[]=`*[T](g: var Grid2[T], idx: Vec2i, value: T) =
  assert idx in g, "Index outside bounds"
  g.cells[idx.y * g.size.x + idx.x] = value

proc `[]=`*[T](g: var Grid2[T], i, j: int, value: T) =
  assert (i, j) in g, "Index outside bounds"
  g.cells[j * g.size.x + i] = value

proc `[]=`*[T](g: var Grid2[T], p: Vec2f, value: T) =
  assert p in g, "Index outside bounds"
  g.cells[iy(g, p.y) * g.size.x + ix(g, p.x)] = value

proc `[]=`*[T](g: var Grid2[T], x, y: fnum, value: T) =
  assert (x, y) in g, "Index outside bounds"
  g.cells[iy(g, y) * g.size.x + ix(g, x)] = value

# Grid3
proc `[]`*[T](g: Grid3[T], idx: Vec3i): T =
  assert idx in g, "Index outside bounds"
  g.cells[(idx.z * g.size.y + idx.y) * g.size.x + idx.x]

proc `[]`*[T](g: Grid3[T], i, j, k: int): T =
  assert (i, j, k) in g, "Index outside bounds"
  g.cells[(k * g.size.y + j * g.size.x) + i]

proc `[]`*[T](g: Grid3[T], p: Vec3f): T =
  assert p in g, "Index outside bounds"
  g.cells[(iz(g, p.z) * g.size.y + iy(g, p.y)) * g.size.x + ix(g, p.x)]

proc `[]`*[T](g: Grid3[T], x, y, z: fnum): T =
  assert (x, y) in g, "Index outside bounds"
  g.cells[(iz(g, z) * g.size.y + iy(g, y)) * g.size.x + ix(g, x)]

proc `[]=`*[T](g: var Grid3[T], idx: Vec3i, value: T) =
  assert idx in g, "Index outside bounds"
  g.cells[(idx.z * g.size.y + idx.y) * g.size.x + idx.x] = value

proc `[]=`*[T](g: var Grid3[T], i, j, k: int, value: T) =
  assert (i, j, k) in g, "Index outside bounds"
  g.cells[(k * g.size.y + j * g.size.x) + i] = value

proc `[]=`*[T](g: var Grid3[T], p: Vec3f, value: T) =
  assert p in g, "Index outside bounds"
  g.cells[(iz(g, p.z) * g.size.y + iy(g, p.y)) * g.size.x + ix(g, p.x)] = value

proc `[]=`*[T](g: var Grid3[T], x, y, z: fnum, value: T) =
  assert (x, y, z) in g, "Index outside bounds"
  g.cells[(iz(g, z) * g.size.y + iy(g, y)) * g.size.x + ix(g, x)] = value

iterator pairs*[T](g: Grid2[T]): (Vec2i, T) =
  var idx: Vec2i = (0, 0)
  for i, value in g.cells:
    yield (idx, value)
    idx = if idx.x < g.size.x: (idx.x + 1, idx.y) else: (0, idx.y + 1)

iterator pairs*[T](g: Grid3[T]): (Vec3i, T) =
  var idx: Vec3i = (0, 0, 0)
  for i, value in g.cells:
    yield (idx, value)
    idx = if idx.x < g.size.x: (idx.x + 1, idx.y, idx.z) elif idx.y < g.size.y: (0, idx.y + 1, idx.z) else: (0, 0, idx.z + 1)

iterator items*[T, U, V](g: Grid[T, U, V]): T =
  for value in g.cells:
    yield value

iterator row*[T](g: Grid2[T], row: int): (int, T) =
  for col in 0..<g.size.x:
    yield (col, g[col, row])

iterator col*[T](g: Grid2[T], col: int): (int, T) =
  for row in 0..<g.size.y:
    yield (row, g[col, row])

# Helpers for iterating through neighbors

template yield2(g: Grid2, idx: Vec2i, i, j: int) =
  let v = idx + (i, j)
  if v in g:
    yield (v, g[v])

template yield3(g: Grid3, idx: Vec3i, i, j, k: int) =
  let v = idx + (i, j, k)
  if v in g:
    yield (v, g[v])

# Grid2

iterator neighbors9*[T](g: Grid2[T], idx: Vec2i): (Vec2i, T) =
  for j in -1..1:
    for i in -1..1:
      let k = idx + (i, j)
      if g.contains(k):
        yield (k, g[k])

iterator neighbors8*[T](g: Grid2[T], idx: Vec2i): (Vec2i, T) =
  for j in -1..1:
    for i in -1..1:
      if i == 0 and j == 0: continue
      let k = idx + (i, j)
      if k in g:
        yield (k, g[k])

iterator neighbors5*[T](g: Grid2[T], idx: Vec2i): (Vec2i, T) =
  yield2(g, idx,  0, -1)
  yield2(g, idx, -1,  0)
  yield2(g, idx,  0,  0)
  yield2(g, idx,  1,  0)
  yield2(g, idx,  0,  1)

iterator neighbors4*[T](g: Grid2[T], idx: Vec2i): (Vec2i, T) =
  yield2(g, idx,  0, -1)
  yield2(g, idx, -1,  0)
  yield2(g, idx,  1,  0)
  yield2(g, idx,  0,  1)

iterator neighbors9*[T](g: Grid2[T], p: Vec2f): (Vec2i, T) =
  for pair in g.neighbors9(g.cellOf(p)):
    yield pair

iterator neighbors8*[T](g: Grid2[T], p: Vec2f): (Vec2i, T) =
  for pair in g.neighbors8(g.cellOf(p)):
    yield pair

iterator neighbors5*[T](g: Grid2[T], p: Vec2f): (Vec2i, T) =
  for pair in g.neighbors5(g.cellOf(p)):
    yield pair

iterator neighbors4*[T](g: Grid2[T], p: Vec2f): (Vec2i, T) =
  for pair in g.neighbors4(g.cellOf(p)):
    yield pair

# Grid3

iterator neighbors27*[T](g: Grid3[T], idx: Vec3i): (Vec3i, T) =
  for k in -1..1:
    for j in -1..1:
      for i in -1..1:
        let v = idx + (i, j, k)
        if v in g:
          yield (v, g[v])

iterator neighbors26*[T](g: Grid3[T], idx: Vec3i): (Vec3i, T) =
  for k in -1..1:
    for j in -1..1:
      for i in -1..1:
        if i == 0 and j == 0 and k == 0: continue
        let v = idx + (i, j, k)
        if v in g:
          yield (v, g[v])

iterator neighbors19*[T](g: Grid3[T], idx: Vec3i): (Vec3i, T) =
  for k in -1..1:
    for j in -1..1:
      for i in -1..1:
        if i != 0 and j != 0 and k != 0: continue
        let v = idx + (i, j, k)
        if v in g:
          yield (v, g[v])

iterator neighbors18*[T](g: Grid3[T], idx: Vec3i): (Vec3i, T) =
  for k in -1..1:
    for j in -1..1:
      for i in -1..1:
        if (i != 0 and j != 0 and k != 0) or (i == 0 and j == 0 and k == 0): continue
        let v = idx + (i, j, k)
        if v in g:
          yield (v, g[v])

iterator neighbors7*[T](g: Grid3[T], idx: Vec3i): (Vec3i, T) =
  yield3(g, idx,  0,  0, -1)
  yield3(g, idx,  0, -1,  0)
  yield3(g, idx, -1,  0,  0)
  yield3(g, idx,  0,  0,  0)
  yield3(g, idx,  1,  0,  0)
  yield3(g, idx,  0,  1,  0)
  yield3(g, idx,  0,  0,  1)

iterator neighbors6*[T](g: Grid3[T], idx: Vec3i): (Vec3i, T) =
  yield3(g, idx,  0,  0, -1)
  yield3(g, idx,  0, -1,  0)
  yield3(g, idx, -1,  0,  0)
  yield3(g, idx,  1,  0,  0)
  yield3(g, idx,  0,  1,  0)
  yield3(g, idx,  0,  0,  1)

proc `$`*[T](g: Grid2[T]): string =
  var r = rope("Grid2(size: " & $g.size.x & "x" & $g.size.y & ", bounds: " & $g.bounds & ", cells: [")
  for j in 0..<g.size.y:
    r.add("\n  [")
    for i in 0..<g.size.x:
      if i > 0: r.add(", ")
      r.add($g[i, j])
    r.add("],")
  r.add("\n])")
  return $r

proc `$`*[T](g: Grid3[T]): string =
  var r = rope("Grid3(size: " & $g.size.x & "x" & $g.size.y & "x" & $g.size.z & ", bounds: " & $g.bounds & ", cells: [")
  for w in 0..<g.size.z:
    if w == 0: r.add("\n  ") else: r.add(", ")
    r.add("[ # z: " & $w)
    for v in 0..<g.size.y:
      r.add("\n    [")
      for u in 0..<g.size.x:
        if u > 0: r.add(", ")
        r.add($g[u, v, w])
      r.add("],")
    r.add("\n  ]")
  r.add("\n])")
  return $r
