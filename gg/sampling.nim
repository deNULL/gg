import gg/[vectors, randomvec, grid]
import random
import math
import deques

proc sampleRandom*(
  region: Region2f, 
  count: int,
  dist: fnum,
  rand: var Rand,
  inset: fnum = 0.0,
  maxit: int = 1000000,
): seq[Vec2f] =
  let bbox = bbox(region).inset(inset)
  let distSq = dist * dist
  let cell = dist / sqrt(2.0)
  var grid: Grid2[int]
  var useGrid = bbox.width * bbox.height / (cell * cell) < 1e7
  if useGrid:
    grid = initGrid2[int](bbox, cell)
    grid.fill(-1)
  result = newSeq[Vec2f](count)
  let maxitSqrt = max(sqrt(maxit.float).ceil.int, 10)
  for i in 0..<count:
    for it in 1..maxitSqrt:
      result[i] = rand.rand(region, inset, maxitSqrt)
      if result[i] != Vec2f.inf:
        if useGrid:
          let i0 = grid.cellOf(result[i])
          for (i1, n) in grid.neighbors9(i0):
            if n != -1:
              if i1 == i0:
                result[i] = Vec2f.inf
                break
              let neighbor = result[n]
              if neighbor.distSq(result[i]) < distSq:
                result[i] = Vec2f.inf
                break
          if result[i] != Vec2f.inf:
            grid[i0] = i
        elif dist > 0.0: # just iterate through all
          for j in 0..<i:
            if result[j].distSq(result[i]) < distSq:
              result[i] = Vec2f.inf
              break
      
      if result[i] != Vec2f.inf:
        break
    if result[i] == Vec2f.inf:
      result.setLen(i)
      return

proc sampleRandom*(
  region: Region2f, 
  count: int,
  dist: fnum,
  rand: int64 = int.high,
  inset: fnum = 0.0,
  maxit: int = 1000000,
): seq[Vec2f] =
  var rand = if rand == int.high: initRand(rand(int.high)) else: initRand(rand)
  sampleRandom(region, count, dist, rand, inset)

proc sampleRectGrid*(
  region: Region2f, 
  dist: fnum,
  rand: var Rand, 
  jitter: fnum,
  seed: Vec2f = Vec2f.zero,
  angle: fnum = 0.0,
  inset: fnum = 0.0,
): seq[Vec2f] =
  let insetSq = inset * inset
  let bbox = bbox(region).inset(inset)
  let corners = bbox.corners
  let axis0 = cartesian(1.0, angle)
  let axis1 = axis0.perp
  var (a0min, a0max) = (Inf, -Inf)
  var (a1min, a1max) = (Inf, -Inf)
  for corner in corners:
    let a0 = corner.proj(seed, axis0)
    let a1 = corner.proj(seed, axis1)
    a0min = min(a0min, a0)
    a0max = max(a0max, a0)
    a1min = min(a1min, a1)
    a1max = max(a1max, a1)
    
  let i0 = floor(a0min / dist).int
  let i1 = ceil(a0max / dist).int
  let j0 = floor(a1min / dist).int
  let j1 = ceil(a1max / dist).int
  for i in i0..i1:
    for j in j0..j1:
      var p = seed + axis0 * i.fnum * dist + axis1 * j.fnum * dist
      if jitter > 0.0:
        p += rand.rand((-jitter, -jitter)..(jitter, jitter))
      if p.within(inset, insetSq, region):
        result.add(p)

proc sampleRectGrid*(
  region: Region2f, 
  dist: fnum,
  rand: int64,
  jitter: fnum,
  seed: Vec2f = Vec2f.zero,
  angle: fnum = 0.0,
  inset: fnum = 0.0,
): seq[Vec2f] =
  var rand = initRand(rand)
  sampleRectGrid(region, dist, rand, jitter, seed, angle, inset)

proc sampleRectGrid*(
  region: Region2f, 
  dist: fnum,
  jitter: fnum = 0.0,
  seed: Vec2f = Vec2f.zero,
  angle: fnum = 0.0,
  inset: fnum = 0.0,
): seq[Vec2f] =
  var rand = if jitter > 0.0: initRand(rand(int.high)) else: Rand()
  sampleRectGrid(region, dist, rand, jitter, seed, angle, inset)

proc sampleTriGrid*(
  region: Region2f, 
  dist: fnum,
  rand: var Rand, 
  jitter: fnum,
  seed: Vec2f = Vec2f.zero,
  angle: fnum = 0.0,
  inset: fnum = 0.0,
): seq[Vec2f] =
  let insetSq = inset * inset
  let bbox = bbox(region)
  let corners = bbox.corners
  let axis0 = cartesian(1.0, angle)
  let axis1 = axis0.perp * sqrt(3.0) / 2.0
  var (a0min, a0max) = (Inf, -Inf)
  var (a1min, a1max) = (Inf, -Inf)
  for corner in corners:
    let a0 = corner.proj(seed, axis0)
    let a1 = corner.proj(seed, axis1)
    a0min = min(a0min, a0)
    a0max = max(a0max, a0)
    a1min = min(a1min, a1)
    a1max = max(a1max, a1)
    
  let i0 = floor(a0min / dist).int - 1
  let i1 = ceil(a0max / dist).int
  let j0 = floor(a1min / dist).int
  let j1 = ceil(a1max / dist).int
  for i in i0..i1:
    for j in j0..j1:
      let ioffs = if j mod 2 == 0: i.fnum else: (i.fnum + 0.5)
      var p = seed + axis0 * ioffs * dist + axis1 * j.fnum * dist
      if jitter > 0.0:
        p += (rand.rand(-jitter..jitter), rand.rand(-jitter..jitter))
      if p.within(inset, insetSq, region):
        result.add(p)

# Uses rand seed to initialise random generator
proc sampleTriGrid*(
  region: Region2f, 
  dist: fnum,
  rand: int64,
  jitter: fnum,
  seed: Vec2f = Vec2f.zero,
  angle: fnum = 0.0,
  inset: fnum = 0.0,
): seq[Vec2f] =
  var rand = initRand(rand)
  sampleTriGrid(region, dist, rand, jitter, seed, angle, inset)

proc sampleTriGrid*(
  region: Region2f, 
  dist: fnum,
  jitter: fnum = 0.0,
  seed: Vec2f = Vec2f.zero,
  angle: fnum = 0.0,
  inset: fnum = 0.0,
): seq[Vec2f] =
  var rand = if jitter > 0.0: initRand(rand(int.high)) else: Rand()
  sampleTriGrid(region, dist, rand, jitter, seed, angle, inset)

# Poisson disk sampling
# (Bridson's algorithm)
proc samplePoisson*(
  region: Region2f, 
  dist: fnum,
  rand: var Rand,
  seed: Vec2f = Vec2f.inf,
  inset: fnum = 0.0,
  maxit: int = 30, # Number of tries before giving up
): seq[Vec2f] =
  let insetSq = inset * inset
  let bbox = bbox(region).inset(inset)
  let cell = dist / sqrt(2.0)
  var grid = initGrid2[int](bbox, cell)
  grid.fill(-1)
  let rInner = dist * dist
  let rOuter = 4.0 * rInner
  var p1 = if seed == Vec2f.inf: rand.rand(region) else: seed
  grid[p1] = result.len
  result.add(p1)
  var queue = initDeque[Vec2f](maxit)
  queue.addLast(p1)
  while queue.len > 0:
    let p0 = queue.popFirst()

    for it in 1..maxit: # Look for new candidates within an annulus of p0
      let theta = rand.rand(2.0 * PI)
      let r = sqrt(rand.rand(rInner..rOuter))
      let p1 = p0 + cartesian(r, theta)
      if not p1.within(inset, insetSq, region):
        continue

      var valid = true
      let i0 = grid.cellOf(p1)
      for (i1, n) in grid.neighbors9(i0):
        if n != -1:
          if i1 == i0:
            valid = false
            break
          let neighbor = result[n]
          if neighbor.distSq(p1) < rInner:
            valid = false
            break

      if valid: # No neighbors too close, we can add this point
        grid[i0] = result.len
        result.add(p1)
        queue.addLast(p1)

proc samplePoisson*(
  region: Region2f, 
  dist: fnum,
  rand: int = int.high,
  seed: Vec2f = Vec2f.inf,
  inset: fnum = 0.0,
  maxit: int = 30, # Number of tries before giving up
): seq[Vec2f] =
  var rand = if rand == int.high: initRand(rand(int.high)) else: initRand(rand)
  samplePoisson(region, dist, rand, seed, inset, maxit)

# More uniform (non-random) version of Poisson disk sampling
# see https://observablehq.com/@techsparx/an-improvement-on-bridsons-algorithm-for-poisson-disc-samp/2
# (it's also faster than regular Poisson in JS, probably due to less calls to random - no difference in Nim)
proc samplePoissonUniform*(
  region: Region2f, 
  dist: fnum,
  rand: var Rand,
  seed: Vec2f = Vec2f.inf,
  inset: fnum = 0.0,
  maxit: int = 30, # Number of tries before giving up
): seq[Vec2f] =
  let insetSq = inset * inset
  let bbox = bbox(region).inset(inset)
  let cell = dist / sqrt(2.0)
  var grid = initGrid2[int](bbox, cell)
  grid.fill(-1)
  let distSq = dist * dist
  let r = dist + 1e-7
  var p1 = if seed == Vec2f.inf: rand.rand(region) else: seed
  grid[p1] = result.len
  result.add(p1)
  var queue = initDeque[Vec2f](maxit)
  queue.addLast(p1)
  while queue.len > 0:
    let p0 = queue.popFirst()
    let s = rand.rand(0.0..1.0)
    for it in 1..maxit: # Look for new candidates within an annulus of p0
      let theta = 2.0 * PI * (s + it.fnum / maxit.fnum)
      let p1 = p0 + cartesian(r, theta)
      if not p1.within(inset, insetSq, region):
        continue

      var valid = true
      let i0 = grid.cellOf(p1)
      for (i1, n) in grid.neighbors9(i0):
        if n != -1:
          if i1 == i0:
            valid = false
            break
          let neighbor = result[n]
          if neighbor.distSq(p1) < distSq:
            valid = false
            break

      if valid: # No neighbors too close, we can add this point
        grid[i0] = result.len
        result.add(p1)
        queue.addLast(p1)

proc samplePoissonUniform*(
  region: Region2f, 
  dist: fnum,
  rand: int = int.high,
  seed: Vec2f = Vec2f.inf,
  inset: fnum = 0.0,
  maxit: int = 30, # Number of tries before giving up
): seq[Vec2f] =
  var rand = if rand == int.high: initRand(rand(int.high)) else: initRand(rand)
  samplePoissonUniform(region, dist, rand, seed, inset, maxit)