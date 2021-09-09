import wNim / [wApp, wFrame, wMenuBar, wPanel, wPaintDC, wMemoryDC, wBitmap, wSlider, wStaticText, wBrush, wPen, wComboBox, wCheckBox, wButton, wSpinCtrl]

import gg/[vectors, randomvec, sampling, earcut, delaunay, poly2tri, cdtjs, voronoi]
import gg/cdt/[dt, edges]
import loader
import strformat
import options
import times
import random
import math

let allTests = loadTestSets()
let demoTests = allTests.demo
var demoNames: seq[string]
for test in demoTests:
  demoNames.add(test.content.get())
demoNames.add("Randomly deformed hexagon")

var input: TestData
var cells: seq[Cell]

# Returns list of current graph's points
let inputPoints = proc (): seq[Vec2f] =
  if input == nil:
    return
  result.add(input.contour)
  for hole in input.holes:
    result.add(hole)
  for p in input.steiner:
    result.add(p)

var points: proc (): seq[Vec2f] = inputPoints
  
let inputEdges = proc(): seq[Segment2f] =
  if input == nil:
    return
  for i in 0..<input.contour.len:
    result.add(input.contour[i] -> input.contour[(i + 1) mod input.contour.len])
  for hole in input.holes:
    for i in 0..<hole.len:
      result.add(hole[i] -> hole[(i + 1) mod hole.len])
var segments: proc (): seq[Segment2f] = inputEdges

var getVoroEdges: proc(): seq[Segment2f] = proc(): seq[Segment2f] = @[]
var getTriangles: proc (): seq[((Vec2f, Vec2f, Vec2f), int)] = proc(): seq[((Vec2f, Vec2f, Vec2f), int)] = @[]

var activePt = -1

let app = App(wSystemDpiAware)
let frame = Frame(title="Geometry & Graphs", size=(1300, 1000))
let menuBar = MenuBar(frame)
let panelLeft = Panel(frame)
let panelRight = Panel(frame)

# Shorthand for .scaleToDpi
proc dp(x: SomeNumber): SomeNumber {.inline.} = frame.dpiScale(x)

proc currentTransform(): (BBox2f, BBox2f) =
  var dataBBox = bbox(points())
  let viewBBox = bbox(Vec2f.zero, (float(panelLeft.size.width), float(panelLeft.size.height))).inset(60.0.dp)
  dataBBox = dataBBox.fit(viewBBox.aspect)
  (dataBBox, viewBBox)

let labelData = StaticText(panelRight, label="Data:")
let comboData = ComboBox(panelRight, style=wCbReadonly, choices=demoNames)
let buttonReload = Button(panelRight, label="Reload")

let labelAddRandom = StaticText(panelRight, label="Add random points:")
let comboSampling = ComboBox(panelRight, style=wCbReadonly, choices = @["Random", "Rect Grid", "Tri Grid", "Poisson Disk", "Poisson Disk Uniform"])
let spinRandomCnt = SpinCtrl(panelRight, value=120)
spinRandomCnt.setRange(1..500000)
let buttonAddRandom = Button(panelRight, label="Add")

let labelAlgo = StaticText(panelRight, label="Algorithm:")
let comboAlgo = ComboBox(panelRight, style=wCbReadonly, choices = @["source", "earcut", "earcut-bounds", "voronoi", "poly2tri", "StefanSalewski/cdt", "cdt-js", "delaunator", "delaunator-voronoi"])
let comboStrategy = ComboBox(panelRight, style=wCbReadonly, choices = @["Always Circumcenters", "Centroids", "Incenters", "Circumcenters or centroids", "Circumcenters or incenters", "Closest to circumcenters", "Always on closest edge", "Centroid/closest 25%", "Centroid/closest 50%", "Centroid/closest 75%", "Random", "Random on edge", "Random mid-edge"])
let buttonRebuild = Button(panelRight, label="Rebuild")

let checkConstrained = CheckBox(panelRight, label="Constrained")

let labelTime = StaticText(panelRight, label="-")

let buttonRelax = Button(panelRight, label="Relax extra points")

var memDc = MemoryDC()
var memBmp = Bitmap(panelLeft.size)
memDc.selectObject(memBmp)

let menu = Menu(menuBar, "&File")
menu.append(wIdExit, "Quit")

frame.wEvent_Menu do (event: wEvent):
  if event.id == wIdExit:
    frame.close()

panelLeft.wEvent_Paint do (event: wEvent):
  var dc = PaintDC(event.window)
  
  #dc.brush = Brush(color=0xD9AA85)
  #dc.drawEllipse(15, 35, 90, 50)

  let pen1 = Pen(color=0x0000FF)
  let pen2 = Pen(color=0x00FF00)
  let pen3 = Pen(color=0xFF0000)

  memDc.clear()
  let (dataBBox, viewBBox) = currentTransform()

  # First, draw triangles
  memDc.pen = wTransparentPen
  for (tri, color) in getTriangles():
    let (t0, t1, t2) = tri
    let p0 = t0.map(dataBBox, viewBBox).ivec
    let p1 = t1.map(dataBBox, viewBBox).ivec
    let p2 = t2.map(dataBBox, viewBBox).ivec
    memDc.brush = Brush(color=color.wColor)
    memDc.drawPolygon([p0, p1, p2])
  
  # First, draw edges
  memDc.pen = Pen(color=0x00c000, width = 5)
  for i, edge in segments():
    let p1 = edge.st.map(dataBBox, viewBBox).ivec
    let p2 = edge.en.map(dataBBox, viewBBox).ivec
    memDc.drawLine(p1.x, p1.y, p2.x, p2.y)

  memDc.pen = Pen(color=0xc000c0, width = 7)
  for i, edge in getVoroEdges():
    let p1 = edge.st.map(dataBBox, viewBBox).ivec
    let p2 = edge.en.map(dataBBox, viewBBox).ivec
    memDc.drawLine(p1.x, p1.y, p2.x, p2.y)

  # overdraw input edges
  memDc.pen = Pen(color=0xc00000, width = 7)
  for i, edge in inputEdges():
    let p1 = edge.st.map(dataBBox, viewBBox).ivec
    let p2 = edge.en.map(dataBBox, viewBBox).ivec
    memDc.drawLine(p1.x, p1.y, p2.x, p2.y)

  # Second, draw points
  memDc.pen = wTransparentPen
  memDc.brush = Brush(color=0x0000c0)
  #canvas.fontSize = 8.0.dp
  #canvas.fontFamily = "Consolas"
  for i, p in points():
    let vp = p.map(dataBBox, viewBBox).ivec
    #canvas.areaColor = if i == activePt: rgb(255, 0, 0) else: rgb(200, 0, 0)
    memDc.drawEllipse(vp.x - 4.dp.int, vp.y - 4.dp.int, 8.dp, 8.dp)
    #if i == activePt:
    #  canvas.drawText(&"{i}", vp.x, vp.y)
  
  dc.blit(source=memDc, width=panelLeft.size.width, height=panelLeft.size.height)
  dc.delete

proc rebuild() =
  getVoroEdges = proc(): seq[Segment2f] = @[]
  getTriangles = proc(): seq[((Vec2f, Vec2f, Vec2f), int)] = @[]
  let time0 = cpuTime()
  case comboAlgo.getListControl().getSelection():
  of 0: # source
    points = inputPoints
    segments = inputEdges

  of 1: # earcut
    var pts: seq[Vec2f] = input.contour
    var idx: seq[int]
    for hole in input.holes:
      idx.add(pts.len)
      pts.add(hole)

    let graph = earcut(pts, idx)
    points = inputPoints
    segments = proc (): seq[Segment2f] =
      for tri in graph:
        result.add(pts[tri[0]] -> pts[tri[1]])
        result.add(pts[tri[1]] -> pts[tri[2]])
        result.add(pts[tri[2]] -> pts[tri[0]])

  of 2: # earcut-bounds
    var bounds = input.contour.bbox
    bounds = bounds.inset(-min(bounds.width, bounds.height) * 0.5)
    var pts: seq[Vec2f]
    var idx: seq[int]
    pts.add(bounds.min)
    pts.add((bounds.max.x, bounds.min.y))
    pts.add(bounds.max)
    pts.add((bounds.min.x, bounds.max.y))
    idx.add(4)
    pts.add(input.contour)
    for hole in input.holes:
      idx.add(pts.len)
      pts.add(hole)

    let graph = earcut(pts, idx)
    points = proc (): seq[Vec2f] = pts
    segments = proc (): seq[Segment2f] =
      for tri in graph:
        result.add(pts[tri[0]] -> pts[tri[1]])
        result.add(pts[tri[1]] -> pts[tri[2]])
        result.add(pts[tri[2]] -> pts[tri[0]])

  of 3: # voronoi
    try:
      var pts: seq[Vec2f] = input.contour
      pts.add(input.steiner)
      for hole in input.holes:
        pts.add(hole)
      let vg = newVoronoi(pts, bbox((-100000.0, -100000.0), (100000.0, 100000.0)))
      points = proc (): seq[Vec2f] = pts
      segments = proc (): seq[Segment2f] =
        var edges = newSeq[Segment2f](vg.edges.len)
        for i, edge in vg.edges:
          edges[i] = vg.nodes[edge.nodes[0]].pos -> vg.nodes[edge.nodes[1]].pos
        return edges
    except:
      echo getCurrentExceptionMsg()
      echo getCurrentException().getStackTrace()
      raise

  of 4: # poly2tri
    #try:
    var pts = input.contour
    var ctx = newSweepContext(pts)

    if checkConstrained.isChecked:
      ctx.addHoles(input.holes)
    else:
      for hole in input.holes:
        ctx.addPoints(hole)
    
    ctx.addPoints(input.steiner)

    ctx.triangulate()
    echo "done, triangles: ", ctx.triangles.len
    segments = proc (): seq[Segment2f] =
      for tri in ctx.triangles:
        result.add(tri[0].pos -> tri[1].pos)
        result.add(tri[1].pos -> tri[2].pos)
        result.add(tri[2].pos -> tri[0].pos)
    #[
    except PointError:
      let e = getCurrentException().PointError
      let msg = getCurrentExceptionMsg()
      echo msg
      echo e.getStackTrace()
      for p in e.points:
        echo " #" & $p.id & ": " & $p.pos
      raise
    ]#
    points = inputPoints

  of 5: # StefanSalewski/cdt
    let bbox = input.contour.bbox.inset(-100.0)
    var dt: DelaunayTriangulation = initDelaunayTriangulation(bbox.min, bbox.max)
    for i, p0 in input.contour:
      let p1 = input.contour[(i + 1) mod input.contour.len]
      discard dt.insert(p0, p1)
    for hole in input.holes:
      for i, p0 in hole:
        let p1 = hole[(i + 1) mod hole.len]
        discard dt.insert(p0, p1)
    for p in input.steiner:
      discard dt.insert(p)
    points = inputPoints
    segments = proc (): seq[Segment2f] =
      for e in dt.edges:
        result.add(e.org.point -> e.dest.point)

  of 6: # cdt-js
  
    var pts = input.contour
    var segm = newSeq[(int, int)]()
    for i in 0..<pts.len:
      segm.add((i, (i + 1) mod pts.len))
    for hole in input.holes:
      for i in 0..<hole.len:
        segm.add((pts.len + i, pts.len + (i + 1) mod hole.len))
      pts.add(hole)
    pts.add(input.steiner)
    try: 
      var mesh = newMesh(pts, segm)
      mesh.triangulate()
      points = proc (): seq[Vec2f] = pts
      segments = proc (): seq[Segment2f] =
        for tri in mesh.tri:
          let coords = [pts[tri[0]], pts[tri[1]], pts[tri[2]]]
          result.add(coords[0] -> coords[1])
          result.add(coords[1] -> coords[2])
          result.add(coords[2] -> coords[0])
    except:
      echo getCurrentExceptionMsg()
      echo getCurrentException().getStackTrace()

  of 7: # Delaunator + Constrainautor
    var pts = input.contour
    var segm = newSeq[(int, int)]()
    var contour = newSeq[int]()
    var holes = newSeq[seq[int]]()
    for i in 0..<pts.len:
      segm.add((i, (i + 1) mod pts.len))
      contour.add(i)
    for hole in input.holes:
      var h = newSeq[int]()
      for i in 0..<hole.len:
        segm.add((pts.len + i, pts.len + (i + 1) mod hole.len))
        h.add(pts.len + i)
      pts.add(hole)
      holes.add(h)
    pts.add(input.steiner)
    try: 
      var del = newDelaunator(pts)
      if segm.len > 0 and checkConstrained.isChecked:
        let constrainautor = newConstrainautor(del)
        constrainautor.constrainAll(segm)
        del = constrainautor.del

        let types = constrainautor.classifyTriangles(contour, holes)
        getTriangles = proc (): seq[((Vec2f, Vec2f, Vec2f), int)] =
          for i, ts in types:
            if ts != tsInside:
              result.add(((pts[del.triangles[i * 3]], pts[del.triangles[i * 3 + 1]], pts[del.triangles[i * 3 + 2]]), if ts == tsOutside: 0xAAAAFF else: 0xFFAAAA))

      points = proc (): seq[Vec2f] = pts
      segments = proc (): seq[Segment2f] =
        for i in countup(0, del.triangles.len - 3, 3):
          let tri = [del.triangles[i], del.triangles[i + 1], del.triangles[i + 2]]
          result.add(pts[tri[0]] -> pts[tri[1]])
          result.add(pts[tri[1]] -> pts[tri[2]])
          result.add(pts[tri[2]] -> pts[tri[0]])
    except:
      echo getCurrentExceptionMsg()
      echo getCurrentException().getStackTrace()

  of 8: # Delaunator + Constrainautor -> Voronoi
    var pts = input.contour
    var segm = newSeq[(int, int)]()
    var contour = newSeq[int]()
    var holes = newSeq[seq[int]]()
    for i in 0..<pts.len:
      segm.add((i, (i + 1) mod pts.len))
      contour.add(i)
    for hole in input.holes:
      var h = newSeq[int]()
      for i in 0..<hole.len:
        segm.add((pts.len + i, pts.len + (i + 1) mod hole.len))
        h.add(pts.len + i)
      pts.add(hole)
      holes.add(h)
    pts.add(input.steiner)
    try: 
      var delaunator = newDelaunator(pts)
      let constrainautor = newConstrainautor(delaunator)
      if segm.len > 0 and checkConstrained.isChecked:
        constrainautor.constrainAll(segm)
        constrainautor.deleteExternalTriangles(contour, holes)

      let strategyIdx = comboStrategy.getListControl().getSelection()
      var strategy: NodePositionStrategy
      if strategyIdx < npsStrategies.len:
        strategy = npsStrategies[strategyIdx]
      elif strategyIdx == npsStrategies.len:
        strategy = npsCircumcenterOrMix(0.25)
      elif strategyIdx == npsStrategies.len + 1:
        strategy = npsCircumcenterOrMix(0.50)
      elif strategyIdx == npsStrategies.len + 2:
        strategy = npsCircumcenterOrMix(0.75)
      elif strategyIdx == npsStrategies.len + 3:
        strategy = npsRandom()
      elif strategyIdx == npsStrategies.len + 4:
        strategy = npsRandomEdge()
      else:
        strategy = npsRandomMidEdge()
      cells = constrainautor.cells(strategy)
      points = proc (): seq[Vec2f] = pts
      segments = proc (): seq[Segment2f] =
        for i in countup(0, delaunator.triangles.len - 3, 3):
          let tri = [delaunator.triangles[i], delaunator.triangles[i + 1], delaunator.triangles[i + 2]]
          result.add(pts[tri[0]] -> pts[tri[1]])
          result.add(pts[tri[1]] -> pts[tri[2]])
          result.add(pts[tri[2]] -> pts[tri[0]])
      getVoroEdges = proc (): seq[Segment2f] =
        for cell in cells:
          for i, p0 in cell.points:
            let p1 = cell.points[(i + 1) mod cell.points.len]
            result.add(p0 -> p1)
        #[
        for e in 0..<delaunator.halfedges.len:
          if delaunator.halfedges[e] == -1:
            # Part of hull
            let p = delaunator.triangleCenterInside(triangleOfEdge(e))
            let q = delaunator.edgeCenter(e)
            result.add(segment(p, q))
          elif e < delaunator.halfedges[e]:
            let p = delaunator.triangleCenterInside(triangleOfEdge(e))
            let q = delaunator.triangleCenterInside(triangleOfEdge(delaunator.halfedges[e]))
            result.add(segment(p, q))
        ]#
    except:
      echo getCurrentExceptionMsg()
      echo getCurrentException().getStackTrace()

  else:
    discard

  labelTime.setLabel($((cpuTime() - time0) * 1000.0) & " ms")
  panelLeft.refresh(eraseBackground=false)

proc layout() =
  let rightWidth = 400
  let clientSize = frame.getClientSize()
  panelLeft.size = (clientSize.width - rightWidth, clientSize.height)
  panelLeft.position = (0, 0)

  panelRight.size = (rightWidth, clientSize.height)
  panelRight.position = (clientSize.width - rightWidth, 0)

  var y = 0
  labelData.size = (rightWidth, labelData.bestSize.height)
  labelData.position = (0, y)
  y += labelData.size.height
  
  comboData.size = (rightWidth, comboData.bestSize.height)
  comboData.position = (0, y)
  y += comboData.size.height

  buttonReload.size = (rightWidth, buttonReload.bestSize.height)
  buttonReload.position = (0, y)
  y += buttonReload.size.height

  labelAddRandom.size = (rightWidth, labelAddRandom.bestSize.height)
  labelAddRandom.position = (0, y)
  y += labelAddRandom.size.height

  comboSampling.size = (rightWidth, comboSampling.bestSize.height)
  comboSampling.position = (0, y)
  y += comboSampling.size.height

  spinRandomCnt.size = (rightWidth div 2, spinRandomCnt.bestSize.height)
  spinRandomCnt.position = (0, y)
  buttonAddRandom.size = (rightWidth div 2, spinRandomCnt.bestSize.height)
  buttonAddRandom.position = (rightWidth div 2, y)
  y += spinRandomCnt.size.height

  labelAlgo.size = (rightWidth, labelAlgo.bestSize.height)
  labelAlgo.position = (0, y)
  y += labelAlgo.size.height
  
  comboAlgo.size = (rightWidth, comboAlgo.bestSize.height)
  comboAlgo.position = (0, y)
  y += comboAlgo.size.height
  comboStrategy.size = (rightWidth, comboStrategy.bestSize.height)
  comboStrategy.position = (0, y)
  y += comboStrategy.size.height
  
  checkConstrained.size = (rightWidth, checkConstrained.bestSize.height)
  checkConstrained.position = (0, y)
  y += checkConstrained.size.height

  buttonRebuild.size = (rightWidth, buttonRebuild.bestSize.height)
  buttonRebuild.position = (0, y)
  y += buttonRebuild.size.height

  labelTime.size = (rightWidth, labelTime.bestSize.height)
  labelTime.position = (0, y)
  y += labelTime.size.height

  buttonRelax.size = (rightWidth, buttonRelax.bestSize.height)
  buttonRelax.position = (0, y)
  y += buttonRelax.size.height

  memBmp = Bitmap(panelLeft.size)
  memDc.selectObject(memBmp)

frame.wEvent_Size do ():
  layout()

proc loadData(test: TestFile) =
  input = test.load()
  panelLeft.refresh(eraseBackground=false)

proc displace(rand: var Rand, points: var seq[float], rng: Slice[int], d: float) =
  if rng.b - rng.a <= 1:
    return
  let mid = (rng.a + rng.b) div 2
  points[mid] = (points[rng.a] + points[rng.b]) * 0.5 + rand.rand(-d..d)
  displace(rand, points, rng.a..mid, d * 0.5)
  displace(rand, points, mid..rng.b, d * 0.5)

proc generateHexagon() =
  input = TestData()
  let subd = 8
  let r = 100.0
  var rnd = initRand(rand(int.high))
  let rough = 13.0
  let disp = 30.0
  var points: array[6, Vec2f]
  for side in 0..5:
    let angle = 2 * PI * side.float / 6
    points[side] = cartesian(r, angle) + rnd.rand((-disp, -disp)..(disp, disp))
  for side, p0 in points:
    let p1 = points[(side + 1) mod 6]
    let norm = (p1 - p0).perp.norm
    var offsets = newSeq[float](subd + 1)
    displace(rnd, offsets, 0..subd, rough)
    for i in 0..<subd:
      let p = mix(p0, p1, i.float / subd.float) + offsets[i] * norm
      input.contour.add(p)

proc reloadData() =
  #comboAlgo.getListControl().setSelection(8)
  #comboAlgo.refresh()
  cells.setLen(0)
  let idx = comboData.getListControl().getSelection()
  if idx < demoTests.len:
    loadData(demoTests[idx])
  else:
    generateHexagon()
  rebuild()

comboData.wEvent_ComboBox do ():
  reloadData()

buttonReload.wEvent_Button do ():
  reloadData()

comboAlgo.wEvent_ComboBox do ():
  rebuild()

checkConstrained.wEvent_CheckBox do ():
  rebuild()

comboStrategy.wEvent_ComboBox do ():
  rebuild()

buttonAddRandom.wEvent_Button do ():
  randomize()
  
  echo "Contour area: ", input.contour.area
  let region = input.contour.poke(input.holes)
  let dist = sqrt(region.area / spinRandomCnt.value.float) * 0.65
  let bbox = region.bbox
  let mind = min(bbox.width, bbox.height) * 0.003
  echo "BBox sise: ", bbox.size
  echo "Min dist: ", mind
  echo "Count: ", spinRandomCnt.value
  echo "Dist: ", dist
  #let samples = samplePoissonFast(holed, dist, r)
  #let samples = sampleRectGrid(holed, dist, angle=PI*0.3)
  var samples: seq[Vec2f]
  let time0 = cpuTime()
  case comboSampling.getListControl().getSelection():
  of 0: samples = sampleRandom(region, spinRandomCnt.value, mind, inset = mind)
  of 1: samples = sampleRectGrid(region, dist, jitter = dist * 0.1, angle = PI*0.3, inset = dist * 0.5)
  of 2: samples = sampleTriGrid(region, dist, jitter = dist * 0.1, angle = PI*0.3, inset = dist * 0.5)
  of 3: samples = samplePoisson(region, dist / 1.2, inset = dist * 0.5)
  of 4: samples = samplePoissonUniform(region, dist, inset = dist * 0.5)
  else: discard
  echo "Added " & $samples.len & " samples"
  labelTime.setLabel($((cpuTime() - time0) * 1000.0) & " ms")
  #input.steiner.add(samples)
  input.steiner = samples
  #rebuild()
  panelLeft.refresh(eraseBackground=false)

buttonRebuild.wEvent_Button do ():
  rebuild()

buttonRelax.wEvent_Button do ():
  # Perform Lloyd relaxation (on steiner points only)
  var offs = input.contour.len
  for hole in input.holes:
    offs += hole.len
  
  for i, cell in cells:
    if i >= offs:
      input.steiner[i - offs] = avg(cell.points)
  rebuild()


comboData.getListControl().setSelection(demoNames.len - 1)
comboAlgo.getListControl().setSelection(0)
comboSampling.getListControl().setSelection(0)
comboStrategy.getListControl().setSelection(0)
layout()
generateHexagon()
frame.center()
frame.show()
app.mainLoop()