# Utility module for loading test polys to test triangulation
# (files in tests/data directory are from https://github.com/r3mi/poly2tri.js)

import gg/vectors
import parseutils
import math
import json
import options

type
  TestFile* = ref object
    name*: string
    content*: Option[string]
    demo*: Option[bool]
    steiner*: Option[string]
    holes*: Option[string]
    throws*: Option[string]
    TODO*: Option[string]

  TestData* = ref object
    contour*: Path2f
    holes*: seq[Path2f]
    steiner*: seq[Vec2f]

  TestSet* = ref object
    title*: string
    source*: string
    files*: seq[TestFile]

proc loadTestSets*(): seq[TestSet] =
  let testsJson = parseFile("tests/data/index.json")
  return to(testsJson, seq[TestSet])

proc demo*(testSets: seq[TestSet]): seq[TestFile] =
  for tests in testSets:
    for test in tests.files:
      if test.demo.isSome and test.demo.get():
        result.add(test)

proc loadPaths*(fname: string): seq[Path2f] =
  var f: File
  if open(f, fname):
    var line: string
    var emptyCnt = 0
    var prevNum: float = NaN
    var path: Path2f
    while f.readLine(line):
      if line.len == 0:
        emptyCnt += 1
        continue
      if emptyCnt > 0 and path.len > 0:
        result.add(path)
        path = @[]
        prevNum = NaN
      emptyCnt = 0
      var i = 0
      while i < line.len:
        i += line.skipUntil({'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', '.', '-'}, i)
        var num: float
        let n = line.parseFloat(num, i)
        if n > 0:
          i += n
          if prevNum.classify == fcNaN:
            prevNum = num
          else:
            path.add((prevNum, num))
            prevNum = NaN
        else:
          i += 1
    if path.len > 0:
      result.add(path)
    close(f)

proc load*(test: TestFile): TestData =
  TestData(
    contour: loadPaths("tests/data/" & test.name)[0],
    holes: if test.holes.isSome: loadPaths("tests/data/" & test.holes.get()) else: @[],
    steiner: if test.steiner.isSome: loadPaths("tests/data/" & test.steiner.get())[0] else: @[]
  )
