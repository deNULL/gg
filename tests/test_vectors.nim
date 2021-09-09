import gg/[vectors, randomvec, sampling]
import unittest
import random

test "Sum vectors":
  let a: Vec2f = (1.0, 2.0)
  let b: Vec2f = (3.0, 4.0)
  check a + b == (4.0, 6.0)

test "Rand vector":
  var rnd = initRand(42)
  let a = rnd.rand((300.0, 100.0)..(333.0, 200.0))
  check a ~= (300.00000516325, 108.715851666)

test "Sampling":
  var rnd = initRand(42)
  let x = (300.0, 100.0)..(333.0, 200.0)
  let pts = x.samplePoisson(10.0, rnd)
  echo pts