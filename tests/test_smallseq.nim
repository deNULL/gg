import gg/smallseq
import unittest

test "smallseq":
  #[
  let prev = getOccupiedMem() + 48
  #echo getOccupiedMem()
  #let x = 1
  #var y: seq[int]
  var z = sseq[int]()
  z.add(1)
  var t: array[2..5, int]
  var y = @@t
  #z.extra.add(1)
  #z
  echo "mem: ", getOccupiedMem() - prev
  echo sizeof(y)
  #echo sizeof(z)
  echo GC_getStatistics()
  ]#
  
  var s = newSSeq[int](100)
  var c = @@[1, 2, 3]
  echo s