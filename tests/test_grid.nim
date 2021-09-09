import unittest
import gg/grid

test "Grid":
  #var grid = initGrid2[int](200, 300)
  #echo grid[2, 3]

  var grid2 = initGrid2[int]((0.5, 0.5), 0.1)
  for idx, value in grid2.neighbors5((0.3, 0.2)):
    echo idx, value