import gg/[vectors, quadtree]
import unittest

test "Quadtree":
  let tree = initQuadtree[Vec2f]()
  echo tree