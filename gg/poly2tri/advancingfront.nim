type
  Node = ref object
    point: Point
    triangle: Triangle
    next, prev: Node
    value: float

  AdvancingFront = object
    head: Node
    tail: Node
    searchNode: Node

proc pos(node: Node): Vec2f {.inline.} = node.point.pos

proc newNode(point: Point, triangle: Triangle = nil): Node =
  Node(point: point, triangle: triangle, value: point.pos.x)

proc newAdvancingFront(head, tail: Node): AdvancingFront =
  AdvancingFront(head: head, tail: tail, searchNode: head)

proc findSearchNode(af: AdvancingFront, x: float): Node =
  # TODO: implement BST index
  af.searchNode

proc locateNode(af: var AdvancingFront, x: float): Node =
  var node = af.searchNode
  if x < node.value:
    node = node.prev
    while node != nil:
      if x >= node.value:
        af.searchNode = node
        return node
      node = node.prev
  else:
    node = node.next
    while node != nil:
      if x < node.value:
        af.searchNode = node.prev
        return node.prev
      node = node.next

proc locatePoint(af: var AdvancingFront, point: Point): Node =
  let px = point.pos.x
  var node = af.findSearchNode(px)
  let nx = node.point.pos.x

  if px == nx:
    # Here we are comparing point references, not values
    if point != node.point:
      # We might have two nodes with same x value for a short time
      if point == node.prev.point:
        node = node.prev
      elif point == node.next.point:
        node = node.next
      else:
        raise newException(AssertionDefect, "poly2tri Invalid AdvancingFront.locatePoint() call")
  elif px < nx:
    node = node.prev
    while node != nil and point != node.point:
      node = node.prev
  else:
    node = node.next
    while node != nil and point != node.point:
      node = node.next

  if node != nil:
    af.searchNode = node
  return node