
type
  PriorityQueue*[T] = seq[T]

proc moveUp[T](pq: var PriorityQueue[T], pos: int): int =
  result = pos
  var node = pq[result]
  var parent = result shr 1
  while result > 1 and pq[parent] > node:
    pq[result] = pq[parent]
    pq[result].pqpos = result
    result = parent
    parent = parent shr 1
  node.pqpos = result
  pq[result] = node

proc maxChild[T](pq: PriorityQueue[T], pos: int): int =
  var child = pos shl 1
  if child >= pq.len - 1:
    return 0
  if child + 1 < pq.len - 1 and pq[child] > pq[child + 1]:
    return child + 1
  return child

proc moveDown[T](pq: var PriorityQueue[T], pos: int): int =
  if pos >= pq.len:
    return
  result = pos
  var node = pq[result]
  var child = pq.maxChild(result)
  while child != 0 and node > pq[child]:
    pq[result] = pq[child]
    pq[result].pqpos = result
    result = child
    child = pq.maxChild(result)
  pq[result] = node
  pq[result].pqpos = result

proc initPriorityQueue*[T](capacity: int): PriorityQueue[T] =
  result = newSeqOfCap[T](capacity)
  result.add(nil)

proc empty*[T](pq: PriorityQueue[T]): bool = pq.len == 1

proc push*[T](pq: var PriorityQueue[T], node: T) =
  pq.add(node)
  discard pq.moveUp(pq.high)

proc pop*[T](pq: var PriorityQueue[T]): T =
  result = pq[1]
  pq.del(1)
  discard pq.moveDown(1)

proc top*[T](pq: PriorityQueue[T]): T = pq[1]

proc remove*[T](pq: var PriorityQueue[T], node: T) =
  if pq.len == 1:
    return
  var pos = node.pqpos
  if pos == 0:
    return
  pq.del(pos)
  if pos == pq.len:
    return
  if node > pq[pos]:
    discard pq.moveUp(pos)
  else:
    discard pq.moveDown(pos)
  node.pqpos = pos
