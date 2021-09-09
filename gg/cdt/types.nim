import gg/vectors

type
  ConstraintID* = distinct int

  QuadEdgeID* = distinct int

  EdgeID* = distinct int

  VertexID* = distinct int

type
  Vertex* = ref object
    point*: Vec2f
    edge*: Edge
    id*: VertexID
    seqPos*: int
    mark*: int

  QuadEdge* = ref object
    e*: array[4, Edge]
    crep*: seq[ConstraintID]
    id*: QuadEdgeID
    mark*: int

  Edge* = ref object
    quadEdge*: QuadEdge
    vertex*: Vertex
    next*{.cursor.}: Edge # added {.cursor.} for v 0.1.1
    num*: int # 0 .. 3

proc `$`*(v: Vertex): string = '(' & $v.point.x & ", " & $v.point.y & ')'
