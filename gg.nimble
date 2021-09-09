# Package

version       = "0.1.0"
author        = "Denis Olshin"
description   = "Geometry & Graphs: a collection of 2D/3D data structures and algorithms"
license       = "MIT"

# Dependencies

requires "nim >= 1.4.6"

# Custom task for running demo application. Note: requires wNim, so currently supports Windows only
task demo, "Run demo app":
  exec("nim c -r tests/demo.nim")