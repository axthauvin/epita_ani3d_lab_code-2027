# The Tom and Jerry Chase

A recreation of a chase inside a house between Tom the cat and Jerry the mouse, inspired by the MGM Cartoons series.

Jerry flees from Tom by following a procedurally generated path (navigation grid, A*, escape heuristics) and can escape through mouse holes in the walls. When Jerry disappears, Tom dashes toward the hole, bumps into the wall, flops to the ground surrounded by stun stars, then gets back up to search for him. A first-person view of Tom can be enabled. The entire scene is visible directly in the scene, along with a control and debug interface.

## Dependencies

The project is developed using Damien Rohmer’s CGP (Computer Graphics Programming) library: [https://graphicscomputing.fr/cgp/](https://graphicscomputing.fr/cgp/). The code of this project is based on one of his lab assignments on character animation, which provides the scene structure, animated model loading, and skinning.

## Compilation

The project compiles using CMake (C++17 or higher recommended).

```bash
cmake -S . -B build/
cmake --build build/ -j
```

```bash
./build/project
```

## Authors

- Axel Thauvin: axel.thauvin@epita.fr
- Gauthier Barier: gauthier.barier@epita.fr
