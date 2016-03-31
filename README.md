
# Renderer [![Build Status](https://travis-ci.org/blacklab/renderer.svg?branch=master)](https://travis-ci.org/blacklab/renderer) [![Coverage Status](https://coveralls.io/repos/github/blacklab/renderer/badge.svg?branch=master)](https://coveralls.io/github/blacklab/renderer?branch=master) [![Stories in Ready](https://badge.waffle.io/blacklab/renderer.png?label=ready&title=Ready)](https://waffle.io/blacklab/renderer) [![Apache 2.0 License](https://img.shields.io/badge/license-Apache%202.0-blue.svg)](LICENSE)

Prototype implementations of unbiased renderers.

![Produced by our simple raytracer](scenes/colored_cube.png)

## Build

```
make            # build renderers
make test       # optional: run tests
```

## Next

* Path Tracer with Monte Carlo Sampling
* Metropolis Light Transport
* Radiosity Rendering

## Done

### Raycasting

[![Cornell box rendered by our simple raycaster](scenes/cornell_raycast_small.png)](scenes/cornell_raycast.png)

Generated with:
```(bash)
./raycaster scenes/cornell_box.blend 3840 > cornell.pbm
convert -resize 640x640 -interpolate bicubic cornell.pbm cornell.png
```

We just use the distance to the eye of the camera as a light measure. So there
is no real lighting.

### Raytracing

[![Cornell box rendered by our raytracer](scenes/cornell_raytrace_small.png)](scenes/cornell_raytrace.png)

Generated with:
```(bash)
./raytracer scenes/cornell_box.blend -w 3840 --max-depth 3 > cornell.pbm
convert -resize 640x640 -interpolate bicubic cornell.pbm cornell.png
```

We don't load material information yet. Every surface has the same reflectance.
Resizing the image is a simple antialiasing method.

## Pathtracing

[![Cornell box rendered by our pathtracer](scenes/cornell_box_pathtrace_300_4_128.png)](scenes/cornell_box_pathtrace_3480_4_128.png)

Generated with:
```(bash)
./pathtracer scenes/cornell_box.blend -w 3480 --max-depth 3 -m 4 --pixel-samples 128 > cornell_box_3480_3_4_128.pbm
convert cornell_box_3480_3_4_128.pbm cornell_box_3480_3_4_128.png
```

Overall brightness is corrected in an external program. Since we are using a point light, the original picture is slightly too dark. The rendering time on 8 cpus was ~28 hours.

[![Stanford dragon rendered by our pathtracer](scenes/stanford_dragon_small.png)](scenes/stanford_dragon.png)

Generated with:
```(bash)
./pathtracer scenes/stanford_dragon.blend -w 1920 --max-depth 3 -m 4 --pixel-samples 128 > stanford_dragon.pbm
convert stanford_dragon.pbm stanford_dragon.png
```

It took roughly 26 hours to render.

## Model

The cornell box is modeled in Blender using the specifications on
http://www.graphics.cornell.edu/online/box/data.html. The camera and distances
are exactly the same. The colors may be different, since we did not transform
the original color definitions from the spectral space to rgb. Feel free to use
it in your projects.

[cornell_box.blend](scenes/cornell_box.blend)

## Algorithm Implementations from Scratch

The following algorithms have been implemented from scratch. While most are not
that complicated it is good practice and gives a nice overview of all things
required for rendering.

- [x] Xorshift64*
- [x] Lambertian Reflectance
- [x] Ray-Triangle Intersection after [Dan Sunday](http://geomalgorithms.com/a06-_intersect-2.html)
- [x] Barycentric Interpolation
- [x] Monte Carlo Integration for Hemisphere Sampling
- [x] Polar Coordinates to Cartesian Coordinates
- [x] KD-Tree

