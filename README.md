# Renderer
Prototype implementations of unbiased renderers.

![Produced by our simple raytracer](scenes/colored_cube.png)

## Next

* Path Tracer with Monte Carlo Sampling
* Metropolis Light Transport
* Radiosity Rendering

## Done

### Raycasting

![Cornell box rendered by our simple raycaster](scenes/cornell_raycast.png)

Generated with:
```(bash)
./raycaster scenes/cornell_box.blend 640 | convert - scenes/cornell.png
```

We just use the distance to the eye of the camera as a light measure. So there
is no real lighting.

## Raytracing

![Cornell box rendered by our raytracer](scenes/cornell_raytrace.png)

Generated with:
```(bash)
./raytrace scenes/cornell_box.blend -w 3840 --max-depth 3 > cornell.pbm
convert -resize 640x640 -interpolate bicubic cornerll.pbm cornell.png
```

We don't load material information yet. Every surface has the same reflectance.
Resizing the image is a simple antialiasing method.

## Model

The cornell box is modeled in Blender using the specifications on
http://www.graphics.cornell.edu/online/box/data.html. The camera and distances
are exactly the same. The colors may be different, since we did not transform
the original color definitions from the spectral space to rgb. Feel free to use
it in your projects.

[cornell_box.blend](scenes/cornell_box.blend)

## Thanks
* Sean Barrett for [stb](https://github.com/nothings/stb)'s image writer
