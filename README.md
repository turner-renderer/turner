# Renderer
Prototype implementations of unbiased renderers.

![Produced by our simple raytraces](scenes/colored_cube.png)

## Next

* Ray Tracer
* Path Tracer with Monte Carlo Sampling
* Metropolis Light Transport
* Radiosity Rendering

## Done

### Raycasting

![Cornell box rendered by our simple raycaster](scenes/cornell.png)

Generated with:
```(bash)
./raycaster scenes/cornell_box.blend 640 | convert - scenes/cornell_box.png
```

We just use the distance to the eye of the camera as a light measure. So there
is no real lighting.

## Thanks
* Sean Barrett for [stb](https://github.com/nothings/stb)'s image writer

