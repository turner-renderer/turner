# Turner [![Build Status](https://travis-ci.org/turner-renderer/turner.svg?branch=master)](https://travis-ci.org/turner-renderer/turner) [![Coverage Status](https://coveralls.io/repos/github/turner-renderer/turner/badge.svg?branch=master)](https://coveralls.io/github/turner-renderer/turner?branch=master)

A Collection of Unbiased Renderers

[![Stanford dragon rendered by our pathtracer](https://f001.backblazeb2.com/file/turner/stanford_dragon_small.png)](https://f001.backblazeb2.com/file/turner/stanford_dragon.png)

## Get Started

Build

```bash
> mkdir build
> cd build
> cmake ..
> make            # build renderers
> make test       # optional: run tests
```

Render

```bash
> ./pathtracer ../scenes/cornell_box.blend \
  --width 320 \                            # image width in pixels
  --max-depth 3 \                          # max number of ray bounces
  --monte-carlo-samples 1 \                # number of samples for ray bounce
  --pixel-samples 8 > cornell_box.pbm      # numer of samples per pixel
Loading scene...
Loading triangles and building kd-tree...
Rendering           ■■■■■■■■■■■■■■■■■■■■ 100.00%
Triangles      : 36
Kd-Tree Height : 0
Rays           : 2632399
Rays (primary) : 819200
Rays/sec       : 129419
Loading time   : 0.00 sec
Rendering time : 20.34 sec
```

The renderers output the image as [PBM](https://en.wikipedia.org/wiki/Netpbm_format#PBM_example)
to stdout. It can be easily converted with [ImageMagick](https://www.imagemagick.org/script/index.php).

```bash
> convert cornell_box.pbm cornell_box.png
```

## Rendered Images

### Raycasting

[![Cornell box rendered by our simple raycaster](https://f001.backblazeb2.com/file/turner/cornell_raycast_small.png)](https://f001.backblazeb2.com/file/turner/cornell_raycast.png)

Generated with:
```(bash)
./raycaster ../scenes/cornell_box.blend 3840 > cornell.pbm
convert -resize 640x640 -interpolate bicubic cornell.pbm cornell.png
```

We just use the distance to the eye of the camera as a light measure. So there
is no real lighting.

### Raytracing

[![Cornell box rendered by our raytracer](https://f001.backblazeb2.com/file/turner/cornell_raytrace_small.png)](https://f001.backblazeb2.com/file/turner/cornell_raytrace.png)

Generated with:
```(bash)
./raytracer ../scenes/cornell_box.blend -w 3840 --max-depth 3 > cornell.pbm
convert -resize 640x640 -interpolate bicubic cornell.pbm cornell.png
```

We don't load material information yet. Every surface has the same reflectance.
Resizing the image is a simple antialiasing method.

### Pathtracing

[![Cornell box rendered by our pathtracer](https://f001.backblazeb2.com/file/turner/cornell_box_pathtrace_small.png)](https://f001.backblazeb2.com/file/turner/cornell_box_pathtrace_3480_4_128.png)

Generated with:
```(bash)
./pathtracer ../scenes/cornell_box.blend -w 3480 --max-depth 3 -m 4 --pixel-samples 128 > cornell_box_3480_3_4_128.pbm
convert cornell_box_3480_3_4_128.pbm cornell_box_3480_3_4_128.png
```

Overall brightness is corrected in an external program. Since we are using a point light, the original picture is slightly too dark. The rendering time on 8 cpus was ~28 hours.

[![Stanford dragon rendered by our pathtracer](https://f001.backblazeb2.com/file/turner/dragon_xs.png)](https://f001.backblazeb2.com/file/turner/stanford_dragon.png)

Generated with:
```(bash)
./pathtracer ../scenes/stanford_dragon.blend -w 1920 --max-depth 3 -m 4 --pixel-samples 128 > stanford_dragon.pbm
convert stanford_dragon.pbm stanford_dragon.png
```

It took roughly 26 hours to render. The Blender file can be found [here](https://f001.backblazeb2.com/b2api/v1/b2_download_file_by_id?fileId=4_zb374779d699429f35cae071c_f1050377135d1e264_d20170211_m230826_c001_v0001036_t0059).

### Model

The cornell box is modeled in Blender using the specifications on
http://www.graphics.cornell.edu/online/box/data.html. The camera and distances
are exactly the same. The colors may be different, since we did not transform
the original color definitions from the spectral space to rgb. Feel free to use
it in your projects.

[cornell_box.blend](scenes/cornell_box.blend)

## References

<a name="RB06"></a>[RB06] Richard P. Brent. _Some Long-Period Random Number Generators using Shifts and Xors_, In _ANZIAM Journal. 48: C188–C202, July 2006_. 

<a name="CW93"></a>[CW93] Michael F. Cohen and John R. Wallace. _Radiosity and Realistic Image Synthesis_, Academic Press Inc., 1993.

<a name="HH11"></a>[HH11] M. Hapala and Vlastimil Havran. Review: Kd-tree Traversal Algorithms for Ray Tracing. In _Computer Graphics Forum, Volume 30, Issue 1, pages 199–213, March 2011_.

<a name="WH06"></a>[WH06] Ingo Wald and Vlastimil Havran. On building fast kd-Trees for Ray Tracing, and on doing that in O(N log N). SCI Technical Report 2006-009.
