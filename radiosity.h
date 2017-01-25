#pragma once

static const char* USAGE =
    R"(Usage:
  radiosity (exact|hierarchical) [options] <filename>

Options:
  -w --width=<px>               Width of the image [default: 640].
  -a --aspect=<num>             Aspect ratio of the image. If the model has
                                specified the aspect ratio, it will be
                                used. Otherwise default value is 1.
  --background=<3x float>       Background color [default: 0 0 0].
  -t --threads=<int>            Number of threads [default: 1].
  --inverse-gamma=<float>       Inverse of gamma for gamma correction
                                [default: 0.454545].
  --no-gamma-correction         Disables gamma correction.
  -e --exposure=<float>         Exposure of the image [default: 1.0].

Hierarchical radiosity options:
  --form-factor-eps=<float>     Link when form factor estimate is below
                                [default: 0.04].
  --rad-shoot-eps=<float>       Refine link when shooting radiosity times
                                form factor is too high [default: 1e-6].
  --max-subdivisions=<int>      Maximum number of subdivisions for smallest
                                triangle [default: 3].
  --max-iterations=<int>        Maximum iterations to solve system [default: 3].

Radiosity options:
  -g --gouraud                  Enable gouraud shading for hierarchical
                                radiosity.
  --mesh=<simple|feature>       Render mesh either without taking depth of
                                objects into account (simple), which is fast, or
                                by computing the correct overlapping (feature),
                                which is slow.
  --links                       Render hierarchical radiosity links.
  --exact                       Render hierarchical mesh with exact radiosity
                                solution. For debugging only.
)";
