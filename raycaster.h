#pragma once

extern const char* USAGE;
const char* USAGE =
    R"(Usage: raycaster <filename> [options]

Options:
  -w --width=<px>            Width of the image [default: 640].
  -a --aspect=<num>          Aspect ratio of the image. If the model has
                             specified the aspect ratio, it will be used
                             [default: 1].
  --background=<3x float>    Background color [default: 0 0 0].
  -t --threads=<int>         Number of threads [default: 1].
  --inverse-gamma=<float>    Inverse of gamma for gamma correction
                             [default: 0.454545].
  --no-gamma-correction      Disables gamma correction.
  --exposure=<float>         Exposure [default: 1].

Raycaster options:
  --max-visibility=<float>   Any object farther away is dark [default: 2.0].
)";
