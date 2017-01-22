#pragma once

extern const char* USAGE;
const char* USAGE =
    R"(Usage: raytracer <filename> [options]

Options:
  -w --width=<px>           Width of the image [default: 640].
  -a --aspect=<num>         Aspect ratio of the image. If the model has
                            specified the aspect ratio, it will be used
                            [default: 1].
  --background=<3x float>   Background color [default: 0 0 0].
  -t --threads=<int>        Number of threads [default: 1].
  --inverse-gamma=<float>   Inverse of gamma for gamma correction
                            [default: 0.454545].
  --no-gamma-correction     Disables gamma correction.
  --exposure=<float>        Exposure [default: 1].

Raytracer options:
  -d --max-depth=<int>      Maximum recursion depth for raytracing [default: 3].
  --shadow=<float>          Intensity of shadow [default: 0.5].
)";
