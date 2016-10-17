#pragma once

#include "types.h"
#include <math.h>

float exposure(float light, float exposure_value) {
    return 1 - expf(-light * exposure_value);
}

Color exposure(Color light, float exposure_value) {
    return {exposure(light.r, exposure_value),
            exposure(light.g, exposure_value),
            exposure(light.b, exposure_value), light.a};
}

float gamma(float light, float inverse_gamma) {
    return powf(light, inverse_gamma);
}

Color gamma(Color light, float inverse_gamma) {
    return {gamma(light.r, inverse_gamma), gamma(light.g, inverse_gamma),
            gamma(light.b, inverse_gamma), light.a};
}
