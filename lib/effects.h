#pragma once

#include "types.h"
#include <math.h>

/**
 * Apply explosure to given light.
 * @param  light light intensity in [0,1]
 * @param  value in [0,infty); 0 - no light,
 *               small value - darker light
 *               big value - brighter light
 *               0 << value - white
 * @return 		 light with applied explosure
 */
float exposure(float light, float value) { return 1 - expf(-light * value); }

/**
 * Apply explosure to the rgb channels of the given color.
 * @param  c     color
 * @param  value explosure (cf. overloaded explosure function)
 * @return       color with applied explosure
 */
Color exposure(const Color& c, float value) {
    return {exposure(c.r, value), exposure(c.g, value), exposure(c.b, value),
            c.a};
}

/**
 * Apply gamma to given light.
 * @param  light         light intensity in [0,1]
 * @param  inverse_gamma 1.f/gamma factor [default: gamma correction]
 * @return               light with applied gamma
 */
float gamma(float light, float inverse_gamma = 1 / 2.2f) {
    return powf(light, inverse_gamma);
}

/**
 * Apply gamma to the rgb channels of the given color.
 * @param  c         	 color
 * @param  inverse_gamma 1.f/gamma factor [default: gamma correction]
 * @return               color with applied gamma
 */
Color gamma(Color c, float inverse_gamma = 1 / 2.2f) {
    return {gamma(c.r, inverse_gamma), gamma(c.g, inverse_gamma),
            gamma(c.b, inverse_gamma), c.a};
}
