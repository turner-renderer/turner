#include <cmath>

/**
 * Bresenham's line algorithm
 * @param x0, y0    line start
 * @param x1, y1    line end
 * @param set_pixel callback function: (int, int) -> void, which sets pixel
 *                  at the given position
 */
template <typename SetPixelFun>
void bresenham(int x0, int y0, int x1, int y1, SetPixelFun&& set_pixel) {
    int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy, e2; /* error value e_xy */

    while (1) {
        set_pixel(x0, y0);
        if (x0 == x1 && y0 == y1)
            break;
        e2 = 2 * err;
        if (e2 > dy) { // e_xy+e_x > 0
            err += dy;
            x0 += sx;
        }
        if (e2 < dx) { // e_xy+e_y < 0
            err += dx;
            y0 += sy;
        }
    }
}
