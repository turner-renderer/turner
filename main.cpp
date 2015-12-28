#include <iostream>
#include <fstream>
#include <vector>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

struct Color {
    uint8_t r;
    uint8_t b;
    uint8_t g;
};

const Color WHITE = { 255, 255, 255 };
const Color RED = { 255, 0, 0 };

class Image {
public:
    // The pixels are stored left-to-right and top-to-bottom.
    // 0,0 is top left and width -1 , height -1 is bottom right.
    std::vector<Color> data;
    const int width;
    const int height;

    Image(int width, int height):
        data(width * height, WHITE),
        width(width),
        height(height) {}

    void set(int x, int y, Color color) {
        //TODO: Asserts for 0 < x < width and 0 < y < height
        int index = x + y * width;
        data[index] = color;
    }
};


int main() {
    // Image with red diagonal
    Image image = Image(100, 100);
    for(int i = 0; i < 100; i++) {
        image.set(i, i, RED);
    }

    // Save image.
    uint8_t image_raw[3 * 100 * 100];
    for(int i = 0; i < 100 * 100; i ++) {
        image_raw[3 * i] = image.data[i].r;
        image_raw[3 * i + 1] = image.data[i].g;
        image_raw[3 * i + 2] = image.data[i].b;
    }
    stbi_write_tga("test.tga", 100, 100, 3, image_raw);
}
