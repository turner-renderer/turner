#include <iostream>
#include <fstream>
#include <vector>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

struct Color {
    uint8_t r;
    uint8_t g;
    uint8_t b;
};

constexpr Color WHITE = { 255, 255, 255 };
constexpr Color RED = { 255, 0, 0 };

class Image {
public:
    const size_t width;
    const size_t height;

    Image(size_t width, size_t height):
        width(width),
        height(height),
        data_(width * height, WHITE)
    {}

    Color& operator()(size_t x, size_t y) {
        return data_[x + y * width];
    }

    const Color& operator()(size_t x, size_t y) const {
        return data_[x + y * width];
    }

    int save(const std::string& filename) {
        return stbi_write_tga(filename.c_str(), width, height, 3, &data_[0]);
    }

private:
    // The pixels are stored left-to-right and top-to-bottom.
    // 0,0 is top left and width -1 , height -1 is bottom right.
    std::vector<Color> data_;
};


int main() {
    // Image with red diagonal
    Image image(100, 100);
    for(int i = 0; i < 100; i++) {
        image(i, i) = RED;
    }

    // Save image.
    image.save("test.tga");
    return 0;
}
