#include <iostream>
#include <fstream>
#include <vector>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "lib/stb_image_write.h"

struct Color {
    uint8_t r;
    uint8_t g;
    uint8_t b;
};

constexpr Color WHITE = { 255, 255, 255 };
constexpr Color RED = { 255, 0, 0 };
constexpr Color GREEN = { 0, 255, 0 };

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

struct Point {
    const float x;
    const float y;
    const float z;

    Point(float x, float y, float z): x(x), y(y), z(z) {}
};

const Point operator-(const Point& l, const Point& r) {
    return Point(l.x - r.x, l.y - r.y, l.z - r.z);
}

const Point operator/(const Point& l, const float divisor) {
    return Point(l.x / divisor, l.y / divisor, l.z / divisor);
}

float operator*(const Point& p, const Point& q) {
    return p.x * q.x + p.y * q.y + p.z * q.z;
}

class Sphere {
public:
    const float diameter;
    const Point center;
    const Color color;

    Sphere(float diameter, Point center, Color color):
        diameter(diameter),
        center(center),
        color(color) {}
};

int main(int argc, char* args[]) {
    if(argc != 5) {
        std::cout << "Usage: renderer d x y z" << std::endl;
    }

    const Point eye = Point(0, 0, -20);
    std::cout << "eye = " << eye.x << ", " << eye.y << ", " << eye.z << std::endl;
    const Sphere ball = Sphere(
        std::atof(args[1]),
        Point(std::atof(args[2]), std::atof(args[3]), std::atof(args[4])),
        GREEN);

    // Ray casting.
    // The screen's top left is at [-50, 50] and has a width and height of 100.
    // It lies on the x-y plane.
    Image image(100, 100);

    //Draw x axis as horizon.
    for(int x = 0; x < 100; x++) {
        image(x, 50) = RED;
    }

    for(int x = 0; x < 100; x++) {
        for(int y = 0; y < 100; y++) {

            // Fill all pixels with the color of the sphere if a ray through eye
            // and pixel cross the sphere.

            // Point of screen pixel in space
            const auto p = Point(-50 + x, 50 - y, 0);

            Point p_eye = p - eye;
            Point l = p_eye / sqrt(p_eye * p_eye);
            Point o = eye;
            float r = ball.diameter;
            Point c = ball.center;

            auto loc = l * (o - c);
            auto cross = loc * loc - (o - c) * (o - c) + r * r;

            if (!(cross < 0)) {
                image(x, y) =  ball.color;
            }
        }
    }

    // Save image.
    image.save("test.tga");
    return 0;
}
