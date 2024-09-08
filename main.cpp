#include <vector>
#include <cmath>
#include "model.h"
#include "geometry.h"
#include "tgaimage.h"

const TGAColor white = TGAColor(255, 255, 255, 255);
const TGAColor red = TGAColor(255, 0, 0, 255);
const int height = 800, width = 800;

Model *model = NULL;

void line(int x0, int y0, int x1, int y1, TGAImage &image, TGAColor color)
{
    bool steep = false;
    if (std::abs(y0 - y1) > std::abs(x0 - x1))
    {
        steep = true;
        std::swap(x0, y0);
        std::swap(x1, y1);
    }

    if (x0 > x1)
    {
        std::swap(x0, x1);
        std::swap(y0, y1);
    }

    int dx = x1 - x0;
    int dy = y1 - y0;
    float derr = std::abs(dy) * 2;
    float err = 0;
    int y = y0;

    for (int x = x0; x <= x1; x++)
    {
        if (steep)
        {
            image.set(y, x, color);
        }
        else
        {
            image.set(x, y, color);
        }
        err += derr;
        if (err > dx)
        {
            y += (y1 > y0 ? 1 : -1);
            err -= dx * 2;
        }
    }
}

void line(Vec2i p0, Vec2i p1, TGAImage &image, TGAColor color)
{
    bool steep = false;
    if (std::abs(p0.y - p1.y) > std::abs(p0.x - p1.x))
    {
        steep = true;
        std::swap(p0.x, p0.y);
        std::swap(p1.x, p1.y);
    }

    if (p0.x > p1.x)
    {
        std::swap(p0.x, p1.x);
        std::swap(p0.y, p1.y);
    }

    int dx = p1.x - p0.x;
    int dy = p1.y - p0.y;
    float derr = std::abs(dy) * 2;
    float err = 0;
    int y = p0.y;

    for (int x = p0.x; x <= p1.x; x++)
    {
        if (steep)
        {
            image.set(y, x, color);
        }
        else
        {
            image.set(x, y, color);
        }
        err += derr;
        if (err > dx)
        {
            y += (p1.y > p0.y ? 1 : -1);
            err -= dx * 2;
        }
    }
}

void triangle(Vec2i t0, Vec2i t1, Vec2i t2, TGAImage &image, TGAColor color)
{
    line(t0, t1, image, color);
    line(t1, t2, image, color);
    line(t2, t0, image, color);
    return;
}

int main(int argc, char **argv)
{
    model = new Model("obj/african_head.obj");

    TGAImage image(width, height, TGAImage::RGB);

    for (int i = 0; i < model->nfaces(); i++)
    {
        std::vector<int> f = model->face(i);
        for (int j = 0; j < 3; j++)
        {
            Vec3f v0 = model->vert(f[j]);
            Vec3f v1 = model->vert(f[(j + 1) % 3]);

            int x0 = (v0.x + 1) * 0.5 * width;
            int y0 = (v0.y + 1) * 0.5 * height;

            int x1 = (v1.x + 1) * 0.5 * width;
            int y1 = (v1.y + 1) * 0.5 * height;

            line(x0, y0, x1, y1, image, white);
        }
    }

    image.flip_vertically(); // i want to have the origin at the left bottom corner of the image
    image.write_tga_file("output.tga");
    return 0;
}