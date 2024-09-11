#include <vector>
#include <cmath>
#include "model.h"
#include "geometry.h"
#include "tgaimage.h"

const TGAColor white = TGAColor(255, 255, 255, 255);
const TGAColor red = TGAColor(255, 0, 0, 255);
const TGAColor green = TGAColor(0, 255, 0, 255);
const int height = 200, width = 200;

typedef std::vector<Vec2i> Triangle;

std::pair<Vec2i, Vec2i> get_bbox(const Triangle &pts, TGAImage &image)
{
    Vec2i bbox_min = Vec2i(image.get_width() - 1, image.get_height() - 1);
    Vec2i bbox_max = Vec2i(0, 0);
    for (int i = 0; i < pts.size(); i++)
    {
        bbox_min.x = std::max(0, std::min(bbox_min.x, pts[i].x));
        bbox_min.y = std::max(0, std::min(bbox_min.y, pts[i].y));

        bbox_max.x = std::min(image.get_width() - 1, std::max(bbox_max.x, pts[i].x));
        bbox_max.y = std::min(image.get_height() - 1, std::max(bbox_max.y, pts[i].y));
    }

    return {bbox_min, bbox_max};
}

Vec3f barycentric(const Triangle &vertices, Vec2i query_pt)
{
    Vec2i AB = vertices[1] - vertices[0];
    Vec2i AC = vertices[2] - vertices[0];
    Vec2i PA = vertices[0] - query_pt;

    Vec3f Eq_x = Vec3f(float(AB.x), float(AC.x), float(PA.x));
    Vec3f Eq_y = Vec3f(float(AB.y), float(AC.y), float(PA.y));

    Vec3f b = Eq_x ^ Eq_y;

    if (std::abs(b.z) < 1)
    {
        return Vec3f(-1, 1, 1);
    }

    float coeffA = 1.f - (b.x + b.y) / b.z;
    float coeffB = b.y / b.z;
    float coeffC = b.x / b.z;

    return Vec3f(coeffA, coeffB, coeffC);
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
    float derr = std::abs(dy / float(dx));
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
        if (err > .5)
        {
            y += (p1.y > p0.y ? 1 : -1);
            err -= 1;
        }
    }
    return;
}

void triangle(Triangle &t, TGAImage &image, TGAColor color)
{
    std::pair<Vec2i, Vec2i> bbox = get_bbox(t, image);
    Vec2i bbox_min = bbox.first;
    Vec2i bbox_max = bbox.second;

    for (int x = bbox_min.x; x <= bbox_max.x; x++)
    {
        for (int y = bbox_min.y; y <= bbox_max.y; y++)
        {
            Vec2i q = Vec2i(x, y);
            Vec3f b_coords = barycentric(t, q);
            if (b_coords.x < 0 || b_coords.y < 0 || b_coords.z < 0)
            {
                continue;
            }
            else
            {
                image.set(x, y, color);
            }
        }
    }
    // if (t0.y == t1.y && t0.y == t2.y)
    //     return;

    // // bubble sort -> lower to upper
    // if (t0.y > t1.y)
    //     std::swap(t0, t1);
    // if (t0.y > t2.y)
    //     std::swap(t0, t2);
    // if (t1.y > t2.y)
    //     std::swap(t1, t2);

    // int full_height = t2.y - t0.y;

    // for (int current_y = t0.y; current_y <= t2.y; current_y++)
    // {
    //     bool lower = current_y > t1.y - t0.y || t1.y == t0.y;
    //     int segment_height = lower ? t1.y - t0.y + 1 : t2.y - t1.y + 1;

    //     float d1 = (current_y - t0.y) / float(full_height);
    //     float d2 = (current_y - (lower ? t0.y : t1.y)) / float(segment_height);

    //     Vec2i s1 = t0 + (t2 - t0) * d1;
    //     Vec2i s2 = lower ? t0 + (t1 - t0) * d2 : t1 + (t2 - t1) * d2;

    //     if (s1.x > s2.x)
    //     {
    //         std::swap(s1, s2);
    //     }

    //     for (int current_x = s1.x; current_x <= s2.x; current_x++)
    //     {
    //         image.set(current_x, current_y, color);
    //     }
    // }

    // return;
}

int main(int argc, char **argv)
{
    TGAImage image(width, height, TGAImage::RGB);

    Triangle t0 = {Vec2i(10, 70), Vec2i(50, 160), Vec2i(70, 80)};
    Triangle t1 = {Vec2i(180, 50), Vec2i(150, 1), Vec2i(70, 180)};
    Triangle t2 = {Vec2i(180, 150), Vec2i(120, 160), Vec2i(130, 180)};

    triangle(t0, image, red);
    triangle(t1, image, white);
    triangle(t2, image, green);

    image.flip_vertically(); // i want to have the origin at the left bottom corner of the image
    image.write_tga_file("output.tga");
    return 0;
}