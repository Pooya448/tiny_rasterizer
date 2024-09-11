#include <vector>
#include <cmath>
#include <algorithm>
#include "model.h"
#include "geometry.h"
#include "tgaimage.h"

const TGAColor white = TGAColor(255, 255, 255, 255);
const TGAColor red = TGAColor(255, 0, 0, 255);
const TGAColor green = TGAColor(0, 255, 0, 255);
const int height = 800, width = 800;
Model *model = NULL;

std::pair<Vec2f, Vec2f> get_bbox(Vec3f *pts, TGAImage &image)
{
    Vec2f bbox_min = Vec2f(std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
    Vec2f bbox_max = Vec2f(-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max());

    for (int i = 0; i < 3; i++)
    {
        bbox_min.x = std::max(0.f, std::min(bbox_min.x, pts[i].x));
        bbox_min.y = std::max(0.f, std::min(bbox_min.y, pts[i].y));

        bbox_max.x = std::min(float(image.get_width() - 1), std::max(bbox_max.x, pts[i].x));
        bbox_max.y = std::min(float(image.get_height() - 1), std::max(bbox_max.y, pts[i].y));
    }

    return {bbox_min, bbox_max};
}

Vec3f barycentric(Vec3f *vertices, Vec3f query_pt)
{
    Vec3f AB = vertices[2] - vertices[0];
    Vec3f AC = vertices[1] - vertices[0];
    Vec3f PA = vertices[0] - query_pt;

    Vec3f Eq_x(float(AB.x), float(AC.x), float(PA.x));
    Vec3f Eq_y(float(AB.y), float(AC.y), float(PA.y));

    Vec3f b = Eq_x ^ Eq_y;

    if (std::abs(b.z) < 1e-2)
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

void triangle(Vec3f *t, TGAImage &image, TGAColor color, float **zbuff)
{
    std::pair<Vec2f, Vec2f> bbox = get_bbox(t, image);
    Vec2f bbox_min = bbox.first;
    Vec2f bbox_max = bbox.second;

    for (int x = bbox_min.x; x <= bbox_max.x; x++)
    {
        for (int y = bbox_min.y; y <= bbox_max.y; y++)
        {
            Vec3f q(x, y, 0);
            Vec3f b_coords = barycentric(t, q);
            if (b_coords.x < 0 || b_coords.y < 0 || b_coords.z < 0)
            {
                continue;
            }
            for (int i = 0; i < 3; i++)
            {
                q.z += t[i].z * b_coords[i];
            }
            if (zbuff[x][y] < q.z)
            {
                zbuff[x][y] = q.z;
                image.set(x, y, color);
            }
        }
    }
}

float get_illumination(Vec3f *w, Vec3f light_dir)
{
    Vec3f v1 = w[2] - w[0];
    Vec3f v2 = w[1] - w[0];
    Vec3f normal = (v1 ^ v2).normalize();
    float brightness = normal * light_dir;
    return brightness;
}

Vec3f world2screen(Vec3f v)
{
    return Vec3f((v.x * 0.5 + 0.5) * width, (v.y * 0.5 + 0.5) * height, v.z);
}

int main(int argc, char **argv)
{
    TGAImage image(width, height, TGAImage::RGB);

    model = new Model("obj/african_head.obj");

    float **zbuff = new float *[width];
    for (int i = 0; i < width; i++)
    {
        zbuff[i] = new float[height];
        std::fill_n(zbuff[i], height, -std::numeric_limits<float>::max());
    }

    Vec3f light_dir(0, 0, -1);

    for (int i = 0; i < model->nfaces(); i++)
    {
        std::vector<int> f = model->face(i);

        Vec3f t[3];
        Vec3f w[3];

        for (int j = 0; j < 3; j++)
        {
            Vec3f vertex = model->vert(f[j]);
            w[j] = vertex;
            t[j] = world2screen(vertex);
        }

        float brightness = get_illumination(w, light_dir);
        TGAColor c = TGAColor(brightness * 255, brightness * 255, brightness * 255, 255);
        triangle(t, image, c, zbuff);
    }

    image.flip_vertically(); // i want to have the origin at the left bottom corner of the image
    image.write_tga_file("output.tga");
    return 0;
}