#include <vector>
#include <cmath>
#include <algorithm>
#include "model.h"
#include "geometry.h"
#include "tgaimage.h"

const int height = 800, width = 800, depth = 255;
Vec3f light_dir(0, 0, -1);
Vec3f camera(0, 0, 3);
Model *model = NULL;
float **zbuff = NULL;

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

void triangle(Vec3f *t, TGAImage &image, Vec2f *uvs, float **zbuff, Model *model, float intensity)
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

                Vec2f uv_interp = (uvs[0] * b_coords.x) + (uvs[1] * b_coords.y) + (uvs[2] * b_coords.z);
                Vec2i uv_q = Vec2i(int(uv_interp.x), int(uv_interp.y));
                TGAColor texture = model->diffuse(uv_q);
                TGAColor final_color = TGAColor(texture.r * intensity, texture.g * intensity, texture.b * intensity, texture.a * intensity);
                image.set(x, y, final_color);
            }
        }
    }
}

float light_intensity(Vec3f *w)
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

void initialize_zbuffer()
{
    zbuff = new float *[width];
    for (int i = 0; i < width; i++)
    {
        zbuff[i] = new float[height];
        std::fill_n(zbuff[i], height, -std::numeric_limits<float>::max());
    }
    return;
}

Vec3f m2v(Matrix m)
{
    Vec3f v(m[0][0] / m[3][0], m[1][0] / m[3][0], m[2][0] / m[3][0]);
    return v;
}
Matrix v2m(Vec3f v)
{
    Matrix m(4, 1);
    m[0][0] = v.x;
    m[1][0] = v.y;
    m[2][0] = v.z;
    m[3][0] = 1.f;
    return m;
}
Matrix viewport_transform(int x, int y, int w, int h)
{
    Matrix m = Matrix::identity(4);
    m[0][3] = x + (w / 2.f); // Translate X to the center of the viewport
    m[1][3] = y + h / 2.f;   // Translate Y to the center of the viewport
    m[2][3] = depth / 2.f;   // Translate Z to the middle of the depth range

    m[0][0] = w / 2.f;     // Scale X to fit the viewport width
    m[1][1] = h / 2.f;     // Scale Y to fit the viewport height
    m[2][2] = depth / 2.f; // Scale Z to fit the depth range
    return m;
}
Matrix perspective_projection()
{
    Matrix m = Matrix::identity(4);
    m[3][2] = -1.f / camera.z;
    return m;
}
Matrix orthographic_projection()
{
    Matrix m = Matrix::identity(4);
    return m;
}
int main(int argc, char **argv)
{
    TGAImage image(width, height, TGAImage::RGB);

    model = new Model("obj/african_head.obj");

    initialize_zbuffer();
    Matrix projection = perspective_projection();
    // Matrix projection = orthographic_projection();
    Matrix viewport = viewport_transform(width / 8, height / 8, width * 3 / 4, height * 3 / 4);
    Matrix scene_transform = viewport * projection;

    for (int i = 0; i < model->nfaces(); i++)
    {
        std::vector<int> f = model->face(i);

        Vec3f t[3];
        Vec3f w[3];
        Vec2f uvs[3];

        for (int j = 0; j < 3; j++)
        {
            Vec3f vertex = model->vert(f[j]);
            w[j] = vertex;
            t[j] = m2v(scene_transform * v2m(vertex));
            // t[j] = world2screen(vertex);
            uvs[j] = model->uv(i, j);
        }

        float intensity = light_intensity(w);
        if (intensity > 0)
        {
            triangle(t, image, uvs, zbuff, model, intensity);
        }
    }

    image.flip_vertically(); // i want to have the origin at the left bottom corner of the image
    image.write_tga_file("output.tga");
    return 0;
}