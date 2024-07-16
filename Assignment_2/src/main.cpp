// C++ include
#include <iostream>
#include <string>
#include <vector>

// Utilities for the Assignment
#include "utils.h"

// Image writing library
#define STB_IMAGE_WRITE_IMPLEMENTATION // Do not include this line twice in your project!
#include "stb_image_write.h"

// Shortcut to avoid Eigen:: everywhere, DO NOT USE IN .h
using namespace Eigen;

float triIntersect(Vector3d ro, Vector3d rd, Vector3d v0, Vector3d v1, Vector3d v2)
{
    const Vector3d v1v0 = v1;  // Vector from vertex v0 to vertex v1
    const Vector3d v2v0 = v2;  // Vector from vertex v0 to vertex v2
    const Vector3d rov0 = ro - v0;  // Vector from ray origin to vertex v0
    const Vector3d  n = v1v0.cross(v2v0);  // Normal vector of the triangle
    const Vector3d  q = rov0.cross(rd);    // Perpendicular vector to the ray direction
    float d = 1.0/rd.dot(n);     // Reciprocal of the dot product of the ray direction and the normal
    float u = d*-q.dot(v2v0);    // Barycentric coordinate u
    float v = d*q.dot(v1v0);    // Barycentric coordinate v
    float t = d*-n.dot(rov0);   // Distance from the ray origin to the intersection point
    if( u<0.0 || v<0.0 || (u+v)>1.0 ) t = -1.0;  // Check if the intersection point is outside the triangle
    return t;  // Return the distance 
}

Vector2d sphIntersect(Vector3d ro, Vector3d rd, Vector3d ce, float ra )
{
   
    Vector3d oc = ro - ce; // Calculate the vector from the ray origin to the center of the sphere
    float b = oc.dot(rd); // Compute the projection of the oc vector onto the ray direction (rd)
    float c = oc.dot(oc) - ra * ra; // Compute the squared length of the oc vector minus the square of the sphere's radius
    float h = b * b - c; // Calculate the discriminant (h), which determines if the ray intersects the sphere
    if (h < 0.0) return Vector2d(-1.0, 0); // If the discriminant is less than zero, there is no intersection
    h = sqrt(h);// Calculate the square root of the discriminant (h)
    // Return the two intersection points along the ray direction
    // (-b - h) and (-b + h) represent the distances from the ray origin to the intersection points
    return Vector2d(-b - h, -b + h);

}

void raytrace_sphere()
{
    std::cout << "Simple ray tracer, one sphere with orthographic projection" << std::endl;

    const std::string filename("sphere_orthographic.png");
    MatrixXd C = MatrixXd::Zero(800, 800); // Store the color
    MatrixXd A = MatrixXd::Zero(800, 800); // Store the alpha mask

    const Vector3d camera_origin(0, 0, 3);
    const Vector3d camera_view_direction(0, 0, -1);

    // The camera is orthographic, pointing in the direction -z and covering the
    // unit square (-1,1) in x and y
    const Vector3d image_origin(-1, 1, 1);
    const Vector3d x_displacement(2.0 / C.cols(), 0, 0);
    const Vector3d y_displacement(0, -2.0 / C.rows(), 0);

    // Single light source
    const Vector3d light_position(-1, 1, 1);

    for (unsigned i = 0; i < C.cols(); ++i)
    {
        for (unsigned j = 0; j < C.rows(); ++j)
        {
            const Vector3d pixel_center = image_origin + double(i) * x_displacement + double(j) * y_displacement;

            // Prepare the ray
            const Vector3d ray_origin = pixel_center;
            const Vector3d ray_direction = camera_view_direction;

            // Intersect with the sphere
            // NOTE: this is a special case of a sphere centered in the origin and for orthographic rays aligned with the z axis

            // Changed to fit for the general case
            const Vector3d sphere_center(0, 0, 0);
            Vector2d ray_on_xy(ray_origin(0), ray_origin(1));
            const double sphere_radius = 0.9;

            Vector2d intersections = sphIntersect(ray_origin, camera_view_direction, sphere_center ,sphere_radius );

            if (intersections[0] != -1)
            {
                // The ray hit the sphere, compute the exact intersection point
                Vector3d ray_intersection(
                    ray_on_xy(0), ray_on_xy(1),
                    sqrt(sphere_radius * sphere_radius - ray_on_xy.squaredNorm()));

                // Compute normal at the intersection point
                Vector3d ray_normal = ray_intersection.normalized();

                // Simple diffuse model
                C(i, j) = (light_position - ray_intersection).normalized().transpose() * ray_normal;

                // Clamp to zero
                C(i, j) = std::max(C(i, j), 0.);

                // Disable the alpha mask for this pixel
                A(i, j) = 1;
            }
        }
    }

    // Save to png
    write_matrix_to_png(C, C, C, A, filename);
}


void raytrace_parallelogram()
{
    std::cout << "Simple ray tracer, one parallelogram with orthographic projection" << std::endl;

    const std::string filename("plane_orthographic.png");
    MatrixXd C = MatrixXd::Zero(800, 800); // Store the color
    MatrixXd A = MatrixXd::Zero(800, 800); // Store the alpha mask

    const Vector3d camera_origin(0, 0, 3);
    const Vector3d camera_view_direction(0, 0, -1);

    // The camera is orthographic, pointing in the direction -z and covering the unit square (-1,1) in x and y
    const Vector3d image_origin(-1, 1, 1);
    const Vector3d x_displacement(2.0 / C.cols(), 0, 0);
    const Vector3d y_displacement(0, -2.0 / C.rows(), 0);

    // Parameters of the parallelogram (position of the lower-left corner + two sides)
    // TODO: Parameters of the parallelogram 
    const Vector3d pgram_origin(-0.5, -0.5, 0);
    const Vector3d pgram_u(0, 0.7, -10);
    const Vector3d pgram_v(1, 0.4, 0);

    const Vector3d pgram_uv = pgram_origin + pgram_u + pgram_v;
    const Vector3d pgram_vR = pgram_origin + pgram_v - pgram_uv;
    const Vector3d pgram_uR = pgram_origin + pgram_u - pgram_uv;

    // Vectors
    const Vector3d vec_ov = pgram_origin + pgram_v;
    const Vector3d vec_ou = pgram_origin + pgram_u;

    const Vector3d paralellogram_norm = vec_ou.cross(vec_ov);

    const double d_point = -paralellogram_norm.dot(pgram_v);
  

    // Single light source
    const Vector3d light_position(-1, 1, 1);

    for (unsigned i = 0; i < C.cols(); ++i)
    {
        for (unsigned j = 0; j < C.rows(); ++j)
        {
            const Vector3d pixel_center = image_origin + double(i) * x_displacement + double(j) * y_displacement;

            // Prepare the ray
            const Vector3d ray_origin = pixel_center;
            const Vector3d ray_direction = camera_view_direction;

            const float in = triIntersect(ray_origin, ray_direction, pgram_origin, pgram_v, pgram_u);
            const float in_2 = triIntersect(ray_origin, ray_direction, pgram_uv, pgram_vR, pgram_uR);

            // TODO: Check if the ray intersects with the parallelogram
            if (in != -1 || in_2 != -1)
            {
                // TODO: The ray hit the parallelogram, compute the exact intersection
                // point
                float mag = 0;

                if(in == -1) {
                    mag = in_2;
                } else {
                    mag = in;
                }

                Vector3d ray_intersection = ray_origin + mag*ray_direction;

                // TODO: Compute normal at the intersection point
                Vector3d ray_normal = -1*pgram_u.cross(pgram_v).normalized();

                // Simple diffuse model
                C(i, j) = (light_position - ray_intersection).normalized().transpose() * ray_normal;

                // Clamp to zero
                C(i, j) = std::max(C(i, j), 0.);

                // Disable the alpha mask for this pixel
                A(i, j) = 1;
            }
        }
    }

    // Save to png
    write_matrix_to_png(C, C, C, A, filename);
}

void raytrace_perspective()
{
    std::cout << "Simple ray tracer, one parallelogram with perspective projection" << std::endl;

    const std::string filename("plane_perspective.png");
    MatrixXd C = MatrixXd::Zero(800, 800); // Store the color
    MatrixXd A = MatrixXd::Zero(800, 800); // Store the alpha mask

    const Vector3d camera_origin(0, 0, 3);
    const Vector3d camera_view_direction(0, 0, -1);

    // The camera is perspective, pointing in the direction -z and covering the unit square (-1,1) in x and y
    const Vector3d image_origin(-1, 1, 1);
    const Vector3d x_displacement(2.0 / C.cols(), 0, 0);
    const Vector3d y_displacement(0, -2.0 / C.rows(), 0);

    // TODO: Parameters of the parallelogram 
    const Vector3d pgram_origin(-0.5, -0.5, 0);
    const Vector3d pgram_u(0, 0.7, -10);
    const Vector3d pgram_v(1, 0.4, 0);

    // Vectors
    const Vector3d vec_ov = pgram_origin - pgram_v;
    const Vector3d vec_ou = pgram_origin - pgram_u;

    const Vector3d pgram_uv = pgram_origin + pgram_u + pgram_v;
    const Vector3d pgram_vR = pgram_origin + pgram_v - pgram_uv;
    const Vector3d pgram_uR = pgram_origin + pgram_u - pgram_uv;


    const Vector3d paralellogram_norm = vec_ov.cross(vec_ou);

    const double d_point = paralellogram_norm.dot(pgram_v);

    // Single light source
    const Vector3d light_position(-1, 1, 1);

    for (unsigned i = 0; i < C.cols(); ++i)
    {
        for (unsigned j = 0; j < C.rows(); ++j)
        {
            const Vector3d pixel_center = image_origin + double(i) * x_displacement + double(j) * y_displacement;

            // TODO: Prepare the ray (origin point and direction)
            const Vector3d ray_origin = pixel_center;
            const Vector3d ray_direction = camera_origin-pixel_center;

            //const double mag = ((paralellogram_norm.dot(ray_origin)) + d_point) / (paralellogram_norm.dot(ray_direction));
            //const Vector3d inters_point = ray_origin + mag*ray_direction;

            const float in = triIntersect(pixel_center, ray_direction, pgram_origin, pgram_v, pgram_u);
            const float in_2 = triIntersect(pixel_center, ray_direction, pgram_uv, pgram_vR, pgram_uR);


            // TODO: Check if the ray intersects with the parallelogram
            if (in != -1 || in_2 != -1)
            {

                // TODO: The ray hit the parallelogram, compute the exact intersection point

                float mag = 0;

                if(in != -1) {
                    mag = in;
                } else {
                    mag = in_2;
                }

                Vector3d ray_intersection = ray_origin + mag*ray_direction;

                // TODO: Compute normal at the intersection point
                Vector3d ray_normal = -1*pgram_u.cross(pgram_v).normalized();

                // Simple diffuse model
                C(i, j) = (light_position - ray_intersection).normalized().transpose() * ray_normal;

                // Clamp to zero
                C(i, j) = std::max(C(i, j), 0.);

                // Disable the alpha mask for this pixel
                A(i, j) = 1;
            }
        }
    }

    // Save to png
    write_matrix_to_png(C, C, C, A, filename);
}

void raytrace_shading()
{
    std::cout << "Simple ray tracer, one sphere with different shading" << std::endl;

    const std::string filename("shading.png");
    MatrixXd C = MatrixXd::Zero(800, 800); // Store the color
    MatrixXd R = MatrixXd::Zero(800, 800); // Store the color
    MatrixXd G = MatrixXd::Zero(800, 800); // Store the color
    MatrixXd B = MatrixXd::Zero(800, 800); // Store the color

    MatrixXd A = MatrixXd::Zero(800, 800); // Store the alpha mask

    const Vector3d camera_origin(0, 0, 3);
    const Vector3d camera_view_direction(0, 0, -1);

    // The camera is perspective, pointing in the direction -z and covering the unit square (-1,1) in x and y
    const Vector3d image_origin(-1, 1, 1);
    const Vector3d x_displacement(2.0 / A.cols(), 0, 0);
    const Vector3d y_displacement(0, -2.0 / A.rows(), 0);

    //Sphere setup
    const Vector3d sphere_center(0, 0, 0);
    const double sphere_radius = 0.9;

    //material params
    const Vector3d diffuse_color(1, 0, 1);
    const double specular_exponent = 100;
    const Vector3d specular_color(0., 0, 1);

    // Single light source
    const Vector3d light_position(-1, 1, 1);
    double ambient = 0.1;

    for (unsigned i = 0; i < C.cols(); ++i)
    {
        for (unsigned j = 0; j < C.rows(); ++j)
        {
            const Vector3d pixel_center = image_origin + double(i) * x_displacement + double(j) * y_displacement;

            // TODO: Prepare the ray (origin point and direction)
            const Vector3d ray_origin = pixel_center;
            const Vector3d ray_direction = camera_origin-pixel_center;
            Vector3d ray_normalized = ray_direction.normalized();
            // Intersect with the sphere
            // TODO: implement the generic ray sphere intersection

            Vector2d roots = sphIntersect(camera_origin, ray_normalized, sphere_center, sphere_radius);
            if (roots[0] != -1.0)
            {
                // TODO: The ray hit the sphere, compute the exact intersection point
                float mag = 0.0;
                if(roots[0] >= roots[1]){
                    mag = roots[0];
                } else {
                    mag = roots[1];
                }
                Vector3d ray_intersection = camera_origin + mag*ray_normalized;


                // TODO: Compute normal at the intersection point
                Vector3d ray_normal = ray_intersection.normalized();
                
                // TODO: Add shading parameter here
                const double diffuse = (light_position - ray_intersection).normalized().transpose() * ray_normal;

                // SPECULAR
                // Vector from intersection point to the camera (view direction)
                Vector3d view_dir = (camera_origin - ray_intersection).normalized();

                // Vector from intersection point to the light source (light direction)
                Vector3d light_dir = (light_position - ray_intersection).normalized();

                // Calculate the halfway vector correctly
                Vector3d h = (view_dir + light_dir).normalized();
                double reflect_vec = ray_normal.dot(h.normalized());

                // h in the formula     

                const double specular = powf(std::max(reflect_vec, 0.0), specular_exponent);

                // Simple diffuse model
                R(i, j) = ambient + diffuse_color[0]*diffuse + specular_color[0]*specular;
                G(i, j) = ambient + diffuse_color[1]*diffuse + specular_color[1]*specular;
                B(i, j) = ambient + diffuse_color[2]*diffuse + specular_color[2]*specular;

                // Clamp to zero
                C(i, j) = std::max(C(i, j), 0.0);

                // Disable the alpha mask for this pixel
                A(i, j) = 1;
            }
        }
    }

    // Save to png
    write_matrix_to_png(R, G, B, A, filename);
}


int main()
{
    raytrace_sphere();
    raytrace_parallelogram();
    raytrace_perspective();
    raytrace_shading();

    return 0;
}
