////////////////////////////////////////////////////////////////////////////////
#include <algorithm>
#include <complex>
#include <fstream>
#include <iostream>
#include <numeric>
#include <vector>

#include <Eigen/Dense>
// Shortcut to avoid everywhere, DO NOT USE IN .h
using namespace Eigen;
////////////////////////////////////////////////////////////////////////////////

const std::string root_path = DATA_DIR;

// Computes the determinant of the matrix whose columns are the vector u and v
double inline det(const Vector2d &u, const Vector2d &v)
{
    // Since all of our vectors are 2d, we can use the simple physics formula for computing
    // determinants. 
    double result = u[0]*v[1] - u[1]*v[0];

    return result;
}

// Return true iff [a,b] intersects [c,d]

// made inside is_inside function.
bool intersect_segment(const Vector2d &a, const Vector2d &b, const Vector2d &c, const Vector2d &d)
{
    // TODO
    return true;
}

////////////////////////////////////////////////////////////////////////////////

bool is_inside(const std::vector<Vector2d> &poly, const Vector2d &query)
{
    // 1. Compute bounding box and set coordinate of a point outside the polygon
    double max_x = 0.0;
    double max_y = 0.0;

    // compute max x and y values of the polygon
    for (int i = 0; i < poly.size(); i++) {
        if (poly[i][0] > max_x) {
            max_x = poly[i][0];
        }

        if (poly[i][1] > max_y) {
            max_y = poly[i][1];
        }
    }

    // set a value outside that max value
    Vector2d outside(max_x + 1, max_y + 1);

     // 2. Cast a ray from the query point to the 'outside' point, count number of intersections
    // our ray from our point to this max value is as follows:
    Vector2d ray(query[0]- outside[0], query[1]- outside[1]);

    // count intersection
    int num_inters = 0;
    for (int i = 0; i < poly.size(); i++) {
        // Create vectors
        Vector2d vec_1;
        Vector2d vec_2;

        // vertex's are connected two the adjacent ones, but poly[15] is defined and not = poly[0]
        // make sure we use poly[0] to connect to poly[14]
        int b = i + 1;
        if (b == 15){
            b = 0;
        }
        
        // create our rays
        vec_1 << query[0]- poly[i][0], query[1]- poly[i][1];
        vec_2 << query[0]- poly[b][0], query[1]- poly[b][1];

        // get orientation
        double det_1 = det(ray, vec_1);
        double det_2 = det(ray, vec_2);

        // check orientation. if -, opposite orientations
        double res_1 = det_1*det_2;

        // compute polygon boundary ray.
        Vector2d bound;
        bound << poly[i][0] - poly[b][0], poly[i][1] - poly[b][1];

        Vector2d vec_3;
        Vector2d vec_4;
        vec_3 << poly[i][0]- query[0], poly[i][1]- query[1];
        vec_4 << poly[i][0]- outside[0], poly[i][1]-outside[1];

        double det_3 = det(bound, vec_3);
        double det_4 = det(bound, vec_4);

        double res_2 = det_3*det_4;


        // if both checks of orientation are neg, intersection.
        if (res_1 <= 0 && res_2 <= 0) {
            num_inters = num_inters + 1;
        }

    }

    // return true if number of intersections is odd.

    bool result = (num_inters % 2) != 0;

    return result;
}

////////////////////////////////////////////////////////////////////////////////

std::vector<Vector2d> load_xyz(const std::string &filename)
{
    std::vector<Vector2d> points;
    std::ifstream in(filename);

    std::string line;

    std::getline(in, line); // skip the first line

    while (std::getline(in, line)) {
        std::string s;

        std::stringstream ss(line);
    
        // declaring vector to store the string after split
        std::vector<double> v;   
        Vector2d point;

        // for each line in the thing:
        while (getline(ss, s, ' ')) {
            // no point in storing the 0's, get rid of them

            // change the vector to a double
            double num_double = std::stod(s);

            // store it in our vector
            v.push_back(num_double);
            
            
        }
        // store the vector 
        point << v[0], v[1];
        points.push_back(point);
        
    }
    in.close();

    //std::cout << "point is: " << points[0] << std::endl;

    return points;
}

void save_xyz(const std::string &filename, const std::vector<Vector2d> &points)
{
    std::ofstream out(filename);

    for (int i = 0; i < points.size(); i++) {
        out << points[i][0] << " " << points[i][1] << " " << 0 << "\n";
    }

    out.close();
}

std::vector<Vector2d> load_obj(const std::string &filename)
{
    std::ifstream in(filename);
    std::vector<Vector2d> points;
    std::vector<Vector2d> poly;
    char key;
    while (in >> key)
    {
        if (key == 'v')
        {
            double x, y, z;
            in >> x >> y >> z;
            points.push_back(Vector2d(x, y));
        }
        else if (key == 'f')
        {
            std::string line;
            std::getline(in, line);
            std::istringstream ss(line);
            int id;
            while (ss >> id)
            {
                poly.push_back(points[id - 1]);
            }
        }
    }
    return poly;
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char *argv[])
{
    const std::string points_path = root_path + "/points.xyz";
    const std::string poly_path = root_path + "/polygon.obj";

    std::vector<Vector2d> points = load_xyz(points_path);
    // std::cout << "size: " << points.size() << std::endl;

    ////////////////////////////////////////////////////////////////////////////////
    //Point in polygon
    std::vector<Vector2d> poly = load_obj(poly_path);

    // check if object is here...

    std::vector<Vector2d> result;
    for (size_t i = 0; i < points.size(); ++i)
    {
        if (is_inside(poly, points[i]))
        {
            result.push_back(points[i]);
        }
    }
    std::cout << "resulting size: " << result.size() << std::endl;
    save_xyz("output.xyz", result);

    return 0;
}
