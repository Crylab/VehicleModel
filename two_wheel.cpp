//
// Created by Руслан Шаяхметов on 20/12/22.
//

#include "two_wheel.h"

Vector::Vector() {
    x = 0.0;
    y = 0.0;
    z = 0.0;
}

void Vector::Initialize(double x_in, double y_in, double z_in) {
    x = x_in;
    y = y_in;
    z = z_in;
}

Vector Vector::operator+(const Vector &obj) const {
    Vector result;
    result.Initialize(x+obj.x, y+obj.y, z+obj.z);
    return result;
}

double Vector::norm() const {
    return sqrt(x*x+y*y+z*z);
}

Vector Vector::operator*(double obj) const {
    Vector result;
    result.Initialize(x*obj, y*obj, z*obj);
    return result;
}

Vector& Vector::operator=(Vector const &obj) {
    // Guard self assignment
    if (this == &obj)
        return *this; // delete[]/size=0 would also be ok
    x = obj.x;
    y = obj.y;
    z = obj.z;
    return *this;
}

Vector Vector::operator - (Vector const &obj) const {
    Vector result;
    result.Initialize(x-obj.x, y-obj.y, z-obj.z);
    return result;
}

double Vector::distance(Vector to) {
    Vector diff;
    diff = *this - to;
    return diff.norm();
}

Vector Vector::operator / (double obj) const {
    Vector result;
    result.Initialize(x/obj, y/obj, z/obj);
    return result;
}

std::vector <Vector> Vector::gen_path(Vector to, double delta) {
    double steps = distance(to)/delta;
    Vector dv;
    dv = (*this-to)/steps;
    Vector temp = *this;
    std::vector<Vector> result;
    result.push_back(temp);
    for(unsigned int i = 0; i<steps; i++) {
        temp = temp - dv;
        result.push_back(temp);
    }
    result.back() = to;
    return result;
}

std::string Vector::str() {
    std::string result = std::to_string(x)
                         + " " + std::to_string(y)
                         + " " + std::to_string(z);
    return result;
}


Vector Vector::operator*(Vector obj) const {
    Vector result;
    double res_x = y*obj.z-z*obj.y;
    double res_y = z*obj.x-x*obj.z;
    double res_z = x*obj.y-y*obj.x;
    result.Initialize(res_x,res_y,res_z);
    return result;
}

void Vector::normalize() {
    double len = norm();
    x /= len;
    y /= len;
    z /= len;
}


bool Vector::operator==(Vector obj) {
    const double tol = 5e-3;
    if(abs(x-obj.x) > tol)
        return false;
    if(abs(y-obj.y) > tol)
        return false;
    if(abs(z-obj.z) > tol)
        return false;
    return true;
}

bool Vector::operator!=(Vector obj) {
    return (!(*this == obj));
}

Vector::coordinates Vector::components() {
    coordinates result{};
    result.x = x;
    result.y = y;
    result.z = z;
    return result;
}

void Vector::planar_rotate(double angle) {
    if(z != 0.0) {
        std::cout << "Error: planar rotation is not possible for non-planar vectors" << std::endl;
        return;
    }
    double x_new = x*cos(angle) - y*sin(angle);
    double y_new = x*sin(angle) + y*cos(angle);
    x = x_new;
    y = y_new;
}


TwoWheel::TwoWheel(double base_length_in, double base_width_in) {
    base_length = base_length_in;
    base_width = base_width_in;
    velocity.Initialize(0.0,0.0,0.0);
    position.Initialize(0.0,0.0,0.0);
    theta = 0.0;
}

VehicleState TwoWheel::update(double steer, double acceleration_scalar, double dt) {
    double theta_dot = velocity.norm() * tan(steer) / base_length;
    theta += theta_dot * dt;
    Vector acceleration;
    acceleration.Initialize(acceleration_scalar*cos(theta), acceleration_scalar*sin(theta), 0.0);
    velocity = velocity + acceleration * dt;
    velocity.planar_rotate(theta);
    position = position + velocity * dt;
    VehicleState result;
    result.position = position;
    result.theta = theta;
    return result;
}
