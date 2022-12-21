//
// Created by Ruslan Shaiakhmetov on 20/12/22.
//

#ifndef VEHICLEMODEL_TWO_WHEEL_H
#define VEHICLEMODEL_TWO_WHEEL_H

#include <cmath>
#include <iostream>
#include <vector>
#include <string>
//The class that represents all vectors quantities
//such as: position, velocity, acceleration, force
class Vector {
public:
    Vector();
    void Initialize(double x_in, double y_in, double z_in);
    Vector operator + (Vector const &obj) const;
    Vector operator - (Vector const &obj) const;
    Vector operator * (double obj) const;
    Vector operator * (Vector obj) const;
    Vector operator / (double obj) const;
    Vector& operator = (Vector const &obj);
    void normalize();
    double norm() const;
    double distance(Vector to);
    bool operator == (Vector obj);
    bool operator != (Vector obj);
    std::vector <Vector> gen_path(Vector to, double delta);
    std::string str();
    struct coordinates {
        double x, y, z;
    };
    void planar_rotate(double angle);
    struct coordinates components();
protected:
    double x,y,z;
};

struct VehicleState {
    Vector position;
    double theta;
};

//The class that represents the vehicle
class TwoWheel{
private:
    double base_length;
    double base_width;
    double theta;
    Vector position;
    Vector velocity;
public:
    TwoWheel(double base_length_in, double base_width_in);
    VehicleState update(double steer, double acceleration_scalar, double dt);
};

#endif //VEHICLEMODEL_TWO_WHEEL_H
