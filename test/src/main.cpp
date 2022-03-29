#include <iostream>
#include <cmath>
#include <vector>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Eigen>

constexpr double MY_PI = 3.1415926;

inline double deg2rad(double deg) {return deg * MY_PI / 180.0f;}

Eigen::Matrix4f get_model_matrix_formula(float rotation_angle)
{
    // TODO: Rodrigues' rotation formula
    
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    double rad = deg2rad(rotation_angle);
    std::cout << "rad:\n" << rad << std::endl;
    Eigen::Vector3f z = Eigen::Vector3f(0, 0, 1);
    Eigen::Matrix3f N;
    N << 0,-1, 0,
         1, 0, 0,
         0, 0, 0;
    Eigen::Matrix3f rot_model = Eigen::Matrix3f::Identity();;
    rot_model = cos(rad) * rot_model + 
                     (1-cos(rad)) * z * z.transpose() + 
                     sin(rad) * N;
    std::cout << "rot_model\n" << rot_model << std::endl;
    model.block<3, 3>(0, 0) = rot_model;
    std::cout << "model\n" << std::endl;
    return model;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    // Done
    float rotation_rad = acos(-1) * (rotation_angle) / 180.0f;
    std::cout << "rotation_rad:\n" << rotation_rad << std::endl;
    // std::clog << "rotation_rad=" << rotation_rad << std::endl;
    Eigen::Matrix3f rotation_model = Eigen::Matrix3f::Identity();
    Eigen::Vector3f z_axis = Eigen::Vector3f(0, 0, 1);
    Eigen::Matrix3f prod_z_axis;
    prod_z_axis << 0, -1, 0, 
                   1, 0, 0, 
                   0, 0, 0;
    rotation_model = cos(rotation_rad) * rotation_model + 
                     (1-cos(rotation_rad)) * z_axis * z_axis.transpose() + 
                     sin(rotation_rad) * prod_z_axis;
    std::cout << "rot_model\n" << rotation_model << std::endl;
    model.block<3, 3>(0, 0) = rotation_model;


    return model;
}

Eigen::Matrix4f get_model_matrix_raw(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    // model(0, 0) = cos(rotation_angle);
    // model(0, 1) = -sin(rotation_angle);
    // model(1, 0) = sin(rotation_angle);
    // model(1, 1) = cos(rotation_angle);
    double rad = deg2rad(rotation_angle);
    model << cos(rad),-sin(rad), 0, 0,
             sin(rad), cos(rad), 0, 0,
             0, 0, 1, 0,
             0, 0, 0, 1; 
    return model;
}

int main() {
    std::cout << "get_model_matrix_raw:\n" << get_model_matrix_raw(60.0) << std::endl;
    // std::cout << "get_model_matrix_formula:\n" << get_model_matrix_formula(60.0) << std::endl;
    // get_model_matrix_formula(60);
    // std::cout << "get_model_matrix:\n" << get_model_matrix(60.0) << std::endl;
    get_model_matrix_formula(60);
    get_model_matrix(60.0);
    return 0;
}