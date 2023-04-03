#ifndef ALGORITHM_H
#define ALGORITHM_H

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <vector>
#include "geometry.h"

using std::endl;
using std::cout;
using std::vector;

using Eigen::Matrix3f, Eigen::Matrix4f, Eigen::Vector3f, Eigen::Vector4f, Eigen::MatrixXf, Eigen::Vector2f, Eigen::Vector2i, Eigen::Vector3i;
using cv::Mat;
using cv::Scalar;

namespace MyRender {

/*
 * Bresenham's line algorithm
 * 在 img 上从 (x0,y0) 到 (x1, y1) 画直线，颜色为 line_color
 */

// viewport matrix.[-1,1]*[-1,1]*[-1,1] 到 [x,x+w]*[y,y+h]*[0,d]
Matrix4f MyViewport(int x, int y, int w, int h);

void drawLine(cv::Vec2i v1, cv::Vec2i v2, Mat& img, cv::Vec3b& line_color);

// 平顶平底法画三角形
void drawTriangle(cv::Vec2i t0, cv::Vec2i t1, cv::Vec2i t2, Mat& img, cv::Vec3b color);

// 求给定p点的重心坐标
cv::Vec3f barycentric(cv::Vec3f v0, cv::Vec3f v1, cv::Vec3f v2, cv::Vec3f p);

// 重心坐标法画三角形
void drawTriangle_barycenter(std::vector<Vec3f>& ndc_tri, cv::Vec3f v0, cv::Vec3f v1, cv::Vec3f v2, float *zbuffer, cv::Mat& img,
                             cv::Mat& normal_map, cv::Mat& diffuse_map, MatrixXf& varying_uv, Vector3f& light_dir,
                             Matrix4f& uniform_M, cv::Mat& specular_map, cv::Mat& tagent_normal_map, float *shadowbuffer,
                             Matrix4f& uniform_Mshadow);

// 对点 vertex 进行透视投影，相机位于坐标轴原点z轴c处
Matrix4f prejection(float c);

Matrix4f lookat(const Vector3f& eye, const Vector3f& center, const Vector3f& up);

void shadow_mapping(cv::Size img_size, std::vector<cv::Vec3f>& v, float *shadowbuffer);

}


#endif // ALGORITHM_H
