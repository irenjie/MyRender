#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "algorithm.h"
#include <time.h>
#include "model.h"
#include <opencv2/core/eigen.hpp>
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

using namespace MyRender;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow) {
    ui->setupUi(this);

    rendering();
}


void MainWindow::rendering() {
    // 初始化加载
    const int width = 800, height = 800;
    Mat img = Mat::eye(height, width, CV_8UC3);    // img
    Vector3f light_dir(1, 1, 1);
    light_dir.normalize();
    Vector3f          eye(1, 1, 3);
    Vector3f       center(0, 0, 0);
    Vector3f           up(0, 1, 0);
    Model model("D:\\Program\\CPP\\MyRender_Qt\\file\\african_head.obj");
    cv::Mat diffuse_map = cv::imread("D:\\Program\\CPP\\MyRender_Qt\\file\\african_head_diffuse.png");
    cv::Mat normal_map = cv::imread("D:\\Program\\CPP\\MyRender_Qt\\file\\african_head_nm.png");    // global frame
    cv::Mat tangent_normal_map = cv::imread("D:\\Program\\CPP\\MyRender_Qt\\file\\african_head_nm_tangent.png");    // tagent frame
    cv::Mat specular_map = cv::imread("D:\\Program\\CPP\\MyRender_Qt\\file\\african_head_spec.png");
    float *zbuffer = new float[width * height];     // zbuffer

    // look at(modelView)
    Matrix4f lookat = MyRender::lookat(eye, center, up);
    Matrix4f lookat_shadow = MyRender::lookat(light_dir, center, up);
    // projection
    float c = (eye - center).norm();
    Matrix4f projection = MyRender::prejection(c);
    // viewport
    Matrix4f viewport = MyRender::MyViewport(0, 0, width, height);
    Matrix4f uniform_M = projection * lookat;
    Matrix4f uniform_shadow = projection * lookat_shadow;
    Matrix4f uniform_Mshadow = viewport * uniform_shadow * (viewport * uniform_M).inverse();

    // 开始渲染
    clock_t start, end;
    start = clock();

    // 计算 shadowbuffer
    float *shadowbuffer = new float[width * height];
    for (int i = 0; i < width; ++i)
        for (int j = 0; j < height; ++j) {
            zbuffer[i * width + height] = std::numeric_limits<int>::min();
            shadowbuffer[i * width + height] = std::numeric_limits<int>::min();
        }
    for (int i = 0; i < model.nfaces(); i++) {
        std::vector<int> face = model.face(i);
        std::vector<cv::Vec3f> tri_points(3);
        std::vector<Vec3f> v(3);
        MatrixXf varying_uv(2, 3);
        for (int j = 0; j < 3; j++) {
            v[j] = model.vert(face[j]);
            // lookat, projection
            Vector4f v_homo(v[j].x, v[j].y, v[j].z, 1.0f);
            Vector4f v_proj = uniform_shadow * v_homo;
            v_proj /= v_proj(3);
            // viewport
            Vector3f t = (viewport * v_proj).head(3);
            cv::eigen2cv(t, tri_points[j]);
        }
        MyRender::shadow_mapping(img.size(), tri_points, shadowbuffer);
    }

    // 渲染图像
    for (int i = 0; i < model.nfaces(); i++) {
        std::vector<int> face = model.face(i);
        std::vector<int> face_texture = model.face_textures(i);
        std::vector<cv::Vec3f> tri_points(3);
        std::vector<Vec3f> v(3);
        std::vector<Vec3f> vn(3);
        MatrixXf varying_uv(2, 3);
        for (int j = 0; j < 3; j++) {
            v[j] = model.vert(face[j]);
            Vec3f vt = model.vert_texture(face_texture[j]);
            vn[j] = model.vert_normal(face[j]);

            // lookat, projection
            Vector4f v_homo(v[j].x, v[j].y, v[j].z, 1.0f);
            Vector4f v_proj = uniform_M * v_homo;
            v_proj /= v_proj(3);
            // viewport
            Vector3f t = (viewport * v_proj).head(3);
            t /= v_proj(3);
            cv::eigen2cv(t, tri_points[j]);
            // 计算 uv
            cv::Vec3f vt_real((vt.x) * (diffuse_map.size().width - 1), (vt.y) * (diffuse_map.size().height - 1), vt.z);
            int row = diffuse_map.size().height - 1 - vt_real[1], col = vt_real[0];
            varying_uv.block(0, j, 2, 1) << row, col;
        }
        drawTriangle_barycenter(v, tri_points[0], tri_points[1], tri_points[2], zbuffer, img, normal_map,
                                diffuse_map, varying_uv, light_dir, uniform_M, specular_map, tangent_normal_map,
                                shadowbuffer, uniform_Mshadow);
    }

    end = clock();
    cout << "time = " << double(end - start) / CLOCKS_PER_SEC << "s" << endl;
    cv::imwrite("D:\\Program\\CPP\\MyRender_Qt\\test.bmp", img);
}

MainWindow::~MainWindow() {
    delete ui;
}

