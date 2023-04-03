#include <algorithm.h>
#include <opencv2/core/eigen.hpp>


Matrix4f MyRender::MyViewport(int x, int y, int w, int h) {
    float depth = 255.f;
    Matrix4f m = Matrix4f::Identity();
    m(0, 3) = x + w / 2.f;
    m(1, 3) = y + h / 2.f;
    m(2, 3) = depth / 2;

    m(0, 0) = w / 2.f;
    m(1, 1) = h / 2.f;
    m(2, 2) = depth / 2.f;
    return m;
}

void MyRender::drawLine(cv::Vec2i v1, cv::Vec2i v2, cv::Mat& img, cv::Vec3b& line_color) {
    bool steep = false;
    if (std::abs(v1[0] - v2[0]) < std::abs(v1[1] - v2[1])) {
        std::swap(v1[0], v1[1]);
        std::swap(v2[0], v2[1]);
        steep = true;
    }
    if (v1[0] > v2[0]) {
        std::swap(v1[0], v2[0]);
        std::swap(v1[1], v2[1]);
    }

    int cvx, cvy;
    // 两点在同一位置
    if (v1[0] == v2[0]) {
        cvx = img.size().height - 1 - v1[1];
        cvy = v1[0];
        img.ptr<cv::Vec3b>(cvx)[cvy] = line_color;
        return;
    }

    int y = v1[1];
    float slope = (float)(v2[1] - v1[1]) / (float)(v2[0] - v1[0]);
    for (int x = v1[0]; x <= v2[0]; x++) {
        y = (int)round(slope * (x - v2[0]) + v2[1]);
        // 需将x,y从普通坐标系转入opencv坐标系
        if (steep) {
            cvy = img.size().height - 1 - x;
            cvx = y;
            img.ptr<cv::Vec3b>(cvy)[cvx] = line_color;
        } else {
            cvx = img.size().height - 1 - y;
            cvy = x;
            img.ptr<cv::Vec3b>(cvx)[cvy] = line_color;
        }
    }
}

void MyRender::drawTriangle(cv::Vec2i t0, cv::Vec2i t1, cv::Vec2i t2, cv::Mat& img, cv::Vec3b color) {
    int height = img.size().height, width = img.size().width;
    // Sort vertices of the triangle by their y-coordinates
    if (t0[1] > t1[1])
        std::swap(t0, t1);

    if (t0[1] > t2[1])
        std::swap(t0, t2);

    if (t1[1] > t2[1])
        std::swap(t1, t2);

    // 求出平顶平底三角形所需的点，以opencv坐标系为准，y坐标相同
    cv::Vec2i t_seg(0, t1[1]);
    if (t2[1] == t0[1]) {
        drawLine(t0, t2, img, color);
        drawLine(t0, t1, img, color);
        return;
    }
    if (t2[0] == t0[0]) {
        t_seg[0] = t0[0];
    } else {
        float slope_t0_t2 = (float)(t2[1] - t0[1]) / (float)(t2[0] - t0[0]);
        t_seg[0] = (int)round((t1[1] - t0[1]) / slope_t0_t2 + t0[0]);
    }
    if (t1[0] > t_seg[0])
        std::swap(t1, t_seg);

    // 上三角形
    if (t2[1] == t1[1])
        drawLine(t1, t_seg, img, color);
    else {
        for (int y = t2[1]; y > t1[1]; y-- ) {
            int x21, x2seg;
            if (t2[0] == t1[0])
                x21 = t2[0];
            else {
                float slope_t2_t1 = (float)(t2[1] - t1[1]) / (float)(t2[0] - t1[0]);
                x21 = (int)round((y - t2[1]) / slope_t2_t1 + t2[0]);
            }
            if (t2[0] == t_seg[0])
                x2seg = t2[0];
            else {
                float slope_t2_tseg = (float)(t2[1] - t_seg[1]) / (float)(t2[0] - t_seg[0]);
                x2seg = (int)round((y - t2[1]) / slope_t2_tseg + t2[0]);
            }

            int cvx, cvy;
            for (int x = x21; x <= x2seg; x++) {
                cvx = height - 1 - y;
                cvy = x;
                img.ptr<cv::Vec3b>(cvx)[cvy] = color;
            }
        }
    }
    // 下三角形
    if (t0[1] == t1[1])
        drawLine(t1, t_seg, img, color);
    else {
        for (int y = t1[1]; y >= t0[1]; y-- ) {
            int x10, xseg0;
            if (t1[0] == t0[0])
                x10 = t0[0];
            else {
                float slope_t1_t0 = (float)(t1[1] - t0[1]) / (float)(t1[0] - t0[0]);
                x10 = (int)round((y - t0[1]) / slope_t1_t0 + t0[0]);
            }
            if (t_seg[0] == t0[0])
                xseg0 = t0[0];
            else {
                float slope_tseg_t0 = (float)(t_seg[1] - t0[1]) / (float)(t_seg[0] - t0[0]);
                xseg0 = (int)round((y - t0[1]) / slope_tseg_t0 + t0[0]);
            }

            int cvx, cvy;
            for (int x = x10; x <= xseg0; x++) {
                cvx = height - 1 - y;
                cvy = x;
                img.ptr<cv::Vec3b>(cvx)[cvy] = color;
            }
        }
    }
}

cv::Vec3f MyRender::barycentric(cv::Vec3f v0, cv::Vec3f v1, cv::Vec3f v2, cv::Vec3f p) {
    cv::Vec3f u = cv::Vec3f(v1[0] - v0[0], v2[0] - v0[0], v0[0] - p[0]).cross(
                      cv::Vec3f(v1[1] - v0[1], v2[1] - v0[1], v0[1] - p[1]));
    if (std::abs(u[2]) < 1)
        return cv::Vec3f(-1, 1, 1);

    return cv::Vec3f(1.f - (u[0] + u[1]) / u[2], u[0] / u[2], u[1] / u[2]);
}

void MyRender::drawTriangle_barycenter(std::vector<Vec3f>& ndc_tri, cv::Vec3f v0, cv::Vec3f v1, cv::Vec3f v2,
                                       float *zbuffer, cv::Mat& img, cv::Mat& normal_map, cv::Mat& diffuse_map,
                                       MatrixXf& varying_uv, Vector3f& light_dir, Matrix4f& uniform_M,
                                       cv::Mat& specular_map, cv::Mat& tangent_normal_map, float *shadowbuffer,
                                       Matrix4f& uniform_Mshadow) {
    float width = img.size().height, height = img.size().width;
    // bounding box, 左下和右上
    cv::Vec2f bbox_bot_left, bbox_top_right;
    bbox_bot_left[0] = std::min(0.f, std::min(v0[0], std::min(v1[0], v2[0])));
    bbox_bot_left[1] = std::min(0.f, std::min(v0[1], std::min(v1[1], v2[1])));
    bbox_top_right[0] = std::max(width - 1, std::max(v0[0], std::max(v1[0], v2[0])));
    bbox_top_right[1] = std::max(height - 1, std::max(v0[1], std::max(v1[1], v2[1])));

    // my clip
    if (bbox_top_right[0] < 0 || bbox_bot_left[0] >= width || bbox_bot_left[1] >= height || bbox_top_right[1] < 0)
        return;
    if (bbox_bot_left[0] < 0)
        bbox_bot_left[0] = 0;
    if (bbox_top_right[0] >= width)
        bbox_top_right[0] = width - 1;
    if (bbox_bot_left[1] < 0)
        bbox_bot_left[1] = 0;
    if (bbox_top_right[1] >= height)
        bbox_top_right[1] = height - 1;

    cv::Vec3f p;
    for (p[0] = bbox_bot_left[0]; p[0] < bbox_top_right[0]; p[0]++) {
        for (p[1] = bbox_bot_left[1]; p[1] < bbox_top_right[1]; p[1]++) {
            cv::Vec3f bc = barycentric(v0, v1, v2, p);
            if (bc[0] < 0 || bc[1] < 0 || bc[2] < 0)
                continue;
            p[2] = v0[2] * bc[0] + v1[2] * bc[1] + v2[2] * bc[2];  // zbuffer
            if (zbuffer[int(p[0] + p[1]*width)] < p[2]) {
                zbuffer[int(p[0] + p[1]*width)] = p[2];
                int row = height - 1 - p[1], col = p[0];
                Vector3f bc_ei;
                cv::cv2eigen(bc, bc_ei);
                Vector2i uv = (varying_uv * bc_ei).cast<int>();

                // global normal mapping
                cv::Vec3b normal = normal_map.ptr<cv::Vec3b>(uv[0])[uv[1]];
                Vector4f n_h;
                n_h << normal[0], normal[1], normal[2], 1;
                n_h = uniform_M.inverse() * n_h;
                Vector3f n = n_h.head(3);
                n.normalize();

                // tagent normal
                // https://github.com/ssloy/tinyrenderer/wiki/Lesson-6bis:-tangent-space-normal-mapping
                /*std::vector<Vector3f> varying_normal(3);
                Vector3f bn = Vector3f::Zero();
                for (int i = 0; i < 3; i++) {
                    cv::Vec3b vn = normal_map.ptr<cv::Vec3b>((int)varying_uv(0, i))[(int)varying_uv(1, i)];
                    varying_normal[i] << vn[0], vn[1], vn[2];
                    bn += varying_normal[i] * bc[i];
                }
                bn.normalize();

                Matrix3f A = Matrix3f::Identity();
                A << (ndc_tri[1] - ndc_tri[0]).x, (ndc_tri[1] - ndc_tri[0]).y, (ndc_tri[1] - ndc_tri[0]).z,
                (ndc_tri[2] - ndc_tri[0]).x, (ndc_tri[2] - ndc_tri[0]).y, (ndc_tri[2] - ndc_tri[0]).z,
                bn[0], bn[1], bn[2];
                // compute two unknown vectors (i,j) of Darboux basis
                Vector3f tan_i = (A.inverse() * Vector3f(varying_uv(0, 1) - varying_uv(0, 0), varying_uv(0, 2) - varying_uv(0, 0), 0)).normalized();
                Vector3f tan_j = (A.inverse() * Vector3f(varying_uv(1, 1) - varying_uv(1, 0), varying_uv(1, 2) - varying_uv(1, 0), 0)).normalized();
                // apply the basis change from the tangent basis to the global coordinates
                Matrix3f T_tan_glo = Matrix3f::Identity();
                T_tan_glo.col(0) = tan_i;
                T_tan_glo.col(1)  = tan_j;
                T_tan_glo.col(2)  = bn;
                // 读取法线
                cv::Vec3b tagent_normal_tangent = tangent_normal_map.ptr<cv::Vec3b>(uv[0])[uv[1]];
                Vector3f n;
                n << tagent_normal_tangent[0], tagent_normal_tangent[1], tagent_normal_tangent[2];
                n = (T_tan_glo.inverse() * n).normalized();
                n = (uniform_M.inverse() * n.homogeneous()).head(3).normalized();*/

                // light direction
                Vector4f l;
                l << light_dir, 1;
                l = uniform_M * l;
                Vector3f light_proj = l.head(3);
                light_proj.normalize();

                // diffuse
                float diffuse = n.transpose() * light_proj;
                diffuse = std::max(0.f, diffuse);

                // specular
                Vector3f r = 2.f * n * (n.transpose() * light_proj) - light_proj;
                r.normalize();
                float spec_coff = specular_map.ptr(uv[0])[uv[1]];
                float speclar = std::pow(std::max(0.f, r[2]), spec_coff);
                if (spec_coff == 0)
                    speclar = 0;

                // shadow
                Vector4f sp_c(p[0], p[1], p[2], 1);   // screen pixel in camera view
                Vector4f sp_l = uniform_Mshadow * sp_c;     // 相机视角下的屏幕坐标在光源视角下的像素坐标
                sp_l /= sp_l(3);
                float shadow = .3 + .7 * (shadowbuffer[int(sp_l[0] + sp_l[1] * width)] < (sp_l[2] + 40));

                // 最终上色
                cv::Vec3b color = diffuse_map.ptr<cv::Vec3b>(uv[0])[uv[1]];
                // 环境光（5） + 漫反射 + 镜面反射
                for (int i = 0; i < 3; i++)
                    //                    color[i] = std::min<float>(5 + color[i] * (1 * diffuse + .6 * speclar), 255);
                    color[i] = std::min<float>(20 + color[i] * shadow * (1.2 * diffuse + .6 * speclar), 255);
                img.ptr<cv::Vec3b>(row)[col] = color;
            } else
                continue;
        }
    }
}

Matrix4f MyRender::prejection(float c) {
    Matrix4f proj_mat = Matrix4f::Identity(4, 4);
    proj_mat(3, 2) = -1.0f / c;

    return proj_mat;
}

Matrix4f MyRender::lookat(const Vector3f& eye, const Vector3f& center, const Vector3f& up) {
    Vector3f z = (eye - center).normalized();
    Vector3f x = (up.cross(z)).normalized();
    Vector3f y = (z.cross(x)).normalized();
    Matrix4f minv = Matrix4f::Identity();
    Matrix4f Tr = Matrix4f::Identity();

    minv.block(0, 0, 3, 1) = x;
    minv.block(0, 1, 3, 1) = y;
    minv.block(0, 2, 3, 1) = z;
    Tr.block(0, 3, 3, 1) = -center;
    //    minv.block(0, 0, 1, 3) = x.transpose();
    //    minv.block(1, 0, 1, 3) = y.transpose();
    //    minv.block(2, 0, 1, 3) = z.transpose();

    return minv * Tr;
}

void MyRender::shadow_mapping(cv::Size img_size, std::vector<cv::Vec3f>& v, float *shadowbuffer) {
    float width = img_size.height, height = img_size.width;
    // bounding box, 左下和右上
    cv::Vec2f bbox_bot_left, bbox_top_right;
    bbox_bot_left[0] = std::min(0.f, std::min(v[0][0], std::min(v[1][0], v[2][0])));
    bbox_bot_left[1] = std::min(0.f, std::min(v[0][1], std::min(v[1][1], v[2][1])));
    bbox_top_right[0] = std::max(width - 1, std::max(v[0][0], std::max(v[1][0], v[2][0])));
    bbox_top_right[1] = std::max(height - 1, std::max(v[0][1], std::max(v[1][1], v[2][1])));

    // my clip
    if (bbox_top_right[0] < 0 || bbox_bot_left[0] >= width || bbox_bot_left[1] >= height || bbox_top_right[1] < 0)
        return;
    if (bbox_bot_left[0] < 0)
        bbox_bot_left[0] = 0;
    if (bbox_top_right[0] >= width)
        bbox_top_right[0] = width - 1;
    if (bbox_bot_left[1] < 0)
        bbox_bot_left[1] = 0;
    if (bbox_top_right[1] >= height)
        bbox_top_right[1] = height - 1;

    cv::Vec3f p;
    for (p[0] = bbox_bot_left[0]; p[0] < bbox_top_right[0]; p[0]++) {
        for (p[1] = bbox_bot_left[1]; p[1] < bbox_top_right[1]; p[1]++) {
            cv::Vec3f bc = barycentric(v[0], v[1], v[2], p);
            if (bc[0] < 0 || bc[1] < 0 || bc[2] < 0)
                continue;
            p[2] = v[0][2] * bc[0] + v[1][2] * bc[1] + v[2][2] * bc[2];
            if (shadowbuffer[int(p[0] + p[1]*width)] < p[2])
                shadowbuffer[int(p[0] + p[1]*width)] = p[2];
        }
    }
}























