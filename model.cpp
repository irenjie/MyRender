#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <vector>
#include "model.h"

Model::Model(const char *filename) : verts_(), texture_verts_(), faces_(), face_textures_(), verts_normal_() {
    std::ifstream in;
    in.open (filename, std::ifstream::in);
    if (in.fail())
        return;
    std::string line;
    while (!in.eof()) {
        std::getline(in, line);
        std::istringstream iss(line.c_str());
        char trash;
        if (!line.compare(0, 2, "v ")) {
            iss >> trash;
            Vec3f v;
            for (int i = 0; i < 3; i++)
                iss >> v.raw[i];
            verts_.push_back(v);
        } else if (!line.compare(0, 2, "f ")) {
            std::vector<int> f, ft;
            int itrash, idx, idx_vt;
            iss >> trash;
            while (iss >> idx >> trash >> idx_vt >> trash >> itrash) {
                idx--; // in wavefront obj all indices start at 1, not zero
                idx_vt--;
                f.push_back(idx);
                ft.push_back(idx_vt);
            }
            faces_.push_back(f);
            face_textures_.push_back(ft);
        } else if (!line.compare(0, 3, "vt ")) {
            iss >> trash >> trash;
            Vec3f vt;
            for (int i = 0; i < 3; i++)
                iss >> vt.raw[i];
            texture_verts_.push_back(vt);
        } else if (!line.compare(0, 3, "vn ")) {
            iss >> trash >> trash;
            Vec3f vn;
            for (int i = 0; i < 3; i++)
                iss >> vn.raw[i];
            verts_normal_.push_back(vn);
        }
    }
    std::cerr << "# v# " << verts_.size() << " f# "  << faces_.size() << " vt# "  << texture_verts_.size()
              << " ft# "  << face_textures_.size() << " vn# "  << verts_normal_.size() << std::endl;
}

Model::~Model() {
}

int Model::nverts() {
    return (int)verts_.size();
}

int Model::ntexture_verts() {
    return (int)texture_verts_.size();
}

int Model::nfaces() {
    return (int)faces_.size();
}

Vec3f Model::vert(int i) {
    return verts_[i];
}

Vec3f Model::vert_normal(int i) {
    return verts_normal_[i];
}

Vec3f Model::vert_texture(int i) {
    return texture_verts_[i];
}

std::vector<int> Model::face(int idx) {
    return faces_[idx];
}

std::vector<int> Model::face_textures(int idx) {
    return face_textures_[idx];
}

