// -------------------------------------------
// gMini : a minimal OpenGL/GLUT application
// for 3D graphics.
// Copyright (C) 2006-2008 Tamy Boubekeur
// All rights reserved.
// -------------------------------------------

// -------------------------------------------
// Disclaimer: this code is dirty in the
// meaning that there is no attention paid to
// proper class attribute access, memory
// management or optimisation of any kind. It
// is designed for quick-and-dirty testing
// purpose.
// -------------------------------------------

#include <iostream>
#include <fstream>
#include <iterator>
#include <vector>
#include <algorithm>
#include <string>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>

#include <GL/glut.h>
#include <float.h>
#include "src/Vec3.h"
#include "src/Camera.h"

std::vector< float > current_field; //normalized field of each vertex

enum DisplayMode{ WIRE=0, SOLID=1, LIGHTED_WIRE=2, LIGHTED=3 };

struct Triangle {
    inline Triangle () {
        v[0] = v[1] = v[2] = 0;
    }
    inline Triangle (const Triangle & t) {
        v[0] = t.v[0];   v[1] = t.v[1];   v[2] = t.v[2];
    }
    inline Triangle (unsigned int v0, unsigned int v1, unsigned int v2) {
        v[0] = v0;   v[1] = v1;   v[2] = v2;
    }
    unsigned int & operator [] (unsigned int iv) { return v[iv]; }
    unsigned int operator [] (unsigned int iv) const { return v[iv]; }
    const bool operator == (const Triangle& t) const {return v[0] == t.v[0] && v[1] == t.v[1] && v[2] == t.v[2];}
    inline virtual ~Triangle () {}
    inline Triangle & operator = (const Triangle & t) {
        v[0] = t.v[0];   v[1] = t.v[1];   v[2] = t.v[2];
        return (*this);
    }
    // membres indices des sommets du triangle:
    unsigned int v[3];
};

void collect_one_ring (std::vector<Vec3> const & i_vertices,
                       std::vector<Triangle> const & i_triangles,
                       std::vector<std::vector<unsigned int> > & o_one_ring);

struct Mesh {
    std::vector< Vec3 > vertices; //array of mesh vertices positions
    std::vector<float> vunicurvature_; // Correspond à la demi-longueur du Laplacien
    std::vector<Vec3> vertex_laplacien;
    std::vector< Vec3 > normals; //array of vertices normals useful for the display
    std::vector< Triangle > triangles; //array of mesh triangles
    std::vector< Vec3 > triangle_normals; //triangle normals to display face normals
    std::vector<double> tshape_;
    std::vector<std::vector<double>> cot_weights; // Pour chaque voisin, on a un poids
    std::vector<Vec3> vertex_laplacien_beltrami;
    std::vector<float> vcurvature_; // Correspond à la demi-longueur du Laplacien-Beltrami
    std::vector<float> vgausscurvature_; // Courbure gaussienne

    //Compute face normals for the display
    void computeTrianglesNormals(){

        //A faire : implémenter le calcul des normales par face
        //Attention commencer la fonction par triangle_normals.clear();
        //Iterer sur les triangles

        //La normal du triangle i est le resultat du produit vectoriel de deux ses arêtes e_10 et e_20 normalisé (e_10^e_20)
        //L'arete e_10 est représentée par le vecteur partant du sommet 0 (triangles[i][0]) au sommet 1 (triangles[i][1])
        //L'arete e_20 est représentée par le vecteur partant du sommet 0 (triangles[i][0]) au sommet 2 (triangles[i][2])

        //Normaliser et ajouter dans triangle_normales

        triangle_normals.clear();
        for( unsigned int i = 0 ; i < triangles.size() ;i++ ){
            const Vec3 & e0 = vertices[triangles[i][1]] - vertices[triangles[i][0]];
            const Vec3 & e1 = vertices[triangles[i][2]] - vertices[triangles[i][0]];
            Vec3 n = Vec3::cross( e0, e1 );
            n.normalize();
            triangle_normals.push_back( n );
        }
    }

    //Compute vertices normals as the average of its incident faces normals
    void computeVerticesNormals() {
        //Utiliser weight_type : 0 uniforme, 1 aire des triangles, 2 angle du triangle

        //A faire : implémenter le calcul des normales par sommet comme la moyenne des normales des triangles incidents
        //Attention commencer la fonction par normals.clear();
        //Initializer le vecteur normals taille vertices.size() avec Vec3(0., 0., 0.)
        //Iterer sur les triangles

        //Pour chaque triangle i
        //Ajouter la normal au triangle à celle de chacun des sommets en utilisant des poids
        //0 uniforme, 1 aire du triangle, 2 angle du triangle

        //Iterer sur les normales et les normaliser
        normals.clear();
        normals.resize( vertices.size(), Vec3(0., 0., 0.) );
        for( unsigned int i = 0 ; i < triangles.size() ;i++ ){
            for( unsigned int t = 0 ; t < 3 ; t++ )
                normals[ triangles[i][t] ] += triangle_normals[i];
        }
        for( unsigned int i = 0 ; i < vertices.size() ;i++ )
            normals[ i ].normalize();
    }

    void calc_uniform_mean_curvature() { // Laplacien ici
        int nbVertices = vertices.size();
        vertex_laplacien.clear();
        vunicurvature_.clear();
        std::vector<std::vector<unsigned int>> neighbors;
        collect_one_ring(vertices, triangles, neighbors);
        for(int i = 0; i < nbVertices; ++i) {
            std::vector<unsigned int> vNeighbors = neighbors[i];
            int nbNeighbors = vNeighbors.size();
            Vec3 laplacien(0.0f,0.0f,0.0f);
            for(int j = 0; j < nbNeighbors; ++j) {
                laplacien += vertices[vNeighbors[j]];
            }
            laplacien /= nbNeighbors;
            laplacien -= vertices[i];
            vertex_laplacien.push_back(laplacien);
            double curvature = laplacien.length() / 2;
            vunicurvature_.push_back(curvature);
        }
    }

    void uniform_smooth(const unsigned int iterations) {
        for(int i = 0; i < iterations; ++i) {
            calc_uniform_mean_curvature();
            int nbVertices = vertices.size();
            for(int j = 0; j < nbVertices; ++j) {
                vertices[j] += 0.5 * vertex_laplacien[j];
            }
            computeNormals();
        }
        setup_current_field_with_uniform_mean_curvature();
        calc_gaussian_curvature();
    }

    void addNoise() {
        unsigned int nbVertices = vertices.size();
        for(unsigned int i  = 0; i < nbVertices; ++i) {
            float factor = 0.03f;
            const Vec3& p = vertices[i];
            const Vec3& n = normals[i];
            vertices[i] = Vec3( p[0] + factor*((double)(rand()) / (double)(RAND_MAX))*n[0],
                                p[1] + factor*((double)(rand()) / (double)(RAND_MAX))*n[1],
                                p[2] + factor*((double)(rand()) / (double)(RAND_MAX))*n[2]);
        }
    }

    void taubinSmooth(const unsigned int iterations, float lambda, float mu) {
        unsigned int nbVertices = vertices.size();
        for(unsigned int i = 0; i < iterations; ++i) {
            calc_uniform_mean_curvature();
            for(unsigned int j = 0; j < nbVertices; ++j) {
                vertices[j] += lambda * vertex_laplacien[j];
            }
            calc_uniform_mean_curvature();
            for(unsigned int j = 0; j < nbVertices; ++j) {
                vertices[j] += mu * vertex_laplacien[j];
            }
            computeNormals();
        }
        setup_current_field_with_uniform_mean_curvature();
        calc_gaussian_curvature();
    }

    double find_smallest_edge_length_in_triangle(const Triangle &t) {
        const Vec3& v0 = vertices[t.v[0]];
        const Vec3& v1 = vertices[t.v[1]];
        const Vec3& v2 = vertices[t.v[2]];
        
        const Vec3& a = v1 - v0;
        const Vec3& b = v2 - v0;
        const Vec3& c = v2 - v1;
        
        double a_length = a.length();
        double b_length = b.length();
        double c_length = c.length();

        return std::min({a_length, b_length, c_length});
    }

    void calc_triangle_quality() {
        tshape_.clear();
        for(auto& t : triangles) {
            const Vec3& a = vertices[t.v[1]] - vertices[t.v[0]];
            const Vec3& b = vertices[t.v[2]] - vertices[t.v[0]];
            const Vec3& c = vertices[t.v[2]] - vertices[t.v[1]];
            double A = Vec3::cross(a,b).length() / 2; // Aire du triangle
            double R = (a.length() * b.length() * c.length()) / (4*A); // Rayon du cercle circonscrit au triangle (formule de wikipedia)
            double smallest_edge_length = find_smallest_edge_length_in_triangle(t); // Plus petite arête du triangle
            tshape_.push_back(R / smallest_edge_length);
        }
    }

    std::vector<Triangle> find_two_adjacent_triangles(const unsigned int& v1, const unsigned int& v2) {
        std::vector<Triangle> corresponding_triangles;
        for(const auto& t : triangles) {
            if(std::find(std::begin(t.v), std::end(t.v), v1) != std::end(t.v) && std::find(std::begin(t.v), std::end(t.v), v2) != std::end(t.v)) {
                corresponding_triangles.push_back(t);
            }
        }
        return corresponding_triangles;
    }

    double find_angle_between_vectors(const Vec3& v1, const Vec3& v2) {
        return acos(Vec3::dot(v1, v2) / (v1.length() * v2.length()));
    }

    double cot(double theta) {
        return cos(theta) / sin(theta);
    }

    unsigned int find_different_element(const std::vector<unsigned int>& elements, const unsigned int& a, const unsigned int& b) {
        for(const unsigned int& e : elements) {
            if(e != a && e != b) {
                return e;
            }
        }
        return -1;
    }

    void calc_cot_weights() {
        int nbVertices = vertices.size();
        cot_weights.clear();
        cot_weights.resize(nbVertices);
        std::vector<std::vector<unsigned int>> neighbors;
        collect_one_ring(vertices, triangles, neighbors);
        for(unsigned int i = 0; i < nbVertices; ++i) {
            std::vector<unsigned int> vNeighbors = neighbors[i];
            int nbNeighbors = vNeighbors.size();
            cot_weights[i].resize(nbNeighbors);
            double alpha_i = 0, beta_i = 0;
            for(unsigned int j = 0; j < nbNeighbors; ++j) {
                unsigned int currentVertex = i;
                unsigned int currentNeighbor = vNeighbors[j];
                std::vector<Triangle> adjacent_triangles = find_two_adjacent_triangles(currentVertex, currentNeighbor);
                int angles_computed = 0;
                for(auto& t : adjacent_triangles) {
                    unsigned int index = find_different_element(std::vector<unsigned int>(std::begin(t.v), std::end(t.v)), currentVertex, currentNeighbor);
                    Vec3 edge_1 = vertices[currentVertex] - vertices[index];
                    Vec3 edge_2 = vertices[currentNeighbor] - vertices[index];
                    // if(std::isnan(vertices[currentVertex][0]) || std::isnan(vertices[currentVertex][1]) || std::isnan(vertices[currentVertex][2])) {
                    //     std::cout<<"Problème"<<std::endl;
                    // }
                    // std::cout<<edge_1<<std::endl;
                    double angle = find_angle_between_vectors(edge_1, edge_2);
                    // Pour remplir alpha_i et beta_i l'un après l'autre (pas trouvé mieux)
                    if(angles_computed == 0) {
                        alpha_i = find_angle_between_vectors(edge_1, edge_2);
                    }
                    else if(angles_computed == 1) {
                        beta_i = find_angle_between_vectors(edge_1, edge_2);
                    }
                    angles_computed++;

                    if(angles_computed == 2) {
                        break;
                    }
                }
                double cot_weight = cot(alpha_i) + cot(beta_i);
                //std::cout<<cot_weight<<std::endl;
                cot_weights[i][j] = cot_weight;
            }
        }
    }

    void calc_mean_curvature() { // Laplace-Beltrami ici
        int nbVertices = vertices.size();
        vcurvature_.clear();
        vertex_laplacien_beltrami.clear();
        std::vector<std::vector<unsigned int>> neighbors;
        collect_one_ring(vertices, triangles, neighbors);
        calc_cot_weights();
        for(unsigned int i = 0; i < nbVertices; ++i) {
            std::vector<unsigned int> vNeighbors = neighbors[i];
            int nbNeighbors = vNeighbors.size();
            Vec3 laplacien_beltrami(0.0f,0.0f,0.0f);
            double sum_neighbor_weights = 0.0f;
            for(unsigned int j = 0; j < nbNeighbors; ++j) {
                sum_neighbor_weights += cot_weights[i][j];
                laplacien_beltrami += cot_weights[i][j] * (vertices[vNeighbors[j]] - vertices[i]);
            }
            laplacien_beltrami /= sum_neighbor_weights;
            vertex_laplacien_beltrami.push_back(laplacien_beltrami);
            double curvature = laplacien_beltrami.length() / 2;
            vcurvature_.push_back(curvature);
        }
    }

    void laplace_beltrami_smooth(const unsigned int iterations) {
        int nbVertices = vertices.size();
        for(unsigned int i = 0; i < iterations; ++i) {
            calc_mean_curvature();
            for(unsigned int j = 0; j < nbVertices; ++j) {
                vertices[j] += 0.5 * vertex_laplacien_beltrami[j];
            }
            computeNormals();
        }
        setup_current_field_with_mean_curvature();
        calc_gaussian_curvature();
    }

    void laplace_beltrami_taubin_smooth(const unsigned int iterations, float lambda, float mu) {
        unsigned int nbVertices = vertices.size();
        for(unsigned int i = 0; i < iterations; ++i) {
            calc_mean_curvature();
            for(unsigned int j = 0; j < nbVertices; ++j) {
                vertices[j] += lambda * vertex_laplacien_beltrami[j];
            }
            calc_mean_curvature();
            for(unsigned int j = 0; j < nbVertices; ++j) {
                vertices[j] += mu * vertex_laplacien_beltrami[j];
            }
            computeNormals();
        }
        setup_current_field_with_mean_curvature();
        calc_gaussian_curvature();
    }

    void setup_current_field_with_uniform_mean_curvature() {
        current_field.clear();
        float min_curvature = *std::min_element(vunicurvature_.begin(), vunicurvature_.end());
        float max_curvature = *std::max_element(vunicurvature_.begin(), vunicurvature_.end());
        for(auto& unicurvature : vunicurvature_) {
            unicurvature = (unicurvature - min_curvature) / (max_curvature - min_curvature);
            current_field.push_back((float)unicurvature);
        }
    }

    void setup_current_field_with_mean_curvature() {
        current_field.clear();
        float min_curvature = *std::min_element(vcurvature_.begin(), vcurvature_.end());
        float max_curvature = *std::max_element(vcurvature_.begin(), vcurvature_.end());
        for(auto& curvature : vcurvature_) {
            curvature = (curvature - min_curvature) / (max_curvature - min_curvature);
            current_field.push_back((float)curvature);
        }
    }

    void setup_current_field_with_gaussian_curvature() {
        current_field.clear();
        float min_curvature = *std::min_element(vgausscurvature_.begin(), vgausscurvature_.end());
        float max_curvature = *std::max_element(vgausscurvature_.begin(), vgausscurvature_.end());
        for(auto& gcurvature : vgausscurvature_) {
            gcurvature = (gcurvature - min_curvature) / (max_curvature - min_curvature);
            current_field.push_back(gcurvature);
        }
    }

    std::vector<Triangle> find_all_adjacents_triangles(unsigned int& vertex) {
        std::vector<Triangle> all_adjacents_triangles;
        for(const auto& t : triangles) {
            if(std::find(std::begin(t.v), std::end(t.v), vertex) != std::end(t.v)) {
                all_adjacents_triangles.push_back(t);
            }
        }
        return all_adjacents_triangles;
    }
    
    void calc_gaussian_curvature() {
        int nbVertices = vertices.size();
        std::vector<std::vector<unsigned int>> neighbors;
        collect_one_ring(vertices, triangles, neighbors);
        for(unsigned int i = 0; i < nbVertices; ++i) {
            std::vector<Triangle> all_adjacents_triangles = find_all_adjacents_triangles(i);
            float sum_angles = 0.0f;
            for(const auto& t : all_adjacents_triangles) {
                Vec3 edge_1, edge_2;
                unsigned int v0 = t.v[0];
                unsigned int v1 = t.v[1];
                unsigned int v2 = t.v[2];
                if(i == v0) {
                    edge_1 = vertices[v1] - vertices[v0];
                    edge_2 = vertices[v2] - vertices[v0];
                }
                else if(i == v1) {
                    edge_1 = vertices[v0] - vertices[v1];
                    edge_2 = vertices[v2] - vertices[v1];
                }
                else if(i == v2) {
                    edge_1 = vertices[v0] - vertices[v2];
                    edge_2 = vertices[v1] - vertices[v2];
                }
                //std::cout<<"Angle : "<<find_angle_between_vectors(edge_1, edge_2) * (180 / M_PI)<<std::endl;
                sum_angles += find_angle_between_vectors(edge_1, edge_2);
            }
            //std::cout<<"Somme angles : "<<sum_angles * (180 / M_PI)<<std::endl;
            float gauss_curvature = 2*M_PI - sum_angles;
            vgausscurvature_.push_back(gauss_curvature);
            //std::cout<<"Nb triangles voisins = "<<all_adjacents_triangles.size()<<std::endl;
        }
    }

    void computeNormals(){
        computeTrianglesNormals();
        computeVerticesNormals();
    }
};

//Transformation made of a rotation and translation
struct Transformation {
    Mat3 rotation;
    Vec3 translation;
};


bool contain(std::vector<unsigned int> const & i_vector, unsigned int element) {
    for (unsigned int i = 0; i < i_vector.size(); i++) {
        if (i_vector[i] == element) return true;
    }
    return false;
}

void collect_one_ring (std::vector<Vec3> const & i_vertices,
                       std::vector< Triangle > const & i_triangles,
                       std::vector<std::vector<unsigned int> > & o_one_ring) {
    o_one_ring.clear();
    o_one_ring.resize(i_vertices.size()); //one-ring of each vertex, i.e. a list of vertices with which it shares an edge
    //Parcourir les triangles et ajouter les voisins dans le 1-voisinage
    //Attention verifier que l'indice n'est pas deja present
    for (unsigned int i = 0; i < i_triangles.size(); i++) {
        //Tous les points opposés dans le triangle sont reliés
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 3; k++) {
                if (j != k) {
                    if (!contain(o_one_ring[i_triangles[i][j]], i_triangles[i][k])) {
                        o_one_ring[i_triangles[i][j]].push_back(i_triangles[i][k]);
                    }
                }
            }
        }
    }
}


//Input mesh loaded at the launch of the application
Mesh mesh;

bool display_normals;
bool display_smooth_normals;
bool display_mesh;

DisplayMode displayMode;
int weight_type;

// -------------------------------------------
// OpenGL/GLUT application code.
// -------------------------------------------

static GLint window;
static unsigned int SCREENWIDTH = 1600;
static unsigned int SCREENHEIGHT = 900;
static Camera camera;
static bool mouseRotatePressed = false;
static bool mouseMovePressed = false;
static bool mouseZoomPressed = false;
static int lastX=0, lastY=0, lastZoom=0;
static bool fullScreen = false;

// ------------------------------------
// File I/O
// ------------------------------------
bool saveOFF( const std::string & filename ,
              std::vector< Vec3 > const & i_vertices ,
              std::vector< Vec3 > const & i_normals ,
              std::vector< Triangle > const & i_triangles,
              std::vector< Vec3 > const & i_triangle_normals ,
              bool save_normals = false ) {
    std::ofstream myfile;
    myfile.open(filename.c_str());
    if (!myfile.is_open()) {
        std::cout << filename << " cannot be opened" << std::endl;
        return false;
    }

    myfile << "OFF" << std::endl ;

    unsigned int n_vertices = i_vertices.size() , n_triangles = i_triangles.size();
    myfile << n_vertices << " " << n_triangles << " 0" << std::endl;

    for( unsigned int v = 0 ; v < n_vertices ; ++v ) {
        myfile << i_vertices[v][0] << " " << i_vertices[v][1] << " " << i_vertices[v][2] << " ";
        if (save_normals) myfile << i_normals[v][0] << " " << i_normals[v][1] << " " << i_normals[v][2] << std::endl;
        else myfile << std::endl;
    }
    for( unsigned int f = 0 ; f < n_triangles ; ++f ) {
        myfile << 3 << " " << i_triangles[f][0] << " " << i_triangles[f][1] << " " << i_triangles[f][2]<< " ";
        if (save_normals) myfile << i_triangle_normals[f][0] << " " << i_triangle_normals[f][1] << " " << i_triangle_normals[f][2];
        myfile << std::endl;
    }
    myfile.close();
    return true;
}

void openOFF( std::string const & filename,
              std::vector<Vec3> & o_vertices,
              std::vector<Vec3> & o_normals,
              std::vector< Triangle > & o_triangles,
              std::vector< Vec3 > & o_triangle_normals,
              bool load_normals = true )
{
    std::ifstream myfile;
    myfile.open(filename.c_str());
    if (!myfile.is_open())
    {
        std::cout << filename << " cannot be opened" << std::endl;
        return;
    }

    std::string magic_s;

    myfile >> magic_s;

    if( magic_s != "OFF" )
    {
        std::cout << magic_s << " != OFF :   We handle ONLY *.off files." << std::endl;
        myfile.close();
        exit(1);
    }

    int n_vertices , n_faces , dummy_int;
    myfile >> n_vertices >> n_faces >> dummy_int;

    o_vertices.clear();
    o_normals.clear();

    for( int v = 0 ; v < n_vertices ; ++v )
    {
        float x , y , z ;

        myfile >> x >> y >> z ;
        o_vertices.push_back( Vec3( x , y , z ) );

        if( load_normals ) {
            myfile >> x >> y >> z;
            o_normals.push_back( Vec3( x , y , z ) );
        }
    }

    o_triangles.clear();
    o_triangle_normals.clear();
    for( int f = 0 ; f < n_faces ; ++f )
    {
        int n_vertices_on_face;
        myfile >> n_vertices_on_face;

        if( n_vertices_on_face == 3 )
        {
            unsigned int _v1 , _v2 , _v3;
            myfile >> _v1 >> _v2 >> _v3;

            o_triangles.push_back(Triangle( _v1, _v2, _v3 ));

            if( load_normals ) {
                float x , y , z ;
                myfile >> x >> y >> z;
                o_triangle_normals.push_back( Vec3( x , y , z ) );
            }
        }
        else if( n_vertices_on_face == 4 )
        {
            unsigned int _v1 , _v2 , _v3 , _v4;
            myfile >> _v1 >> _v2 >> _v3 >> _v4;

            o_triangles.push_back(Triangle(_v1, _v2, _v3 ));
            o_triangles.push_back(Triangle(_v1, _v3, _v4));
            if( load_normals ) {
                float x , y , z ;
                myfile >> x >> y >> z;
                o_triangle_normals.push_back( Vec3( x , y , z ) );
            }

        }
        else
        {
            std::cout << "We handle ONLY *.off files with 3 or 4 vertices per face" << std::endl;
            myfile.close();
            exit(1);
        }
    }

}

// ------------------------------------
// Application initialization
// ------------------------------------
void initLight () {
    GLfloat light_position1[4] = {22.0f, 16.0f, 50.0f, 0.0f};
    GLfloat direction1[3] = {-52.0f,-16.0f,-50.0f};
    GLfloat color1[4] = {1.0f, 1.0f, 1.0f, 1.0f};
    GLfloat ambient[4] = {0.3f, 0.3f, 0.3f, 0.5f};

    glLightfv (GL_LIGHT1, GL_POSITION, light_position1);
    glLightfv (GL_LIGHT1, GL_SPOT_DIRECTION, direction1);
    glLightfv (GL_LIGHT1, GL_DIFFUSE, color1);
    glLightfv (GL_LIGHT1, GL_SPECULAR, color1);
    glLightModelfv (GL_LIGHT_MODEL_AMBIENT, ambient);
    glEnable (GL_LIGHT1);
    glEnable (GL_LIGHTING);
}

void init () {
    camera.resize (SCREENWIDTH, SCREENHEIGHT);
    initLight ();
    glCullFace (GL_BACK);
    glDisable (GL_CULL_FACE);
    glDepthFunc (GL_LESS);
    glEnable (GL_DEPTH_TEST);
    glClearColor (0.2f, 0.2f, 0.3f, 1.0f);
    glEnable(GL_COLOR_MATERIAL);
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);

    display_normals = false;
    display_mesh = true;
    display_smooth_normals = true;
    displayMode = LIGHTED;
}

// ------------------------------------
// Rendering.
// ------------------------------------

void drawVector( Vec3 const & i_from, Vec3 const & i_to ) {

    glBegin(GL_LINES);
    glVertex3f( i_from[0] , i_from[1] , i_from[2] );
    glVertex3f( i_to[0] , i_to[1] , i_to[2] );
    glEnd();
}

void drawAxis( Vec3 const & i_origin, Vec3 const & i_direction ) {

    glLineWidth(4); // for example...
    drawVector(i_origin, i_origin + i_direction);
}

void drawReferenceFrame( Vec3 const & origin, Vec3 const & i, Vec3 const & j, Vec3 const & k ) {

    glDisable(GL_LIGHTING);
    glColor3f( 0.8, 0.2, 0.2 );
    drawAxis( origin, i );
    glColor3f( 0.2, 0.8, 0.2 );
    drawAxis( origin, j );
    glColor3f( 0.2, 0.2, 0.8 );
    drawAxis( origin, k );
    glEnable(GL_LIGHTING);

}

typedef struct {
    float r;       // ∈ [0, 1]
    float g;       // ∈ [0, 1]
    float b;       // ∈ [0, 1]
} RGB;

RGB scalarToRGB( float scalar_value ) //Scalar_value ∈ [0, 1]
{
    RGB rgb;
    float H = scalar_value*360., S = 1., V = 0.85,
            P, Q, T,
            fract;

    (H == 360.)?(H = 0.):(H /= 60.);
    fract = H - floor(H);

    P = V*(1. - S);
    Q = V*(1. - S*fract);
    T = V*(1. - S*(1. - fract));

    if      (0. <= H && H < 1.)
        rgb = (RGB){.r = V, .g = T, .b = P};
    else if (1. <= H && H < 2.)
        rgb = (RGB){.r = Q, .g = V, .b = P};
    else if (2. <= H && H < 3.)
        rgb = (RGB){.r = P, .g = V, .b = T};
    else if (3. <= H && H < 4.)
        rgb = (RGB){.r = P, .g = Q, .b = V};
    else if (4. <= H && H < 5.)
        rgb = (RGB){.r = T, .g = P, .b = V};
    else if (5. <= H && H < 6.)
        rgb = (RGB){.r = V, .g = P, .b = Q};
    else
        rgb = (RGB){.r = 0., .g = 0., .b = 0.};

    return rgb;
}

void drawSmoothTriangleMesh( Mesh const & i_mesh , bool draw_field = false ) {
    glBegin(GL_TRIANGLES);
    for(unsigned int tIt = 0 ; tIt < i_mesh.triangles.size(); ++tIt) {

        for(unsigned int i = 0 ; i < 3 ; i++) {
            const Vec3 & p = i_mesh.vertices[i_mesh.triangles[tIt][i]]; //Vertex position
            const Vec3 & n = i_mesh.normals[i_mesh.triangles[tIt][i]]; //Vertex normal

            if( draw_field && current_field.size() > 0 ){
                RGB color = scalarToRGB( current_field[i_mesh.triangles[tIt][i]] );
                glColor3f( color.r, color.g, color.b );
            }
            glNormal3f( n[0] , n[1] , n[2] );
            glVertex3f( p[0] , p[1] , p[2] );
        }
    }
    glEnd();
}

void drawTriangleMesh(Mesh const & i_mesh , bool draw_field = false  ) {
    glBegin(GL_TRIANGLES);
    for(unsigned int tIt = 0 ; tIt < i_mesh.triangles.size(); ++tIt) {
        const Vec3 & n = i_mesh.triangle_normals[ tIt ]; //Triangle normal
        for(unsigned int i = 0 ; i < 3 ; i++) {
            const Vec3 & p = i_mesh.vertices[i_mesh.triangles[tIt][i]]; //Vertex position
            if( draw_field && current_field.size() > 0){
                RGB color = scalarToRGB( current_field[i_mesh.triangles[tIt][i]] );
                glColor3f( color.r, color.g, color.b );
            }
            glNormal3f( n[0] , n[1] , n[2] );
            glVertex3f( p[0] , p[1] , p[2] );
        }
    }
    glEnd();
}

void drawMesh( Mesh const & i_mesh , bool draw_field = false ){
    if(display_smooth_normals)
        drawSmoothTriangleMesh(i_mesh, draw_field) ; //Smooth display with vertices normals
    else
        drawTriangleMesh(i_mesh, draw_field); //Display with face normals
}

void drawVectorField( std::vector<Vec3> const & i_positions, std::vector<Vec3> const & i_directions ) {
    glLineWidth(1.);
    for(unsigned int pIt = 0 ; pIt < i_directions.size() ; ++pIt) {
        Vec3 to = i_positions[pIt] + 0.02*i_directions[pIt];
        drawVector(i_positions[pIt], to);
    }
}

void drawNormals(Mesh const& i_mesh){

    if(display_smooth_normals){
        drawVectorField( i_mesh.vertices, i_mesh.normals );
    } else {
        std::vector<Vec3> triangle_baricenters;
        for ( const Triangle& triangle : i_mesh.triangles ){
            Vec3 triangle_baricenter (0.,0.,0.);
            for( unsigned int i = 0 ; i < 3 ; i++ )
                triangle_baricenter += i_mesh.vertices[triangle[i]];
            triangle_baricenter /= 3.;
            triangle_baricenters.push_back(triangle_baricenter);
        }

        drawVectorField( triangle_baricenters, i_mesh.triangle_normals );
    }
}

//Draw fonction
void draw () {

    if(displayMode == LIGHTED || displayMode == LIGHTED_WIRE){

        glPolygonMode (GL_FRONT_AND_BACK, GL_FILL);
        glEnable(GL_LIGHTING);

    }  else if(displayMode == WIRE){

        glPolygonMode (GL_FRONT_AND_BACK, GL_LINE);
        glDisable (GL_LIGHTING);

    }  else if(displayMode == SOLID ){
        glDisable (GL_LIGHTING);
        glPolygonMode (GL_FRONT_AND_BACK, GL_FILL);

    }

    glColor3f(0.8,1,0.8);
    drawMesh(mesh, true);

    if(displayMode == SOLID || displayMode == LIGHTED_WIRE){
        glEnable (GL_POLYGON_OFFSET_LINE);
        glPolygonMode (GL_FRONT_AND_BACK, GL_LINE);
        glLineWidth (1.0f);
        glPolygonOffset (-2.0, 1.0);

        glColor3f(0.,0.,0.);
        drawMesh(mesh, false);

        glDisable (GL_POLYGON_OFFSET_LINE);
        glEnable (GL_LIGHTING);
    }



    glDisable(GL_LIGHTING);
    if(display_normals){
        glColor3f(1.,0.,0.);
        drawNormals(mesh);
    }

    glEnable(GL_LIGHTING);

}

void changeDisplayMode(){
    if(displayMode == LIGHTED)
        displayMode = LIGHTED_WIRE;
    else if(displayMode == LIGHTED_WIRE)
        displayMode = SOLID;
    else if(displayMode == SOLID)
        displayMode = WIRE;
    else
        displayMode = LIGHTED;
}

void display () {
    glLoadIdentity ();
    glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    camera.apply ();
    draw ();
    glFlush ();
    glutSwapBuffers ();
}

void idle () {
    glutPostRedisplay ();
}

// ------------------------------------
// User inputs
// ------------------------------------
//Keyboard event
void key (unsigned char keyPressed, int x, int y) {
    switch (keyPressed) {
    case 'f':
        if (fullScreen == true) {
            glutReshapeWindow (SCREENWIDTH, SCREENHEIGHT);
            fullScreen = false;
        } else {
            glutFullScreen ();
            fullScreen = true;
        }
        break;


    case 'w':
        changeDisplayMode();
        break;


    case 'n': //Press n key to display normals
        display_normals = !display_normals;
        break;

    case '1': //Toggle loaded mesh display
        display_mesh = !display_mesh;
        break;

    case 's': //Switches between face normals and vertices normals
        display_smooth_normals = !display_smooth_normals;
        break;

    case '+': //Changes weight type: 0 uniforme, 1 aire des triangles, 2 angle du triangle
        weight_type ++;
        if(weight_type == 3) weight_type = 0;
        mesh.computeVerticesNormals(); //recalcul des normales avec le type de poids choisi
        break;

    default:
        break;
    }
    idle ();
}

//Mouse events
void mouse (int button, int state, int x, int y) {
    if (state == GLUT_UP) {
        mouseMovePressed = false;
        mouseRotatePressed = false;
        mouseZoomPressed = false;
    } else {
        if (button == GLUT_LEFT_BUTTON) {
            camera.beginRotate (x, y);
            mouseMovePressed = false;
            mouseRotatePressed = true;
            mouseZoomPressed = false;
        } else if (button == GLUT_RIGHT_BUTTON) {
            lastX = x;
            lastY = y;
            mouseMovePressed = true;
            mouseRotatePressed = false;
            mouseZoomPressed = false;
        } else if (button == GLUT_MIDDLE_BUTTON) {
            if (mouseZoomPressed == false) {
                lastZoom = y;
                mouseMovePressed = false;
                mouseRotatePressed = false;
                mouseZoomPressed = true;
            }
        }
    }

    idle ();
}

//Mouse motion, update camera
void motion (int x, int y) {
    if (mouseRotatePressed == true) {
        camera.rotate (x, y);
    }
    else if (mouseMovePressed == true) {
        camera.move ((x-lastX)/static_cast<float>(SCREENWIDTH), (lastY-y)/static_cast<float>(SCREENHEIGHT), 0.0);
        lastX = x;
        lastY = y;
    }
    else if (mouseZoomPressed == true) {
        camera.zoom (float (y-lastZoom)/SCREENHEIGHT);
        lastZoom = y;
    }
}


void reshape(int w, int h) {
    camera.resize (w, h);
}

// ------------------------------------
// Start of graphical application
// ------------------------------------
int main (int argc, char ** argv) {
    if (argc > 2) {
        exit (EXIT_FAILURE);
    }
    glutInit (&argc, argv);
    glutInitDisplayMode (GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);
    glutInitWindowSize (SCREENWIDTH, SCREENHEIGHT);
    window = glutCreateWindow ("TP HAI917I");

    init ();
    glutIdleFunc (idle);
    glutDisplayFunc (display);
    glutKeyboardFunc (key);
    glutReshapeFunc (reshape);
    glutMotionFunc (motion);
    glutMouseFunc (mouse);
    key ('?', 0, 0);

    //Mesh loaded with precomputed normals
    openOFF("data/elephant_n.off", mesh.vertices, mesh.normals, mesh.triangles, mesh.triangle_normals);
    mesh.computeNormals();
    //mesh.addNoise();
    mesh.uniform_smooth(20);
    //mesh.laplace_beltrami_smooth(15);
    //mesh.taubinSmooth(1000, 0.330, -0.331);
    //mesh.laplace_beltrami_taubin_smooth(20, 0.330, -0.331);
    //mesh.setup_current_field_with_gaussian_curvature();
    mesh.calc_triangle_quality();
    double triangle_quality = 0.0f;
    for(auto& q : mesh.tshape_) {
        triangle_quality += q;
    }
    triangle_quality /= mesh.tshape_.size();
    std::cout<<triangle_quality<<std::endl;
    // A faire : normaliser les champs pour avoir une valeur flotante entre 0. et 1. dans current_field
    //***********************************************//

    //current_field.clear();

    glutMainLoop ();
    return EXIT_SUCCESS;
}