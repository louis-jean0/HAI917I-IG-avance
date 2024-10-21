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

#include <GL/gl.h>
#include <cmath>
#include <iostream>
#include <fstream>
#include <utility>
#include <vector>
#include <algorithm>
#include <string>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <array>

#include <algorithm>
#include <GL/glut.h>
#include <float.h>
#include "src/Vec3.h"
#include "src/Camera.h"
#include "src/jmkdtree.h"

std::vector< Vec3 > positions;
std::vector< Vec3 > normals;

std::vector< Vec3 > positions2;
std::vector< Vec3 > normals2;

std::vector<Vec3> dualContouringPoints;
std::vector<unsigned int> dualContouringTriangles;

// -------------------------------------------
// OpenGL/GLUT application code.
// -------------------------------------------

static GLint window;
static unsigned int SCREENWIDTH = 1920;
static unsigned int SCREENHEIGHT = 1080;
static Camera camera;
static bool mouseRotatePressed = false;
static bool mouseMovePressed = false;
static bool mouseZoomPressed = false;
static int lastX=0, lastY=0, lastZoom=0;
static bool fullScreen = false;
static bool drawFirstPointset = true;
static bool drawSecondPointset = true;
static bool drawGridInScene = false;
static bool drawDualContouring = false;
static bool drawMesh = false;

// ------------------------------------------------------------------------------------------------------------
// i/o and some stuff
// ------------------------------------------------------------------------------------------------------------
void loadPN (const std::string & filename , std::vector< Vec3 > & o_positions , std::vector< Vec3 > & o_normals ) {
    unsigned int surfelSize = 6;
    FILE * in = fopen (filename.c_str (), "rb");
    if (in == NULL) {
        std::cout << filename << " is not a valid PN file." << std::endl;
        return;
    }
    size_t READ_BUFFER_SIZE = 1000; // for example...
    float * pn = new float[surfelSize*READ_BUFFER_SIZE];
    o_positions.clear ();
    o_normals.clear ();
    while (!feof (in)) {
        unsigned numOfPoints = fread (pn, 4, surfelSize*READ_BUFFER_SIZE, in);
        for (unsigned int i = 0; i < numOfPoints; i += surfelSize) {
            o_positions.push_back (Vec3 (pn[i], pn[i+1], pn[i+2]));
            o_normals.push_back (Vec3 (pn[i+3], pn[i+4], pn[i+5]));
        }

        if (numOfPoints < surfelSize*READ_BUFFER_SIZE) break;
    }
    fclose (in);
    delete [] pn;
}

void savePN (const std::string & filename , std::vector< Vec3 > const & o_positions , std::vector< Vec3 > const & o_normals ) {
    if ( o_positions.size() != o_normals.size() ) {
        std::cout << "The pointset you are trying to save does not contain the same number of points and normals." << std::endl;
        return;
    }
    FILE * outfile = fopen (filename.c_str (), "wb");
    if (outfile == NULL) {
        std::cout << filename << " is not a valid PN file." << std::endl;
        return;
    }
    for(unsigned int pIt = 0 ; pIt < o_positions.size() ; ++pIt) {
        fwrite (&(o_positions[pIt]) , sizeof(float), 3, outfile);
        fwrite (&(o_normals[pIt]) , sizeof(float), 3, outfile);
    }
    fclose (outfile);
}

void scaleAndCenter( std::vector< Vec3 > & io_positions ) {
    Vec3 bboxMin( FLT_MAX , FLT_MAX , FLT_MAX );
    Vec3 bboxMax( FLT_MIN , FLT_MIN , FLT_MIN );
    for(unsigned int pIt = 0 ; pIt < io_positions.size() ; ++pIt) {
        for( unsigned int coord = 0 ; coord < 3 ; ++coord ) {
            bboxMin[coord] = std::min<float>( bboxMin[coord] , io_positions[pIt][coord] );
            bboxMax[coord] = std::max<float>( bboxMax[coord] , io_positions[pIt][coord] );
        }
    }
    Vec3 bboxCenter = (bboxMin + bboxMax) / 2.f;
    float bboxLongestAxis = std::max<float>( bboxMax[0]-bboxMin[0] , std::max<float>( bboxMax[1]-bboxMin[1] , bboxMax[2]-bboxMin[2] ) );
    for(unsigned int pIt = 0 ; pIt < io_positions.size() ; ++pIt) {
        io_positions[pIt] = (io_positions[pIt] - bboxCenter) / bboxLongestAxis;
    }
}

void applyRandomRigidTransformation( std::vector< Vec3 > & io_positions , std::vector< Vec3 > & io_normals ) {
    srand(time(NULL));
    Mat3 R = Mat3::RandRotation();
    Vec3 t = Vec3::Rand(1.f);
    for(unsigned int pIt = 0 ; pIt < io_positions.size() ; ++pIt) {
        io_positions[pIt] = R * io_positions[pIt] + t;
        io_normals[pIt] = R * io_normals[pIt];
    }
}

void subsample( std::vector< Vec3 > & i_positions , std::vector< Vec3 > & i_normals , float minimumAmount = 0.1f , float maximumAmount = 0.2f ) {
    std::vector< Vec3 > newPos , newNormals;
    std::vector< unsigned int > indices(i_positions.size());
    for( unsigned int i = 0 ; i < indices.size() ; ++i ) indices[i] = i;
    srand(time(NULL));
    std::random_shuffle(indices.begin() , indices.end());
    unsigned int newSize = indices.size() * (minimumAmount + (maximumAmount-minimumAmount)*(float)(rand()) / (float)(RAND_MAX));
    newPos.resize( newSize );
    newNormals.resize( newSize );
    for( unsigned int i = 0 ; i < newPos.size() ; ++i ) {
        newPos[i] = i_positions[ indices[i] ];
        newNormals[i] = i_normals[ indices[i] ];
    }
    i_positions = newPos;
    i_normals = newNormals;
}

bool save( const std::string & filename , std::vector< Vec3 > & vertices , std::vector< unsigned int > & triangles ) {
    std::ofstream myfile;
    myfile.open(filename.c_str());
    if (!myfile.is_open()) {
        std::cout << filename << " cannot be opened" << std::endl;
        return false;
    }

    myfile << "OFF" << std::endl;

    unsigned int n_vertices = vertices.size() , n_triangles = triangles.size()/3;
    myfile << n_vertices << " " << n_triangles << " 0" << std::endl;

    for( unsigned int v = 0 ; v < n_vertices ; ++v ) {
        myfile << vertices[v][0] << " " << vertices[v][1] << " " << vertices[v][2] << std::endl;
    }
    for( unsigned int f = 0 ; f < n_triangles ; ++f ) {
        myfile << 3 << " " << triangles[3*f] << " " << triangles[3*f+1] << " " << triangles[3*f+2];
        myfile << std::endl;
    }
    myfile.close();
    return true;
}

// ------------------------------------------------------------------------------------------------------------
// rendering.
// ------------------------------------------------------------------------------------------------------------

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
    glEnable (GL_CULL_FACE);
    glDepthFunc (GL_LESS);
    glEnable (GL_DEPTH_TEST);
    glClearColor (0.2f, 0.2f, 0.3f, 1.0f);
    glEnable(GL_COLOR_MATERIAL);
}

void drawTriangleMesh( std::vector< Vec3 > const & i_positions , std::vector< unsigned int > const & i_triangles ) {
    glBegin(GL_TRIANGLES);
    for(unsigned int tIt = 0 ; tIt < i_triangles.size() / 3 ; ++tIt) {
        Vec3 p0 = i_positions[i_triangles[3*tIt]];
        Vec3 p1 = i_positions[i_triangles[3*tIt+1]];
        Vec3 p2 = i_positions[i_triangles[3*tIt+2]];

        Vec3 n = Vec3::cross(p1-p0 , p2-p0);
        n.normalize();
        glNormal3f( n[0] , n[1] , n[2] );
        glVertex3f( p0[0] , p0[1] , p0[2] );
        glVertex3f( p1[0] , p1[1] , p1[2] );
        glVertex3f( p2[0] , p2[1] , p2[2] );
    }
    glEnd();
}

void drawPointSet( std::vector< Vec3 > const & i_positions , std::vector< Vec3 > const & i_normals ) {
    glDisable(GL_LIGHTING);
    glBegin(GL_POINTS);
    for(unsigned int pIt = 0 ; pIt < i_positions.size() ; ++pIt) {
        glNormal3f( i_normals[pIt][0] , i_normals[pIt][1] , i_normals[pIt][2] );
        glVertex3f( i_positions[pIt][0] , i_positions[pIt][1] , i_positions[pIt][2] );
    }
    glEnd();
}

void idle () {
    glutPostRedisplay ();
}

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
        GLint polygonMode[2];
        glGetIntegerv(GL_POLYGON_MODE, polygonMode);
        if(polygonMode[0] != GL_FILL)
            glPolygonMode (GL_FRONT_AND_BACK, GL_FILL);
        else
            glPolygonMode (GL_FRONT_AND_BACK, GL_LINE);
        break;

    case '1':
        drawFirstPointset = !drawFirstPointset;
        break;

    case '2':
        drawSecondPointset = !drawSecondPointset;
        break;

    case 'g':
        drawGridInScene = !drawGridInScene;           
        break;

    case 'd':
        drawDualContouring = !drawDualContouring;
        break;

    case 'm':
        drawMesh = !drawMesh;
        break;

    default:
        break;
    }
    idle ();
}

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

Vec3 projectOnPlane(Vec3 &inputPoint, const Vec3 &planePoint, const Vec3 &planeNormal) {
    Vec3 normalizedNormal = planeNormal;
    normalizedNormal.normalize();
    return inputPoint - (Vec3::dot(inputPoint - planePoint, normalizedNormal) * normalizedNormal);
}


enum KernelType {
    Singular,
    Gaussian,
    Wendland
};

double SingularWeight(Vec3 const &inputPoint, Vec3 const &neighbor, double radius, double s = 2) {
    double d = Vec3::euclideanDistance(inputPoint, neighbor);
    return pow(radius / d, s);
}

double GaussianWeight(Vec3 const &inputPoint, Vec3 const &neighbor, double radius) {
    double d = Vec3::euclideanDistance(inputPoint, neighbor);
    return exp(-pow(d,2) / pow(radius,2));
}

double WendlandWeight(Vec3 const &inputPoint, Vec3 const &neighbor, double radius) {
    double d = Vec3::euclideanDistance(inputPoint, neighbor);
    return pow(1 - (d / radius),4) * (1 + 4 * (d / radius));
}

void SPSS(Vec3 inputPoint, Vec3 &outputPoint, Vec3& outputNormal, std::vector<Vec3> const &positions, std::vector<Vec3> const &normals, BasicANNkdTree const &kdtree, KernelType kernel_type, float radius, unsigned int nbIterations = 10, unsigned int knn = 200) {
    for(unsigned int i = 0; i < nbIterations; ++i) {
        ANNidxArray id_nearest_neighbors = new ANNidx[knn];
        ANNdistArray square_distances_to_neighbors = new ANNdist[knn];
        kdtree.knearest(inputPoint, knn, id_nearest_neighbors, square_distances_to_neighbors);

        std::vector<double> weight_neighbors;
        weight_neighbors.resize(knn);
        for(unsigned int j = 0; j < knn; ++j) {
            weight_neighbors[j] = kernel_type == Singular ? SingularWeight(inputPoint, positions[id_nearest_neighbors[j]], radius) : kernel_type == Gaussian ? GaussianWeight(inputPoint, positions[id_nearest_neighbors[j]], radius) : WendlandWeight(inputPoint, positions[id_nearest_neighbors[j]], radius);
        }

        Vec3 centroide(0.0,0.0,0.0);
        Vec3 normal(0.0,0.0,0.0);
        double weight_sum = 0.0;
        for(unsigned int j = 0; j < knn; ++j) {
            centroide += weight_neighbors[j] * positions[id_nearest_neighbors[j]];
            normal += weight_neighbors[j] * normals[id_nearest_neighbors[j]];
            weight_sum += weight_neighbors[j];
        }

        centroide /= weight_sum;
        normal /= weight_sum;

        outputPoint = projectOnPlane(inputPoint, centroide, normal);
        outputNormal = normal;
        inputPoint = outputPoint;

        delete [] id_nearest_neighbors;
        delete [] square_distances_to_neighbors;
    }
}

void HPSS(Vec3 inputPoint, Vec3 &outputPoint, Vec3& outputNormal, std::vector<Vec3> const &positions, std::vector<Vec3> const &normals, BasicANNkdTree const &kdtree, KernelType kernel_type, float radius, unsigned int nbIterations = 10, unsigned int knn = 20) {
    for(unsigned int i = 0; i < nbIterations; ++i) {
        ANNidxArray id_nearest_neighbors = new ANNidx[knn];
        ANNdistArray square_distances_to_neighbors = new ANNdist[knn];
        kdtree.knearest(inputPoint, knn, id_nearest_neighbors, square_distances_to_neighbors);

        std::vector<double> weight_neighbors;
        weight_neighbors.resize(knn);
        for(unsigned int j = 0; j < knn; ++j) {
            weight_neighbors[j] = kernel_type == Singular ? SingularWeight(inputPoint, positions[id_nearest_neighbors[j]], radius) : kernel_type == Gaussian ? GaussianWeight(inputPoint, positions[id_nearest_neighbors[j]], radius) : WendlandWeight(inputPoint, positions[id_nearest_neighbors[j]], radius);
        }

        std::vector<Vec3> projected_points;
        projected_points.resize(knn);
        for(unsigned int j = 0; j < knn; ++j) {
            projected_points[j] = projectOnPlane(inputPoint, positions[id_nearest_neighbors[j]], normals[id_nearest_neighbors[j]]);
        }
        Vec3 centroide(0.0,0.0,0.0);
        Vec3 normal(0.0,0.0,0.0);
        double weight_sum = 0.0;
        for(unsigned int j = 0; j < knn; ++j) {
            centroide += weight_neighbors[j] * projected_points[j];
            normal += weight_neighbors[j] * normals[id_nearest_neighbors[j]];
            weight_sum += weight_neighbors[j];
        }

        centroide /= weight_sum;
        normal /= weight_sum;

        outputPoint = projectOnPlane(inputPoint, centroide, normal);
        outputNormal = normal;
        inputPoint = outputPoint;

        delete [] id_nearest_neighbors;
        delete [] square_distances_to_neighbors;
    }
}

void noiseAlongNormal(std::vector<Vec3> &points, std::vector<Vec3> &normals, double range) {
    std::mt19937 rng(time(NULL));
    std::uniform_real_distribution<double> gen(-range,range);
    for(int i = 0; i < points.size(); ++i) {
        double randomNumber = gen(rng);
        points[i] += randomNumber * normals[i];
    }
}

// IMPLEMENTATION DU TP ICI CAR LES FORWARDS DECLARATIONS C'EST CHIANT //

// Créé un vecteur contenant les x, y, z minimaux du vecteur passé en paramètre
Vec3 minVectorVec3(const std::vector<Vec3> &V) {
    size_t V_size = V.size();
    float minX = std::numeric_limits<float>::max();
    float minY = std::numeric_limits<float>::max();
    float minZ = std::numeric_limits<float>::max();

    for(size_t i = 0; i < V_size; i++) {
        if(V[i][0] < minX) {
            minX = V[i][0];
        }
        if(V[i][1] < minY) {
            minY = V[i][1];
        }
        if(V[i][2] < minZ) {
            minZ = V[i][2];
        }
    }
    return Vec3(minX,minY,minZ);
}

// Créé un vecteur contenant les x, y, z maximaux du vecteur passé en paramètre
Vec3 maxVectorVec3(const std::vector<Vec3> &V) {                                    
    size_t V_size = V.size();
    float maxX = -std::numeric_limits<float>::max();
    float maxY = -std::numeric_limits<float>::max();
    float maxZ = -std::numeric_limits<float>::max();

    for(size_t i = 0; i < V_size; i++) {
        if(V[i][0] > maxX) {
            maxX = V[i][0];
        }

        if(V[i][1] > maxY) {
            maxY = V[i][1];
        }

        if(V[i][2] > maxZ) {
            maxZ = V[i][2];
        }
    }
    return Vec3(maxX,maxY,maxZ);
}

struct Cell {
    std::array<std::pair<Vec3,float>, 8> corners;
    bool isCandidate = false;
};

struct Grid {
    std::vector<Cell> cells;
    int dimX, dimY, dimZ;
    float dx, dy, dz; // Ici car grille régulière
    void buildGrid(const std::vector<Vec3>& positions, std::vector<Vec3>& normals, BasicANNkdTree& kdtree, KernelType kernel_type, float radius);
    void drawGrid();
    void drawCandidateCells();
};

Grid grid; // Déclaration ici car l'appel à drawGrid en a besoin (pas pratique mais bon)

float computeSDF(Vec3& gridPoint, const std::vector<Vec3>& positions, const std::vector<Vec3>& normals, BasicANNkdTree& kdtree, KernelType kernel_type, float radius) {
    Vec3 projectedPoint, projectedNormal;
    HPSS(gridPoint, projectedPoint, projectedNormal, positions, normals, kdtree, kernel_type, radius);
    float distance = (gridPoint - projectedPoint).length();
    if (Vec3::dot(gridPoint - projectedPoint, projectedNormal) < 0.0f) {
        distance = -distance;
    }
    return distance;
}

void Grid::buildGrid(const std::vector<Vec3>& positions, std::vector<Vec3>& normals, BasicANNkdTree& kdtree, KernelType kernel_type, float radius) {
    Vec3 BBmin = minVectorVec3(positions);
    Vec3 BBmax = maxVectorVec3(positions);
    BBmin -= 0.01f;
    BBmax += 0.01f;
    grid.dx = (BBmax[0] - BBmin[0]) / grid.dimX;
    grid.dy = (BBmax[1] - BBmin[1]) / grid.dimY;
    grid.dz = (BBmax[2] - BBmin[2]) / grid.dimZ;
    grid.cells.resize(grid.dimX*grid.dimY*grid.dimZ);
    for(int i = 0; i < grid.dimX; ++i) {
        for(int j = 0; j < grid.dimY; ++j) {
            for(int k = 0; k < grid.dimZ; ++k) {
                Cell c;
                for(int di = 0; di < 2; ++di) {
                    for(int dj = 0; dj < 2; ++dj) {
                        for(int dk = 0; dk < 2; ++dk) {
                            Vec3 currentCorner(BBmin[0] + (i+di)*grid.dx,
                                               BBmin[1] + (j+dj)*grid.dy,
                                               BBmin[2] + (k+dk)*grid.dz);
                            float distance = computeSDF(currentCorner, positions, normals, kdtree, kernel_type, radius);
                            c.corners[(di*4)+(dj*2)+dk] = std::make_pair(currentCorner, distance);
                        }
                    }
                }
                grid.cells[i * grid.dimY * grid.dimZ + j * grid.dimZ + k] = c;
            }
        }
    }
}

void Grid::drawCandidateCells() {
    glEnable(GL_LIGHTING);
    glDisable(GL_CULL_FACE);
    glBegin(GL_QUADS);
    for (int i = 0; i < grid.dimX; ++i) {
        for (int j = 0; j < grid.dimY; ++j) {
            for (int k = 0; k < grid.dimZ; ++k) {
                const Cell& cell = grid.cells[i * grid.dimY * grid.dimZ + j * grid.dimZ + k];
                
                if (cell.isCandidate) {

                    // Face avant (z min)
                    glNormal3f(0.0f, 0.0f, -1.0f);
                    glVertex3f(cell.corners[0].first[0], cell.corners[0].first[1], cell.corners[0].first[2]);
                    glVertex3f(cell.corners[1].first[0], cell.corners[1].first[1], cell.corners[1].first[2]);
                    glVertex3f(cell.corners[3].first[0], cell.corners[3].first[1], cell.corners[3].first[2]);
                    glVertex3f(cell.corners[2].first[0], cell.corners[2].first[1], cell.corners[2].first[2]);

                    // Face arrière (z max)
                    glNormal3f(0.0f, 0.0f, 1.0f);
                    glVertex3f(cell.corners[4].first[0], cell.corners[4].first[1], cell.corners[4].first[2]);
                    glVertex3f(cell.corners[5].first[0], cell.corners[5].first[1], cell.corners[5].first[2]);
                    glVertex3f(cell.corners[7].first[0], cell.corners[7].first[1], cell.corners[7].first[2]);
                    glVertex3f(cell.corners[6].first[0], cell.corners[6].first[1], cell.corners[6].first[2]);

                    // Face gauche (x min)
                    glNormal3f(-1.0f, 0.0f, 0.0f);
                    glVertex3f(cell.corners[0].first[0], cell.corners[0].first[1], cell.corners[0].first[2]);
                    glVertex3f(cell.corners[2].first[0], cell.corners[2].first[1], cell.corners[2].first[2]);
                    glVertex3f(cell.corners[6].first[0], cell.corners[6].first[1], cell.corners[6].first[2]);
                    glVertex3f(cell.corners[4].first[0], cell.corners[4].first[1], cell.corners[4].first[2]);

                    // Face droite (x max)
                    glNormal3f(1.0f, 0.0f, 0.0f);
                    glVertex3f(cell.corners[1].first[0], cell.corners[1].first[1], cell.corners[1].first[2]);
                    glVertex3f(cell.corners[3].first[0], cell.corners[3].first[1], cell.corners[3].first[2]);
                    glVertex3f(cell.corners[7].first[0], cell.corners[7].first[1], cell.corners[7].first[2]);
                    glVertex3f(cell.corners[5].first[0], cell.corners[5].first[1], cell.corners[5].first[2]);

                    // Face supérieure (y max)
                    glNormal3f(0.0f, 1.0f, 0.0f);
                    glVertex3f(cell.corners[2].first[0], cell.corners[2].first[1], cell.corners[2].first[2]);
                    glVertex3f(cell.corners[3].first[0], cell.corners[3].first[1], cell.corners[3].first[2]);
                    glVertex3f(cell.corners[7].first[0], cell.corners[7].first[1], cell.corners[7].first[2]);
                    glVertex3f(cell.corners[6].first[0], cell.corners[6].first[1], cell.corners[6].first[2]);

                    // Face inférieure (y min)
                    glNormal3f(0.0f, -1.0f, 0.0f);
                    glVertex3f(cell.corners[0].first[0], cell.corners[0].first[1], cell.corners[0].first[2]);
                    glVertex3f(cell.corners[1].first[0], cell.corners[1].first[1], cell.corners[1].first[2]);
                    glVertex3f(cell.corners[5].first[0], cell.corners[5].first[1], cell.corners[5].first[2]);
                    glVertex3f(cell.corners[4].first[0], cell.corners[4].first[1], cell.corners[4].first[2]);
                }
            }
        }
    }
    glEnd();
}

void Grid::drawGrid() {
    glDisable(GL_LIGHTING);
    glLineWidth(0.01f);
    glBegin(GL_LINES);
    for(int i = 0; i < grid.dimX; ++i) {
        for(int j = 0; j < grid.dimY; ++j) {
            for(int k = 0; k < grid.dimZ; ++k) {
                const Cell& cell = grid.cells[i * grid.dimY * grid.dimZ + j * grid.dimZ + k];
                glVertex3f(cell.corners[0].first[0], cell.corners[0].first[1], cell.corners[0].first[2]);
                glVertex3f(cell.corners[1].first[0], cell.corners[1].first[1], cell.corners[1].first[2]);

                glVertex3f(cell.corners[0].first[0], cell.corners[0].first[1], cell.corners[0].first[2]);
                glVertex3f(cell.corners[2].first[0], cell.corners[2].first[1], cell.corners[2].first[2]);

                glVertex3f(cell.corners[0].first[0], cell.corners[0].first[1], cell.corners[0].first[2]);
                glVertex3f(cell.corners[4].first[0], cell.corners[4].first[1], cell.corners[4].first[2]);

                glVertex3f(cell.corners[1].first[0], cell.corners[1].first[1], cell.corners[1].first[2]);
                glVertex3f(cell.corners[3].first[0], cell.corners[3].first[1], cell.corners[3].first[2]);

                glVertex3f(cell.corners[1].first[0], cell.corners[1].first[1], cell.corners[1].first[2]);
                glVertex3f(cell.corners[5].first[0], cell.corners[5].first[1], cell.corners[5].first[2]);

                glVertex3f(cell.corners[2].first[0], cell.corners[2].first[1], cell.corners[2].first[2]);
                glVertex3f(cell.corners[3].first[0], cell.corners[3].first[1], cell.corners[3].first[2]);

                glVertex3f(cell.corners[2].first[0], cell.corners[2].first[1], cell.corners[2].first[2]);
                glVertex3f(cell.corners[6].first[0], cell.corners[6].first[1], cell.corners[6].first[2]);

                glVertex3f(cell.corners[3].first[0], cell.corners[3].first[1], cell.corners[3].first[2]);
                glVertex3f(cell.corners[7].first[0], cell.corners[7].first[1], cell.corners[7].first[2]);

                glVertex3f(cell.corners[4].first[0], cell.corners[4].first[1], cell.corners[4].first[2]);
                glVertex3f(cell.corners[5].first[0], cell.corners[5].first[1], cell.corners[5].first[2]);

                glVertex3f(cell.corners[4].first[0], cell.corners[4].first[1], cell.corners[4].first[2]);
                glVertex3f(cell.corners[6].first[0], cell.corners[6].first[1], cell.corners[6].first[2]);

                glVertex3f(cell.corners[5].first[0], cell.corners[5].first[1], cell.corners[5].first[2]);
                glVertex3f(cell.corners[7].first[0], cell.corners[7].first[1], cell.corners[7].first[2]);

                glVertex3f(cell.corners[6].first[0], cell.corners[6].first[1], cell.corners[6].first[2]);
                glVertex3f(cell.corners[7].first[0], cell.corners[7].first[1], cell.corners[7].first[2]);
            }   
        }
    }
    glEnd();
}

void dualContouringBarycentre(Grid& grid, std::vector<Vec3>& dualContouringPoints) {
    for(auto& cell : grid.cells) {
        bool hasPositive = false;
        bool hasNegative = false;
        for(int i = 0; i < 8; ++i) {
            float sdf = cell.corners[i].second;
            if(sdf < 0.0) {
                hasNegative = true;
            }
            if(sdf > 0.0) {
                hasPositive = true;
            }
        }
        if(hasNegative && hasPositive) {
            Vec3 bary(0.0f,0.0f,0.0f);
            for(int i = 0; i < 8; ++i) {
                bary += cell.corners[i].first;
            }
            bary /= 8;
            dualContouringPoints.push_back(bary);
            cell.isCandidate = true;
        }
    } 
}

void generateMesh(Grid& grid, const std::vector<Vec3>& points, std::vector<unsigned int>& triangles) {
    for(unsigned int i = 0; i < points.size(); ++i) {

    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void draw () {
    glPointSize(2); // for example...
    glEnable(GL_LIGHTING);

    if(drawFirstPointset) {
        glColor3f(0.8,0.8,1);
        drawPointSet(positions , normals);
    }

    if(drawSecondPointset) {
        glColor3f(1,0.5,0.5);
        drawPointSet(positions2 , normals2);
    }

    if(drawGridInScene) {
        glColor3f(0.5f, 0.2f, 0.3f);
        grid.drawGrid();
    }

    if(drawDualContouring) {
        glColor3f(0.1f, 0.7f, 0.1f);
        drawPointSet(dualContouringPoints, normals);
    }

    if(drawMesh) {
        glColor3f(0.5f, 0.2f, 0.5f);
        grid.drawCandidateCells();
    }
}

void display () {
    glLoadIdentity ();
    glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    camera.apply ();
    draw ();
    glFlush ();
    glutSwapBuffers ();
}

int main (int argc, char ** argv) {
    if (argc > 2) {
        exit (EXIT_FAILURE);
    }
    glutInit (&argc, argv);
    glutInitDisplayMode (GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);
    glutInitWindowSize (SCREENWIDTH, SCREENHEIGHT);
    window = glutCreateWindow ("tp point processing");

    init ();
    glutIdleFunc (idle);
    glutDisplayFunc (display);
    glutKeyboardFunc (key);
    glutReshapeFunc (reshape);
    glutMotionFunc (motion);
    glutMouseFunc (mouse);
    key ('?', 0, 0);

    {
        // Load a first pointset, and build a kd-tree:
        loadPN("pointsets/face.pn" , positions , normals);
        
        BasicANNkdTree kdtree;
        kdtree.build(positions);

        int uniformDim = 32;
        grid.dimX = uniformDim; grid.dimY = uniformDim; grid.dimZ = uniformDim;
        grid.buildGrid(positions, normals, kdtree, KernelType::Gaussian, 0.5);
        dualContouringBarycentre(grid, dualContouringPoints);
        for(unsigned int i = 0; i < dualContouringPoints.size(); ++i) {
            Vec3 outputPoint;
            Vec3 outputNormal;
            HPSS(dualContouringPoints[i], outputPoint, outputNormal, positions, normals, kdtree, KernelType::Gaussian, 0.5);
            dualContouringPoints[i] = outputPoint;
        }
        generateMesh(grid, dualContouringPoints, dualContouringTriangles);
    }

    glutMainLoop ();
    return EXIT_SUCCESS;
}

