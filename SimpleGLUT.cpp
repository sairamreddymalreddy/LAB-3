#include "stdafx.h"

// standard

#include <math.h>
#include<stdlib.h>
// glut
#include <GL/glut.h>

// Include standard headers
#include <assert.h>





// Include standard headers
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cmath>
#include <sstream>

// Include GLUT and OpenGL
#define GLUT_DISABLE_ATEXIT_HACK


// Define constants
#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

#include "Quaternion.h"
#include "Interpolation.h"
#include <cstdlib>
#include <ctime>


//==================================================
// Sphere Data Structure
//==================================================
struct Sphere {
    Vec3 pos;
    Vec3 vel;
    Vec3 angVel;     // Angular velocity vector
    Quaternion ori;  // Orientation
    double radius;
    double mass;
    Vec3 color; // Add this line: a color 
};

//==================================================
// Global Variables
//==================================================
static const int NUM_SPHERES = 3;
static double dt = 0.016;
static double restitution = 0.8;
static double floorLevel = 0.0;
static double groundFriction = 0.99;

std::vector<Sphere> spheres;

int g_windowWidth = 800;
int g_windowHeight = 600;

//==================================================
// Initialization
//==================================================
void initSpheres() {
    spheres.resize(NUM_SPHERES);
    std::srand((unsigned)std::time(0));
    for (int i = 0; i < NUM_SPHERES; i++) {
        spheres[i].radius = 1.0;
        spheres[i].mass = 1.0;
        spheres[i].pos.x = (std::rand() % 10) - 5.0;
        spheres[i].pos.y = 5.0 + (std::rand() % 5);
        spheres[i].pos.z = (std::rand() % 10) - 5.0;

        spheres[i].vel.x = ((std::rand() % 100) / 100.0 - 0.5) * 2.0;
        spheres[i].vel.y = 0.0;
        spheres[i].vel.z = ((std::rand() % 100) / 100.0 - 0.5) * 2.0;

        spheres[i].ori = { 1,0,0,0 };
        spheres[i].angVel.x = ((std::rand() % 100) / 100.0 - 0.5) * 2.0;
        spheres[i].angVel.y = ((std::rand() % 100) / 100.0 - 0.5) * 2.0;
        spheres[i].angVel.z = ((std::rand() % 100) / 100.0 - 0.5) * 2.0;

        spheres[i].color.x = (std::rand() % 100) / 100.0;
        spheres[i].color.y = (std::rand() % 100) / 100.0;
        spheres[i].color.z = (std::rand() % 100) / 100.0;

    }
}

//==================================================
// Physics Update
//==================================================
void handleFloorCollision(Sphere& s) {
    if (s.pos.y - s.radius < floorLevel) {
        s.pos.y = floorLevel + s.radius;
        s.vel.y = -s.vel.y * restitution;
        s.vel.x *= groundFriction;
        s.vel.z *= groundFriction;
        s.angVel.x *= 0.9; s.angVel.y *= 0.9; s.angVel.z *= 0.9;
    }
}

void handleSphereCollisions() {
    for (int i = 0; i < NUM_SPHERES; i++) {
        for (int j = i + 1; j < NUM_SPHERES; j++) {
            Vec3 diff;
            diff.x = spheres[j].pos.x - spheres[i].pos.x;
            diff.y = spheres[j].pos.y - spheres[i].pos.y;
            diff.z = spheres[j].pos.z - spheres[i].pos.z;
            double dist2 = diff.x * diff.x + diff.y * diff.y + diff.z * diff.z;
            double minDist = spheres[i].radius + spheres[j].radius;

            if (dist2 < minDist * minDist) {
                double dist = std::sqrt(dist2);
                if (dist < 1e-8) {
                    diff.x = 0.0; diff.y = 1.0; diff.z = 0.0;
                    dist = minDist;
                }
                else {
                    diff.x /= dist; diff.y /= dist; diff.z /= dist;
                }

                double overlap = (minDist - dist) * 0.5;
                spheres[i].pos.x -= diff.x * overlap;
                spheres[i].pos.y -= diff.y * overlap;
                spheres[i].pos.z -= diff.z * overlap;

                spheres[j].pos.x += diff.x * overlap;
                spheres[j].pos.y += diff.y * overlap;
                spheres[j].pos.z += diff.z * overlap;

                double rvx = spheres[j].vel.x - spheres[i].vel.x;
                double rvy = spheres[j].vel.y - spheres[i].vel.y;
                double rvz = spheres[j].vel.z - spheres[i].vel.z;

                double relativeVel = rvx * diff.x + rvy * diff.y + rvz * diff.z;

                if (relativeVel < 0.0) {
                    double e = restitution;
                    double invMass1 = 1.0 / spheres[i].mass;
                    double invMass2 = 1.0 / spheres[j].mass;
                    double jImpulse = -(1 + e) * relativeVel / (invMass1 + invMass2);

                    spheres[i].vel.x -= jImpulse * diff.x * invMass1;
                    spheres[i].vel.y -= jImpulse * diff.y * invMass1;
                    spheres[i].vel.z -= jImpulse * diff.z * invMass1;

                    spheres[j].vel.x += jImpulse * diff.x * invMass2;
                    spheres[j].vel.y += jImpulse * diff.y * invMass2;
                    spheres[j].vel.z += jImpulse * diff.z * invMass2;

                    spheres[i].angVel.x += (std::rand() % 100 / 100.0 - 0.5) * 0.2;
                    spheres[i].angVel.y += (std::rand() % 100 / 100.0 - 0.5) * 0.2;
                    spheres[i].angVel.z += (std::rand() % 100 / 100.0 - 0.5) * 0.2;

                    spheres[j].angVel.x += (std::rand() % 100 / 100.0 - 0.5) * 0.2;
                    spheres[j].angVel.y += (std::rand() % 100 / 100.0 - 0.5) * 0.2;
                    spheres[j].angVel.z += (std::rand() % 100 / 100.0 - 0.5) * 0.2;
                }
            }
        }
    }
}

void updatePhysics() {
    Vec3 gravity = { 0.0, -9.8, 0.0 };

    for (int i = 0; i < NUM_SPHERES; i++) {
        spheres[i].vel.x += gravity.x * dt;
        spheres[i].vel.y += gravity.y * dt;
        spheres[i].vel.z += gravity.z * dt;

        spheres[i].pos.x += spheres[i].vel.x * dt;
        spheres[i].pos.y += spheres[i].vel.y * dt;
        spheres[i].pos.z += spheres[i].vel.z * dt;

        handleFloorCollision(spheres[i]);

        spheres[i].ori = quatIntegrate(spheres[i].ori, spheres[i].angVel, dt);
    }

    handleSphereCollisions();
}

//==================================================
// Rendering
//==================================================
static GLfloat light0_pos[4] = { 10.0f, 20.0f, 10.0f, 1.0f };

void drawSphere(const Sphere& s) {
    glPushMatrix();
    float M[16];
    // Build the full transform matrix from the quaternion and position
    buildTransformMatrix(s.ori, s.pos, M);

    // Apply the full transformation using matrix multiplication
    glMultMatrixf(M);

    // Set the sphere color
    glColor3f((GLfloat)s.color.x, (GLfloat)s.color.y, (GLfloat)s.color.z);
    // Draw the sphere at the origin of local coordinates
    glutSolidSphere(s.radius, 50, 50);
    glPopMatrix();
}

void renderFloor() {
    glDisable(GL_LIGHTING);
    glColor3f(0.3f, 0.9f, 0.3f);
    double size = 30.0;
    glBegin(GL_QUADS);
    glVertex3f(-size, floorLevel, -size);
    glVertex3f(size, floorLevel, -size);
    glVertex3f(size, floorLevel, size);
    glVertex3f(-size, floorLevel, size);
    glEnd();
    glEnable(GL_LIGHTING);
}

void display() {
    
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f); // black background
    //glClearDepth(1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Render state
    glEnable(GL_DEPTH_TEST);
    glShadeModel(GL_SMOOTH);

    // Enable lighting
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);

    // Light source attributes
    GLfloat LightAmbient[] = { 0.4f, 0.4f, 0.4f, 1.0f };
    GLfloat LightDiffuse[] = { 0.3f, 0.3f, 0.3f, 1.0f };
    GLfloat LightSpecular[] = { 0.4f, 0.4f, 0.4f, 1.0f };
    GLfloat LightPosition[] = { 5.0f, 5.0f, 5.0f, 1.0f };

    glLightfv(GL_LIGHT0, GL_AMBIENT, LightAmbient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, LightDiffuse);
    glLightfv(GL_LIGHT0, GL_SPECULAR, LightSpecular);
    glLightfv(GL_LIGHT0, GL_POSITION, LightPosition);

    // Surface material attributes
    GLfloat material_Ka[] = { 0.8f, 0.8f, 0.8f, 1.0f };
    GLfloat material_Kd[] = { 0.8f, 0.8f, 0.8f, 1.0f };
    GLfloat material_Ks[] = { 0.9f, 0.9f, 0.9f, 1.0f };
    GLfloat material_Ke[] = { 0.1f, 0.0f, 0.1f, 1.0f };
    GLfloat material_Se = 10.0f;

    glMaterialfv(GL_FRONT, GL_AMBIENT, material_Ka);
    glMaterialfv(GL_FRONT, GL_DIFFUSE, material_Kd);
    glMaterialfv(GL_FRONT, GL_SPECULAR, material_Ks);
    glMaterialf(GL_FRONT, GL_SHININESS, material_Se);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    
    // Applying camera transforms
    gluLookAt(0.0f, 20.0f, 30.0f,   // Eye position
        0.0f, 0.0f, 0.0f,    // Look-at point
        0.0f, 1.0f, 0.0f);   // Up direction

    glLightfv(GL_LIGHT0, GL_POSITION, light0_pos);

    // Render ground
    glDisable(GL_LIGHTING); // Disable lighting for ground
    renderFloor();
    glEnable(GL_LIGHTING);  // Re-enable lighting
    
 

    for (int i = 0; i < NUM_SPHERES; i++) {
        drawSphere(spheres[i]);
    }

    glutSwapBuffers();
}

void reshape(int w, int h) {
    g_windowWidth = w;
    g_windowHeight = h;
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0, (double)w / (double)h, 0.1, 1000.0);
    glMatrixMode(GL_MODELVIEW);
}

void timerFunc(int value) {
    updatePhysics();
    glutPostRedisplay();
    glutTimerFunc((unsigned int)(dt * 1000), timerFunc, 0);
}

void keyboard(unsigned char key, int x, int y) {
    if (key == 27) {
        exit(0);
    }
}

int main(int argc, char** argv) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(g_windowWidth, g_windowHeight);
    glutCreateWindow("Physics-Based Bouncing Spheres with Quaternion Orientation (No glRotate)");

    initSpheres();

    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_NORMALIZE);
    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);

    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard);
    glutTimerFunc((unsigned int)(dt * 1000), timerFunc, 0);

    glutMainLoop();

    return 0;
}
