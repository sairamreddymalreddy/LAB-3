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


//==================================================
// Sphere Data Structure
//==================================================
struct Rigidbody {
    Vec3 pos;
    Vec3 vel;
    Vec3 angVel;     // Angular velocity vector (radians per second)
    Quaternion ori;  // Orientation quaternion
    double radius;
    double mass;
    double momentOfInertia;
    Vec3 color; // color for balls RGB values
};

std::vector<Rigidbody> spheres;

//==================================================
// Global Variables
//==================================================
//physics properties
double NUM_SPHERES = 13;
//change dt as you need
double dt = 0.016; 
double restitution = 0.8;
double floorLevel = 0.0;
double groundFriction = 0.99;



int g_windowWidth = 800;
int g_windowHeight = 600;

// Timing Variables
double lastFrameTime = 0.0; 
double totalTime = 0.0;

//==================================================
// Initialization
//==================================================
void initSpheres() {
    spheres.resize(NUM_SPHERES);
    std::srand(static_cast<unsigned>(std::time(0)));
    for (int i = 0; i < NUM_SPHERES; i++) {
        //physics properties values
        spheres[i].radius = 1;
        spheres[i].mass = 1;

        // Random position within a range
        spheres[i].pos.x = (std::rand() % 10) - 5.0; // [-5, 5]
        spheres[i].pos.y = 10.0 + (std::rand() % 5); // [5, 10]
        spheres[i].pos.z = (std::rand() % 10) - 5.0; // [-5, 5]

        // Calculating moment of inertia for a solid sphere: I = 2/5 m r^2
        spheres[i].momentOfInertia = (2.0 / 5.0) * spheres[i].mass * (spheres[i].radius * spheres[i].radius);

        // Random linear velocity
        spheres[i].vel.x = ((std::rand() % 100) / 100.0 - 0.5) * 2.0; // [-1, 1]
        spheres[i].vel.y = 0.0;
        spheres[i].vel.z = ((std::rand() % 100) / 100.0 - 0.5) * 2.0; // [-1, 1]

        //Initialize random orientation for the ball using Euler angles
        double roll = ((std::rand() % 360) / 180.0) * M_PI;   // [0, 2PI]
        double pitch = ((std::rand() % 360) / 180.0) * M_PI;  // [0, 2PI]
        double yaw = ((std::rand() % 360) / 180.0) * M_PI;    // [0, 2PI]

        //this changes the random initial Euler orientation to random initial Quaternion orientation
        spheres[i].ori = eulerToQuaternion(roll, pitch, yaw);

        //spheres[i].ori = {1,0,0,0} one of random initial quaternion orientation

        // Random angular velocity
        spheres[i].angVel.x = ((std::rand() % 100) / 100.0 - 0.5) * 2.0; // [-1, 1] rad/s
        spheres[i].angVel.y = ((std::rand() % 100) / 100.0 - 0.5) * 2.0; // [-1, 1] rad/s
        spheres[i].angVel.z = ((std::rand() % 100) / 100.0 - 0.5) * 2.0; // [-1, 1] rad/s

        // Random color
        spheres[i].color.x = (std::rand() % 100) / 100.0; // [0, 1]
        spheres[i].color.y = (std::rand() % 100) / 100.0; // [0, 1]
        spheres[i].color.z = (std::rand() % 100) / 100.0; // [0, 1]
    }
}

//==================================================
// Physics Update
//==================================================
void handleFloorCollision(Rigidbody& ball) {
    if (ball.pos.y - ball.radius < floorLevel) {
        // Correct position
        ball.pos.y = floorLevel + ball.radius;

        // Reflect velocity with restitution
        ball.vel.y = -ball.vel.y * restitution;

        // Apply ground friction to horizontal velocities
        ball.vel.x *= groundFriction;
        ball.vel.z *= groundFriction;

        // Apply angular friction
        ball.angVel.x *= 0.9;
        ball.angVel.y *= 0.9;
        ball.angVel.z *= 0.9;
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
                    // Prevent division by zero; arbitrarily set direction
                    diff.x = 0.0; diff.y = 1.0; diff.z = 0.0;
                    dist = minDist;
                }
                else {
                    // Normalize difference vector
                    diff.x /= dist;
                    diff.y /= dist;
                    diff.z /= dist;
                }

                // Positional correction to avoid sinking
                double overlap = (minDist - dist) * 0.5;
                spheres[i].pos.x -= diff.x * overlap;
                spheres[i].pos.y -= diff.y * overlap;
                spheres[i].pos.z -= diff.z * overlap;

                spheres[j].pos.x += diff.x * overlap;
                spheres[j].pos.y += diff.y * overlap;
                spheres[j].pos.z += diff.z * overlap;

                // Relative velocity
                double rvx = spheres[j].vel.x - spheres[i].vel.x;
                double rvy = spheres[j].vel.y - spheres[i].vel.y;
                double rvz = spheres[j].vel.z - spheres[i].vel.z;

                double relativeVel = rvx * diff.x + rvy * diff.y + rvz * diff.z;

                // Do not resolve if velocities are separating
                if (relativeVel > 0.0)
                    continue;

                // Compute restitution
                double e = restitution;

                // Compute impulse scalar
                double jImpulse = -(1 + e) * relativeVel;
                jImpulse /= (1.0 / spheres[i].mass) + (1.0 / spheres[j].mass);

                // Apply impulse
                Vec3 impulse = { diff.x * jImpulse, diff.y * jImpulse, diff.z * jImpulse };
                spheres[i].vel.x -= impulse.x * (1.0 / spheres[i].mass);
                spheres[i].vel.y -= impulse.y * (1.0 / spheres[i].mass);
                spheres[i].vel.z -= impulse.z * (1.0 / spheres[i].mass);

                spheres[j].vel.x += impulse.x * (1.0 / spheres[j].mass);
                spheres[j].vel.y += impulse.y * (1.0 / spheres[j].mass);
                spheres[j].vel.z += impulse.z * (1.0 / spheres[j].mass);

                // Apply random angular velocity changes to simulate spin
                spheres[i].angVel.x += ((((std::rand() % 100) / 100.0 - 0.5) * 0.2)/spheres[i].momentOfInertia);
                spheres[i].angVel.y += ((((std::rand() % 100) / 100.0 - 0.5) * 0.2)/spheres[i].momentOfInertia);
                spheres[i].angVel.z += ((((std::rand() % 100) / 100.0 - 0.5) * 0.2)/spheres[i].momentOfInertia);

                spheres[j].angVel.x += ((((std::rand() % 100) / 100.0 - 0.5) * 0.2)/spheres[j].momentOfInertia);
                spheres[j].angVel.y += ((((std::rand() % 100) / 100.0 - 0.5) * 0.2)/spheres[j].momentOfInertia);
                spheres[j].angVel.z += ((((std::rand() % 100) / 100.0 - 0.5) * 0.2)/spheres[j].momentOfInertia);

            }
        }
    }
}

void updatePhysics() {
    // Gravity vector
    Vec3 gravity = { 0.0, -9.8, 0.0 };

    // Update each sphere
    for (int i = 0; i < NUM_SPHERES; i++) {
        // Apply gravity
        spheres[i].vel.x += gravity.x * dt;
        spheres[i].vel.y += gravity.y * dt;
        spheres[i].vel.z += gravity.z * dt;

        // Update position based on velocity
        spheres[i].pos.x += spheres[i].vel.x * dt;
        spheres[i].pos.y += spheres[i].vel.y * dt;
        spheres[i].pos.z += spheres[i].vel.z * dt;

        // Handle collision with floor
        handleFloorCollision(spheres[i]);

        // Integrate orientation based on angular velocity
        spheres[i].ori = quatIntegrate(spheres[i].ori, spheres[i].angVel, dt);
    }

    // Handle collisions between spheres
    handleSphereCollisions();
}

//==================================================
// Rendering
//==================================================
static GLfloat light0_pos[4] = { 10.0f, 20.0f, 10.0f, 1.0f };

void drawSphere(const Rigidbody& ball) {
    glPushMatrix();
    float rotation[16];
    // Build the full transform matrix from the quaternion and position
    buildTransformMatrix(ball.ori, ball.pos, rotation);

    // Apply the full transformation using matrix multiplication
    glMultMatrixf(rotation);

    // Set the sphere color
    glColor3f((ball.color.x), (ball.color.y),(ball.color.z));

    // Draw the sphere with number of polygons
    glutSolidSphere(static_cast<float>(ball.radius), 100, 100);
    glPopMatrix();
}

void renderFloor() {
    glDisable(GL_LIGHTING);
    glColor3f(0.3f, 0.9f, 0.3f); // Green floor
    double size = 25.0;
    glBegin(GL_QUADS);
    glVertex3f(-size, floorLevel, -size);
    glVertex3f(size, floorLevel, -size);
    glVertex3f(size, floorLevel, size);
    glVertex3f(-size, floorLevel, size);
    glEnd();
    glEnable(GL_LIGHTING);
}

void render() {
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f); // Black background
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
    gluLookAt(0.0f, 20.0f, 40.0f,   // Eye position
        0.0f, 0.0f, 0.0f,     // Look-at point
        0.0f, 1.0f, 0.0f);    // Up direction

    glLightfv(GL_LIGHT0, GL_POSITION, light0_pos);

    glDisable(GL_LIGHTING);
    // Render ground
    renderFloor();
    glEnable(GL_LIGHTING);

    // Render all spheres
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

//==================================================
// Timer Callback using Frame Time for Dynamic dt
//==================================================
void timer(int value) {
    // Get current time 
    double currentTime = glutGet(GLUT_ELAPSED_TIME)/1000.0; //time in seconds
    
    double frameTime = currentTime - lastFrameTime;
    lastFrameTime = currentTime;
    totalTime += dt;
   

    // Update physics with dynamic dt
    updatePhysics();

    // Render
    glutPostRedisplay();

    // Register ext timer callback
    glutTimerFunc(16, timer, 0); 
}

void keyboard(unsigned char key, int x, int y) {
    if (key == 27) { // ESC key
        exit(0);
    }
}

int main(int argc, char** argv) {
    // Initialize GLUT
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(g_windowWidth, g_windowHeight);
    glutCreateWindow("Physics-Based Motion Control System");

    // Initialize spheres
    initSpheres();

    // Enable OpenGL features
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_NORMALIZE);
    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);

    
    // Register callbacks
    glutDisplayFunc(render);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard);
    glutTimerFunc(16, timer, 0);

    // Enter the GLUT main loop
    glutMainLoop();

    return 0;
}
