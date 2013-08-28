#include <GL/glut.h>
#include <stdlib.h>
#include <stdio.h>

// Called to draw scene
void RenderScene()
{
    // Clear the window with current clearing color
    glClear(GL_COLOR_BUFFER_BIT);

    // Flush drawing commands
    glFlush();
}

// Setup the rendering color
void SetupRC()
{
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
}


int main(int argc, char* argv[])
{
    // Initialize glut
    glutInit(&argc, argv);

    // Tell us what window we want to create
    // (single buffer, RGBA)
    glutInitDisplayMode(GLUT_SINGLE | GLUT_RGBA);

    // Setup window size/position
    glutInitWindowSize(1027, 768);
    glutInitWindowPosition(300, 50);

    // Create opengl window, the name is: Simple
    glutCreateWindow("**********Simple Window**********");

    // Setup window color
    SetupRC();

    // Setup the callback function for display
    glutDisplayFunc(RenderScene);

    // opengl loop
    glutMainLoop();

    return 0;
}
