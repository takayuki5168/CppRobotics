#include "robotics/core/gl.hpp"

namespace Robotics
{
  namespace GL
  {
    int WINDOW_POS_X = 100;
    int WINDOW_POS_Y = 100;
    int WINDOW_WIDTH = 640;
    int WINDOW_HEIGHT = 480;
    char WINDOW_TITLE[] = "CppRobotics";
    
    void display(void)
    {
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

      glMatrixMode(GL_PROJECTION);
      glLoadIdentity();

      // gluPerspective(30.0,
      // static_cast<double>(WINDOW_WIDTH) / static_cast<double>(WINDOW_HEIGHT),
      // 0.1, 200000.0);



      // gluLookAt(
      // //0, 0, 10000,
      // //0, 0, 0,
      // //0.0, 1.0, 0.0);
      // pos.x - vel.x * 100,
      // pos.y - vel.y * 100,
      // pos.z + 200
      // pos.x,
      // pos.y,
      // pos.z,
      // 0.0, 0.0, 1.0);

      glMatrixMode(GL_MODELVIEW);
      glLoadIdentity();
      glViewport(0, 0, WINDOW_WIDTH, WINDOW_HEIGHT);    

      //draw 

      glutSwapBuffers();
    }

    void keyboard(unsigned char key, int, int)
    {
    }

    void keyboardUp(unsigned char key, int, int)
    {
    }

    void keyboardSpecial(int key, int, int)
    {
    }

    void keyboardSpecialUp(int key, int, int)
    {
    }

    void idle(void)
    {
      glutPostRedisplay();
    }
  } // end namespace GL
} // end namespace Robotics
