#pragma once

#include <GL/glut.h>

namespace Robotics
{
  namespace GL
  {
    extern int WINDOW_POS_X;
    extern int WINDOW_POS_Y;
    extern int WINDOW_WIDTH;
    extern int WINDOW_HEIGHT;
    extern char WINDOW_TITLE[];
    
    void display(void);
    void keyboard(unsigned char key, int, int);
    void keyboardUp(unsigned char key, int, int);
    void keyboardSpecial(int key, int, int);
    void keyboardSpecialUp(int key, int, int);
    void idle(void);
  } // end namespace GL
} // end namespace Robotics
