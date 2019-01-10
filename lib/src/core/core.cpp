#include <thread>
#include <GL/glut.h>
#include "robotics/core/core.hpp"
#include "robotics/core/gl.hpp"

namespace Robotics
{
  using namespace GL;

  void init(int* argc, char** argv, bool with_gui)
  {
    if (with_gui) {
      glutInit(argc, argv);

      glutInitWindowPosition(WINDOW_POS_X, WINDOW_POS_Y);
      glutInitWindowSize(WINDOW_WIDTH, WINDOW_HEIGHT);
      glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);
      glutCreateWindow(WINDOW_TITLE);

      glutDisplayFunc(display);
      glutKeyboardFunc(keyboard);
      glutKeyboardUpFunc(keyboardUp);
      glutSpecialFunc(keyboardSpecial);
      glutSpecialUpFunc(keyboardSpecialUp);
      glutIdleFunc(idle);

      std::thread gl_thread(glutMainLoop);
      gl_thread.detach();
    }
  }
} // end namespace Robotics
