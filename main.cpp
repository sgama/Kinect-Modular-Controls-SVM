/*
*     main.cpp
*
*     This program draws straight lines connecting dots placed with mouse drag
*     It then uses the 1$ recognizer to detect or store gestures
*
*     Author: Fabrizio Pece, ETH Zurich
*     Modified by; Samson Gama, University of British Columbia
*     fabrizio.pece@inf.ethz.ch
*
*/

#include <iostream>
#ifdef __APPLE__
  #include <GLUT/glut.h>
#else
  #include <GL/glut.h>
#endif
#include "Gesture.hpp"
#include "OneDollarRecognizer.hpp"

//! Handles keyboard input.
void myKeyboardFunc( unsigned char key, int x, int y );

//! Handles mouse button input.
void myMouseFunc( int button, int state, int x, int y );

//! Handles mouse motion.
void myMouseMotionFunc(int x, int y);

//! Handles the rendering of the mouse stroke.
void display(void);

//! Initialised OpenGL/GLUT
void initRendering();

//! Handles window resizing.
void resizeWindow(int w, int h);

// Window size in pixels
int WindowHeight, WindowWidth;

bool collectPoints = false, cleanGesture = false;

Gesture gesture;
OneDollarRecognizer* odr;

void myKeyboardFunc (unsigned char key, int x, int y) {
  switch (key) {
    case 27: {// Escape key
      if(odr)
        delete odr;
      exit(0);
      break;
    } case 'a': {// Add gesture to the DB
      std::cout << "Type gesture name:" << std::endl;
      std::string gname = "";
      std::getline(std::cin, gname);
      gesture.setName(gname);
      odr->addGestures(gesture);
      std::cout << "Added gesture: " <<  gesture.getName().c_str() << " to the DB" << std::endl;
      gesture.clear();
      glutPostRedisplay();
      break;
    } case 'r': {// Recognise gesture
      double score = 0.0;
      Gesture gestureFound =  odr->recognize(gesture, &score);
      gesture.clear();
      std::cout << "Recognised: " << gestureFound.getName().c_str() << " With score " << score << std::endl;
      glutPostRedisplay();
      break;
    } case 's': {// Save the DB
      std::cout << "Type file name where you want to save the gesture DB:" << std::endl;
      std::string fname = "";
      std::getline(std::cin, fname);
      odr->save(fname);
      break;
    } default: {
      break;
    }
  }
}

// Left button presses place a control point.
void myMouseFunc( int button, int state, int x, int y ) {
  if ( button==GLUT_LEFT_BUTTON ) {
    if(state==GLUT_DOWN) {
      collectPoints = true;
      cleanGesture = false;
    } else {
      collectPoints = false;
      cleanGesture = true;
      glutPostRedisplay();
    }
  } else if( button==GLUT_RIGHT_BUTTON ) {
    if(state==GLUT_DOWN) {
      gesture.clear();
      std::cout << "Cleaned gesture" << std::endl;
      glutPostRedisplay();
    }
  }
}

void myMouseMotionFunc(int x, int y) {
  if(collectPoints) {
    double xPos = ((double)x)/((double)(WindowWidth-1));
    double yPos = ((double)y)/((double)(WindowHeight-1));
    yPos = (1.0f-yPos);			// Flip value since y position is from top row.
    gesture.addPoint(Point2D(xPos, yPos));
    glutPostRedisplay();
  }
}

void display(void) {
  glClear(GL_COLOR_BUFFER_BIT);

  // Draw the line segments
  glColor3f(1.0f, 0.0f, 0.8f);			// Reddish/purple lines
  if ( gesture.getPointVector().size() > 1 ) {
    glBegin( GL_LINE_STRIP );
    for ( int i=0; i < gesture.getPointVector().size(); ++i ) {
      glVertex2f( gesture.getPointVector()[i].x, gesture.getPointVector()[i].y );
    }
    glEnd();
  }

  // Draw the interpolated points second.
  glColor3f( 0.0f, 0.0f, 0.0f);			// Draw points in black
  glBegin( GL_POINTS );
  for ( int i=0; i < gesture.getPointVector().size(); ++i ) {
    glVertex2f( gesture.getPointVector()[i].x, gesture.getPointVector()[i].y );
  }
  glEnd();

  glFlush();
}

void initRendering() {
  glClearColor( 1.0f, 1.0f, 1.0f, 1.0f );

  // Make big points and wide lines.  (This may be commented out if desired.)
  glPointSize(3);
  glLineWidth(3);

  // The following commands should induce OpenGL to create round points and
  //	antialias points and lines.  (This is implementation dependent unfortunately, and
  //  may slow down rendering considerably.)
  //  You may comment these out if you wish.
  glEnable(GL_POINT_SMOOTH);
  glEnable(GL_LINE_SMOOTH);
  glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);	// Make round points, not square points
  glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);		// Antialias the lines
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}

void resizeWindow(int w, int h) {
  WindowHeight = (h>1) ? h : 2;
  WindowWidth = (w>1) ? w : 2;
  glViewport(0, 0, (GLsizei) w, (GLsizei) h);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluOrtho2D(0.0f, 1.0f, 0.0f, 1.0f);  // Always view [0,1]x[0,1].
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
}

void displayHelper() {
  std::cout << "\n****************************************************************************\n"
            << "Implementation of the classic 1$ recognizer.\n"
            << "You can draw a gesture pressing the mouse left button + drag.\n"
            << "Releasing the left button completes a gesture.\n"
            << "Pressing rhe right button clears the gesture.\n"
            << "'a': add a gesture to the templates. You need to input a name for it.\n"
            << "'r': recognize the current gesture.\n"
            << "'s': export the templates on disk. You need to input a name for the DB file.\n"
            << "****************************************************************************\n" << std::endl;
}

int main(int argc, char** argv) {
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB);
  glutInitWindowSize(500, 500);
  glutInitWindowPosition(100, 100);
  glutCreateWindow(argv[0]);
  initRendering();
  displayHelper();

  if(argc>1) {
    if(strcmp(argv[1],"--gestures")==0) {
      if(argc > 2) {
        odr = new OneDollarRecognizer(64, 250, std::string(argv[2]));
      } else {
        std::cerr << "After switch --gesture you need to specify the gesture file." << std::endl;
        odr = new OneDollarRecognizer(64, 250);
      }
    }
  } else {
    odr = new OneDollarRecognizer(64, 250);
  }

  glutDisplayFunc(display);
  glutReshapeFunc(resizeWindow);
  glutKeyboardFunc(myKeyboardFunc);
  glutMouseFunc(myMouseFunc);
  glutMotionFunc(myMouseMotionFunc);
  glutMainLoop();

  delete odr;
  return 0;					// This line is never reached
}
