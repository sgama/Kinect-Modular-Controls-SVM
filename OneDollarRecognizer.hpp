/*
*     OneDollarRecognizer.hpp
*
*     This class implements the 1$ recognizer
*     Wobbrock, J.O., Wilson, A.D. and Li, Y. -- UIST 2007
*     (http://faculty.washington.edu/wobbrock/pubs/uist-07.1.pdf)
*
*     Author: Fabrizio Pece, ETH Zurich
*     fabrizio.pece@inf.ethz.ch
*
*/

#pragma once

#include <vector>
#include <cmath>
#include <cfloat>
#include <fstream>
#include <sstream>
#include "Gesture.hpp"

//! OneDollarRecognizer class.
//! Implements the 4! Gesagture recognizer algorithm
class OneDollarRecognizer {
public:
  //! OneDollarRecognizer constructor, with empty templates
  /*!
  \param _num_samples Number of samples used to resample each gesture
  \param _square_size size of the square used to normalize the gestures
  */
  OneDollarRecognizer(int _num_samples, double _square_size) :
    num_samples(_num_samples),
    square_size(_square_size) {
      half_diagonal = 0.5 * sqrt((square_size*square_size) + (square_size*square_size));
  }

  //! OneDollarRecognizer constructor that initialises the templates from a file
  /*!
  \param _num_samples Number of samples used to resample each gesture
  \param _square_size Size of the square used to normalize the gestures
  \param _gesturesDB Name of a file containing the gesture database
  */
  OneDollarRecognizer(int _num_samples, double _square_size, const std::string& _gesturesDB):
    num_samples(_num_samples),
    square_size(_square_size) {
      half_diagonal = 0.5 * sqrt((square_size*square_size) + (square_size*square_size));
      load(_gesturesDB);
  }

  //! OneDollarRecognizer destructor
  ~OneDollarRecognizer(){}

  //! Compares a gesture against the stored templates following the the
  //! 1$ recognizer algorithm
  /*!
  \param gest Gesture to recognize
  \param score Score of the best matching gesture
  \return The best matching gesture to gest
  */
  Gesture recognize(Gesture gest, double* score);

  //! Add a gesture to the template collection
  /*!
  \param gest Gesture to add
  */
  inline void addGestures(Gesture gest) {

    /**
        YOUR CODE HERE
    **/

  }

  //! Returns the size of the templates, in points (i.e. how many points
  //! are used for each template)
  /*!
  \return The number of points used to store each template
  */
  inline int getNumSamples(){return num_samples;}

  //! Returns the size of the square used to normalize the templates
  /*!
  \return The size of the square used to normalize the templates
  */
  inline int getSquareSize(){return square_size;}

  //! Saves the templates collection
  /*!
  \param sFile Name of the file onto which save the templates
  \return True if saved correctly
  */
  bool save(std::string sFile);

private:

  //! Loads the templates from a file
  /*!
  \param sFile Name of the file from which load the templates
  \return True if loaded correctly
  */
  bool load(std::string sFile);

  std::vector<Gesture> gestures; /*!< Vector holding the gesture templates */
  double square_size; /*!< Side of the sqaure used for scaling the templates */
  double half_diagonal; /*!<  Used to compute the matching scores*/
  int num_samples; /*!< Number of points stored for each gesture */
};


Gesture OneDollarRecognizer::recognize(Gesture gest, double* score) {
  Gesture found_gesture;
  *score = 0.0;

  /**
  YOUR CODE HERE
  **/

  return found_gesture;
}

bool OneDollarRecognizer::save(std::string sFile) {
  std::ofstream out_file(sFile.c_str(), std::ios::out | std::ios::trunc);
  if(!out_file.is_open()) {
    return false;
  }

  std::vector<Gesture>::const_iterator it = gestures.begin();
  while(it != gestures.end()) {
    out_file << *it << std::endl;
    ++it;
  }

  out_file.close();

  std::cout << "Saved " <<  gestures.size() << " gestures in " << sFile << std::endl;
  return true;
}

bool OneDollarRecognizer::load(std::string sFile) {
  std::ifstream in_file(sFile.c_str());
  if(!in_file) {
    std::cerr << "Error while loading gesture file: " <<  sFile.c_str() << std::endl;
    return false;
  }

  gestures.clear();
  std::stringstream ss;
  ss << in_file.rdbuf();
  in_file.close();
  std::string line;
  while(std::getline(ss, line)) {
    std::stringstream iss;
    iss << line;
    Gesture gesture;
    iss >> gesture;
    gestures.push_back(gesture);
    std::cout << "Loaded gesture:" <<  gesture.getName().c_str() << " with "
              << gesture.getPointVector().size() << " samples." << std::endl;
  }

  std::cout << "Read " <<  gestures.size() << " gestures in " << sFile << std::endl;
  return true;
}
