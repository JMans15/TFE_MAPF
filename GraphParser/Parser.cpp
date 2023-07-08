//
// Created by mansj on 10/11/22.
//

#include "Parser.h"

#include <iostream>
#include <vector>

// setting a max line length for skipped lines
#define MAX_LINE_LENGTH 4096

// #define DEBUG

std::shared_ptr<Graph> Parser::parse(std::string filename) {
#ifdef DEBUG
  std::cout << "Parsing data in file " << filename << std::endl;
#endif
  std::ifstream file;
  file.open(filename);

  // skip first line
  file.ignore(MAX_LINE_LENGTH, '\n');
  std::string text;
  int width, height;
  // get width and height
  getline(file, text, ' '); // move pointer to right before the int value
  file >> height;           // extract the value
  file >> std::ws;          // remove trailing whitespaces
  getline(file, text, ' ');
  file >> width;
  file >> std::ws;
  // DEBUG
#ifdef DEBUG
  std::cout << "Map is " << width << " x " << height << std::endl;
#endif
  // skip next line
  file.ignore(MAX_LINE_LENGTH, '\n');

  // extract the map as a list of strings
  std::vector<std::string> lines(height);
  for (int l = 0; l < height; l++) {
    getline(file, lines[l], '\n');
    file >> std::ws;
  }
  // region eigen
  /*
  // populating the matrix
  // Only checking south and east because adding both ways for each link
  SparseMatrix<int, RowMajor> result(width*height, width*height);
  cout << result.innerSize() << " x " << result.outerSize() << endl;
  result.reserve(Eigen::VectorXi::Constant(width*height, 4));
  for (int l = 0; l < height; l++) {
      for (int c = 0; c < width; c++) {
          if (lines[l][c] != '.') continue;
          // S
          if (l < height-1 && lines[l+1][c] == '.') {
              result.insert(tocellno(l+1, c, width), tocellno(l, c, width)) = 1;
              result.insert(tocellno(l, c, width), tocellno(l+1, c, width)) = 1;
          }
          // E
          if (c < width-1 && lines[l][c+1] == '.') {
              result.insert(tocellno(l, c+1, width), tocellno(l, c, width)) = 1;
              result.insert(tocellno(l, c, width), tocellno(l, c+1, width)) = 1;
          }
      }
  }
  result.makeCompressed();
  */
  // endregion

  auto result = std::make_shared<Graph>(width * height, width);
  for (int l = 0; l < height; l++) {
    for (int c = 0; c < width; c++) {
      if (lines[l][c] != '.')
        continue;
      // S
      if (l < height - 1 && lines[l + 1][c] == '.')
        result->addEdge(tocellno(l + 1, c, width), tocellno(l, c, width));
      // E
      if (c < width - 1 && lines[l][c + 1] == '.')
        result->addEdge(tocellno(l, c + 1, width), tocellno(l, c, width));
    }
  }
#ifdef DEBUG
  std::cout << "===== Graph =====" << std::endl;
  result->print(20);
  std::cout << "=================" << std::endl;
#endif
  return result;
}

int Parser::tocellno(int l, int c, int w) { return l * w + c; }
