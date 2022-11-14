//
// Created by mansj on 10/11/22.
//

#include "parser.h"
#include <iostream>
#include <vector>
// setting a max line length for skipped lines
#define MAX_LINE_LENGTH 4096
#define DEBUG

using std::ifstream;
using std::getline;
using std::ws;
using std::cout;
using std::endl;
using std::ostringstream;
using std::vector;

Graph parser::parse(string filename) {
#ifdef DEBUG
    cout << "Parsing data in file " << filename << endl;
#endif
    ifstream file;
    file.open(filename);

    // skip first line
    file.ignore(MAX_LINE_LENGTH, '\n');
    string text;
    int width, height;
    // get width and height
    getline(file, text, ' ');   // move pointer to right before the int value
    file >> height;                         // extract the value
    file >> ws;                             // remove trailing whitespaces
    getline(file, text, ' ');
    file >> width;
    file >> ws;
    // DEBUG
#ifdef DEBUG
    cout << "Map is " << width << " x " << height << endl;
#endif
    // skip next line
    file.ignore(MAX_LINE_LENGTH, '\n');

    // extract the map as a list of strings
    vector<string> lines(height);
    for (int l = 0; l < height; l++) {
        getline(file, lines[l], '\n');
        file >> ws;
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
    //endregion

    Graph result(width*height);
    for (int l = 0; l < height; l++) {
        for (int c = 0; c < width; c++) {
            if (lines[l][c] != '.') continue;
            // S
            if (l < height-1 && lines[l+1][c] == '.')
                result.add_edge(tocellno(l+1, c, width), tocellno(l, c, width));
            // E
            if (c < width-1 && lines[l][c+1] == '.')
                result.add_edge(tocellno(l, c+1, width), tocellno(l, c, width));
        }
    }
#ifdef DEBUG
    cout << "===== Graph =====" << endl;
    result.print(20);
    cout << "=================" << endl;
#endif
    return result;
}

int parser::tocellno(int l, int c, int w) {
    return l*w+c;
}
