#ifndef DATA_H_
#define DATA_H_ 1

#include "constants.h"

#include <opencv2/opencv.hpp>
#include <zbar.h>
#include <string>

using namespace cv;
using namespace std;
using namespace zbar;

namespace jbc
{

class Contour
{
public:
  Contour();
  explicit Contour(vector<vector<Point> >* contour, size_t index);
  virtual ~Contour();

  vector<Point>* GetContour();
  Point2f* GetPoint();
  size_t GetIndex();
  void DrawContours(Mat &image, Scalar color, int lineThickness, vector<Vec4i> &hierarchy);

private:
  vector<vector<Point> >*   m_pContours;
  Point2f*                  m_pPoint;
  size_t                    m_Index;
};

class Circle
{
public:
  explicit Circle(Contour* A, Contour* B, Contour* C);
  virtual ~Circle();

  Contour* GetA();
  Contour* GetB();
  Contour* GetC();
  Contour* GetTop();
  Contour* GetRight();
  Contour* GetBottom();
  float GetX();
  float GetY();
  float GetRadius();
  bool IsValid();
  void Draw(Mat &image, Mat &traces, vector<Vec4i> &hierarchy, int qrIndex);

private:

  void CalculateCircle();
  void TestIfCircleCenterInTriangle();
  bool TestIfCircleCenterInTriangle(float deltaX, float deltaY);
  float Distance(Point2f*, Point2f*);
  bool IsInRange(float given, float mark, float env);
  void DeterminTopRightBottom();

  Contour*  m_pA;
  Contour*  m_pB;
  Contour*  m_pC;
  Contour*  m_pTop;
  Contour*  m_pRight;
  Contour*  m_pBottom;
  float     m_X;
  float     m_Y;
  float     m_Radius;
  bool      m_IsValid;
  int       m_orientation;
  int       m_slope;
};

float cv_distance(Point2f* P, Point2f* Q);          // Get Distance between two points
float cv_lineEquation(Point2f* L, Point2f* M, Point2f* J);    // Perpendicular Distance of a Point J from line formed by Points L and M; Solution to equation of the line Val = ax+by+c
float cv_lineSlope(Point2f* L, Point2f* M, int& alignement);  // Slope of a line by two Points L and M on it; Slope of line, S = (x1 -x2) / (y1- y2)
void cv_getVertices(Contour* contour, float slope, vector<Point2f>& X);
void cv_updateCorner(Point2f P, Point2f ref ,float& baseline,  Point2f& corner);
void cv_updateCornerOr(int orientation, vector<Point2f> IN, vector<Point2f> &OUT);
bool getIntersectionPoint(Point2f a1, Point2f a2, Point2f b1, Point2f b2, Point2f& intersection);
float cross(Point2f v1,Point2f v2);
string string_format(const string fmt_str, ...);

}


#endif