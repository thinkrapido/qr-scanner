
#include "Circle.h"

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <stdarg.h>
#include <memory>

using namespace std;

namespace jbc
{

const int CV_QR_NORTH = 0;
const int CV_QR_EAST = 1;
const int CV_QR_SOUTH = 2;
const int CV_QR_WEST = 3;

Contour::Contour()
{}

Contour::Contour(vector<vector<Point> >* contours, size_t index)
  : m_pContours(contours), m_Index(index)
{
  m_pPoint = NULL;
}

Contour::~Contour()
{
  if (m_pPoint != NULL) {
    delete m_pPoint;
  }
}

vector<Point>* Contour::GetContour()
{
  return &((*m_pContours)[m_Index]);
}

Point2f* Contour::GetPoint()
{
  if (m_pPoint == NULL) {
    Moments M = moments( *GetContour(), false );;
    m_pPoint = new Point2f( M.m10/M.m00 , M.m01/M.m00 );
  }
  return m_pPoint;
}

size_t Contour::GetIndex()
{
  return m_Index;
}

void Contour::DrawContours(Mat &image, Scalar color, int lineThickness, vector<Vec4i> &hierarchy)
{
  drawContours( image, *m_pContours, GetIndex() , color, lineThickness, 8, hierarchy, 0 );
}


Circle::Circle(Contour* A, Contour* B, Contour* C)
    : m_pA(A), m_pB(B), m_pC(C)
{
  CalculateCircle();
  m_pTop = NULL;
  m_pRight = NULL;
  m_pBottom = NULL;
}

Circle::~Circle()
{
  delete m_pA;
  delete m_pB;
  delete m_pC;
}

void Circle::CalculateCircle()
{
  Point2f* A = m_pA->GetPoint();
  Point2f* B = m_pB->GetPoint();
  Point2f* C = m_pC->GetPoint();
  float mr = (B->y - A->y) / (B->x - A->x);
  float mt = (C->y - B->y) / (C->x - B->x);
  m_X = (mr * mt * (C->y - A->y) + mr * (B->x + C->x) - mt * (A->x + B->x)) / (2 * ( mr - mt ));
  m_Y = (-1 * (m_X - ( A->x + B->x) / 2) / mr) + (A->y + B->y) / 2;
  m_Radius = sqrt(pow(B->x - A->x, 2) + pow(B->y - A->y, 2));
  m_IsValid = true;
  TestIfCircleCenterInTriangle();
}

Contour* Circle::GetA()
{
  return m_pA;
}

Contour* Circle::GetB()
{
  return m_pB;
}

Contour* Circle::GetC()
{
  return m_pC;
}

Contour* Circle::GetTop()
{
  if (m_pTop == NULL) {
    DeterminTopRightBottom();
  }
  return m_pTop;
}

Contour* Circle::GetRight()
{
  if (m_pRight == NULL) {
    DeterminTopRightBottom();
  }
  return m_pRight;
}

Contour* Circle::GetBottom()
{
  if (m_pBottom == NULL) {
    DeterminTopRightBottom();
  }
  return m_pBottom;
}

float Circle::GetX()
{
  return m_X;
}

float Circle::GetY()
{
  return m_Y;
}

float Circle::GetRadius()
{
  return m_Radius;
}

bool Circle::IsValid()
{
  bool out = true;
  out = out && m_IsValid;
  //out = out && !HasIntersection();
  return out;
}

void Circle::TestIfCircleCenterInTriangle()
{
  float deltaX = 1;
  float deltaY = deltaX;
  m_IsValid =   TestIfCircleCenterInTriangle(deltaX, deltaY)
            ||  TestIfCircleCenterInTriangle(deltaX, -deltaY)
            ||  TestIfCircleCenterInTriangle(-deltaX, deltaY)
            ||  TestIfCircleCenterInTriangle(-deltaX, -deltaY);
}
bool Circle::TestIfCircleCenterInTriangle(float deltaX, float deltaY)
{
  bool out = true;

  float s = m_pA->GetPoint()->y * m_pC->GetPoint()->x - m_pA->GetPoint()->x * m_pC->GetPoint()->y + (m_pC->GetPoint()->y - m_pA->GetPoint()->y) * (GetX() + deltaX) + (m_pA->GetPoint()->x - m_pC->GetPoint()->x) * (GetY() + deltaY);
  float t = m_pA->GetPoint()->x * m_pB->GetPoint()->y - m_pA->GetPoint()->y * m_pB->GetPoint()->x + (m_pA->GetPoint()->y - m_pB->GetPoint()->y) * (GetX() + deltaX) + (m_pB->GetPoint()->x - m_pA->GetPoint()->x) * (GetY() + deltaY);

  if ((s < 0) != (t < 0)) {
    out = out && false;
  }

  float A = -m_pB->GetPoint()->y * m_pC->GetPoint()->x + m_pA->GetPoint()->y * (m_pC->GetPoint()->x - m_pB->GetPoint()->x) + m_pA->GetPoint()->x * (m_pB->GetPoint()->y - m_pC->GetPoint()->y) + m_pB->GetPoint()->x * m_pC->GetPoint()->y;
  if (A < 0.0)
  {
    s = -s;
    t = -t;
    A = -A;
  }
  out = out && s > 0 && t > 0 && (s + t) <= A;
  return out;
}

bool Circle::IsInRange(float given, float mark, float env)
{
  return abs(given - mark) < env;
}

float Circle::Distance(Point2f* A, Point2f* B)
{
  float x = B->x - A->x;
  float y = B->y - A->y;
  return sqrt(x * x + y * y);
}

void Circle::DeterminTopRightBottom()
{
  Contour* median1 = NULL;
  Contour* median2 = NULL;
  Contour* outlier = NULL;

  // Determining the 'top' marker
  // Vertex of the triangle NOT involved in the longest side is the 'outlier'
  float AB = cv_distance(GetA()->GetPoint(),GetB()->GetPoint());
  float BC = cv_distance(GetB()->GetPoint(),GetC()->GetPoint());
  float CA = cv_distance(GetC()->GetPoint(),GetA()->GetPoint());

  if ( AB > BC && AB > CA )
  {
    outlier = GetC(); median1=GetA(); median2=GetB();
  }
  else if ( CA > AB && CA > BC )
  {
    outlier = GetB(); median1=GetA(); median2=GetC();
  }
  else if ( BC > AB && BC > CA )
  {
    outlier = GetA();  median1=GetB(); median2=GetC();
  }

  m_pTop = outlier;              // The obvious choice

  int align, orientation;
  float dist = cv_lineEquation(median1->GetPoint(), median2->GetPoint(), outlier->GetPoint());  // Get the Perpendicular distance of the outlier from the longest side
  float slope = cv_lineSlope(median1->GetPoint(), median2->GetPoint(), align);    // Also calculate the slope of the longest side

  // Now that we have the orientation of the line formed median1 & median2 and we also have the position of the outlier w.r.t. the line
  // Determine the 'right' and 'bottom' markers

  if (align == 0)
  {
    m_pBottom = median1;
    m_pRight = median2;
  }
  else if (slope < 0 && dist < 0 )    // Orientation - North
  {
    m_pBottom = median1;
    m_pRight = median2;
    orientation = CV_QR_NORTH;
  }
  else if (slope > 0 && dist < 0 )    // Orientation - East
  {
    m_pRight = median1;
    m_pBottom = median2;
    orientation = CV_QR_EAST;
  }
  else if (slope < 0 && dist > 0 )    // Orientation - South
  {
    m_pRight = median1;
    m_pBottom = median2;
    orientation = CV_QR_SOUTH;
  }

  else if (slope > 0 && dist > 0 )    // Orientation - West
  {
    m_pBottom = median1;
    m_pRight = median2;
    orientation = CV_QR_WEST;
  }

  m_orientation = orientation;
  m_slope = slope;
}

void Circle::Draw(Mat &image, Mat &traces, vector<Vec4i> &hierarchy, int qrIndex)
{
  Mat qr_raw = Mat::zeros(100, 100, CV_8UC3 );
  Mat qr = Mat::zeros(100, 100, CV_8UC3 );
  Mat qr_gray = Mat::zeros(100, 100, CV_8UC1);
  Mat qr_thres = Mat::zeros(100, 100, CV_8UC1);

  vector<Point2f> L,M,O, tempL,tempM,tempO;
  Point2f N;

  vector<Point2f> src,dst;    // src - Source Points basically the 4 end co-ordinates of the overlay image
                  // dst - Destination Points to transform overlay image

  Mat warp_matrix;
  cv_getVertices(GetTop(),m_slope,tempL);
  cv_getVertices(GetRight(),m_slope,tempM);
  cv_getVertices(GetBottom(),m_slope,tempO);

  cv_updateCornerOr(m_orientation, tempL, L);       // Re-arrange marker corners w.r.t orientation of the QR code
  cv_updateCornerOr(m_orientation, tempM, M);       // Re-arrange marker corners w.r.t orientation of the QR code
  cv_updateCornerOr(m_orientation, tempO, O);       // Re-arrange marker corners w.r.t orientation of the QR code

  int iflag = getIntersectionPoint(M[1],M[2],O[3],O[2],N);


  src.push_back(L[0]);
  src.push_back(M[1]);
  src.push_back(N);
  src.push_back(O[3]);

  dst.push_back(Point2f(0,0));
  dst.push_back(Point2f(qr.cols,0));
  dst.push_back(Point2f(qr.cols, qr.rows));
  dst.push_back(Point2f(0, qr.rows));

  if (src.size() == 4 && dst.size() == 4 )      // Failsafe for WarpMatrix Calculation to have only 4 Points with src and dst
  {
    warp_matrix = getPerspectiveTransform(src, dst);
    warpPerspective(image, qr_raw, warp_matrix, Size(qr.cols, qr.rows));
    copyMakeBorder( qr_raw, qr, 10, 10, 10, 10,BORDER_CONSTANT, Scalar(255,255,255) );

    cvtColor(qr,qr_gray,CV_RGB2GRAY);
    threshold(qr_gray, qr_thres, 127, 255, CV_THRESH_BINARY);

    //threshold(qr_gray, qr_thres, 0, 255, CV_THRESH_OTSU);
    //for( int d=0 ; d < 4 ; d++){  src.pop_back(); dst.pop_back(); }
  }

  //Draw contours on the image
  GetTop()->      DrawContours( image, Scalar(255,200,0), 2, hierarchy);
  GetRight()->    DrawContours( image, Scalar(0,0,255)  , 2, hierarchy);
  GetBottom()->   DrawContours( image, Scalar(255,0,100), 2, hierarchy);

  // Insert Debug instructions here
  if(DBG==1)
  {
    // Debug Prints
    // Visualizations for ease of understanding
    if (m_slope > 5)
      circle( traces, Point(10,20) , 5 ,  Scalar(0,0,255), -1, 8, 0 );
    else if (m_slope < -5)
      circle( traces, Point(10,20) , 5 ,  Scalar(255,255,255), -1, 8, 0 );

    // Draw contours on Trace image for analysis
    GetTop()->      DrawContours( traces, Scalar(255,0,100), 1, hierarchy);
    GetRight()->    DrawContours( traces, Scalar(255,0,100), 1, hierarchy);
    GetBottom()->   DrawContours( traces, Scalar(255,0,100), 1, hierarchy);

    // Draw points (4 corners) on Trace image for each Identification marker
    circle( traces, L[0], 2,  Scalar(255,255,0), -1, 8, 0 );
    circle( traces, L[1], 2,  Scalar(0,255,0), -1, 8, 0 );
    circle( traces, L[2], 2,  Scalar(0,0,255), -1, 8, 0 );
    circle( traces, L[3], 2,  Scalar(128,128,128), -1, 8, 0 );

    circle( traces, M[0], 2,  Scalar(255,255,0), -1, 8, 0 );
    circle( traces, M[1], 2,  Scalar(0,255,0), -1, 8, 0 );
    circle( traces, M[2], 2,  Scalar(0,0,255), -1, 8, 0 );
    circle( traces, M[3], 2,  Scalar(128,128,128), -1, 8, 0 );

    circle( traces, O[0], 2,  Scalar(255,255,0), -1, 8, 0 );
    circle( traces, O[1], 2,  Scalar(0,255,0), -1, 8, 0 );
    circle( traces, O[2], 2,  Scalar(0,0,255), -1, 8, 0 );
    circle( traces, O[3], 2,  Scalar(128,128,128), -1, 8, 0 );

    // Draw point of the estimated 4th Corner of (entire) QR Code
    circle( traces, N, 2,  Scalar(255,255,255), -1, 8, 0 );

    circle( traces, Point2f(GetX(), GetY()), GetRadius(), Scalar(0,255,255), 1, 8, 0);

    // Draw the lines used for estimating the 4th Corner of QR Code
    line(traces,M[1],N,Scalar(0,0,255),1,8,0);
    line(traces,O[3],N,Scalar(0,0,255),1,8,0);


    // Show the Orientation of the QR Code wrt to 2D Image Space
    int fontFace = FONT_HERSHEY_PLAIN;

    if(m_orientation == CV_QR_NORTH)
    {
      putText(traces, "NORTH", Point(20,30), fontFace, 1, Scalar(0, 255, 0), 1, 8);
    }
    else if (m_orientation == CV_QR_EAST)
    {
      putText(traces, "EAST", Point(20,30), fontFace, 1, Scalar(0, 255, 0), 1, 8);
    }
    else if (m_orientation == CV_QR_SOUTH)
    {
      putText(traces, "SOUTH", Point(20,30), fontFace, 1, Scalar(0, 255, 0), 1, 8);
    }
    else if (m_orientation == CV_QR_WEST)
    {
      putText(traces, "WEST", Point(20,30), fontFace, 1, Scalar(0, 255, 0), 1, 8);
    }

    // Debug Prints
  }

  if (SHOW_QR_CODES == 1) {
    imshow (string_format("QR code #%d", qrIndex), qr_thres );
  }

  int width = qr_thres.cols;
  int height = qr_thres.rows;
  uchar* raw = (uchar *)(qr_thres.data);

  Image img(width, height, "Y800", raw, width * height);

  ImageScanner scanner;

  // Configure the reader
  scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1);

  scanner.scan(img);

  for (Image::SymbolIterator symbol = img.symbol_begin(); symbol != img.symbol_end(); ++symbol) {
    cout  << "decoded " << symbol->get_type_name()
          << " symbol \"" << symbol->get_data() << "\"" << endl;
  }
}

// Function: Routine to get Distance between two points
// Description: Given 2 points, the function returns the distance

float cv_distance(Point2f* P, Point2f* Q)
{
  return sqrt(pow(abs(P->x - Q->x),2) + pow(abs(P->y - Q->y),2)) ;
}


// Function: Perpendicular Distance of a Point J from line formed by Points L and M; Equation of the line ax+by+c=0
// Description: Given 3 points, the function derives the line quation of the first two points,
//    calculates and returns the perpendicular distance of the the 3rd point from this line.

float cv_lineEquation(Point2f* L, Point2f* M, Point2f* J)
{
  float a,b,c,pdist;

  a = -((M->y - L->y) / (M->x - L->x));
  b = 1.0;
  c = (((M->y - L->y) /(M->x - L->x)) * L->x) - L->y;

  // Now that we have a, b, c from the equation ax + by + c, time to substitute (x,y) by values from the Point J

  pdist = (a * J->x + (b * J->y) + c) / sqrt((a * a) + (b * b));
  return pdist;
}

// Function: Slope of a line by two Points L and M on it; Slope of line, S = (x1 -x2) / (y1- y2)
// Description: Function returns the slope of the line formed by given 2 points, the alignement flag
//    indicates the line is vertical and the slope is infinity.

float cv_lineSlope(Point2f* L, Point2f* M, int& alignement)
{
  float dx,dy;
  dx = M->x - L->x;
  dy = M->y - L->y;

  if ( dy != 0)
  {
    alignement = 1;
    return (dy / dx);
  }
  else        // Make sure we are not dividing by zero; so use 'alignement' flag
  {
    alignement = 0;
    return 0.0;
  }
}


// Function: Routine to calculate 4 Corners of the Marker in Image Space using Region partitioning
// Theory: OpenCV Contours stores all points that describe it and these points lie the perimeter of the polygon.
//  The below function chooses the farthest points of the polygon since they form the vertices of that polygon,
//  exactly the points we are looking for. To choose the farthest point, the polygon is divided/partitioned into
//  4 regions equal regions using bounding box. Distance algorithm is applied between the centre of bounding box
//  every contour point in that region, the farthest point is deemed as the vertex of that region. Calculating
//  for all 4 regions we obtain the 4 corners of the polygon ( - quadrilateral).
void cv_getVertices(Contour* contour, float slope, vector<Point2f>& quad)
{
  Rect box;
  vector<Point>* CCC = contour->GetContour();
  box = boundingRect( *CCC );

  Point2f M0,M1,M2,M3;
  Point2f A, B, C, D, W, X, Y, Z;

  A =  box.tl();
  B.x = box.br().x;
  B.y = box.tl().y;
  C = box.br();
  D.x = box.tl().x;
  D.y = box.br().y;


  W.x = (A.x + B.x) / 2;
  W.y = A.y;

  X.x = B.x;
  X.y = (B.y + C.y) / 2;

  Y.x = (C.x + D.x) / 2;
  Y.y = C.y;

  Z.x = D.x;
  Z.y = (D.y + A.y) / 2;

  float dmax[4];
  dmax[0]=0.0;
  dmax[1]=0.0;
  dmax[2]=0.0;
  dmax[3]=0.0;

  float pd1 = 0.0;
  float pd2 = 0.0;

  if (slope > 5 || slope < -5 )
  {

    for( size_t i = 0; i < (*CCC).size(); i++ )
    {
      Point2f* helper = new Point2f((*CCC)[i].x, (*CCC)[i].y);
      pd1 = cv_lineEquation(&C,&A,helper);  // Position of point w.r.t the diagonal AC
      pd2 = cv_lineEquation(&B,&D,helper);  // Position of point w.r.t the diagonal BD
      delete helper;

      if((pd1 >= 0.0) && (pd2 > 0.0))
      {
          cv_updateCorner((*CCC)[i],W,dmax[1],M1);
      }
      else if((pd1 > 0.0) && (pd2 <= 0.0))
      {
          cv_updateCorner((*CCC)[i],X,dmax[2],M2);
      }
      else if((pd1 <= 0.0) && (pd2 < 0.0))
      {
          cv_updateCorner((*CCC)[i],Y,dmax[3],M3);
      }
      else if((pd1 < 0.0) && (pd2 >= 0.0))
      {
          cv_updateCorner((*CCC)[i],Z,dmax[0],M0);
      }
      else
        continue;
     }
  }
  else
  {
    int halfx = (A.x + B.x) / 2;
    int halfy = (A.y + D.y) / 2;

    for( size_t i = 0; i < (*CCC).size(); i++ )
    {
      if(((*CCC)[i].x < halfx) && ((*CCC)[i].y <= halfy))
      {
          cv_updateCorner((*CCC)[i],C,dmax[2],M0);
      }
      else if(((*CCC)[i].x >= halfx) && ((*CCC)[i].y < halfy))
      {
          cv_updateCorner((*CCC)[i],D,dmax[3],M1);
      }
      else if(((*CCC)[i].x > halfx) && ((*CCC)[i].y >= halfy))
      {
          cv_updateCorner((*CCC)[i],A,dmax[0],M2);
      }
      else if(((*CCC)[i].x <= halfx) && ((*CCC)[i].y > halfy))
      {
          cv_updateCorner((*CCC)[i],B,dmax[1],M3);
      }
    }
  }

  quad.push_back(M0);
  quad.push_back(M1);
  quad.push_back(M2);
  quad.push_back(M3);

}

// Function: Compare a point if it more far than previously recorded farthest distance
// Description: Farthest Point detection using reference point and baseline distance
void cv_updateCorner(Point2f P, Point2f ref , float& baseline,  Point2f& corner)
{
    float temp_dist;
    temp_dist = cv_distance(&P,&ref);

    if(temp_dist > baseline)
    {
        baseline = temp_dist;     // The farthest distance is the new baseline
        corner = P;           // P is now the farthest point
    }

}

// Function: Sequence the Corners wrt to the orientation of the QR Code
void cv_updateCornerOr(int orientation, vector<Point2f> IN,vector<Point2f> &OUT)
{
  Point2f M0,M1,M2,M3;
      if(orientation == CV_QR_NORTH)
  {
    M0 = IN[0];
    M1 = IN[1];
    M2 = IN[2];
    M3 = IN[3];
  }
  else if (orientation == CV_QR_EAST)
  {
    M0 = IN[1];
    M1 = IN[2];
    M2 = IN[3];
    M3 = IN[0];
  }
  else if (orientation == CV_QR_SOUTH)
  {
    M0 = IN[2];
    M1 = IN[3];
    M2 = IN[0];
    M3 = IN[1];
  }
  else if (orientation == CV_QR_WEST)
  {
    M0 = IN[3];
    M1 = IN[0];
    M2 = IN[1];
    M3 = IN[2];
  }

  OUT.push_back(M0);
  OUT.push_back(M1);
  OUT.push_back(M2);
  OUT.push_back(M3);
}

// Function: Get the Intersection Point of the lines formed by sets of two points
bool getIntersectionPoint(Point2f a1, Point2f a2, Point2f b1, Point2f b2, Point2f& intersection)
{
    Point2f p = a1;
    Point2f q = b1;
    Point2f r(a2-a1);
    Point2f s(b2-b1);

    if(cross(r,s) == 0) {return false;}

    float t = cross(q-p,s)/cross(r,s);

    intersection = p + t*r;
    return true;
}

float cross(Point2f v1,Point2f v2)
{
    return v1.x*v2.y - v1.y*v2.x;
}

string string_format(const string fmt_str, ...) {
    int final_n, n = ((int)fmt_str.size()) * 2; /* Reserve two times as much as the length of the fmt_str */
    string str;
    unique_ptr<char[]> formatted;
    va_list ap;
    while(1) {
        formatted.reset(new char[n]); /* Wrap the plain char array into the unique_ptr */
        strcpy(&formatted[0], fmt_str.c_str());
        va_start(ap, fmt_str);
        final_n = vsnprintf(&formatted[0], n, fmt_str.c_str(), ap);
        va_end(ap);
        if (final_n < 0 || final_n >= n)
            n += abs(final_n - n + 1);
        else
            break;
    }
    return string(formatted.get());
}

}

