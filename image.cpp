//______________________________________________________________________________________
// Program : OpenCV based QR code Detection and Retrieval
// Author  : Bharath Prabhuswamy
//______________________________________________________________________________________

#include "constants.h"

#include <opencv2/opencv.hpp>
#include <opencv2/videostab/videostab.hpp>
#include <iostream>
#include <cmath>

#include "Circle.h"

using namespace cv;
using namespace std;
using namespace jbc;

// Start of Main Loop
//------------------------------------------------------------------------------------------------------------------------
int main ( int argc, char **argv )
{


	bool looping = true;

	Mat backup = imread(argv[1]);

	if(backup.empty()){ cerr << "ERR: Unable to find image.\n" << endl;
		return -1;
	}


	// Creation of Intermediate 'Image' Objects required later
	Mat gray(backup.size(), CV_MAKETYPE(backup.depth(), 1));			// To hold Grayscale backup
	Mat edges(backup.size(), CV_MAKETYPE(backup.depth(), 1));			// To hold Grayscale backup
	Mat traces(backup.size(), CV_8UC3);								// For Debug Visuals

	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;

	float areat,arear,areab, large, padding;

	int key = 0;
	while(looping && key != 'q')				// While loop to query for Image Input frame
	{
		Mat image = backup.clone();

		traces = Scalar(0,0,0);

		// capture >> image;				// For Video input		// Capture Image from Image Input

		cvtColor(image,gray,CV_RGB2GRAY);		// Convert Image captured from Image Input to GrayScale
		Canny(gray, edges, 100 , 200, 3);		// Apply Canny edge detection on the gray image


		findContours( edges, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE); // Find contours with hierarchy

		// Start processing the contour Circle

		// Find Three repeatedly enclosed contours A,B,C
		// NOTE: 1. Contour enclosing other contours is assumed to be the three Alignment markings of the QR code.
		// 2. Alternately, the Ratio of areas of the "concentric" squares can also be used for identifying base Alignment markers.
		// The below demonstrates the first method

		vector<Contour*> dots(0);

		for( size_t i = 0; i < contours.size(); i++ )
		{
			int k=i;
			int c=0;

			while(hierarchy[k][2] != -1)
			{
				k = hierarchy[k][2] ;
				c = c+1;
			}
			if(hierarchy[k][2] != -1)
			c = c+1;

			if (c >= 5)
			{
				dots.push_back(new Contour(&contours, i));
			}
		}

		vector<Circle*> permutations(0);

		size_t a = 0;
		size_t b = 1;
		size_t c = 2;
		size_t size = dots.size();

		if (size == 0) {
			return 0;
		}

		while (true) {
			permutations.push_back(new Circle(dots.at(a), dots.at(b), dots.at(c)));
			c++;
			if (c == size) {
				b++;
				c = b + 1;
			}
			if (c == size) {
				a++;
				b = a + 1;
				c = a + 2;
			}
			if (c == size) {
				break;
			}
		}

		size = permutations.size();

		for(a = 0; a < size; a++)		// Ensure we have (atleast 3; namely A,B,C) 'Alignment Markers' discovered
		{
			Circle* circle = permutations.at(a);

			if (!circle->IsValid()) {
				continue;
			}
			// We have found the 3 markers for the QR code; Now we need to determine which of them are 'top', 'right' and 'bottom' markers

			// To ensure any unintended values do not sneak up when QR code is not present
			if(contourArea(*circle->GetTop()->GetContour()) > 10 && contourArea(*circle->GetRight()->GetContour()) > 10 && contourArea(*circle->GetBottom()->GetContour()) > 10 )
			{

				circle->Draw(image, traces, hierarchy, a);
			}
		}

		if (DBG == 1) {
			imshow ( "Image", image );
			imshow ( "Traces", traces );
		}

	if (LOOPING == 1) {
			key = waitKey(500000);	// OPENCV: wait for 1ms before accessing next frame
		}
		else {
			looping = false;
		}

	}	// End of 'while' loop

	return 0;
}

// End of Main Loop
//--------------------------------------------------------------------------------------


// Routines used in Main loops


// EOF
