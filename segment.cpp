// Skeleton code for B657 A4 Part 2.
// D. Crandall
//
//
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <map>
#include <math.h>
#include <CImg.h>
#include <assert.h>

using namespace cimg_library;
using namespace std;

#define INF 1e9

class Point
{
    public:
        Point() {}
        Point(int _col, int _row) : row(_row), col(_col) {}
        int row, col;
};

double get_gaussian_pdf(double x, double mu, double sigma)
{
    return 1.0/sqrt(2.0*cimg::PI*sigma*sigma)*exp(-1.0*(x-mu)*(x-mu)/(2.0*sigma*sigma));
}

vector<double> compute_mean(const CImg<double> &img, const vector<Point> &points)
{
    vector<double> mu(3, 0.0);
    for (int p = 0; p < 3; ++p)
    {
        for (int i = 0; i < points.size(); ++i)
        {
            int x = points[i].col;
            int y = points[i].row;            
            mu[p] += img(x, y, 0, p);
        }
        mu[p] /= points.size();
    }
    return mu;
}

vector<double> compute_variance(const CImg<double> &img, const vector<Point> &points, const vector<double> &mu)
{
    vector<double> sigma(3, 0.0);
    for (int p = 0; p < 3; ++p)
    {
        for (int i = 0; i < points.size(); ++i)
        {
            int x = points[i].col;
            int y = points[i].row;            
            sigma[p] += (img(x, y, 0, p) - mu[p]) * (img(x, y, 0, p) - mu[p]);
        }
        sigma[p] = sqrt(sigma[p] / points.size());        
    }
    return sigma;
}

CImg<int> naive_segment(const CImg<double> &img, const vector<Point> &fg,
                        const vector<Point> &bg, const double beta, CImg<double> &D)
{
    CImg<int> labels(img.width(), img.height(), 1, 1, -1);

    vector<double> mu = compute_mean(img, fg);
    vector<double> sigma = compute_variance(img, fg, mu);

    for (int i = 0; i < fg.size(); ++i)
    {
        labels(fg[i].col, fg[i].row) = 1;
        D(fg[i].col, fg[i].row, 0) = INF;
        D(fg[i].col, fg[i].row, 1) = 0;
    }
    for (int i = 0; i < bg.size(); ++i)
    {
        labels(bg[i].col, bg[i].row) = 0;
        D(bg[i].col, bg[i].row, 0) = 0;
        D(bg[i].col, bg[i].row, 1) = INF;
    }

    for (int i = 0; i < img.width(); i++)
    {
        for (int j = 0; j < img.height(); j++)
        {           
            if (labels(i,j) != -1)
                continue;

            double entropy = 1.0;
            for (int p = 0; p < 3; ++p)
                entropy *= get_gaussian_pdf(img(i,j,0,p), mu[p], sigma[p]);
            entropy = -1.0 * log(entropy) / log(2.0);
            labels(i,j) = entropy < beta? 1 : 0;

            D(i,j,0) = beta;
            D(i,j,1) = min(entropy, INF);
        }
    }

    return labels;
}

double get_neighbors_messages_into_except(int x, int y, int label, int skip_direction, const CImg<double> &message)
{
	// UP - 0, LEFT - 1, RIGHT - 2, DOWN - 3

	double value = 0.0;

	if (skip_direction != 0) // UP
	{
		if (y > 0)
			value += message(x, y-1, label, 3);
	}
	if (skip_direction != 1) // LEFT
	{
		if (x > 0)
			value += message(x-1, y, label, 2);
	}
	if (skip_direction != 2) // RIGHT
	{
		if (x < message.width()-1)
			value += message(x+1, y, label, 1);
	}
	if (skip_direction != 3) // DOWN
	{
		if (y < message.height()-1)
			value += message(x, y+1, label, 0);
	}
	return value;
}

CImg<int> mrf_segment(const CImg<double> &img, const CImg<double> &D, const double alpha)
{
    CImg<int> labels(img.width(), img.height(), 1, 1, 0);

	const int n_labels = 2;
	const int n_neighbors = 4;
    CImg<double> message(img.width(), img.height(), n_labels, n_neighbors);
    CImg<double> prev_message(img.width(), img.height(), n_labels, n_neighbors, 0.0);

    int max_iteration = max(img.width(), img.height());
    max_iteration = 5;
    for (int t = 0; t < max_iteration; ++t)
    {
        for (int x = 0; x < img.width(); x++)
        {
            for (int y = 0; y < img.height(); y++)
            {            	
            	for (int direction = 0; direction < 4; ++direction)
            	{
            		// passing message of label to direction
            		for (int l = 0; l < n_labels; ++l)      		
            		{            		
            			double min_energy = INF;
            			for (int k = 0; k < n_labels; ++k)
            			{
            				double energy = D(x,y,k) + alpha * abs(k-l) +
            									get_neighbors_messages_into_except(x,y,k,direction,prev_message);
            				if (energy < min_energy)
            					min_energy = energy;
            			}
            			message(x,y,l,direction) = min_energy;
            		}
            	}            	
            }
        }

        prev_message = message;
    }


	// Find the best label for each pixel
    for (int x = 0; x < img.width(); x++)
    {
        for (int y = 0; y < img.height(); y++)
        {
        	int selected_label = -1;
        	double min_energy = INF;
			for (int k = 0; k < n_labels; ++k)
			{
				double energy = D(x,y,k) + get_neighbors_messages_into_except(x,y,k,-1,message);
				if (energy < min_energy)
				{
					min_energy = energy;
					selected_label = k;
				}
			}
			labels(x,y)	= selected_label;
        }
    }

    return labels;
}

// Take in an input image and a binary segmentation map. Use the segmentation map to split the 
// input image into foreground and background portions, and then save each one as a separate image.
// also output a disparity map.
//
void output_segmentation(const CImg<double> &img, CImg<int> &labels, const string &fname)
{
    // sanity checks. If one of these asserts fails, you've given this function invalid arguments!
    assert(img.height() == labels.height());
    assert(img.width() == labels.width());

    CImg<double> img_fg = img, img_bg = img;

    for(int i=0; i<labels.height(); i++)
        for(int j=0; j<labels.width(); j++)
        {
            if(labels(j,i) == 0)
                img_fg(j,i,0,0) = img_fg(j,i,0,1) = img_fg(j,i,0,2) = 0;
            else if(labels(j,i) == 1)
            {
                img_bg(j,i,0,0) = img_bg(j,i,0,1) = img_bg(j,i,0,2) = 0;
                labels(j,i) = 255;
            }
            else
                assert(0);
        }


    labels.save((fname + "_disparity.png").c_str());
    //img_fg.get_normalize(0,255).save((fname + "_fg.png").c_str());
    //img_bg.get_normalize(0,255).save((fname + "_bg.png").c_str());
}

int main(int argc, char *argv[])
{
    if(argc < 3 || argc > 5)
    {
        cerr << "usage: " << argv[0] << " image_file seeds_file [optional:beta(default:25.0)]" << endl;
        return 1;
    }

    string input_filename1 = argv[1], input_filename2 = argv[2];

    // read in images and gt
    CImg<double> image_rgb(input_filename1.c_str());
    CImg<double> seeds_rgb(input_filename2.c_str());

    // figure out seed points 
    vector<Point> fg_pixels, bg_pixels;
    for(int i=0; i<seeds_rgb.height(); i++)
        for(int j=0; j<seeds_rgb.width(); j++)
        {
            // blue --> foreground
            if(max(seeds_rgb(j, i, 0, 0), seeds_rgb(j, i, 0, 1)) < 100 && seeds_rgb(j, i, 0, 2) > 100)
                fg_pixels.push_back(Point(j, i));

            // red --> background
            if(max(seeds_rgb(j, i, 0, 2), seeds_rgb(j, i, 0, 1)) < 100 && seeds_rgb(j, i, 0, 0) > 100)
                bg_pixels.push_back(Point(j, i));
        }

    double beta = 25;
    double alpha = 10;

    if (argc >= 4)
    {
        istringstream iss(argv[3]);
        iss >> beta;
    }

    if (argc > 4)
    {
        istringstream iss(argv[4]);
        iss >> alpha;
    }

    CImg<double> D(image_rgb.width(), image_rgb.height(), 2);
    CImg<int> labels = naive_segment(image_rgb, fg_pixels, bg_pixels, beta, D);
    //output_segmentation(image_rgb, labels, input_filename1 + "-naive_segment_result");
    output_segmentation(image_rgb, labels, "segment.png");

    // do mrf segmentation
    labels = mrf_segment(image_rgb, D, alpha);    
    //output_segmentation(image_rgb, labels, input_filename1 + "-mrf_segment_result");
    output_segmentation(image_rgb, labels, "segment2.png");

    return 0;
}
