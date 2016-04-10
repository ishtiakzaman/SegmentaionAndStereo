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

bool find_point(int x, int y, vector<Point> points)
{
    for (int i = 0; i < points.size(); ++i)
        if (points[i].col == x && points[i].row == y)
            return true;
    return false;
}

CImg<int> naive_segment(const CImg<double> &img, const vector<Point> &fg, const vector<Point> &bg, const double beta)
{
    // implement this in step 2...
    //  this placeholder just returns a random disparity map
    CImg<int> labels(img.width(), img.height(), 1, 1, -1);

    vector<double> mu = compute_mean(img, fg);
    vector<double> sigma = compute_variance(img, fg, mu);

    for (int i = 0; i < fg.size(); ++i)
        labels(fg[i].col, fg[i].row) = 1;
    for (int i = 0; i < bg.size(); ++i)
        labels(bg[i].col, bg[i].row) = 0;

    for (int i = 0; i < img.width(); i++)
    {
        for (int j = 0; j < img.height(); j++)
        {           
            if (labels(i,j) != -1)
                continue;

            double alpha = 1.0;
            for (int p = 0; p < 3; ++p)
                alpha *= get_gaussian_pdf(img(i,j,0,p), mu[p], sigma[p]);
            alpha = -1.0 * log(alpha) / log(2);            
            labels(i,j) = alpha < beta? 1 : 0;            
        }
    }

    return labels;
}

CImg<int> mrf_segment(const CImg<double> &img, const vector<Point> &fg, const vector<Point> &bg)
{
    // implement this in step 3...
    //  this placeholder just returns a random disparity map by calling naive_segment
    return naive_segment(img, fg, bg, 25);
}

// Take in an input image and a binary segmentation map. Use the segmentation map to split the 
//  input image into foreground and background portions, and then save each one as a separate image.
//
// You'll just need to modify this to additionally output a disparity map.
//
void output_segmentation(const CImg<double> &img, const CImg<int> &labels, const string &fname)
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
                img_bg(j,i,0,0) = img_bg(j,i,0,1) = img_bg(j,i,0,2) = 0;
            else
                assert(0);
        }

    img_fg.get_normalize(0,255).save((fname + "_fg.png").c_str());
    img_bg.get_normalize(0,255).save((fname + "_bg.png").c_str());
}

int main(int argc, char *argv[])
{
    if(argc < 3 || argc > 4)
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

    // do naive segmentation
    double beta = 25;
    if (argc == 4)
    {
        istringstream iss(argv[3]);
        iss >> beta;
    }
    CImg<int> labels = naive_segment(image_rgb, fg_pixels, bg_pixels, beta);
    //output_segmentation(image_rgb, labels, input_filename1 + "-naive_segment_result");
    output_segmentation(image_rgb, labels, "segment.png");

    // do mrf segmentation
    //labels = naive_segment(image_rgb, fg_pixels, bg_pixels);
    //output_segmentation(image_rgb, labels, input_filename1 + "-mrf_segment_result");

    return 0;
}
