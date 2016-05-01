// Skeleton code for B657 A4 Part 1.
// D. Crandall
//
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <map>
#include <list>
#include <utility>
#include <math.h>
#include <CImg.h>
#include <assert.h>

using namespace cimg_library;
using namespace std;

CImg<double> create_gaussian_filter(int size, double sigma)
{
    if (size % 2 == 0)
        size++;
    CImg<double> filter(size, size, 1, 1, 0.0);
    
    for(int i = -size/2; i <= size/2; i++)    
        for(int j = -size/2; j <= size/2; j++)                  
            filter(i+size/2,j+size/2) = 1.0/(2.0*cimg::PI*sigma*sigma)*exp(-1.0*(i*i+j*j)/(2*sigma*sigma));

    return filter;
}

void convolve_pixel(CImg<double> &image, CImg<double> &filter, int x, int y)
{
    for (int p = 0; p < image.spectrum(); ++p)
    {
        double pixel_value = 0.0;
        for (int i = x-filter.width()/2, fx = 0;  i <= x+filter.width()/2; ++i, ++fx)        
            for (int j = y-filter.height()/2, fy = 0;  j <= y+filter.height()/2; ++j, ++fy)                
                if (i >= 0 && j >= 0 && i < image.width() && j < image.height())   
                    pixel_value += image(i,j,0,p) * filter(fx,fy);
          
        if (pixel_value > 255)
            pixel_value = 255;
        image(x,y,0,p) = pixel_value;
    }
}

CImg<double> generate_stereo(string image_name, CImg<double> &original_image, CImg<double> &disp_image, double disp_rate)
{
    if (original_image.width() != disp_image.width() || original_image.height() != disp_image.height())
    {
        cout << "Original image dimension must be same as disparity image dimension" << endl;
        exit(0);
    }
    CImg<double> stereo_image(original_image.width(), original_image.height(), 1, 3, 0);
    CImg<double> transformed_image(original_image.width(), original_image.height(), 1, 3, 0);
    CImg<bool> has_pixel(original_image.width(), original_image.height(), 1, 1, false);
    vector<list<pair<int,int> > > pixels(256);

    // Convert to gray if color image
    if (disp_image.spectrum() != 1) 
        disp_image = disp_image.get_RGBtoYCbCr().get_channel(0);

    if (disp_rate < 0.0)
        disp_rate = -disp_rate;

    // store disparity image pixels into data structure
    // so that we can start rendering from the highest depth
    for (int i = 0; i < disp_image.width(); ++i)    
        for (int j = 0; j < disp_image.height(); ++j)        
            pixels[disp_image(i,j)].push_back(make_pair(i,j));  

    // Render start from highest depth to lowest depth
    for (int disparity = 0; disparity < 256; ++disparity)
    {
        for (list<pair<int,int> >::iterator it = pixels[disparity].begin(); it != pixels[disparity].end(); ++it)
        {
            int x = (*it).first;
            int y = (*it).second;
            int x_transformed = x+disparity*disp_rate;
            if (x_transformed >= 0 && x_transformed < transformed_image.width())
            {
                for (int p = 0; p < 3; ++p)                
                    transformed_image(x_transformed,y,0,p) = original_image(x,y,0,p);
                has_pixel(x_transformed,y) = true;
            }
        }
    }

    transformed_image.save((image_name+"-transformed.png").c_str());

    list<pair<int,int> > empty_pixels;
    for (int i = 0; i < transformed_image.width(); ++i)            
        for (int j = 0; j < transformed_image.height(); ++j)   
            if (has_pixel(i,j) == false)
                empty_pixels.push_back(make_pair(i,j));
               

    CImg<double> filter = create_gaussian_filter(15, 1);
    for (int iter = 0; iter < 25; ++iter)    
        for (list<pair<int,int> >::iterator it = empty_pixels.begin(); it != empty_pixels.end(); ++it)
            convolve_pixel(transformed_image, filter, (*it).first, (*it).second);               

    transformed_image.save((image_name+"-transformed_filtered.png").c_str());

    for (int i = 0; i < stereo_image.width(); ++i)    
    {
        for (int j = 0; j < stereo_image.height(); ++j)   
        {            
            stereo_image(i,j,0,0) = transformed_image(i,j,0,0);
            stereo_image(i,j,0,1) = original_image(i,j,0,1);
            stereo_image(i,j,0,2) = original_image(i,j,0,2);
        }
    }

    return stereo_image;
}

int main(int argc, char *argv[])
{
    if(argc < 3 || argc > 4)
    {
        cerr << "usage: " << argv[0] << " image_file disp_file (optional)disp_rate" << endl;
        return 1;
    }

    // read in images and gt
    CImg<double> original_image(argv[1]);
    CImg<double> disp_image(argv[2]);
    double disp_rate = 0.05;

    if (argc > 3)
    {
        istringstream iss(argv[3]);
        iss >> disp_rate;
    }

    CImg<double> stereo_image = generate_stereo(argv[1], original_image, disp_image, disp_rate);
    stereo_image.save(strcat(argv[1], "-stereogram.png"));

    return 0;
}