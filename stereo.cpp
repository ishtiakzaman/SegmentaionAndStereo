// Skeleton code for B657 A4 Part 3.
// D. Crandall
//
// Run like this, for example:
//   ./stereo part3/Aloe/view1.png part3/Aloe/view5.png part3/Aloe/gt.png
// and output files will appear in part3/Aloe
//
#include <vector>
#include <iostream>
#include <fstream>
#include <map>
#include <ctime>
#include <math.h>
#include <CImg.h>
#include <assert.h>

using namespace cimg_library;
using namespace std;

#define INF INFINITY

double sqr(double a) { return a*a; }

const string currentDateTime() 
{
    time_t now = time(0);
    struct tm tstruct;
    char buf[80];
    tstruct = *localtime(&now);        
    strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);

    return buf;
}

CImg<double> naive_stereo(const CImg<double> &input1, const CImg<double> &input2,
                                    CImg<double> &D, int window_size, int max_disp)
{  
    CImg<double> result(input1.width(), input1.height());

    for(int x = 0; x < input1.width(); x++)
    {
        for(int y = 0 ; y < input1.height(); y++)
        {
            double min_cost = INFINITY;
            int selected_label;

            for (int d = 0; d < max_disp; d++)
            {
                double cost = 0;
                
                for (int xx = x-window_size; xx <= x+window_size; ++xx)
                {
                    if (xx < 0 || xx+d >= input1.width())
                        continue;                    
                    for (int yy = y-window_size; yy <= y+window_size; ++yy)
                    {
                        if (yy < 0 || yy >= input1.height())
                            continue;
                        for (int p = 0; p < 3; ++p)
                            cost += sqr(input1(xx+d,yy,0,p) - input2(xx,yy,0,p));
                    }
                }

                cost = sqrt(cost);

                D(x,y,d) = cost;

                if(cost < min_cost)
                {
                    min_cost = cost;
                    selected_label = d;
                }
            }

            result(x, y) = selected_label;
        }
    }

    return result;
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

CImg<double> mrf_stereo(const CImg<double> &img, const CImg<double> &D, const double alpha, double max_disp)
{
    CImg<int> labels(img.width(), img.height(), 1, 1, 255);

    int n_labels = max_disp;
    const int n_neighbors = 4;
    CImg<double> message(img.width(), img.height(), n_labels, n_neighbors);
    CImg<double> prev_message(img.width(), img.height(), n_labels, n_neighbors, 0.0);
    
    int n_iteration = img.width() * img.height() * 5;
    while (n_iteration > 0)
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
                            double energy = D(x,y,k) + alpha * (k==l?0:1) +
                                                get_neighbors_messages_into_except(x,y,k,direction,prev_message);
                            if (energy < min_energy)
                                min_energy = energy;
                        }
                        message(x,y,l,direction) = min_energy;
                    }                    
                }    
                --n_iteration;            
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
            labels(x,y) = selected_label;
        }
    }

    return labels;
}



int main(int argc, char *argv[])
{
    if(argc != 4 && argc != 3)
    {
        cerr << "usage: " << argv[0] << " image_file1 image_file2 [gt_file]" << endl;
        return 1;
    }

    string input_filename1 = argv[1], input_filename2 = argv[2];
    string gt_filename;
    if(argc == 4)
    gt_filename = argv[3];

    // read in images and gt
    CImg<double> image1(input_filename1.c_str());
    CImg<double> image2(input_filename2.c_str());
    CImg<double> gt;

    double alpha = 100000;
    alpha = 1e3;
    double max_disp = 50;
    int size = 100;

    if(gt_filename != "")
    {
        gt = CImg<double>(gt_filename.c_str());

        // gt maps are scaled by a factor of 3, undo this...
        /*
        for(int i=0; i<gt.height(); i++)
            for(int j=0; j<gt.width(); j++)
                gt(j,i) = gt(j,i) / 3.0;
        */
    }

    
    //image1.resize(size, size, 1, 3);
    //image2.resize(size, size, 1, 3);
    //gt.resize(size, size, 1, 3);
    cout << currentDateTime() << endl;


    CImg<double> D(image1.width(), image1.height(), 256);

    // do naive stereo (matching only, no MRF)
    cout << "Computing naive disparity (might take a minute)" << endl;
    CImg<double> naive_disp = naive_stereo(image1, image2, D, 5, max_disp);
    //naive_disp.get_normalize(0,255).save((input_filename1 + "-disp_naive.png").c_str());
    naive_disp.get_normalize(0,255).save("disp_naive.png");
    cout << "Naive disparity image saved" << endl;

    cout << currentDateTime() << endl;

    // do stereo using mrf    
    cout << "Computing MRF disparity (might take around 5-6 minutes)" << endl;
    CImg<double> mrf_disp = mrf_stereo(image1, D, alpha, max_disp);
    //mrf_disp.get_normalize(0,255).save((input_filename1 + "-disp_mrf.png").c_str());
    mrf_disp.get_normalize(0,255).save("disp_mrf.png");   
    cout << "MRF disparity image saved" << endl; 

    // Measure error with respect to ground truth, if we have it...
    if(gt_filename != "")
    {
        cout << "Naive stereo technique mean error = " << (naive_disp-gt).sqr().sum()/gt.height()/gt.width() << endl;        
        cout << "MRF stereo technique mean error = " << (mrf_disp-gt).sqr().sum()/gt.height()/gt.width() << endl;

    }
    cout << currentDateTime() << endl;

    return 0;
}