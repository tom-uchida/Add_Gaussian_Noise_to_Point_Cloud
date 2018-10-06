// 点群にガウシアンノイズを付与するプログラム
// ./addGaussianNoise [diff] [ratio_for_sigma(S.D.)] [ratio_for_add_noise]


// 実行例
// ./addGaussianNoise 0.01 0.01 0.2

// Diagonal length
// > 1.73205

// Sigma2(variance)
// > 0.0173205

// Sigma(standard deviation)
// > 0.131607

// Number of points "box.spbr"
// > 1030301

// ----- Stochastically add Gaussian noise -----
// > Add Gaussian noise with 20 percent.

// Number of noised points
// > 205565

#include <kvs/BoxMuller>
#include <kvs/MersenneTwister>
#include <kvs/Vector3>
#include <kvs/ColorMap>
#include <iostream>
#include <fstream>
#include <cstring> 
#include <cstdlib>
#include <cmath>
#include <vector>
#include <math.h>

// Default output file name
const char OUTPUT_SPBR[]        = "/SPBR_DATA/box";
const char OUTPUT_NOISED_SPBR[] = "/SPBR_DATA/box_noised";

int main(int argc, char **argv) {
    if ( argc != 4 ) {
        std::cout << "USAGE: " << argv[0] << " [diff] [ratio_for_sigma(S.D.)] [ratio_for_add_noise]" << std::endl;
        exit(1);
    }



    // ----------------
    // ----- FILE -----
    // ----------------
    // raw point cloud (.spbr)
    std::ofstream   fout_raw;
    std::string     of_name_raw( OUTPUT_SPBR );
    of_name_raw     += "_";
    of_name_raw     += argv[1];
    of_name_raw     += ".spbr";
    fout_raw.open( of_name_raw );

    // noised point cloud (.spbr)
    std::ofstream   fout_noised;
    std::string     of_name_noised( OUTPUT_NOISED_SPBR );
    of_name_noised  += "_";
    of_name_noised  += argv[2];
    of_name_noised  += "_";
    of_name_noised  += argv[3];
    of_name_noised  += ".spbr";     
    fout_noised.open( of_name_noised );

    // noise distribution (.csv)
    std::ofstream fout_gauss_distribution("noise_distribution.csv");



    // ----------------------------------------
    // ----- Generate raw box point cloud -----
    // ----------------------------------------
    // variables
    std::vector<kvs::Vector3d>  box_points;
    kvs::Vector3d               box_point;
    float diff = atof(argv[1]);

    // config. for SPBR header
    fout_raw << "#/SPBR_ASCII_Data"         << std::endl;
    fout_raw << "#/RepeatLevel 1"           << std::endl;
    fout_raw << "#/BGColorRGBByte 0 0 0"    << std::endl;
    fout_raw << "#/ColorRGB 255 255 255"    << std::endl;
    fout_raw << "#/ImageResolution 1440"    << std::endl;
    fout_raw << "#/LOD 0"                   << std::endl;
    fout_raw << "#/EndHeader"               << std::endl;
    for (float diff_z = 0.0; diff_z < 1.0; diff_z += diff ) {
        for (float diff_y = 0.0; diff_y < 1.0; diff_y += diff ) {
            for (float diff_x = 0.0; diff_x < 1.0; diff_x += diff ) {
                box_point.set(diff_x, diff_y, diff_z);
                box_points.push_back( box_point );

                // write to spbr file
                fout_raw << box_point << std::endl;
            }
        }
    }
    
    // standard output
    float ratio_for_sigma   = atof(argv[2]);
    float diagonal_length   = sqrtf(1.0*1.0 + 1.0*1.0 + 1.0*1.0);
    float sigma2            = diagonal_length*ratio_for_sigma;
    std::cout << "Diagonal length"                  << std::endl;
    std::cout << "> " << diagonal_length << "\n"    << std::endl;
    std::cout << "Sigma2(Variance)"                 << std::endl;
    std::cout << "> " << sigma2 << "\n"             << std::endl;
    std::cout << "Sigma(Standard Deviation)"        << std::endl;
    std::cout << "> " << sqrtf(sigma2) << "\n"      << std::endl;
    std::cout << "Number of points \"box.spbr\""    << std::endl;    
    std::cout << "> " << box_points.size()  << "\n" << std::endl;



    // -------------------------------------------------
    // ----- Add Gaussian noise to box point cloud -----
    // -------------------------------------------------
    // variables
    kvs::BoxMuller          gaussRand;
    kvs::MersenneTwister    uniRand;
    kvs::Vector3d           point;
    float ratio_for_add_noise = atof(argv[3]);
    float x, y, z;
    int noise_counter = 0;

    // config. for SPBR header
    fout_noised << "#/SPBR_ASCII_Data"      << std::endl;
    fout_noised << "#/RepeatLevel 1"        << std::endl;
    fout_noised << "#/BGColorRGBByte 0 0 0" << std::endl;
    fout_noised << "#/ColorRGB 255 0 0"     << std::endl;
    fout_noised << "#/ImageResolution 1440" << std::endl;
    fout_noised << "#/LOD 0"                << std::endl;
    fout_noised << "#/EndHeader"            << std::endl;

    // stochastically add Gaussian noise to box point cloud
    std::cout << "\n\n----- Stochastically add Gaussian noise -----" << std::endl;
    std::cout << "> Add Gaussian noise with " << ratio_for_add_noise*100 << " percent.\n" << std::endl;
    for (int i = 0; i < box_points.size(); i++) {

        if ( uniRand() < ratio_for_add_noise ) {
            // N(μ, σ^2)
            // N(0, 1) → μ=0, σ^2=1

            // add Gaussian noise
            x = box_points[i].x() + gaussRand.rand(0, sigma2);
            y = box_points[i].y() + gaussRand.rand(0, sigma2);
            z = box_points[i].z() + gaussRand.rand(0, sigma2);
            
            // write to .spbr file
            point.set(x, y, z);
            fout_noised << point << std::endl;

            // write to .csv file
            fout_gauss_distribution << i << "," << gaussRand.rand(0, sigma2) << std::endl;

            // count number of noised points
            noise_counter++;
        }

    }
    std::cout << "\nNumber of noised points"    << std::endl;
    std::cout << "> " << noise_counter << "\n"  << std::endl;  



    // --------------------------------
    // ----- Gaussian Integration -----
    // --------------------------------
    std::cout << "\nGaussian Integration Value" << std::endl;
    std::cout << "> " << sqrtf(2*M_PI) * sqrtf(sigma2) << "\n" << std::endl;
    std::cout << "\nGaussian Integration Value" << std::endl;
    std::cout << "> " << 1/sqrtf(2*M_PI*sigma2) * sqrtf(2*M_PI) * sqrtf(sigma2) << "\n" << std::endl;



    // ----------------------
    // ----- Exec. SPBR -----
    // ----------------------
    std::string EXEC("./spbr ");
    EXEC += of_name_raw;
    EXEC += " ";
    EXEC += of_name_noised;
    system( EXEC.c_str() );



    // ----------------------
    // ----- FILE close -----
    // ----------------------
    fout_raw.close();
    fout_noised.close();
    fout_gauss_distribution.close();



    return 0;
}