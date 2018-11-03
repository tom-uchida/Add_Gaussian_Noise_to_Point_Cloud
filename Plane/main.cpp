// 点群にガウシアンノイズを付与するプログラム
// ./Plane [ratio_for_sigma(S.D.)] [ratio_for_add_noise]

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

#define  N_TOTAL      10000000
#define  TRUTH_MAX    0.99
#define  TRUTH_MIN    0.01

const char OUTPUT_SPBR[]        = "SPBR_DATA/plane.spbr";
const char OUTPUT_NOISED_SPBR[] = "SPBR_DATA/plane_noised.spbr";
const char OUTPUT_SPBR_HEADER[] = "SPBR_DATA/h_plane.spbr";

int main(int argc, char **argv) {
    if ( argc != 3 ) {
        std::cout << "USAGE: " << argv[0] << " [ratio_for_sigma(S.D.)] [ratio_for_add_noise]" << std::endl;
        exit(1);
    }



    // ----------------
    // ----- FILE -----
    // ----------------
    // raw point cloud (.spbr)
    std::ofstream   fout_raw;
    std::string     of_name_raw( OUTPUT_SPBR );
    fout_raw.open( of_name_raw );

    // noised point cloud (.spbr)
    std::ofstream   fout_noised;
    std::string     of_name_noised( OUTPUT_NOISED_SPBR ); 
    fout_noised.open( of_name_noised );

    // noise distribution (.csv)
    std::ofstream   fout_gauss_distribution("noise_distribution.csv");



    // ----------------------------------------
    // ----- Generate raw box point cloud -----
    // ----------------------------------------
    // variables
    kvs::MersenneTwister        uniRand;
    std::vector<kvs::Vector3d>  plane_points;
    kvs::Vector3d               plane_point;
    double x, y, z;

    // config. for SPBR header
    fout_raw << "#/SPBR_ASCII_Data"         << std::endl;
    fout_raw << "#/RepeatLevel 1"           << std::endl;
    fout_raw << "#/CameraPosition 0 0 8"    << std::endl;
    fout_raw << "#/BGColorRGBByte 0 0 0"    << std::endl;
    fout_raw << "#/ColorRGB 255 255 255"    << std::endl;
    fout_raw << "#/ImageResolution 1024"     << std::endl;
    fout_raw << "#/Shading 0"               << std::endl;
    fout_raw << "#/LOD 0"                   << std::endl;
    fout_raw << "#/EndHeader"               << std::endl;

    // Generate Plane Point Cloud with unirand
    for ( int i = 0 ; i < N_TOTAL; i++ ) {
        x = uniRand(); // random number [0...1] for x
        y = uniRand(); // random number [0...1] for y
        z = 0.0;
        if( x < TRUTH_MAX && x > TRUTH_MIN && y < TRUTH_MAX && y > TRUTH_MIN ){
            fout_raw << x << " " << y << " " << z << " ";
            fout_raw << 0 << " " << 0 << " " << 0 << " ";
            fout_raw << 255 << " " << 255 << " " << 255 << std::endl;

            plane_point.set(x, y, z);
            plane_points.push_back( plane_point );
        
        } else {
            fout_raw << x << " " << y << " " << z << " ";
            fout_raw << 0 << " " << 0 << " " << 0 << " ";
            fout_raw << 255 << " " << 255 << " " << 255 << std::endl;

            plane_point.set(x, y, z);
            plane_points.push_back( plane_point );
        }
    }
    
    // standard output
    float ratio_for_sigma   = atof(argv[1]);
    float diagonal_length   = sqrtf(1.0*1.0 + 1.0*1.0);
    float sigma2            = diagonal_length*ratio_for_sigma;
    std::cout << "Diagonal length"                  << std::endl;
    std::cout << "> " << diagonal_length << "\n"    << std::endl;
    std::cout << "Sigma2(Variance)"                 << std::endl;
    std::cout << "> " << sigma2 << "\n"             << std::endl;
    std::cout << "Sigma(Standard Deviation)"        << std::endl;
    std::cout << "> " << sqrtf(sigma2) << "\n"      << std::endl;
    std::cout << "Number of points \"plane.spbr\""    << std::endl;    
    std::cout << "> " << plane_points.size()  << "\n" << std::endl;



    // -------------------------------------------------
    // ----- Add Gaussian noise to box point cloud -----
    // -------------------------------------------------
    // variables
    kvs::BoxMuller  gaussRand;
    kvs::Vector3d   point;
    float ratio_for_add_noise = atof(argv[2]);
    x = y = z = 0.0;
    int noise_counter = 0;

    // config. for SPBR header
    fout_noised << "#/SPBR_ASCII_Data"      << std::endl;
    fout_noised << "#/RepeatLevel 1"        << std::endl;
    fout_noised << "#/CameraPosition 0 0 8" << std::endl;
    fout_noised << "#/BGColorRGBByte 0 0 0" << std::endl;
    fout_noised << "#/ColorRGB 255 0 0"     << std::endl;
    fout_noised << "#/ImageResolution 1024"  << std::endl;
    fout_noised << "#/Shading 0"            << std::endl;
    fout_noised << "#/LOD 0"                << std::endl;
    fout_noised << "#/EndHeader"            << std::endl;

    // stochastically add Gaussian noise to box point cloud
    std::cout << "\n\n----- Stochastically add Gaussian noise -----" << std::endl;
    std::cout << "> Add Gaussian noise with " << ratio_for_add_noise*100 << " percent.\n" << std::endl;
    for (int i = 0; i < plane_points.size(); i++) {
        x = plane_points[i].x();
        y = plane_points[i].y();
        z = plane_points[i].z();       

        bool isNoise = false;
        if ( uniRand() < ratio_for_add_noise ) {
            // N(μ, σ^2)
            // N(0, 1) → μ=0, σ^2=1

            // add Gaussian noise
            x += gaussRand.rand(0, sigma2);
            y += gaussRand.rand(0, sigma2);
            z += gaussRand.rand(0, sigma2);

            // write to .csv file
            //fout_gauss_distribution << i << "," << gaussRand.rand(0, sigma2) << std::endl;

            // count number of noised points
            noise_counter++;
            isNoise = true;
        }

        // write to .spbr file
        point.set(x, y, z);
        if (isNoise) fout_noised << point << " 0 0 0 255 0 0" << std::endl;
        else         fout_noised << point << " 0 0 0 255 255 255" << std::endl;
    }
    std::cout << "\nNumber of noised points"    << std::endl;
    std::cout << "> " << noise_counter << "\n"  << std::endl;  



    // --------------------------------
    // ----- Gaussian Integration -----
    // --------------------------------
    // std::cout << "\nGaussian Integration Value" << std::endl;
    // std::cout << "> " << sqrtf(2*M_PI) * sqrtf(sigma2) << "\n" << std::endl;

    // std::cout << "\nGaussian Integration Value" << std::endl;
    // std::cout << "> " << 1/sqrtf(2*M_PI*sigma2) * sqrtf(2*M_PI) * sqrtf(sigma2) << "\n" << std::endl;



    // ----------------------
    // ----- Exec. SPBR -----
    // ----------------------
    std::string of_name_header( OUTPUT_SPBR_HEADER ); 
    std::string EXEC("./spbr ");
    EXEC += of_name_header;
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