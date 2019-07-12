// Add Gaussian noise to Point Cloud
// ./Plane [sigma2(variance)] [ratio_for_add_noise]

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

#define  N_TOTAL      1e07
#define  TRUTH_MAX    0.99
#define  TRUTH_MIN    0.01

const char OUTPUT_NOISED_SPBR[] = "SPBR_DATA/box.spbr";

int main(int argc, char **argv) {
    if ( argc != 3 ) {
        std::cout << "USAGE: " << argv[0] << " [sigma2(variance)] [ratio_for_add_noise]" << std::endl;
        exit(1);
    }

    // FILE open
    std::ofstream   fout;
    std::string     of_name( OUTPUT_NOISED_SPBR ); 
    fout.open( of_name );

    float sigma2 = atof(argv[1]);
    std::cout << "Sigma2(Variance)"                 << std::endl;
    std::cout << "> " << sigma2 << "\n"             << std::endl;
    std::cout << "Sigma(Standard Deviation)"        << std::endl;
    std::cout << "> " << sqrtf(sigma2) << "\n"      << std::endl;
    std::cout << "Number of points"                 << std::endl;    
    std::cout << "> " << N_TOTAL  << "\n"           << std::endl;

    // variables
    kvs::MersenneTwister        uniRand;
    std::vector<kvs::Vector3d>  plane_points;
    kvs::Vector3d               plane_point;
    double x, y, z;
    x = y = z = 0.0;
    kvs::BoxMuller  gaussRand;
    kvs::Vector3d   point;
    float ratio_for_add_noise = atof(argv[2]);
    int noise_counter = 0;

    // Set SPBR header
    fout << "#/SPBR_ASCII_Data"      << std::endl;
    fout << "#/RepeatLevel 1"        << std::endl;
    fout << "#/CameraPosition 0 0 8" << std::endl;
    fout << "#/BGColorRGBByte 0 0 0" << std::endl;
    fout << "#/BoundingBox -0.05 -0.05 0 1.05 1.05 0" << std::endl;
    fout << "#/ImageResolution 1024" << std::endl;
    fout << "#/Shading 0"            << std::endl;
    fout << "#/LOD 0"                << std::endl;
    fout << "#/EndHeader"            << std::endl;

    // stochastically add Gaussian noise to box point cloud
    std::cout << "Adding Gaussian noise... (with " << ratio_for_add_noise*100 << "%)\n" << std::endl;

    // Generate points
    for ( int i = 0 ; i < N_TOTAL; i++ ) {
        x = uniRand(); // random number [0...1] for x
        y = uniRand(); // random number [0...1] for y
        z = 0.0;

        // Noise point (Red)
        if ( uniRand() < ratio_for_add_noise ) {
            // N(μ, σ^2)
            // N(0, 1) → μ=0, σ^2=1

            // Add Gaussian noise
            x += gaussRand.rand(0, sigma2);
            y += gaussRand.rand(0, sigma2);
            z += gaussRand.rand(0, sigma2);

            // Write to .spbr file 
            fout << x   << " " << y << " " << z << " ";
            fout << 0   << " " << 0 << " " << 0 << " ";
            fout << 255 << " " << 0 << " " << 0 << std::endl; // Red

            // Count number of noised points
            noise_counter++;

        // Not noise point (White)
        } else {
            fout << x   << " " << y << " " << z << " ";
            fout << 0   << " " << 0 << " " << 0 << " ";
            fout << 255 << " " << 255 << " " << 255 << std::endl; // White
        } // end if
    } // end for

    std::cout << "\nNumber of noised points"    << std::endl;
    std::cout << "> " << noise_counter << "\n"  << std::endl;  

    // FILE close
    fout.close();

    return 0;
}