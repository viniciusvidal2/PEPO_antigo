#include "main.hpp"
#include <cstring>
#include "importar.cpp"
using namespace cv;
using namespace std;

int main()
{ 
    //importar imagens
    vector <Mat>Externa;
    Externa = importar();

    //caminho  onde serão guardadis os resultados da panoramica
    string result_name = "/home/grin/B9/Panoramic_and_Dome/Externa/result.JPG";


    Mat panoramica;
    //stitching
    Stitcher stitcher = Stitcher::createDefault();
    stitcher.stitch(Externa, panoramica);
    //stitcher.stitch(imageAT, panoramica);
    //stitcher.stitch(imageMJ2, panoramica);
    //stitcher.stitch(imageAT2, panoramica);
    //stitcher.stitch(imageMJ, panoramica);
    //stitcher.stitch(imageAT, panoramica);

    //escrever o resultado da panoramica
    imwrite(result_name, panoramica);

    //mostrar na tela o resultado da panoramica
    namedWindow( "Display window", WINDOW_NORMAL );     // WINDOW_NORMAL permite que a janela ajuste o seu tamanho.
    imshow( "Display window", panoramica);             // Show our image inside it.
    waitKey(0);                                         // Wait for a keystroke in the window
    return 0;
    
}
