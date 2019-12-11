#include "main.hpp"
using namespace cv;
using namespace std;


vector<Mat> importar()
{
    char file_name[100];

    //Importação das imagens Externas - Fotos  com drone
    vector <Mat> Externa; // Max até a 70
    for (int n = 1; n <= 2; n++)
    {
        sprintf(file_name,"/home/grin/B9/Panoramic_and_Dome/Externa/%d.JPG",n);
        Mat Cima = imread(file_name, CV_LOAD_IMAGE_COLOR); Externa.push_back(Cima);

    }

    //----Importação das imagens da Baia da Amanda - Fotos  com drone----
    //    vector <Mat>imageAT2; //Erro na 62
    //    for (int n = 1; n <=10; n++)
    //    {
    //        if (n == 62){}
    //        else
    //        {
    //            sprintf(file_name,"/home/grin/B9/Panoramic_and_Dome/DomoAT_2.0/%d.JPG",n);
    //            Mat Cima = imread(file_name, CV_LOAD_IMAGE_COLOR); imageAT2.push_back(Cima);
    //        }
    //    }

    //----Importação das imagens da Baia da MAria Julia - Fotos  com drone----

    //    vector <Mat>imageMJ2; //Erro na 40,41,53
    //    for (int n = 1; n <= 60; n++)
    //    {
    //        if (n == 40 || n== 41 || n == 53){}
    //        else{
    //            sprintf(file_name,"/home/grin/B9/Panoramic_and_Dome/ProjetoGrin/DomoMJ_2.0/%d.JPG",n);
    //            Mat Cima = imread(file_name, CV_LOAD_IMAGE_COLOR); imageMJ2.push_back(Cima);
    //        }
    //    }

   
   return Externa;
}
