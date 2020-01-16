#include "main.hpp"
#include <cstring>
#include "importar.cpp"
using namespace cv;
using namespace std;
using namespace cv::xfeatures2d;

int main()
{ 
    //    //importar imagens
    //    vector <Mat>Externa;
    //    Externa = importar();

    //    //caminho  onde serão guardadis os resultados da panoramica
    //    string result_name = "/home/grin/B9/Panoramic_and_Dome/Externa/result.JPG";


    //    Mat panoramica;
    //    //stitching
    //    Stitcher stitcher = Stitcher::createDefault();
    //    stitcher.stitch(Externa, panoramica);
    //    //stitcher.stitch(imageAT, panoramica);
    //    //stitcher.stitch(imageMJ2, panoramica);
    //    //stitcher.stitch(imageAT2, panoramica);
    //    //stitcher.stitch(imageMJ, panoramica);
    //    //stitcher.stitch(imageAT, panoramica);

    //    //escrever o resultado da panoramica
    //    imwrite(result_name, panoramica);

    //    //mostrar na tela o resultado da panoramica
    //    namedWindow( "Display window", WINDOW_NORMAL );     // WINDOW_NORMAL permite que a janela ajuste o seu tamanho.
    //    imshow( "Display window", panoramica);             // Show our image inside it.
    //    waitKey(0);                                         // Wait for a keystroke in the window
    //    return 0;
    //    ---------------------------------------------------------------//
    //Criando panoramica por meio da Homografia

    Mat image1= imread("/home/grin/B9/Panoramic_and_Dome/Externa/31.JPG",CV_LOAD_IMAGE_COLOR);
    Mat image2= imread("/home/grin/B9/Panoramic_and_Dome/Externa/result.JPG",CV_LOAD_IMAGE_COLOR);
    Mat gray_image1;
    Mat gray_image2;

    // Convert to Grayscale
    cvtColor( image1, gray_image1, CV_RGB2GRAY );
    cvtColor( image2, gray_image2, CV_RGB2GRAY );

    if( !gray_image1.data || !gray_image2.data )
    { std::cout<< " --(!) Error reading images " << std::endl; return -1; }

    //-- Step 1: Detect the keypoints using SURF Detector
    int minHessian = 400;

    Ptr<SURF> detector = SURF::create( minHessian );

    std::vector< KeyPoint > keypoints_object, keypoints_scene;

    detector->detect( gray_image1, keypoints_object );
    detector->detect( gray_image2, keypoints_scene );

    //-- Step 2: Calculate descriptors (feature vectors)
    Ptr<SURF> extractor = SURF::create();

    Mat descriptors_object, descriptors_scene;

    extractor->compute( gray_image1, keypoints_object, descriptors_object );
    extractor->compute( gray_image2, keypoints_scene, descriptors_scene );

    //-- Step 3: Matching descriptor vectors using FLANN matcher
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);

    std::vector< DMatch > matches;
    matcher->match( descriptors_object, descriptors_scene, matches );

    double max_dist = 0; double min_dist = 100;

    //-- Quick calculation of max and min distances between keypoints
    for( int i = 0; i < descriptors_object.rows; i++ )
    {
        double dist = matches[i].distance;
        if( dist < min_dist ) min_dist = dist;
        if( dist > max_dist ) max_dist = dist;
    }

    printf("-- Max dist : %f \n", max_dist );
    printf("-- Min dist : %f \n", min_dist );

    //-- Use only "good" matches
    std::vector< DMatch > good_matches;

    for( int i = 0; i < descriptors_object.rows; i++ )
    { if( matches[i].distance < 3*min_dist )
        { good_matches.push_back( matches[i]); }
    }
    //-- Filter matches using the Lowe's ratio test
    std::vector< std::vector<DMatch> > knn_matches;
    matcher->knnMatch( descriptors_object, descriptors_scene, knn_matches, 2 );
    const float ratio_thresh = 0.75f;
    for (size_t i = 0; i < knn_matches.size(); i++)
    {
        if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
        {
            good_matches.push_back(knn_matches[i][0]);
        }
    }
    std::vector< Point2f > obj;
    std::vector< Point2f > scene;

    for( int i = 0; i < good_matches.size(); i++ )
    {
        //-- Get the keypoints from the good matches
        obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
        scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
    }

    // Find the Homography Matrix
    Mat H = findHomography( obj, scene, CV_RANSAC );
    // Use the Homography Matrix to warp the images
    cv::Mat result;
    warpPerspective(image1,result,H,cv::Size(image1.cols+image2.cols,image1.rows));
    cv::Mat half(result,cv::Rect(0,0,image2.cols,image2.rows));
    image2.copyTo(half);
    //     imshow( "Result", result );
    string resul = "/home/grin/B9/Panoramic_and_Dome/Externa/result1.JPG";
    imwrite(resul, result);
    //     waitKey(0);
    //     return 0;


}
