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

    //     Load the images
    Mat img_object= imread("/home/grin/B9/Panoramic_and_Dome/Externa/result27-38.JPG",CV_LOAD_IMAGE_COLOR);
    Mat img_scene= imread("/home/grin/B9/Panoramic_and_Dome/Externa/result29-30.JPG" ,CV_LOAD_IMAGE_COLOR);
    Mat gray_image1;
    Mat gray_image2;
    // Convert to Grayscale
    cvtColor( img_object, gray_image1, CV_RGB2GRAY );
    cvtColor( img_scene, gray_image2, CV_RGB2GRAY );
    //    namedWindow( " window", WINDOW_NORMAL );
    //    imshow(" window",img_scene);
    //    namedWindow( "Display window", WINDOW_NORMAL );
    //    imshow("Display window",img_object);

    if( !gray_image1.data || !gray_image2.data )
    { std::cout<< " --(!) Error reading images " << std::endl; return -1; }

    //-- Step 1: Detect the keypoints using SURF Detector, compute the descriptors
    int minHessian = 400;
    Ptr<SURF> detector = SURF::create( minHessian );
    std::vector<KeyPoint> keypoints_object, keypoints_scene;
    Mat descriptors_object, descriptors_scene;
    detector->detectAndCompute( img_scene, noArray(), keypoints_object, descriptors_object );
    detector->detectAndCompute( img_scene, noArray(), keypoints_scene, descriptors_scene );


    //-- Step 2: Calculate descriptors (feature vectors)
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
    std::vector< std::vector<DMatch> > knn_matches;
    matcher->knnMatch( descriptors_object, descriptors_scene, knn_matches, 2 );

    //-- Filter matches using the Lowe's ratio test
    const float ratio_thresh = 0.75f;
    std::vector<DMatch> good_matches;
    for (size_t i = 0; i < knn_matches.size(); i++)
    {
        if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
        {
            good_matches.push_back(knn_matches[i][0]);
        }
    }
    //-- Draw matches
    Mat img_matches;
    drawMatches( img_object, keypoints_object, img_scene, keypoints_scene, good_matches, img_matches, Scalar::all(-1),
                 Scalar::all(-1), std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
    //-- Localize the object
    std::vector<Point2f> obj;
    std::vector<Point2f> scene;
    for( size_t i = 0; i < good_matches.size(); i++ )
    {
        //-- Get the keypoints from the good matches
        obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
        scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
    }
    Mat H = findHomography( obj, scene, CV_RANSAC );
    //-- Get the corners from the image_1 ( the object to be "detected" )
    std::vector<Point2f> obj_corners(4);
    obj_corners[0] = Point2f(0, 0);
    obj_corners[1] = Point2f( (float)img_object.cols, 0 );
    obj_corners[2] = Point2f( (float)img_object.cols, (float)img_object.rows );
    obj_corners[3] = Point2f( 0, (float)img_object.rows );
    std::vector<Point2f> scene_corners(4);
    perspectiveTransform( obj_corners, scene_corners, H);
    //-- Draw lines between the corners (the mapped object in the scene - image_2 )
    line( img_matches, scene_corners[0] + Point2f((float)img_object.cols, 0),
            scene_corners[1] + Point2f((float)img_object.cols, 0), Scalar(0, 255, 0), 4 );
    line( img_matches, scene_corners[1] + Point2f((float)img_object.cols, 0),
            scene_corners[2] + Point2f((float)img_object.cols, 0), Scalar( 0, 255, 0), 4 );
    line( img_matches, scene_corners[2] + Point2f((float)img_object.cols, 0),
            scene_corners[3] + Point2f((float)img_object.cols, 0), Scalar( 0, 255, 0), 4 );
    line( img_matches, scene_corners[3] + Point2f((float)img_object.cols, 0),
            scene_corners[0] + Point2f((float)img_object.cols, 0), Scalar( 0, 255, 0), 4 );
    //-- Show detected matches
    //             namedWindow( "Good Matches & Object detection", WINDOW_NORMAL );
    //            imshow("Good Matches & Object detection", img_matches );

    string result_name = "/home/grin/B9/Panoramic_and_Dome/Externa/match.JPG";
    imwrite(result_name, img_matches);

    // Use the Homography Matrix to warp the images
    cv::Mat result;
    //            warpPerspective(img_scene,result,H,cv::Size(2*img_scene.cols,2*img_scene.rows));

    warpPerspective(img_scene,result,H,img_scene.size());


    //        cv::Mat half(result,cv::Rect(0,0,img_object.cols,img_object.rows));
    //        img_object.copyTo(half);
    string resul = "/home/grin/B9/Panoramic_and_Dome/Externa/result_final.JPG";
    imwrite(resul, result);
    //----------------------------------------------------------------------------------------------------------//



}
