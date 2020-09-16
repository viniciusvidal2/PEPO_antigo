
#include <iostream>
#include <string>
#include <math.h>
#include <sys/stat.h>
#include <ostream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <dirent.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/mat.hpp>
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/calib3d.hpp"
#include <opencv2/stitching/detail/blenders.hpp>

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Core>



 //Definicoes e namespaces

using namespace pcl;
using namespace pcl::io;
using namespace cv;
using namespace std;
using namespace Eigen;
using namespace cv::xfeatures2d;
typedef PointXYZRGBNormal PointTN;
typedef PointXYZRGB       PointC;
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define T_DIST 20  // thres. for distance in RANSAC algorithm 
struct norm {
	cv::Mat_<double> Normalization_matrix;
	std::vector<cv::Point2f> points3d;
};
//



struct norm normalize_matri(std::vector<cv::Point2d> pointsVec) {

	// Averaging
	double count = (double)pointsVec.size();
	double xAvg = 0;
	double yAvg = 0;
	for (auto& member : pointsVec) {
		xAvg = xAvg + member.x;
		yAvg = yAvg + member.y;
	}
	xAvg = xAvg / count;
	yAvg = yAvg / count;

	// Normalization
	std::vector<cv::Point2f> points3d;
	std::vector<double> distances;
	for (auto& member : pointsVec) {

		double distance = (std::sqrt(std::pow((member.x - xAvg), 2) + std::pow((member.y - yAvg), 2)));
		distances.push_back(distance);
	}
	double xy_norm = std::accumulate(distances.begin(), distances.end(), 0.0) / distances.size();

	// Create a matrix transforming the points into having mean (0,0) and mean distance to the center equal to sqrt(2)
	cv::Mat_<double> Normalization_matrix(3, 3);
	double diagonal_element = sqrt(2) / xy_norm;
	double element_13 = -sqrt(2) * xAvg / xy_norm;
	double element_23 = -sqrt(2) * yAvg / xy_norm;

	Normalization_matrix << diagonal_element, 0, element_13, 0, diagonal_element, element_23, 0, 0, 1;

	// Multiply the original points with the normalization matrix
	for (auto& member : pointsVec) {
		cv::Mat triplet = (cv::Mat_<double>(3, 1) << member.x, member.y, 1);
		//points3d.emplace_back(Normalization_matrix * triplet);

		cv::Mat r = Normalization_matrix * triplet;
		cv::Point2f sample;
		sample.x = r.at<double>(0, 0);
		sample.y = r.at<double>(1, 0);
		points3d.emplace_back(sample);
	}

	struct norm r;
	r.Normalization_matrix = Normalization_matrix;
	r.points3d = points3d;
	return r;
}

vector<KeyPoint> Descriptors(Mat image)
{
	// Encontrando os Keypoints com SIFT
	vector<KeyPoint> kps;
	Ptr<FeatureDetector> detector = SIFT::create();
	detector->detect(image, kps);
	// Extraindo os descriptor com Root SIFT
	return kps;
}
Mat RootSift(Mat image, vector < KeyPoint> kps)
{
	Ptr<DescriptorExtractor> extractor = SIFT::create();
	Mat descs;
	extractor->compute(image, kps, descs);
	for (int i = 0; i < descs.rows; ++i) {
		// Perform L1 normalization
		normalize(descs.row(i), descs.row(i), 1.0, 0.0, cv::NORM_L1);
		//descs.row(i) /= (cv::norm(descs.row(i), cv::NORM_L1) + eps);
	}
	// Perform sqrt on the whole descriptor matrix
	sqrt(descs, descs);
	return (descs);
}
void gaussian_elimination(double* input, int n) {


	double* A = input;
	int i = 0;
	int j = 0;
	int m = n - 1;
	while (i < m && j < n) {
		// Find pivot in column j, starting in row i:
		int maxi = i;
		for (int k = i + 1; k < m; k++) {
			if (fabs(A[k * n + j]) > fabs(A[maxi * n + j])) {
				maxi = k;
			}
		}
		if (A[maxi * n + j] != 0) {
			//swap rows i and maxi, but do not change the value of i
			if (i != maxi)
				for (int k = 0; k < n; k++) {
					double aux = A[i * n + k];
					A[i * n + k] = A[maxi * n + k];
					A[maxi * n + k] = aux;
				}
			//Now A[i,j] will contain the old value of A[maxi,j].
			//divide each entry in row i by A[i,j]
			double A_ij = A[i * n + j];
			for (int k = 0; k < n; k++) {
				A[i * n + k] /= A_ij;
			}
			//Now A[i,j] will have the value 1.
			for (int u = i + 1; u < m; u++) {
				//subtract A[u,j] * row i from row u
				double A_uj = A[u * n + j];
				for (int k = 0; k < n; k++) {
					A[u * n + k] -= A_uj * A[i * n + k];
				}
				//Now A[u,j] will be 0, since A[u,j] - A[i,j] * A[u,j] = A[u,j] - 1 * A[u,j] = 0.
			}

			i++;
		}
		j++;
	}

	//back substitution
	for (int i = m - 2; i >= 0; i--) {
		for (int j = i + 1; j < n - 1; j++) {
			A[i * n + m] -= A[i * n + j] * A[j * n + m];
			//A[i*n+j]=0;
		}
	}
}

Mat findHomography_(Point2d src[4], Point2d dst[4])
{

	double P[8][9] = {
			{-src[0].x, -src[0].y, -1,   0,   0,  0, src[0].x * dst[0].x, src[0].y * dst[0].x, -dst[0].x }, // h11
			{  0,   0,  0, -src[0].x, -src[0].y, -1, src[0].x * dst[0].y, src[0].y * dst[0].y, -dst[0].y }, // h12

			{-src[1].x, -src[1].y, -1,   0,   0,  0, src[1].x * dst[1].x, src[1].y * dst[1].x, -dst[1].x }, // h13
			{  0,   0,  0, -src[1].x, -src[1].y, -1, src[1].x * dst[1].y, src[1].y * dst[1].y, -dst[1].y }, // h21

			{-src[2].x, -src[2].y, -1,   0,   0,  0, src[2].x * dst[2].x, src[2].y * dst[2].x, -dst[2].x }, // h22
			{  0,   0,  0, -src[2].x, -src[2].y, -1, src[2].x * dst[2].y, src[2].y * dst[2].y, -dst[2].y }, // h23

			{-src[3].x, -src[3].y, -1,   0,   0,  0, src[3].x * dst[3].x, src[3].y * dst[3].x, -dst[3].x }, // h31
			{  0,   0,  0, -src[3].x, -src[3].y, -1, src[3].x * dst[3].y, src[3].y * dst[3].y, -dst[3].y }, // h32
	};

	gaussian_elimination(&P[0][0], 9);

	cv::Mat A = (cv::Mat_<double>(3, 3) << P[0][8], P[1][8], P[2][8], P[3][8], P[4][8], P[5][8], P[6][8], P[7][8], 1);

	return A;


}
float get_warped_x(float x, float y, Mat H) {
	Mat P = (Mat_<double>(3, 1) << x, y, 1);
	Mat Pt;
	Pt = H * P;
	Pt = Pt / Pt.at<double>(2);;
	return Pt.at<double>(0, 0);

	//return H.at<double>(0, 0) * x + H.at<double>(0, 1) * y + H.at<double>(0, 2) * x * y + H.at<double>(1, 0);
}
float get_warped_y(float x, float y, Mat H) {
	Mat P = (Mat_<double>(3, 1) << x, y, 1);
	Mat Pt;
	Pt = H * P;
	Pt = Pt / Pt.at<double>(2);;
	return Pt.at<double>(1, 0);


	//return H.at<double>(1, 1)* x + H.at<double>(1, 2) * y + H.at<double>(2, 0) * x * y + H.at<double>(2, 1);
}
int ComputeNumberOfInliers(int num, Mat H, vector<Point2d> obj, vector<Point2d> scene, Mat* inlier_mask, double* dist_std) {

	int i, num_inlier;
	double curr_dist, sum_dist, mean_dist;
	Point2f tmp_pt;

	Mat dist = cv::Mat(num, 1, CV_64FC1);

	Mat Hin = cv::Mat(3, 1, CV_64FC1);
	H.copyTo(Hin);
	Mat invH = cv::Mat(3, 1, CV_64FC1);

	invH = Hin.inv();

	// check each correspondence
	sum_dist = 0;
	num_inlier = 0;


	//cvZero(inlier_mask);
	for (int i = 0; i < inlier_mask->rows; i++) {
		inlier_mask->at<double>(i, 0) = 0;
	}
	float threshDist = 5;
	//	omp_set_dynamic(0);
	//#pragma omp parallel for num_threads(100)
	//	for (i = 0; i < num; i++) {
	//
	//
	//		float p1_x = obj[i].x;
	//		float p1_y = obj[i].y;
	//		float p2_x = scene[i].x;
	//		float p2_y = scene[i].y;
	//		float warped_x = get_warped_x(p1_x, p1_y, H);
	//		float warped_y = get_warped_y(p1_x, p1_y, H);
	//
	//		float dist = sqrt((warped_x - p2_x)*(warped_x - p2_x) +
	//			(warped_y - p2_y)*(warped_y - p2_y));
	//		if (dist < threshDist) {
	//			inlier_mask->at<double>(i, 0) = 1;
	//			num_inlier++;
	//		}
	//
	//	}

	for (i = 0; i < num; i++) {

		// initial point x
		Mat x = (Mat_<double>(3, 1) << obj[i].x, obj[i].y, 1);

		// initial point x'
		Mat xp = (Mat_<double>(3, 1) << scene[i].x, scene[i].y, 1);

		//para cada correspondência , calcule a distância di = d(Xi',H*Xi)+d(Xi,H^-1*X')
		Mat pt = Mat(3, 1, CV_64FC1);
		// d(Hx, x')
		pt = Hin * x;
		tmp_pt.x = pt.at<double>(0, 0) / pt.at<double>(2, 0);
		tmp_pt.y = pt.at<double>(1, 0) / pt.at<double>(2, 0);
		curr_dist = pow(tmp_pt.x - scene[i].x, 2.0) + pow(tmp_pt.y - scene[i].y, 2.0);

		// d(x, invH x')
		pt = invH * xp;
		tmp_pt.x = pt.at<double>(0, 0) / pt.at<double>(2, 0);
		tmp_pt.y = pt.at<double>(1, 0) / pt.at<double>(2, 0);
		curr_dist += pow(tmp_pt.x - obj[i].x, 2.0) + pow(tmp_pt.y - obj[i].y, 2.0);

		if (curr_dist < T_DIST) {
			// an inlier
			num_inlier++;
			inlier_mask->at<double>(i, 0) = 1;
			dist.at<double>(i, 0) = curr_dist;
			sum_dist += curr_dist;
			//inliers->at(i) = true;
		}
	}



	//calcular o desvio padrão da distância inlier
	mean_dist = sum_dist / (double)num_inlier;


	*dist_std = 0;
	for (i = 0; i < num; i++) {
		if (inlier_mask->at<double>(i, 0) == 1)
			*dist_std += pow(dist.at<double>(i, 0) - mean_dist, 2.0);
	}

	*dist_std /= (double)(num_inlier - 1);
	return num_inlier;
}

cv::Mat leastSquare(std::vector< cv::Point2d > obj, std::vector< cv::Point2d > scene, int numOfIn, cv::Mat InliersMat) {


	std::vector< cv::Point2d > obj_inliers;
	std::vector< cv::Point2d > scene_inliers;

	for (int i = 0; i < obj.size(); i++) {

		if (InliersMat.at<double>(i, 0) == 1) {
			obj_inliers.push_back(obj[i]);
			scene_inliers.push_back(scene[i]);
		}
	}

	/*
		Normalize the coordinate system to stabilize the Least Squares algorithm
	*/
	struct norm norm_obj;
	struct norm norm_scene;
	norm_obj = normalize_matri(obj_inliers);
	norm_scene = normalize_matri(scene_inliers);
	/*
		let's create a opencv matrix to store X and Y values
	*/
	cv::Mat X = cv::Mat_<double>(2 * numOfIn, 9, double(0));

	int j = 0;

	std::cout << norm_scene.points3d.size() << std::endl;

	for (int i = 0; i < scene_inliers.size(); i++) {

		cv::Mat X_ = (cv::Mat_<double>(1, 9) << -norm_obj.points3d[i].x, -norm_obj.points3d[i].y, -1, 0, 0, 0,
			norm_obj.points3d[i].x * norm_scene.points3d[i].x, norm_obj.points3d[i].y * norm_scene.points3d[i].x, norm_scene.points3d[i].x);
		X.row(j) += X_;

		X_ = (cv::Mat_<double>(1, 9) << 0, 0, 0, -norm_obj.points3d[i].x, -norm_obj.points3d[i].y, -1,
			norm_obj.points3d[i].x * norm_scene.points3d[i].y, norm_obj.points3d[i].y * norm_scene.points3d[i].y, norm_scene.points3d[i].y);
		X.row(j + 1) += X_;


		j = j + 2;


	}

	cv::Mat U = cv::Mat_<double>(2 * numOfIn, 9, double(0));
	cv::Mat W = cv::Mat_<double>(9, 9, double(0));
	cv::Mat Vt = cv::Mat_<double>(9, 9, double(0));

	cv::SVDecomp(X, W, U, Vt);

	cv::Mat V = cv::Mat_<double>(9, 9, double(0));
	V = Vt.t();

	cv::Mat H = (cv::Mat_<double>(3, 3) << V.at<double>(0, 8), V.at<double>(1, 8), V.at<double>(2, 8),
		V.at<double>(3, 8), V.at<double>(4, 8), V.at<double>(5, 8), V.at<double>(6, 8), V.at<double>(7, 8), V.at<double>(8, 8));

	H = norm_scene.Normalization_matrix.inv() * H * norm_obj.Normalization_matrix;
	H = H / H.at<double>(2, 2);

	return H;
}
Mat computeHomography(vector<Point2d> obj, vector<Point2d> scene)
{

	//iniciando as matrizes que serão necessarias no calculo da homografia
	Point2d src[4];
	Point2d dst[4];
	cv::Point2d good_src[4];
	cv::Point2d good_dst[4];


	cv::Point2d test_src;
	cv::Point2d test_dst;

	cv::Point2d max_src;
	cv::Point2d max_dst;
	//lets create a variable to count how many of the points are inliners
	int numOfIn = 0;
	bool bSuccess(false);
	// and another variable to store the max number of inliners acheived by 
	// the last selection of points
	int maxNumOfIn = 0;
	vector<bool> TInliers;
	vector<bool> Inliers;
	Mat inlier_mask = Mat_<double>(obj.size(), 1, double(0));
	for (int i = 0; i <= 500; i++) //for ith (i = 1 : N) estimation
	{
		//escolhendo aleatoriamente 4 correspondências
		int indx0 = (rand() % obj.size());
		src[0] = obj[indx0];
		dst[0] = scene[indx0];

		int indx1 = (rand() % obj.size());
		src[1] = obj[indx1];
		dst[1] = scene[indx1];

		int indx2 = (rand() % obj.size());
		src[2] = obj[indx2];
		dst[2] = scene[indx2];

		int indx3 = (rand() % obj.size());
		src[3] = obj[indx3];
		dst[3] = scene[indx3];


		//calcular a homografia  por DLT normalizado a partir dos pares de 4 pontos
		Mat H = (Mat_<double>(3, 3) << 0, 0, 0, 0, 0, 0, 0, 0, 0);
		H = findHomography_(src, dst);

		//reset the number of inliner for the nex iteration
		numOfIn = 0;

		//clear the list on inliers for this iteration
		TInliers.clear();

		Mat Temp_inlier_mask = cv::Mat_<double>(obj.size(), 1, double(0));

		double dist_std;
		numOfIn = ComputeNumberOfInliers(obj.size(), H, obj, scene, &Temp_inlier_mask, &dist_std);

		//save the maximum number of inliners found in the data set and the points that form them
		if (maxNumOfIn < numOfIn) {

			maxNumOfIn = numOfIn;

			good_src[0] = obj[indx0];
			good_dst[0] = scene[indx0];

			good_src[1] = obj[indx1];
			good_dst[1] = scene[indx1];

			good_src[2] = obj[indx2];
			good_dst[2] = scene[indx2];

			good_src[3] = obj[indx3];
			good_dst[3] = scene[indx3];

			//copy the inliners to the Output inliners
			Inliers = TInliers;
			inlier_mask = Temp_inlier_mask;
		}
	}
	bSuccess = true;
	// if successful, write the line parameterized as two points, 
	// and each input point along with its inlier status

	cv::Mat H1 = (cv::Mat_<double>(3, 3) << 0, 0, 0, 0, 0, 0, 0, 0, 0);
	H1 = leastSquare(obj, scene, maxNumOfIn, inlier_mask);

	return H1;
}






//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int getdir(string dir, vector<string> &nuvens) {
	// Abrindo a pasta raiz para contar os arquivos de imagem e nuvem que serao lidos e enviados
	DIR *dp;
	struct dirent *dirp;
	string nome_temp;
	if ((dp = opendir(dir.c_str())) == NULL)
		printf("Nao foi possivel abrir o diretorio");

	while ((dirp = readdir(dp)) != NULL) {
		nome_temp = string(dirp->d_name);
		// Todas as nuvens na pasta
		if (nome_temp.substr(nome_temp.find_last_of(".") + 1) == "ply")
			nuvens.push_back(nome_temp);
	}
	closedir(dp);

	return 0;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Matrix4f calculateCameraPose(Quaternion<float> q, Vector3f &C, int i) {
	Matrix3f r = q.matrix();
	Vector3f t = C;

	Matrix4f T = Matrix4f::Identity();
	T.block<3, 3>(0, 0) = r.transpose(); T.block<3, 1>(0, 3) = t;

	return T;
	//if (i == 1)
	//{
	//	Matrix3f r = q.matrix();
	//	Vector3f t = C;

	//	Matrix4f T = Matrix4f::Identity();
	//	T.block<3, 3>(0, 0) = r.transpose(); T.block<3, 1>(0, 3) = t;
	//	return T;
	//}
	//else
	//{

	//	Matrix3f r = q.matrix();
	//	Vector3f angulos = r.eulerAngles(0, 1, 2);
	//	r = AngleAxisf(angulos[0], Vector3f::UnitX()) *
	//		AngleAxisf(angulos[1] +0.023, Vector3f::UnitY()) *
	//		AngleAxisf(angulos[2], Vector3f::UnitZ());

	//	Vector3f t = C;

	//	Matrix4f T = Matrix4f::Identity();
	//	T.block<3, 3>(0, 0) = r.transpose(); T.block<3, 1>(0, 3) = t;
	//	return T;
	//}


}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void createFrustrumPlane(PointCloud<PointTN>::Ptr plano, Vector3f p2, Vector3f p4, Vector3f p5, Mat im) {
	// Resolucao a ser somada igual a da imagem, dividir os vetores pela resolucao da imagem
	Vector3f hor_step, ver_step;
	hor_step = (p4 - p5) / float(im.cols);
	ver_step = (p2 - p5) / float(im.rows);
	plano->resize(im.rows*im.cols);
	// Criar a posicao de cada ponto em 3D e ainda atribuir a cor ali
	// Posicao 3D somando os vetores com a origem em p5
	omp_set_dynamic(0);
#pragma omp parallel for num_threads(20)
	for (int i = 0; i < im.rows; i++) {
		for (int j = 0; j < im.cols; j++) {
			Vector3f ponto;
			PointTN ponto_nuvem;
			ponto = p5 + hor_step * j + ver_step * i;
			ponto_nuvem.x = ponto(0); ponto_nuvem.y = ponto(1); ponto_nuvem.z = ponto(2);
			ponto_nuvem.r = im.at<Vec3b>(Point(j, im.rows - 1 - i))[2];
			ponto_nuvem.g = im.at<Vec3b>(Point(j, im.rows - 1 - i))[1];
			ponto_nuvem.b = im.at<Vec3b>(Point(j, im.rows - 1 - i))[0];
			// Adiciona ponto na nuvem do plano de saida
			plano->points[i*im.cols + j] = ponto_nuvem;
		}
	}
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
PointTN intersectSphere(Vector3f C, PointTN ponto_plano, float R) {
	Vector3f p(ponto_plano.x, ponto_plano.y, ponto_plano.z);
	// Equacao da reta saindo de C para ponto_plano
	Vector3f pDiff = p - C;
	float x0 = C(0), y0 = C(1), z0 = C(2);
	float xd = pDiff(0), yd = pDiff(1), zd = pDiff(2);
	// Equacao da reta na origem e x2 + y2 + z2 = R2
	// Intersecao com a equacao da reta gera duas solucoes para t. Os coeficientes da equacao de
	// segundo grau para resolver Bhaskara seriam a, b e c
	float a, b, c, D, t1, t2;
	a = xd * xd + yd * yd + zd * zd;
	b = 2 * (x0*xd + y0 * yd + z0 * zd);
	c = x0 * x0 + y0 * y0 + z0 * z0 - R * R;
	// Duas solucoes para dois pontos de intersecao, o mais perto e o que segue
	D = sqrt(b*b - 4 * a*c);
	//    cout << "a: " << xd << "  b: " << yd << "  c: " << zd << "  d:  " << D << endl;
	t1 = (-b + D) / (2 * a);
	t2 = (-b - D) / (2 * a);
	Vector3f int1, int2;
	PointTN final;
	int1 = C + t1 * pDiff;
	int2 = C + t2 * pDiff;
	//    cout << "t1: " << t1 << "  t2: " << t2 << endl;
	//    if( (p - int1).norm() < (p - int2).norm() ){
	if (t1 > 0) {
		final.x = int1(0); final.y = int1(1); final.z = int1(2);
	}
	else {
		final.x = int2(0); final.y = int2(1); final.z = int2(2);
	}

	return final;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float calculateFrustrumFromPose(Quaternion<float> q, Matrix4f T, float r, Vector3f C) {
	// Comprimento do eixo focal no frustrum
	float F;
	// Criar vetor unitario n na orientacao vinda do quaternion
	Vector3f k(0, 0, 1);
	Vector3f n = q.matrix()*k;
	// Calculo de P1 a partir da matriz de transformacao homogenea e o ponto central homogeneo
	Vector4f Ch(C[0], C[1], C[2], 1);
	Vector4f p1h = T * Ch;
	Vector3f p1 = p1h.block<3, 1>(0, 0);
	//    cout << "T:  " << T << endl;
	//    cout << "p1:  " << p1h.transpose() << endl;
		// Definir coeficientes do plano a, b e c pelo vetor unitario n
	float a = n(0), b = n(1), c = n(2);
	// Achar a intersecao da reta que vai do centro da esfera C, com a orientacao de n, ate a superficie da esfera
	// pois o plano sera tangente ali
	Vector3f p_tangente = C + r * n; // Multiplicar o vetor unitario de orientacao pelo raio
//    cout << "p_tangente:  " << p_tangente.transpose() << endl;
	// Calcular coeficiente d do plano tangente
	float d = -(a*p_tangente(0) + b * p_tangente(1) + c * p_tangente(2));

	//    cout << "a: " << a << "   b: " << b << "   c: " << c << "  d: " << d << endl;
		// Valor de F a partir da distancia entre o centro da camera p1 e o plano tangente
	float num = abs(a*p1(0) + b * p1(1) + c * p1(2) + d), den = sqrt(a*a + b * b + c * c);
	F = num / den;

	return F;
}
int searchNeighbors(MatrixXi im, int r, int c, int l) {
	// Duplo for sobre a vizinhanca pra ver se encontra, e o mais perto de todos leva
	int indice = -1; // Indice comeca como -1, se for encontrado temos coisa melhor
	for (int i = r - l; i < r + l; i++)
	{
		if (i >= 0 && i < im.cols())
		{
			for (int j = c - l; j < c + l; j++)
			{
				if (j > 0 && j < im.rows())
				{
					// Se nesse lugar esta algo diferente de 0
					if (im(j, i) != -1)
					{
						indice = im(j, i);
						return indice;
					}
				}
			}
		}
	}

	return indice;
}
bool getGaussianMask(cv::Mat& mask, cv::Size mask_size, float sigma)
{
	// assume mask is 1-channel image
	if (mask.channels() != 1) {
		cout << "Error in getGaussianMask(): mask should be 1-channel." << endl;
		return false;
	}
	// create as double type
	mask.create(mask_size.height, mask_size.width, CV_64F);
	double cx = static_cast<double>(mask_size.width) / 2;
	double cy = static_cast<double>(mask_size.height) / 2;

	for (int c = 0; c < mask.cols; c++) {
		for (int r = 0; r < mask.rows; r++) {
			mask.at<double>(r, c) = exp(-((c - cx)*(c - cx) + (r - cy)*(r - cy)) / 2 / sigma / sigma);
		}
	}

	return true;
}
bool projectImagesToCylinderMask(Mat &images, cv::Mat &mask)
{
	// same map for all images since they all have same size
	cv::Mat xmap, ymap;
	//getCylinderMaps(xmap, ymap, images.size(), static_cast<float>(1127));

	cv::Mat orig_mask;
	float sigma = static_cast<float>(images.cols / 2);
	getGaussianMask(mask, images.size(), 50);
	//cv::remap(orig_mask, mask, xmap, ymap, cv::INTER_LINEAR); //, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0,0,0) );

	return true;
}
//bool isEmpty(Mat img, int x, int y)
//{
//	//assert(img.spectrum() == 3);
//
//	return (img.at<Vec3b>(Point(x, y))[0] == 0 || img.at<Vec3b>(Point(x, y))[1] == 0 || img.at<Vec3b>(Point(x, y))[2] == 0);
//}
Mat GaussianFilter(Mat ori, float sigma)
{
	// A good size for kernel support
	int ksz = (int)((sigma - 0.35) / 0.15);
	if (ksz < 0) ksz = 1;
	if (ksz % 2 == 0) ksz++; // Must be odd

	Size kernel_size(ksz, ksz);

	Mat fin;
	GaussianBlur(ori, fin, kernel_size, sigma, sigma);

	return fin;
}

bool isEmpty(Mat img, int x, int y) {
	//assert(img.spectrum() == 3);

	return (img.at<Vec3f>(Point(x, y))[0] == 0 && img.at<Vec3f>(Point(x, y))[1] && img.at<Vec3f>(Point(x, y))[2]);
}
bool checkColColor(Mat & img, int x) {
	Vec3f keyColor(0, 0, 0);
	for (int y = 0; y < img.cols; y++) {
		if (img.at<Vec3f>(x, y) != keyColor)
			return false;
	}
	return true;
}
bool checkColColor1(Mat & img, int x) {
	Vec3f keyColor(0, 0, 0);
	for (int y = 0; y < img.rows; y++) {
		if (img.at<Vec3f>(y, x) != keyColor)
			return false;
	}
	return true;
}
Mat alphaBlending(Mat foreground, const Mat background)
{
	Mat src_gray;
	cvtColor(foreground, src_gray, CV_BGR2GRAY);
	src_gray.convertTo(src_gray, CV_8UC3, 255);
	Mat dst(src_gray.rows, src_gray.cols, CV_8UC3, Scalar::all(0));
	vector<vector<Point> > contours; // Vector for storing contour
	vector<Vec4i> hierarchy;

	findContours(src_gray, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE); // Find the contours in the image

	for (int i = 0; i < contours.size(); i++) // iterate through each contour. 
	{

		Scalar color(255, 255, 255);
		drawContours(dst, contours, i, color, CV_FILLED);
	}
	imwrite("C:/dataset3/mask.png", dst);
	//dst.convertTo(dst, CV_32FC3, 1.0 / 255.0);


	Mat src_gray1;
	cvtColor(background, src_gray1, CV_BGR2GRAY);
	src_gray1.convertTo(src_gray1, CV_8UC3, 255);
	Mat dst1(src_gray1.rows, src_gray1.cols, CV_8UC3, Scalar::all(0));
	vector<vector<Point> > contours1; // Vector for storing contour
	vector<Vec4i> hierarchy1;

	findContours(src_gray1, contours1, hierarchy1, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE); // Find the contours in the image

	for (int i = 0; i < contours1.size(); i++) // iterate through each contour. 
	{

		Scalar color(255, 255, 255);
		drawContours(dst1, contours1, i, color, CV_FILLED);
	}
	imwrite("C:/dataset3/mask1.png", dst1);
	Mat out(dst1.rows, dst1.cols, CV_8UC3, Scalar::all(0));
	Mat alpha(dst1.rows, dst1.cols, CV_8UC3, Scalar::all(0));
	Mat mask(src_gray1.rows, src_gray1.cols, CV_8UC1, Scalar::all(0));

	bitwise_and(dst, dst1, out);



	for (int x = 0; x < foreground.cols; x++)
	{
		for (int y = 0; y < foreground.rows; y++)
		{
			for (int c = 0; c < 3; c++) {
				alpha.at<Vec3b>(Point(x, y))[c] = dst1.at<Vec3b>(Point(x, y))[c] - out.at<Vec3b>(Point(x, y))[c];
			}
		}
	}
	//	alpha = out;

	imwrite("C:/dataset3/maskfinal.png", alpha);

	foreground.convertTo(foreground, CV_32FC3);

	background.convertTo(background, CV_32FC3);

	alpha.convertTo(alpha, CV_32FC3, 1.0 / 255); //

	Mat ouImage = Mat::zeros(foreground.size(), foreground.type());
	// Multiply the foreground with the alpha matte

	multiply(alpha, background, background);

	multiply(Scalar::all(1.0) - alpha, foreground, foreground);

	add(background, foreground, ouImage);

	// Display image
	imwrite("C:/dataset3/result1.jpg", ouImage);

	return ouImage;
}
Scalar findColor(int i, int num) {
	Scalar Source(0, 0, 0);  // blue
	Scalar Target(255, 255, 255);  // red

	Scalar MyColour = Source;
	vector<int> red, blue, green;
	const int NumSteps = num;
	/*for (int i = 0; i < NumSteps; i++)
	{*/


	MyColour[0] = Source[0] + (((Target[0] - Source[0])   * i) / NumSteps);
	MyColour[1] = Source[1] + (((Target[1] - Source[1]) * i) / NumSteps);
	MyColour[2] = Source[2] + (((Target[2] - Source[2])  * i) / NumSteps);
	/*red.push_back(MyColour[0]);
	green.push_back(MyColour[1]);
	blue.push_back(MyColour[2]);*/
	// Do something, like update the display
//	DoSomethingWithColour(MyColour);

	return MyColour;
	/*}*/
}
Mat multiband_blending(Mat &a, const Mat &b, int i) {
	int w = a.cols, h = a.rows; // a and b has the same size
	int min_len = (a.cols < a.rows) ? a.cols : a.rows;

	//int level_num = floor(log2(min_len));
	int level_num = 4;
	Mat a_pyramid[100];
	Mat b_pyramid[100];
	Mat mask[100];
	a_pyramid[0] = a;
	b_pyramid[0] = b;

	Mat teste(h, w, CV_32FC3, Scalar(0, 0, 0));
	teste.convertTo(teste, CV_32F, 1.0 / 255.0);
	mask[0] = teste;
	//*********************Contorno imagem 1*************************************
	Mat src_gray;
	cvtColor(a, src_gray, CV_BGR2GRAY);
	src_gray.convertTo(src_gray, CV_8UC3, 255);
	Mat dst(src_gray.rows, src_gray.cols, CV_8UC3, Scalar::all(0));
	vector<vector<Point> > contours; // Vector for storing contour
	vector<Vec4i> hierarchy;

	findContours(src_gray, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE); // Find the contours in the image

	for (int i = 0; i < contours.size(); i++) // iterate through each contour. 
	{

		Scalar color(255, 255, 255);
		drawContours(dst, contours, i, color, CV_FILLED);
	}
	imwrite("C:/dataset3/Imagem1.png", dst);

	//*********************Contorno imagem 2*************************************

	Mat src_gray1;
	cvtColor(b, src_gray1, CV_BGR2GRAY);
	src_gray1.convertTo(src_gray1, CV_8UC3, 255);
	Mat dst1(src_gray1.rows, src_gray1.cols, CV_8UC3, Scalar::all(0));
	vector<vector<Point> > contours1; // Vector for storing contour
	vector<Vec4i> hierarchy1;

	findContours(src_gray1, contours1, hierarchy1, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE); // Find the contours in the image

	for (int i = 0; i < contours1.size(); i++) // iterate through each contour. 
	{

		Scalar color(255, 255, 255);
		drawContours(dst1, contours1, i, color, CV_FILLED);
	}
	imwrite("C:/dataset3/Imagem2.png", dst1);

	/////////////Contorno Parte comum
	
	//Aumentar Contorno saida
	vector<int> rows;
	vector<int> cols;
	int cont = 0;
	
	
	Mat out(dst1.rows, dst1.cols, CV_8UC3, Scalar::all(0));
	bitwise_and(dst, dst1, out);

	imwrite("C:/dataset3/AreaComum.png", out);
	for (int x = 0; x < dst.cols; x++)
	{
		for (int y = 0; y < dst.rows; y++)
		{
			for (int c = 0; c < 3; c++)
			{

				dst.at<Vec3b>(Point(x, y))[c] = dst.at<Vec3b>(Point(x, y))[c] - out.at<Vec3b>(Point(x, y))[c];
			}
		}
	}
	imwrite("C:/dataset3/Imagem1SemSobreposicao.png", dst);
	Mat src_gray3;
	cvtColor(out, src_gray3, CV_BGR2GRAY);
	src_gray3.convertTo(src_gray3, CV_8UC3, 255);
	Mat dst3(src_gray3.rows, src_gray3.cols, CV_8UC3, Scalar::all(0));
	vector<vector<Point> > contours3; // Vector for storing contour
	vector<Vec4i> hierarchy3;

	findContours(src_gray3, contours3, hierarchy3, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE); // Find the contours in the image

	for (int i = 0; i < contours3.size(); i++) // iterate through each contour. 
	{

		Scalar color(255, 255, 255);
		drawContours(dst3, contours3, i, color, 30);
	}
	
	Mat src_gray4;
	cvtColor(dst3, src_gray4, CV_BGR2GRAY);
	src_gray4.convertTo(src_gray4, CV_8UC3, 255);
	Mat dst4(src_gray4.rows, src_gray4.cols, CV_8UC3, Scalar::all(0));
	vector<vector<Point> > contours4; // Vector for storing contour
	vector<Vec4i> hierarchy4;

	findContours(src_gray4, contours4, hierarchy4, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE); // Find the contours in the image

	for (int i = 0; i < contours4.size(); i++) // iterate through each contour. 
	{
		//Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
		Scalar color(255, 255, 255);
		drawContours(dst4, contours4, i, color, -1);
	}
	imwrite("C:/dataset3/Contorno1.png", dst4);
	
	for (int x = 0; x < dst.cols; x++)
	{
		for (int y = 0; y < dst.rows; y++)
		{
			for (int c = 0; c < 3; c++)
			{

				dst.at<Vec3b>(Point(x, y))[c] = dst.at<Vec3b>(Point(x, y))[c]- dst4.at<Vec3b>(Point(x, y))[c];
			}
		}
	}
	
	
	imwrite("C:/dataset3/CorrecaoContorno.png", dst);
		dst1.convertTo(dst1, CV_32FC3, 1.0 / 255.0);
		dst.convertTo(dst, CV_32FC3, 1.0 / 255.0);
		out.convertTo(out, CV_32FC3, 1.0 / 255.0);
		dst3.convertTo(dst3, CV_32FC3, 1.0 / 255.0);
		//dst2.convertTo(dst2, CV_32FC3, 1.0 / 255.0);
		mask[0] = dst;




		//	imwrite("C:/dataset3/mask1.png", dst);
			/* ii. Perform a local Gaussian weighted averaging
			function in a neighborhood about each pixel,
			sampling so that the result is a reduced image
			of half the size in each dimension. */
			/* ii. Perform a local Gaussian weighted averaging
			function in a neighborhood about each pixel,
			sampling so that the result is a reduced image
			of half the size in each dimension. */

		for (int i = 1; i < level_num; ++i)
		{


			int wp = a_pyramid[i - 1].rows / 2;
			int hp = a_pyramid[i - 1].cols / 2;
			int wp1 = b_pyramid[i - 1].rows / 2;
			int hp1 = b_pyramid[i - 1].cols / 2;

			Mat a_pic, b_pic, mask_pic;

			a_pic = GaussianFilter(a_pyramid[i - 1], 2);
			b_pic = GaussianFilter(b_pyramid[i - 1], 2);
			mask_pic = GaussianFilter(mask[i - 1], 2);

			Mat new_a(wp, hp, CV_32FC3, cv::Scalar(0, 0, 0));
			Mat new_b(wp1, hp1, CV_32FC3, cv::Scalar(0, 0, 0));
			Mat new_mask(wp1, hp1, CV_32FC3, cv::Scalar(0, 0, 0));

			omp_set_dynamic(0);
#pragma omp parallel for num_threads(100)
			for (int r = 0; r < new_a.cols; r++) {
				for (int c = 0; c < new_a.rows; c++)
				{

					new_a.at<Vec3f>(Point(r, c))[0] = a_pic.at<Vec3f>(Point(2 * r, 2 * c))[0];
					new_a.at<Vec3f>(Point(r, c))[1] = a_pic.at<Vec3f>(Point(2 * r, 2 * c))[1];
					new_a.at<Vec3f>(Point(r, c))[2] = a_pic.at<Vec3f>(Point(2 * r, 2 * c))[2];

					new_b.at<Vec3f>(Point(r, c))[0] = b_pic.at<Vec3f>(Point(2 * r, 2 * c))[0];
					new_b.at<Vec3f>(Point(r, c))[1] = b_pic.at<Vec3f>(Point(2 * r, 2 * c))[1];
					new_b.at<Vec3f>(Point(r, c))[2] = b_pic.at<Vec3f>(Point(2 * r, 2 * c))[2];

					new_mask.at<Vec3f>(Point(r, c))[0] = mask_pic.at<Vec3f>(Point(2 * r, 2 * c))[0];
					new_mask.at<Vec3f>(Point(r, c))[1] = mask_pic.at<Vec3f>(Point(2 * r, 2 * c))[1];
					new_mask.at<Vec3f>(Point(r, c))[2] = mask_pic.at<Vec3f>(Point(2 * r, 2 * c))[2];
				}
			}
			a_pyramid[i] = new_a;
			b_pyramid[i] = new_b;

			mask[i] = new_mask;

		}

		/* 1. Compute Laplacian pyramid of images and mask */
		  /* Making the Laplacians Li=Gi-expand(Gi+1)*/
		  /*  subtract each level of the pyramid from the next lower one
		  EXPAND:  interpolate new samples between those of
		  a given image to make it big enough to subtract*/
		for (int i = 0; i < level_num - 1; ++i) {
			int wp = a_pyramid[i].cols;
			int hp = a_pyramid[i].rows;

			cv::Mat dst_a, dst_b;


			cv::resize(a_pyramid[i + 1], dst_a, cv::Size(wp, hp));
			cv::resize(b_pyramid[i + 1], dst_b, cv::Size(wp, hp));
			omp_set_dynamic(0);
#pragma omp parallel for num_threads(100)
			for (int x = 0; x < a_pyramid[i].cols; x++)
			{
				for (int y = 0; y < a_pyramid[i].rows; y++)
				{


					a_pyramid[i].at<Vec3f>(Point(x, y))[0] -= dst_a.at<Vec3f>(Point(x, y))[0];
					a_pyramid[i].at<Vec3f>(Point(x, y))[1] -= dst_a.at<Vec3f>(Point(x, y))[1];
					a_pyramid[i].at<Vec3f>(Point(x, y))[2] -= dst_a.at<Vec3f>(Point(x, y))[2];

					b_pyramid[i].at<Vec3f>(Point(x, y))[0] -= dst_b.at<Vec3f>(Point(x, y))[0];
					b_pyramid[i].at<Vec3f>(Point(x, y))[1] -= dst_b.at<Vec3f>(Point(x, y))[1];
					b_pyramid[i].at<Vec3f>(Point(x, y))[2] -= dst_b.at<Vec3f>(Point(x, y))[2];


				}
			}



		}

		/* 2. Create blended image at each level of pyramid */
		/* Forming the New Pyramid
		A third Laplacian pyramid LS is constructed by copying
		nodes from the left half of LA to the corresponding
		nodes of LS and nodes from the right half of LB to the
		right half of LS.
		Nodes along the center line are set equal to
		the average of corresponding LA and LB nodes */

		Mat blend_pyramid[100];
		Mat fin(a_pyramid[i].rows, a_pyramid[i].cols, CV_8UC3);
		for (int i = 0; i < level_num; ++i)
		{

			Mat fin(a_pyramid[i].rows, a_pyramid[i].cols, CV_32FC3, cv::Scalar(0, 0, 0));
			blend_pyramid[i] = fin;
			/*blend_pyramid[i] = CImg<float>(a_pyramid[i].cols,
				a_pyramid[i].rows, 1, a_pyramid[i].spectrum(), 0);*/
			omp_set_dynamic(0);
#pragma omp parallel for num_threads(100)
			for (int x = 0; x < blend_pyramid[i].cols; x++)
			{
				for (int y = 0; y < blend_pyramid[i].rows; y++)
				{

					blend_pyramid[i].at<Vec3f>(Point(x, y))[0] = a_pyramid[i].at<Vec3f>(Point(x, y))[0] * mask[i].at<Vec3f>(Point(x, y))[0] + b_pyramid[i].at<Vec3f>(Point(x, y))[0] * (1.0 - mask[i].at<Vec3f>(Point(x, y))[0]);
					blend_pyramid[i].at<Vec3f>(Point(x, y))[1] = a_pyramid[i].at<Vec3f>(Point(x, y))[1] * mask[i].at<Vec3f>(Point(x, y))[1] + b_pyramid[i].at<Vec3f>(Point(x, y))[1] * (1.0 - mask[i].at<Vec3f>(Point(x, y))[1]);
					blend_pyramid[i].at<Vec3f>(Point(x, y))[2] = a_pyramid[i].at<Vec3f>(Point(x, y))[2] * mask[i].at<Vec3f>(Point(x, y))[2] + b_pyramid[i].at<Vec3f>(Point(x, y))[2] * (1.0 - mask[i].at<Vec3f>(Point(x, y))[2]);


				}
			}
		}
		
		/* 3. Reconstruct complete image */
		/* Using the new Laplacian Pyramid
		Use the new Laplacian pyramid with the reverse of how it
		was created to create a Gaussian pyramid. Gi=Li+expand(Gi+1)
		The lowest level of the new Gaussian pyramid gives the final
		result. */

		Mat expand = blend_pyramid[level_num - 1];
		for (int i = level_num - 2; i >= 0; --i)
		{
			cv::resize(expand, expand, cv::Size(blend_pyramid[i].cols, blend_pyramid[i].rows));
			omp_set_dynamic(0);
#pragma omp parallel for num_threads(100)
			for (int x = 0; x < blend_pyramid[i].cols; x++)
			{
				for (int y = 0; y < blend_pyramid[i].rows; y++)
				{

					expand.at<Vec3f>(Point(x, y))[0] = blend_pyramid[i].at<Vec3f>(Point(x, y))[0] + expand.at<Vec3f>(Point(x, y))[0];
					expand.at<Vec3f>(Point(x, y))[1] = blend_pyramid[i].at<Vec3f>(Point(x, y))[1] + expand.at<Vec3f>(Point(x, y))[1];
					expand.at<Vec3f>(Point(x, y))[2] = blend_pyramid[i].at<Vec3f>(Point(x, y))[2] + expand.at<Vec3f>(Point(x, y))[2];


					for (int c = 0; c < 3; c++)
					{
						if (expand.at<Vec3f>(Point(x, y))[c] > 255)
						{
							expand.at<Vec3f>(Point(x, y))[c] = 255;
						}
						else if (expand.at<Vec3f>(Point(x, y))[c] < 0)
						{

							expand.at<Vec3f>(Point(x, y))[c] = 0;


						}
					}



				}

			}
		}
		cout << "\n blending \n";
		return expand;
	}
	bool mergeImages(const std::vector<cv::Mat> &images_warped,
		const std::vector<cv::Mat> &masks_warped,
		cv::Mat &result)
	{

		// merge images
		result.create(images_warped[0].rows, images_warped[0].cols, images_warped[0].type());
		omp_set_dynamic(0);
#pragma omp parallel for num_threads(40)
		for (int c = 0; c < result.cols; c++)
		{
			for (int r = 0; r < result.rows; r++)
			{
				// blend images using the isInImage markers
				double sum = 0;
				for (int i = 0; i < images_warped.size(); i++)
					sum += masks_warped[i].at<double>(r, c);


				if (images_warped[0].channels() == 1)
				{  // if one channel in uchar
					if (sum == 0)
						result.at<uchar>(r, c) = 0;
					else
					{
						double gray = 0;
						for (int i = 0; i < images_warped.size(); i++)
						{
							double weight = masks_warped[i].at<double>(r, c) / sum;
							gray += static_cast<double>(images_warped[i].at<uchar>(r, c)) * weight;
						}
						result.at<uchar>(r, c) = static_cast<uchar>(gray);

					}
				}
				else if (images_warped[0].channels() == 3)
				{   // if three channels in Vec3b (uchar 3components)
					if (sum == 0)
					{
						result.at<cv::Vec3b>(r, c)[0] = 0;
						result.at<cv::Vec3b>(r, c)[1] = 0;
						result.at<cv::Vec3b>(r, c)[2] = 0;
					}
					else
					{
						double blue = 0, green = 0, red = 0;
						for (int i = 0; i < images_warped.size(); i++)
						{
							double weight = masks_warped[i].at<double>(r, c) / sum;
							// BGR order
							blue += static_cast<double>(images_warped[i].at<cv::Vec3b>(r, c)[0]) * weight;
							green += static_cast<double>(images_warped[i].at<cv::Vec3b>(r, c)[1]) * weight;
							red += static_cast<double>(images_warped[i].at<cv::Vec3b>(r, c)[2]) * weight;
						}
						result.at<cv::Vec3b>(r, c)[0] = static_cast<uchar>(blue);
						result.at<cv::Vec3b>(r, c)[1] = static_cast<uchar>(green);
						result.at<cv::Vec3b>(r, c)[2] = static_cast<uchar>(red);

					}
				} // end if channels

			}
		} // end for all pixels 



		return true;
	}
	void img_shift(Mat src, Mat dst) {
		omp_set_dynamic(0);
#pragma omp parallel for num_threads(100)
		for (int x = 0; x < dst.cols; x++)
		{
			for (int y = 0; y < dst.rows; y++)
			{

				if (src.channels() == 1) {
					dst.at<double>(Point(x, y)) = src.at<double>(Point(x, y));

				}
				else {
					dst.at<Vec3b>(Point(x, y))[0] = src.at<Vec3b>(Point(x, y))[0];
					dst.at<Vec3b>(Point(x, y))[1] = src.at<Vec3b>(Point(x, y))[1];
					dst.at<Vec3b>(Point(x, y))[2] = src.at<Vec3b>(Point(x, y))[2];

				}

			}
		}
	}
	//bool checkColColor(Mat & img, int x) {
	//	Vec3b keyColor(0, 0, 0);
	//	for (int y = 0; y < img.rows; y++) {
	//		if (img.at<Vec3b>(y, x) != keyColor)
	//			return false;
	//	}
	//	return true;
	//}
	Mat cortarLaterales(Mat & img) {
		//Para controlar la posición de las cuatro esquinas
		int posXi = 0, posXf = img.cols - 1;
		bool cambio;
		//Buscamos por cada lado, los bordes a eliminar		
		do {
			cambio = false;
			if (checkColColor(img, posXi)) { posXi++;	cambio = true; }//Columna izquierda		
			if (checkColColor(img, posXf)) { posXf--;	cambio = true; }//Columna derecha
		} while (cambio);
		return img(Rect(posXi, 0, posXf - posXi + 1, img.rows));
	}
	static void SetImageAlpha(Mat img, float blendRadius)
	{
		// *** BEGIN TODO ***
		// fill in this routine..

		int cx = img.cols / 2.0f, cy = img.rows / 2.0f;

		for (int y = 0; y < img.rows; y++)
			for (int x = 0; x < img.cols; x++)
			{
				float wx = 1.0f - abs(x - cx) / cx;
				float wy = 1.0f - abs(y - cy) / cy;
				img.at<Vec4b>((Point(x, y)))[3] = (uchar)(wx * wy * 255);
				/*f/*loat x1 = x - 0.5f*width;
				float y1 = y - 0.5f*height;
				float alpha = 1.0f - (pow(x1, 2) + pow(y1, 2)) / pow(blendRadius, 2);
				if (alpha < 0)
					alpha = 0;

				img.at<Vec4b>((Point(x, y)))[3] = (unsigned int)(255 * alpha);*/
			}
		// *** END TODO ***
	}

	///******************* TO DO *********************
	// * AccumulateBlend:
	// *	INPUT:
	// *		img: a new image to be added to acc
	// *		acc: portion of the accumulated image where img is to be added
	// *		blendRadius: radius of the blending function
	// *	OUTPUT:
	// *		add a weighted copy of img to the subimage specified in acc
	// *		the first 3 band of acc records the weighted sum of pixel colors
	// *		the fourth band of acc records the sum of weight
	// */
	static void AccumulateBlend(Mat img, Mat acc, float blendRadius)
	{
		// *** BEGIN TODO ***
		// fill in this routine..
		//acc = (wAcc*acc + wImg*img)/(wAcc+wImg)
		int width = img.cols, height = img.rows;

		for (int y = 0; y < height; y++)
			for (int x = 0; x < width; x++)
			{
				float alpha = (img.at<Vec4f>(Point(x, y))[3] / 255.f);
				for (int c = 0; c < 3; c++)
				{
					acc.at<Vec4f>(Point(x, y))[c] = (acc.at<Vec4f>(Point(x, y))[c] + float(img.at<Vec4f>(Point(x, y))[c] / 255.f * alpha));
				}
				acc.at<Vec4f>(Point(x, y))[3] = acc.at<Vec4f>(Point(x, y))[3] + alpha;

				/*for (int c = 0; c < 3; c++)
				{

					acc.at<Vec4f>(Point(x, y))[c] += float(img.at<Vec4b>(Point(x, y))[c]* img.at<Vec4b>(Point(x, y))[3]);
				}
				acc.at<Vec4f>(Point(x, y))[3] += float (img.at<Vec4b>(Point(x, y))[3]);*/

				/*float alpha = img.at<Vec4b>(Point(x, y))[3] / 255.0f;
				for (int z = 0; z < 3; z++)
					acc.at<Vec4f>(Point(x, y))[z] = acc.at<Vec4f>(Point(x, y))[z] + (float)(acc.at<Vec4f>(Point(x, y))[z] *alpha);
				acc.at<Vec4f>(Point(x, y))[3] = acc.at<Vec4f>(Point(x, y))[3] + float (img.at<Vec4b>(Point(x, y))[3]);*/
			}
		// *** END TODO ***
	}

	///******************* TO DO *********************
	// * NormalizeBlend:
	// *	INPUT:
	// *		acc: input image whose alpha channel (4th channel) contains
	// *		     normalizing weight values
	// *		img: where output image will be stored
	// *	OUTPUT:
	// *		normalize r,g,b values (first 3 channels) of acc and store it into img
	// */
	static void NormalizeBlend(Mat acc, Mat img)
	{
		// *** BEGIN TODO ***
		// fill in this routine..
		int width = acc.cols, height = acc.rows;

		for (int y = 0; y < height; y++) {
			for (int x = 0; x < width; x++)
			{

				for (int c = 0; c < 3; c++)
				{
					img.at<Vec4b>(Point(x, y))[c] = (uchar)255 * (acc.at<Vec4f>(Point(x, y))[c] / acc.at<Vec4f>(Point(x, y))[3]);
				}


				img.at<Vec4b>(Point(x, y))[3] = 255;
				/*float sumAlpha = acc.at<Vec4f>(Point(x, y))[3] / 255;
				for (int z = 0; z < 3; z++)
					img.at<Vec4b>(Point(x, y))[z] = (uchar)(acc.at<Vec4f>(Point(x, y))[z] / sumAlpha);
				img.at<Vec4b>(Point(x, y))[3] = 255;*/
			}
		}

		// *** END TODO ***
	}
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	int main(int argc, char **argv)
	{
		char* home;
		home = getenv("HOME");
		std::string pasta = "C:/Users/julia/Pictures/GERADOR_ICE/trafo2/";
		std::string arquivo_nvm = pasta + "cameras2.nvm";


		/// Le arquivo NVM e carrega tudo nos vetores de variaveis respectivas
		///
		ifstream nvm(arquivo_nvm);
		int contador_linhas = 1;
		vector<Quaternion<float>> rots;
		vector<Vector3f> Cs;
		vector<std::string> nomes_imagens, linhas;
		std::string linha;
		printf("Abrindo e lendo arquivo NVM ...\n");
		if (nvm.is_open()) {
			while (getline(nvm, linha)) {
				if (contador_linhas > 3 && linha.size() > 4)
					linhas.push_back(linha);

				contador_linhas++;
			}
		}
		else {
			printf("Arquivo de cameras nao encontrado. Desligando ...\n");
			return 0;
		}
		// Alocar nos respectivos vetores
		rots.resize(linhas.size()); Cs.resize(linhas.size()), nomes_imagens.resize(linhas.size());
		float foco;
		// Para cada imagem, obter valores
		for (int i = 0; i < linhas.size(); i++) {
			istringstream iss(linhas[i]);
			vector<string> splits(istream_iterator<string>{iss}, istream_iterator<string>());
			// Nome
			string nome_fim = splits[0].substr(splits[0].find_last_of('/') + 1, splits[0].size() - 1);
			nomes_imagens[i] = pasta + nome_fim;
			// Foco
			foco = stof(splits[1]);
			// Quaternion
			Quaternion<float> q;
			q.w() = stof(splits[2]); q.x() = stof(splits[3]); q.y() = stof(splits[4]); q.z() = stof(splits[5]);
			rots[i] = q;
			// Centro
			Vector3f C(stof(splits[6]), stof(splits[7]), stof(splits[8]));
			Cs[i] = C;
		}

		/// Ler todas as nuvens, somar e salvar
		///
		struct stat buffer;
		//string nome_acumulada = pasta + "acumulada_hd.ply";
		//if (stat(nome_acumulada.c_str(), &buffer)) {
		//	printf("Lendo e salvando as nuvens ...\n");
		//	vector<string> nomes_nuvens;
		//	getdir(pasta, nomes_nuvens);
		//	PointCloud<PointTN>::Ptr acc(new PointCloud<PointTN>);
		//	PointCloud<PointC>::Ptr  temp(new PointCloud<PointC>);
		//	PointCloud<PointTN>::Ptr tempn(new PointCloud<PointTN>);
		//	for (int i = 0; i < nomes_nuvens.size(); i++) {
		//		loadPLYFile<PointC>(pasta + nomes_nuvens[i], *temp);
		//		pc.calculateNormals(temp, tempn);
		//		StatisticalOutlierRemoval<PointTN> sor;
		//		sor.setInputCloud(tempn);
		//		sor.setMeanK(20);
		//		sor.setStddevMulThresh(2.5);
		//		sor.filter(*tempn);
		//		*acc += *tempn;
		//	}
		//	savePLYFileBinary<PointTN>(nome_acumulada, *acc);
		//	temp->clear(); tempn->clear(); acc->clear();
		//}

		/// Desenha a ESFERA - adicionando pontos
		///
		PointCloud<PointTN>::Ptr esfera(new PointCloud<PointTN>);
		PointCloud<PointTN>::Ptr esfera1(new PointCloud<PointTN>);
		float R = 1
			; // Raio da esfera [m]
		// Angulos para lat e lon, 360 de range para cada, resolucao a definir no step_deg
		float step_deg = 0.2; // [DEGREES]
		int raios_360 = int(360.0 / step_deg), raios_180 = raios_360 / 2; // Quantos raios sairao do centro para formar 360 e 180 graus de um circulo 2D

		string nome_nuvem_esfera = pasta + "esfera_raw.ply";

		printf("Criando esfera com resolucao de %.4f graus ...\n", step_deg);
		esfera->resize(raios_360*raios_180);
		esfera1->resize(raios_360*raios_180);
		omp_set_dynamic(0);
#pragma omp parallel for num_threads(50)
		for (int a = 0; a < raios_180; a++) {
			for (int o = 0; o < raios_360; o++) {
				PointTN ponto;
				float lat, lon;
				lat = DEG2RAD(a*step_deg), lon = DEG2RAD(o*step_deg);
				// Calculo segundo latitude e longitude do ponto para coordenadas - latitude corre em Y !!!
				ponto.y = R * cos(lat);
				ponto.x = R * sin(lat)*cos(-lon);
				ponto.z = R * sin(lat)*sin(-lon);
				/*ponto.g = 255;
			ponto.r = 255;
			ponto.b = 255;*/
				ponto.normal_x = o; ponto.normal_y = raios_180 - 1 - a; ponto.normal_z = 0;
				// Adiciona ponto na esfera
				esfera->points[a*raios_360 + o] = ponto;
				esfera1->points[a*raios_360 + o] = ponto;
			}
		}

		/// Para cada imagem

		printf("Rodando o processo das imagens sobre a esfera ...\n");
		Mat imagem_esferica[100];


		Mat anterior = Mat::zeros(Size(raios_360, raios_180), CV_8UC3);
		Mat accumulator(imagem_esferica[0].rows, imagem_esferica[0].cols, CV_32FC3, Scalar(0, 0, 0)), compImage(imagem_esferica[0].rows, imagem_esferica[0].cols, CV_8UC3, Scalar(0, 0, 0));
		Mat accumulator1, compImage1;
		cvtColor(accumulator, accumulator1, COLOR_BGR2BGRA);
		cvtColor(compImage, compImage1, COLOR_BGR2BGRA);
		/*omp_set_dynamic(0);
	#pragma omp parallel for num_threads(4)*/
		for (int i = 0; i < nomes_imagens.size(); i++)
		{

			printf("Projetando imagem %d ...\n", i + 1);
			// Ler a imagem a ser usada
			Mat image = imread(nomes_imagens[i]);
			if (image.cols < 3) {
				printf("Imagem nao foi encontrada, checar NVM ...\n");

			}

			//temp.copyTo(image);
	// Calcular a vista da camera pelo Rt inverso - rotacionar para o nosso mundo, com Z para cima
			printf("Calculando pose da camera e frustrum da imagem %d...\n", i + 1);
			Matrix4f T = calculateCameraPose(rots[i], Cs[i], i);
			// Definir o foco em dimensoes fisicas do frustrum
			float F = R;//calculateFrustrumFromPose(rots[i], T, R, Cs[i]);
			Vector3f C = Cs[i];
			double minX, minY, maxX, maxY;
			maxX = F * (float(image.cols) / (2.0*foco));
			minX = -maxX;
			maxY = F * (float(image.rows) / (2.0*foco));
			minY = -maxY;
			// Calcular os 4 pontos do frustrum
			/*
									origin of the camera = p1
									p2--------p3
									|          |
									|  pCenter |<--- Looking from p1 to pCenter
									|          |
									p5--------p4
			*/
			Vector4f p, p1, p2, p3, p4, p5, pCenter;
			float p_dist = sqrt(pow(F, 2) + sqrt(pow(minX, 2) + pow(minY, 2))); // Comprimento da diagonal que vai do centro da camera a cada ponto pX
			p << 0, 0, 0, 1;
			p1 = T * p;
			p << minX, minY, F, 1;
			p2 = T * p;
			p << maxX, minY, F, 1;
			p3 = T * p;
			p << maxX, maxY, F, 1;
			p4 = T * p;
			p << minX, maxY, F, 1;
			p5 = T * p;
			p << 0, 0, F, 1;
			pCenter = T * p;
			// Criar plano colorido a partir dos vetores de canto
			PointCloud<PointTN>::Ptr plano_cam(new PointCloud<PointTN>);
			createFrustrumPlane(plano_cam, p2.block<3, 1>(0, 0), p4.block<3, 1>(0, 0), p5.block<3, 1>(0, 0), image);
			PointTN tempP;
			/*tempP.x = p5(0); tempP.y = p5(1); tempP.z = p5(2);
			tempP.r = 100; tempP.g = 250; tempP.b = 40;
			plano_cam->push_back(tempP);
			tempP.x = p4(0); tempP.y = p4(1); tempP.z = p4(2);
			tempP.r = 0; tempP.g = 250; tempP.b = 0;
			plano_cam->push_back(tempP);
			tempP.x = p3(0); tempP.y = p3(1); tempP.z = p3(2);
			tempP.r = 0; tempP.g = 250; tempP.b = 0;
			plano_cam->push_back(tempP);
			tempP.x = p2(0); tempP.y = p2(1); tempP.z = p2(2);
			tempP.r = 0; tempP.g = 250; tempP.b = 0;
			plano_cam->push_back(tempP);
			tempP.x = p1(0); tempP.y = p1(1); tempP.z = p1(2);
			tempP.r = 0; tempP.g = 250; tempP.b = 0;
			plano_cam->push_back(tempP);
			tempP.x = pCenter(0); tempP.y = pCenter(1); tempP.z = pCenter(2);
			tempP.r = 0; tempP.g = 250; tempP.b = 0;
			plano_cam->push_back(tempP);*/
			//savePLYFileBinary<PointTN>(pasta + "plano_tangente" + std::to_string(i + 1) + ".ply", *plano_cam);
			// Para cada ponto no plano
			printf("Ajustando plano do frustrum %d sobre a esfera ...\n", i + 1);
			KdTreeFLANN<PointTN> tree; // Kdtree utilizada na esfera
			tree.setInputCloud(esfera);
			omp_set_dynamic(0);
#pragma omp parallel for num_threads(100)
			for (int k = 0; k < plano_cam->size(); k++)
			{
				// Achar intersecao entre a reta que sai do centro da camera ate o ponto do plano e a esfera
				PointTN inters = intersectSphere(p1.block<3, 1>(0, 0), plano_cam->points[k], R);
				if (!isnan(inters.x) && !isnan(inters.y) && !isnan(inters.z)) {
					// Encontrar ponto na esfera mais proximo da intersecao por Kdtree
					vector<int> indices;
					vector<float> distancias_elevadas;
					tree.nearestKSearch(inters, 1, indices, distancias_elevadas);

					// Se encontrado, pintar ponto na esfera com a cor do ponto no plano
					if (indices.size() > 0) {

						esfera->points[indices[0]].r = plano_cam->points[k].r;
						esfera->points[indices[0]].g = plano_cam->points[k].g;
						esfera->points[indices[0]].b = plano_cam->points[k].b;

						esfera1->points[indices[0]].r = plano_cam->points[k].r;
						esfera1->points[indices[0]].g = plano_cam->points[k].g;
						esfera1->points[indices[0]].b = plano_cam->points[k].b;
					}
				}
			}

			//imwrite(pasta + "teste.png", anterior);

			imagem_esferica[i] = Mat::zeros(Size(raios_360, raios_180), CV_8UC3);
#pragma omp parallel for num_threads(40)
			for (int j = 0; j < esfera->size(); j++)
			{
				// Coordenada da imagem salva na normal
				int u = esfera->points[j].normal_x, v = esfera->points[j].normal_y;
				// Pega as cores da nuvem ali e coloca na imagem final
				Vec3f cor_im;
				cor_im[0] = esfera->points[j].b; cor_im[1] = esfera->points[j].g; cor_im[2] = esfera->points[j].r;
				imagem_esferica[i].at<Vec3b>(Point(u, v)) = cor_im;
				esfera->points[j].r = 0;
				esfera->points[j].g = 0;
				esfera->points[j].b = 0;
			}
			if (i == 0) {
				anterior = imagem_esferica[i];
				//Mat dst1, dts2;
				////cvtColor(anterior, dst1, COLOR_BGR2BGRA);
				anterior.convertTo(anterior, CV_32F, 1.0 / 255.0);
				imwrite("C:/dataset3/ant.png", imagem_esferica[i]);

				//AccumulateBlend(dst1, accumulator1, 1);
			}
			else {
				imwrite("C:/dataset3/teste.png", imagem_esferica[i]);
				Mat result;

				imagem_esferica[i].convertTo(imagem_esferica[i], CV_32F, 1.0 / 255.0);
				result = multiband_blending(imagem_esferica[i], anterior, i);
				//result = alphaBlending(imagem_esferica[i],anterior);

				anterior = result;

				// anterior.convertTo(anterior, CV_32F, 1.0 / 255.0);
				result.convertTo(result, CV_8UC3, 255);

				imwrite("C:/dataset3/final1.png", result);




			}
		} // Fim do for imagens;











		// Salvar a nuvem da esfera agora colorida
		printf("Salvando esfera final colorida ...\n");
		//	savePLYFileBinary<PointTN>(pasta + "esfera_cor.ply", *esfera);

			 //Salva imagem circular ao final
		printf("Salvando imagem esferica ...\n");
		Mat imagem_esferica1 = Mat::zeros(Size(raios_360, raios_180), CV_8UC3);
		omp_set_dynamic(0);
#pragma omp parallel for num_threads(40)
		for (int i = 0; i < esfera1->size(); i++) {
			// Coordenada da imagem salva na normal
			int u = esfera1->points[i].normal_x, v = esfera1->points[i].normal_y;
			// Pega as cores da nuvem ali e coloca na imagem final
			Vec3b cor_im;
			cor_im[0] = esfera1->points[i].b; cor_im[1] = esfera1->points[i].g; cor_im[2] = esfera1->points[i].r;
			imagem_esferica1.at<Vec3b>(Point(u, v)) = cor_im;
		}
		// Salvando imagem esferica final
		imwrite(pasta + "imagem_esferica_result.png", imagem_esferica1);
		//create blender

		printf("Processo finalizado.");

		return 0;
	}
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
