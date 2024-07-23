#include <gtest/gtest.h>
#include <iostream>
#include "DenseCloud/PatchMatchStereo.h"
#include <chrono>
#include <opencv2/opencv.hpp>

using namespace std::chrono;

/*��ʾ�Ӳ�ͼ*/
void ShowDisparityMap(const float32* disp_map, const sint32& width, const sint32& height, const std::string& name);
/*�����Ӳ�ͼ*/
void SaveDisparityMap(const float32* disp_map, const sint32& width, const sint32& height, const std::string& path);
/*�����Ӳ����*/
void SavePointCloud(const uint8* img_bytes, const float32* disp_map, const sint32& width, const sint32& height, const std::string& path);


TEST(MYTEST0, A)
{
	// ��ȡӰ��
	std::string path_left = "C:/codes/MyMVS/Datas/Cone/im2.png";
	std::string path_right = "C:/codes/MyMVS/Datas/Cone/im6.png";

	cv::Mat img_left = cv::imread(path_left, cv::IMREAD_COLOR);
	cv::Mat img_right = cv::imread(path_right, cv::IMREAD_COLOR);

	if (img_left.data == nullptr || img_right.data == nullptr) {
		std::cout << "��ȡӰ��ʧ�ܣ�" << std::endl;
		return;
	}
	if (img_left.rows != img_right.rows || img_left.cols != img_right.cols) {
		std::cout << "����Ӱ��ߴ粻һ�£�" << std::endl;
		return;
	}


	//��������������������������������������������������������������������������������������������������������������������������������������������������������������//
	const sint32 width = static_cast<uint32>(img_left.cols);
	const sint32 height = static_cast<uint32>(img_right.rows);

	// ����Ӱ��Ĳ�ɫ����
	auto bytes_left = new uint8[width * height * 3];
	auto bytes_right = new uint8[width * height * 3];
	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			bytes_left[i * 3 * width + 3 * j] = img_left.at<cv::Vec3b>(i, j)[0];
			bytes_left[i * 3 * width + 3 * j + 1] = img_left.at<cv::Vec3b>(i, j)[1];
			bytes_left[i * 3 * width + 3 * j + 2] = img_left.at<cv::Vec3b>(i, j)[2];
			bytes_right[i * 3 * width + 3 * j] = img_right.at<cv::Vec3b>(i, j)[0];
			bytes_right[i * 3 * width + 3 * j + 1] = img_right.at<cv::Vec3b>(i, j)[1];
			bytes_right[i * 3 * width + 3 * j + 2] = img_right.at<cv::Vec3b>(i, j)[2];
		}
	}
	printf("Done!\n");

	// PMSƥ��������
	PMSOption pms_option;
	// patch��С
	pms_option.patch_size = 35;
	// ��ѡ�ӲΧ
	pms_option.min_disparity = 0;
	pms_option.max_disparity = 64;
	// gamma
	pms_option.gamma = 10.0f;
	// alpha
	pms_option.alpha = 0.9f;
	// t_col
	pms_option.tau_col = 10.0f;
	// t_grad
	pms_option.tau_grad = 2.0f;
	// ������������
	pms_option.num_iters = 3;

	// һ���Լ��
	pms_option.is_check_lr = true;
	pms_option.lrcheck_thres = 1.0f;
	// �Ӳ�ͼ���
	pms_option.is_fill_holes = false;

	// ǰ��ƽ�д���
	pms_option.is_fource_fpw = false;

	// �����Ӳ��
	pms_option.is_integer_disp = false;

	// ����PMSƥ����ʵ��
	PatchMatchStereo pms;

	printf("PatchMatch Initializing...");
	auto start = std::chrono::steady_clock::now();
	//��������������������������������������������������������������������������������������������������������������������������������������������������������������//
	// ��ʼ��
	if (!pms.Initialize(width, height, pms_option)) {
		std::cout << "PMS��ʼ��ʧ�ܣ�" << std::endl;
		return;
	}
	auto end = std::chrono::steady_clock::now();
	auto tt = duration_cast<std::chrono::milliseconds>(end - start);
	printf("Done! Timing : %lf s\n", tt.count() / 1000.0);

	printf("PatchMatch Matching...");
	start = std::chrono::steady_clock::now();
	//��������������������������������������������������������������������������������������������������������������������������������������������������������������//
	// ƥ��
	// disparity���鱣�������ص��Ӳ���
	auto disparity = new float32[uint32(width * height)]();
	if (!pms.Match(bytes_left, bytes_right, disparity)) {
		std::cout << "PMSƥ��ʧ�ܣ�" << std::endl;
		return;
	}
	end = std::chrono::steady_clock::now();
	tt = duration_cast<std::chrono::milliseconds>(end - start);
	printf("Done! Timing : %lf s\n", tt.count() / 1000.0);

#if 0
	// ��ʾ�ݶ�ͼ
	cv::Mat grad_left_x = cv::Mat(height, width, CV_8UC1);
	auto* grad_map = pms.GetGradientMap(0);
	if (grad_map) {
		for (sint32 i = 0; i < height; i++) {
			for (sint32 j = 0; j < width; j++) {
				const auto grad = grad_map[i * width + j].x;
				grad_left.data[i * width + j] = grad;
			}
		}
	}
	cv::imshow("�ݶ�ͼ-��X", grad_left);
	cv::Mat grad_left_y = cv::Mat(height, width, CV_8UC1);
	auto* grad_map = pms.GetGradientMap(0);
	if (grad_map) {
		for (sint32 i = 0; i < height; i++) {
			for (sint32 j = 0; j < width; j++) {
				const auto grad = grad_map[i * width + j].y;
				grad_left.data[i * width + j] = grad;
			}
		}
	}
	cv::imshow("�ݶ�ͼ-��Y", grad_left);
#endif

	//��������������������������������������������������������������������������������������������������������������������������������������������������������������//
	// ��ʾ�Ӳ�ͼ
	ShowDisparityMap(pms.GetDisparityMap(0), width, height, "disp-left");
	ShowDisparityMap(pms.GetDisparityMap(1), width, height, "disp-right");
	// �����Ӳ�ͼ
	SaveDisparityMap(pms.GetDisparityMap(0), width, height, path_left);
	SaveDisparityMap(pms.GetDisparityMap(1), width, height, path_right);
	// �������
	SavePointCloud(bytes_left, pms.GetDisparityMap(0), width, height, path_left);
	SavePointCloud(bytes_right, pms.GetDisparityMap(1), width, height, path_left);

	cv::waitKey(0);

	//��������������������������������������������������������������������������������������������������������������������������������������������������������������//
	// �ͷ��ڴ�
	delete[] disparity;
	disparity = nullptr;
	delete[] bytes_left;
	bytes_left = nullptr;
	delete[] bytes_right;
	bytes_right = nullptr;

	return;
}


void ShowDisparityMap(const float32* disp_map, const sint32& width, const sint32& height, const std::string& name)
{
	// ��ʾ�Ӳ�ͼ
	const cv::Mat disp_mat = cv::Mat(height, width, CV_8UC1);
	float32 min_disp = float32(width), max_disp = -float32(width);
	for (sint32 i = 0; i < height; i++) {
		for (sint32 j = 0; j < width; j++) {
			const float32 disp = abs(disp_map[i * width + j]);
			if (disp != Invalid_Float) {
				min_disp = std::min(min_disp, disp);
				max_disp = std::max(max_disp, disp);
			}
		}
	}
	for (sint32 i = 0; i < height; i++) {
		for (sint32 j = 0; j < width; j++) {
			const float32 disp = abs(disp_map[i * width + j]);
			if (disp == Invalid_Float) {
				disp_mat.data[i * width + j] = 0;
			}
			else {
				disp_mat.data[i * width + j] = static_cast<uchar>((disp - min_disp) / (max_disp - min_disp) * 255);
			}
		}
	}

	cv::imshow(name, disp_mat);
	cv::Mat disp_color;
	applyColorMap(disp_mat, disp_color, cv::COLORMAP_JET);
	cv::imshow(name + "-color", disp_color);

}

void SaveDisparityMap(const float32* disp_map, const sint32& width, const sint32& height, const std::string& path)
{
	// �����Ӳ�ͼ
	const cv::Mat disp_mat = cv::Mat(height, width, CV_8UC1);
	float32 min_disp = float32(width), max_disp = -float32(width);
	for (sint32 i = 0; i < height; i++) {
		for (sint32 j = 0; j < width; j++) {
			const float32 disp = abs(disp_map[i * width + j]);
			if (disp != Invalid_Float) {
				min_disp = std::min(min_disp, disp);
				max_disp = std::max(max_disp, disp);
			}
		}
	}
	for (sint32 i = 0; i < height; i++) {
		for (sint32 j = 0; j < width; j++) {
			const float32 disp = abs(disp_map[i * width + j]);
			if (disp == Invalid_Float) {
				disp_mat.data[i * width + j] = 0;
			}
			else {
				disp_mat.data[i * width + j] = static_cast<uchar>((disp - min_disp) / (max_disp - min_disp) * 255);
			}
		}
	}

	cv::imwrite(path + "-d.png", disp_mat);
	cv::Mat disp_color;
	applyColorMap(disp_mat, disp_color, cv::COLORMAP_JET);
	cv::imwrite(path + "-c.png", disp_color);
}

void SavePointCloud(const uint8* img_bytes, const float32* disp_map, const sint32& width, const sint32& height, const std::string& path)
{
	// ��ע�⣬��ͬ���ݣ�������һ�������޸����в���ֵ
	float32 B = 193.001;		// ����
	float32 f = 999.421;		// ����
	float32 x0l = 294.182;		// ����ͼ������x0
	float32 y0l = 252.932;		// ����ͼ������y0
	float32 x0r = 326.95975;	// ����ͼ������x0


	// �������
	FILE* fp_disp_cloud = nullptr;
	fopen_s(&fp_disp_cloud, (path + "-cloud.txt").c_str(), "w");
	if (fp_disp_cloud) {
		for (sint32 y = 0; y < height; y++) {
			for (sint32 x = 0; x < width; x++) {
				const float32 disp = abs(disp_map[y * width + x]);
				if (disp == Invalid_Float) {
					continue;
				}
				float32 Z = B * f / (disp + (x0r - x0l));
				float32 X = Z * (x - x0l) / f;
				float32 Y = Z * (y - y0l) / f;
				// X Y Z R G B
				fprintf_s(fp_disp_cloud, "%f %f %f %d %d %d\n", X, Y,
					Z, img_bytes[y * width * 3 + 3 * x + 2], img_bytes[y * width * 3 + 3 * x + 1], img_bytes[y * width * 3 + 3 * x]);
			}
		}
		fclose(fp_disp_cloud);
	}
}