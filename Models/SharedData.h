#pragma once
#include <opencv2/opencv.hpp>
#include <vector>

#include <voxel3d.h>

struct SensorData
{
	
	cv::Mat depth;
	cv::Mat conf;
	cv::Mat flir;
	cv::Mat rgb;

	cv::Mat pointCloudXYZ;

	IMU_DATA imuData = { 0, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f };

	std::vector<int> rectData;
	std::vector<float> cubeData;

	SensorData() = default;

	SensorData(
		int tofWidth, int tofHeight, 
		int rgbWdth, int rgbHeight,
		int flirWidth, int flirHeight)
	{
		depth = cv::Mat::zeros(tofHeight, tofWidth, CV_16UC1);
		conf = cv::Mat::zeros(tofHeight, tofWidth, CV_16UC1);
		flir = cv::Mat::zeros(flirHeight, flirWidth, CV_32FC1);
		rgb = cv::Mat::zeros(rgbHeight, rgbWdth, CV_8UC3);
		pointCloudXYZ = cv::Mat::zeros(tofHeight, tofWidth, CV_32FC3);

		rectData.resize(tofHeight * tofWidth, 0);
		cubeData.resize(tofHeight * tofWidth * 3, 0.0f);
	}
};


struct DisplayData
{
	cv::Mat DisplayDepth;
	cv::Mat DisplayRgb;
	cv::Mat DisplayIr;
	cv::Mat DisplayFlir;

	DisplayData(int imgH, int imgW)
	{
		DisplayDepth = cv::Mat::zeros(imgH, imgW, CV_8UC3);
		DisplayRgb = cv::Mat::zeros(imgH, imgW, CV_8UC3);
		DisplayIr = cv::Mat::zeros(imgH, imgW, CV_8UC1);
		DisplayFlir = cv::Mat::zeros(imgH, imgW, CV_8UC1);
	}
};




// struct Convex
// {
// 	int num = 0;
// 	float height = 0;
// 	float depth = 0;
// 	float data[30] = {0};
// };


// struct Point
// {
// 	float x = 0;
// 	float y = 0;
// 	float z = 0;
// };

// struct Boundary
// {
// 	Point boundary[8];
// };


// struct RoiData
// {
// 	Boundary boundarySet[10];
// 	Convex convex[10];
// };



// struct AcaasResult
// {
// 	int segNumber;
// 	float planeBoundary[400] = { 0 };
// 	float wordSpaceRotationMatrix[9] = { 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f };
// 	std::vector<float> pointCloud2GL;
// 	std::vector<float> pointCloudColor;
// 	int sensorHeight;
// 	RoiData roiData;

// 	AcaasResult() = default;

// 	AcaasResult(int imgH, int imgW)
// 	{
// 		segNumber = 0;
// 		pointCloud2GL.resize(imgH * imgW * 3, 0.0f);
// 		pointCloudColor.resize(imgH * imgW * 3, 0.0f);
// 	}
// };




struct SensorInfo
{
	bool IsExist;

	int Height;
	int Width;

	float hfov;
	float vfov;

	float fps;
	
	CameraInfo Info;
	std::vector<float> Params;

	SensorInfo()
	{
		IsExist = false;
		Height = 0;
		Width = 0;

		hfov = -1.0f;
		vfov = -1.0f;
		
		fps = -1.0f;

		Params.resize(12, 0);
		memset(&Info, 0x0, sizeof(Info));
	}

};

struct hiRabSensorInfo
{
	SensorInfo rgbInfo;
	SensorInfo tofInfo;
	SensorInfo flirInfo;
};



// struct YoloRes {
// 	int trackId = -1; 
// 	std::string label = "";
// 	int x0 = -1;
// 	int y0 = -1;
// 	int x1 = 0;
// 	int y1 = 0;
// 	float depth = 0.f;
// };


// struct AiResult {
// 	int num = 0;
// 	YoloRes yoloRes[20];

// 	AiResult() = default;
// };