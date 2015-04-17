// KinectApplication.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <iostream>
#include <Windows.h>
#include <Kinect.h>

using namespace std;

template<class Interface>
inline void SafeRelease(Interface*& ppInterfaceToRelease){
	if (ppInterfaceToRelease != NULL){
		ppInterfaceToRelease->Release();
		ppinterfaceToRelease = NULL;
	}
}


int _tmain(int argc, _TCHAR* argv[]) {
	//Sensor
	IKinectSensor* pSensor;
	HRESULT hResult = S_OK;

	hResult = GetDefaultKinectSensor(&pSensor);
	if (FAILED(hResult)){
		std:cerr << "ERROR: GetDefaultKinectSensor()" << std::endl;
		return -1;
	}

	hResult = pSensor->Open();
	if (FAILED(hResult)){
		std:cerr << "ERROR: IKinectSensor::Open()" << std::endl;
		return -1;
	}

	//Source
	IColorFrameSource* pSource;
	hResult = pSensor->get_ColorFrameSource(&pSource);
	if (FAILED(hResult)){
		std:cerr << "ERROR: IKinectSensor::get_ColorFrameSource()" << std::endl;
		return -1;
	}

	//Reader
	IColorFrameReader* pReader;
	hResult = pSource->OpenReader(&pReader);
	if (FAILED(hResult)){
		std:cerr << "ERROR: IColorFrameSource::OpenReader()" << std::endl;
		return -1;
	}

	while (1){
		//Frame
		IColorFrame* pFrame = nullptr;
		hResult = pReader->AcquireLatestFrame(&pFrame);
		if (SUCCEEDED(hResult)){
			//Data
		}

		SafeRelease(pFrame);
	}

	return 0;
}

