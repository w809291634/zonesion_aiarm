#include<iostream>
#include<fstream>
#include<opencv2/opencv.hpp>
#include<opencv2/dnn.hpp>
 
// #include <Windows.h>
// #pragma comment( lib, "winmm.lib")
 
using namespace std;
using namespace cv;
using namespace cv::dnn;
 
Mat predict(Net net, char * dir)
{
	Mat img = imread(dir, -1);
	Mat inputBlob = blobFromImage(img, 1.0 / 255.0, Size(340, 340), Scalar(), false, false);
 
	net.setInput(inputBlob);
 
	Mat detection = net.forward();
 
	return detection;
}


// int main(int argc, char *argv[])
// {
// 	string strResult, strTemp;
// 	char czPath[256] = { 0 };


// 	GetModuleFileName(NULL, czPath, 256);  //获取全路径


// 	string strPath = czPath;
// 	cout << "文件名(带全路径) : " << strPath.c_str() << endl;
// 	int nPos = strPath.find('x'); //查找最后一个 \ 出现的位置
// 	strTemp = strPath.substr(0, nPos);
// 	String weight = strTemp+"tmp\\new_tensor_model.pb";
// 	cout<< weight<< endl;
 
// 	Net net = readNetFromTensorflow(weight);
 
// 	//设置计算后台（GPU调用，如需修改按ctrl+点击setPreferableTarget）
// 	net.setPreferableBackend(DNN_BACKEND_OPENCV);
// 	net.setPreferableTarget(DNN_TARGET_OPENCL);
 
// 	/*打出节点
//     vector<string> outLayersNames = net.getLayerNames();
// 	int lens = outLayersNames.size();
// 	cout << lens << endl;
// 	for(int i = 0; i < lens; i++)
// 	{ 
// 		cout << outLayersNames[i] << endl;
// 	}*/
 
 
// 	DWORD t1, t2;
// 	char dir[] = "D:/hyd/silk_baby2/si/2/l2_9.jpg";
// 	Mat detection1 = predict(net, dir);
// 	t1 = GetTickCount();
// 	Mat detection = predict(net, dir);
// 	cout << detection << endl;
// 	t2 = GetTickCount();
// 	cout<< (t2 - t1)*1.0 / 1000 << endl;
// }


using namespace cv;
using namespace std;
 
int main()
{
	//读取视频或摄像头
	cv::VideoCapture capture(4);
 
	while (true)
	{
		Mat frame;
		capture >> frame;
		cv::imshow("读取视频", frame);
		cv::waitKey(30);	//延时30
	}
	return 0;
}