#include<iostream>
#include<fstream>
#include<opencv2/opencv.hpp>
#include<opencv2/dnn.hpp>
#include <iostream>
#include <fstream>
#include <utility>

#include <Eigen/Core>
#include <Eigen/Dense>
 
#include "tensorflow/cc/ops/const_op.h"
#include "tensorflow/cc/ops/image_ops.h"
#include "tensorflow/cc/ops/standard_ops.h"
 
#include "tensorflow/core/framework/graph.pb.h"
#include "tensorflow/core/framework/tensor.h"
 
#include "tensorflow/core/graph/default_device.h"
#include "tensorflow/core/graph/graph_def_builder.h"
 
#include "tensorflow/core/lib/core/errors.h"
#include "tensorflow/core/lib/core/stringpiece.h"
#include "tensorflow/core/lib/core/threadpool.h"
#include "tensorflow/core/lib/io/path.h"
#include "tensorflow/core/lib/strings/stringprintf.h"
 
#include "tensorflow/core/public/session.h"
#include "tensorflow/core/util/command_line_flags.h"
 
#include "tensorflow/core/platform/env.h"
#include "tensorflow/core/platform/init_main.h"
#include "tensorflow/core/platform/logging.h"
#include "tensorflow/core/platform/types.h"
 
using namespace tensorflow::ops;
using namespace tensorflow;
using namespace std;
using namespace cv;
using tensorflow::Flag;
using tensorflow::Tensor;
using tensorflow::Status;
using tensorflow::string;
using tensorflow::int32 ;
using namespace cv::dnn;

// 定义一个函数将Opencv的Mat数据转化为tensorflow的tensor，在python里面只要对cv2.imread()读进来的矩阵进行np.reshape之后，数据类型就成了一个tensor，即tensor与矩阵一样，然后就连可以输入到网络的入口了
// 而C++版本，网络的输入也需要是tensor数据类型，因此需要将输入图片转换成一个tensor，若使用Opencv读取图片，格式是一个Mat，需要考虑怎样将一个Mat转换为tensor
void CVMat_to_Tensor(cv::Mat img,tensorflow::Tensor * output_tensor,int input_rows, int input_cols)
{
	imshow("input image",img);
	// 对输入图像进行resize处理
	resize(img,img,cv::Size(input_cols,input_rows));
	imshow("resizes image",img);
 
	// 归一化
	img.convertTo(img,CV_32FC1);
	img = 1 - img/255;
 
	//创建一个指向tensor的内容指针
	float * p = output_tensor->flat<float>().data();
 
	// 创建一个Mat，与tensor的指针进行绑定，改变这个Mat的值，就相当于改变tensor的值
	cv::Mat tempMat(input_rows,input_cols,CV_32FC1,p);
	img.convertTo(tempMat,CV_32FC1);
 
	cv::waitKey(1000);
	cv::destroyAllWindows();
}
 
int main(int argc, char ** argv)
{
	/* --------------------配置关键信息------------------------------------*/
	std::string model_path = "./model/inception_v3_2016_08_28_frozen.pb"; // pb模型地址
	std::string image_path = "./model/cat.jpg"; // 测试图片
	int input_height = 299; // 输入网络的图片高度
	int input_width = 299; // 输入网络的图片宽度
	std::string input_tensor_name = "input"; // 网络的输入节点的名称
	std::string output_tensor_name = "InceptionV3/Predictions/Reshape_1"; // 网络的输出节点的名称
 
	/* --------------------创建session------------------------------------*/
	tensorflow::Session * session;
	tensorflow::Status status = tensorflow::NewSession(tensorflow::SessionOptions(), &session); // 创建新会话Session
 
	/* --------------------从pb文件中读取模型------------------------------------*/
	tensorflow::GraphDef graphdef; //为当前的模型定义一张图
	tensorflow::Status status_load = tensorflow::ReadBinaryProto(tensorflow::Env::Default(),model_path,&graphdef); // 从pb文件中读取图模型
	if (!status_load.ok()) // 判断读取模型是否正确，错误的话则打印出错误的信息
	{
		std::cout << "ERROR: Loading model failed..." << model_path << std::endl;
		std::cout << status_load.ToString() << "\n";
		return -1;
	}
	tensorflow::Status status_create = session->Create(graphdef); // 将模型导入会话Session中
	if (!status_create.ok()) // 判断将模型导入会话中是否成功，错误的话打印出错误信息
	{
		std::cout << "ERROR: Creating graph in session failed..." << status_create.ToString() << std::endl;
		return -1;
	}
	std::cout << "<------Sucessfully created session and load graph------>" << std::endl;
 
	/* --------------------载入测试图片------------------------------------*/
	cv::Mat img = cv::imread(image_path,0); // 读取图片，读取灰度图
	if (img.empty())
	{
		std::cout << "can't open the image!!!!!" << std::endl;
		return -1;
	}
	// 创建一个tensor作为输入网络的接口
	tensorflow::Tensor resized_tensor(tensorflow::DT_FLOAT,tensorflow::TensorShape({1,input_height,input_width,3}));
	// 将opencv读取的Mat格式的图片存入tensor
	CVMat_to_Tensor(img,&resized_tensor,input_height,input_width);
	std::cout << resized_tensor.DebugString() << std::endl;
 
	/* --------------------用网络进行测试------------------------------------*/
	std::cout << std::endl << "<------------------Runing the model with test_image------------------->" << std::endl;
	// 前向运行，输出结果一定是一个tensor的vector
	std::vector<tensorflow::Tensor> outputs;
	std::string output_node = output_tensor_name; // 输出节点名
	tensorflow::Status status_run = session->Run({{input_tensor_name,resized_tensor}},{output_node},{},&outputs);
	if (!status_run.ok())
	{
		std::cout << "ERROR: Run failed..." << std::endl;
		std::cout << status_run.ToString() << std::endl;
		return -1;
	}
 
	// 把输出值提取出来
	std::cout << "Output tensor size: " << outputs.size() << std::endl;
	for (std::size_t i = 0; i < outputs.size();i++)
	{
		std::cout << outputs[i].DebugString() << std::endl;
	}
	tensorflow::Tensor t = outputs[0];
	auto tmap = t.tensor<float,2>();
	int output_dim = t.shape().dim_size(1);
 
	int output_class_id = -1;
	double output_prob = 0.0;
	for (int j = 0; j < output_dim; j++)
	{
		std::cout << "Class " << j << " prob: " << tmap(0,j) << "," << std::endl;
		if (tmap(0,j) >= output_prob)
		{
			output_class_id = j;
			output_prob = tmap(0,j);
		}
	}
	// 输出结果
	std::cout << "Final class id : " << output_class_id << std::endl;
	std::cout << "Final class prob : " << output_prob << std::endl;
}




// int main()
// {
// 	//读取视频或摄像头
// 	cv::VideoCapture capture(0);
 
// 	while (true)
// 	{
// 		Mat frame;
// 		capture >> frame;
// 		cv::imshow("读取视频", frame);
// 		cv::waitKey(30);	//延时30
// 	}
// 	return 0;
// }