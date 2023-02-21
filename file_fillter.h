#pragma once
#include <io.h>
#include <fstream>
#include <string>
#include <vector>
#include <iostream>


//获取所有的文件名
void GetAllFiles(std::string path, std::vector<std::string>& files)
{

	long   hFile = 0;
	//文件信息  
	struct _finddata_t fileinfo;
	std::string p;
	if ((hFile = _findfirst(p.assign(path).append("\\*").c_str(), &fileinfo)) != -1)
	{
		do
		{
			if ((fileinfo.attrib & _A_SUBDIR))
			{
				if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
				{
					files.push_back(p.assign(path).append("\\").append(fileinfo.name));
					GetAllFiles(p.assign(path).append("\\").append(fileinfo.name), files);
				}
			}
			else
			{
				files.push_back(p.assign(path).append("\\").append(fileinfo.name));
			}

		} while (_findnext(hFile, &fileinfo) == 0);

		_findclose(hFile);
	}

}

//获取特定格式的文件名
void GetAllFormatFiles(std::string path, std::vector<std::string>& files, std::string format)
{
	//文件句柄  
	_int64   hFile = 0;
	//文件信息  
	struct _finddata_t fileinfo;
	std::string p;
	if ((hFile = _findfirst(p.assign(path).append("\\*" + format).c_str(), &fileinfo)) != -1)
	{
		do
		{
			if ((fileinfo.attrib & _A_SUBDIR))
			{
				if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
				{
					//files.push_back(p.assign(path).append("\\").append(fileinfo.name) );
					GetAllFormatFiles(p.assign(path).append("/").append(fileinfo.name), files, format);
				}
			}
			else
			{
				files.push_back(p.assign(path).append("/").append(fileinfo.name));
			}
		} while (_findnext(hFile, &fileinfo) == 0);

		_findclose(hFile);
	}
}

// 该函数有两个参数，第一个为路径字符串(string类型，最好为绝对路径)；
// 第二个参数为文件夹与文件名称存储变量(vector类型,引用传递)。
// 在主函数中调用格式(并将结果保存在文件"AllFiles.txt"中，第一行为总数)：

//int main()
//{
//	string filePath = "testimages\\water";
//	vector<string> files;
//	char* distAll = "AllFiles.txt";
//
//	//读取所有的文件，包括子文件的文件
//	//GetAllFiles(filePath, files);
//
//	//读取所有格式为jpg的文件
//	string format = ".jpg";
//	GetAllFormatFiles(filePath, files, format);
//	ofstream ofn(distAll);
//	int size = files.size();
//	ofn << size << endl;
//	for (int i = 0;i < size;i++)
//	{
//		ofn << files[i] << endl;
//		cout << files[i] << endl;
//	}
//	ofn.close();
//	return 0;
//}


