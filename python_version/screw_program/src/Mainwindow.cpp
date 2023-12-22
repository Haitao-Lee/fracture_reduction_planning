#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkGlyph3D.h>
#include <vtkSphereSource.h>
#include <vtkActor.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkNamedColors.h>
#include <vtkOBBTree.h>
#include <vtkSurfaceReconstructionFilter.h>
#include <vtkContourFilter.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkSTLWriter.h>
#include <vtkDelaunay2D.h>

#include <io.h>
#include <fstream>
#include <string>
#include <vector>
#include <iostream>
#include<iostream>
#include<filesystem>

using namespace std;

//��ȡ�ض���ʽ���ļ���
void getFiles(string path, vector<string>& files)
{
	intptr_t   hFile = 0;//�ļ����
	struct _finddata_t fileinfo;//�ļ���Ϣ
	string p;
	if ((hFile = _findfirst(p.assign(path).append("//*.txt").c_str(), &fileinfo)) != -1)
		//������ҵ���һ���ļ�
	{
		do
		{
			if ((fileinfo.attrib & _A_SUBDIR))//������ļ���
			{
				if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
					getFiles(p.assign(path).append("\\").append(fileinfo.name), files);
				;
			}
			else//������ļ�
			{
				files.push_back(p.assign(path).append("\\").append(fileinfo.name));
			}
		} while (_findnext(hFile, &fileinfo) == 0);    //��Ѱ�ҵ������ļ�

		_findclose(hFile);    //�������ң��رվ��
	}
}

int main()
{
	////////////////////// ����Ĺ� ////////////////////////////////
	// ��ȡ�ļ��������е����ļ�
	//string path_root = "points_data/image0011/"; // ���ս��
	string path_root = "points_data_cluster/image0087/"; // ������
	//string path_root = "points_data_ori/image0087/"; // ��ʼ�ؽ����
	string path = path_root;
	path.append("left");
	vector<string> files_left;
	getFiles(path, files_left);

	int size = files_left.size();

	auto renderer = vtkSmartPointer<vtkRenderer>::New();

	float radius = 1.2; //���õ���ֱ��

	int isBBX = 1;//1������Χ�� 0�ް�Χ��

	float opacity = 0.3;//��Χ�в�͸����

	for (int i = 0; i < size; i++)
	{
		vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
		vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New();
		ifstream fs(files_left[i]);
		vtkIdType idtype;
		double x, y, z;
		while (fs >> x >> y >> z)
		{
			idtype = points->InsertNextPoint(x, y, z);
			cells->InsertNextCell(1, &idtype);
		}

		vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
		polydata->SetPoints(points);

		vtkSmartPointer<vtkSphereSource> SphereSource = vtkSmartPointer<vtkSphereSource>::New();
		SphereSource->SetRadius(radius);
		vtkSmartPointer<vtkGlyph3D> Glyph3D = vtkSmartPointer<vtkGlyph3D>::New();
		Glyph3D->SetSourceConnection(SphereSource->GetOutputPort());
		Glyph3D->SetInputData(polydata);
		Glyph3D->Update();
		vtkSmartPointer<vtkPolyDataMapper> mapperForView = vtkSmartPointer<vtkPolyDataMapper>::New();
		mapperForView->SetInputData(Glyph3D->GetOutput());
		vtkSmartPointer<vtkActor> actorPoints = vtkSmartPointer<vtkActor>::New();
		//actorPoints->GetProperty()->SetDiffuse(0.4); //������
		//actorPoints->GetProperty()->SetAmbient(0.4); //������
		actorPoints->GetProperty()->SetSpecular(0.3);//�߹�ϵ��
		actorPoints->GetProperty()->SetSpecularPower(7);//�߹�ǿ�� 
		actorPoints->SetMapper(mapperForView);

		double color[3] = { 0 };
		if (i == 0)
		{
			/*actorPoints->GetProperty()->SetColor(248.0 / 255.0, 248.0 / 255.0, 255.0 / 255.0);*/
			/*color[0] = 255.0 / 255.0;
			color[1] = 243.0 / 255.0;
			color[2] = 21*/9.0 / 255.0;

			color[0] = 211.0 / 255.0;
			color[1] = 211.0 / 255.0;
			color[2] = 211.0 / 255.0;

			//actorPoints->GetProperty()->SetDiffuse(0.5); //������
			//actorPoints->GetProperty()->SetAmbient(0.4); //������
			actorPoints->GetProperty()->SetColor(color);
			//actorPoints->GetProperty()->SetColor(255.0 / 255.0, 243.0 / 255.0, 219.0 / 255.0);
		}

		//else
		//{
		//	color[0] = 255.0 / 255.0;
		//	color[1] = 176.0 / 255.0;
		//	color[2] = 176.0 / 255.0;

		//	actorPoints->GetProperty()->SetColor(color);
		//}

		else if (i == 1)
		{
			// ��ɫ
			//color[0] = 255.0 / 255.0;
			//color[1] = 176.0 / 255.0;
			//color[2] = 176.0 / 255.0;

			color[0] = 200.0 / 255.0;
			color[1] = 36.0 / 255.0;
			color[2] = 35.0 / 255.0;


			actorPoints->GetProperty()->SetColor(color);
		}

		else if (i == 2)
		{
			// ��ɫ
			//color[0] = 136.0 / 255.0;
			//color[1] = 194.0 / 255.0;
			//color[2] = 225.0 / 255.0;

			color[0] = 40.0 / 255.0;
			color[1] = 120.0 / 255.0;
			color[2] = 181.0 / 255.0;

			actorPoints->GetProperty()->SetColor(color);
		}
		else if (i == 3)
		{
			//��ɫ
			//color[0] = 111.0 / 255.0;
			//color[1] = 182.0 / 255.0;
			//color[2] = 166.0 / 255.0;

			color[0] = 50.0 / 255.0;
			color[1] = 184.0 / 255.0;
			color[2] = 151.0 / 255.0;

			actorPoints->GetProperty()->SetColor(color);
		}

		else if (i == 4)
		{
			// ��ɫ
			//color[0] = 250.0 / 255.0;
			//color[1] = 127.0 / 255.0;
			//color[2] = 111.0 / 255.0;

			color[0] = 248.0 / 255.0;
			color[1] = 172.0 / 255.0;
			color[2] = 140.0 / 255.0;

			actorPoints->GetProperty()->SetColor(color);
		}

		else if (i == 5)
		{
			// ��ɫ
			color[0] = 190.0 / 255.0;
			color[1] = 184.0 / 255.0;
			color[2] = 220.0 / 255.0;


			actorPoints->GetProperty()->SetColor(color);
		}

		else if (i > 5)
		{
			// ����ɫ
			color[0] = 84.0 / 255.0;
			color[1] = 179.0 / 255.0;
			color[2] = 69.0 / 255.0;

			actorPoints->GetProperty()->SetColor(color);
		}

		renderer->AddActor(actorPoints);

		// �����Χ��
		if (i != 0 && isBBX == 1)
		{
			////////////////////////////// �����Χ�м�������Ϣ  ///////////////////////////////////
			//vtkSmartPointer<vtkNamedColors> colors = vtkSmartPointer<vtkNamedColors>::New();
			int maxLevel = 10;
			vtkSmartPointer<vtkOBBTree> obbTree = vtkSmartPointer<vtkOBBTree>::New();
			obbTree->SetDataSet(Glyph3D->GetOutput());
			obbTree->SetMaxLevel(maxLevel);
			obbTree->BuildLocator();

			double corner[3] = { 0 };
			double max_obb[3] = { 0 };
			double mid_obb[3] = { 0 };
			double min_obb[3] = { 0 };
			double size[3] = { 0 };

			obbTree->ComputeOBB(Glyph3D->GetOutput(), corner, max_obb, mid_obb, min_obb, size);

			vtkSmartPointer<vtkPolyData> obbPolydata = vtkSmartPointer<vtkPolyData>::New();
			obbTree->GenerateRepresentation(0, obbPolydata);

			vtkSmartPointer<vtkPolyDataMapper> obbTreeMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
			obbTreeMapper->SetInputData(obbPolydata);

			vtkSmartPointer<vtkActor> obbTreeActor = vtkSmartPointer<vtkActor>::New();
			obbTreeActor->SetMapper(obbTreeMapper);
			obbTreeActor->GetProperty()->SetInterpolationToFlat();
			obbTreeActor->GetProperty()->SetOpacity(opacity);
			obbTreeActor->GetProperty()->SetColor(color);
			renderer->AddActor(obbTreeActor);
		}	
	}

	////////////////////// �Ҳ��Ĺ� ////////////////////////////////
	// ��ȡ�ļ��������е����ļ�
	string path_right = path_root;
	path_right.append("right");
	vector<string> files_right;
	getFiles(path_right, files_right);

	int size_right = files_right.size();

	for (int i = 0; i < size_right; i++)
	{
		vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
		vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New();
		ifstream fs(files_right[i]);
		vtkIdType idtype;
		double x, y, z;
		while (fs >> x >> y >> z)
		{
			idtype = points->InsertNextPoint(x, y, z);
			cells->InsertNextCell(1, &idtype);
		}

		vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
		polydata->SetPoints(points);
		vtkSmartPointer<vtkSphereSource> SphereSource = vtkSmartPointer<vtkSphereSource>::New();
		SphereSource->SetRadius(radius);
		//SphereSource->SetRadius(0);
		//SphereSource->SetRadius(1.0);
		vtkSmartPointer<vtkGlyph3D> Glyph3D = vtkSmartPointer<vtkGlyph3D>::New();
		Glyph3D->SetSourceConnection(SphereSource->GetOutputPort());
		Glyph3D->SetInputData(polydata);
		Glyph3D->Update();
		vtkSmartPointer<vtkPolyDataMapper> mapperForView = vtkSmartPointer<vtkPolyDataMapper>::New();
		mapperForView->SetInputData(Glyph3D->GetOutput());
		vtkSmartPointer<vtkActor> actorPoints = vtkSmartPointer<vtkActor>::New();
		//actorPoints->GetProperty()->SetDiffuse(0.8); //������
		//actorPoints->GetProperty()->SetAmbient(0.4); //������
		actorPoints->GetProperty()->SetSpecular(0.3);//�߹�ϵ��
		actorPoints->GetProperty()->SetSpecularPower(7);//�߹�ǿ�� 
		actorPoints->SetMapper(mapperForView);
		double color[3] = { 0 };

		if (i == 0)
		{
			/*actorPoints->GetProperty()->SetColor(248.0 / 255.0, 248.0 / 255.0, 255.0 / 255.0);*/
		/*	color[0] = 255.0 / 255.0;
			color[1] = 243.0 / 255.0;
			color[2] = 219.0 / 255.0;*/

			color[0] = 211.0 / 255.0;
			color[1] = 211.0 / 255.0;
			color[2] = 211.0 / 255.0;

			actorPoints->GetProperty()->SetColor(color);
			//actorPoints->GetProperty()->SetColor(255.0 / 255.0, 243.0 / 255.0, 219.0 / 255.0);
		}

		//// ��һ��ɫ
		//else
		//{
		//	color[0] = 136.0 / 255.0;
		//	color[1] = 194.0 / 255.0;
		//	color[2] = 225.0 / 255.0;

		//	actorPoints->GetProperty()->SetColor(color);
		//}

		//// ������ɫ
		else if (i == 1)
		{
			// ��ɫ
			//color[0] = 255.0 / 255.0;
			//color[1] = 176.0 / 255.0;
			//color[2] = 176.0 / 255.0;

			color[0] = 200.0 / 255.0;
			color[1] = 36.0 / 255.0;
			color[2] = 35.0 / 255.0;

			actorPoints->GetProperty()->SetColor(color);
		}

		else if (i == 2)
		{
			// ��ɫ
			//color[0] = 136.0 / 255.0;
			//color[1] = 194.0 / 255.0;
			//color[2] = 225.0 / 255.0;

			color[0] = 40.0 / 255.0;
			color[1] = 120.0 / 255.0;
			color[2] = 181.0 / 255.0;
			actorPoints->GetProperty()->SetColor(color);
		}
		else if (i == 3)
		{
			//��ɫ
			//color[0] = 111.0 / 255.0;
			//color[1] = 182.0 / 255.0;
			//color[2] = 166.0 / 255.0;

			color[0] = 50.0 / 255.0;
			color[1] = 184.0 / 255.0;
			color[2] = 151.0 / 255.0;

			actorPoints->GetProperty()->SetColor(color);
		}

		else if (i == 4)
		{
			// ��ɫ
			//color[0] = 250.0 / 255.0;
			//color[1] = 127.0 / 255.0;
			//color[2] = 111.0 / 255.0;

			color[0] = 248.0 / 255.0;
			color[1] = 172.0 / 255.0;
			color[2] = 140.0 / 255.0;


			actorPoints->GetProperty()->SetColor(color);
		}

		else if (i == 5)
		{
			// ��ɫ
			color[0] = 190.0 / 255.0;
			color[1] = 184.0 / 255.0;
			color[2] = 220.0 / 255.0;


			actorPoints->GetProperty()->SetColor(color);
		}

		else if (i > 5)
		{
			// ����ɫ
			color[0] = 84.0 / 255.0;
			color[1] = 179.0 / 255.0;
			color[2] = 69.0 / 255.0;


			actorPoints->GetProperty()->SetColor(color);
		}
		
		renderer->AddActor(actorPoints);

		// �����Χ��
		if (i != 0 && isBBX == 1)
		{
			////////////////////////////// �����Χ�м�������Ϣ  ///////////////////////////////////
			//vtkSmartPointer<vtkNamedColors> colors = vtkSmartPointer<vtkNamedColors>::New();
			int maxLevel = 10;
			vtkSmartPointer<vtkOBBTree> obbTree = vtkSmartPointer<vtkOBBTree>::New();
			obbTree->SetDataSet(Glyph3D->GetOutput());
			obbTree->SetMaxLevel(maxLevel);
			obbTree->BuildLocator();

			double corner[3] = { 0 };
			double max_obb[3] = { 0 };
			double mid_obb[3] = { 0 };
			double min_obb[3] = { 0 };
			double size[3] = { 0 };

			obbTree->ComputeOBB(Glyph3D->GetOutput(), corner, max_obb, mid_obb, min_obb, size);

			vtkSmartPointer<vtkPolyData> obbPolydata = vtkSmartPointer<vtkPolyData>::New();
			obbTree->GenerateRepresentation(0, obbPolydata);

			vtkSmartPointer<vtkPolyDataMapper> obbTreeMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
			obbTreeMapper->SetInputData(obbPolydata);

			vtkSmartPointer<vtkActor> obbTreeActor = vtkSmartPointer<vtkActor>::New();
			obbTreeActor->SetMapper(obbTreeMapper);
			obbTreeActor->GetProperty()->SetInterpolationToFlat();
			obbTreeActor->GetProperty()->SetOpacity(opacity);
			obbTreeActor->GetProperty()->SetColor(color);
			renderer->AddActor(obbTreeActor);
		}
	}

	////////////////////// ���� ////////////////////////////////
	// ��ȡ�ļ��������е����ļ�
	string path_sacrum = path_root;
	path_sacrum.append("sacrum");
	vector<string> files_sacrum;
	getFiles(path_sacrum, files_sacrum);

	int size_sacrum = files_sacrum.size();

	for (int i = 0; i < size_sacrum; i++)
	{
		vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
		vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New();
		ifstream fs(files_sacrum[i]);
		vtkIdType idtype;
		double x, y, z;
		while (fs >> x >> y >> z)
		{
			idtype = points->InsertNextPoint(x, y, z);
			cells->InsertNextCell(1, &idtype);
		}

		vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
		polydata->SetPoints(points);
		vtkSmartPointer<vtkSphereSource> SphereSource = vtkSmartPointer<vtkSphereSource>::New();
		SphereSource->SetRadius(radius);
		//SphereSource->SetRadius(0);
		//SphereSource->SetRadius(1.0);
		vtkSmartPointer<vtkGlyph3D> Glyph3D = vtkSmartPointer<vtkGlyph3D>::New();
		Glyph3D->SetSourceConnection(SphereSource->GetOutputPort());
		Glyph3D->SetInputData(polydata);
		Glyph3D->Update();
		vtkSmartPointer<vtkPolyDataMapper> mapperForView = vtkSmartPointer<vtkPolyDataMapper>::New();
		mapperForView->SetInputData(Glyph3D->GetOutput());
		vtkSmartPointer<vtkActor> actorPoints = vtkSmartPointer<vtkActor>::New();
		//actorPoints->GetProperty()->SetDiffuse(0.4); //������
		//actorPoints->GetProperty()->SetAmbient(0.4); //������
		actorPoints->GetProperty()->SetSpecular(0.3);//�߹�ϵ��
		actorPoints->GetProperty()->SetSpecularPower(7);//�߹�ǿ�� 
		actorPoints->SetMapper(mapperForView);
		double color[3] = { 0 };
		if (i == 0)
		{
			/*actorPoints->GetProperty()->SetColor(248.0 / 255.0, 248.0 / 255.0, 255.0 / 255.0);*/
			//color[0] = 250.0 / 255.0;
			//color[1] = 245.0 / 255.0;
			//color[2] = 231.0 / 255.0;

			color[0] = 211.0 / 255.0;
			color[1] = 210.0 / 255.0;
			color[2] = 211.0 / 255.0;

		/*	color[0] = 250.0 / 255.0;
			color[1] = 245.0 / 255.0;
			color[2] = 231.0 / 255.0;*/

			//color[0] = 255.0 / 255.0;
			//color[1] = 243.0 / 255.0;
			//color[2] = 219.0 / 255.0;

			//actorPoints->GetProperty()->SetDiffuse(0.8); //������
			actorPoints->GetProperty()->SetColor(color);
			//actorPoints->GetProperty()->SetColor(255.0 / 255.0, 243.0 / 255.0, 219.0 / 255.0);
		}

		//else
		//{
			//color[0] = 111.0 / 255.0;
			//color[1] = 182.0 / 255.0;
			//color[2] = 166.0 / 255.0;
		//	actorPoints->GetProperty()->SetColor(color);
		//}

		renderer->AddActor(actorPoints);

		//// �����Χ��
		//if (i != 0 && isBBX == 1)
		//{
		//	////////////////////////////// �����Χ�м�������Ϣ  ///////////////////////////////////
		//	//vtkSmartPointer<vtkNamedColors> colors = vtkSmartPointer<vtkNamedColors>::New();
		//	int maxLevel = 10;
		//	vtkSmartPointer<vtkOBBTree> obbTree = vtkSmartPointer<vtkOBBTree>::New();
		//	obbTree->SetDataSet(Glyph3D->GetOutput());
		//	obbTree->SetMaxLevel(maxLevel);
		//	obbTree->BuildLocator();

		//	double corner[3] = { 0 };
		//	double max_obb[3] = { 0 };
		//	double mid_obb[3] = { 0 };
		//	double min_obb[3] = { 0 };
		//	double size[3] = { 0 };

		//	obbTree->ComputeOBB(Glyph3D->GetOutput(), corner, max_obb, mid_obb, min_obb, size);

		//	vtkSmartPointer<vtkPolyData> obbPolydata = vtkSmartPointer<vtkPolyData>::New();
		//	obbTree->GenerateRepresentation(0, obbPolydata);

		//	vtkSmartPointer<vtkPolyDataMapper> obbTreeMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
		//	obbTreeMapper->SetInputData(obbPolydata);

		//	vtkSmartPointer<vtkActor> obbTreeActor = vtkSmartPointer<vtkActor>::New();
		//	obbTreeActor->SetMapper(obbTreeMapper);
		//	obbTreeActor->GetProperty()->SetInterpolationToFlat();
		//	obbTreeActor->GetProperty()->SetOpacity(opacity);
		//	obbTreeActor->GetProperty()->SetColor(color);
		//	renderer->AddActor(obbTreeActor);
		//}
	}

	//renderer->SetBackground(220.0 / 255.0, 220.0 / 255.0, 220.0 / 255.0); //���ñ�����ɫ
	renderer->SetBackground(255.0 / 255.0, 255.0 / 255.0, 255.0 / 255.0); //���ñ�����ɫ

	auto RenderWindow = vtkSmartPointer <vtkRenderWindow>::New();
	RenderWindow->AddRenderer(renderer);
	RenderWindow->Render();

	vtkSmartPointer<vtkRenderWindowInteractor>iren = vtkSmartPointer<vtkRenderWindowInteractor>::New();
	iren->SetRenderWindow(RenderWindow);
	vtkSmartPointer<vtkInteractorStyleTrackballCamera>style = vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
	iren->SetInteractorStyle(style);
	iren->Initialize();
	iren->Start();

	return 0;
}