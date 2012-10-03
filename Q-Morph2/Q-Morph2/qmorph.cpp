#include "Front_Traits.h"
#include "FrontsManager.h"
#include <iostream>
#include <set>

int main(int argc, char* argv[]){

	PolyFront_Mesh * _poly_Front_Mesh = new PolyFront_Mesh;
	//Load the mesh files.
	std::cout<<"Start loading mesh files...\n";
	if(!OpenMesh::IO::read_mesh(* _poly_Front_Mesh, argv[1])){
		std::cout<<"Creating Polygon mesh ERROR\n";
		return -1;
	}

	//--------------------------TESTING CODE-------------------------------------------------
// 	PolyFront_Mesh::FaceIter face_iter = _poly_Front_Mesh->faces_begin();
// 	_poly_Front_Mesh->data(face_iter.handle()).setQuad(true);
// 	int _id = face_iter.handle().idx();
// 	std::cout<<face_iter.handle()<<"\n";
// 	FaceHandle face_h = _poly_Front_Mesh->faces_begin().handle();
// 	bool isq = _poly_Front_Mesh->data(face_h).isQuadFace();
// 
// 	//Set works for handle !!!
// 	std::set<FaceHandle> set_test;
// 	set_test.insert(face_h);
// 	set_test.insert(face_h);
// 	set_test.insert(face_h);
// 
// 	for (; face_iter != _poly_Front_Mesh->faces_end(); ++face_iter)
// 	{
// 		//FaceHandle face_h = _poly_Front_Mesh->faces_begin().handle();
// 		bool isq = _poly_Front_Mesh->data(face_iter.handle()).isQuadFace();
// 
// 		std::cout<<"face id" << face_iter.handle()<<"   is quad: "<<isq<<"\n";
// 	}
	//--------------------------TESTING CODE-------------------------------------------------

	//the data can not be updated to the global mesh?
	//Yes, it does :-)

	FrontsManager mesh_Front_Manager(_poly_Front_Mesh);
	mesh_Front_Manager.UpdateState();
	mesh_Front_Manager.PrintStateInformation();

	//Process the front state by state.
	while(1){
		if (mesh_Front_Manager.getFrontNumber(3)>0)
		{
			mesh_Front_Manager.ProcessFront_InState(3);
			continue;
		}
		else if (mesh_Front_Manager.getFrontNumber(2)>0)
		{
			mesh_Front_Manager.ProcessFront_InState(2);
			continue;
		}
		else if(mesh_Front_Manager.getFrontNumber(1)>0)
		{
			mesh_Front_Manager.ProcessFront_InState(1);
			continue;
		}
		else if (mesh_Front_Manager.getFrontNumber(0)>0)
		{
			mesh_Front_Manager.ProcessFront_InState(0);
			continue;
		}
		else{
			std::cout<<"All fronts are processed!!!!";
			break;
		}
	}

	system("pause");
	return -1;
}