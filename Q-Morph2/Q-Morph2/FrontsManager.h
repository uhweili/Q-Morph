#pragma once
#include <list>
#include "Front_Traits.h"
using namespace OpenMesh;
class FrontsManager
{
public:
	FrontsManager(PolyFront_Mesh * _pf_mesh):m_mesh(_pf_mesh){};
	~FrontsManager(void){};

	PolyFront_Mesh * getMesh(){return m_mesh;}
	void UpdateState();
	int getFrontNumber(int _state){
		assert(_state < 4);
		return m_front_state[_state].size();
	};

	void PrintStateInformation(){
		for (int i=0; i<4; ++i)
		{
			std::cout<<"State "<<i<<":  "<<m_front_state[i].size()<<"\n";
		}
	}

	bool ProcessFront_InState(int _state); 

private:
	PolyFront_Mesh * m_mesh;
	std::list<HalfedgeHandle> m_front_state[4];

	HalfedgeHandle SelectFrontInState(int& _state);





	//bool TopEdgeRecover(VertexHandle vh_from, VertexHandle vh_to, HalfedgeHandle& _top_edge);

	bool TopEdgeRecover(VertexHandle vh_from, VertexHandle vh_to, HalfedgeHandle& _top_edge, HalfedgeHandle test_edge_info);
	bool SideEdgeRecovery(HalfedgeHandle _front, int reconsturct_type, HalfedgeHandle & _side_edge);
	
	
	//bool getInterationEdge(VertexHandle vh_from, VertexHandle vh_to, std::vector<EdgeHandle>& _interation_edgehandle);
	bool getInterationEdge(VertexHandle vh_from, VertexHandle vh_to, std::list<EdgeHandle>& _interation_edgehandle);


	FaceHandle QuadFormation(HalfedgeHandle _front, HalfedgeHandle _side_edge_to, HalfedgeHandle _top_edge, HalfedgeHandle _side_edge_from);
	
	//smoothing mesh part
	bool QuadSmooth(FaceHandle _quad);
	bool VertexSmooth(VertexHandle _ver);
	bool VertexSmooth_Front(VertexHandle _ver);
	bool VertexSmooth_NonFront(VertexHandle _ver);
};
