#pragma once
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Mesh/Traits.hh>
#include <OpenMesh/Core/Mesh/PolyMeshT.hh>
#include <cassert>

//Customize the Traits

struct Front_Traits: public OpenMesh::DefaultTraits
{
	VertexAttributes(OpenMesh::Attributes::Status);
	FaceAttributes(OpenMesh::Attributes::Status);
	EdgeAttributes(OpenMesh::Attributes::Status);


	VertexTraits
	{
private:
	bool is_OnFront;
	float angle_Between_Fronts;

public:
	VertexT():is_OnFront(false), angle_Between_Fronts(M_PI){};
	const bool& isVertexOnFront() const{ return is_OnFront;}
	void setOnFront(const bool& _of){is_OnFront = _of;}

	const float& angleBetweenFronts() const{
		assert(is_OnFront);
		return angle_Between_Fronts;
	}
	void setAngleBetweenFronts(const float& _abf){
		assert(is_OnFront);
		angle_Between_Fronts = _abf;
	}
	};

	
	
	FaceTraits{
private:
		bool is_Quad;

		int	_level;
		                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        
public:
	FaceT():is_Quad(false), _level(0){};
	
	const bool& isQuadFace() const{ return is_Quad;}
	void setQuad(const bool& _iq){is_Quad = _iq;}
	
	const int& QuadLevel() const {return _level;}
	void setQuadLevel(const int& _l){_level = _l;}
	
	};

	
	
	
	EdgeTraits{
private:
	bool is_OnFront;
public:
	EdgeT():is_OnFront(false){};
	const bool& isEdgeOnFront() const{ return is_OnFront;}
	void setOnFront(const bool& _of){is_OnFront = _of;}
	};



	HalfedgeTraits{
private:
	bool is_InQuad;								//is belonged to quad polygon
	bool is_OnFront;							//is belonged place on front
	int	_level;										// the level value of front

	OpenMesh::HalfedgeHandle _next_front;		//next boundary halfedge( front halfedge)
	OpenMesh::HalfedgeHandle _prev_front;		//previous boundary halfedge
	
	float _next_length_ratio;				// length(this) / length(_next_front)
	float _prev_length_ratio;				// length(this) / length(_prev_front)


public:
	HalfedgeT():is_InQuad(false), is_OnFront(false), _level(0){}
	const bool& isOnFront() const{return is_OnFront;}
	void setOnFront(const bool& _of){is_OnFront = _of;}

	const int& frontLevel() const{return _level;}
	void setFrontLevel(const int& _l){_level = _l;}

	const OpenMesh::HalfedgeHandle prevFrontHalfEdge(){return _prev_front;}
	void setPrevFrontHalfEdge(const OpenMesh::HalfedgeHandle& _pfhe){_prev_front = _pfhe;}


	const OpenMesh::HalfedgeHandle nextFrontHalfEdge(){return _next_front;}
	void setNextFrontHalfEdge(const OpenMesh::HalfedgeHandle& _nfhe){_next_front = _nfhe;}


	const float toPrevFrontLengthRatio()const { return _prev_length_ratio;}
	void setToPrevFrontLengthRatio(const float& _pflr){_prev_length_ratio = _pflr;}

	const float toNextFrontLengthRatio()const {return _next_length_ratio;}
	void setToNextFrontLengthRatio(const float& _nflr){_next_length_ratio = _nflr;}

	};

};

typedef OpenMesh::PolyMesh_ArrayKernelT<Front_Traits>  PolyFront_Mesh;