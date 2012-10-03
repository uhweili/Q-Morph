#pragma once
#include "Front_Traits.h"
#include <set>
#define DRIFT_VALUE 0.01

HalfedgeHandle getOppsiteHalfedgeInFace(PolyFront_Mesh * m, FaceHandle f, VertexHandle v){
	
	HalfedgeHandle he = m->halfedge_handle(f);
	
	VertexHandle v_from = m->from_vertex_handle(he);
	VertexHandle v_to = m->to_vertex_handle(he);

	if (v_from == v && v_to != v)
		return m->next_halfedge_handle(he);
	else if(v_from != v && v_to == v)
		return m->prev_halfedge_handle(he);
	else if(v_from != v && v_to != v){
		HalfedgeHandle he2 = m->next_halfedge_handle(he);
		VertexHandle v_test = m->to_vertex_handle(he2);
		if(v_test == v)
			return he;
		else{
			//v did not contains in f
			std::cout<<"Vertex "<<v <<" dose not in Face "<<f<<"\n";
			assert(0);
		}
	}
}


VertexHandle getOppsiteVertexInFace(PolyFront_Mesh*m, FaceHandle f, HalfedgeHandle he){
	FaceHandle f_he = m->face_handle(he);
	assert(f_he == f);
	HalfedgeHandle he_next = m->next_halfedge_handle(he);
	VertexHandle v  = m->to_vertex_handle(he_next);
	return v;
}



bool isVertexOnTriangle(PolyFront_Mesh* m, FaceHandle f, VertexHandle v){

	HalfedgeHandle he = m->halfedge_handle(f);

	VertexHandle v_1 = m->from_vertex_handle(he);
	VertexHandle v_2 = m->to_vertex_handle(he);
	VertexHandle v_3 = m->to_vertex_handle(m->next_halfedge_handle(he));

	if(v_1 == v || v_2 == v || v_3 == v)
		return true;
	else
		return false;
}


bool  getCommonEdge(PolyFront_Mesh *m, FaceHandle f1, FaceHandle f2, EdgeHandle & common_edge){

	PolyFront_Mesh::FaceHalfedgeIter fhe_it  = m->fh_iter(f1);
	for (; fhe_it; ++fhe_it)
	{
		if (m->opposite_face_handle(fhe_it.handle()) == f2)
		{
			common_edge =  m->edge_handle(fhe_it.handle());
			return true;
		}
	}
	std::cout<<"ERROR: Two face not share the same edge!!!\n ";
	return false;
}

void DeleteFacesUsingBoundary(PolyFront_Mesh *m, HalfedgeHandle start, HalfedgeHandle b1, HalfedgeHandle b2, HalfedgeHandle b3, HalfedgeHandle b4){
	//HalfedgeHandle h1 = m->opposite_halfedge_handle(b1);
	HalfedgeHandle h1 = start;
	HalfedgeHandle h2 = m->next_halfedge_handle(h1);
	HalfedgeHandle h3 = m->prev_halfedge_handle(h1);

	HalfedgeHandle h2_opp = m->opposite_halfedge_handle(h2);
	HalfedgeHandle h3_opp = m->opposite_halfedge_handle(h3);

// 	HalfedgeHandle h2_opp = h2;
// 	HalfedgeHandle h3_opp = h3;


	bool is_h2_hit_boundary;
	//is_h2_hit_boundary = (h2_opp == b2 || h2_opp == b3 || h2_opp == b4);
	is_h2_hit_boundary = (h2 == b1 || h2 == b2 || h2 == b3 || h2 == b4);
// 		std::cout<<"Hit boundary..\n";
// 	else
// 		deleteFaceUsingBoundary(m, h2_opp, b2, b3, b4);
	
	bool is_h3_hit_boundary; 
//	is_h3_hit_boundary = (h3_opp == b2 || h3_opp == b3 || h3_opp == b4);
	is_h3_hit_boundary = (h3 == b1 || h3 == b2 || h3 == b3 || h3 == b4);

	// 		std::cout<<"Hit boundary..\n";
// 	else
// 		deleteFaceUsingBoundary(m, h3_opp, b2, b3, b4);
	//if(!is_h3_hit_boundary || !is_h2_hit_boundary)
	m->delete_face(m->face_handle(h1), false);

	if(m->face_handle(h2_opp).is_valid())
		if (!is_h2_hit_boundary && !m->status(m->face_handle(h2_opp)).deleted() )
			DeleteFacesUsingBoundary(m,h2_opp, b1, b2, b3, b4 );

	if(m->face_handle(h3_opp).is_valid())
		if (!is_h3_hit_boundary && !m->status(m->face_handle(h3_opp)).deleted())
			DeleteFacesUsingBoundary(m,h3_opp, b1, b2, b3, b4 );
}


FaceHandle getFaceContainVector(PolyFront_Mesh * m, VertexHandle start_node, Vector3D _vec){

	//some plug-in work
	PolyFront_Mesh * m_mesh = m;
	VertexHandle vh_from = start_node;
	Vector3D _Vs = _vec;

	PolyFront_Mesh::VertexIHalfedgeIter v_in_he_iter = m_mesh->vih_iter(vh_from);
	FaceHandle _face_contain_vs;

	PolyFront_Mesh::Normal face_normal(0,0,1);
	//Step 1: find the triangle contains vector _Vs;
	while(true){
		HalfedgeHandle he_1 = v_in_he_iter.handle();
		FaceHandle fh = m_mesh->face_handle(he_1);
		if(!fh.is_valid()){
			--v_in_he_iter;
			continue;
		}
		//PolyFront_Mesh::Normal face_normal = m_mesh->calc_face_normal(fh);
		Vector3D _Vk_1= cross((Vec3f)face_normal, (Vec3f)HalfEdge_to_Vector(m_mesh, he_1));
		float _dot_k_1 = dot(_Vs/_Vs.norm(), _Vk_1/_Vk_1.norm());
		if (fabs(_dot_k_1)<1e-4)
		{
			//The node places on the vec _Vs
			VertexHandle _from_vertex = m_mesh->from_vertex_handle(he_1);
			PolyFront_Mesh::Point _p_from = m_mesh->point(_from_vertex);
			//+++++++++++++NOTE:
			//Add or Subtract??? Use Add now. 
			_p_from += _p_from + _Vk_1/_Vk_1.norm()*DRIFT_VALUE;
			m_mesh->set_point(_from_vertex, _p_from);
			continue;
		}
		else if(_dot_k_1 < 0)
		{
			--v_in_he_iter;
			continue;
		}

		HalfedgeHandle he_2 = m_mesh->next_halfedge_handle(he_1);
		Vector3D _Vk_2= cross((Vec3f)face_normal, (Vec3f)HalfEdge_to_Vector(m_mesh, he_2));
		float _dot_k_2 = dot(_Vs/_Vs.norm(), _Vk_2/_Vk_2.norm());
		if(fabs(_dot_k_2) < 1e-3){
			VertexHandle _from_vertex = m_mesh->from_vertex_handle(he_2);
			PolyFront_Mesh::Point _p_from = m_mesh->point(_from_vertex);
			//+++++++++++++NOTE:
			//Add or Subtract??? Use Add now. 
			_p_from += _p_from + _Vk_2/ _Vk_2.norm()* DRIFT_VALUE ;
			m_mesh->set_point(_from_vertex, _p_from);
			continue;
		}
		else if(_dot_k_2 < 0){
			--v_in_he_iter;
			continue;
		}
		else{
			_face_contain_vs = m_mesh->face_handle(he_2);

			//-------------------------------------------------------
			//Test Code:  Test code is not CORRECT HERE!!!!
// 			Vector3D test_1 = cross((Vec3f) _Vk_1, (Vec3f)_Vs);
// 			Vector3D test_2 = cross((Vec3f) _Vs, (Vec3f)_Vk_2);
// 			float test_dot = dot((Vec3f)test_1/test_1.norm(), (Vec3f)test_2/test_2.norm());
// 			if(test_dot >0)
// 				std::cout<<"Get Triangle contains Vs Pass the test\n";
// 			else
// 			{
// 				std::cout<<"Get Triangle contains Vs CAN NOT Pass the test\n";
// 				assert(0);
// 				system("pause");
// 				exit(0);
// 			}
			//-------------------------------------------------------
			break;
		}
	}

	return _face_contain_vs;
}