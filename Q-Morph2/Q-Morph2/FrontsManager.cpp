#include "FrontsManager.h"
#include "point/Point.h"
#include "UserDefineOpertion.h"


#define RC_TO_SE 1
#define RC_FROM_SE 0


void FrontsManager::UpdateState()
{
	for (int _state =0; _state < 4; ++_state)
	{
		m_front_state[_state].clear();
	}

	PolyFront_Mesh::HalfedgeIter half_edge_iter, half_edge_iter_end(m_mesh->halfedges_end());
	for (half_edge_iter = m_mesh->halfedges_begin(); half_edge_iter != half_edge_iter_end; ++half_edge_iter)
	{
		/*The halfedge is classed as front in case:
		1. the halfedge in the boundary and it's opposite halfedge is in triangle 
		2. the halfedge in the quad polygon and it's opposite halfedge is in triangle
		*/

		HalfedgeHandle _halfedge_h = half_edge_iter.handle();
		HalfedgeHandle _halfedge_oppsite_h = m_mesh->opposite_halfedge_handle(_halfedge_h);

// 		HalfedgeHandle _next_front_halfedge_h_test;
// 		_next_front_halfedge_h_test = m_mesh->next_halfedge_handle(_halfedge_h);
// 
// 		HalfedgeHandle _prev_front_halfedge_h_test;
// 		_prev_front_halfedge_h_test = m_mesh->prev_halfedge_handle(_halfedge_h);
// 
// 		FaceHandle _next_front_face_h_test


		//Case 1 : the front located on the boundary of non-quad face
		if(m_mesh->is_boundary(m_mesh->edge_handle(_halfedge_h))  )
		{
			if(!m_mesh->is_boundary(_halfedge_h)) //if current halfedge is not on boundary, skip it
				continue;
			//if hits a quad's boundary, skip it. 
			else if(m_mesh->data(m_mesh->opposite_face_handle(_halfedge_h)).isQuadFace())
				continue;
			else{
				//current halfedge located on boundary
				FaceHandle _face_oppsite_h = m_mesh->face_handle(_halfedge_oppsite_h);
				//if(_face_oppsite_h.idx() ==0)
			//	std::cout<<"Face id: "<<_face_oppsite_h<<"  Is Quad: "<<m_mesh->data(_face_oppsite_h).isQuadFace()<<"\n";
				//if the edge do not located on quad face (means it located on triangle face), save it as front
				if(!m_mesh->data(_face_oppsite_h).isQuadFace()){
					
					//Set halfedge , edge, vertex, front frag.
					m_mesh->data(_halfedge_h).setOnFront(true);
					m_mesh->data(m_mesh->edge_handle(_halfedge_h)).setOnFront(true);
					m_mesh->data(m_mesh->from_vertex_handle(_halfedge_h)).setOnFront(true);
					m_mesh->data(m_mesh->to_vertex_handle(_halfedge_h)).setOnFront(true);

					//Update the level Information.
					m_mesh->data(_halfedge_h).setFrontLevel(0);


					//If the halfedge located on boundary, the next/prev front is its next() /prev()
					//NOTE: the boundary-front and non-boundary neighbors case is not considered. 
					//BUG HERE!!!!

					HalfedgeHandle _next_front_halfedge_h;
					_next_front_halfedge_h = m_mesh->next_halfedge_handle(_halfedge_h);
					//----------Bug fixing here-------
					while(m_mesh->data(m_mesh->opposite_face_handle(_next_front_halfedge_h)).isQuadFace()){
						_next_front_halfedge_h = m_mesh->opposite_halfedge_handle(_next_front_halfedge_h);
						_next_front_halfedge_h = m_mesh->next_halfedge_handle(_next_front_halfedge_h);
					}

					m_mesh->data(_halfedge_h).setNextFrontHalfEdge(_next_front_halfedge_h);
		

					HalfedgeHandle _prev_front_halfedge_h;
					_prev_front_halfedge_h = m_mesh->prev_halfedge_handle(_halfedge_h);
					//----------Bug fixing here-------
					while(m_mesh->data(m_mesh->opposite_face_handle(_prev_front_halfedge_h)).isQuadFace()){
						_prev_front_halfedge_h = m_mesh->opposite_halfedge_handle(_prev_front_halfedge_h);
						_prev_front_halfedge_h = m_mesh->prev_halfedge_handle(_prev_front_halfedge_h);
					}

					m_mesh->data(_halfedge_h).setPrevFrontHalfEdge(_prev_front_halfedge_h);

					//compute the length ratio
					//-----1. get vector of each halfedge
					//Vector3D _vec_current_front, _vec_next_front, _vec_prev_front;
					//m_mesh->calc_edge_vector(_halfedge_h, _vec_current_front);
					//m_mesh->calc_edge_vector(_prev_front_halfedge_h, _vec_prev_front);
					//m_mesh->calc_edge_vector(_next_front_halfedge_h, _vec_next_front);

					//####NOTE: directly computing using vector is not correct. 
					//when the angle is bigger than 180, th result is not good. 
					
					
					//_vec_current_front = HalfEdge_to_Vector(m_mesh, _halfedge_h);
					//_vec_next_front = HalfEdge_to_Vector(m_mesh, _next_front_halfedge_h);
					//_vec_prev_front = HalfEdge_to_Vector(m_mesh, _prev_front_halfedge_h);

					float ratio_current_to_next = m_mesh->calc_edge_length(_halfedge_h) / m_mesh->calc_edge_length(_next_front_halfedge_h);
					float ratio_current_to_prev = m_mesh->calc_edge_length(_halfedge_h) / m_mesh->calc_edge_length(_prev_front_halfedge_h);

					m_mesh->data(_halfedge_h).setToNextFrontLengthRatio(ratio_current_to_next);
					m_mesh->data(_halfedge_h).setToPrevFrontLengthRatio(ratio_current_to_prev);
					
					//OpenMesh::VectorT<PolyFront_Mesh::Scalar, 3> j(1,1,1);
					//j= _vec_next_front * _vec_current_front;



					//////////////////////////////////////////////////////////////////////////
					//Compute the state of current front, 
					//Loop the edge to get the angle between 2 front.
					HalfedgeHandle looper_he_to = _halfedge_h;
					FaceHandle looper_face_to;
					float angle_front_to = 0;
					while(true) //loop until looper_from hit the boundary
					{
						looper_he_to = m_mesh->opposite_halfedge_handle(looper_he_to);
						looper_he_to = m_mesh->prev_halfedge_handle(looper_he_to);
						looper_face_to = m_mesh->opposite_face_handle(looper_he_to);
						angle_front_to += m_mesh->calc_sector_angle(looper_he_to);

						if(!looper_face_to.is_valid())
							break;
						else if(m_mesh->data(looper_face_to).isQuadFace())
							break;
					//	else
					}

					HalfedgeHandle looper_he_from = _halfedge_h;
					FaceHandle looper_face_from;
					float angle_front_from = 0;
					while(true){
						looper_he_from = m_mesh->opposite_halfedge_handle(looper_he_from);
						looper_he_from = m_mesh->next_halfedge_handle(looper_he_from);
						looper_face_from = m_mesh->opposite_face_handle(looper_he_from);
						angle_front_from += m_mesh->calc_sector_angle(m_mesh->prev_halfedge_handle(looper_he_from));

						if(!looper_face_from.is_valid())
							break;
						else if(m_mesh->data(looper_face_from).isQuadFace())
							break;			
					}

					bool have_from_side_edge = false;
					bool have_to_side_edge = false;
					if(angle_front_from < 3*M_PI/4)
						have_from_side_edge = true;
					if(angle_front_to < 3*M_PI/4)
						have_to_side_edge = true;


					int _state =have_from_side_edge + 2*have_to_side_edge; 
					//non---->non: 0
					//edge--->non: 1
					//edge---->edge: 3
					//non------>edge: 2

					//std::cout<<"Real Boundary Front id:"<<_halfedge_h <<"Level Value: "<<0<<" State Value: "<<_state<<"\n";
					m_front_state[_state].push_back(_halfedge_h);
					continue;
				}
				// 			else{
				// 				m_mesh->data(_halfedge_h).setOnFront(false);
				// 				m_mesh->data(m_mesh->edge_handle(_halfedge_h)).setOnFront(false);
				// 				m_mesh->data(m_mesh->from_vertex_handle(_halfedge_h)).setOnFront(false);
				// 				m_mesh->data(m_mesh->to_vertex_handle(_halfedge_h)).setOnFront(false);
				// 			}
			}
		}
		//Case 2:
		else{
			bool _is_half_edge_in_quad = m_mesh->data(m_mesh->face_handle(_halfedge_h)).isQuadFace();
			bool _is_oppsite_half_edge_in_quad = m_mesh->data(m_mesh->opposite_face_handle(_halfedge_h)).isQuadFace();

			HalfedgeHandle _half_edge_front;

			if (_is_half_edge_in_quad && !_is_oppsite_half_edge_in_quad)
				_half_edge_front = _halfedge_h;
			
			//just keep the outside(means not in triangle side) halfedge as front
			//This bug cost me 1 HOUR!!
//			else if(!_is_half_edge_in_quad && _is_oppsite_half_edge_in_quad)
///				_half_edge_front = _halfedge_h;
			else
				continue;

			m_mesh->data(_halfedge_h).setOnFront(true);
			m_mesh->data(m_mesh->edge_handle(_halfedge_h)).setOnFront(true);
			m_mesh->data(m_mesh->from_vertex_handle(_halfedge_h)).setOnFront(true);
			m_mesh->data(m_mesh->to_vertex_handle(_halfedge_h)).setOnFront(true);


			//Add the Level value onto halfedge
			int opp_quad_level =  m_mesh->data(m_mesh->opposite_face_handle(_halfedge_h)).QuadLevel();
			int current_front_level = opp_quad_level+1;
			m_mesh->data(_halfedge_h).setFrontLevel(current_front_level);

			//compute state
		//	int _state =0;


			//compute next /prev halfedge ..... and solve the side front angle at the same time
			//need to loop around the from/to node until looper_from hit the triangle boundary
			HalfedgeHandle _next_front_halfedge_h = _halfedge_h;
			//HalfedgeHandle _next_front_looper;
			FaceHandle _looper_face_to;
			float angle_front_to = 0;
			while(true){
				_next_front_halfedge_h = m_mesh->opposite_halfedge_handle(_next_front_halfedge_h);
				_next_front_halfedge_h = m_mesh->prev_halfedge_handle(_next_front_halfedge_h);
				angle_front_to += m_mesh->calc_sector_angle(_next_front_halfedge_h);
				_looper_face_to = m_mesh->opposite_face_handle(_next_front_halfedge_h);
				if(!_looper_face_to.is_valid()){
					_next_front_halfedge_h = m_mesh->opposite_halfedge_handle(_next_front_halfedge_h);
					break;
				}
				else if(m_mesh->data(_looper_face_to).isQuadFace())
				{
					_next_front_halfedge_h = m_mesh->opposite_halfedge_handle(_next_front_halfedge_h);
					break;
				}
			}
			m_mesh->data(_halfedge_h).setNextFrontHalfEdge(_next_front_halfedge_h);

			HalfedgeHandle _prev_front_halfedge_h = _halfedge_h;
			FaceHandle _looper_face_from;
			float angle_front_from = 0; 
			while (true)
			{
				_prev_front_halfedge_h = m_mesh->opposite_halfedge_handle(_prev_front_halfedge_h);
				angle_front_from += m_mesh->calc_sector_angle(_prev_front_halfedge_h);
				_prev_front_halfedge_h = m_mesh->next_halfedge_handle(_prev_front_halfedge_h);
				_looper_face_from = m_mesh->opposite_face_handle(_prev_front_halfedge_h);
				if (!_looper_face_from.is_valid())
				{
					_prev_front_halfedge_h = m_mesh->opposite_halfedge_handle(_prev_front_halfedge_h);
					break;
				}
				else if(m_mesh->data(_looper_face_from).isQuadFace()){
					_prev_front_halfedge_h = m_mesh->opposite_halfedge_handle(_prev_front_halfedge_h);
					break;
				}
			}
			//_prev_front_halfedge_h = m_mesh->prev_halfedge_handle(_halfedge_h);
			m_mesh->data(_halfedge_h).setPrevFrontHalfEdge(_prev_front_halfedge_h);
			//HalfedgeHandle  

			//compute the length ratio
			float ratio_current_to_next = m_mesh->calc_edge_length(_halfedge_h) / m_mesh->calc_edge_length(_next_front_halfedge_h);
			float ratio_current_to_prev = m_mesh->calc_edge_length(_halfedge_h) / m_mesh->calc_edge_length(_prev_front_halfedge_h);
			m_mesh->data(_halfedge_h).setToNextFrontLengthRatio(ratio_current_to_next);
			m_mesh->data(_halfedge_h).setToPrevFrontLengthRatio(ratio_current_to_prev);


			bool have_from_side_edge = false;
			bool have_to_side_edge = false;
			if(angle_front_from < 3*M_PI/4)
				have_from_side_edge = true;
			if(angle_front_to < 3*M_PI/4)
				have_to_side_edge = true;

			//compute state
			int _state =have_from_side_edge + 2*have_to_side_edge; 

			//std::cout<<"Real Boundary Front id:"<<_halfedge_h <<"Level Value: "<<current_front_level<<" State Value: "<<_state<<"\n";
			m_front_state[_state].push_back(_halfedge_h);
			continue;
		}
	}
}

//main algorithm to process the fronts
bool FrontsManager::ProcessFront_InState( int _state )
{
	assert(_state < 4);

	HalfedgeHandle _front_to_process = SelectFrontInState(_state);
	HalfedgeHandle _front_side_edge_from;
	HalfedgeHandle _front_side_edge_to;
	HalfedgeHandle _front_top_edge;
	bool is_process_success = false;

	VertexHandle v1 = m_mesh->to_vertex_handle(_front_to_process);
	VertexHandle v2 = m_mesh->from_vertex_handle(_front_to_process);

	VertexHandle p_v_from = m_mesh->from_vertex_handle(_front_to_process);
	VertexHandle p_v_to = m_mesh->to_vertex_handle(_front_to_process);
	VertexHandle p_v_to_up, p_v_from_up;

	// 		PolyFront_Mesh::Point z_adder(0,0,0.2);
	PolyFront_Mesh::Point v1_z_adder = m_mesh->point(v1) ;
	PolyFront_Mesh::Point v2_z_adder = m_mesh->point(v2) ;

	PolyFront_Mesh::Point v3_to_z, v4_from_z;

	PrintStateInformation();
	std::cout<<"Processing front at state"<<_state<<"\n";

	switch (_state)
	{
	case 3:
		//Process the front in state-3

// 
// 		m_mesh->set_point(v1, v1_z_adder);
// 		m_mesh->set_point(v2, v2_z_adder);

		//PolyFront_Mesh::Point p_new = m_mesh->point(m_mesh->to_vertex_handle(_front_to_process));

		_front_side_edge_from = m_mesh->data(_front_to_process).prevFrontHalfEdge();

		p_v_from_up = m_mesh->from_vertex_handle(_front_side_edge_from);

		_front_side_edge_to = m_mesh->data(_front_to_process).nextFrontHalfEdge();

		p_v_to_up = m_mesh->to_vertex_handle(_front_side_edge_to);


		v3_to_z = m_mesh->point(m_mesh->to_vertex_handle(_front_side_edge_to));
		v4_from_z = m_mesh->point(m_mesh->from_vertex_handle(_front_side_edge_from));


		//NOTE: +++++++++++++++++++++++++++
		//should have some logical for fail cases checking
		//is_process_success = TopEdgeRecover( m_mesh->from_vertex_handle(_front_side_edge_from), 	m_mesh->to_vertex_handle(_front_side_edge_to), _front_top_edge);
		is_process_success = TopEdgeRecover( m_mesh->from_vertex_handle(_front_side_edge_from), 	m_mesh->to_vertex_handle(_front_side_edge_to), _front_top_edge, _front_side_edge_from);

		// 		if(m_mesh->from_vertex_handle(_front_top_edge) == m_mesh->from_vertex_handle(_front_side_edge_from))
// 			_front_top_edge = m_mesh->opposite_halfedge_handle(_front_top_edge);
		

		//update the state list, wil
// 		if(is_process_success){
// 			m_front_state[3].pop_front();
// 		}
// 		else{
// 			m_front_state[3].push_back(m_front_state[3].front());
// 			m_front_state[3].pop_front();
// 		}
		break;
	case 2:
		//Process the front in state-2 (to side edge on "to" node), 
		//Therefore, we need to reconstruct the side edge on "from" node
		_front_side_edge_to = m_mesh->data(_front_to_process).nextFrontHalfEdge();
		p_v_to_up = m_mesh->to_vertex_handle(_front_side_edge_to);
		is_process_success = SideEdgeRecovery(_front_to_process, RC_FROM_SE, _front_side_edge_from);
		p_v_from_up = m_mesh->from_vertex_handle(_front_side_edge_from);

		v1 = m_mesh->from_vertex_handle(_front_side_edge_from);
		v2 = m_mesh->to_vertex_handle(_front_side_edge_from);
		//is_process_success = TopEdgeRecover( m_mesh->from_vertex_handle(_front_side_edge_from), 	m_mesh->to_vertex_handle(_front_side_edge_to), _front_top_edge);
		is_process_success = TopEdgeRecover( m_mesh->from_vertex_handle(_front_side_edge_from), 	m_mesh->to_vertex_handle(_front_side_edge_to), _front_top_edge,		_front_side_edge_from);

		v1 = m_mesh->from_vertex_handle(_front_side_edge_from);
		v2 = m_mesh->to_vertex_handle(_front_side_edge_from);
		break;
	case 1:
		//Process the front in state-1
		_front_side_edge_from = m_mesh->data(_front_to_process).prevFrontHalfEdge();
		p_v_from_up = m_mesh->from_vertex_handle(_front_side_edge_from);
		is_process_success = SideEdgeRecovery(_front_to_process, RC_TO_SE, _front_side_edge_to);
		p_v_to_up = m_mesh->to_vertex_handle(_front_side_edge_to);
		//is_process_success = TopEdgeRecover( m_mesh->from_vertex_handle(_front_side_edge_from), 	m_mesh->to_vertex_handle(_front_side_edge_to), _front_top_edge);
		is_process_success = TopEdgeRecover( m_mesh->from_vertex_handle(_front_side_edge_from), 	m_mesh->to_vertex_handle(_front_side_edge_to), _front_top_edge,		_front_side_edge_from);
		break;
	case 0:
		//Process the front in state-0
		is_process_success = SideEdgeRecovery(_front_to_process, RC_FROM_SE, _front_side_edge_from);
		p_v_from_up = m_mesh->from_vertex_handle(_front_side_edge_from);
		is_process_success = SideEdgeRecovery(_front_to_process, RC_TO_SE, _front_side_edge_to);
		p_v_to_up = m_mesh->to_vertex_handle(_front_side_edge_to);
		//is_process_success = TopEdgeRecover( m_mesh->from_vertex_handle(_front_side_edge_from), 	m_mesh->to_vertex_handle(_front_side_edge_to), _front_top_edge);
		is_process_success = TopEdgeRecover( m_mesh->from_vertex_handle(_front_side_edge_from), 	m_mesh->to_vertex_handle(_front_side_edge_to), _front_top_edge,		_front_side_edge_from);
		break;
	default:
		std::cout<<"No front has this state!!! YOU HAVE BUGS!!!";
		assert(0);
		break;
	}

	_front_to_process = m_mesh->find_halfedge(p_v_from, p_v_to);
	p_v_from = m_mesh->from_vertex_handle(_front_to_process);
	p_v_to = m_mesh->to_vertex_handle(_front_to_process);
	//From a quad face in QuadFormation function
	v1 = m_mesh->from_vertex_handle(_front_side_edge_from);
	v2 = m_mesh->to_vertex_handle(_front_side_edge_from);
	_front_side_edge_from = m_mesh->find_halfedge(p_v_from_up, p_v_from);
	_front_side_edge_to = m_mesh->find_halfedge(p_v_to, p_v_to_up);
	_front_top_edge = m_mesh->find_halfedge(p_v_to_up, p_v_from_up);


	//Make Some Assert to make sure all edge is reconstruct correctly, 
	//If any problems here, all mesh will be delete!!!
	assert(m_mesh->from_vertex_handle(_front_to_process) == m_mesh->to_vertex_handle(_front_side_edge_from));
	assert(m_mesh->to_vertex_handle(_front_to_process) == m_mesh->from_vertex_handle(_front_side_edge_to));
	assert(m_mesh->from_vertex_handle(_front_top_edge) == m_mesh->to_vertex_handle(_front_side_edge_to));
	assert(m_mesh->to_vertex_handle(_front_top_edge) == m_mesh->from_vertex_handle(_front_side_edge_from));


	FaceHandle _quad = QuadFormation(_front_to_process, _front_side_edge_to, _front_top_edge, _front_side_edge_from);
	
	//Put smoothing code here






	m_mesh->data(_quad).setQuad(true);
	PolyFront_Mesh::FaceHalfedgeIter fhe_it = m_mesh->fh_iter(_quad);
	//for (; fhe_it; ++fhe_it)
	//	m_mesh->data(fhe_it.handle())->
	UpdateState();

	return false;
}

OpenMesh::HalfedgeHandle FrontsManager::SelectFrontInState( int& _state )
{
	//NOTE++++++++++++++++++++++++++++++++++++++++
	//right now the algorithm just return the first element in the list, 
	//should improvement further.

	//return m_front_state[_state].front();

	//Return the front with smallest level in given state number.
	HalfedgeHandle select_front;
	int smallest_level = RAND_MAX;   //USE RANDOM MAX
	int select_state = _state;

	for (int st = _state; st>=0; --st)
	{
		std::list<HalfedgeHandle> _current_state_container = m_front_state[st];

		//loop to get the halfedge handle with smallest level.
		for (std::list<HalfedgeHandle>::iterator fit = _current_state_container.begin(); fit != _current_state_container.end(); ++fit)
		{
			HalfedgeHandle he_it = *fit;
			int he_it_level = m_mesh->data(he_it).frontLevel();
			if(smallest_level > he_it_level)
			{
				smallest_level = he_it_level;
				select_front = he_it;
				select_state = st;
			}
		}
	}


	//No matter the sequence to select the front from halfedge container, update function will update all things

	_state = select_state;
	return select_front;
}

//bool FrontsManager::TopEdgeRecover( VertexHandle vh_from, VertexHandle vh_to, HalfedgeHandle& _top_edge)
bool FrontsManager::TopEdgeRecover( VertexHandle vh_from, VertexHandle vh_to, HalfedgeHandle& _top_edge, HalfedgeHandle test_edge_info)
{
	//A BUG INSIDE!!!! 

	
	VertexHandle v1, v2;

	//////////////////////////////////////////////////////////////////////////
	//Test code
	v1 = m_mesh->from_vertex_handle(test_edge_info);
	v2 = m_mesh->to_vertex_handle(test_edge_info);
	//////////////////////////////////////////////////////////////////////////

	std::cout<<"Building a top edge from "<<vh_to<<" to "<<vh_from<<"\n"; 

	bool _success = true;

	//Check is edge (vh_from, vh_to) exsist
	//FIX one bug here:  the op edge should have a direction vh_to -----> vh_from
	//PolyFront_Mesh::VertexOHalfedgeIter v_o_he_it = m_mesh->voh_iter(vh_from);
	PolyFront_Mesh::VertexOHalfedgeIter v_o_he_it = m_mesh->voh_iter(vh_to);
	for (; v_o_he_it; ++v_o_he_it)
	{
		VertexHandle v = m_mesh->to_vertex_handle(v_o_he_it.handle());
	//	if(vh_to == v){
		if(vh_from == v){
			_top_edge = v_o_he_it.handle();
			return true;
		}
	}

	//std::vector<EdgeHandle> intersection_edge;
	std::list<EdgeHandle> intersection_edge;

	_success = getInterationEdge(vh_from, vh_to, intersection_edge);
	if (!_success)
		return false;

	//////////////////////////////////////////////////////////////////////////
	//Test code
	v1 = m_mesh->from_vertex_handle(test_edge_info);
	v2 = m_mesh->to_vertex_handle(test_edge_info);
	//////////////////////////////////////////////////////////////////////////


	//std::vector<EdgeHandle>::iterator intersection_egde_iter;
	while(intersection_edge.size()>0){
		EdgeHandle eh  = intersection_edge.front();
		HalfedgeHandle heh_1 = m_mesh->halfedge_handle(eh, 0);
		HalfedgeHandle heh_2 = m_mesh->halfedge_handle(eh, 1);

		//to check if the flip operation will cause inverse triangle
		bool _is_inverse = false;

		HalfedgeHandle tri_1_he_1 = m_mesh->next_halfedge_handle(heh_1);
		HalfedgeHandle tri_1_he_2 = m_mesh->prev_halfedge_handle(heh_2);

		HalfedgeHandle tri_2_he_1 = m_mesh->next_halfedge_handle(heh_2);
		HalfedgeHandle tri_2_he_2 = m_mesh->prev_halfedge_handle(heh_1);

		Vector3D vec_tri_1_he_1 = HalfEdge_to_Vector(m_mesh, tri_1_he_1);
		Vector3D vec_tri_1_he_2 = HalfEdge_to_Vector(m_mesh, tri_1_he_2);

		Vec3f cross_res = cross(vec_tri_1_he_2/vec_tri_1_he_2.norm(), vec_tri_1_he_1/vec_tri_1_he_1.norm());
		if(cross_res[2] <0)
			_is_inverse = true;

		Vector3D vec_tri_2_he_1 = HalfEdge_to_Vector(m_mesh, tri_2_he_1);
		Vector3D vec_tri_2_he_2 = HalfEdge_to_Vector(m_mesh, tri_2_he_2);
		cross_res = cross(vec_tri_2_he_2/vec_tri_2_he_2.norm(), vec_tri_2_he_1/vec_tri_1_he_1.norm());
		if(cross_res[2]<0)
			_is_inverse = true;

		if(_is_inverse){
			intersection_edge.push_back(eh);
			intersection_edge.pop_front();
			continue;
		}
		else{
			//////////////////////////////////////////////////////////////////////////
			//Test code
			v1 = m_mesh->from_vertex_handle(test_edge_info);
			v2 = m_mesh->to_vertex_handle(test_edge_info);
			//////////////////////////////////////////////////////////////////////////

			//no inverse triangle, let's flip it!!!

			VertexHandle v_flip_tri_1_1 = m_mesh->from_vertex_handle(tri_1_he_1);
			VertexHandle v_flip_tri_1_2 = m_mesh->to_vertex_handle(tri_1_he_1);
			VertexHandle v_flip_tri_1_3 = m_mesh->from_vertex_handle(tri_1_he_2);

			VertexHandle v_flip_tri_2_1 = m_mesh->from_vertex_handle(tri_2_he_1);
			VertexHandle v_flip_tri_2_2 = m_mesh->to_vertex_handle(tri_2_he_1);
			VertexHandle v_flip_tri_2_3 = m_mesh->from_vertex_handle(tri_2_he_2);

			std::vector<PolyFront_Mesh::VertexHandle>  tmp_face_1_vhandles;
			tmp_face_1_vhandles.push_back(v_flip_tri_1_1);
			tmp_face_1_vhandles.push_back(v_flip_tri_1_2);
			tmp_face_1_vhandles.push_back(v_flip_tri_1_3);

			std::vector<PolyFront_Mesh::VertexHandle>  tmp_face_2_vhandles;
			tmp_face_2_vhandles.push_back(v_flip_tri_2_1);
			tmp_face_2_vhandles.push_back(v_flip_tri_2_2);
			tmp_face_2_vhandles.push_back(v_flip_tri_2_3);

			//remove the edge->halfedge->face
			m_mesh->delete_edge(eh, false);
			m_mesh->garbage_collection();

			//////////////////////////////////////////////////////////////////////////
			//Test code
	//		v1 = m_mesh->from_vertex_handle(test_edge_info);
	//		v2 = m_mesh->to_vertex_handle(test_edge_info);
			//////////////////////////////////////////////////////////////////////////


			OpenMesh::IO::write_mesh(*m_mesh, "topedge_before_fill.ply");

			//add the new face, finish the processing of flip
			FaceHandle tri_face_1 = m_mesh->add_face(tmp_face_1_vhandles);
			FaceHandle tri_face_2 = m_mesh->add_face(tmp_face_2_vhandles);

			//-------------------------------Some Testing output code ---------------------
			try {
				if ( !OpenMesh::IO::write_mesh(*m_mesh, "topedge.ply") ) {
					std::cerr << "Cannot write mesh to file 'output.off'" << std::endl;
					return false;
				}
			}
			catch( std::exception& x )
			{
				std::cerr << x.what() << std::endl;
				return false;
			}
			//------------------------------------- ----------------------------------------------------

			EdgeHandle common_edge;
			if(!getCommonEdge(m_mesh, tri_face_1, tri_face_2, common_edge)){
				std::cout<<"Some error in face construction!!!!!";
				assert(0);
				return false;
			}
			//check if common edge intersect with Vs
			//if does, insert common edge again and continue the processing
			//if not, pop-out the current edge
			HalfedgeHandle heh_common = m_mesh->halfedge_handle(common_edge, 1);
			VertexHandle heh_common_v_from = m_mesh->from_vertex_handle(heh_common);
			VertexHandle heh_common_v_to = m_mesh->to_vertex_handle(heh_common);
			if(is_VertexSegment_Intersection(m_mesh, 
				heh_common_v_from, heh_common_v_to, 
				vh_from, vh_to))
			{
				intersection_edge.pop_front();
				intersection_edge.push_back(common_edge);
			}
			else
			{
				//NOTE+++++++++++++++++:
				//IS set here right?? 
				_top_edge = heh_common;
				//well, not problem in current testing result. 

				intersection_edge.pop_front();
			}
		}
	}
	if(m_mesh->from_vertex_handle(_top_edge) != vh_from && m_mesh->to_vertex_handle(_top_edge) != vh_from)
	{
		std::cout<<"The top edge cover FAIL!!!!\n";
		assert(0);
		return false;
	}

	if(m_mesh->from_vertex_handle(_top_edge) != vh_to && m_mesh->to_vertex_handle(_top_edge) != vh_to)
	{
		std::cout<<"The top edge cover FAIL!!!!\n";
		assert(0);
		return false;
	}

	if (m_mesh->from_vertex_handle(_top_edge) == vh_from)
		_top_edge = m_mesh->opposite_halfedge_handle(_top_edge);

	std::cout<<"Top Edge  "<< m_mesh->from_vertex_handle(_top_edge) 
		<<" to "<<m_mesh->to_vertex_handle(_top_edge) 
		<<"Recover finish!!!\n";

	return true;
}

//bool FrontsManager::getInterationEdge( VertexHandle vh_from, VertexHandle vh_to, std::vector<EdgeHandle>& _interation_edgehandle )
bool FrontsManager::getInterationEdge( VertexHandle vh_from, VertexHandle vh_to, std::list<EdgeHandle>& _interation_edgehandle )

{
	Vector3D _Vs = Vertex_to_Vector(m_mesh, vh_from, vh_to);

	//CCW halfedge iterator on the from edge 
	//NOTE:++++++++++++++++++++++++++++++
	// Is this iterator loop infinity?? if not, there will be some problem. 
	//+++++++++++++++++++++++++++++++++++
	PolyFront_Mesh::VertexIHalfedgeIter v_in_he_iter = m_mesh->vih_iter(vh_from);
	FaceHandle _face_contain_vs;
	
	//Step 1: find the triangle contains vector _Vs;
	PolyFront_Mesh::Normal face_normal(0,0,1);

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


	//some test code of function getFaceContainVector
	FaceHandle _test_face = getFaceContainVector(m_mesh, vh_from, _Vs);
	assert(_test_face == _face_contain_vs);




	//Step 2: Loop to get all the edge intersect with the line
	HalfedgeHandle HE_1 = getOppsiteHalfedgeInFace(m_mesh, _face_contain_vs, vh_from);
	EdgeHandle E_1 = m_mesh->edge_handle(HE_1);
	bool is_E_i_front = false;
	if (m_mesh->data(E_1).isEdgeOnFront())
	{
		std::cout<<"Recover Process Hits a Front "<< E_1 <<" \n";
		return false;
	}
	else
		_interation_edgehandle.push_back(E_1);

//	PolyFront_Mesh::Normal face_normal = m_mesh->calc_face_normal();

	bool _process_done = false;
	FaceHandle Ti_1;
	VertexHandle Ni_1;
	HalfedgeHandle HE_2;
	while(!_process_done){
		HE_2 = m_mesh->opposite_halfedge_handle(HE_1);
		Ti_1 = m_mesh->face_handle(HE_2);
		if(isVertexOnTriangle(m_mesh, Ti_1, vh_to)){
			_process_done = true;
			break;
		}

		Ni_1 = getOppsiteVertexInFace(m_mesh, Ti_1, HE_2);
		PolyFront_Mesh::Normal face_normal = m_mesh->calc_face_normal(m_mesh->face_handle(HE_2));
		Vector3D vec_Ni_Nfrom = Vertex_to_Vector(m_mesh, Ni_1, vh_from);
		Vector3D Vi = cross((Vec3f) face_normal, (Vec3f) vec_Ni_Nfrom);
		float dot_vec_Nfrom_Ni_Vi = dot(_Vs/_Vs.norm(), Vi/Vi.norm());
		if(fabs(dot_vec_Nfrom_Ni_Vi) < 1e-3)
		{
			//VertexHandle _from_vertex = m_mesh->from_vertex_handle(he_2);
			PolyFront_Mesh::Point _p_from = m_mesh->point(Ni_1);
			//+++++++++++++NOTE:
			//Add or Subtract??? Use Add now. 
			_p_from += _p_from + Vi/Vi.norm()* DRIFT_VALUE;
			m_mesh->set_point(Ni_1, _p_from);
			continue;
		}
		else if(dot_vec_Nfrom_Ni_Vi <0)
			HE_1 = m_mesh->prev_halfedge_handle(HE_2);
		else if(dot_vec_Nfrom_Ni_Vi >0)
			HE_1 = m_mesh->next_halfedge_handle(HE_2);
		E_1 = m_mesh->edge_handle(HE_1);
		if(m_mesh->data(E_1).isEdgeOnFront())
		{
			std::cout<<"Recover Process Hits a Front "<< E_1 <<" \n";
			return false;
		}
		else
			_interation_edgehandle.push_back(E_1);
	}

	return true;
}



OpenMesh::FaceHandle FrontsManager::QuadFormation( HalfedgeHandle _front, HalfedgeHandle _side_edge_to, HalfedgeHandle _top_edge, HalfedgeHandle _side_edge_from )
{
	//Save five corner vertex of the quad.
	//shoud have CCW order 
	VertexHandle v1 = m_mesh->from_vertex_handle(_front);

	if(m_mesh->from_vertex_handle(_side_edge_from) == v1)
		_side_edge_from = m_mesh->opposite_halfedge_handle(_side_edge_from);
	VertexHandle v2 = m_mesh->from_vertex_handle(_side_edge_from);
	
	if(m_mesh->from_vertex_handle(_top_edge) == v2)
		_top_edge = m_mesh->opposite_halfedge_handle(_top_edge);
	VertexHandle v3 = m_mesh->from_vertex_handle(_top_edge);

	if(m_mesh->from_vertex_handle(_side_edge_to) == v3)
		_side_edge_to = m_mesh->opposite_halfedge_handle(_side_edge_to);
	VertexHandle v4 = m_mesh->from_vertex_handle(_side_edge_to);

	std::vector<VertexHandle> _quad_vertex_container;
	_quad_vertex_container.push_back(v1);
	_quad_vertex_container.push_back(v2);
	_quad_vertex_container.push_back(v3);
	_quad_vertex_container.push_back(v4);

	//delete the face inside

	//compute the inside boundary
	HalfedgeHandle _front_inside = m_mesh->opposite_halfedge_handle(_front);
	HalfedgeHandle _side_edge_to_inside = m_mesh->opposite_halfedge_handle(_side_edge_to);
	HalfedgeHandle _top_edge_inside = m_mesh->opposite_halfedge_handle(_top_edge);
	HalfedgeHandle _side_edge_from_inside = m_mesh->opposite_halfedge_handle(_side_edge_from);


//	DeleteFacesUsingBoundary(m_mesh, m_mesh->opposite_halfedge_handle(_front), _side_edge_to, _top_edge, _side_edge_from);
	DeleteFacesUsingBoundary(m_mesh, _front_inside, _front_inside,_side_edge_to_inside, _top_edge_inside, _side_edge_from_inside);
	m_mesh->garbage_collection();

	try {
		if ( !OpenMesh::IO::write_mesh(*m_mesh, "firstquad_before.ply") ) {
			std::cerr << "Cannot write mesh to file 'output.off'" << std::endl;
			//return false;
		}
	}
	catch( std::exception& x )
	{
		std::cerr << x.what() << std::endl;
		//return false;
	}

	FaceHandle f = m_mesh->add_face(v1, v2, v3, v4);

	int _front_level = m_mesh->data(_front).frontLevel();
	m_mesh->data(f).setQuadLevel(_front_level);

	try {
		if ( !OpenMesh::IO::write_mesh(*m_mesh, "firstquad.ply") ) {
			std::cerr << "Cannot write mesh to file 'output.off'" << std::endl;
			//return false;
		}
	}
	catch( std::exception& x )
	{
		std::cerr << x.what() << std::endl;
		//return false;
	}
	return f;
}

bool FrontsManager::SideEdgeRecovery( HalfedgeHandle _front, int reconsturct_type,  HalfedgeHandle & _side_edge)
{

	HalfedgeHandle front_he_in;
	HalfedgeHandle front_he_out;

	if(reconsturct_type == RC_TO_SE)
	{
		front_he_in = _front;
		front_he_out = m_mesh->data(_front).nextFrontHalfEdge();
	}
	else if(reconsturct_type == RC_FROM_SE)
	{
		front_he_out = _front;
		front_he_in = m_mesh->data(_front).prevFrontHalfEdge();
	}

	//compute the bisector of the front
	Vector3D mesh_normal = m_mesh->calc_face_normal(m_mesh->opposite_face_handle(_front));
	Vector3D vec_front_he_in = HalfEdge_to_Vector(m_mesh,front_he_in);
	Vector3D vec_front_he_out = HalfEdge_to_Vector(m_mesh, front_he_out);

	Vector3D normal_front_he_in = cross((Vec3f)vec_front_he_in, (Vec3f)mesh_normal);
	Vector3D normal_front_he_out = cross((Vec3f)vec_front_he_out, (Vec3f)mesh_normal);

	normal_front_he_in /= normal_front_he_in.norm();
	normal_front_he_out /= normal_front_he_out.norm();

	Vector3D Vk = normal_front_he_in + normal_front_he_out;
	Vk /= Vk.norm();
	Vk *= (vec_front_he_in.norm() + vec_front_he_out.norm())/2;


DEECTION:

	PolyFront_Mesh::VertexOHalfedgeIter v_o_he_it = m_mesh->voh_iter(m_mesh->from_vertex_handle(front_he_out));
	float smallest_angle = M_PI;
	HalfedgeHandle halfedge_with_smallest_angle;
	for (; v_o_he_it; ++v_o_he_it)
	{
		HalfedgeHandle _he = v_o_he_it.handle();
		Vector3D _vec_he = HalfEdge_to_Vector(m_mesh, _he);
		float dot_projection = dot(_vec_he, Vk)/(_vec_he.norm() * Vk.norm());
		float _angle = acos(dot_projection);
		if(_angle < smallest_angle)
		{
			smallest_angle = _angle;
			halfedge_with_smallest_angle = _he;
		}
	}

	VertexHandle Nm = m_mesh->to_vertex_handle(halfedge_with_smallest_angle);
	bool _oppsite_side_selectable = false;
	bool _oppsite_side = false;
	if (m_mesh->data(Nm).isVertexOnFront())
	{
		//Tag: FRCL
		//NOTE:+++++++++++++++++++++++++++
		// Front Closing Algorithm put here
		//Even/Odd loop check, implement later
		//HalfedgeHandle he_result;
		//if (reconsturct_type == RC_FROM_SE)
		//	halfedge_with_smallest_angle = m_mesh->opposite_halfedge_handle(halfedge_with_smallest_angle);
		//_side_edge = halfedge_with_smallest_angle;
		//assert(_side_edge.is_valid());
		std::cout<<"Hit the front!!!! Front Closing \n";
		//Check is loop-1 even number, if it does, select it
		_oppsite_side = true;
		int lopp_1_fronts_number = 0;			//equals loop-number -2
		HalfedgeHandle looper_from = _front;
		VertexHandle v_looper_start = m_mesh->from_vertex_handle(looper_from);
		
		while(v_looper_start != Nm)
		{
			looper_from = m_mesh->data(looper_from).prevFrontHalfEdge();
			v_looper_start = m_mesh->from_vertex_handle(looper_from);
			lopp_1_fronts_number++;
		}


		int loop_2_fronts_number = 0; //equals loop-number -2
		HalfedgeHandle looper_to = _front;
		VertexHandle v_looper_to = m_mesh->to_vertex_handle(looper_to);
		while(v_looper_to != Nm){
			looper_to = m_mesh->data(looper_to).nextFrontHalfEdge();
			v_looper_to =  m_mesh->to_vertex_handle(looper_to);
			loop_2_fronts_number++;
		}

		if (loop_2_fronts_number%2 ==0 && lopp_1_fronts_number %2==0)
			_oppsite_side_selectable == true;

		//return true;
	}

	if ( 
		(smallest_angle <= M_PI/6 && ! _oppsite_side) || 
		(  smallest_angle <= M_PI/4&&_oppsite_side&&_oppsite_side_selectable )  //NOTE:+++++++++++++using one constant here !!!!!!!
		)
	{
		// have a exist edge to select
		if (reconsturct_type == RC_FROM_SE)
			halfedge_with_smallest_angle = m_mesh->opposite_halfedge_handle(halfedge_with_smallest_angle);
		_side_edge = halfedge_with_smallest_angle;
		assert(_side_edge.is_valid());
		return true;
	}

	//do not have exist edge to select
	//use flip operation to creat edge
	VertexHandle start_vertex_handle = m_mesh->from_vertex_handle(front_he_out);
	FaceHandle face_contain_vk = getFaceContainVector(m_mesh, m_mesh->from_vertex_handle(front_he_out), Vk);
 	HalfedgeHandle He_opp = getOppsiteHalfedgeInFace(m_mesh, face_contain_vk, start_vertex_handle);
	EdgeHandle E_opp = m_mesh->edge_handle(He_opp);
	if (m_mesh->data(E_opp).isEdgeOnFront())
	{
		std::cout<<"Can not filp edge, will destory the front!!!!!\n";
		//do something.... actully, the program will not come here, if the oppsite vertex is front, 
		//code will return the side edge in Tag: FRCL
		system("pause");
		//HalfedgeHandle he_result;
		return false;
	}

	FaceHandle Face_opp_opp = m_mesh->opposite_face_handle(He_opp);
	HalfedgeHandle He_opp_opp = m_mesh->opposite_halfedge_handle(He_opp);
	VertexHandle Nm_opp = getOppsiteVertexInFace(m_mesh, Face_opp_opp, He_opp_opp);

	//////////////////////////////////////////////////////////////////////////
	//Have to detect if the Nm opp is on front.
	//Even/Odd Check

	//////////////////////////////////////////////////////////////////////////














	//////////////////////////////////////////////////////////////////////////

	Vector3D _vec_start_Nm = Vertex_to_Vector(m_mesh, start_vertex_handle, Nm_opp);
	float _angle_Nm = acos((float) dot(_vec_start_Nm, Vk)/(_vec_start_Nm.norm()*Vk.norm()));

	if (_angle_Nm < M_PI/6 && _vec_start_Nm.norm() < sqrt(3.0)*Vk.norm())
	{
		//NOTE:+++++++++++++++++++++++++++++++++++++++++++++
		//May put triangle inverse here, seems we have vector contain check here, 
		//do check at this time. 
		HalfedgeHandle heh_1 = m_mesh->halfedge_handle(E_opp, 0);
		HalfedgeHandle heh_2 = m_mesh->halfedge_handle(E_opp, 1);

		//to check if the flip operation will cause inverse triangle
		bool _is_inverse = false;

		HalfedgeHandle tri_1_he_1 = m_mesh->next_halfedge_handle(heh_1);
		HalfedgeHandle tri_1_he_2 = m_mesh->prev_halfedge_handle(heh_2);

		HalfedgeHandle tri_2_he_1 = m_mesh->next_halfedge_handle(heh_2);
		HalfedgeHandle tri_2_he_2 = m_mesh->prev_halfedge_handle(heh_1);

		Vector3D vec_tri_1_he_1 = HalfEdge_to_Vector(m_mesh, tri_1_he_1);
		Vector3D vec_tri_1_he_2 = HalfEdge_to_Vector(m_mesh, tri_1_he_2);

		Vec3f cross_res = cross(vec_tri_1_he_2, vec_tri_1_he_1);
		if(cross_res[2] <0)
			_is_inverse = true;

		Vector3D vec_tri_2_he_1 = HalfEdge_to_Vector(m_mesh, tri_2_he_1);
		Vector3D vec_tri_2_he_2 = HalfEdge_to_Vector(m_mesh, tri_2_he_2);
		cross_res = cross(vec_tri_2_he_2, vec_tri_2_he_1);
		if(cross_res[2]<0)
			_is_inverse = true;

		if(_is_inverse){
			std::cout<<"Can not filp edge, Inverse Triangle!!!!!\n";
			system("pause");
			return false;
		}
		else{
			VertexHandle v_flip_tri_1_1 = m_mesh->from_vertex_handle(tri_1_he_1);
			VertexHandle v_flip_tri_1_2 = m_mesh->to_vertex_handle(tri_1_he_1);
			VertexHandle v_flip_tri_1_3 = m_mesh->from_vertex_handle(tri_1_he_2);

			VertexHandle v_flip_tri_2_1 = m_mesh->from_vertex_handle(tri_2_he_1);
			VertexHandle v_flip_tri_2_2 = m_mesh->to_vertex_handle(tri_2_he_1);
			VertexHandle v_flip_tri_2_3 = m_mesh->from_vertex_handle(tri_2_he_2);

			std::vector<PolyFront_Mesh::VertexHandle>  tmp_face_1_vhandles;
			tmp_face_1_vhandles.push_back(v_flip_tri_1_1);
			tmp_face_1_vhandles.push_back(v_flip_tri_1_2);
			tmp_face_1_vhandles.push_back(v_flip_tri_1_3);

			std::vector<PolyFront_Mesh::VertexHandle>  tmp_face_2_vhandles;
			tmp_face_2_vhandles.push_back(v_flip_tri_2_1);
			tmp_face_2_vhandles.push_back(v_flip_tri_2_2);
			tmp_face_2_vhandles.push_back(v_flip_tri_2_3);

			//remove the edge->halfedge->face
			//m_mesh->delete_edge(E_opp);
			m_mesh->delete_edge(E_opp, false);
			m_mesh->garbage_collection();

			//add the new face, finish the processing of flip
			FaceHandle tri_face_1 = m_mesh->add_face(tmp_face_1_vhandles);
			FaceHandle tri_face_2 = m_mesh->add_face(tmp_face_2_vhandles);


			//-------------------------------Some Testing output code ---------------------
			try {
				if ( !OpenMesh::IO::write_mesh(*m_mesh, "side_edge.ply") ) {
					std::cerr << "Cannot write mesh to file 'output.off'" << std::endl;
					return false;
				}
			}
			catch( std::exception& x )
			{
				std::cerr << x.what() << std::endl;
				return false;
			}
			//------------------------------------- ----------------------------------------------------

			EdgeHandle common_edge;
			if(!getCommonEdge(m_mesh, tri_face_1, tri_face_2, common_edge)){
				std::cout<<"Some error in face construction!!!!!";
				assert(0);
				return false;
			}
			else{
				//return the correct halfedge following the reconstruct type
				HalfedgeHandle common_he_1 = m_mesh->halfedge_handle(common_edge, 0);
				HalfedgeHandle common_he_2 = m_mesh->halfedge_handle(common_edge, 1);

				//VertexHandle common_from = m_mesh->from_vertex_handle(common_he);

				//BUG HERE: HAVE TO MAKE SURE THE RIGHT TYPE OUTPUT

				HalfedgeHandle common_he = m_mesh->find_halfedge(start_vertex_handle, Nm_opp);
				if (reconsturct_type == RC_TO_SE)
					_side_edge = common_he;
				else
					_side_edge = m_mesh->opposite_halfedge_handle(common_he);
				VertexHandle v_t = m_mesh->to_vertex_handle(_side_edge);
				VertexHandle v_f = m_mesh->from_vertex_handle(_side_edge);

				PolyFront_Mesh::Point pvt = m_mesh->point(v_t);
				PolyFront_Mesh::Point pvf = m_mesh->point(v_f);

				assert(_side_edge.is_valid());
				return true;
			}
		}
	}

	//Have to slip the edge to reconstruct side edge
	else
	{
		//compute the intersection point
		VertexHandle v1 = m_mesh->to_vertex_handle(He_opp_opp);
		VertexHandle v2 = m_mesh->from_vertex_handle(He_opp_opp);

		PolyFront_Mesh::Point p1 = m_mesh->point(v1);
		PolyFront_Mesh::Point p2 = m_mesh->point(v2);

		VertexHandle v3 = m_mesh->from_vertex_handle(front_he_out);
		PolyFront_Mesh::Point p3 = m_mesh->point(v3);

		//Constant here!!!!!!
		PolyFront_Mesh::Point p4 = p3 + Vk*10/Vk.norm();

		float ua, ub;
		if(!is_PointSegment_Initersection(p1, p2, p3, p4, ua, ub)){
			std::cout<<"No intersection!!!!\n";
			assert(0);
			system("pause");
		}

		PolyFront_Mesh::Point inter_p = p1*(1.0-ua) + p2*ua;


		//Split the mesh

		try {
			if ( !OpenMesh::IO::write_mesh(*m_mesh, "before_split.ply") ) {
				std::cerr << "Cannot write mesh to file 'output.off'" << std::endl;
				//return false;
			}
		}
		catch( std::exception& x )
		{
			std::cerr << x.what() << std::endl;
			//return false;
		}

		FaceHandle f1 = m_mesh->face_handle(He_opp);
		FaceHandle f2 = m_mesh->face_handle(He_opp_opp);

		VertexHandle new_vertex_handle = m_mesh->add_vertex(inter_p);
		//save the prev node for further reconver triangle
		VertexHandle _vertex_handle_E_opp_to = m_mesh->to_vertex_handle(He_opp);
		VertexHandle _vertex_handle_E_opp_from = m_mesh->from_vertex_handle(He_opp);

		m_mesh->delete_edge(E_opp, false);
		m_mesh->garbage_collection();


//		m_mesh->delete_face(f1, false);
//		m_mesh->delete_face(f2, false);

		FaceHandle new_face = m_mesh->add_face(_vertex_handle_E_opp_to, 
			start_vertex_handle, 
			_vertex_handle_E_opp_from, 
			Nm_opp);

		m_mesh->split(new_face, new_vertex_handle);

		//m_mesh->split(E_opp, new_vertex_handle);
		
		//HalfedgeHandle he_split_1 = m_mesh->find_halfedge(new_vertex_handle, _vertex_handle_temp);
		//HalfedgeHandle he_split_2 = m_mesh->opposite_halfedge_handle(he_split_1);



		//m_mesh->triangulate(f1);
		//m_mesh->triangulate(f2);

//		m_mesh->triangulate(m_mesh->face_handle(he_split_1));
//		m_mesh->triangulate(m_mesh->face_handle(he_split_2));

//		m_mesh->insert_edge(new_vertex_handle, start_vertex_handle);
//		m_mesh->insert_edge(new_vertex_handle, Nm_opp);



		
		try {
			if ( !OpenMesh::IO::write_mesh(*m_mesh, "after_split.ply") ) {
				std::cerr << "Cannot write mesh to file 'output.off'" << std::endl;
				//return false;
			}
		}
		catch( std::exception& x )
		{
			std::cerr << x.what() << std::endl;
			//return false;
		}
		
		goto DEECTION;
	}
}

bool FrontsManager::QuadSmooth( FaceHandle _quad )
{
	PolyFront_Mesh::FaceVertexIter fvit = m_mesh->fv_begin();
	PolyFront_Mesh::FaceVertexIter fvit_end = m_mesh->fv_end();
	for (;fvit != fvit_end; ++fvit)
	{
		VertexHandle v = fvit.handle();
		bool _success = VertexSmooth(v);
	}

}

bool FrontsManager::VertexSmooth( VertexHandle _ver )
{
	//check if the vertex locates on front
	bool is_vertex_on_front  = m_mesh->data(_ver).



}


