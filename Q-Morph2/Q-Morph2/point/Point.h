#ifndef _XMESHLIB_CORE_POINT_H_
#define _XMESHLIB_CORE_POINT_H_

#include <assert.h>
#include <math.h>
#include "Front_Traits.h"

namespace XMeshLib{
	class Point	
	{
	public:
		Point( double x, double y, double z ){ v[0] = x; v[1] = y; v[2] = z;}
		Point() { v[0] = v[1] = v[2] = 0; }
		~Point(){}

		double & operator[]( int i)		  { assert( 0<=i && i<3 ); return v[i]; };
		double   operator()( int i) const { assert( 0<=i && i<3 ); return v[i]; };
		double   operator[]( int i) const { assert( 0<=i && i<3 ); return v[i]; };
		double norm() const { return sqrt( fabs( v[0] * v[0] + v[1] * v[1] + v[2] * v[2] ) ); };
		void SetZero(){v[0] = v[1] = v[2] = 0;}

		Point  & operator += ( const Point & p) { v[0] += p(0); v[1] += p(1); v[2] += p(2); return *this; }; 
		Point  & operator -= ( const Point & p) { v[0] -= p(0); v[1] -= p(1); v[2] -= p(2); return *this; };
		Point  & operator *= ( const double  s) { v[0] *= s   ; v[1] *=    s; v[2] *=    s; return *this; };
		Point  & operator /= ( const double  s) { v[0] /= s   ; v[1] /=    s; v[2] /=    s; return *this; };

		double   operator*( const Point & p ) const 
		{
			return v[0] * p[0] + v[1] * p[1] + v[2] * p[2]; 
		};

		Point   operator+( const Point & p  ) const
		{
			Point r( v[0] + p[0], v[1] + p[1], v[2] + p[2] );
			return r;
		};
		Point   operator-( const Point & p  ) const
		{
			Point r( v[0] - p[0], v[1] - p[1], v[2] - p[2] );
			return r;
		};
		Point   operator*( const double s  ) const
		{
			Point r( v[0] * s, v[1] * s, v[2] * s );
			return r;
		};
		Point   operator/( const double s  ) const
		{
			Point r( v[0] / s, v[1] / s, v[2] / s );
			return r;
		};

		Point operator^( const Point & p2) const
		{
			Point r( v[1] * p2[2] - v[2] * p2[1],
					 v[2] * p2[0] - v[0] * p2[2],
					 v[0] * p2[1] - v[1] * p2[0]);
			return r;
		};

		Point operator-() const
		{
			Point p(-v[0],-v[1],-v[2]);
			return p;
		};
	
		double v[3];
	};

}//name space XMeshLib

//typedef XMeshLib::Point Vector3D;
typedef PolyFront_Mesh::Point Vector3D; 

Vector3D HalfEdge_to_Vector(PolyFront_Mesh * _mesh, HalfedgeHandle _heh){
	PolyFront_Mesh::Point p_from = _mesh->point(_mesh->from_vertex_handle(_heh));
	PolyFront_Mesh::Point p_to = _mesh->point(_mesh->to_vertex_handle(_heh));
	return (p_to - p_from);
}

Vector3D Vertex_to_Vector(PolyFront_Mesh * _mesh, VertexHandle _v_start, VertexHandle _v_end){
	PolyFront_Mesh::Point p_from = _mesh->point(_v_start);
	PolyFront_Mesh::Point p_to = _mesh->point(_v_end);
	return (p_to -p_from);
}


bool is_PointSegment_Initersection(PolyFront_Mesh::Point p1, 
								   PolyFront_Mesh::Point p2,
								   PolyFront_Mesh::Point p3,
								   PolyFront_Mesh::Point p4,
								   float & ua, float& ub)
{
	float x1 = p1[0];
	float y1 = p1[1];

	float x2 = p2[0];
	float y2 = p2[1];

	float x3 = p3[0];
	float y3 = p3[1];

	float x4 = p4[0];
	float y4 = p4[1];

	 ua = ((x4 - x3)*(y1-y3) - (y4-y3)*(x1-x3))/
		((y4-y3)*(x2-x1) - (x4-x3)*(y2-y1));
	
	
	  ub = ((x2-x1)*(y1-y3) - (y2-y1)*(x1-x3))/
		((y4-y3)*(x2-x1)-(x4-x3)*(y2-y1));

	if(ua > 1e-3 && ua <1.0 && ub >1e-3 && ub <1.0)
		return true;
	else
		return false;
}

bool Test_is_algorithm_correct(){
	float x1 = 0;
	float y1 = 0;

	float x2 = 1;
	float y2 = 1;

	float x3 = 0;
	float y3 = 1;

	float x4 = 1;
	float y4 = 0;

	float ua = ((x4 - x3)*(y1-y3) - (y4-y3)*(x1-x3))/
		((y4-y3)*(x2-x1) - (x4-x3)*(y2-y1));


	float  ub = ((x2-x1)*(y1-y3) - (y2-y1)*(x1-x3))/
		((y4-y3)*(x2-x1)-(x4-x3)*(y2-y1));

	if(ua > 1e-3 && ua <1.0 && ub >1e-3 && ub <1.0)
		return true;
}

bool is_VertexSegment_Intersection(PolyFront_Mesh * m, 
								   VertexHandle v1, VertexHandle v2, 
								   VertexHandle v3, VertexHandle v4)
{
	PolyFront_Mesh::Point p1 = m->point(v1);
	PolyFront_Mesh::Point p2 = m->point(v2);
	PolyFront_Mesh::Point p3 = m->point(v3);
	PolyFront_Mesh::Point p4 = m->point(v4);

	//test the algorithm
	assert(Test_is_algorithm_correct());

	float ua, ub;
	return is_PointSegment_Initersection(p1, p2, p3, p4, ua, ub);
}

bool is_VertexSegment_Intersection(PolyFront_Mesh * m, 
								   VertexHandle v1, VertexHandle v2, 
								   VertexHandle v3, VertexHandle v4,
								   float &ua, float& ub)
{
	PolyFront_Mesh::Point p1 = m->point(v1);
	PolyFront_Mesh::Point p2 = m->point(v2);
	PolyFront_Mesh::Point p3 = m->point(v3);
	PolyFront_Mesh::Point p4 = m->point(v4);

	//test the algorithm
	assert(Test_is_algorithm_correct());

	//float ua, ub;
	return is_PointSegment_Initersection(p1, p2, p3, p4, ua, ub);
}



#endif //_XMESHLIB_CORE_POINT_H_ defined