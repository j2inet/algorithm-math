#include <iostream>

struct Point
{
	union {
		struct {
			float x;
			float y;
		};
		float pointList[2];
	};
};

struct Triangle
{
	union {
		struct {
			Point p1;
			Point p2;
			Point p3;
		};
		Point pointList[3];
	};	
};



inline float sign(Point p1, Point p2, Point p3)
{
	return (p1.x - p3.x) * (p2.y - p3.y) - (p2.x - p3.x) * (p1.y - p3.y);
}

bool pointInTriangle(Point pt, Triangle tri)
{
	float d1, d2, d3;
	bool has_neg, has_pos;
	d1 = sign(pt, tri.pointList[0], tri.pointList[1]);
	d2 = sign(pt, tri.pointList[1], tri.pointList[2]);
	d3 = sign(pt, tri.pointList[2], tri.pointList[0]);
	has_neg = (d1 < 0) || (d2 < 0) || (d3 < 0);
	has_pos = (d1 > 0) || (d2 > 0) || (d3 > 0);
	return !(has_neg && has_pos);
}

inline double Det2D(const Triangle& triangle)
{
	return +triangle.p1.x* (triangle.p2.y- triangle.p3.y)
		+ triangle.p2.x * (triangle.p3.y - triangle.p1.y)
		+ triangle.p3.x * (triangle.p1.y - triangle.p2.y);
}

void CheckTriWinding(const Triangle t, bool allowReversed = true)
{
	double detTri = Det2D(t);
	[[unlikely]]
	if (detTri < 0.0)
	{
		if (allowReversed)
		{
			Triangle tReverse = t;
			tReverse.p1 = t.p3;
			tReverse.p3 = t.p1;
			tReverse.p2 = t.p2;
			return CheckTriWinding(tReverse, false);
		}
		else throw std::runtime_error("triangle has wrong winding direction");
	}
}

bool BoundaryCollideChk(const Triangle& t, double eps)
{
	return Det2D(t) < eps;
}

bool BoundaryCollideChk(const Point& p1, const Point& p2, const Point& p3, double eps)
{
	Triangle t = { {{p1, p2, p3}} };
	return BoundaryCollideChk(t, eps);

}

bool BoundaryDoesntCollideChk(const Triangle& t, double eps)
{
	return Det2D(t) <= eps;
}

bool BoundaryDoesntCollideChk(const Point& p1, const Point& p2, const Point& p3, double eps)
{
	Triangle t = { {{p1, p2, p3}} };
	return BoundaryDoesntCollideChk(t, eps);
}



bool TriangleTriangleCollision(const Triangle& triangle1,
	const Triangle& triangle2,
	double eps = 0.0, bool allowReversed = true, bool onBoundary = true)
{
	//Trangles must be expressed anti-clockwise
	CheckTriWinding(triangle1, allowReversed);
	CheckTriWinding(triangle2, allowReversed);	

	//For edge E of trangle 1,
	for (int i = 0; i < 3; i++)
	{
		int j = (i + 1) % 3;
		[[likely]]
		if (onBoundary)
		{

			//Check all points of trangle 2 lay on the external side of the edge E. If
			//they do, the triangles do not collide.
			if (BoundaryCollideChk(triangle1.pointList[i], triangle1.pointList[j], triangle2.pointList[0], eps) &&
				BoundaryCollideChk(triangle1.pointList[i], triangle1.pointList[j], triangle2.pointList[1], eps) &&
				BoundaryCollideChk(triangle1.pointList[i], triangle1.pointList[j], triangle2.pointList[2], eps))
				return false;
		}
		else
		{
			if (BoundaryDoesntCollideChk(triangle1.pointList[i], triangle1.pointList[j], triangle2.pointList[0], eps) &&
				BoundaryDoesntCollideChk(triangle1.pointList[i], triangle1.pointList[j], triangle2.pointList[1], eps) &&
				BoundaryDoesntCollideChk(triangle1.pointList[i], triangle1.pointList[j], triangle2.pointList[2], eps))
				return false;
		}

		if (onBoundary)
		{

			//Check all points of trangle 2 lay on the external side of the edge E. If
			//they do, the triangles do not collide.
			if (BoundaryCollideChk(triangle2.pointList[i], triangle2.pointList[j], triangle1.pointList[0], eps) &&
				BoundaryCollideChk(triangle2.pointList[i], triangle2.pointList[j], triangle1.pointList[1], eps) &&
				BoundaryCollideChk(triangle2.pointList[i], triangle2.pointList[j], triangle1.pointList[2], eps))
				return false;
		}
		else
		{
			if (BoundaryDoesntCollideChk(triangle2.pointList[i], triangle2.pointList[j], triangle1.pointList[0], eps) &&
				BoundaryDoesntCollideChk(triangle2.pointList[i], triangle2.pointList[j], triangle1.pointList[1], eps) &&
				BoundaryDoesntCollideChk(triangle2.pointList[i], triangle2.pointList[j], triangle1.pointList[2], eps))
				return false;
		}
	}
	//The triangles collide
	return true;
}


int main()
{
	{
		Point p1 = { 5, 7 };
		Point p2 = { 3, 4 };
		Point p3 = { 3, 3 };

		Triangle t1 = { Triangle{Point{2, 2}, Point{5, 6}, Point{10, 0} } };


		std::wcout << "Point p1 " << (pointInTriangle(p1, t1) ? "is" : "is not") << " in triangle t1" << std::endl;
		std::wcout << "Point p2 " << (pointInTriangle(p2, t1) ? "is" : "is not") << " in triangle t1" << std::endl;
		std::wcout << "Point p3 " << (pointInTriangle(p3, t1) ? "is" : "is not") << " in triangle t1" << std::endl;
	}

	{
Triangle t1 = { Triangle{Point{3, 6}, Point{6, 5}, Point{6, 7} } },
			t2 = { Triangle{Point{4, 2}, Point{1, 5}, Point{6, 4} } },
			t3 = { Triangle{Point{3, 12}, Point{9, 8}, Point{9, 12} } },
			t4 = { Triangle{Point{3, 10}, Point{5, 9}, Point{5, 13} } };

auto collision1 = TriangleTriangleCollision(t1, t2);
std::wcout << "Triangles t1 and t2 " << (collision1 ? "do" : "do not") << " collide" << std::endl;
auto collision2 = TriangleTriangleCollision(t3, t4);
std::wcout << "Triangles t3 and t4 " << (collision2 ? "do" : "do not") << " collide" << std::endl;
auto collision3 = TriangleTriangleCollision(t1, t3);
std::wcout << "Triangles t1 and t3 " << (collision3 ? "do" : "do not") << " collide" << std::endl;


	}

}

