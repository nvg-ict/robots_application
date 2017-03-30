#ifndef ASTAR_HPP_
#define ASTAR_HPP_

//#include "Config.hpp"

#include <iostream>
#include <map>
#include <mutex>
#include <set>
#include <vector>

#include "Point.hpp"
#include "Size.hpp"

#include "forwardKinematics.hpp"

namespace PathAlgorithm
{

struct VertexPoint
{
	/**
	 *
	 */
	VertexPoint(const std::string& aName, int anX, int anY) :
			name(aName), x(anX), y(anY), actualCost(0.0), heuristicCost(0.0)
	{
	}
	/**
	 *
	 */
	VertexPoint(const Point& aPoint) :
			name("UNKNOWN"), x(aPoint.x), y(aPoint.y), actualCost(0.0), heuristicCost(
					0.0)
	{
	}
	/**
	 *
	 */
	Point asPoint() const
	{
		return Point(x, y);
	}
	/**
	 *
	 */
	bool lessCost(const VertexPoint& aVertex) const
	{
		if (heuristicCost < aVertex.heuristicCost)
			return true;
		// less uncertainty if the actual cost is smaller
		if (heuristicCost == aVertex.heuristicCost)
			return actualCost > aVertex.actualCost;
		return false;
	}
	/**
	 *
	 */
	bool lessId(const VertexPoint& aVertex) const
	{
		if (x < aVertex.x)
			return true;
		if (x == aVertex.x)
			return y < aVertex.y;
		return false;
	}
	/**
	 *
	 */
	bool equalPoint(const VertexPoint& aVertex) const
	{
		return x == aVertex.x && y == aVertex.y;
	}

	bool operator==(const VertexPoint& aVertex) const
	{
		return name == aVertex.name;
	}

	std::string name;
	int x;
	int y;

	double actualCost;
	double heuristicCost;
};
// struct VertexPoint

struct Vertex
{
	/**
	 *
	 */
	Vertex(short aPhi1, short aPhi2, short aPhi3) :
			name(
					std::to_string(aPhi1) + "_" + std::to_string(aPhi2) + "_"
							+ std::to_string(aPhi3)), phi1(aPhi1), phi2(aPhi2), phi3(
					aPhi3), actualCost(0.0), heuristicCost(0.0)
	{
	}

	Vertex(const Vertex& v) :
			name(v.name), phi1(v.phi1), phi2(v.phi2), phi3(v.phi3), actualCost(
					v.actualCost), heuristicCost(v.heuristicCost)
	{
	}
	/**
	 *
	 */
//			Vertex( const Point& aPoint) :
//				name("UNKNOWN"),
//				x( aPoint.x),
//				y( aPoint.y),
//				actualCost( 0.0),
//				heuristicCost( 0.0)
//			{
//			}
	/**
	 *
	 */
//			Point asPoint() const
//			{
//				return Point( x, y);
//			}
	/**
	 *
	 */
	bool lessCost(const Vertex& aVertex) const
	{
		if (heuristicCost < aVertex.heuristicCost)
			return true;
		// less uncertainty if the actual cost is smaller
		if (heuristicCost == aVertex.heuristicCost)
			return actualCost > aVertex.actualCost;
		return false;
	}
	/**
	 *
	 */
	bool lessId(const Vertex& aVertex) const
	{
		if (phi1 < aVertex.phi1)
			return true;
		if (phi1 == aVertex.phi1)
			return phi2 < aVertex.phi2;
		if (phi1 == aVertex.phi1 && phi2 == aVertex.phi2)
			return phi3 < aVertex.phi3;
		return false;
	}
	/**
	 *
	 */
	bool equalPoint(const Vertex& aVertex) const
	{
		return phi1 == aVertex.phi1 && phi2 == aVertex.phi2
				&& phi3 == aVertex.phi3;
	}

	bool equalPoint(const VertexPoint& aVertex) const
	{
		std::pair<double, double> pos = calcPosition(phi1, phi2, phi3);
		return pos.first == aVertex.x && pos.second == aVertex.y;
	}

//			bool operator==( const Vertex& aVertex) const
//			{
//				return name == aVertex.name;
//			}

	bool operator==(const Vertex& aVertex) const
	{
		return phi1 == aVertex.phi1 && phi2 == aVertex.phi2 && phi3 == aVertex.phi3;
	}

	std::string name;
	short phi1;
	short phi2;
	short phi3;

	double actualCost;
	double heuristicCost;
};

/**
 *
 */
struct VertexLessCostCompare
{
	bool operator()(const Vertex& lhs, const Vertex& rhs) const
	{
		return lhs.lessCost(rhs);
	}
};
// struct VertexCostCompare
/**
 *
 */
struct VertexLessIdCompare
{
	bool operator()(const Vertex& lhs, const Vertex& rhs) const
	{
		return lhs.lessId(rhs);
	}
};
// struct VertexIdCompare
/**
 *
 */
struct VertexEqualPointCompare
{
	bool operator()(const Vertex& lhs, const Vertex& rhs) const
	{
		return lhs.equalPoint(rhs);
	}
};
// struct VertexPointCompare
struct Edge
{
	Edge(const Vertex& aVertex1, const Vertex& aVertex2) :
			vertex1(aVertex1), vertex2(aVertex2)
	{
	}
	Edge(const Edge& anEdge) :
			vertex1(anEdge.vertex1), vertex2(anEdge.vertex2)
	{
	}

	const Vertex& thisSide(const Vertex& aVertex) const
	{
		if (vertex1.equalPoint(aVertex))
			return vertex1;
		if (vertex2.equalPoint(aVertex))
			return vertex2;
		throw std::logic_error("thisSide: huh???: " + aVertex.name);
	}

	const Vertex& otherSide(const Vertex& aVertex) const
	{
		if (vertex1.equalPoint(aVertex))
			return vertex2;
		if (vertex2.equalPoint(aVertex))
			return vertex1;
		throw std::logic_error("otherSide: huh???: " + aVertex.name);
	}

	Vertex vertex1;
	Vertex vertex2;
};
// struct Edge
/**
 *
 * @param os
 * @param aVertex
 * @return
 */
inline std::ostream& operator<<(std::ostream& os, const Vertex & aVertex)
{
	return os << "(" << aVertex.phi1 << "," << aVertex.phi2 << ","
			<< aVertex.phi3 << "), " << aVertex.actualCost << " "
			<< aVertex.heuristicCost;
}

inline std::ostream& operator<<(std::ostream& os, const VertexPoint & aVertex)
{
	return os << "(" << aVertex.x << "," << aVertex.y << "), " << aVertex.actualCost << " "
			<< aVertex.heuristicCost;
}
/**
 *
 * @param os
 * @param anEdge
 * @return
 */
inline std::ostream& operator<<(std::ostream& os, const Edge& anEdge)
{
	return os << anEdge.vertex1 << " -> " << anEdge.vertex2;
}
/*
 *
 */
typedef std::vector<Vertex> Path;
typedef std::vector<Vertex> OpenSet;
typedef std::set<Vertex, VertexLessIdCompare> ClosedSet;
typedef std::map<Vertex, Vertex, VertexLessIdCompare> VertexMap;
/**
 *
 */
class AStar
{
public:
	/**
	 *
	 */
	Path search(const Vertex& aStartPoint, const Point& aGoalPoint);
	/**
	 *
	 */
	Path search(Vertex aStart, const VertexPoint& aGoal);
	/**
	 *
	 */
	void addToOpenSet(const Vertex& aVertex);
	/**
	 *
	 */
	void removeFromOpenSet(const Vertex& aVertex);
	/**
	 *
	 */
	void removeFromOpenSet(OpenSet::iterator& i);
	/**
	 *
	 */
	OpenSet::iterator findInOpenSet(const Vertex& aVertex);
	/**
	 *
	 */
	bool findRemoveInOpenSet(const Vertex& aVertex);
	/**
	 *
	 */
	void removeFirstFromOpenSet();
	/**
	 *
	 */
	void addToClosedSet(const Vertex& aVertex);
	/**
	 *
	 */
	void removeFromClosedSet(const Vertex& aVertex);
	/**
	 *
	 */
	void removeFromClosedSet(ClosedSet::iterator& i);
	/**
	 *
	 */
	ClosedSet::iterator findInClosedSet(const Vertex& aVertex);
	/**
	 *
	 */
	bool findRemoveClosedSet(const Vertex& aVertex);
	/**
	 *
	 */
	ClosedSet getClosedSet() const;
	/**
	 *
	 */
	OpenSet getOpenSet() const;
	/**
	 *
	 */
	VertexMap getPredecessorMap() const;

protected:
	/**
	 *
	 */
	ClosedSet& getCS();
	/**
	 *
	 */
	const ClosedSet& getCS() const;
	/**
	 *
	 */
	OpenSet& getOS();
	/**
	 *
	 */
	const OpenSet& getOS() const;
	/**
	 *
	 */
	VertexMap& getPM();
	/**
	 *
	 */
	const VertexMap& getPM() const;

private:
	/**
	 *
	 */
	ClosedSet closedSet;
	/**
	 *
	 */
	OpenSet openSet;
	/**
	 *
	 */
	VertexMap predecessorMap;

	mutable std::recursive_mutex openSetMutex;
	mutable std::recursive_mutex closedSetMutex;
	mutable std::recursive_mutex predecessorMapMutex;

};
// class AStar
}// namespace PathAlgorithm
#endif // ASTAR_HPP_
