/*
 * Graph.hpp
 *
 *  Created on: Mar 24, 2017
 *      Author: nico
 */

#ifndef GRAPH_HPP_
#define GRAPH_HPP_

#include <vector>
#include <algorithm>
#include <iterator>

#include "AStar.hpp"

namespace PathAlgorithm
{

struct Graph
{
private:
	Graph() :
			e(), v()
	{
	}
	virtual ~Graph()
	{
	}
public:
	static Graph& GetInstance()
	{
		static Graph instance;
		return instance;
	}

	const Vertex& getVertex(const std::string& name)
	{
		auto it = std::find_if(v.begin(), v.end(), [&name](const Vertex& rhs)
		{
			return rhs.name==name;
		});
		if (it != v.end())
		{
			return *it;
		}
		throw std::logic_error("Error. Could not find: " + name);
	}

	std::vector<Vertex> GetNeighbours(const Vertex& aVertex)
	{
		std::vector<Vertex> neighbours;

		for (const Edge& edge : e)
		{
			try {
				neighbours.push_back(edge.otherSide(aVertex));
			} catch (std::exception &e) {
			}
			try {
				neighbours.push_back(edge.thisSide(aVertex));
			} catch (std::exception &e) {
			}
		}
		return neighbours;
	}

	std::vector<Edge> e;
	std::vector<Vertex> v;
};

} /* namespace PathAlgorithm */

#endif /* GRAPH_HPP_ */
