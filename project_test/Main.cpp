#include "osrm/match_parameters.hpp"
#include "osrm/nearest_parameters.hpp"
#include "osrm/route_parameters.hpp"
#include "osrm/table_parameters.hpp"
#include "osrm/trip_parameters.hpp"

#include "osrm/coordinate.hpp"
#include "osrm/extractor_config.hpp"
#include "osrm/engine_config.hpp"
#include "osrm/json_container.hpp"

#include "osrm/extractor.hpp"
#include "osrm/osrm.hpp"
#include "osrm/status.hpp"

#include <exception>
#include <iostream>
#include <string>
#include <utility>

#include <cstdlib>

using namespace osrm;

int main(int argc, const char *argv[])
{
	std::string base_path = "../datasets/osrm/nyc";
	std::string osm_path = "../datasets/maps/new-york.osm.pbf";
	std::string profile_path = "../datasets/profiles/car.lua";

	ExtractorConfig extractor_config;
	extractor_config.generate_edge_lookup = true;
	extractor_config.requested_num_threads = 4;
	extractor_config.base_path = base_path;
	extractor_config.input_path = osm_path;
	extractor_config.profile_path = profile_path;

	extract(extractor_config);

	// Configure based on a .osrm base path, and no datasets in shared mem from osrm-datastore
	EngineConfig config;

	config.storage_config = { base_path };
	config.use_shared_memory = true;

	// We support two routing speed up techniques:
	// - Contraction Hierarchies (CH): requires extract+contract pre-processing
	// - Multi-Level Dijkstra (MLD): requires extract+partition+customize pre-processing
	//
	// config.algorithm = EngineConfig::Algorithm::CH;
	config.algorithm = EngineConfig::Algorithm::MLD;

	// Routing machine with several services (such as Route, Table, Nearest, Trip, Match)
	const OSRM osrm{ config };

	// The following shows how to use the Route service; configure this service
	RouteParameters params;

	// Route in monaco
	params.coordinates.push_back({ util::FloatLongitude{ 7.419758 }, util::FloatLatitude{ 43.731142 } });
	params.coordinates.push_back({ util::FloatLongitude{ 7.419505 }, util::FloatLatitude{ 43.736825 } });

	// Response is in JSON format
	json::Object result;

	// Execute routing request, this does the heavy lifting
	const auto status = osrm.Route(params, result);

	if (status == Status::Ok)
	{
		auto &routes = result.values["routes"].get<json::Array>();

		// Let's just use the first route
		auto &route = routes.values.at(0).get<json::Object>();
		const auto distance = route.values["distance"].get<json::Number>().value;
		const auto duration = route.values["duration"].get<json::Number>().value;

		// Warn users if extract does not contain the default coordinates from above
		if (distance == 0 || duration == 0)
		{
			std::cout << "Note: distance or duration is zero. ";
			std::cout << "You are probably doing a query outside of the OSM extract.\n\n";
		}

		std::cout << "Distance: " << distance << " meter\n";
		std::cout << "Duration: " << duration << " seconds\n";
		system("PAUSE");
		return EXIT_SUCCESS;
	}
	else if (status == Status::Error)
	{
		const auto code = result.values["code"].get<json::String>().value;
		const auto message = result.values["message"].get<json::String>().value;

		std::cout << "Code: " << code << "\n";
		std::cout << "Message: " << code << "\n";
		system("PAUSE");
		return EXIT_FAILURE;
	}
}