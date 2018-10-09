#include "util/log.hpp"
#include "util/timing_util.hpp"

#include "osrm/match_parameters.hpp"
#include "osrm/nearest_parameters.hpp"
#include "osrm/route_parameters.hpp"
#include "osrm/table_parameters.hpp"
#include "osrm/trip_parameters.hpp"

#include "osrm/coordinate.hpp"
#include "osrm/extractor_config.hpp"
#include "partitioner/partitioner.hpp"
#include "partitioner/partitioner_config.hpp"
#include "osrm/engine_config.hpp"
#include "osrm/json_container.hpp"

#include "osrm/extractor.hpp"
#include "osrm/osrm.hpp"
#include "osrm/status.hpp"

#include <tbb/task_scheduler_init.h>
#include <exception>
#include <iostream>
#include <string>
#include <utility>
#include <cstdlib>

using namespace osrm;

std::string osm_path = "../datasets/maps/new-york-latest.osm.pbf";
std::string base_path = "../datasets/osrm/nyc";
std::string profile_path = "../datasets/profiles/car.lua";

auto check_file = [](const boost::filesystem::path &path) {
	if (!boost::filesystem::is_regular_file(path))
	{
		util::Log(logERROR) << "Input file " << path << " not found!";
		return false;
	}
	else
	{
		return true;
	}
};

void extract() {
	ExtractorConfig extractor_config;
	extractor_config.generate_edge_lookup = false;
	extractor_config.requested_num_threads = 1;
	extractor_config.base_path = base_path;
	extractor_config.input_path = osm_path;
	extractor_config.profile_path = profile_path;

	if (!check_file(extractor_config.GetPath(".osrm"))) {
		extract(extractor_config);
	}
	std::cout << "==================Extraction Complete================" << std::endl;
}

int partition() {

	partitioner::PartitionerConfig partition_config;
	partition_config.base_path = base_path;
	partition_config.requested_num_threads = tbb::task_scheduler_init::default_num_threads();
	partition_config.balance = 1.2;
	partition_config.boundary_factor = 0.25;
	partition_config.num_optimizing_cuts = 10;
	partition_config.small_component_size = 1000;
	//partition_config.max_cell_sizes = std::vector<size_t>(4);

	if (!check_file(partition_config.GetPath(".osrm.ebg")) ||
		!check_file(partition_config.GetPath(".osrm.cnbg_to_ebg")) ||
		!check_file(partition_config.GetPath(".osrm.cnbg")))
	{
		return EXIT_FAILURE;
	}

	util::Log() << "Computing recursive bisection";

	TIMER_START(bisect);
	auto exitcode = partitioner::Partitioner().Run(partition_config);
	TIMER_STOP(bisect);
	util::Log() << "Bisection took " << TIMER_SEC(bisect) << " seconds.";
}

int main(int argc, const char *argv[])
{ 
	util::LogPolicy::GetInstance().Unmute();
	std::string verbosity = "DEBUG";
	util::LogPolicy::GetInstance().SetLevel(verbosity);

	// step 1
	extract();

	// step 2
	partition();

	// Configure based on a .osrm base path
	EngineConfig config;

	config.storage_config = { base_path };
	config.use_shared_memory = false;

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