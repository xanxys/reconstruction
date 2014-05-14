#pragma once

#include <string>
#include <thread>
#include <vector>

#include <boost/optional.hpp>
#include <jsoncpp/json/json.h>

#include "data_source.h"

class EstimateL1Job {
public:
	// Existence of Json::Value somehow prevents default move constructor
	// generation, so we need manual move constructor.
	EstimateL1Job(EstimateL1Job&& that);
	EstimateL1Job(std::string id, DataSource& data_source, std::vector<std::string> scene_ids);

	float getProgress();
	boost::optional<Json::Value> getResult();
private:
	// Calculate L1 norm for all scene_ids and aggreate result as array.
	void estimateL1(std::vector<std::string> scene_ids);

	// assuming shared, high-bandwidth DataSource, scene_id is enough.
	Json::Value mapEstimateL1(std::string scene_id);
public:
	const std::string id;
private:
	DataSource& data_source;
	std::thread worker;

	// TODO: thread-unsafe
	Json::Value result;
	int count;

	const int count_all;
};


// JobPool initiates, maintains, possibly kills jobs.
// Typically jobs depends on filtered version of DataSource.
class JobPool {
public:
	JobPool(DataSource& data_source);

	// TODO: dummy interface for prototype. remove it.
	Json::Value getJobDescription(std::string id);

	// TODO: dummy interface for prototype. remove it.
	std::vector<std::string> listJobs();

	std::string createJob();
private:
	// Borrowed. (TODO: maybe ideally it should be const?)
	DataSource& data_source;

	std::vector<EstimateL1Job> all_jobs;
	int job_id;
};
