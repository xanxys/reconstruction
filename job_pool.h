#pragma once

#include "data_source.h"

// JobPool initiates, maintains, possibly kills jobs.
// Typically jobs depends on filtered version of DataSource.
class JobPool {
public:
	JobPool(DataSource& data_source);

	// TODO: dummy interface for prototype. remove it.
	std::vector<std::string> listJobs();

	std::string createJob();
private:
	// Borrowed. (TODO: maybe ideally it should be consr?)
	DataSource& data_source;

	std::vector<std::string> all_jobs;
	int job_id;
};
