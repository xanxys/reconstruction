#include "job_pool.h"

JobPool::JobPool(DataSource& data_source) : data_source(data_source), job_id(0) {

}

std::vector<std::string> JobPool::listJobs() {
	return all_jobs;
}

std::string JobPool::createJob() {
	const std::string id = std::to_string(job_id++);
	all_jobs.push_back(id);
	return id;
}
