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

Json::Value JobPool::getJobDescription(std::string query_id) {
	for(const std::string id : all_jobs) {
		if(id == query_id) {
			Json::Value job;
			job["id"] = id;
			job["status"] = "complete";
			job["memo"] = "Calculate L1 norm";
			job["source"] = "Random 10 scenes";
			return job;
		}
	}
	throw std::runtime_error("Job query did not match anything");
}
