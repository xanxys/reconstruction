#include "job_pool.h"

#include "analyzer/analyzer.h"

EstimateL1Job::EstimateL1Job(EstimateL1Job&& that) :
	id(that.id),
	data_source(that.data_source),
	worker(std::move(that.worker)),
	result(that.result) {
}

EstimateL1Job::EstimateL1Job(std::string id, DataSource& data_source, std::vector<std::string> scene_ids) :
	id(id), data_source(data_source),
	worker(&EstimateL1Job::estimateL1, this, scene_ids) {
}

float EstimateL1Job::getProgress() {
	if(result.isNull()) {
		return 0.1;
	} else {
		return 1;
	}
}

boost::optional<Json::Value> EstimateL1Job::getResult() {
	if(result.isNull()) {
		return boost::optional<Json::Value>();
	} else {
		return boost::optional<Json::Value>(result);
	}
}


// Calculate L1 norm for all scene_ids and aggreate result as array.
void EstimateL1Job::estimateL1(std::vector<std::string> scene_ids) {
	// TODO: use multiple cores or remote nodes.
	Json::Value result;
	for(const auto scene_id : scene_ids) {
		result.append(mapEstimateL1(scene_id));
	}
	this->result = result;
}

// assuming shared, high-bandwidth DataSource, scene_id is enough.
Json::Value EstimateL1Job::mapEstimateL1(std::string scene_id) {
	const auto& cloud = data_source.getScene(scene_id);
	SceneAnalyzer analyzer(cloud);
	
	Json::Value result;
	result["id"] = scene_id;
	result["L1"] = analyzer.getScore(*analyzer.getBestBelief());
	return result;
}


JobPool::JobPool(DataSource& data_source) : data_source(data_source), job_id(0) {
}

std::vector<std::string> JobPool::listJobs() {
	std::vector<std::string> ids;
	for(const auto& job : all_jobs) {
		ids.push_back(job.id);
	}
	return ids;
}

std::string JobPool::createJob() {
	const std::string id = std::to_string(job_id++);

	// Create input.
	const auto all_ids = data_source.listScenes();
	std::vector<std::string> input(all_ids.begin(), all_ids.begin() + 20);

	// Create job.
	all_jobs.emplace_back(id, data_source, input);

	return id;
}

Json::Value JobPool::getJobDescription(std::string query_id) {
	for(auto& job : all_jobs) {
		if(job.id == query_id) {
			Json::Value job_desc;
			job_desc["id"] = job.id;
			job_desc["progress"] = job.getProgress();
			job_desc["memo"] = "Calculate L1 norm";
			job_desc["source"] = "Random 10 scenes";

			const auto result = job.getResult();
			if(result) {
				job_desc["result"] = *result;
			}
			return job_desc;
		}
	}
	throw std::runtime_error("Job query did not match anything");
}
