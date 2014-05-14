"use strict";

var SceneSummary = Backbone.Model.extend({
});

var SceneSummaryList = Backbone.Collection.extend({
	url: '/scenes',
	model: SceneSummary,
});

var SceneSummaryListView = Backbone.View.extend({
	el: '#scene_list',

	events: {
		"click #ui_update": "takeSnapshot",
		"click .ui_random": "selectRandom"
	},

	initialize: function(options) {
		this.current_id = null;

		this.listenTo(this.model, 'sync', this.render);
	},

	takeSnapshot: function() {
		var _this = this;
		$.post('/at').done(function(data) {
			_this.current_id = data.id;
			window.location = '/#/scene/' + data.id;
			_this.model.fetch();
		});
	},

	selectRandom: function() {
		var model_index = Math.floor(Math.random() * this.model.length);
		this.current_id = this.model.at(model_index).id;
		this.render();
		window.location = '/#/scene/' + this.current_id;
	},

	render: function() {
		var _this = this;
		this.$('#ui_scenes').empty();

		var list = this.$('#ui_scenes');
		this.model.each(function(model) {
			var entry = $('<a/>')
				.text(model.id).attr('href', '#/scene/' + model.id).addClass('list-group-item');
			if(model.id === _this.current_id) {
				entry.addClass('active');
			}

			entry.click(function() {
				_this.current_id = model.id;
				_this.render();
			});

			list.append(entry);
		});

		this.$('.scene_status').text(this.model.length + ' scenes');
	}
});


var JobList = Backbone.Collection.extend({
	url: '/job',
	model: Job,
});

var JobListView = Backbone.View.extend({
	el: '#jobs_summary',

	events: {
		'click #add_job': 'addJob',
	},

	initialize: function(options) {
		this.current_id = null;
		this.listenTo(this.model, 'sync', this.render);
	},

	addJob: function() {
		var _this = this;
		$.post('/job', 'description').done(function(data) {
			console.log(data);
			_this.model.fetch();
		});
	},

	render: function() {
		var _this = this;
		this.$('#ui_jobs').empty();

		var list = this.$('#ui_jobs');
		this.model.each(function(model) {
			var entry = $('<a/>')
				.text(model.id).attr('href', '#/job/' + model.id).addClass('list-group-item');

			entry.click(function() {
				_this.current_id = model.id;
				_this.render();
			});

			list.append(entry);
		});
	}
});

var ReconRouter = Backbone.Router.extend({
	routes: {
		'dashboard': 'dashboard',
		'job/:id': 'job',
		'scene/:id': 'scene',
	},

	dashboard: function() {
		console.log('->dashboard');
		$('#panel').html(_.template($('script[name=dashboard_panel]').html()));
	},

	job: function(id) {
		console.log('->job', id);
		var job = new Job({id: id});
		job.fetch();
		new JobPanel({model: job}).render();
	},

	scene: function(id) {
		console.log('->scene', id);
		new ScenePanel({id: id}).render();
	},
});

var ReconApp = function() {
	this.router = new ReconRouter();
	this.router.dashboard();
	Backbone.history.start();


	this.job_list = new JobList();
	this.job_list_view = new JobListView({
		model: this.job_list,
	});
	this.job_list.fetch();

	this.scene_summary_list = new SceneSummaryList();
	this.scene_summary_list_view = new SceneSummaryListView({
		model: this.scene_summary_list,
	});
	this.scene_summary_list.fetch();
};

window.app = new ReconApp();
