"use strict";

var Job = Backbone.Model.extend({
	urlRoot: '/job'
});

var JobPanel = Backbone.View.extend({
	initialize: function(options) {
		this.listenTo(this.model, 'change', this.render);
	},

	render: function() {
		$('#panel').html(_.template($('script[name=job_panel]').html()));

		$('#job_summary').empty();
		$('#job_summary').append($('<h1/>').text('Job ' + this.model.id));
		$('#job_summary').append($('<h2/>').text('Raw Result'));
		$('#job_summary').append($('<pre/>')
			.text(JSON.stringify(this.model.attributes, undefined, '  '))
			.css('height', '400px')
			.css('overflow', 'scroll'));
	}
});
