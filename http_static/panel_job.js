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

		$('#job_summary').text(JSON.stringify(this.model.attributes));
	}
});
