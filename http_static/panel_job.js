"use strict";


var JobPanel = Backbone.View.extend({
	initialize: function(options) {
		this.id = options.id;
	},

	render: function() {
		$('#panel').html(_.template($('script[name=job_panel]').html()));
	}
});
