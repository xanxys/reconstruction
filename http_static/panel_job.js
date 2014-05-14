"use strict";

var Job = Backbone.Model.extend({
	urlRoot: '/job'
});

var JobPanel = Backbone.View.extend({
	initialize: function(options) {
		this.listenTo(this.model, 'change', this.render);

		var _this = this;
		var interval = setInterval(function() {
			_this.model.fetch();
			if(_this.model.get('progress') >= 1) {
				clearInterval(interval);
			}
		}, 250);
	},

	render: function() {
		$('#panel').html(_.template($('script[name=job_panel]').html()));

		// Construct title
		$('#job_summary h1').text('Job ' + this.model.id);

		if(this.model.get('progress') >= 1) {
			$('#job_summary .exec_status')
				.text('finished')
				.removeClass('label-warning')
				.addClass('label-success');
			$('#job_summary .progress').hide();
		} else {
			$('#job_summary .exec_status')
				.text('running')
				.removeClass('label-success')
				.addClass('label-warning');

			$('#job_summary .progress-bar')
				.css('width', (100 * this.model.get('progress')) + '%')
				.attr('aria-valuenow', 100 * this.model.get('progress'));
			$('#job_summary .progress').show();
		}
		
		// Construct raw data
		$('#job_summary pre')
			.text(JSON.stringify(this.model.attributes, undefined, '  '))
			.css('height', '400px')
			.css('overflow', 'scroll');
	}
});
