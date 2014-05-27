
var WorldView = Backbone.View.extend({
	el: '<canvas/>',

	initialize: function(options) {
		this.$el[0].width = 640;
		this.$el[0].height = 480;

		//this.setup();
	},

	// Call after element is inserted to DOM. Ohterwise, trackball
	// control doesn't work.
	setup: function() {
		this.camera = new THREE.PerspectiveCamera(75, 800 / 600, 0.001, 50);
		this.camera.up = new THREE.Vector3(0, -1, 0);
		this.camera.position = new THREE.Vector3(0, -1, -1);
		this.camera.lookAt(new THREE.Vector3(0, 0, 0));

		this.scene = new THREE.Scene();

		// start canvas
		this.renderer = new THREE.WebGLRenderer({
			canvas: this.el
		});
		this.renderer.setClearColor(this.for_figure ? '#222': '#444');

		// add mouse control (do this after canvas insertion)
		this.controls = new THREE.TrackballControls(this.camera, this.renderer.domElement);
		this.controls.maxDistance = 15;

		// start
		this.animate();

				this.scene.add(new THREE.Mesh(
			new THREE.BoxGeometry(1, 0.5, 0.3),
			new THREE.MeshBasicMaterial({
				color: 'red'
			})));
	},

	animate: function() {
		var _this = this;
		requestAnimationFrame(function(){_this.animate()});

		this.renderer.render(this.scene, this.camera);
		this.controls.update();
	},
});


var deserPointcloud = function(data) {
	var rgb_is_valid = (data.length > 0 && data[0].r !== undefined);

	var points_geom = new THREE.BufferGeometry();

	var position_attr = new THREE.Float32Attribute(data.length, 3);
	points_geom.addAttribute('position', position_attr);
	_.each(data, function(vertex, index) {
		position_attr.setXYZ(index, vertex.x, vertex.y, vertex.z);
	});

	if(rgb_is_valid) {
		var color_attr = new THREE.Float32Attribute(data.length, 3);
		points_geom.addAttribute('color', color_attr);
		_.each(data, function(vertex, index) {
			color_attr.setXYZ(index,
				vertex.r / 255, vertex.g / 255, vertex.b / 255);
		});
	}

	var material = new THREE.ParticleSystemMaterial(rgb_is_valid ? {
			size: 0.005,
			vertexColors: true
		} : {
			size: 0.005,
			color: '#aaf'
		});
	return new THREE.ParticleSystem(points_geom, material);
};

var deserMesh = function(data) {
	// TODO: use efficient version
	// var geom = new THREE.BufferGeometry()
	var geom = new THREE.Geometry();
	geom.faces = data.faces;
	geom.vertices = _.map(data.vertices, function(vertex) {
		return new THREE.Vector3(vertex.x, vertex.y, vertex.z);
	});

	return new THREE.Mesh(
		geom,
		new THREE.MeshBasicMaterial({
			color: 'green'
		}));
};


var SnippetView = Backbone.View.extend({
	events: {
		'click .send_snippet': "send"
	},

	initialize: function(options) {
		this.$el.html(_.template($('#template-snippet').text()));

		this.code_view = CodeMirror.fromTextArea(this.$('textarea')[0], {
			value: "1+2",
			mode: "python"
		});
		this.code_view.setSize(640, 80);
	},

	send: function() {
		this.$('.result').empty();
		this.trigger('beforeSend');

		var code = this.code_view.getValue();
		var _this = this;
		$.post('/snippet', {"code": code}).done(function(data) {
			var result_str = $('<pre/>').text(data['str']);
			if(data.type === 'exception') {
				result_str.addClass('bg-warning');
			}
			_this.$('.result').append(result_str);

			if(data.type === 'success') {
				if(data.json.type === 'pointcloud') {
					var obj = deserPointcloud(data.json.data);
					_this.trigger('shareObject', obj);
				} else if(data.json.type === 'mesh') {
					var obj = deserMesh(data.json.data);
					_this.trigger('shareObject', obj);
				}
			}
		});
	}
});


var Notebook = function() {
	this.view = new WorldView();
	$('#world_view').append(this.view.$el);
	this.view.setup();

	this.snippet_views = [];
	this.addNewEmptySnippet();
};

Notebook.prototype.addNewEmptySnippet = function() {
	var snippet_view = new SnippetView();
	$('#snippets').append(snippet_view.$el);
	this.snippet_views.push(snippet_view);

	var _this = this;
	snippet_view.on('beforeSend', function() {
		if(_.last(_this.snippet_views) === snippet_view) {
			_this.addNewEmptySnippet();
		}
	});

	snippet_view.on('shareObject', function(obj) {
		_this.view.scene.add(obj);
	});
};

var notebook = new Notebook();
