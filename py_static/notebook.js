
var WorldView = Backbone.View.extend({
	el: '<canvas/>',

	initialize: function(options) {
		console.log('hello');
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
	// Deserialize XYZ & RGB components.
	var points_geom = new THREE.Geometry();
	var vs = _.map(data, function(v) {
		return new THREE.Vector3(v.x, v.y, v.z);
	});
	points_geom.vertices = vs;

	var rgb_is_valid = false;
	if(data.length > 0 && data[0].r !== undefined) {
		rgb = _.map(data, function(v) {
			return new THREE.Color().setRGB(v.r / 255, v.g / 255, v.b / 255);
		});
		points_geom.colors = rgb;
		rgb_is_valid = true;
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

var Notebook = function() {
	this.view = new WorldView();
	$('#world_view').append(this.view.$el);
	this.view.setup();

	// TODO: refactor
	var code_view = CodeMirror.fromTextArea($('#snippet')[0], {
		value: "1+2",
		mode: "python"
	});
	code_view.setSize(640, 80);

	var _this = this;
	$('#send_snippet').click(function() {
		var code = code_view.getValue();
		$.post('/snippet', {"code": code}).done(function(data) {
			$('#result').empty();
			$('#result').append($('<div/>').text(data['type']));
			$('#result').append($('<div/>').text(data['str']));

			if(data.type === 'success' && data.json.type === 'pointcloud') {
				var obj = deserPointcloud(data.json.data);
				_this.view.scene.add(obj);
			}
		});
	});
};

var notebook = new Notebook();
