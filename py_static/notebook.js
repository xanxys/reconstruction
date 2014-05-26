
var WorldView = Backbone.View.extend({
	el: '<canvas/>',

	initialize: function(options) {
		console.log('hello');
		this.$el.width = 640;
		this.$el.height = 480;

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

var Notebook = function() {
	this.view = new WorldView();
	$('#world_view').append(this.view.$el);
	this.view.setup();

	// TODO: refactor
	$('#send_snippet').click(function() {
		var code = $('#snippet').val();
		$.post('/snippet', {"code": code}).done(function(data) {
			$('#result').text(data);
		});
	});
};

var notebook = new Notebook();
