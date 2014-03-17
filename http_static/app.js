
var DebugFE = function() {
	// When this mode is enabled, try producing high-contrast, big-text, less-clutter imagery.
	this.for_figure = false;

	var _this = this;
	$('#ui_update').click(function() {
		$.ajax('/points').done(function(data) {
			var geom = new THREE.Geometry();
			geom.vertices = _.map(data, function(p) {
				return new THREE.Vector3(p.x, p.y, p.z);
			});
			geom.colors = _.map(data, function(p) {
				return new THREE.Color(p.r, p.g, p.b);
			});
			var cloud = new THREE.ParticleSystem(geom,
				new THREE.ParticleSystemMaterial({
					vertexColors: true,
					size: 0.005
				}));

			if(_this.cloud !== undefined) {
				_this.scene.remove(_this.cloud);
			}
			_this.cloud = cloud;
			_this.scene.add(_this.cloud);
		});
	});
};

DebugFE.prototype.run = function() {
	// three.js
	this.camera = new THREE.PerspectiveCamera(75, 800 / 600, 0.001, 50);
	this.camera.up = new THREE.Vector3(0, 0, 1);
	this.camera.position = new THREE.Vector3(0, 3, 3);
	this.camera.lookAt(new THREE.Vector3(0, 0, 0));

	this.scene = new THREE.Scene();

	this.scene.add(this.generateVoxelGrid());

	// start canvas
	this.renderer = new THREE.WebGLRenderer();
	this.renderer.setClearColor(this.for_figure ? '#222': '#444');
	this.renderer.setSize(800, 600); //window.innerWidth, window.innerHeight);
	document.body.appendChild(this.renderer.domElement);

	// add mouse control (do this after canvas insertion)
	this.controls = new THREE.TrackballControls(this.camera, this.renderer.domElement);
	this.controls.maxDistance = 15;

	var _this = this;

	// start
	this.animate();
};

// return :: Object3D
DebugFE.prototype.generateVoxelGrid = function() {
	var voxel_size = 0.1;
	var voxel_n = 30;

	var voxel_geom = new THREE.Geometry();
	var e0 = new THREE.Vector3(1, 0, 0);
	var e1 = new THREE.Vector3(0, 1, 0);
	var e2 = new THREE.Vector3(0, 0, 1);
	var origin = new THREE.Vector3(
		-voxel_size * voxel_n / 2,
		-voxel_size * voxel_n / 2,
		1);

	var addBundles = function(e0, e1, e2) {
		_.each(_.range(voxel_n), function(i0) {
			_.each(_.range(voxel_n), function(i1) {
				var p0 = origin.clone().add(
					e0.clone().multiplyScalar(i0 * voxel_size)).add(
					e1.clone().multiplyScalar(i1 * voxel_size));
				var p1 = p0.clone().add(e2.clone().multiplyScalar(voxel_n * voxel_size));

				voxel_geom.vertices.push(p0);
				voxel_geom.vertices.push(p1);
			});
		});
	};

	addBundles(e0, e1, e2);
	addBundles(e1, e2, e0);
	addBundles(e2, e0, e1);
	
	return new THREE.Line(
		voxel_geom,
		new THREE.LineBasicMaterial({
			color: 'white',
			opacity: 0.3,
			transparent: true
		}),
		THREE.LinePieces);
};

// obj :: THREE.Object3D
// status :: bool
DebugFE.prototype.setVisible = function(obj, status) {
	obj.visible = status;
	_.each(obj.children, function(child) {
		this.setVisible(child, status);
	}, this);
};

// return :: THREE.Object3D
DebugFE.prototype.generateCameraCone = function(fov_h, fov_v) {
	var cone = new THREE.Object3D();
	_.each([0.025, 0.05, 0.075], function(dist) {
		var geom = new THREE.Geometry();
		geom.vertices.push(new THREE.Vector3(-Math.tan(fov_h / 2) * dist, -Math.tan(fov_v / 2) * dist, 0));
		geom.vertices.push(new THREE.Vector3( Math.tan(fov_h / 2) * dist, -Math.tan(fov_v / 2) * dist, 0));
		geom.vertices.push(new THREE.Vector3( Math.tan(fov_h / 2) * dist,  Math.tan(fov_v / 2) * dist, 0));
		geom.vertices.push(new THREE.Vector3(-Math.tan(fov_h / 2) * dist,  Math.tan(fov_v / 2) * dist, 0));
		geom.faces.push(new THREE.Face3(0, 1, 2));
		geom.faces.push(new THREE.Face3(0, 2, 3));

		var frame = new THREE.Mesh(geom, new THREE.MeshBasicMaterial({wireframe: true, color: '#ddd'}));
		frame.position.z = dist;
		cone.add(frame);
	});
	return cone;
};

DebugFE.prototype.animate = function() {
	var _this = this;
	requestAnimationFrame(function(){_this.animate()});

	this.renderer.render(this.scene, this.camera);
	this.controls.update();
};

new DebugFE().run();
