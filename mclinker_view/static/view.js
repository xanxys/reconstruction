
var Viewer = function() {
	this.init3D();

	this.floor_grid = this.createFloorGrid();
	this.scene.add(this.floor_grid);

	this.v_cameras = new THREE.Object3D();
	this.scene.add(this.v_cameras);
};


Viewer.prototype.init3D = function() {
	this.scene = new THREE.Scene();
	var width = 1000;
	var height = 600;

	// camera & control
	this.camera = new THREE.PerspectiveCamera(75, width / height, 0.01, 50);
	this.scene.add(this.camera);
	this.camera.up.set(0, 0, 1);
	this.camera.position.set(0.5, 0.5, 0.5);
	this.camera.lookAt(new THREE.Vector3(0, 0, 0));

	// start canvas
	this.renderer = new THREE.WebGLRenderer();
	this.renderer.setClearColor('#444');
	this.renderer.setSize(width, height);
	$('body').append(this.renderer.domElement);

	this.controls = new THREE.TrackballControls(this.camera, this.renderer.domElement);
	this.controls.maxDistance = 10;
};


Viewer.prototype.createFloorGrid = function(z) {
	var floor_grid = new THREE.Object3D();

	_.each(_.range(-5, 6), function(ix) {
		_.each(_.range(-5, 6), function(iy) {
			var tile = new THREE.Mesh(
				new THREE.BoxGeometry(0.95, 0.95, 0.01),
				new THREE.MeshBasicMaterial({color: '#888'}));
			tile.position.x = ix;
			tile.position.y = iy;
			tile.position.z = z;
			floor_grid.add(tile);
		}, this);
	}, this);

	return floor_grid;
};


// Return camera such that
// * X+<Camera>: right
// * Y+<Camera>: down
// * Z+<Camera>: forward
//
// return :: Object3D
Viewer.prototype.generateCameraMock = function(size) {
	if(size === undefined) {
		size = 0.2;
	}
	var fov_x = 60 / 180 * Math.PI;
	var fov_y = 40 / 180 * Math.PI;

	// Create "real" vertices
	var verts = [
		new THREE.Vector3(+Math.tan(fov_x / 2), +Math.tan(fov_y / 2), 1),
		new THREE.Vector3(-Math.tan(fov_x / 2), +Math.tan(fov_y / 2), 1),
		new THREE.Vector3(-Math.tan(fov_x / 2), -Math.tan(fov_y / 2), 1),
		new THREE.Vector3(+Math.tan(fov_x / 2), -Math.tan(fov_y / 2), 1),
	];
	var origin = new THREE.Vector3(0, 0, 0);
	_.each(verts, function(vert) {
		vert.multiplyScalar(size);
	});

	// Create lines.
	var geom = new THREE.Geometry();
	_.each(verts, function(vert) {
		geom.vertices.push(origin);
		geom.vertices.push(vert);
	});
	_.each(verts, function(vert, ix) {
		var vert_next = verts[(ix + 1) % verts.length];
		geom.vertices.push(vert);
		geom.vertices.push(vert_next);
	});

	return new THREE.Line(
		geom,
		new THREE.LineBasicMaterial({
			"color": 'red',
		}),
		THREE.LinePieces);
};



Viewer.prototype.update = function(data) {
	var _this = this;

	_this.scene.remove(_this.floor_grid);
	_this.floor_grid = _this.createFloorGrid(data.rframe.z0);
	_this.scene.add(_this.floor_grid);

	_this.scene.remove(_this.clusters);
	_this.clusters = new THREE.Object3D();
	_.each(data.clusters, function(cluster) {
		var points_geom = new THREE.Geometry();
		points_geom.vertices = _.map(cluster.cloud, function(v) {
			return new THREE.Vector3(v.x, v.y, v.z);
		});
		points_geom.colors = _.map(cluster.cloud, function(v) {
			return new THREE.Color().setRGB(v.r / 255, v.g / 255, v.b / 255);
		});

		var material = new THREE.PointCloudMaterial({
			size: 0.005,
			vertexColors: true
		});
		_this.clusters.add(
			new THREE.PointCloud(points_geom, material));
	});
	_this.scene.add(_this.clusters);

	var mcid_to_pos = function(mcid, pos_hint) {
		if(mcid === -1) {
			return new THREE.Vector3(pos_hint.x, pos_hint.y, data.rframe.z0);
		} else {
			console.log(mcid);
			var pt = data.clusters[mcid].cloud[0];
			return new THREE.Vector3(pt.x, pt.y, pt.z);
		}
	};

	console.log(data.edges);
	_this.scene.remove(_this.edges);
	_this.edges = new THREE.Object3D();
	_.each(data.edges, function(edge) {
		var v_parent = edge[1];
		var v_child = edge[0];

		var to = mcid_to_pos(v_child);
		var from = mcid_to_pos(v_parent, to);

		var arrow = new THREE.ArrowHelper(
			to.clone().sub(from).normalize(), from,
			to.clone().sub(from).length(), 0xffff00,
			0.05, 0.02);
		_this.edges.add(arrow);

	});
	_this.scene.add(_this.edges);
};


Viewer.prototype.animate = function() {
	var _this = this;
	requestAnimationFrame(function() {
		_this.animate();
	});

	this.renderer.render(this.scene, this.camera);
	this.controls.update();
};


var viewer = new Viewer();
viewer.animate();

$('#btn_reload').click(function() {
	$.ajax('/data').done(function(data) {
		console.log("loaded", data.status);
		console.log(data.data.clusters.length);

		viewer.update(data.data);
	});
});
