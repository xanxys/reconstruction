
var Viewer = function() {
	this.init3D();

	this.floor_grid = this.createFloorGrid();
	this.scene.add(this.floor_grid);

	this.v_cameras = new THREE.Object3D();
	this.scene.add(this.v_cameras);

	this.data = null;
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


Viewer.prototype.update = function() {
	var _this = this;
	if(_this.data === null) {
		return;
	}
	var data = _this.data;

	_this.scene.remove(_this.floor_grid);
	_this.floor_grid = _this.createFloorGrid(0);
	_this.scene.add(_this.floor_grid);

	_.each(_this.scans, function(scan_view) {
		_this.scene.remove(scan_view);
	});
	_this.scans = _.map(data.scans, function(scan) {
		var scan_view = new THREE.Object3D();
		var points_geom = new THREE.Geometry();
		points_geom.vertices = _.map(scan.cloud, function(v) {
			return new THREE.Vector3(v.x, v.y, v.z);
		});
		points_geom.colors = _.map(scan.cloud, function(v) {
			return new THREE.Color(v.r / 255, v.g / 255, v.b / 255);
		});
		var material = new THREE.PointCloudMaterial({
			size: 0.01,
			vertexColors: true
		});

		scan_view.add(new THREE.PointCloud(points_geom, material));
		return scan_view;
	});
	_.each(_this.scans, function(scan_view) {
		_this.scene.add(scan_view);
	});
};

Viewer.prototype.update_data = function(data) {
	var _this = this;

	_this.data = data;
	// Initialize pose data.
	_.each(_this.data.scans, function(scan) {
		scan.pose = {
			"trans_x": 0,
			"trans_y": 0,
			"trans_z": 0,
			"rot_z": 0
		};
	});

	// Create UI based on scans.
	$('#scan_list').empty();
	_.each(data.scans, function(scan, ix) {
		var li = $('<li/>').text(scan.name);
		li.click(function() {
			_this.selected_scan_ix = ix;
			// Highlight selected item in the list.
			$('#scan_list > li').css('background-color', 'white');
			li.css('background-color', 'skyblue');
			// Apply current values.
			set_pose_sliders(_this.data.scans[ix].pose);
		});
		$('#scan_list').append(li);
	});
	_this.update();
}

Viewer.prototype.update_curr_pose = function(pose) {
	var _this = this;
	if(_this.selected_scan_ix === undefined) {
		return;
	}

	_this.data.scans[_this.selected_scan_ix].pose = pose;
	_this.scans[_this.selected_scan_ix].position.set(
		pose.trans_x, pose.trans_y, pose.trans_z);
	_this.scans[_this.selected_scan_ix].quaternion
		.setFromAxisAngle(
			new THREE.Vector3(0, 0, 1),
			pose.rot_z / 180 * Math.PI);
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

$.ajax('/list').done(function(data) {
	console.log(data.status);
	_.each(data.data, function(name) {
		var button = $('<button/>').text("Load " + name);
		button.click(function() {
			load_data(name);
		});
		$('#data_list').append(button);
	})
});

function load_data(name) {
	console.log(name);
	$.ajax('/data/' + name).done(function(data) {
		console.log("loaded", data.status);
		viewer.update_data(data.data);
	});
}


// Pose slider UI
var slider_ids = ["s_tx", "s_ty", "s_tz", "s_rz"];
_.each(slider_ids, function(slider_id) {
	$('#' + slider_id).mousemove(function() {
		viewer.update_curr_pose(get_pose_sliders());
	});
});

function get_pose_sliders() {
	return {
		"trans_x": parseFloat($('#s_tx').val()),
		"trans_y": parseFloat($('#s_ty').val()),
		"trans_z": parseFloat($('#s_tz').val()),
		"rot_z": parseFloat($('#s_rz').val())
	};
}

function set_pose_sliders(pose) {
	$('#s_tx').val(pose.trans_x);
	$('#s_ty').val(pose.trans_y);
	$('#s_tz').val(pose.trans_z);
	$('#s_rz').val(pose.rot_z);
}
