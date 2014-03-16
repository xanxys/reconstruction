
var DebugFE = function() {
};

DebugFE.prototype.run = function() {
	// When this mode is enabled, try produce high-contrast, big-text, less-clutter imagery.
	var for_figure = true;

	// three.js
	this.camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.01, 500);
	this.camera.up = new THREE.Vector3(0, 0, 1);
	this.camera.position = new THREE.Vector3(0, 0.1, 0.1);
	this.camera.lookAt(new THREE.Vector3(0, 0, 0));

	this.scene = new THREE.Scene();

	// Add table
	var tex_table = THREE.ImageUtils.loadTexture('/table/background', null);
	var table = new THREE.Mesh(
		new THREE.CubeGeometry(100e-3, 100e-3, 5e-3),
		new THREE.MeshBasicMaterial({transparent: true, opacity: 0.8, map: tex_table}));
	table.position.z = -2.5e-3;
	this.scene.add(table);

	// Add depth camera mock
	var mock_depth = new THREE.Mesh(
		new THREE.CubeGeometry(0.04, 0.04, 0.03),
		new THREE.MeshBasicMaterial({color: '#dfd', transparent: true, opacity: 0.5}));
	var fov_h_pmd = 90 / 180 * Math.PI;
	var fov_v_pmd = 68 / 180 * Math.PI;

	mock_depth.add(this.generateCameraCone(fov_h_pmd, fov_v_pmd));	
	mock_depth.position = new THREE.Vector3(0, 0.19, 0.04);
	mock_depth.quaternion.setFromAxisAngle(new THREE.Vector3(1, 0 ,0), Math.PI / 2);
	this.scene.add(mock_depth);

	// Add RGB camera mock
	var mock_rgb = null;

	// Add marker
	var mock_marker = new THREE.Mesh(
		new THREE.CubeGeometry(0.03, 0.03, 1e-3),
		new THREE.MeshBasicMaterial({color: "red", transparent: true, opacity: 0.2}));
	this.scene.add(mock_marker);

	// start canvas
	this.renderer = new THREE.WebGLRenderer();
	this.renderer.setClearColor(for_figure ? '#222': '#444');
	this.renderer.setSize(window.innerWidth, window.innerHeight);
	document.body.appendChild(this.renderer.domElement);

	// add mouse control (do this after canvas insertion)
	this.controls = new THREE.TrackballControls(this.camera, this.renderer.domElement);
	this.controls.maxDistance = 2;

	var _this = this;

	var points = null;
	var updatePoints = function() {
		$.ajax('/camera_distance/points').done(function(data) {
			var geom = new THREE.Geometry();
			geom.vertices = _.map(data, function(p) {
				return new THREE.Vector3(p.x, p.y, p.z);
			});

			var new_points = new THREE.ParticleSystem(geom,
				new THREE.ParticleBasicMaterial({color: 'red', size: for_figure ? 1.5e-3 : 0.5e-3}));

			if(points !== null) {
				mock_depth.remove(points);
			}
			points = new_points;
			mock_depth.add(points);

			setTimeout(updatePoints, 100);
		});
	};

	updatePoints();

	var endpoint_rgb = "/camera_rgb/frame";
	var update_calib = function() {
		$.ajax('/camera_rgb/config').done(function(data) {
			if(mock_rgb === null) {
				mock_rgb = new THREE.Mesh(
					new THREE.CubeGeometry(0.07, 0.03, 0.02),
					new THREE.MeshBasicMaterial({color: '#333', transparent: true, opacity: 0.5}));
				var fov_h_rgb = data['fov_h'];
				var fov_v_rgb = data['fov_v'];

				mock_rgb.add(_this.generateCameraCone(fov_h_rgb, fov_v_rgb));
				_this.scene.add(mock_rgb);

				var vcanvas = null;
				var updateTex = function() {
					THREE.ImageUtils.loadTexture(endpoint_rgb, null, function(tex) {
						var virt_canvas = new THREE.Mesh(
							new THREE.CubeGeometry(
								2 * 0.2 * Math.tan(fov_h_rgb / 2),
								2 * 0.2 * Math.tan(fov_v_rgb / 2),
								0.001),
							new THREE.MeshBasicMaterial({map: tex}));
						virt_canvas.quaternion.setFromAxisAngle(new THREE.Vector3(0, 0, 1), Math.PI);
						virt_canvas.position.z = 0.2;

						if(vcanvas !== null) {
							mock_rgb.remove(vcanvas);
						}
						mock_rgb.add(virt_canvas);
						vcanvas = virt_canvas;

						setTimeout(updateTex, 100);
					});
				};

				updateTex();
			}
			

			var rot =
				new THREE.Matrix4(
					data['rot'][0][0], data['rot'][0][1], data['rot'][0][2], 0,
					data['rot'][1][0], data['rot'][1][1], data['rot'][1][2], 0,
					data['rot'][2][0], data['rot'][2][1], data['rot'][2][2], 0,
					0, 0, 0, 0);

			var trans =
				new THREE.Vector3(
					data['trans'][0], data['trans'][1], data['trans'][2]);

			mock_rgb.quaternion.setFromRotationMatrix(rot);
			mock_rgb.position = trans;

			setTimeout(update_calib, 5000);
		});
	};
	update_calib();

	var mock_cuboid = null;
	var update_cuboid = function() {
		$.ajax('/cuboid').done(function(data) {
			if(mock_cuboid === null) {
				mock_cuboid = new THREE.Mesh(
					new THREE.CubeGeometry(data.size[0], data.size[1], data.size[2]),
					new THREE.MeshBasicMaterial({color: '#888'}));
				_this.scene.add(mock_cuboid);
			}
		
			var rot =
				new THREE.Matrix4(
					data['rot'][0][0], data['rot'][0][1], data['rot'][0][2], 0,
					data['rot'][1][0], data['rot'][1][1], data['rot'][1][2], 0,
					data['rot'][2][0], data['rot'][2][1], data['rot'][2][2], 0,
					0, 0, 0, 0);

			var trans =
				new THREE.Vector3(
					data['trans'][0], data['trans'][1], data['trans'][2]);

			mock_cuboid.quaternion.setFromRotationMatrix(rot);
			mock_cuboid.position = trans;

			setTimeout(update_cuboid, 250);
		});
	};
	update_cuboid();

	// Connect signals.
	$('#snapshot').click(function() {
		$.post('/snapshot');
	});

	$('input[name="table_layer"]:radio').change(function(ev) {
		var url = "/table/" + ev.currentTarget.value;
		table.material.map = THREE.ImageUtils.loadTexture(url, null);
		table.material.needsUpdate = true;
	});

	$('input[name="rgb_layer"]:radio').change(function(ev) {
		endpoint_rgb = "/camera_rgb/" + ev.currentTarget.value;
	});

	$('input[name="show_rgb_camera"]:checkbox').change(function(ev) {
		var val = ev.currentTarget.checked;
		_this.setVisible(mock_rgb, val);
	});

	$('input[name="show_depth_camera"]:checkbox').change(function(ev) {
		var val = ev.currentTarget.checked;
		_this.setVisible(mock_depth, val);
	});

	$('input[name="show_reconstruction"]:checkbox').change(function(ev) {
		var val = ev.currentTarget.checked;
		_this.setVisible(mock_marker, val);
		_this.setVisible(mock_cuboid, val);
		_this.setVisible(table, val);
	});

	// start
	this.animate();
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
