"use strict";

var Scene = Backbone.Model.extend({
	urlRoot: '/at'
});


var DebugFE = function() {
	// When this mode is enabled, try producing high-contrast, big-text, less-clutter imagery.
	this.for_figure = false;
	this.layers = {};
	this.layer_descs = [
		{label:'Voxels', endpoint:'voxels', generator:function(d){return _this.showVoxels(d);}},
		{label:'EmptyVoxels', endpoint:'voxels_empty', generator:function(d){return _this.showVoxels(d);}},
		{label:'Objects', endpoint:'objects', generator:function(d){return _this.showObjects(d);}},
		{label:'Points', endpoint:'points', generator:function(d){return _this.showPoints(d);}},
		{label:'Planes', endpoint:'planes', generator:function(d){return _this.showPlanes(d);}},
	];

	var _this = this;
	$('#ui_update').click(function() {
		$.post('/at').done(function(data) {
			_this.current_id = data.id;
			_this.updateViews();
			_this.updateSceneList();
		});
	});



	var ctx = $('#ui_grabcut_drawing')[0].getContext('2d');
	ctx.fillStyle = 'blue';
	var drawing = false;
	$('#ui_grabcut_drawing').mousedown(function(event) {
		drawing = true;
	});
	$('#ui_grabcut_drawing').mousemove(function(event) {
		if(!drawing) {
			return;
		}

		ctx.beginPath();
		ctx.arc(event.offsetX, event.offsetY, 5, 0, 2 * Math.PI);
		ctx.fill();
	});
	$('#ui_grabcut_drawing').mouseup(function(event) {
		drawing = false;
	});

	$('#ui_draw_fg').click(function() {
		ctx.fillStyle = 'blue';
	});

	$('#ui_draw_bg').click(function() {
		ctx.fillStyle = 'red';
	});

	$('#ui_clear').click(function() {
		ctx.clearRect(0, 0, 640, 480);
	});

	$('#ui_do_grabcut').click(function() {
		$.ajax({
			type: 'POST',
			url: '/at/' + _this.current_id + '/grabcut',
			data: JSON.stringify({
				image: $('#ui_grabcut_drawing')[0].toDataURL()
			}),
			contentType: 'application/json'
		}).done(function(data) {
			$('#ui_modal_result').empty();
			var img = new Image();
			img.src = data['image'];
			$('#ui_modal_result').append(img);
			$('#myModal').modal();
		});
	});


	// Setup layer selector
	_.each(_this.layer_descs, function(layer_desc) {
		var layer = $('<a/>').attr('src', '#')
			.addClass('list-group-item').addClass('active')
			.text(layer_desc.label);

		layer.click(function() {
			layer.toggleClass('active');
			
			if(layer.hasClass('active')) {
				_this.scene.add(_this.layers[layer_desc.endpoint]);
			} else {
				_this.scene.remove(_this.layers[layer_desc.endpoint]);
			}
		});

		$('#ui_layers').append(layer);
	});

	_this.updateSceneList();
};

DebugFE.prototype.updateSceneList = function() {
	var _this = this;
	$('#ui_scenes').empty();
	$.ajax('/scenes').done(function(scenes) {
		_.each(scenes, function(scene) {
			var entry = $('<a/>')
			.text(scene.id).attr('href', '#').addClass('list-group-item');

			if(scene.id === _this.current_id) {
				entry.addClass('active');
			}

			entry.click(function() {
				_this.current_id = scene.id;
				_this.updateSceneList();
				_this.updateViews();
			});
			$('#ui_scenes').append(entry);	
		});
	});
};

DebugFE.prototype.updateViews = function() {
	var _this = this;

	// console.log(Model({id: this.current_id}).fetch());

	$.ajax('/at/' + _this.current_id).done(function(data_all) {
		_.each(_this.layer_descs, function(layer_desc) {
			var data = data_all[layer_desc.endpoint];
			var object = layer_desc.generator(data);

			if(_this.layers[layer_desc.endpoint] !== undefined) {
				_this.scene.remove(_this.layers[layer_desc.endpoint]);
			}
			_this.layers[layer_desc.endpoint] = object;
			if($('#ui_layers a:contains(' + layer_desc.label + ')').hasClass('active')) {
				_this.scene.add(object);
			}
		});


		var img = new Image();
		img.onload = function() {
			var ctx = $('#ui_grabcut')[0].getContext('2d');
			ctx.drawImage(img, 0, 0);
		};
		img.src = data_all['rgb'];

		$('#peeling').empty();
		$('#peeling').append($('<img/>').attr('src', data_all['peeling']['target']));
		$('#peeling').append($('<img/>').attr('src', data_all['peeling']['render']));
	});

};

DebugFE.prototype.showPoints = function(data) {
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
	return cloud;
};

DebugFE.prototype.showObjects = function(data) {
	var objects = new THREE.Object3D();

	var voxel_size = 0.1;

	var num_invalid = 0;
	_.each(data, function(object_desc) {
		if(!object_desc.valid) {
			num_invalid += 1;
			return;
		}

		var obj = new THREE.Mesh(
			new THREE.CubeGeometry(
				object_desc.sx,
				object_desc.sy,
				object_desc.sz),
			new THREE.MeshBasicMaterial({
				color: object_desc.valid ?
				new THREE.Color(
					object_desc.r / 255,
					object_desc.g / 255,
					object_desc.b / 255) :
				'red',
				opacity: 0.3,
				transparent: true
						//wireframe: true
					}));
		obj.position = new THREE.Vector3(
			object_desc.px,
			object_desc.py,
			object_desc.pz);

		obj.quaternion.setFromAxisAngle(new THREE.Vector3(0, 1, 0), object_desc.ry);
		objects.add(obj);
	});
	console.log('Invalid Object Proxies ', num_invalid, '/', data.length);
	return objects;
};

DebugFE.prototype.showVoxels = function(data) {
	var voxel_size = 0.1;

	var iy_floor = _.max(_.map(data, function(vx) {
		return vx.y;
	}));

	var voxels = new THREE.Object3D();
	_.each(data, function(vx) {
		var vx_three = new THREE.Mesh(
			new THREE.CubeGeometry(voxel_size, voxel_size, voxel_size),
			new THREE.MeshBasicMaterial({
				color: 'orange',
				opacity: 0.3,
				transparent: true
			}));
		vx_three.position = new THREE.Vector3(vx.x + 0.5, vx.y + 0.5, vx.z + 0.5).multiplyScalar(voxel_size);
		voxels.add(vx_three);

		if(vx.y === iy_floor) {
			var shadow = new THREE.Mesh(new THREE.CubeGeometry(voxel_size * 2, 0.05, voxel_size * 2),
				new THREE.MeshBasicMaterial({
					color: 'black',
					opacity: 0.1,
					transparent: true
				}));
			shadow.position = new THREE.Vector3(
				vx.x + 0.5,
				1 + iy_floor,
				vx.z + 0.5).multiplyScalar(voxel_size);
			voxels.add(shadow);
		}
	});

	return voxels;
};

DebugFE.prototype.showPlanes = function(data) {
	

	var floor = new THREE.Mesh(new THREE.CubeGeometry(10, 0.01, 10),
		new THREE.MeshBasicMaterial({
			color: '#ccc',
			map: THREE.ImageUtils.loadTexture(data.planes.tex)
		}));
	floor.position = new THREE.Vector3(0, data.planes.y, 0);

	


	return floor;
};

DebugFE.prototype.run = function() {
	// three.js
	this.camera = new THREE.PerspectiveCamera(75, 800 / 600, 0.001, 50);
	this.camera.up = new THREE.Vector3(0, 0, 1);
	this.camera.position = new THREE.Vector3(0, 2, 2);
	this.camera.lookAt(new THREE.Vector3(0, 0, 0));

	this.scene = new THREE.Scene();

	//this.scene.add(this.generateVoxelGrid());
	this.scene.add(this.generateCameraCone(0.994837674, 0.750491578));

	// start canvas
	this.renderer = new THREE.WebGLRenderer({
		canvas: $('#ui_3d')[0]
	});
	this.renderer.setClearColor(this.for_figure ? '#222': '#444');

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
