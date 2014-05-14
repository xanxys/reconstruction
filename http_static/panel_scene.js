"use strict";

// Common deserializations for primitives.
var deserializeVector3 = function(d) {
	return new THREE.Vector3(d.x, d.y, d.z);
};

var SceneSearchTree = Backbone.Model.extend({
	urlRoot: '/scene'
});

// TODO: deprecated
var Scene = Backbone.Model.extend({
	urlRoot: '/scene'
});

// TODO: rename properly
var NewScene = Backbone.Model.extend({
	urlRoot: '/scene',

	initialize: function(options) {
		this.scene_id = options.scene_id;
	},

	url: function() {
		return this.urlRoot + '/' + this.scene_id + '/belief/' + this.id;
	}
});



var GrabcutView = Backbone.View.extend({
	el: '#grabcut',

	events: {
		"click #ui_draw_fg": "changeToForeground",
		"click #ui_draw_bg": "changeToBackground",
		"click #ui_clear": "clear",
		"click #ui_do_grabcut": "doGrabcut",
		"mousedown #ui_grabcut_drawing": "beginDrawing",
		"mousemove #ui_grabcut_drawing": "moveTool",
		"mouseup #ui_grabcut_drawing": "endDrawing",
	},

	initialize: function(options) {
		this.ctx = this.$('#ui_grabcut_drawing')[0].getContext('2d');
		this.drawing = false;

		this.changeToForeground();
		this.listenTo(this.model, 'change', this.render);
	},

	clear: function() {
		this.ctx.clearRect(0, 0, 640, 480);
	},

	beginDrawing: function() {
		this.drawing = true;
	},

	endDrawing: function() {
		this.drawing = false;
	},

	moveTool: function(event) {
		if(!this.drawing) {
			return;
		}

		this.ctx.beginPath();
		this.ctx.arc(event.offsetX, event.offsetY, 5, 0, 2 * Math.PI);
		this.ctx.fill();
	},

	changeToForeground: function() {
		this.ctx.fillStyle = 'blue';
	},

	changeToBackground: function() {
		this.ctx.fillStyle = 'red';
	},

	doGrabcut: function() {
		// HACK ALERT
		// TODO: remove this
		var id = (this.model.scene_id !== undefined) ? this.model.scene_id : this.model.id;

		$.ajax({
			type: 'POST',
			url: '/at/' + id + '/grabcut',
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
	},

	render: function() {
		var img = new Image();
		img.onload = function() {
			var ctx = $('#ui_grabcut')[0].getContext('2d');
			ctx.drawImage(img, 0, 0);
		};
		img.src = this.model.get('rgb');
	}
});


var CameraLayer = function() {
	this.label = 'Camera';
	this.endpoint = 'camera';
};

CameraLayer.prototype.generator = function(d) {
	var camera = new THREE.Object3D();
	camera.add(this.generateCameraCone(0.994837674, 0.750491578));
	camera.add(generateAxes(0.3));
	camera.quaternion = new THREE.Quaternion(d.x, d.y, d.z, d.w);
	return camera;
};

CameraLayer.prototype.generateCameraCone = function(fov_h, fov_v) {
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


var PointsLayer = function() {
	this.label = 'Points';
	this.endpoint = 'points';
};

PointsLayer.prototype.generator = function(data) {
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


var PlanesLayer = function() {
	this.label = 'Planes';
	this.endpoint = 'planes';
};

PlanesLayer.prototype.generator = function(data) {
	var planes = new THREE.Object3D();
	_.each(data, function(plane_desc) {
		// TODO: proper texturing
		var geom = null;
		if(plane_desc.normal[0] === 'x') {
			geom = new THREE.CubeGeometry(0.01, plane_desc.size, plane_desc.size);
		} else if(plane_desc.normal[0] === 'y') {
			geom = new THREE.CubeGeometry(plane_desc.size, 0.01, plane_desc.size);
		} else {
			geom = new THREE.CubeGeometry(plane_desc.size, plane_desc.size, 0.01);
		}
	
		var plane = new THREE.Mesh(geom,
			new THREE.MeshBasicMaterial({
				color: '#ccc',
				map: THREE.ImageUtils.loadTexture(plane_desc.tex)
		}));
		plane.position = deserializeVector3(plane_desc.center);
		planes.add(plane);
	});
	return planes;
};


var ObjectsLayer = function() {
	this.label = 'Objects';
	this.endpoint = 'objects';
};

ObjectsLayer.prototype.generator = function(data) {
	var objects = new THREE.Object3D();

	var voxel_size = 0.1;

	var num_invalid = 0;
	_.each(data, function(object_desc) {
		if(!object_desc.valid) {
			num_invalid += 1;
			return;
		}

		var size = deserializeVector3(object_desc.size);

		var obj = new THREE.Mesh(
			new THREE.CubeGeometry(size.x, size.y, size.z),
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
		obj.position = deserializeVector3(object_desc.pos);

		obj.quaternion.setFromAxisAngle(new THREE.Vector3(0, 1, 0), object_desc.ry);
		objects.add(obj);
	});
	console.log('Invalid Object Proxies ', num_invalid, '/', data.length);
	return objects;
};


var VoxelsLayer = function(name_attrib, ep, color) {
	this.label = 'Voxels (' + name_attrib + ')';
	this.color = color;
	this.endpoint = ep;
};

VoxelsLayer.prototype.generator = function(data) {
	var voxel_size = 0.1;

	var iy_floor = _.max(_.map(data, function(vx) {
		return vx.y;
	}));

	var _this = this;
	var voxels = new THREE.Object3D();
	var voxels_geom = new THREE.Geometry();
	_.each(data, function(vx) {
		var vx_three = new THREE.Mesh(
			new THREE.CubeGeometry(voxel_size, voxel_size, voxel_size),
			new THREE.MeshBasicMaterial());
		vx_three.position = new THREE.Vector3(vx.x + 0.5, vx.y + 0.5, vx.z + 0.5).multiplyScalar(voxel_size);
		THREE.GeometryUtils.merge(voxels_geom, vx_three);

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

	voxels.add(new THREE.Mesh(
		voxels_geom,
		new THREE.MeshBasicMaterial({
			color: _this.color,
				opacity: 0.3,
				transparent: true
			})
		));

	return voxels;
};

var SceneSearchView = Backbone.View.extend({
	el: '#search',

	initialize: function(options) {
		this.selected = null;
		this.listenTo(this.model, 'change', this.render);
	},

	render: function() {
		this.$el.empty();

		var _this = this;
		_.each(this.model.get('candidates'), function(candidate) {
			var entry = $('<a/>')
				.text(candidate).attr('href', '#').addClass('list-group-item');
			if(candidate === _this.selected) {
				entry.addClass('active');
			}
			entry.click(function() {
				_this.selected = candidate;
				_this.render();
				_this.trigger('selectBelief', _this.selected);
			});
			_this.$el.append(entry);	
		});
	}
});

var LayersConfig = Backbone.Model.extend({
	initialize: function(options) {
		this.layer_descs = [
			new VoxelsLayer('filled', 'voxels', 'orangered'),
			new VoxelsLayer('empty', 'voxels_empty', 'orange'),
			new ObjectsLayer(),
			new PointsLayer(),
			new PlanesLayer(),
			new CameraLayer()
		];

		this.visible = _.map(this.layer_descs, function() {
			return true;
		});
	}
});

var MainView = Backbone.View.extend({
	el: '#ui_3d',

	initialize: function(options) {
		this.layers_config = options.layers_config;

		this.layer_cache = {};
		this.run();

//		this.listenTo(this.model, 'change', this.refresh);
		this.listenTo(this.layers_config, 'change', this.render);
	},

	run: function() {
		// three.js
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
	},

	animate: function() {
		var _this = this;
		requestAnimationFrame(function(){_this.animate()});

		this.renderer.render(this.scene, this.camera);
		this.controls.update();
	},

	refresh: function() {
		var _this = this;
		_.each(_this.layers_config.layer_descs, function(layer_desc) {
			var data = _this.model.get(layer_desc.endpoint);
			_this.layer_cache[layer_desc.endpoint] = layer_desc.generator(data);
		});

		this.render();
	},

	render: function() {
		this.empty();

		// Add visible layers.
		var _this = this;
		_.each(_this.layers_config.layer_descs, function(layer_desc, ix) {
			if(_this.layers_config.visible[ix]) {
				_this.scene.add(_this.layer_cache[layer_desc.endpoint]);
			}
		});
	},

	empty: function() {
		_.each(_.clone(this.scene.children), function(child) {
			this.scene.remove(child);
		}, this);
	}
});

var PeelingView = Backbone.View.extend({
	el: '#peeling',

	initialize: function(options) {
		this.listenTo(this.model, 'change', this.render);
	},

	render: function() {
		var peeling = this.model.get('peeling');

		$(this.el).empty();

		$(this.el).append($('<h2/>').text('Thumbnails'));
		$(this.el).append($('<img/>').attr('src', peeling['target']).attr('width', '160px'));
		$(this.el).append($('<img/>').attr('src', peeling['render']).attr('width', '160px'));
		$(this.el).append($('<img/>').attr('src', peeling['delta']).attr('width', '160px'));

		$(this.el).append($('<br/>'));
		$(this.el).append($('<span/>').text(
			'L1 norm: ' + peeling['l1'] + ' / L1 norm(per px): ' + peeling['l1'] / (640 * 480)));

		$(this.el).append($('<h2/>').text('Original'));
		$(this.el).append($('<img/>').attr('src', peeling['target']));
		$(this.el).append($('<img/>').attr('src', peeling['render']));
		$(this.el).append($('<img/>').attr('src', peeling['delta']));
	}
});

var CalibrationView = Backbone.View.extend({
	el: '#calibration',

	initialize: function(options) {
		this.listenTo(this.model, 'change', this.render);
	},

	render: function() {
		$(this.el).empty();
		$(this.el).append($('<img/>').attr('src', this.model.get('rgb')));
		$(this.el).append($('<img/>').attr('src', this.model.get('depth')));
	}
});

var LayersView = Backbone.View.extend({
	el: '#ui_layers',

	initialize: function(options) {
		this.listenTo(this.model, 'change', this.render);
	},

	render: function() {
		this.$el.empty();

		var _this = this;
		_.each(this.model.layer_descs, function(layer_desc, ix) {
			var layer = $('<a/>').attr('src', '#')
				.addClass('list-group-item')
				.text(layer_desc.label);

			if(_this.model.visible[ix]) {
				layer.addClass('active');
			}

			layer.click(function() {
				_this.model.visible[ix] = !_this.model.visible[ix];
				_this.model.trigger('change');
			});
			_this.$el.append(layer);
		});
	}
});

var LogView = Backbone.View.extend({
	el: '#log',

	initialize: function(options) {
		this.listenTo(this.model, 'change', this.render);
	},

	render: function() {
		this.$('pre').text(this.model.get('log'));
	}
});

var ScenePanel = Backbone.View.extend({
	initialize: function(options) {
		// When this mode is enabled, try producing high-contrast, big-text, less-clutter imagery.
		this.for_figure = false;

		this.id = options.id;
	},

	render: function() {
		$('#panel').html(_.template($('script[name=scene_panel]').html()));

		var _this = this;
		var id = this.id;

		this.layers_config = new LayersConfig();

		this.main_view = new MainView({
			layers_config: this.layers_config
		});

		var config_view = new LayersView({model: this.layers_config});
		this.layers_config.trigger('change');



		var tree = new SceneSearchTree({id: id});
		var scene_search_view = new SceneSearchView({model: tree});
		tree.fetch({});

		var _this = this;
		scene_search_view.on('selectBelief', function(ev) {
			var scene = new NewScene({
				scene_id: id,
				id: ev
			});

			var peeling_view = new PeelingView({model: scene});
			var log_view = new LogView({model: scene});
			var calibration_view = new CalibrationView({model: scene});
			var grabcut_view = new GrabcutView({model: scene});
			_this.main_view.model = scene;
			_this.main_view.listenTo(scene, 'change', _this.main_view.refresh);
			scene.fetch();
		});

		// TODO: deprecate Scene
		var scene = new Scene({id: id});
		var peeling_view = new PeelingView({model: scene});
		var log_view = new LogView({model: scene});
		var calibration_view = new CalibrationView({model: scene});
		var grabcut_view = new GrabcutView({model: scene});
		this.main_view.model = scene;
		this.main_view.listenTo(scene, 'change', this.main_view.refresh);
		scene.fetch();
	}
});
