// General purpose parametric three.js models

// Return colored XYZ axes.
var generateAxes = function(size) {
	var descriptions = [
		{
			color: 'red',
			rotation: new THREE.Euler(0, 0, Math.PI / 2, 'XYZ'),
			position: new THREE.Vector3(size / 2, 0, 0),
		},
		{
			color: 'green',
			rotation: new THREE.Euler(0, 0, 0, 'XYZ'),
			position: new THREE.Vector3(0, size / 2, 0),
		},
		{
			color: 'blue',
			rotation: new THREE.Euler(Math.PI / 2, 0, 0, 'XYZ'),
			position: new THREE.Vector3(0, 0, size / 2),
		},
	];
	
	var obj = new THREE.Object3D();
	_.each(descriptions, function(description) {
		var geom = new THREE.CylinderGeometry(size / 50, size / 50, size);
		var mesh = new THREE.Mesh(geom, new THREE.MeshBasicMaterial({
			color: description.color
		}));
		mesh.quaternion.setFromEuler(description.rotation);
		mesh.position = description.position;
		obj.add(mesh);
	})
	return obj;
};


var generateVoxelGrid = function(voxel_size, voxel_n) {
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
