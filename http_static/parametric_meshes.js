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
