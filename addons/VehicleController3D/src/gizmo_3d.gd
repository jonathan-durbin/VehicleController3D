## Visual debug gizmo that renders axis force indicators.
extends Node3D
class_name Gizmo3D


## Mesh instances keyed by axis name.
var axes: Dictionary[String, MeshInstance3D] = {}


## Creates axis meshes when the gizmo enters the scene tree.
func _ready() -> void:
	_create_axis_mesh("x")
	_create_axis_mesh("y")
	_create_axis_mesh("z")


## Sets the axis mesh length and offset based on a force magnitude.
func set_targetf(axis: String, target: float) -> void:
	var _axis: String = axis.to_lower()
	if _axis not in axes.keys():
		push_error("Axis not found: ", axis)
		return
	match _axis:
		"x":
			axes[_axis].position.x = target / 2.0
			axes[_axis].mesh.height = absf(target)
		"y":
			axes[_axis].position.y = target / 2.0
			axes[_axis].mesh.height = absf(target)
		"z":
			axes[_axis].position.z = target / 2.0
			axes[_axis].mesh.height = absf(target)


## Builds a colored cylinder mesh to represent an axis direction.
func _create_axis_mesh(axis: String) -> void:
	# Create mesh and shape
	var mesh := MeshInstance3D.new()
	mesh.mesh = CylinderMesh.new()

	mesh.mesh.top_radius = 0.05
	mesh.mesh.bottom_radius = 0.05
	mesh.mesh.radial_segments = 8
	mesh.material_override = StandardMaterial3D.new()
	add_child(mesh)

	match axis:
		"x":
			mesh.material_override.albedo_color = Color("f73d65")
			mesh.rotate_z(PI/2.0)
		"y":
			mesh.material_override.albedo_color = Color("b0e952")
		"z":
			mesh.material_override.albedo_color = Color("32a9f5")
			mesh.rotate_x(PI/2.0)

	axes[axis] = mesh
