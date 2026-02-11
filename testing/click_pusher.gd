extends Node
class_name ClickPusher
## Allows clicking to push RigidBody3D's


@export_range(-10.0, 10.0, 0.1) var impulse_magnitude: float = 5.0
@export var exclude_nodes: Array[CollisionObject3D] = []
@export_flags_3d_physics var layers: int


func _unhandled_input(event: InputEvent) -> void:
	if (
		event is InputEventMouseButton
		and event.button_index == MouseButton.MOUSE_BUTTON_LEFT
		and event.pressed
	):
		var camera: Camera3D = get_viewport().get_camera_3d()
		var mouse_pos: Vector2 = get_viewport().get_mouse_position()
		var origin: Vector3 = camera.project_ray_origin(mouse_pos)
		var normal: Vector3 = camera.project_ray_normal(mouse_pos)
		var state: PhysicsDirectSpaceState3D = camera.get_world_3d().direct_space_state
		var query: PhysicsRayQueryParameters3D = PhysicsRayQueryParameters3D.create(
			origin,
			origin + (normal * 1000.0),
			layers
		)
		var to_exclude: Array[RID] = []

		for i in exclude_nodes:
			if is_instance_valid(i):
				#for c in i.find_children("*", "CollisionObject3D", false) + [i]:
				to_exclude.append(i.get_rid())
		query.exclude = to_exclude

		var res: Dictionary = state.intersect_ray(query)
		if res.has("collider") and res.collider is RigidBody3D:
			res.collider.apply_impulse(normal * impulse_magnitude, res.position - res.collider.global_position)
