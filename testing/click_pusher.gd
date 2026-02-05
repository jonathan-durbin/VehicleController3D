extends Node

@export var exclude: Array[CollisionObject3D] = []


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
			origin + (normal * 1000.0)
		)
		var to_exclude: Array[RID] = []

		for i in exclude:
			if is_instance_valid(i):
				for c in i.find_children("*", "CollisionObject3D") + [i]:
					to_exclude.append(c.get_rid())
		query.exclude = to_exclude

		var res: Dictionary = state.intersect_ray(query)
		if res.has("collider") and res.collider is RigidBody3D:
			res.collider.apply_impulse(normal * 5.0, res.position - res.collider.global_position)
