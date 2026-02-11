extends Camera3D


@export var min_distance: float = 4.0
@export var max_distance: float = 8.0
@export var height: float = 3.0


@onready var target: Node3D = get_parent()


func _physics_process(_delta: float) -> void:
	var from_target: Vector3 = global_position - target.global_position

	if from_target.length() < min_distance:
		from_target = from_target.normalized() * min_distance
	elif from_target.length() > max_distance:
		from_target = from_target.normalized() * max_distance

	from_target.y = height
	global_position = target.global_position + from_target

	var look_dir: Vector3 = global_position.direction_to(target.global_position).abs() - Vector3.UP
	if not look_dir.is_zero_approx():
		look_at_from_position(global_position, target.global_position, Vector3.UP)
