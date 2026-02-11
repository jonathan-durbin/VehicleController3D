extends Node3D
class_name SBVWheelRoot


@export var spring_strength: float = 100.0
@export var wheel_radius: float = 0.3

@export var num_nubs: int = 6
@export var nub_radius: float = 0.05

@export_flags_3d_physics var nub_layer: int
@export_flags_3d_physics var nub_mask: int


var nubs: Array[SBVWheelNub] = []


func _ready() -> void:
	for n in num_nubs:
		var nub_sphere_shape = SphereShape3D.new()
		nub_sphere_shape.radius = nub_radius
		nub_sphere_shape.margin = 0.01
		var nub_coll_shape: CollisionShape3D = CollisionShape3D.new()
		nub_coll_shape.shape = nub_sphere_shape
		var nub: SBVWheelNub = SBVWheelNub.new()
		add_child(nub)
		nub.add_child(nub_coll_shape)
		var nub_rot: float = lerpf(0.0, TAU, float(n)/float(num_nubs))
		nub.rotation.x = nub_rot
		var nub_pos_2d: Vector2 = Vector2(0.0, wheel_radius).rotated(nub_rot)
		nub.position = Vector3(0.0, nub_pos_2d.y, nub_pos_2d.x)
		nub.target_pos = nub.position
		nub.collision_layer = nub_layer
		nub.collision_mask = nub_mask
