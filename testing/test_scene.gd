extends Node3D


const GLOBAL_CTX = preload("uid://c2hrb2jqlkrmw")


# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	GUIDE.enable_mapping_context(GLOBAL_CTX)
	
