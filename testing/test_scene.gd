## Simple test scene script for GUIDE context setup.
extends Node3D


## Global GUIDE mapping context used for test input.
const GLOBAL_CTX = preload("uid://c2hrb2jqlkrmw")


## Called when the node enters the scene tree for the first time.
func _ready() -> void:
	GUIDE.enable_mapping_context(GLOBAL_CTX)
	
