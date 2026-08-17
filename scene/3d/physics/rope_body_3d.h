/**************************************************************************/
/*  rope_body_3d.h                                                        */
/**************************************************************************/
/*                         This file is part of:                          */
/*                             GODOT ENGINE                               */
/*                        https://godotengine.org                         */
/**************************************************************************/
/* Copyright (c) 2014-present Godot Engine contributors (see AUTHORS.md). */
/* Copyright (c) 2007-2014 Juan Linietsky, Ariel Manzur.                  */
/*                                                                        */
/* Permission is hereby granted, free of charge, to any person obtaining  */
/* a copy of this software and associated documentation files (the        */
/* "Software"), to deal in the Software without restriction, including    */
/* without limitation the rights to use, copy, modify, merge, publish,    */
/* distribute, sublicense, and/or sell copies of the Software, and to     */
/* permit persons to whom the Software is furnished to do so, subject to  */
/* the following conditions:                                              */
/*                                                                        */
/* The above copyright notice and this permission notice shall be         */
/* included in all copies or substantial portions of the Software.        */
/*                                                                        */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,        */
/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF     */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. */
/* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY   */
/* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,   */
/* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE      */
/* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                 */
/**************************************************************************/

#pragma once

#include "scene/3d/mesh_instance_3d.h"
#include "scene/property_list_helper.h"
#include "scene/resources/curve.h"
#include "scene/resources/material.h"
#include "servers/physics_3d/physics_server_3d.h"

class ArrayMesh;
class RopeAttachment3D;

// A rope: a chain of point masses with distance constraints, full segment-level collision, and
// pinning or body attachment at any point along its length.
//
// Like `SoftBody3D` this derives from `MeshInstance3D` rather than `CollisionObject3D`, because the
// simulation produces deforming geometry rather than a rigid transform, and because doing so brings
// material slots, LOD, shadow casting, and visibility ranges along for free.
//
// The simulation runs in world space, but the generated mesh is written in the node's *local*
// space, so the node keeps its authored transform in both the editor and the game. Moving the node
// at runtime therefore does not move an already-simulating rope; call `reset()` for that.
//
// Ropes are implemented only by the Jolt Physics backend. On any other backend the node is inert
// and says so through `get_configuration_warnings()`.
class RopeBody3D : public MeshInstance3D {
	GDCLASS(RopeBody3D, MeshInstance3D);

public:
	// Compliance at or above this disables bending entirely, matching `JoltRope3D`.
	static constexpr float ROPE_BEND_DISABLED = 1e6f;

	enum RenderMode {
		// No geometry. Drive your own visuals from `get_point_position()`.
		RENDER_NONE,
		// A full tube. Best looking, `radial_segments` vertices per rope point.
		RENDER_TUBE,
		// Two vertices per rope point, expanded into a camera-facing quad strip in the vertex
		// shader. Much cheaper than a tube and correct in stereo, where a CPU-side billboard
		// computed for one camera is visibly wrong in the other eye.
		RENDER_RIBBON,
	};

	enum LengthMode {
		// `length` is used directly.
		LENGTH_MANUAL,
		// The distance between the outermost pins, plus `slack`.
		LENGTH_FROM_ENDPOINTS,
		// The arc length of `spawn_curve`.
		LENGTH_FROM_CURVE,
	};

private:
	// One authored pin. `ratio` is a normalized position along the rope rather than a particle
	// index, so pins survive a change to `point_count`.
	struct Pin {
		float ratio = 0.0f;
		NodePath node_path;
		Vector3 offset;

		// Resolved once the node is in the tree.
		ObjectID node_id;
		bool is_body = false;
	};

	// A pin after `ratio` and the driving node have been resolved. Built fresh whenever the rope is
	// laid out, and merged from both the `pins` array and any `RopeAttachment3D` children.
	struct ResolvedPin {
		int index = 0;
		Vector3 position;
		ObjectID node_id;
		bool is_body = false;
		Vector3 offset;
	};

	RID rope;

	int point_count = 16;
	float length = 4.0f;
	LengthMode length_mode = LENGTH_MANUAL;
	float slack = 0.0f;
	bool auto_sag = true;
	Vector3 initial_direction = Vector3(0, -1, 0);

	// The pose the rope spawns in, in node-local space. Optional: with no curve (or a degenerate
	// one) the rope spawns straight along `initial_direction`.
	//
	// A curve of exactly two points with no tangents carries no shape beyond its endpoints, and
	// those endpoints are precisely what `length` / `initial_direction` / the attach nodes are for.
	// So while the curve is in that state it is kept in sync with them, and the moment a third
	// point or a tangent appears it belongs to the user and is never rewritten. That rule is
	// stateless, so it survives a save and reload unchanged.
	Ref<Curve3D> spawn_curve;
	// Re-entrancy guard: seeding writes the curve one point at a time, and each write emits
	// `changed`, which would otherwise re-lay the rope several times per edit.
	bool curve_reseeding = false;
	// False until the rope has actually simulated a frame. Up to that point a pin change can still
	// re-lay the spawn pose, which is what makes scene load order between a rope and the
	// `RopeAttachment3D` nodes pointing at it stop mattering.
	bool simulation_started = false;

	float radius = 0.05f;
	float total_mass = 1.0f;
	int substeps = 4;

	float stretch_compliance = 0.0f;
	float bend_compliance = ROPE_BEND_DISABLED;
	float linear_damping = 0.1f;
	float drag = 0.0f;
	float gravity_scale = 1.0f;
	float friction = 0.5f;
	float restitution = 0.0f;
	float max_reaction_impulse = 100.0f;

	bool inextensible = true;
	bool two_way_coupling = false;
	bool collision_enabled = true;

	uint32_t collision_layer = 1;
	uint32_t collision_mask = 1;

	NodePath attach_start_path;
	NodePath attach_end_path;

	LocalVector<Pin> pins;
	LocalVector<RopeAttachment3D *> attachments;
	LocalVector<ResolvedPin> resolved_pins;

	RenderMode render_mode = RENDER_TUBE;
	int radial_segments = 6;

	Ref<ArrayMesh> rope_mesh;
	Ref<ShaderMaterial> ribbon_material;

	// Cached vertex-buffer layout of the generated surface, so the per-frame update is a memcpy
	// into a staging buffer rather than a full mesh rebuild.
	Vector<uint8_t> vertex_buffer;
	uint32_t vertex_stride = 0;
	uint32_t normal_stride = 0;
	uint32_t offset_vertices = 0;
	uint32_t offset_normal = 0;
	int mesh_vertex_count = 0;
	int mesh_ring_stride = 0;
	bool mesh_dirty = true;

	// Reused across frames to keep the per-frame update allocation-free.
	LocalVector<Vector3> point_cache;
	LocalVector<Vector3> tangent_cache;
	LocalVector<Vector3> normal_cache;
	LocalVector<Vector3> binormal_cache;

	static inline PropertyListHelper base_property_helper;
	PropertyListHelper property_helper;

	bool is_backend_supported() const;

	// True once the curve carries a shape of its own, i.e. more than two points or any tangent.
	bool _is_curve_authored() const;

	void _update_simulation_params();
	void _queue_rebuild();
	void _rebuild_points();
	void _resolve_pins();
	// Pushes the resolved pin set to the solver. Split out of `_rebuild_points()` so that a pin
	// being added, moved along the rope, or removed does not have to re-lay the spawn pose --
	// which, once the rope is running, would throw the simulation away.
	void _apply_pins();
	void _pins_changed();
	void _update_kinematic_pins();
	void _curve_changed();
	void _reseed_curve();
	bool _sample_spawn_polyline(LocalVector<Vector3> &r_points) const;
	void _apply_sag(LocalVector<Vector3> &r_points, float p_target_length) const;
	Node3D *_get_pin_node(const ResolvedPin &p_pin) const;

	void _mark_mesh_dirty();
	void _rebuild_mesh();
	void _update_mesh();
	void _update_frames();
	Ref<ShaderMaterial> _get_ribbon_material();

protected:
	void _notification(int p_what);
	static void _bind_methods();
	void _validate_property(PropertyInfo &p_property) const;

	bool _set(const StringName &p_name, const Variant &p_value) { return property_helper.property_set_value(p_name, p_value); }
	bool _get(const StringName &p_name, Variant &r_ret) const { return property_helper.property_get_value(p_name, r_ret); }
	void _get_property_list(List<PropertyInfo> *p_list) const { property_helper.get_property_list(p_list); }
	bool _property_can_revert(const StringName &p_name) const { return property_helper.property_can_revert(p_name); }
	bool _property_get_revert(const StringName &p_name, Variant &r_property) const { return property_helper.property_get_revert(p_name, r_property); }

public:
	RID get_physics_rid() const { return rope; }

	void set_point_count(int p_count);
	int get_point_count() const { return point_count; }

	void set_length(float p_length);
	float get_length() const { return length; }

	void set_length_mode(LengthMode p_mode);
	LengthMode get_length_mode() const { return length_mode; }

	void set_slack(float p_slack);
	float get_slack() const { return slack; }

	void set_auto_sag(bool p_enabled);
	bool is_auto_sag() const { return auto_sag; }

	// Derived. The distance between two neighbouring particles at rest.
	float get_segment_length() const;

	void set_spawn_curve(const Ref<Curve3D> &p_curve);
	Ref<Curve3D> get_spawn_curve() const { return spawn_curve; }

	void set_initial_direction(const Vector3 &p_direction);
	Vector3 get_initial_direction() const { return initial_direction; }

	void set_radius(float p_radius);
	float get_radius() const { return radius; }

	void set_total_mass(float p_mass);
	float get_total_mass() const { return total_mass; }

	void set_simulation_substeps(int p_substeps);
	int get_simulation_substeps() const { return substeps; }

	void set_stretch_compliance(float p_compliance);
	float get_stretch_compliance() const { return stretch_compliance; }

	void set_bend_compliance(float p_compliance);
	float get_bend_compliance() const { return bend_compliance; }

	void set_linear_damping(float p_damping);
	float get_linear_damping() const { return linear_damping; }

	void set_drag(float p_drag);
	float get_drag() const { return drag; }

	void set_gravity_scale(float p_scale);
	float get_gravity_scale() const { return gravity_scale; }

	void set_friction(float p_friction);
	float get_friction() const { return friction; }

	void set_restitution(float p_restitution);
	float get_restitution() const { return restitution; }

	void set_max_reaction_impulse(float p_impulse);
	float get_max_reaction_impulse() const { return max_reaction_impulse; }

	void set_inextensible(bool p_enabled);
	bool is_inextensible() const { return inextensible; }

	void set_two_way_coupling(bool p_enabled);
	bool is_two_way_coupling() const { return two_way_coupling; }

	void set_collision_enabled(bool p_enabled);
	bool is_collision_enabled() const { return collision_enabled; }

	void set_collision_layer(uint32_t p_layer);
	uint32_t get_collision_layer() const { return collision_layer; }

	void set_collision_mask(uint32_t p_mask);
	uint32_t get_collision_mask() const { return collision_mask; }

	void set_collision_layer_value(int p_layer_number, bool p_value);
	bool get_collision_layer_value(int p_layer_number) const;

	void set_collision_mask_value(int p_layer_number, bool p_value);
	bool get_collision_mask_value(int p_layer_number) const;

	void set_attach_start(const NodePath &p_path);
	NodePath get_attach_start() const { return attach_start_path; }

	void set_attach_end(const NodePath &p_path);
	NodePath get_attach_end() const { return attach_end_path; }

	/* Authored pins. */
	void set_pin_count(int p_count);
	int get_pin_count() const { return (int)pins.size(); }

	void set_pin_ratio(int p_pin, float p_ratio);
	float get_pin_ratio(int p_pin) const;

	void set_pin_node(int p_pin, const NodePath &p_path);
	NodePath get_pin_node(int p_pin) const;

	void set_pin_offset(int p_pin, const Vector3 &p_offset);
	Vector3 get_pin_offset(int p_pin) const;

	// Adds a pin at `p_ratio` if there is not one there already, or removes the one that is.
	// Returns the resulting pin count. Used by the editor gizmo's click-to-pin.
	int toggle_pin_at_ratio(float p_ratio);
	int find_pin_at_index(int p_point_index) const;

	// Called by `RopeAttachment3D` nodes that point at this rope. They live under whatever they
	// tie the rope to (usually a `PhysicsBody3D`), not under the rope, so they have to announce
	// themselves rather than be discovered as children.
	void register_attachment(RopeAttachment3D *p_attachment);
	void unregister_attachment(RopeAttachment3D *p_attachment);
	void attachment_changed();

	// Where along the rope a world-space point is nearest, as a 0-1 ratio measured the same way
	// pins and `RopeAttachment3D` measure it -- so the result can be handed straight to one. This
	// is how you grab a rope: raycast or use the hand position, ask for the ratio, attach there.
	float get_closest_ratio(const Vector3 &p_world_point) const;
	// The nearest point on the rope itself, along a segment rather than snapped to a particle.
	Vector3 get_closest_point(const Vector3 &p_world_point) const;
	// World position at a 0-1 ratio, interpolated between particles.
	Vector3 sample_ratio(float p_ratio) const;

	Vector3 get_point_position(int p_index) const;
	Vector3 get_point_velocity(int p_index) const;
	PackedVector3Array get_points() const;
	// Sum of the simulated segment lengths. Diagnostic; compare against `get_length()`.
	float get_simulated_length() const;

	void pin_point(int p_index, bool p_pin);
	bool is_point_pinned(int p_index) const;
	void set_pin_position(int p_index, const Vector3 &p_position);

	void attach_point(int p_index, Node *p_body);
	void detach_point(int p_index);

	void apply_point_impulse(int p_index, const Vector3 &p_impulse);
	void apply_central_impulse(const Vector3 &p_impulse);

	// Re-lays the rope out from its spawn pose and clears all motion.
	void reset();

	void set_render_mode(RenderMode p_mode);
	RenderMode get_render_mode() const { return render_mode; }

	void set_radial_segments(int p_segments);
	int get_radial_segments() const { return radial_segments; }

	virtual PackedStringArray get_configuration_warnings() const override;

	RopeBody3D();
	~RopeBody3D();
};

VARIANT_ENUM_CAST(RopeBody3D::RenderMode);
VARIANT_ENUM_CAST(RopeBody3D::LengthMode);
