/**************************************************************************/
/*  rope_body_3d.cpp                                                      */
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

#include "rope_body_3d.h"

#include "core/config/engine.h"
#include "core/config/project_settings.h"
#include "core/object/callable_mp.h"
#include "core/object/class_db.h"
#include "scene/3d/physics/area_3d.h"
#include "scene/3d/physics/physics_body_3d.h"
#include "scene/3d/physics/rope_attachment_3d.h"
#include "scene/main/scene_tree.h"
#include "scene/resources/3d/world_3d.h"
#include "scene/resources/curve.h"
#include "scene/resources/material.h"
#include "scene/resources/mesh.h"
#include "scene/resources/shader.h"
#include "servers/rendering/rendering_server.h"

// Expands the two-vertex-per-point ribbon strip into a camera-facing quad on the GPU.
//
// This must happen in the shader rather than on the CPU: with multiview, `INV_VIEW_MATRIX` differs
// per eye, so a ribbon billboarded once on the CPU for a single camera is measurably wrong in the
// other eye. Doing it here is both correct in stereo and cheaper than rewriting the strip every
// frame.
//
// The mesh stores the rope centreline in VERTEX, the segment tangent in NORMAL, and which side of
// the ribbon each vertex belongs to in UV.x (0 or 1) -- all in *model* space, since the node keeps
// its own transform. The billboard has to be computed against the world-space camera position, so
// the offset is built in world space and brought back with `transpose(mat3(MODEL_MATRIX))`, which
// is the exact inverse for a rigid transform and far cheaper than a full `inverse()`. A
// non-uniformly scaled rope node is not supported.
static const char *ROPE_RIBBON_SHADER_CODE =
		"shader_type spatial;\n"
		"render_mode cull_disabled;\n"
		"\n"
		"uniform float rope_radius : hint_range(0.001, 10.0) = 0.05;\n"
		"uniform vec3 albedo : source_color = vec3(0.6, 0.5, 0.4);\n"
		"uniform float roughness : hint_range(0.0, 1.0) = 0.9;\n"
		"\n"
		"void vertex() {\n"
		"\tmat3 model_basis = mat3(MODEL_MATRIX);\n"
		"\tmat3 to_model = transpose(model_basis);\n"
		"\tvec3 world_position = (MODEL_MATRIX * vec4(VERTEX, 1.0)).xyz;\n"
		"\tvec3 tangent_dir = normalize(model_basis * NORMAL);\n"
		"\tvec3 to_camera = normalize(INV_VIEW_MATRIX[3].xyz - world_position);\n"
		"\tvec3 side = cross(tangent_dir, to_camera);\n"
		"\tfloat side_length = length(side);\n"
		"\tif (side_length > 0.0001) {\n"
		"\t\tside /= side_length;\n"
		"\t\tVERTEX += (to_model * side) * (rope_radius * (UV.x * 2.0 - 1.0));\n"
		"\t}\n"
		"\tNORMAL = normalize(to_model * to_camera);\n"
		"}\n"
		"\n"
		"void fragment() {\n"
		"\tALBEDO = albedo;\n"
		"\tROUGHNESS = roughness;\n"
		"}\n";

RopeBody3D::RopeBody3D() {
	rope = PhysicsServer3D::get_singleton()->rope_create();

	// No curve is instantiated here on purpose: an Object as a ClassDB default value is a
	// documented anti-pattern. `spawn_curve` carries PROPERTY_USAGE_EDITOR_INSTANTIATE_OBJECT
	// instead, so the editor creates one when the node is added and the gizmo always has handles.
	property_helper.setup_for_instance(base_property_helper, this);
}

RopeBody3D::~RopeBody3D() {
	if (spawn_curve.is_valid()) {
		spawn_curve->disconnect_changed(callable_mp(this, &RopeBody3D::_curve_changed));
	}

	ERR_FAIL_NULL(PhysicsServer3D::get_singleton());
	PhysicsServer3D::get_singleton()->free_rid(rope);
}

bool RopeBody3D::is_backend_supported() const {
	// Ropes are a Jolt-only body type. Every other backend inherits the no-op defaults on
	// `PhysicsServer3D`, which return an invalid RID from `rope_create()`.
	return rope.is_valid();
}

/* -------------------------------------------------------------------------- */
/* Lifecycle                                                                  */
/* -------------------------------------------------------------------------- */

void RopeBody3D::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_ENTER_WORLD: {
			if (!is_backend_supported()) {
				break;
			}

			// Deliberately not added to the space in the editor. The editor's `World3D` space is
			// active like any other, so the rope would quietly fall while you were authoring it;
			// the point of the spawn pose is to see exactly the pose you drew.
			if (!Engine::get_singleton()->is_editor_hint()) {
				PhysicsServer3D::get_singleton()->rope_set_space(rope, get_world_3d()->get_space());
			}

			_update_simulation_params();
			_reseed_curve();
			_rebuild_points();

			// Built here rather than lazily on the first draw, so the mesh resource exists as soon
			// as the node is in the tree -- scripts can inspect it, and a headless run still has it.
			_rebuild_mesh();

			set_physics_process_internal(true);
			RS::get_singleton()->connect("frame_pre_draw", callable_mp(this, &RopeBody3D::_update_mesh));

			all_ropes.push_back(this);
		} break;

		case NOTIFICATION_EXIT_WORLD: {
			if (!is_backend_supported()) {
				break;
			}

			all_ropes.erase(this);

			if (RS::get_singleton()->is_connected("frame_pre_draw", callable_mp(this, &RopeBody3D::_update_mesh))) {
				RS::get_singleton()->disconnect("frame_pre_draw", callable_mp(this, &RopeBody3D::_update_mesh));
			}

			set_physics_process_internal(false);

			PhysicsServer3D::get_singleton()->rope_set_space(rope, RID());
		} break;

		case NOTIFICATION_READY: {
			// Attach paths only resolve once the whole branch is in the tree.
			_reseed_curve();
			_rebuild_points();
		} break;

		case NOTIFICATION_RESET_PHYSICS_INTERPOLATION: {
			// A teleported or freshly spawned rope has no meaningful previous pose, and interpolating
			// from one would smear it across the jump.
			if (is_backend_supported()) {
				PhysicsServer3D::get_singleton()->rope_reset_interpolation(rope);
			}
		} break;

		case NOTIFICATION_TRANSFORM_CHANGED: {
			// The mesh is written in local space, so a transform change needs no compensation at
			// runtime -- the rope simply stays where it is being simulated. In the editor the node
			// is still being authored, so a move re-lays the spawn pose from its new transform.
			if (is_inside_tree() && Engine::get_singleton()->is_editor_hint()) {
				_reseed_curve();
				_rebuild_points();
			}
		} break;

		case NOTIFICATION_INTERNAL_PHYSICS_PROCESS: {
			simulation_started = true;
			_update_kinematic_pins();
		} break;
	}
}

void RopeBody3D::_update_simulation_params() {
	if (!is_backend_supported()) {
		return;
	}

	PhysicsServer3D *ps = PhysicsServer3D::get_singleton();

	ps->rope_set_param(rope, PhysicsServer3D::ROPE_PARAM_RADIUS, radius);
	ps->rope_set_param(rope, PhysicsServer3D::ROPE_PARAM_TOTAL_MASS, total_mass);
	ps->rope_set_param(rope, PhysicsServer3D::ROPE_PARAM_STRETCH_COMPLIANCE, stretch_compliance);
	ps->rope_set_param(rope, PhysicsServer3D::ROPE_PARAM_BEND_COMPLIANCE, bend_compliance);
	ps->rope_set_param(rope, PhysicsServer3D::ROPE_PARAM_TWIST_COMPLIANCE, twist_compliance);
	ps->rope_set_param(rope, PhysicsServer3D::ROPE_PARAM_TWIST_DAMPING, twist_damping);
	ps->rope_set_param(rope, PhysicsServer3D::ROPE_PARAM_LINEAR_DAMPING, linear_damping);
	ps->rope_set_param(rope, PhysicsServer3D::ROPE_PARAM_DRAG, drag);
	ps->rope_set_param(rope, PhysicsServer3D::ROPE_PARAM_GRAVITY_SCALE, gravity_scale);
	ps->rope_set_param(rope, PhysicsServer3D::ROPE_PARAM_FRICTION, friction);
	ps->rope_set_param(rope, PhysicsServer3D::ROPE_PARAM_RESTITUTION, restitution);
	ps->rope_set_param(rope, PhysicsServer3D::ROPE_PARAM_MAX_REACTION_IMPULSE, max_reaction_impulse);

	ps->rope_set_flag(rope, PhysicsServer3D::ROPE_FLAG_INEXTENSIBLE, inextensible);
	ps->rope_set_flag(rope, PhysicsServer3D::ROPE_FLAG_TWO_WAY_COUPLING, two_way_coupling);
	ps->rope_set_flag(rope, PhysicsServer3D::ROPE_FLAG_COLLISION_ENABLED, collision_enabled);

	ps->rope_set_simulation_substeps(rope, substeps);
	ps->rope_set_collision_layer(rope, collision_layer);
	ps->rope_set_collision_mask(rope, collision_mask);
}

void RopeBody3D::_queue_rebuild() {
	if (is_inside_tree()) {
		_rebuild_points();
	}
}

/* -------------------------------------------------------------------------- */
/* Spawn pose                                                                 */
/* -------------------------------------------------------------------------- */

bool RopeBody3D::_is_curve_authored() const {
	if (spawn_curve.is_null()) {
		return false;
	}
	if (spawn_curve->get_point_count() != 2) {
		return spawn_curve->get_point_count() > 2;
	}

	for (int i = 0; i < 2; i++) {
		if (spawn_curve->get_point_in(i).length_squared() > CMP_EPSILON ||
				spawn_curve->get_point_out(i).length_squared() > CMP_EPSILON) {
			return true;
		}
	}
	return false;
}

void RopeBody3D::_curve_changed() {
	if (curve_reseeding) {
		return;
	}

	_queue_rebuild();
	update_gizmos();
}

void RopeBody3D::_reseed_curve() {
	if (spawn_curve.is_null() || _is_curve_authored()) {
		return;
	}

	Vector3 start;
	Vector3 end;

	Vector3 direction = initial_direction;
	if (direction.length_squared() < CMP_EPSILON) {
		direction = Vector3(0, -1, 0);
	}
	end = start + direction.normalized() * length;

	// With endpoints assigned, seed straight between them (in local space) so the default pose
	// already spans what it is going to be pinned to.
	if (is_inside_tree()) {
		const Transform3D inv = get_global_transform().affine_inverse();
		Node3D *start_node = Object::cast_to<Node3D>(get_node_or_null(attach_start_path));
		Node3D *end_node = Object::cast_to<Node3D>(get_node_or_null(attach_end_path));

		if (start_node != nullptr) {
			start = inv.xform(start_node->get_global_position());
			if (end_node == nullptr) {
				end = start + direction.normalized() * length;
			}
		}
		if (end_node != nullptr) {
			end = inv.xform(end_node->get_global_position());
		}
	}

	if (spawn_curve->get_point_count() == 2 &&
			spawn_curve->get_point_position(0).is_equal_approx(start) &&
			spawn_curve->get_point_position(1).is_equal_approx(end)) {
		return;
	}

	curve_reseeding = true;
	spawn_curve->set_point_count(2);
	spawn_curve->set_point_position(0, start);
	spawn_curve->set_point_position(1, end);
	curve_reseeding = false;

	update_gizmos();
}

bool RopeBody3D::_sample_spawn_polyline(LocalVector<Vector3> &r_points) const {
	r_points.resize(point_count);

	const Transform3D transform = get_global_transform();

	if (spawn_curve.is_valid() && spawn_curve->get_point_count() >= 2) {
		const real_t curve_length = spawn_curve->get_baked_length();
		if (curve_length > (real_t)CMP_EPSILON) {
			for (int i = 0; i < point_count; i++) {
				const real_t offset = curve_length * (real_t)i / (real_t)(point_count - 1);
				r_points[i] = transform.xform(spawn_curve->sample_baked(offset));
			}
			return true;
		}
	}

	Vector3 direction = transform.basis.xform(initial_direction);
	if (direction.length_squared() < CMP_EPSILON) {
		direction = Vector3(0, -1, 0);
	}
	direction.normalize();

	const float step = length / (float)(point_count - 1);
	for (int i = 0; i < point_count; i++) {
		r_points[i] = transform.origin + direction * (step * i);
	}
	return true;
}

// Bows each span between consecutive pins downhill so that a rope with slack spawns slack instead
// of spawning taut and dropping on the first frame. The sag depth per span is found by bisection
// on the arc length of a quadratic, which is both cheap and monotone in the depth.
void RopeBody3D::_apply_sag(LocalVector<Vector3> &r_points, float p_target_length) const {
	const int count = (int)r_points.size();
	if (count < 3) {
		return;
	}

	float current = 0.0f;
	for (int i = 0; i + 1 < count; i++) {
		current += (float)r_points[i].distance_to(r_points[i + 1]);
	}

	const float excess = p_target_length - current;
	if (excess <= p_target_length * 0.001f || current <= CMP_EPSILON) {
		return;
	}

	Vector3 down = Vector3(0, -1, 0);
	const Vector3 gravity = GLOBAL_GET("physics/3d/default_gravity_vector");
	if (gravity.length_squared() > CMP_EPSILON) {
		down = gravity.normalized();
	}

	// Spans are delimited by pinned particles; an unpinned rope is one span end to end.
	LocalVector<int> boundaries;
	boundaries.push_back(0);
	for (const ResolvedPin &pin : resolved_pins) {
		if (pin.index > boundaries[boundaries.size() - 1] && pin.index < count - 1) {
			boundaries.push_back(pin.index);
		}
	}
	boundaries.push_back(count - 1);

	// Excess is shared out in proportion to each span's length, so a long span sags more.
	for (uint32_t s = 0; s + 1 < boundaries.size(); s++) {
		const int from = boundaries[s];
		const int to = boundaries[s + 1];
		if (to - from < 2) {
			continue;
		}

		float span_length = 0.0f;
		for (int i = from; i < to; i++) {
			span_length += (float)r_points[i].distance_to(r_points[i + 1]);
		}
		if (span_length <= CMP_EPSILON) {
			continue;
		}

		const float target = span_length + excess * (span_length / current);

		// A parabola of depth `d` over a chord of length `c` has arc length that grows
		// monotonically with `d`, so bisection converges without needing the closed form.
		float low = 0.0f;
		float high = target;
		float depth = 0.0f;

		for (int iteration = 0; iteration < 24; iteration++) {
			depth = (low + high) * 0.5f;

			float sagged = 0.0f;
			Vector3 previous = r_points[from];
			for (int i = from + 1; i <= to; i++) {
				const float t = (float)(i - from) / (float)(to - from);
				const Vector3 current_point = r_points[i] + down * (depth * 4.0f * t * (1.0f - t));
				sagged += (float)previous.distance_to(current_point);
				previous = current_point;
			}

			if (sagged < target) {
				low = depth;
			} else {
				high = depth;
			}
		}

		for (int i = from + 1; i < to; i++) {
			const float t = (float)(i - from) / (float)(to - from);
			r_points[i] += down * (depth * 4.0f * t * (1.0f - t));
		}

		// Sampling the parabola at uniform *parameter* bunches points near the bottom and stretches
		// them at the ends, and since rest lengths are taken from this spacing that would leave the
		// rope with segments several times longer than its average -- coarse collision capsules and
		// a visibly chunky rope. Redistribute the span evenly by arc length.
		LocalVector<float> distances;
		distances.resize(to - from + 1);
		distances[0] = 0.0f;
		for (int i = from; i < to; i++) {
			distances[i - from + 1] = distances[i - from] + (float)r_points[i].distance_to(r_points[i + 1]);
		}

		const float total = distances[to - from];
		if (total <= CMP_EPSILON) {
			continue;
		}

		LocalVector<Vector3> source;
		source.resize(to - from + 1);
		for (int i = from; i <= to; i++) {
			source[i - from] = r_points[i];
		}

		int cursor = 0;
		for (int i = from + 1; i < to; i++) {
			const float wanted = total * (float)(i - from) / (float)(to - from);
			while (cursor + 2 < (int)distances.size() && distances[cursor + 1] < wanted) {
				cursor++;
			}
			const float segment = distances[cursor + 1] - distances[cursor];
			const float t = segment > CMP_EPSILON ? (wanted - distances[cursor]) / segment : 0.0f;
			r_points[i] = source[cursor].lerp(source[cursor + 1], t);
		}
	}
}

void RopeBody3D::_rebuild_points() {
	if (!is_backend_supported() || point_count < 2) {
		return;
	}

	_resolve_pins();

	LocalVector<Vector3> points;
	_sample_spawn_polyline(points);

	// Warp the sampled pose so every pinned particle lands exactly on its pin, translating the
	// particles in between by a blend of the neighbouring pins' deltas. One pass, and it handles
	// zero, one, two or many pins uniformly -- which is why there is no separate "fit the curve
	// between the two endpoints" step.
	if (!resolved_pins.is_empty()) {
		LocalVector<Vector3> deltas;
		deltas.resize(resolved_pins.size());
		for (uint32_t i = 0; i < resolved_pins.size(); i++) {
			deltas[i] = resolved_pins[i].position - points[resolved_pins[i].index];
		}

		LocalVector<Vector3> warped;
		warped.resize(points.size());

		uint32_t pin = 0;
		for (int i = 0; i < point_count; i++) {
			while (pin + 1 < resolved_pins.size() && resolved_pins[pin + 1].index <= i) {
				pin++;
			}

			Vector3 delta = deltas[pin];
			if (pin + 1 < resolved_pins.size()) {
				const int from = resolved_pins[pin].index;
				const int to = resolved_pins[pin + 1].index;
				if (i > from && to > from) {
					const float t = (float)(i - from) / (float)(to - from);
					delta = deltas[pin].lerp(deltas[pin + 1], t);
				}
			}

			warped[i] = points[i] + delta;
		}

		points = warped;
	}

	// Now that the pose is final, work out how long the rope actually is.
	float target_length = length;
	switch (length_mode) {
		case LENGTH_FROM_ENDPOINTS: {
			const Vector3 from = resolved_pins.is_empty() ? points[0] : resolved_pins[0].position;
			const Vector3 to = resolved_pins.size() < 2 ? points[point_count - 1] : resolved_pins[resolved_pins.size() - 1].position;
			target_length = (float)from.distance_to(to) + slack;
		} break;

		case LENGTH_FROM_CURVE: {
			float total = 0.0f;
			for (int i = 0; i + 1 < point_count; i++) {
				total += (float)points[i].distance_to(points[i + 1]);
			}
			target_length = total;
		} break;

		case LENGTH_MANUAL:
			break;
	}

	target_length = MAX(target_length, 0.001f);

	if (auto_sag) {
		_apply_sag(points, target_length);
	}

	PhysicsServer3D *ps = PhysicsServer3D::get_singleton();

	Vector<Vector3> server_points;
	server_points.resize(point_count);
	Vector3 *write = server_points.ptrw();
	for (int i = 0; i < point_count; i++) {
		write[i] = points[i];
	}

	ps->rope_set_points(rope, server_points);
	ps->rope_set_param(rope, PhysicsServer3D::ROPE_PARAM_LENGTH, target_length);

	_apply_pins();

	// Deliberately *not* `_mark_mesh_dirty()`. Only the point positions changed here, and those are
	// rewritten from `rope_get_points()` every frame anyway; the mesh's topology depends solely on
	// `point_count`, `render_mode` and `radial_segments`, which mark it dirty themselves. Rebuilding
	// here would re-run `set_mesh()` on every drag tick of a float in the inspector, and that fires
	// `notify_property_list_changed()`, which rebuilds the inspector out from under the spinner you
	// are dragging and drops the grab.
}

void RopeBody3D::_apply_pins() {
	if (!is_backend_supported()) {
		return;
	}

	PhysicsServer3D *ps = PhysicsServer3D::get_singleton();

	// Detach only what has genuinely gone, rather than clearing the lot and rebuilding it.
	//
	// Rebuilding would be simpler, and it is what this did originally, but attachments carry captured
	// state now -- the pose a lock fastened the rope in. A `RopeAttachment3D` parented to a moving
	// body reports a transform change every frame, so a full rebuild would re-capture every lock
	// every frame and the rope would never accumulate any twist at all.
	for (int index : applied_pins) {
		bool still_pinned = false;
		for (const ResolvedPin &pin : resolved_pins) {
			if (pin.index == index) {
				still_pinned = true;
				break;
			}
		}
		if (!still_pinned) {
			ps->rope_detach_point(rope, index);
			ps->rope_pin_point(rope, index, false);
		}
	}

	applied_pins.clear();
	for (const ResolvedPin &pin : resolved_pins) {
		applied_pins.push_back(pin.index);
	}

	for (const ResolvedPin &pin : resolved_pins) {
		bool attached = false;

		if (pin.is_body) {
			PhysicsBody3D *body = ObjectDB::get_instance<PhysicsBody3D>(pin.node_id);
			if (body != nullptr) {
				const Vector3 local_offset = body->get_global_transform().affine_inverse().xform(pin.position);
				ps->rope_attach_point_to_body(rope, pin.index, body->get_rid(), local_offset);
				attached = true;
			}
		}

		if (!attached) {
			ps->rope_pin_point(rope, pin.index, true);
			ps->rope_set_pin_position(rope, pin.index, pin.position);
		}

		// Set after the attachment exists, since the locks live on it. Each is captured against the
		// rope's current pose the first frame it is solved, so applying them here never jerks it.
		ps->rope_set_attachment_flag(rope, pin.index, PhysicsServer3D::ROPE_ATTACHMENT_LOCK_TWIST, pin.lock_twist);
		ps->rope_set_attachment_flag(rope, pin.index, PhysicsServer3D::ROPE_ATTACHMENT_LOCK_DIRECTION, pin.lock_direction);
		ps->rope_set_attachment_flag(rope, pin.index, PhysicsServer3D::ROPE_ATTACHMENT_SOLVER_LIMIT, pin.solver_limit);
		ps->rope_set_attachment_param(rope, pin.index, PhysicsServer3D::ROPE_ATTACHMENT_PARAM_DIRECTION_COMPLIANCE, pin.direction_compliance);
	}
}

void RopeBody3D::_pins_changed() {
	if (!is_inside_tree()) {
		return;
	}

	// Before the first simulated frame the pose is still provisional, so a pin arriving late (scene
	// load order between a rope and the attachments pointing at it is not guaranteed) can still
	// shape it. Afterwards only the pin set is updated, leaving the rope where it is.
	if (simulation_started && !Engine::get_singleton()->is_editor_hint()) {
		_resolve_pins();
		_apply_pins();
	} else {
		_rebuild_points();
	}

	update_gizmos();
}

void RopeBody3D::reset() {
	_rebuild_points();
}

/* -------------------------------------------------------------------------- */
/* Pins                                                                       */
/* -------------------------------------------------------------------------- */

Node3D *RopeBody3D::_get_pin_node(const ResolvedPin &p_pin) const {
	return ObjectDB::get_instance<Node3D>(p_pin.node_id);
}

void RopeBody3D::_resolve_pins() {
	resolved_pins.clear();

	if (point_count < 2) {
		return;
	}

	const int last = point_count - 1;

	struct Entry {
		int index;
		Node3D *node;
		Vector3 offset;
		bool lock_twist;
		bool lock_direction;
		float direction_compliance;
		bool solver_limit;
	};

	LocalVector<Entry> entries;

	auto add_entry = [&](float p_ratio, const NodePath &p_path, const Vector3 &p_offset, bool p_lock_twist, bool p_lock_direction, float p_direction_compliance, bool p_solver_limit) {
		Node3D *node = p_path.is_empty() ? nullptr : Object::cast_to<Node3D>(get_node_or_null(p_path));
		entries.push_back({ (int)Math::round(CLAMP(p_ratio, 0.0f, 1.0f) * (float)last), node, p_offset, p_lock_twist, p_lock_direction, p_direction_compliance, p_solver_limit });
	};

	if (is_inside_tree()) {
		if (!attach_start_path.is_empty()) {
			add_entry(0.0f, attach_start_path, Vector3(), false, false, 0.0f, false);
		}
		if (!attach_end_path.is_empty()) {
			add_entry(1.0f, attach_end_path, Vector3(), false, false, 0.0f, false);
		}
	}

	for (const Pin &pin : pins) {
		add_entry(pin.ratio, pin.node_path, pin.offset, pin.lock_twist, pin.lock_direction, pin.direction_compliance, pin.solver_limit);
	}

	// Attachment nodes come last so that one you positioned in the viewport wins over an array
	// entry that happens to resolve to the same particle.
	for (RopeAttachment3D *attachment : attachments) {
		if (!attachment->is_inside_tree()) {
			continue;
		}

		const int index = (int)Math::round(CLAMP(attachment->get_ratio(), 0.0f, 1.0f) * (float)last);
		const Vector3 world_point = attachment->get_global_transform().origin;

		// The attachment marks *where on the body* the rope ties on, so the body is what the rope
		// actually attaches to and the attachment's position becomes the offset into it. With no
		// body above it, the attachment is just a pin the rope follows.
		const bool lock_twist = attachment->get_lock_twist();
		const bool lock_direction = attachment->get_lock_direction();
		const float direction_compliance = attachment->get_direction_compliance();
		const bool solver_limit = attachment->get_solver_limit();

		if (PhysicsBody3D *body = attachment->get_attached_body()) {
			entries.push_back({ index, body, body->get_global_transform().affine_inverse().xform(world_point),
					lock_twist, lock_direction, direction_compliance, solver_limit });
		} else {
			entries.push_back({ index, attachment, Vector3(), lock_twist, lock_direction, direction_compliance, solver_limit });
		}
	}

	// Later entries overwrite earlier ones at the same particle; the result is sorted by index,
	// which both the warp and the sag rely on.
	for (int index = 0; index <= last; index++) {
		const Entry *winner = nullptr;
		for (const Entry &entry : entries) {
			if (entry.index == index) {
				winner = &entry;
			}
		}
		if (winner == nullptr) {
			continue;
		}

		ResolvedPin pin;
		pin.index = index;
		pin.offset = winner->offset;
		pin.lock_twist = winner->lock_twist;
		pin.lock_direction = winner->lock_direction;
		pin.direction_compliance = winner->direction_compliance;
		pin.solver_limit = winner->solver_limit;

		if (winner->node != nullptr) {
			pin.node_id = winner->node->get_instance_id();
			pin.is_body = Object::cast_to<PhysicsBody3D>(winner->node) != nullptr;
			pin.position = winner->node->get_global_transform().xform(winner->offset);
		} else {
			// No driver node: pinned wherever the spawn pose puts it. Filled in by the caller,
			// which is the only place that knows the pose.
			pin.position = Vector3();
		}

		resolved_pins.push_back(pin);
	}

	// Pins with no driving node hold the particle at its spawn position, which is only known once
	// the pose has been sampled. Sample it here so `_rebuild_points()` can warp against a complete
	// set; the warp delta for these is zero by construction.
	if (!resolved_pins.is_empty()) {
		LocalVector<Vector3> pose;
		_sample_spawn_polyline(pose);
		for (ResolvedPin &pin : resolved_pins) {
			if (!pin.node_id.is_valid()) {
				pin.position = pose[pin.index];
			}
		}
	}
}

void RopeBody3D::_update_kinematic_pins() {
	if (!is_backend_supported()) {
		return;
	}

	PhysicsServer3D *ps = PhysicsServer3D::get_singleton();

	for (ResolvedPin &pin : resolved_pins) {
		if (pin.is_body || !pin.node_id.is_valid()) {
			continue;
		}

		Node3D *node = _get_pin_node(pin);
		if (node == nullptr) {
			continue;
		}

		pin.position = node->get_global_transform().xform(pin.offset);
		ps->rope_set_pin_position(rope, pin.index, pin.position);
	}
}

void RopeBody3D::set_pin_count(int p_count) {
	ERR_FAIL_COND(p_count < 0);

	if ((int)pins.size() == p_count) {
		return;
	}

	pins.resize(p_count);
	notify_property_list_changed();
	_pins_changed();
}

void RopeBody3D::set_pin_ratio(int p_pin, float p_ratio) {
	ERR_FAIL_INDEX(p_pin, (int)pins.size());
	pins[p_pin].ratio = CLAMP(p_ratio, 0.0f, 1.0f);
	_pins_changed();
}

float RopeBody3D::get_pin_ratio(int p_pin) const {
	ERR_FAIL_INDEX_V(p_pin, (int)pins.size(), 0.0f);
	return pins[p_pin].ratio;
}

void RopeBody3D::set_pin_node(int p_pin, const NodePath &p_path) {
	ERR_FAIL_INDEX(p_pin, (int)pins.size());
	pins[p_pin].node_path = p_path;
	_pins_changed();
}

NodePath RopeBody3D::get_pin_node(int p_pin) const {
	ERR_FAIL_INDEX_V(p_pin, (int)pins.size(), NodePath());
	return pins[p_pin].node_path;
}

void RopeBody3D::set_pin_offset(int p_pin, const Vector3 &p_offset) {
	ERR_FAIL_INDEX(p_pin, (int)pins.size());
	pins[p_pin].offset = p_offset;
	_pins_changed();
}

Vector3 RopeBody3D::get_pin_offset(int p_pin) const {
	ERR_FAIL_INDEX_V(p_pin, (int)pins.size(), Vector3());
	return pins[p_pin].offset;
}

void RopeBody3D::set_pin_lock_twist(int p_pin, bool p_enabled) {
	ERR_FAIL_INDEX(p_pin, (int)pins.size());
	pins[p_pin].lock_twist = p_enabled;
	_pins_changed();
}

bool RopeBody3D::get_pin_lock_twist(int p_pin) const {
	ERR_FAIL_INDEX_V(p_pin, (int)pins.size(), false);
	return pins[p_pin].lock_twist;
}

void RopeBody3D::set_pin_lock_direction(int p_pin, bool p_enabled) {
	ERR_FAIL_INDEX(p_pin, (int)pins.size());
	pins[p_pin].lock_direction = p_enabled;
	_pins_changed();
}

bool RopeBody3D::get_pin_lock_direction(int p_pin) const {
	ERR_FAIL_INDEX_V(p_pin, (int)pins.size(), false);
	return pins[p_pin].lock_direction;
}

void RopeBody3D::set_pin_direction_compliance(int p_pin, float p_compliance) {
	ERR_FAIL_INDEX(p_pin, (int)pins.size());
	pins[p_pin].direction_compliance = MAX(p_compliance, 0.0f);
	_pins_changed();
}

float RopeBody3D::get_pin_direction_compliance(int p_pin) const {
	ERR_FAIL_INDEX_V(p_pin, (int)pins.size(), 0.0f);
	return pins[p_pin].direction_compliance;
}

void RopeBody3D::set_pin_solver_limit(int p_pin, bool p_enabled) {
	ERR_FAIL_INDEX(p_pin, (int)pins.size());
	pins[p_pin].solver_limit = p_enabled;
	_pins_changed();
}

bool RopeBody3D::get_pin_solver_limit(int p_pin) const {
	ERR_FAIL_INDEX_V(p_pin, (int)pins.size(), false);
	return pins[p_pin].solver_limit;
}

int RopeBody3D::find_pin_at_index(int p_point_index) const {
	if (point_count < 2) {
		return -1;
	}

	const int last = point_count - 1;
	for (uint32_t i = 0; i < pins.size(); i++) {
		if ((int)Math::round(CLAMP(pins[i].ratio, 0.0f, 1.0f) * (float)last) == p_point_index) {
			return (int)i;
		}
	}
	return -1;
}

int RopeBody3D::toggle_pin_at_ratio(float p_ratio) {
	ERR_FAIL_COND_V(point_count < 2, (int)pins.size());

	const int last = point_count - 1;
	const int index = (int)Math::round(CLAMP(p_ratio, 0.0f, 1.0f) * (float)last);

	const int existing = find_pin_at_index(index);
	if (existing >= 0) {
		pins.remove_at(existing);
	} else {
		Pin pin;
		pin.ratio = (float)index / (float)last;
		pins.push_back(pin);
	}

	notify_property_list_changed();
	_pins_changed();

	return (int)pins.size();
}

void RopeBody3D::register_attachment(RopeAttachment3D *p_attachment) {
	if (attachments.find(p_attachment) == -1) {
		attachments.push_back(p_attachment);
	}
	_pins_changed();
}

void RopeBody3D::unregister_attachment(RopeAttachment3D *p_attachment) {
	attachments.erase(p_attachment);
	_pins_changed();
}

void RopeBody3D::attachment_changed() {
	_pins_changed();
}


/* -------------------------------------------------------------------------- */
/* Runtime API                                                                */
/* -------------------------------------------------------------------------- */

void RopeBody3D::attach_point(int p_index, Node *p_body) {
	ERR_FAIL_COND(!is_backend_supported());
	ERR_FAIL_INDEX(p_index, point_count);

	PhysicsServer3D *ps = PhysicsServer3D::get_singleton();

	PhysicsBody3D *body = Object::cast_to<PhysicsBody3D>(p_body);
	ERR_FAIL_NULL_MSG(body, "RopeBody3D::attach_point() requires a PhysicsBody3D.");

	const Vector3 world_point = ps->rope_get_point_position(rope, p_index);
	const Vector3 local_offset = body->get_global_transform().affine_inverse().xform(world_point);

	ps->rope_attach_point_to_body(rope, p_index, body->get_rid(), local_offset);
}

void RopeBody3D::detach_point(int p_index) {
	ERR_FAIL_COND(!is_backend_supported());
	ERR_FAIL_INDEX(p_index, point_count);

	PhysicsServer3D::get_singleton()->rope_detach_point(rope, p_index);
}

void RopeBody3D::pin_point(int p_index, bool p_pin) {
	ERR_FAIL_COND(!is_backend_supported());
	ERR_FAIL_INDEX(p_index, point_count);

	PhysicsServer3D::get_singleton()->rope_pin_point(rope, p_index, p_pin);
}

bool RopeBody3D::is_point_pinned(int p_index) const {
	ERR_FAIL_COND_V(!is_backend_supported(), false);
	ERR_FAIL_INDEX_V(p_index, point_count, false);

	return PhysicsServer3D::get_singleton()->rope_is_point_pinned(rope, p_index);
}

void RopeBody3D::set_pin_position(int p_index, const Vector3 &p_position) {
	ERR_FAIL_COND(!is_backend_supported());
	ERR_FAIL_INDEX(p_index, point_count);

	PhysicsServer3D::get_singleton()->rope_set_pin_position(rope, p_index, p_position);
}

void RopeBody3D::apply_point_impulse(int p_index, const Vector3 &p_impulse) {
	ERR_FAIL_COND(!is_backend_supported());
	ERR_FAIL_INDEX(p_index, point_count);

	PhysicsServer3D::get_singleton()->rope_apply_point_impulse(rope, p_index, p_impulse);
}

void RopeBody3D::apply_central_impulse(const Vector3 &p_impulse) {
	ERR_FAIL_COND(!is_backend_supported());

	PhysicsServer3D::get_singleton()->rope_apply_central_impulse(rope, p_impulse);
}

// Shared by the three query helpers below: the segment index and the position along it that come
// nearest `p_world_point`. Returns a fractional particle index, which every one of them wants.
static float rope_closest_findex(const PackedVector3Array &p_points, const Vector3 &p_world_point) {
	const int count = p_points.size();
	if (count < 2) {
		return 0.0f;
	}

	float best_findex = 0.0f;
	real_t best_distance = -1.0;

	for (int i = 0; i + 1 < count; i++) {
		const Vector3 &from = p_points[i];
		const Vector3 &to = p_points[i + 1];

		const Vector3 segment = to - from;
		const real_t length_squared = segment.length_squared();

		real_t t = 0.0;
		if (length_squared > (real_t)CMP_EPSILON) {
			t = CLAMP((p_world_point - from).dot(segment) / length_squared, (real_t)0.0, (real_t)1.0);
		}

		const real_t distance = p_world_point.distance_squared_to(from + segment * t);
		if (best_distance < 0.0 || distance < best_distance) {
			best_distance = distance;
			best_findex = (float)i + (float)t;
		}
	}

	return best_findex;
}

/* -------------------------------------------------------------------------- */
/* Area overlap                                                               */
/* -------------------------------------------------------------------------- */

LocalVector<RopeBody3D *> RopeBody3D::all_ropes;

namespace {

// Most points sit inside at most a handful of areas at once; anything deeper than this and the
// answer for one more overlapping area is not worth the query time.
constexpr int ROPE_AREA_QUERY_RESULTS = 32;

// A capsule the size of one rope segment, which is exactly the shape the rope collides with. Created
// once per sweep and resized per segment rather than per rope, since a sweep may cover many.
struct RopeAreaProbe {
	RID shape;
	PhysicsDirectSpaceState3D::ShapeParameters params;

	RopeAreaProbe(const Area3D *p_area) {
		shape = PhysicsServer3D::get_singleton()->capsule_shape_create();

		params.shape_rid = shape;
		params.collide_with_bodies = false;
		params.collide_with_areas = true;

		// The *area's own layer*, so the broadphase discards everything else before any narrow-phase
		// work happens -- without it each segment would be tested against every area in the level. An
		// area with no layer bits set would then match nothing, which is never what the caller meant
		// when they passed that area in explicitly, so it falls back to matching everything.
		const uint32_t layer = p_area->get_collision_layer();
		params.collision_mask = layer != 0 ? layer : UINT32_MAX;
	}

	~RopeAreaProbe() {
		if (shape.is_valid()) {
			PhysicsServer3D::get_singleton()->free_rid(shape);
		}
	}
};

// Orthonormal basis with its Y axis along `p_axis`, which is the axis a Godot capsule stands on.
Basis rope_capsule_basis(const Vector3 &p_axis) {
	const Vector3 reference = Math::abs(p_axis.y) > 0.9 ? Vector3(1, 0, 0) : Vector3(0, 1, 0);
	const Vector3 x = reference.cross(p_axis).normalized();

	Basis basis;
	basis.set_columns(x, p_axis, p_axis.cross(x));
	return basis;
}

// Segments of `p_points` that overlap `p_area`, tested as the capsules the rope actually collides
// with rather than as bare points.
//
// Testing the particles alone is only as accurate as the rope's resolution: a rope lying across a
// doorway-sized trigger with its particles either side of it reads as completely outside, and so does
// any rope thicker than the area it is passing through. Sweeping the segment volume is the same test
// the rope's own collision uses, so the answer agrees with what the rope visibly does.
//
// `p_first_only` stops at the first hit, which is all a plain membership test needs and usually saves
// most of the queries.
LocalVector<int> rope_segments_in_area(PhysicsDirectSpaceState3D *p_state, const PackedVector3Array &p_points, float p_radius, ObjectID p_area_id, RopeAreaProbe &p_probe, bool p_first_only) {
	LocalVector<int> inside;

	PhysicsDirectSpaceState3D::ShapeResult results[ROPE_AREA_QUERY_RESULTS];
	const int count = p_points.size();
	const real_t radius = MAX((real_t)p_radius, (real_t)0.001);

	Dictionary shape_data;
	shape_data["radius"] = radius;

	for (int i = 0; i + 1 < count; i++) {
		const Vector3 from = p_points[i];
		const Vector3 to = p_points[i + 1];

		const Vector3 segment = to - from;
		const real_t length = segment.length();
		if (length < (real_t)CMP_EPSILON) {
			continue;
		}

		// A Godot capsule's height spans the whole shape, hemispherical caps included, so the
		// cylindrical part is the segment and the caps are the rope's own round ends.
		shape_data["height"] = length + radius * 2.0;
		PhysicsServer3D::get_singleton()->shape_set_data(p_probe.shape, shape_data);

		p_probe.params.transform = Transform3D(rope_capsule_basis(segment / length), (from + to) * 0.5);

		const int hits = p_state->intersect_shape(p_probe.params, results, ROPE_AREA_QUERY_RESULTS);
		for (int h = 0; h < hits; h++) {
			if (results[h].collider_id != p_area_id) {
				continue;
			}

			inside.push_back(i);
			break;
		}

		if (p_first_only && !inside.is_empty()) {
			break;
		}
	}

	return inside;
}

} // namespace

PackedFloat32Array RopeBody3D::get_ratios_in_area(Area3D *p_area) const {
	PackedFloat32Array ratios;

	ERR_FAIL_NULL_V(p_area, ratios);
	if (!is_backend_supported() || !is_inside_tree() || !p_area->is_inside_tree()) {
		return ratios;
	}
	// Comparing worlds rather than spaces: the query runs against the area's world, and a rope in a
	// different one can never overlap it however close the coordinates look.
	if (p_area->get_world_3d() != get_world_3d()) {
		return ratios;
	}

	PhysicsDirectSpaceState3D *state = p_area->get_world_3d()->get_direct_space_state();
	if (state == nullptr) {
		return ratios;
	}

	const PackedVector3Array points = PhysicsServer3D::get_singleton()->rope_get_points(rope);
	const int count = points.size();
	if (count < 2) {
		return ratios;
	}

	RopeAreaProbe probe(p_area);
	const LocalVector<int> inside = rope_segments_in_area(state, points, radius, p_area->get_instance_id(), probe, false);

	// Both ends of every overlapping segment, in order and without repeats. Reporting the segment's
	// endpoints rather than only the particles that happen to test inside is what makes a rope merely
	// *crossing* an area report where it crosses instead of reporting nothing at all.
	const float last = (float)(count - 1);
	int previous = -1;
	for (uint32_t i = 0; i < inside.size(); i++) {
		const int segment = inside[i];
		if (segment != previous) {
			ratios.push_back((float)segment / last);
		}
		ratios.push_back((float)(segment + 1) / last);
		previous = segment + 1;
	}

	return ratios;
}

bool RopeBody3D::is_in_area(Area3D *p_area) const {
	ERR_FAIL_NULL_V(p_area, false);
	if (!is_backend_supported() || !is_inside_tree() || !p_area->is_inside_tree()) {
		return false;
	}
	if (p_area->get_world_3d() != get_world_3d()) {
		return false;
	}

	PhysicsDirectSpaceState3D *state = p_area->get_world_3d()->get_direct_space_state();
	if (state == nullptr) {
		return false;
	}

	const PackedVector3Array points = PhysicsServer3D::get_singleton()->rope_get_points(rope);
	if (points.size() < 2) {
		return false;
	}

	RopeAreaProbe probe(p_area);
	return !rope_segments_in_area(state, points, radius, p_area->get_instance_id(), probe, true).is_empty();
}

TypedArray<RopeBody3D> RopeBody3D::get_ropes_in_area(Area3D *p_area) {
	TypedArray<RopeBody3D> found;

	ERR_FAIL_NULL_V(p_area, found);
	if (!p_area->is_inside_tree()) {
		return found;
	}

	const Ref<World3D> world = p_area->get_world_3d();
	PhysicsDirectSpaceState3D *state = world.is_valid() ? world->get_direct_space_state() : nullptr;
	if (state == nullptr) {
		return found;
	}

	RopeAreaProbe probe(p_area);
	const ObjectID area_id = p_area->get_instance_id();

	for (RopeBody3D *candidate : all_ropes) {
		if (!candidate->is_backend_supported() || candidate->get_world_3d() != world) {
			continue;
		}

		const PackedVector3Array points = PhysicsServer3D::get_singleton()->rope_get_points(candidate->rope);
		if (points.size() < 2) {
			continue;
		}

		if (!rope_segments_in_area(state, points, candidate->radius, area_id, probe, true).is_empty()) {
			found.push_back(candidate);
		}
	}

	return found;
}

float RopeBody3D::get_closest_ratio(const Vector3 &p_world_point) const {
	const PackedVector3Array points = get_points();
	if (points.size() < 2) {
		return 0.0f;
	}

	return CLAMP(rope_closest_findex(points, p_world_point) / (float)(points.size() - 1), 0.0f, 1.0f);
}

Vector3 RopeBody3D::get_closest_point(const Vector3 &p_world_point) const {
	const PackedVector3Array points = get_points();
	if (points.is_empty()) {
		return Vector3();
	}
	if (points.size() < 2) {
		return points[0];
	}

	const float findex = rope_closest_findex(points, p_world_point);
	const int index = CLAMP((int)Math::floor(findex), 0, points.size() - 2);
	return points[index].lerp(points[index + 1], findex - (float)index);
}

Vector3 RopeBody3D::sample_ratio(float p_ratio) const {
	const PackedVector3Array points = get_points();
	if (points.is_empty()) {
		return Vector3();
	}
	if (points.size() < 2) {
		return points[0];
	}

	const float findex = CLAMP(p_ratio, 0.0f, 1.0f) * (float)(points.size() - 1);
	const int index = CLAMP((int)Math::floor(findex), 0, points.size() - 2);
	return points[index].lerp(points[index + 1], findex - (float)index);
}

Vector3 RopeBody3D::get_point_position(int p_index) const {
	ERR_FAIL_COND_V(!is_backend_supported(), Vector3());
	ERR_FAIL_INDEX_V(p_index, point_count, Vector3());

	return PhysicsServer3D::get_singleton()->rope_get_point_position(rope, p_index);
}

Vector3 RopeBody3D::get_point_velocity(int p_index) const {
	ERR_FAIL_COND_V(!is_backend_supported(), Vector3());
	ERR_FAIL_INDEX_V(p_index, point_count, Vector3());

	return PhysicsServer3D::get_singleton()->rope_get_point_velocity(rope, p_index);
}

PackedVector3Array RopeBody3D::get_points() const {
	ERR_FAIL_COND_V(!is_backend_supported(), PackedVector3Array());

	return PhysicsServer3D::get_singleton()->rope_get_points(rope);
}

float RopeBody3D::get_simulated_length() const {
	const PackedVector3Array points = get_points();

	float total = 0.0f;
	for (int i = 0; i + 1 < points.size(); i++) {
		total += (float)points[i].distance_to(points[i + 1]);
	}
	return total;
}

float RopeBody3D::get_segment_length() const {
	return point_count > 1 ? length / (float)(point_count - 1) : length;
}

/* -------------------------------------------------------------------------- */
/* Rendering                                                                  */
/* -------------------------------------------------------------------------- */

void RopeBody3D::_mark_mesh_dirty() {
	mesh_dirty = true;
}

Ref<ShaderMaterial> RopeBody3D::_get_ribbon_material() {
	if (ribbon_material.is_valid()) {
		return ribbon_material;
	}

	Ref<Shader> shader;
	shader.instantiate();
	shader->set_code(ROPE_RIBBON_SHADER_CODE);

	ribbon_material.instantiate();
	ribbon_material->set_shader(shader);
	ribbon_material->set_shader_parameter("rope_radius", radius);

	return ribbon_material;
}

void RopeBody3D::_rebuild_mesh() {
	mesh_dirty = false;

	vertex_buffer.clear();
	mesh_vertex_count = 0;
	mesh_ring_stride = 0;

	if (render_mode == RENDER_NONE || point_count < 2) {
		rope_mesh.unref();
		if (get_mesh().is_valid()) {
			set_mesh(Ref<Mesh>());
		}
		return;
	}

	// A tube duplicates the seam column so the UV wrap is not smeared across one quad; a ribbon has
	// exactly two vertices per ring.
	const int ring_stride = (render_mode == RENDER_TUBE) ? (radial_segments + 1) : 2;
	const int vertex_count = point_count * ring_stride;

	PackedVector3Array vertices;
	PackedVector3Array normals;
	PackedVector2Array uvs;
	PackedInt32Array indices;

	vertices.resize(vertex_count);
	normals.resize(vertex_count);
	uvs.resize(vertex_count);

	Vector3 *vertices_write = vertices.ptrw();
	Vector3 *normals_write = normals.ptrw();
	Vector2 *uvs_write = uvs.ptrw();

	const float placeholder_step = get_segment_length();

	// Placeholder geometry. Real positions arrive on the first `_update_mesh()`; only the UVs and
	// the index buffer built here are permanent, since the per-frame update rewrites the vertex
	// buffer only.
	for (int ring = 0; ring < point_count; ring++) {
		const float v = point_count > 1 ? (float)ring / (float)(point_count - 1) : 0.0f;

		for (int j = 0; j < ring_stride; j++) {
			const int index = ring * ring_stride + j;
			const float u = (ring_stride > 1) ? (float)j / (float)(ring_stride - 1) : 0.0f;

			vertices_write[index] = Vector3(0, (real_t)(placeholder_step * ring), 0);
			normals_write[index] = Vector3(0, 0, 1);
			uvs_write[index] = Vector2(u, v);
		}
	}

	for (int ring = 0; ring + 1 < point_count; ring++) {
		for (int j = 0; j + 1 < ring_stride; j++) {
			const int a = ring * ring_stride + j;
			const int b = a + 1;
			const int c = a + ring_stride;
			const int d = c + 1;

			indices.push_back(a);
			indices.push_back(c);
			indices.push_back(b);

			indices.push_back(b);
			indices.push_back(c);
			indices.push_back(d);
		}
	}

	Array arrays;
	arrays.resize(Mesh::ARRAY_MAX);
	arrays[Mesh::ARRAY_VERTEX] = vertices;
	arrays[Mesh::ARRAY_NORMAL] = normals;
	arrays[Mesh::ARRAY_TEX_UV] = uvs;
	arrays[Mesh::ARRAY_INDEX] = indices;

	// Reuse the existing mesh rather than instantiating a fresh one. `MeshInstance3D::set_mesh()`
	// calls `notify_property_list_changed()` (it has to, for blend shapes and surface overrides),
	// which tears down and rebuilds the inspector -- and that cancels an in-progress drag on any
	// property that got us here, such as `point_count` or `radial_segments`. Keeping the same
	// resource means `set_mesh()` only ever runs once.
	if (rope_mesh.is_valid()) {
		rope_mesh->clear_surfaces();
	} else {
		rope_mesh.instantiate();
	}

	// `ARRAY_FLAG_USE_DYNAMIC_UPDATE` is what makes `mesh_surface_update_vertex_region()` legal on
	// this surface; attribute compression is left off so the vertex buffer layout stays the plain
	// one the per-frame update writes.
	rope_mesh->add_surface_from_arrays(Mesh::PRIMITIVE_TRIANGLES, arrays, TypedArray<Array>(), Dictionary(), Mesh::ARRAY_FLAG_USE_DYNAMIC_UPDATE);

	if (get_mesh() != rope_mesh) {
		set_mesh(rope_mesh);
	}

	if (render_mode == RENDER_RIBBON) {
		set_surface_override_material(0, _get_ribbon_material());
	} else {
		set_surface_override_material(0, Ref<Material>());
	}

	// Cache the vertex-buffer layout so the per-frame update never has to query it again.
	const RID mesh_rid = rope_mesh->get_rid();
	RenderingServerTypes::SurfaceData surface_data = RS::get_singleton()->mesh_get_surface(mesh_rid, 0);

	uint32_t surface_offsets[RSE::ARRAY_MAX];
	uint32_t attrib_stride;
	uint32_t skin_stride;
	RS::get_singleton()->mesh_surface_make_offsets_from_format(surface_data.format, surface_data.vertex_count, surface_data.index_count, surface_offsets, vertex_stride, normal_stride, attrib_stride, skin_stride);

	vertex_buffer = surface_data.vertex_data;
	offset_vertices = surface_offsets[RSE::ARRAY_VERTEX];
	offset_normal = surface_offsets[RSE::ARRAY_NORMAL];
	mesh_vertex_count = vertex_count;
	mesh_ring_stride = ring_stride;
}

// Tangents from the sampled points, and the material frame from the solver.
//
// The frame has to come from there. Deriving a perpendicular here is stable *along* the rope -- which
// is what the previous version did, and did correctly -- but not stable *over time*: the seed was
// rebuilt every frame from an arbitrary reference axis, so the whole tube's surface rolled about its
// own axis as the rope swung. It was also seeded through a `|tangent.y| > 0.99` branch whose two
// sides disagree by roughly a quarter turn for a rope tilting out of the plane the branch happens to
// favour, and a rope hanging from a hook crosses that threshold twice per swing. The solver instead
// carries one frame across time and parallel-transports the rest along the rope, which is coherent in
// both directions.
//
// The fallback below is the old spatial propagation, and it is reached only when the solver has no
// frame to give -- in the editor, where the rope does not simulate and a static preview has nothing
// to be temporally coherent about.
void RopeBody3D::_update_frames(const Basis &p_to_local) {
	const int count = point_cache.size();

	tangent_cache.resize(count);
	normal_cache.resize(count);
	binormal_cache.resize(count);

	if (count < 2) {
		return;
	}

	for (int i = 0; i < count; i++) {
		Vector3 tangent;
		if (i == 0) {
			tangent = point_cache[1] - point_cache[0];
		} else if (i == count - 1) {
			tangent = point_cache[count - 1] - point_cache[count - 2];
		} else {
			tangent = point_cache[i + 1] - point_cache[i - 1];
		}

		if (tangent.length_squared() < CMP_EPSILON) {
			tangent = Vector3(0, 1, 0);
		}
		tangent_cache[i] = tangent.normalized();
	}

	const PackedVector3Array normals = is_backend_supported()
			? PhysicsServer3D::get_singleton()->rope_get_point_normals(rope)
			: PackedVector3Array();

	if (normals.size() == count) {
		const Vector3 *normals_read = normals.ptr();

		for (int i = 0; i < count; i++) {
			const Vector3 &tangent = tangent_cache[i];

			// The solver's tangents are per segment and these are per particle, so the two disagree
			// slightly at a bend; re-perpendicularising is what reconciles them.
			Vector3 normal = p_to_local.xform(normals_read[i]);
			normal -= tangent * normal.dot(tangent);

			if (normal.length_squared() < CMP_EPSILON) {
				const Vector3 reference = Math::abs(tangent.y) > 0.9 ? Vector3(1, 0, 0) : Vector3(0, 1, 0);
				normal = reference.cross(tangent);
			}

			normal_cache[i] = normal.normalized();
			binormal_cache[i] = tangent.cross(normal_cache[i]);
		}

		return;
	}

	const Vector3 &first_tangent = tangent_cache[0];
	Vector3 reference = Math::abs(first_tangent.y) > 0.9 ? Vector3(1, 0, 0) : Vector3(0, 1, 0);
	normal_cache[0] = reference.cross(first_tangent).normalized();
	binormal_cache[0] = first_tangent.cross(normal_cache[0]);

	for (int i = 1; i < count; i++) {
		const Vector3 &tangent = tangent_cache[i];

		Vector3 normal = normal_cache[i - 1] - tangent * normal_cache[i - 1].dot(tangent);

		if (normal.length_squared() < CMP_EPSILON) {
			reference = Math::abs(tangent.y) > 0.9 ? Vector3(1, 0, 0) : Vector3(0, 1, 0);
			normal = reference.cross(tangent);
		}

		normal_cache[i] = normal.normalized();
		binormal_cache[i] = tangent.cross(normal_cache[i]);
	}
}

void RopeBody3D::_update_mesh() {
	if (!is_backend_supported() || !is_inside_tree()) {
		return;
	}

	if (mesh_dirty) {
		_rebuild_mesh();
	}

	if (render_mode == RENDER_NONE || mesh_vertex_count == 0 || rope_mesh.is_null()) {
		return;
	}

	// With physics interpolation on, every other object in the scene is drawn somewhere between the
	// last two physics steps, while a mesh built from vertex data is drawn at the current one -- which
	// puts the rope up to a whole step *ahead* of everything it is tied to, by an amount that changes
	// every rendered frame. That reads as jitter rather than as lag, and it is why a rope tied to a
	// tracked hand never quite sits on it. Vertex data is not something the renderer can interpolate
	// for us, so the rope has to be asked for the pose at the same instant everything else is showing.
	PhysicsServer3D *physics = PhysicsServer3D::get_singleton();
	const PackedVector3Array points = is_physics_interpolated_and_enabled()
			? physics->rope_get_points_interpolated(rope, (float)Engine::get_singleton()->get_physics_interpolation_fraction())
			: physics->rope_get_points(rope);

	if (points.size() != point_count) {
		// Topology changed underneath us; rebuild rather than write out of bounds.
		_mark_mesh_dirty();
		return;
	}

	// The simulation is world-space but the node keeps its own transform, so the mesh has to be
	// written in the node's local space or it would be transformed twice. This is what used to make
	// the rope draw at double its offset in the editor, where the node is not forced to identity.
	const Transform3D to_local = get_global_transform().affine_inverse();

	point_cache.resize(point_count);
	const Vector3 *points_read = points.ptr();
	for (int i = 0; i < point_count; i++) {
		point_cache[i] = to_local.xform(points_read[i]);
	}

	_update_frames(to_local.basis);

	uint8_t *write_buffer = vertex_buffer.ptrw();

	for (int ring = 0; ring < point_count; ring++) {
		const Vector3 &centre = point_cache[ring];

		for (int j = 0; j < mesh_ring_stride; j++) {
			const int index = ring * mesh_ring_stride + j;

			Vector3 position;
			Vector3 normal;

			if (render_mode == RENDER_TUBE) {
				const float angle = Math::TAU * (float)j / (float)radial_segments;
				normal = normal_cache[ring] * Math::cos(angle) + binormal_cache[ring] * Math::sin(angle);
				position = centre + normal * radius;
			} else {
				// Ribbon: both vertices sit on the centreline and carry the tangent; the shader
				// expands them sideways per eye.
				position = centre;
				normal = tangent_cache[ring];
			}

			float *vertex_write = reinterpret_cast<float *>(write_buffer + index * vertex_stride + offset_vertices);
			vertex_write[0] = (float)position.x;
			vertex_write[1] = (float)position.y;
			vertex_write[2] = (float)position.z;

			const Vector2 encoded = normal.octahedron_encode();
			uint32_t value = 0;
			value |= (uint16_t)CLAMP(encoded.x * 65535, 0, 65535);
			value |= (uint32_t)((uint16_t)CLAMP(encoded.y * 65535, 0, 65535)) << 16;
			memcpy(&write_buffer[index * normal_stride + offset_normal], &value, sizeof(uint32_t));
		}
	}

	const RID mesh_rid = rope_mesh->get_rid();
	RS::get_singleton()->mesh_surface_update_vertex_region(mesh_rid, 0, 0, vertex_buffer);

	// The simulated geometry has nothing to do with the authored mesh bounds, so the AABB has to be
	// supplied explicitly -- in local space, to match the vertices -- or the rope will be culled as
	// soon as it swings away from its origin.
	AABB aabb = to_local.xform(PhysicsServer3D::get_singleton()->rope_get_bounds(rope));
	if (render_mode == RENDER_RIBBON) {
		// The shader pushes vertices up to `radius` sideways, past the simulated bounds.
		aabb.grow_by(radius);
	}
	RS::get_singleton()->mesh_set_custom_aabb(mesh_rid, aabb);
}

/* -------------------------------------------------------------------------- */
/* Property setters                                                           */
/* -------------------------------------------------------------------------- */

void RopeBody3D::set_point_count(int p_count) {
	const int clamped = CLAMP(p_count, 2, 512);
	if (point_count == clamped) {
		return;
	}

	point_count = clamped;
	_mark_mesh_dirty();
	_queue_rebuild();
	update_gizmos();
}

void RopeBody3D::set_length(float p_length) {
	const float clamped = MAX(p_length, 0.001f);
	if (Math::is_equal_approx(length, clamped)) {
		return;
	}

	length = clamped;
	_reseed_curve();
	_queue_rebuild();
	update_gizmos();
}

void RopeBody3D::set_length_mode(LengthMode p_mode) {
	if (length_mode == p_mode) {
		return;
	}

	length_mode = p_mode;
	notify_property_list_changed();
	_queue_rebuild();
}

void RopeBody3D::set_slack(float p_slack) {
	slack = MAX(p_slack, 0.0f);
	_queue_rebuild();
}

void RopeBody3D::set_auto_sag(bool p_enabled) {
	auto_sag = p_enabled;
	_queue_rebuild();
}

void RopeBody3D::set_spawn_curve(const Ref<Curve3D> &p_curve) {
	if (spawn_curve == p_curve) {
		return;
	}

	if (spawn_curve.is_valid()) {
		spawn_curve->disconnect_changed(callable_mp(this, &RopeBody3D::_curve_changed));
	}

	spawn_curve = p_curve;

	if (spawn_curve.is_valid()) {
		spawn_curve->connect_changed(callable_mp(this, &RopeBody3D::_curve_changed));
		// A freshly instantiated (empty) curve gets seeded; anything with a shape is left alone.
		_reseed_curve();
	}

	_queue_rebuild();
	update_gizmos();
}

void RopeBody3D::set_initial_direction(const Vector3 &p_direction) {
	initial_direction = p_direction;
	_reseed_curve();
	_queue_rebuild();
	update_gizmos();
}

void RopeBody3D::set_radius(float p_radius) {
	radius = MAX(p_radius, 0.001f);

	if (is_backend_supported()) {
		PhysicsServer3D::get_singleton()->rope_set_param(rope, PhysicsServer3D::ROPE_PARAM_RADIUS, radius);
	}
	if (ribbon_material.is_valid()) {
		ribbon_material->set_shader_parameter("rope_radius", radius);
	}
	update_gizmos();
}

void RopeBody3D::set_total_mass(float p_mass) {
	total_mass = MAX(p_mass, 0.0001f);

	if (is_backend_supported()) {
		PhysicsServer3D::get_singleton()->rope_set_param(rope, PhysicsServer3D::ROPE_PARAM_TOTAL_MASS, total_mass);
	}
}

void RopeBody3D::set_simulation_substeps(int p_substeps) {
	substeps = CLAMP(p_substeps, 1, 32);

	if (is_backend_supported()) {
		PhysicsServer3D::get_singleton()->rope_set_simulation_substeps(rope, substeps);
	}
}

void RopeBody3D::set_stretch_compliance(float p_compliance) {
	stretch_compliance = MAX(p_compliance, 0.0f);

	if (is_backend_supported()) {
		PhysicsServer3D::get_singleton()->rope_set_param(rope, PhysicsServer3D::ROPE_PARAM_STRETCH_COMPLIANCE, stretch_compliance);
	}
}

void RopeBody3D::set_bend_compliance(float p_compliance) {
	bend_compliance = CLAMP(p_compliance, 0.0f, ROPE_BEND_DISABLED);

	if (is_backend_supported()) {
		PhysicsServer3D::get_singleton()->rope_set_param(rope, PhysicsServer3D::ROPE_PARAM_BEND_COMPLIANCE, bend_compliance);
	}
}

void RopeBody3D::set_twist_compliance(float p_compliance) {
	twist_compliance = CLAMP(p_compliance, 0.0f, ROPE_TWIST_DISABLED);

	if (is_backend_supported()) {
		PhysicsServer3D::get_singleton()->rope_set_param(rope, PhysicsServer3D::ROPE_PARAM_TWIST_COMPLIANCE, twist_compliance);
	}
}

void RopeBody3D::set_twist_damping(float p_damping) {
	twist_damping = MAX(p_damping, 0.0f);

	if (is_backend_supported()) {
		PhysicsServer3D::get_singleton()->rope_set_param(rope, PhysicsServer3D::ROPE_PARAM_TWIST_DAMPING, twist_damping);
	}
}

void RopeBody3D::set_linear_damping(float p_damping) {
	linear_damping = MAX(p_damping, 0.0f);

	if (is_backend_supported()) {
		PhysicsServer3D::get_singleton()->rope_set_param(rope, PhysicsServer3D::ROPE_PARAM_LINEAR_DAMPING, linear_damping);
	}
}

void RopeBody3D::set_drag(float p_drag) {
	drag = MAX(p_drag, 0.0f);

	if (is_backend_supported()) {
		PhysicsServer3D::get_singleton()->rope_set_param(rope, PhysicsServer3D::ROPE_PARAM_DRAG, drag);
	}
}

void RopeBody3D::set_gravity_scale(float p_scale) {
	gravity_scale = p_scale;

	if (is_backend_supported()) {
		PhysicsServer3D::get_singleton()->rope_set_param(rope, PhysicsServer3D::ROPE_PARAM_GRAVITY_SCALE, gravity_scale);
	}
}

void RopeBody3D::set_friction(float p_friction) {
	friction = CLAMP(p_friction, 0.0f, 1.0f);

	if (is_backend_supported()) {
		PhysicsServer3D::get_singleton()->rope_set_param(rope, PhysicsServer3D::ROPE_PARAM_FRICTION, friction);
	}
}

void RopeBody3D::set_restitution(float p_restitution) {
	restitution = CLAMP(p_restitution, 0.0f, 1.0f);

	if (is_backend_supported()) {
		PhysicsServer3D::get_singleton()->rope_set_param(rope, PhysicsServer3D::ROPE_PARAM_RESTITUTION, restitution);
	}
}

void RopeBody3D::set_max_reaction_impulse(float p_impulse) {
	max_reaction_impulse = MAX(p_impulse, 0.0f);

	if (is_backend_supported()) {
		PhysicsServer3D::get_singleton()->rope_set_param(rope, PhysicsServer3D::ROPE_PARAM_MAX_REACTION_IMPULSE, max_reaction_impulse);
	}
}

void RopeBody3D::set_inextensible(bool p_enabled) {
	inextensible = p_enabled;

	if (is_backend_supported()) {
		PhysicsServer3D::get_singleton()->rope_set_flag(rope, PhysicsServer3D::ROPE_FLAG_INEXTENSIBLE, inextensible);
	}
}

void RopeBody3D::set_two_way_coupling(bool p_enabled) {
	two_way_coupling = p_enabled;

	if (is_backend_supported()) {
		PhysicsServer3D::get_singleton()->rope_set_flag(rope, PhysicsServer3D::ROPE_FLAG_TWO_WAY_COUPLING, two_way_coupling);
	}
	notify_property_list_changed();
}

void RopeBody3D::set_collision_enabled(bool p_enabled) {
	collision_enabled = p_enabled;

	if (is_backend_supported()) {
		PhysicsServer3D::get_singleton()->rope_set_flag(rope, PhysicsServer3D::ROPE_FLAG_COLLISION_ENABLED, collision_enabled);
	}
}

void RopeBody3D::set_collision_layer(uint32_t p_layer) {
	collision_layer = p_layer;

	if (is_backend_supported()) {
		PhysicsServer3D::get_singleton()->rope_set_collision_layer(rope, collision_layer);
	}
}

void RopeBody3D::set_collision_mask(uint32_t p_mask) {
	collision_mask = p_mask;

	if (is_backend_supported()) {
		PhysicsServer3D::get_singleton()->rope_set_collision_mask(rope, collision_mask);
	}
}

void RopeBody3D::set_collision_layer_value(int p_layer_number, bool p_value) {
	ERR_FAIL_COND_MSG(p_layer_number < 1, "Collision layer number must be between 1 and 32, inclusive.");
	ERR_FAIL_COND_MSG(p_layer_number > 32, "Collision layer number must be between 1 and 32, inclusive.");

	uint32_t layer = get_collision_layer();
	if (p_value) {
		layer |= 1 << (p_layer_number - 1);
	} else {
		layer &= ~(1 << (p_layer_number - 1));
	}
	set_collision_layer(layer);
}

bool RopeBody3D::get_collision_layer_value(int p_layer_number) const {
	ERR_FAIL_COND_V_MSG(p_layer_number < 1, false, "Collision layer number must be between 1 and 32, inclusive.");
	ERR_FAIL_COND_V_MSG(p_layer_number > 32, false, "Collision layer number must be between 1 and 32, inclusive.");

	return get_collision_layer() & (1 << (p_layer_number - 1));
}

void RopeBody3D::set_collision_mask_value(int p_layer_number, bool p_value) {
	ERR_FAIL_COND_MSG(p_layer_number < 1, "Collision layer number must be between 1 and 32, inclusive.");
	ERR_FAIL_COND_MSG(p_layer_number > 32, "Collision layer number must be between 1 and 32, inclusive.");

	uint32_t mask = get_collision_mask();
	if (p_value) {
		mask |= 1 << (p_layer_number - 1);
	} else {
		mask &= ~(1 << (p_layer_number - 1));
	}
	set_collision_mask(mask);
}

bool RopeBody3D::get_collision_mask_value(int p_layer_number) const {
	ERR_FAIL_COND_V_MSG(p_layer_number < 1, false, "Collision layer number must be between 1 and 32, inclusive.");
	ERR_FAIL_COND_V_MSG(p_layer_number > 32, false, "Collision layer number must be between 1 and 32, inclusive.");

	return get_collision_mask() & (1 << (p_layer_number - 1));
}

void RopeBody3D::set_attach_start(const NodePath &p_path) {
	attach_start_path = p_path;
	_reseed_curve();
	_queue_rebuild();
	update_gizmos();
}

void RopeBody3D::set_attach_end(const NodePath &p_path) {
	attach_end_path = p_path;
	_reseed_curve();
	_queue_rebuild();
	update_gizmos();
}

void RopeBody3D::set_render_mode(RenderMode p_mode) {
	if (render_mode == p_mode) {
		return;
	}

	render_mode = p_mode;
	_mark_mesh_dirty();
	notify_property_list_changed();
}

void RopeBody3D::set_radial_segments(int p_segments) {
	const int clamped = CLAMP(p_segments, 3, 32);
	if (radial_segments == clamped) {
		return;
	}

	radial_segments = clamped;
	_mark_mesh_dirty();
}

void RopeBody3D::_validate_property(PropertyInfo &p_property) const {
	if (p_property.name == "radial_segments" && render_mode != RENDER_TUBE) {
		p_property.usage = PROPERTY_USAGE_NO_EDITOR;
	}
	if (p_property.name == "max_reaction_impulse" && !two_way_coupling) {
		p_property.usage = PROPERTY_USAGE_NO_EDITOR;
	}
	if (p_property.name == "slack" && length_mode != LENGTH_FROM_ENDPOINTS) {
		p_property.usage = PROPERTY_USAGE_NO_EDITOR;
	}
	if (p_property.name == "length" && length_mode != LENGTH_MANUAL) {
		p_property.usage |= PROPERTY_USAGE_READ_ONLY;
	}
}

PackedStringArray RopeBody3D::get_configuration_warnings() const {
	PackedStringArray warnings = MeshInstance3D::get_configuration_warnings();

	if (!is_backend_supported()) {
		warnings.push_back(RTR("RopeBody3D requires the Jolt Physics backend. Set Project Settings > Physics > 3D > Physics Engine to \"Jolt Physics\" (a restart is required); the rope will not simulate otherwise."));
	}

	if (resolved_pins.size() >= 2) {
		const float span = (float)resolved_pins[0].position.distance_to(resolved_pins[resolved_pins.size() - 1].position);
		if (length_mode == LENGTH_MANUAL && length < span * 0.999f) {
			warnings.push_back(vformat(RTR("Length (%.2f m) is shorter than the distance between the outermost pins (%.2f m). The rope will snap taut on the first frame and pull on whatever it is pinned to."), length, span));
		}
	}

	if (two_way_coupling && !collision_enabled && attach_start_path.is_empty() && attach_end_path.is_empty() && pins.is_empty()) {
		warnings.push_back(RTR("Two-way coupling has no effect while collision is disabled and the rope is not attached to any body."));
	}

	const Transform3D transform = get_transform();
	const Vector3 scale = transform.basis.get_scale();
	if (!Math::is_equal_approx(scale.x, scale.y) || !Math::is_equal_approx(scale.y, scale.z)) {
		warnings.push_back(RTR("RopeBody3D does not support a non-uniformly scaled transform; the generated geometry will be skewed."));
	}

	return warnings;
}

/* -------------------------------------------------------------------------- */
/* Bindings                                                                   */
/* -------------------------------------------------------------------------- */

void RopeBody3D::_bind_methods() {
	ClassDB::bind_method(D_METHOD("get_physics_rid"), &RopeBody3D::get_physics_rid);

	ClassDB::bind_method(D_METHOD("set_point_count", "count"), &RopeBody3D::set_point_count);
	ClassDB::bind_method(D_METHOD("get_point_count"), &RopeBody3D::get_point_count);

	ClassDB::bind_method(D_METHOD("set_length", "length"), &RopeBody3D::set_length);
	ClassDB::bind_method(D_METHOD("get_length"), &RopeBody3D::get_length);

	ClassDB::bind_method(D_METHOD("set_length_mode", "mode"), &RopeBody3D::set_length_mode);
	ClassDB::bind_method(D_METHOD("get_length_mode"), &RopeBody3D::get_length_mode);

	ClassDB::bind_method(D_METHOD("set_slack", "slack"), &RopeBody3D::set_slack);
	ClassDB::bind_method(D_METHOD("get_slack"), &RopeBody3D::get_slack);

	ClassDB::bind_method(D_METHOD("set_auto_sag", "enabled"), &RopeBody3D::set_auto_sag);
	ClassDB::bind_method(D_METHOD("is_auto_sag"), &RopeBody3D::is_auto_sag);

	ClassDB::bind_method(D_METHOD("get_segment_length"), &RopeBody3D::get_segment_length);

	ClassDB::bind_method(D_METHOD("set_spawn_curve", "curve"), &RopeBody3D::set_spawn_curve);
	ClassDB::bind_method(D_METHOD("get_spawn_curve"), &RopeBody3D::get_spawn_curve);

	ClassDB::bind_method(D_METHOD("set_initial_direction", "direction"), &RopeBody3D::set_initial_direction);
	ClassDB::bind_method(D_METHOD("get_initial_direction"), &RopeBody3D::get_initial_direction);

	ClassDB::bind_method(D_METHOD("set_radius", "radius"), &RopeBody3D::set_radius);
	ClassDB::bind_method(D_METHOD("get_radius"), &RopeBody3D::get_radius);

	ClassDB::bind_method(D_METHOD("set_total_mass", "mass"), &RopeBody3D::set_total_mass);
	ClassDB::bind_method(D_METHOD("get_total_mass"), &RopeBody3D::get_total_mass);

	ClassDB::bind_method(D_METHOD("set_simulation_substeps", "substeps"), &RopeBody3D::set_simulation_substeps);
	ClassDB::bind_method(D_METHOD("get_simulation_substeps"), &RopeBody3D::get_simulation_substeps);

	ClassDB::bind_method(D_METHOD("set_stretch_compliance", "compliance"), &RopeBody3D::set_stretch_compliance);
	ClassDB::bind_method(D_METHOD("get_stretch_compliance"), &RopeBody3D::get_stretch_compliance);

	ClassDB::bind_method(D_METHOD("set_bend_compliance", "compliance"), &RopeBody3D::set_bend_compliance);
	ClassDB::bind_method(D_METHOD("get_bend_compliance"), &RopeBody3D::get_bend_compliance);
	ClassDB::bind_method(D_METHOD("set_twist_compliance", "compliance"), &RopeBody3D::set_twist_compliance);
	ClassDB::bind_method(D_METHOD("get_twist_compliance"), &RopeBody3D::get_twist_compliance);
	ClassDB::bind_method(D_METHOD("set_twist_damping", "damping"), &RopeBody3D::set_twist_damping);
	ClassDB::bind_method(D_METHOD("get_twist_damping"), &RopeBody3D::get_twist_damping);

	ClassDB::bind_method(D_METHOD("set_linear_damping", "damping"), &RopeBody3D::set_linear_damping);
	ClassDB::bind_method(D_METHOD("get_linear_damping"), &RopeBody3D::get_linear_damping);

	ClassDB::bind_method(D_METHOD("set_drag", "drag"), &RopeBody3D::set_drag);
	ClassDB::bind_method(D_METHOD("get_drag"), &RopeBody3D::get_drag);

	ClassDB::bind_method(D_METHOD("set_gravity_scale", "scale"), &RopeBody3D::set_gravity_scale);
	ClassDB::bind_method(D_METHOD("get_gravity_scale"), &RopeBody3D::get_gravity_scale);

	ClassDB::bind_method(D_METHOD("set_friction", "friction"), &RopeBody3D::set_friction);
	ClassDB::bind_method(D_METHOD("get_friction"), &RopeBody3D::get_friction);

	ClassDB::bind_method(D_METHOD("set_restitution", "restitution"), &RopeBody3D::set_restitution);
	ClassDB::bind_method(D_METHOD("get_restitution"), &RopeBody3D::get_restitution);

	ClassDB::bind_method(D_METHOD("set_max_reaction_impulse", "impulse"), &RopeBody3D::set_max_reaction_impulse);
	ClassDB::bind_method(D_METHOD("get_max_reaction_impulse"), &RopeBody3D::get_max_reaction_impulse);

	ClassDB::bind_method(D_METHOD("set_inextensible", "enabled"), &RopeBody3D::set_inextensible);
	ClassDB::bind_method(D_METHOD("is_inextensible"), &RopeBody3D::is_inextensible);

	ClassDB::bind_method(D_METHOD("set_two_way_coupling", "enabled"), &RopeBody3D::set_two_way_coupling);
	ClassDB::bind_method(D_METHOD("is_two_way_coupling"), &RopeBody3D::is_two_way_coupling);

	ClassDB::bind_method(D_METHOD("set_collision_enabled", "enabled"), &RopeBody3D::set_collision_enabled);
	ClassDB::bind_method(D_METHOD("is_collision_enabled"), &RopeBody3D::is_collision_enabled);

	ClassDB::bind_method(D_METHOD("set_collision_layer", "layer"), &RopeBody3D::set_collision_layer);
	ClassDB::bind_method(D_METHOD("get_collision_layer"), &RopeBody3D::get_collision_layer);

	ClassDB::bind_method(D_METHOD("set_collision_mask", "mask"), &RopeBody3D::set_collision_mask);
	ClassDB::bind_method(D_METHOD("get_collision_mask"), &RopeBody3D::get_collision_mask);

	ClassDB::bind_method(D_METHOD("set_collision_layer_value", "layer_number", "value"), &RopeBody3D::set_collision_layer_value);
	ClassDB::bind_method(D_METHOD("get_collision_layer_value", "layer_number"), &RopeBody3D::get_collision_layer_value);

	ClassDB::bind_method(D_METHOD("set_collision_mask_value", "layer_number", "value"), &RopeBody3D::set_collision_mask_value);
	ClassDB::bind_method(D_METHOD("get_collision_mask_value", "layer_number"), &RopeBody3D::get_collision_mask_value);

	ClassDB::bind_method(D_METHOD("set_attach_start", "path"), &RopeBody3D::set_attach_start);
	ClassDB::bind_method(D_METHOD("get_attach_start"), &RopeBody3D::get_attach_start);

	ClassDB::bind_method(D_METHOD("set_attach_end", "path"), &RopeBody3D::set_attach_end);
	ClassDB::bind_method(D_METHOD("get_attach_end"), &RopeBody3D::get_attach_end);

	ClassDB::bind_method(D_METHOD("set_pin_count", "count"), &RopeBody3D::set_pin_count);
	ClassDB::bind_method(D_METHOD("get_pin_count"), &RopeBody3D::get_pin_count);

	ClassDB::bind_method(D_METHOD("set_pin_lock_twist", "pin_index", "enabled"), &RopeBody3D::set_pin_lock_twist);
	ClassDB::bind_method(D_METHOD("get_pin_lock_twist", "pin_index"), &RopeBody3D::get_pin_lock_twist);
	ClassDB::bind_method(D_METHOD("set_pin_lock_direction", "pin_index", "enabled"), &RopeBody3D::set_pin_lock_direction);
	ClassDB::bind_method(D_METHOD("get_pin_lock_direction", "pin_index"), &RopeBody3D::get_pin_lock_direction);
	ClassDB::bind_method(D_METHOD("set_pin_direction_compliance", "pin_index", "compliance"), &RopeBody3D::set_pin_direction_compliance);
	ClassDB::bind_method(D_METHOD("get_pin_direction_compliance", "pin_index"), &RopeBody3D::get_pin_direction_compliance);
	ClassDB::bind_method(D_METHOD("set_pin_solver_limit", "pin_index", "enabled"), &RopeBody3D::set_pin_solver_limit);
	ClassDB::bind_method(D_METHOD("get_pin_solver_limit", "pin_index"), &RopeBody3D::get_pin_solver_limit);
	ClassDB::bind_method(D_METHOD("set_pin_ratio", "pin_index", "ratio"), &RopeBody3D::set_pin_ratio);
	ClassDB::bind_method(D_METHOD("get_pin_ratio", "pin_index"), &RopeBody3D::get_pin_ratio);

	ClassDB::bind_method(D_METHOD("set_pin_node", "pin_index", "path"), &RopeBody3D::set_pin_node);
	ClassDB::bind_method(D_METHOD("get_pin_node", "pin_index"), &RopeBody3D::get_pin_node);

	ClassDB::bind_method(D_METHOD("set_pin_offset", "pin_index", "offset"), &RopeBody3D::set_pin_offset);
	ClassDB::bind_method(D_METHOD("get_pin_offset", "pin_index"), &RopeBody3D::get_pin_offset);

	ClassDB::bind_method(D_METHOD("toggle_pin_at_ratio", "ratio"), &RopeBody3D::toggle_pin_at_ratio);
	ClassDB::bind_method(D_METHOD("find_pin_at_index", "point_index"), &RopeBody3D::find_pin_at_index);

	ClassDB::bind_method(D_METHOD("set_render_mode", "mode"), &RopeBody3D::set_render_mode);
	ClassDB::bind_method(D_METHOD("get_render_mode"), &RopeBody3D::get_render_mode);

	ClassDB::bind_method(D_METHOD("set_radial_segments", "segments"), &RopeBody3D::set_radial_segments);
	ClassDB::bind_method(D_METHOD("get_radial_segments"), &RopeBody3D::get_radial_segments);

	ClassDB::bind_method(D_METHOD("get_point_position", "point_index"), &RopeBody3D::get_point_position);
	ClassDB::bind_method(D_METHOD("get_point_velocity", "point_index"), &RopeBody3D::get_point_velocity);
	ClassDB::bind_method(D_METHOD("get_points"), &RopeBody3D::get_points);
	ClassDB::bind_method(D_METHOD("get_simulated_length"), &RopeBody3D::get_simulated_length);

	ClassDB::bind_method(D_METHOD("get_ratios_in_area", "area"), &RopeBody3D::get_ratios_in_area);
	ClassDB::bind_method(D_METHOD("is_in_area", "area"), &RopeBody3D::is_in_area);
	ClassDB::bind_static_method("RopeBody3D", D_METHOD("get_ropes_in_area", "area"), &RopeBody3D::get_ropes_in_area);

	ClassDB::bind_method(D_METHOD("get_closest_ratio", "world_point"), &RopeBody3D::get_closest_ratio);
	ClassDB::bind_method(D_METHOD("get_closest_point", "world_point"), &RopeBody3D::get_closest_point);
	ClassDB::bind_method(D_METHOD("sample_ratio", "ratio"), &RopeBody3D::sample_ratio);

	ClassDB::bind_method(D_METHOD("pin_point", "point_index", "pin"), &RopeBody3D::pin_point);
	ClassDB::bind_method(D_METHOD("is_point_pinned", "point_index"), &RopeBody3D::is_point_pinned);
	ClassDB::bind_method(D_METHOD("set_pin_position", "point_index", "position"), &RopeBody3D::set_pin_position);

	ClassDB::bind_method(D_METHOD("attach_point", "point_index", "body"), &RopeBody3D::attach_point);
	ClassDB::bind_method(D_METHOD("detach_point", "point_index"), &RopeBody3D::detach_point);

	ClassDB::bind_method(D_METHOD("apply_point_impulse", "point_index", "impulse"), &RopeBody3D::apply_point_impulse);
	ClassDB::bind_method(D_METHOD("apply_central_impulse", "impulse"), &RopeBody3D::apply_central_impulse);

	ClassDB::bind_method(D_METHOD("reset"), &RopeBody3D::reset);

	ADD_GROUP("Shape", "");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "length_mode", PROPERTY_HINT_ENUM, "Manual,From Endpoints,From Curve"), "set_length_mode", "get_length_mode");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "length", PROPERTY_HINT_RANGE, "0.001,100,0.001,or_greater,suffix:m"), "set_length", "get_length");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "slack", PROPERTY_HINT_RANGE, "0,10,0.001,or_greater,suffix:m"), "set_slack", "get_slack");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "auto_sag"), "set_auto_sag", "is_auto_sag");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "point_count", PROPERTY_HINT_RANGE, "2,512,1"), "set_point_count", "get_point_count");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "radius", PROPERTY_HINT_RANGE, "0.001,1,0.001,or_greater,suffix:m"), "set_radius", "get_radius");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "initial_direction"), "set_initial_direction", "get_initial_direction");
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "spawn_curve", PROPERTY_HINT_RESOURCE_TYPE, Curve3D::get_class_static(), PROPERTY_USAGE_DEFAULT | PROPERTY_USAGE_EDITOR_INSTANTIATE_OBJECT), "set_spawn_curve", "get_spawn_curve");

	ADD_GROUP("Simulation", "");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "total_mass", PROPERTY_HINT_RANGE, "0.001,1000,0.001,or_greater,exp,suffix:kg"), "set_total_mass", "get_total_mass");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "simulation_substeps", PROPERTY_HINT_RANGE, "1,32,1"), "set_simulation_substeps", "get_simulation_substeps");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "inextensible"), "set_inextensible", "is_inextensible");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "stretch_compliance", PROPERTY_HINT_RANGE, "0,0.01,0.000001,or_greater"), "set_stretch_compliance", "get_stretch_compliance");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "bend_compliance", PROPERTY_HINT_RANGE, "0,1000000,0.001,exp"), "set_bend_compliance", "get_bend_compliance");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "twist_compliance", PROPERTY_HINT_RANGE, "0,1000000,0.001,exp"), "set_twist_compliance", "get_twist_compliance");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "twist_damping", PROPERTY_HINT_RANGE, "0,10,0.01,or_greater"), "set_twist_damping", "get_twist_damping");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "linear_damping", PROPERTY_HINT_RANGE, "0,10,0.001,or_greater"), "set_linear_damping", "get_linear_damping");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "drag", PROPERTY_HINT_RANGE, "0,10,0.001,or_greater"), "set_drag", "get_drag");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "gravity_scale", PROPERTY_HINT_RANGE, "-8,8,0.001,or_less,or_greater"), "set_gravity_scale", "get_gravity_scale");

	ADD_GROUP("Collision", "");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "collision_enabled"), "set_collision_enabled", "is_collision_enabled");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "collision_layer", PROPERTY_HINT_LAYERS_3D_PHYSICS), "set_collision_layer", "get_collision_layer");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "collision_mask", PROPERTY_HINT_LAYERS_3D_PHYSICS), "set_collision_mask", "get_collision_mask");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "friction", PROPERTY_HINT_RANGE, "0,1,0.01"), "set_friction", "get_friction");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "restitution", PROPERTY_HINT_RANGE, "0,1,0.01"), "set_restitution", "get_restitution");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "two_way_coupling"), "set_two_way_coupling", "is_two_way_coupling");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "max_reaction_impulse", PROPERTY_HINT_RANGE, "0,1000,0.1,or_greater"), "set_max_reaction_impulse", "get_max_reaction_impulse");

	ADD_GROUP("Attachment", "");
	ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "attach_start", PROPERTY_HINT_NODE_PATH_VALID_TYPES, "Node3D"), "set_attach_start", "get_attach_start");
	ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "attach_end", PROPERTY_HINT_NODE_PATH_VALID_TYPES, "Node3D"), "set_attach_end", "get_attach_end");

	ADD_GROUP("Rendering", "");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "render_mode", PROPERTY_HINT_ENUM, "None,Tube,Ribbon"), "set_render_mode", "get_render_mode");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "radial_segments", PROPERTY_HINT_RANGE, "3,32,1"), "set_radial_segments", "get_radial_segments");

	ADD_ARRAY_COUNT("Pins", "pin_count", "set_pin_count", "get_pin_count", "pins/");

	BIND_ENUM_CONSTANT(RENDER_NONE);
	BIND_ENUM_CONSTANT(RENDER_TUBE);
	BIND_ENUM_CONSTANT(RENDER_RIBBON);

	BIND_ENUM_CONSTANT(LENGTH_MANUAL);
	BIND_ENUM_CONSTANT(LENGTH_FROM_ENDPOINTS);
	BIND_ENUM_CONSTANT(LENGTH_FROM_CURVE);

	Pin defaults;

	base_property_helper.set_prefix("pins/");
	base_property_helper.set_array_length_getter(&RopeBody3D::get_pin_count);
	base_property_helper.register_property(PropertyInfo(Variant::FLOAT, "ratio", PROPERTY_HINT_RANGE, "0,1,0.001"), defaults.ratio, &RopeBody3D::set_pin_ratio, &RopeBody3D::get_pin_ratio);
	base_property_helper.register_property(PropertyInfo(Variant::NODE_PATH, "node", PROPERTY_HINT_NODE_PATH_VALID_TYPES, "Node3D"), defaults.node_path, &RopeBody3D::set_pin_node, &RopeBody3D::get_pin_node);
	base_property_helper.register_property(PropertyInfo(Variant::VECTOR3, "offset"), defaults.offset, &RopeBody3D::set_pin_offset, &RopeBody3D::get_pin_offset);
	base_property_helper.register_property(PropertyInfo(Variant::BOOL, "lock_twist"), defaults.lock_twist, &RopeBody3D::set_pin_lock_twist, &RopeBody3D::get_pin_lock_twist);
	base_property_helper.register_property(PropertyInfo(Variant::BOOL, "lock_direction"), defaults.lock_direction, &RopeBody3D::set_pin_lock_direction, &RopeBody3D::get_pin_lock_direction);
	base_property_helper.register_property(PropertyInfo(Variant::FLOAT, "direction_compliance", PROPERTY_HINT_RANGE, "0,1,0.0001,exp"), defaults.direction_compliance, &RopeBody3D::set_pin_direction_compliance, &RopeBody3D::get_pin_direction_compliance);
	base_property_helper.register_property(PropertyInfo(Variant::BOOL, "solver_limit"), defaults.solver_limit, &RopeBody3D::set_pin_solver_limit, &RopeBody3D::get_pin_solver_limit);
	PropertyListHelper::register_base_helper(get_class_static(), &base_property_helper);
}
