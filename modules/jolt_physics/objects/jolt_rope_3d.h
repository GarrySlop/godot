/**************************************************************************/
/*  jolt_rope_3d.h                                                        */
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

#include "core/math/aabb.h"
#include "core/math/basis.h"
#include "core/math/transform_3d.h"
#include "core/math/vector3.h"
#include "core/object/object_id.h"
#include "core/templates/hash_map.h"
#include "core/templates/local_vector.h"
#include "core/templates/rid.h"
#include "core/templates/self_list.h"
#include "core/templates/vset.h"

#include "servers/physics_3d/physics_server_3d.h"

#include <Jolt/Jolt.h>

#include <Jolt/Physics/Body/BodyID.h>

class JoltBody3D;
class JoltSpace3D;

// A rope simulated as a chain of point masses connected by XPBD distance constraints.
//
// This is deliberately NOT a `JoltObject3D`: it owns no `JPH::Body` and does not participate in
// Jolt's own solver. Jolt is used for two things only, both of which it does far better than we
// could: narrow-phase collision (a `JPH::CapsuleShape` per segment, so the rope collides along its
// whole length rather than only at its particles) and rigid-body coupling.
//
// The solver follows Macklin et al. 2019 ("Small Steps in Physics Simulation"): many substeps with
// a single constraint iteration each, rather than one step with many iterations. Combined with a
// symmetric (forward + backward) Gauss-Seidel sweep and long-range attachment constraints, this is
// what lets the rope behave as genuinely inextensible without a large iteration count.
//
// The performance-critical property is that collision *detection* runs once per frame while
// collision *resolution* runs every substep against the cached result. Re-querying per substep is
// the obvious implementation and is roughly an order of magnitude more expensive.
class JoltRope3D {
public:
	enum AttachMode {
		ATTACH_NONE,
		ATTACH_STATIC,
		ATTACH_BODY,
	};

	struct Attachment {
		AttachMode mode = ATTACH_NONE;
		Vector3 static_position;

		RID body_rid;
		JoltBody3D *body = nullptr;
		Vector3 local_offset;

		// Refreshed once per frame by `_update_attachment_targets()`.
		Vector3 target;
		int collider = -1;
		bool soft = false;

		// The nearest genuinely immovable anchor along the rope, and the total rest length between
		// the two. A body attachment can never be further from that anchor than this, which is what
		// makes the rope act as a limit joint on the body it is tied to.
		int limit_anchor = -1;
		float limit_distance = 0.0f;
	};

private:
	// A collider the rope is currently touching. Cached once per frame so the substep loop can
	// re-derive contact planes from the collider's *current* transform (which keeps rope-on-moving-
	// platform correct) without paying for another narrow-phase query.
	struct ColliderInfo {
		JPH::BodyID body_id;
		Transform3D transform;
		Transform3D inv_transform;
		Vector3 com;
		Vector3 linear_velocity;
		Vector3 angular_velocity;
		Basis inv_inertia;
		float inv_mass = 0.0f;
		float friction = 0.5f;
		float restitution = 0.0f;
		bool dynamic = false;

		// Proxy state. Jolt has already finished stepping by the time the rope runs, so the real
		// body is frozen for the whole substep loop. Without somewhere to record the motion our own
		// corrections imply, every substep would re-derive the *same* full correction against a
		// body that never moved -- the constraint would never converge, and the reaction impulses
		// would compound. These two accumulate that motion so each substep sees a body that has
		// already responded to the last one; `rotation_delta` is a small-angle rotation vector,
		// which is accurate enough for the sub-degree corrections one frame produces.
		Vector3 position_delta;
		Vector3 rotation_delta;

		// Where a point that started the frame at `p_world` has been moved to by our corrections.
		Vector3 displaced(const Vector3 &p_world) const {
			return p_world + position_delta + rotation_delta.cross(p_world - com);
		}
	};

	struct Contact {
		int segment = 0;
		float t = 0.0f;
		int collider = 0;
		Vector3 local_point;
		Vector3 local_normal;
	};

	RID rid;
	ObjectID instance_id;
	JoltSpace3D *space = nullptr;

	SelfList<JoltRope3D> active_list;

	// Particle state as parallel arrays. `rest_lengths`/`lambdas` are one entry shorter than the
	// particle arrays, `bend_rest_lengths` two entries shorter.
	LocalVector<Vector3> positions;
	LocalVector<Vector3> prev_positions;
	LocalVector<Vector3> velocities;
	LocalVector<float> inv_masses;
	LocalVector<float> base_inv_masses;
	LocalVector<float> rest_lengths;
	LocalVector<float> bend_rest_lengths;
	LocalVector<float> lambdas;
	// Rest length from particle 0 to each particle, so the rest distance between any two is a
	// subtraction rather than a walk. Rebuilt with `rest_lengths`.
	LocalVector<float> cumulative_rest;
	// Per-particle gravity, sampled once per frame. Godot applies gravity through Areas rather than
	// through Jolt's system gravity (which `JoltSpace3D` zeroes), and `compute_gravity()` is
	// position-dependent so that point gravity works.
	LocalVector<Vector3> gravity_cache;

	// Long-range attachment: for each particle, the nearest anchor and the total rest length
	// between them. A particle can never be further from its anchor than that, which enforces
	// inextensibility globally in a single O(n) pass instead of relying on constraint propagation.
	LocalVector<int> lra_anchor;
	LocalVector<float> lra_distance;
	bool lra_dirty = true;

	HashMap<int, Attachment> attachments;
	VSet<RID> exceptions;

	LocalVector<ColliderInfo> colliders;
	LocalVector<Contact> contacts;
	HashMap<uint32_t, int> collider_lookup;

	AABB bounds;

	uint32_t collision_layer = 1;
	uint32_t collision_mask = 1;

	int substeps = 4;
	// Duration of the whole physics frame. Positional corrections are turned into impulses with
	// this rather than the substep duration: Jolt has already integrated the bodies across the full
	// frame before the rope runs, so the displacement a correction undoes accumulated over that
	// frame. Dividing by the substep would overstate every reaction by `substeps` times.
	float frame_step = 0.0f;

	float radius = 0.05f;
	float total_mass = 1.0f;
	float stretch_compliance = 0.0f;
	float bend_compliance = 1e9f;
	float linear_damping = 0.1f;
	float drag = 0.0f;
	float gravity_scale = 1.0f;
	float friction = 0.5f;
	float restitution = 0.0f;
	float max_reaction_impulse = 100.0f;
	// Requested total rest length. Zero means "use the spacing of the authored points as-is".
	float rest_length = 0.0f;

	bool inextensible = true;
	bool two_way_coupling = false;
	bool collision_enabled = true;

	void _update_masses();
	void _update_rest_lengths();
	void _update_lra();
	void _update_attachment_targets();
	void _update_bounds();
	void _update_gravity();

	void _gather_contacts(float p_step);
	int _resolve_collider(const JPH::BodyID &p_body_id);

	void _integrate(float p_step);
	void _solve_distance(float p_step);
	void _solve_bending(float p_step);
	void _solve_lra();
	void _solve_strain_limit();
	void _solve_attachments(float p_step);
	void _solve_collisions(float p_step);
	void _update_velocities(float p_step);
	void _apply_reactions();

public:
	JoltRope3D();
	~JoltRope3D();

	RID get_rid() const { return rid; }
	void set_rid(const RID &p_rid) { rid = p_rid; }

	ObjectID get_instance_id() const { return instance_id; }
	void set_instance_id(ObjectID p_id) { instance_id = p_id; }

	JoltSpace3D *get_space() const { return space; }
	void set_space(JoltSpace3D *p_space);

	SelfList<JoltRope3D> *get_active_list_element() { return &active_list; }

	void set_points(const Vector<Vector3> &p_points);
	Vector<Vector3> get_points() const;
	int get_point_count() const { return (int)positions.size(); }

	Vector3 get_point_position(int p_index) const;
	void set_point_position(int p_index, const Vector3 &p_position);
	Vector3 get_point_velocity(int p_index) const;

	void set_param(PhysicsServer3D::RopeParameter p_param, float p_value);
	float get_param(PhysicsServer3D::RopeParameter p_param) const;

	void set_flag(PhysicsServer3D::RopeFlag p_flag, bool p_enabled);
	bool get_flag(PhysicsServer3D::RopeFlag p_flag) const;

	void set_simulation_substeps(int p_substeps);
	int get_simulation_substeps() const { return substeps; }

	uint32_t get_collision_layer() const { return collision_layer; }
	void set_collision_layer(uint32_t p_layer) { collision_layer = p_layer; }

	uint32_t get_collision_mask() const { return collision_mask; }
	void set_collision_mask(uint32_t p_mask) { collision_mask = p_mask; }

	void add_collision_exception(RID p_body) { exceptions.insert(p_body); }
	void remove_collision_exception(RID p_body) { exceptions.erase(p_body); }
	const VSet<RID> &get_collision_exceptions() const { return exceptions; }

	void pin_point(int p_index, bool p_pin);
	bool is_point_pinned(int p_index) const;

	void set_pin_position(int p_index, const Vector3 &p_position);
	Vector3 get_pin_position(int p_index) const;

	void attach_point_to_body(int p_index, RID p_body_rid, JoltBody3D *p_body, const Vector3 &p_local_offset);
	void detach_point(int p_index);
	void remove_all_attachments();
	// Called when a body is freed, so attachments never keep a dangling pointer.
	void detach_from_body(RID p_body_rid);

	void apply_point_impulse(int p_index, const Vector3 &p_impulse);
	void apply_central_impulse(const Vector3 &p_impulse);

	AABB get_bounds() const { return bounds; }

	void step(float p_step);
};
